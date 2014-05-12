/* drivers/char/isaac.c
 * Implementation (c) 2014 Ryan Pennucci
 * Bob Jenkins has placed ISAAC in the public domain.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
/* I'm not qualified to make changes to the ISAAC algorithms, but I have
 * substantially re-implemented the algorithms to be more suitable for the
 * kernel.
 *	- RP
 */
/* TODO:
 *  - Fix seeding
 *  - Intergrate with kernel
 */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/percpu.h>
#include <linux/cpumask.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#ifdef MODULE
#include <linux/cdev.h>
#include <linux/device.h>
#endif

#define ISAAC_DEV MKDEV(235, 11)

/* Provide (untested) get_random_...()? */
//#define ISAAC_RANDOM

/* Use ISAAC+ generator? */
#define ISAAC_PLUS

#define RANDSIZL (8)
#define RANDSIZ  (1<<RANDSIZL)
#define RANDSIZB (RANDSIZ<<2)
struct isaac_ctx {
	u32		mem[RANDSIZ];
	union {
		u32	res[RANDSIZ];
		u8	bytes[RANDSIZB];
	};
	u32		a, b, c;
	u16		idx;
	struct semaphore sem;
};

/* Private per-CPU seeds.  These are used to initialize per-fd contexts, and
 * will eventually be used for small reads, get_random_long, and so forth.
 */
static DEFINE_PER_CPU(struct isaac_ctx, cpu_seeds);

/* Generate random bytes. */
#ifndef ISAAC_PLUS
#define IND(x) (*(u32 *)((u8 *)(mm) + ((x) & ((RANDSIZ-1)<<2))))
#define STEP(mix) {				\
	x = *m;					\
	a = (a ^ (mix)) + *(m2++);		\
	*(m++) = y = IND(x) + a + b;		\
	*(r++) = b = IND(y >> RANDSIZL) + x;	\
}
#else
/* Jean-Philippe Aumasson's ISAAC+ algorithm, almost.  ISAAC's IND() macro is
 * used, since it's equivalent (and generates better code) for RANDSIZL < 16.
 */
//#define IND2(x,s) (mm[ror32(x,s)&(RANDSIZ-1)])
#define IND(x) (*(u32 *)((u8 *)(mm) + ((x) & ((RANDSIZ-1)<<2))))
#define STEP(mix) {				\
	x = *m;					\
	a = (a ^ (mix)) + *(m2++);		\
	*(m++) = y = IND(x) + a ^ b;		\
	*(r++) = b = IND(y >> RANDSIZL) ^ a + x;\
}
#endif
static void isaac(struct isaac_ctx *ctx, u32 *buf)
{
	register u32 a, b, x, y, *m, *mm, *m2, *r, *mend;
	mm = ctx->mem;
	r = buf;
	a = ctx->a;
	b = ctx->b + (++ctx->c);
	for (m = mm, mend = m2 = m + (RANDSIZ/2); m < mend; ) {
		STEP(a<<13);
		STEP(a>>6);
		STEP(a<<2);
		STEP(a>>16);
	}
	for (m2 = mm; m2 < mend; ) {
		STEP(a<<13);
		STEP(a>>6);
		STEP(a<<2);
		STEP(a>>16);
	}
	ctx->a = a;
	ctx->b = b;
}

#define MIX()				\
{					\
	a ^= b << 11;	d += a; b += c;	\
	b ^= c >> 2;	e += b; c += d;	\
	c ^= d << 8;	f += c; d += e;	\
	d ^= e >> 16;	g += d; e += f;	\
	e ^= f << 10;	h += e; f += g;	\
	f ^= g >> 4;	a += f; g += h;	\
	g ^= h << 8;	b += g; h += a;	\
	h ^= a >> 9;	c += h; a += b;	\
}
#define LOAD(t)				\
{					\
	a += t[i    ]; b += t[i + 1];	\
	c += t[i + 2]; d += t[i + 3];	\
	e += t[i + 4]; f += t[i + 5];	\
	g += t[i + 6]; h += t[i + 7];	\
}
#define STORE()				\
{					\
	m[i    ] = a; m[i + 1] = b;	\
	m[i + 2] = c; m[i + 3] = d;	\
	m[i + 4] = e; m[i + 5] = f;	\
	m[i + 6] = g; m[i + 7] = h;	\
}
static void isaac_init(struct isaac_ctx *ctx)
{
	int i;
	u32 a, b, c, d, e, f, g, h;
	u32 *m, *r;

	m = ctx->mem;
	r = ctx->res;
	ctx->a = ctx->b = ctx->c = 0;
	ctx->idx = RANDSIZB;
	a = b = c = d = e = f = g = h = 0x9e3779b9;

	for (i = 5; --i; ) {
		MIX();
	}

	isaac(&get_cpu_var(cpu_seeds), r);
	put_cpu_var(cpu_seeds);

	for (i = 0; i < RANDSIZ; i += 8) {
		LOAD(r);
		MIX();
		STORE();
	}
	for (i = 0; i < RANDSIZ; i += 8) {
		LOAD(m);
		MIX();
		STORE();
	}
}

/* Seed a new context with a specific seed.  During initialization, we seed
 * each cpu_seed using the previous seed.
 */
static void isaac_init_seed(struct isaac_ctx *seed, struct isaac_ctx *ctx)
{
	int i;
	u32 a, b, c, d, e, f, g, h;
	u32 *m, *r;

	m = ctx->mem;
	r = ctx->res;
	ctx->a = ctx->b = ctx->c = 0;
	ctx->idx = RANDSIZB;
	a = b = c = d = e = f = g = h = 0x9e3779b9;

	for (i = 5; --i; ) {
		MIX();
	}

	if (seed) {
		/* To reseed a cpu_seed, isaac_init_seed(ctx, ctx); */
		if (seed != ctx)
			isaac(seed, r);
		for (i = 0; i < RANDSIZ; i += 8) {
			LOAD(r);
			MIX();
			STORE();
		}
		for (i = 0; i < RANDSIZ; i += 8) {
			LOAD(m);
			MIX();
			STORE();
		}
	} else {
		pr_warn("initializing without a seed!\n");
		for (i = 0; i < RANDSIZ; i += 8) {
			MIX();
			STORE();
		}
		for (i = 9; --i; ) {
			isaac(ctx, ctx->res);
		}
	}
}

void isaac_add_randomness(const void *buf, unsigned int len)
{
	const u32 *src = buf;
	u32 *dst;
	struct isaac_ctx *ctx = &get_cpu_var(cpu_seeds);
	while (len >= 4) {
		dst = &ctx->res[ctx->idx >> 2];
		*dst = *dst + *src;
		len -= 4;
		src++;
		ctx->idx = (ctx->idx + 4) % RANDSIZB;
		if (!ctx->idx) {
			pr_info("reseeding seed for cpu %i\n",
				smp_processor_id());
			isaac_init_seed(ctx, ctx);
		}
	}
}

static int isaac_open(struct inode *inode, struct file *filp)
{
	struct isaac_ctx *ctx;

	ctx = kmalloc(sizeof(struct isaac_ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	isaac_init(ctx);
	sema_init(&ctx->sem, 1);

	filp->private_data = ctx;

	return 0;
}

static int isaac_release(struct inode *inode, struct file *filp)
{
	printk(KERN_DEBUG "%s: instance destroyed after %i blocks\n",
		__func__, ((struct isaac_ctx *)filp->private_data)->c);
	kfree(filp->private_data);
	return 0;
}

static ssize_t isaac_read(struct file *filp, char __user *buf, size_t count,
		loff_t *f_pos)
{
	struct isaac_ctx *ctx;
	size_t cnt;
	ssize_t ret = count;

	ctx = filp->private_data;
	if (down_interruptible(&ctx->sem))
		return -ERESTARTSYS;

	/* Copy out any existing bytes */
	if (ctx->idx != RANDSIZB) {
		cnt = min_t(size_t, count, RANDSIZB - ctx->idx);
		if (copy_to_user(buf, ctx->bytes + ctx->idx, cnt)) {
			ret = -EFAULT;
			goto out;
		}
		ctx->idx += cnt;
		buf += cnt;
		count -= cnt;
	}
	/* Generate more bytes as needed */
	if (count) {
		do {
			isaac(ctx, ctx->res);
			cnt = min_t(size_t, count, RANDSIZB);
			if (copy_to_user(buf, ctx->bytes, cnt)) {
				ret = -EFAULT;
				goto out;
			}
			buf += cnt;
			count -= cnt;
		} while (count);
		ctx->idx = RANDSIZB - cnt;
	}

out:
	up(&ctx->sem);

	return (ssize_t)ret;
}

static ssize_t isaac_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	u32 tmp[64];
	ssize_t ret = count;
	size_t cnt;

	count &= ~3;
	while (count) {
		cnt = min_t(size_t, count, sizeof(tmp));
		if (copy_from_user(tmp, buf, cnt)) {
			ret = -EFAULT;
			break;
		}
		isaac_add_randomness(tmp, cnt);
		buf += cnt;
		count -= cnt;
	}

	return ret;
}

const struct file_operations isaac_fops = {
	.open = isaac_open,
	.release = isaac_release,
	.read = isaac_read,
	.write = isaac_write,
	//.unlocked_ioctl = isaac_ioctl,
};

#ifdef ISAAC_RANDOM
static inline void isaac_copyout(struct isaac_ctx *ctx, void *buf, size_t len)
{
	/* Micro-optimization: by ignoring output ordering, we can avoid a
	 * memcpy and (occasionally) run one less iteration of isaac().
	 */
	while (len > RANDSIZB) {
		isaac(ctx, buf);
		len -= RANDSIZB;
		buf += RANDSIZB;
	}
	while (len) {
		size_t cnt = RANDSIZB - ctx->idx;
		if (!cnt) {
			isaac(ctx, ctx->res);
			ctx->idx = 0;
			cnt = RANDSIZB;
		}
		cnt = min_t(size_t, len, cnt);
		memcpy(buf, ctx->bytes + ctx->idx, cnt);
		ctx->idx += cnt;
		buf += cnt;
		len -= cnt;
	}
}

unsigned int get_random_int(void)
{
	unsigned int r;

	isaac_copyout(&get_cpu_var(cpu_seeds), &r, sizeof(r));
	put_cpu_var(cpu_seeds);
	return r;
}

void get_random_bytes(void *buf, int nbytes)
{
	isaac_copyout(&get_cpu_var(cpu_seeds), buf, nbytes);
	put_cpu_var(cpu_seeds);
}
#endif

#ifdef MODULE
static struct class *isaac_class;
static struct cdev isaac_cdev;
static struct device *isaac_device;
static int __init register_device(void)
{
	int ret;

	isaac_class = class_create(THIS_MODULE, "fastrng");
	if (IS_ERR(isaac_class)) {
		pr_warn("failed to register class\n");
		return PTR_ERR(isaac_class);
	}

	cdev_init(&isaac_cdev, &isaac_fops);
	isaac_cdev.owner = THIS_MODULE;
	ret = cdev_add(&isaac_cdev, ISAAC_DEV, 1);
	if (ret < 0) {
		pr_warn("failed to add cdev\n");
		goto out_destroy;
	}

	ret = register_chrdev_region(ISAAC_DEV, 1, "/dev/isaac");
	if (ret < 0) {
		pr_warn("failed to allocate major/minor\n");
		goto out_remove;
	}

	isaac_device = device_create(isaac_class, NULL, ISAAC_DEV,
		NULL, "isaac");
	if (IS_ERR(isaac_device)) {
		pr_warn("failed to create device\n");
		ret = PTR_ERR(isaac_device);
		goto out_unregister;
	}

	return 0;

out_unregister:
	unregister_chrdev_region(ISAAC_DEV, 1);
out_remove:
	cdev_del(&isaac_cdev);
out_destroy:
	class_destroy(isaac_class);

	return ret;
}

static void __exit unregister_device(void)
{
	unregister_chrdev_region(ISAAC_DEV, 1);
	cdev_del(&isaac_cdev);
	device_destroy(isaac_class, ISAAC_DEV);
	class_destroy(isaac_class);
}
module_exit(unregister_device);
#endif

/* FIXME: There's this new thing called locking */
static int __init isaac_seeds_init(void)
{
	int i, j;
	struct isaac_ctx *seed, *ctx;

	i = 0;
	j = num_possible_cpus() - 1;

	ctx = &get_cpu_var(cpu_seeds);
	isaac_init_seed(NULL, ctx);

	do {
		seed = ctx;
		ctx = &per_cpu(cpu_seeds, j);
		isaac_init_seed(seed, ctx);
	} while (--j >= 0);

	put_cpu_var(cpu_seeds);
#ifdef MODULE
	return register_device();
#else
	return 0;
#endif
}
core_initcall(isaac_seeds_init);

MODULE_DESCRIPTION("ISAAC PRNG");
MODULE_LICENSE("GPL");
