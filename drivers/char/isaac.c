/*
 * drivers/char/isaac.c
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
 *  - External seeding
 *  - implement random_int, random_long, etc.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/percpu.h>
#include <linux/cpumask.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/fs.h>
#ifdef MODULE
#include <linux/cdev.h>
#include <linux/device.h>
#endif

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

static DEFINE_PER_CPU(struct isaac_ctx, cpu_seeds);

/* Generate random bytes. */
#define IND(x) (*(u32 *)((u8 *)(mm) + ((x) % (RANDSIZB))))
#define STEP(mix) {			\
	x = *m;				\
	a = (a ^ (mix)) + *(m2++);	\
	*(m++) = y = IND(x) + a + b;	\
	*(r++) = b = IND(y >> RANDSIZL) + x;	\
}
static void isaac(struct isaac_ctx *ctx, u32 *buf)
{
	register u32 a, b, x, y, *m, *mm, *m2, *r, *mend;
	mm = ctx->mem;
	r = buf;
	a = ctx->a;
	b = ctx->b + (++ctx->randc);
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

#if defined(FUTURE_USE) && defined(UNTESTED)
/* Copy out random bytes, in no particular order */
static void isaac_copyout(struct isaac_ctx *ctx, void *mem, size_t len)
{
	int lim = RANDSIZB;

	if (len >= lim) {
		len -= lim;
		do {
			isaac(ctx, mem);
			mem += lim;
			len -= lim;
		} while (len >= 0);
		len += lim;
		if (!len)
			return;
	}

	lim -= ctx->idx;
	if (len > lim) {
		if (lim) {
			memcpy(mem, ctx->bytes + ctx->idx, lim);
			mem += lim;
			len -= lim;
		}

		isaac(ctx, ctx->res);
		ctx->idx = 0;
	}
	if (len) {
		memcpy(mem, ctx->bytes + ctx->idx, len);
		ctx->idx += len;
	}
}
#endif

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
static void isaac_init(struct isaac_ctx *seed, struct isaac_ctx *ctx)
{
	int i;
	u32 a, b, c, d, e, f, g, h;
	u32 *m, *r;

	m = ctx->mem;
	r = ctx->res;
	ctx->randa = ctx->randb = ctx->randc = 0;
	ctx->idx = RANDSIZB;
	a = b = c = d = e = f = g = h = 0x9e3779b9;

	for (i = 5; --i; ) {
		MIX();
	}

	if (likely(seed)) {
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
		for (i = 0; i < RANDSIZ; i += 8) {
			MIX();
			STORE();
		}
	}
}

static int isaac_open(struct inode *inode, struct file *filp)
{
	struct isaac_ctx *seed, *ctx;

	ctx = kmalloc(sizeof(struct isaac_ctx), GFP_KERNEL);

	seed = get_cpu_var(cpu_seeds);
	isaac_init(seed, ctx);
	put_cpu_var(cpu_seeds);

	sema_init(&ctx->sem, 1);

	filp->private_data = ctx;

	return 0;
}

static int isaac_release(struct inode *inode, struct file *filp)
{
	kfree(filp->private_data);
	return 0;
}

static ssize_t isaac_read(struct file *filp, char __user *buf, size_t count,
		loff_t *f_pos)
{
	struct isaac_ctx *ctx = (struct isaac_ctx *)filp->priavate_data;
	ssize_t cnt, ret = count;
	if (down_interruptible(ctx->sem))
		return -ERESTARTSYS;

	/* Copy out any existing bytes */
	if (ctx->idx != RANDSIZB) {
		cnt = min_t(size_t, len, RANDSIZB - ctx->idx);
		if (copy_to_user(mem, ctx->bytes + ctx->idx, cnt)) {
			ret = -EFAULT;
			goto out;
		}
		mem += cnt;
		len -= cnt;
	}
	/* Generate more bytes as needed */
	if (len) {
		do {
			isaac(ctx, ctx->res);
			cnt = min_t(size_t, len, RANDSIZB);
			if (copy_to_user(mem, ctx->bytes, cnt)) {
				ret = -EFAULT;
				goto out;
			}
			len -= cnt;
		} while (len);
		ctx->idx = RANDSIZB - cnt;
	}

out:
	up(ctx->sem);

	return (ssize_t)count;
}

const struct file_operations isaac_fops = {
	.open = isaac_open,
	.release = isaac_release,
	.read = isaac_read,
};

#ifdef MODULE
static struct class *isaac_class;
static struct cdev isaac_cdev;
static struct device *isaac_device;
static int __init register_device(void)
{
	int ret;
	
	isaac_class = class_create(THIS_MODULE, "fastrng");
	if (IS_ERR(isaac_class)) {
		printk(KERN_WARNING "%s: failed to register class\n", __func__);
		return PTR_ERR(isaac_class);
	}

	cdev_init(&isaac_cdev, &isaac_fops);
	isaac_cdev.owner = THIS_MODULE;
	ret = cdev_add(&isaac_cdev, MKDEV(235, 11), 1);
	if (ret < 0) {
		printk(KERN_WARNING "%s: failed to add cdev\n", __func__);
		goto out_destroy;
	}

	ret = register_chrdev_region(MKDEV(235, 11), 1, "/dev/isaac");
	if (ret < 0) {
		printk(KERN_WARNING "%s: failed to allocate major/minor\n",
			__func__);
		goto out_remove;
	}

	isaac_device = device_create(isaac_class, NULL, MKDEV(235, 11),
		NULL, "isaac");
	if (IS_ERR(isaac_device)) {
		printk(KERN_WARNING "%s: failed to create device\n", __func__);
		ret = PTR_ERR(isaac_device);
		goto out_unregister;
	}

	return 0;

out_unregister:
	unregister_chrdev_region(MKDEV(235, 11), 1);
out_remove:
	cdev_del(&isaac_cdev);
out_destroy:
	class_destroy(isaac_class);

	return ret;
}

static void __exit unregister_device(void)
{
	unreegister_chrdev_region(MKDEV(235, 11), 1);
	cdev_del(&isaac_cdev);
	device_destroy(isaac_class, MKDEV(235, 11));
	class_destroy(isaac_class);
}
module_exit(unregister_device);
#endif

/* FIXME: There's this new thing called locking */
static int __init isaac_seeds_init(void)
{
	int i, j, n;
	struct isaac_ctx *seed, *ctx;

	i = 0;
	j = num_possible_cpus() - 1;

	ctx = &per_cpu(cpu_seeds, 0);
	isaac_init(NULL, ctx);

	do {
		seed = ctx;
		ctx = &per_cpu(cpu_seeds, j);
		isaac_init(seed, ctx);
	} while (--j >= 0);

#ifdef MODULE
	return register_device();
#else
	return 0;
#endif
}
core_initcall(isaac_seeds_init);
