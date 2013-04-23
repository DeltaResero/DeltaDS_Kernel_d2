/*
 * dkp.c: dkp kobject and generic handlers
 */
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/dkp.h>
#include <linux/export.h>

struct kobject *dkp_global_kobject;
EXPORT_SYMBOL(dkp_global_kobject);

void _dkp_register(struct dkp_gattr *gattr) {
        if (sysfs_create_file(dkp_global_kobject, (struct attribute *)gattr))
                printk(KERN_ERR "Couldn't register dkp entry: %s\n", gattr->attr.name);
}

/* Pro tip: don't touch kobj.  It's probably not a kobject. */
ssize_t dkp_generic_store(struct kobject *kobj, struct attribute *attr,
		const char *buf, size_t count) {
	int i, l, t, r;
	int *v;
	struct dkp_gattr *gattr = (struct dkp_gattr *)attr;
	if (gattr->store && gattr->store != dkp_generic_store)
		return gattr->store(kobj, (struct attribute *)gattr, buf, count);
	v = kmalloc(gattr->cnt * sizeof(int), GFP_KERNEL);
	if (!v)
		return -ENOMEM;
	for (i = 0, t = 0; i < gattr->cnt; i++) {
		r = sscanf(buf + t, " %i%n", &v[i], &l);
		if (!r || v[i] < gattr->min || v[i] > gattr->max) break;
		t += l;
	}
	while (buf[t] == ' ' && t < count) l++;
	if (i != gattr->cnt || t < count - 1) {
		count = -EINVAL;
		goto out;
	}
	memcpy(gattr->ptr, v, gattr->cnt * sizeof(int));
	if (gattr->cb)
		gattr->cb();
out:
	kfree(v);
	return count;
}

ssize_t dkp_generic_show(struct kobject *kobj, struct attribute *attr, char *buf) {
	char *fmt;
	int i, l;
	struct dkp_gattr *gattr = (struct dkp_gattr *)attr;
	if (gattr->show && gattr->show != dkp_generic_show)
		return gattr->show(kobj, (struct attribute *)gattr, buf);
	fmt = gattr->fmt ? : "%i ";
	for (i = 0, l = 0; i < gattr->cnt; i++)
		l += sprintf(buf + l, fmt, gattr->ptr[i]);
	buf[l-1] = '\n';
	return l;
}

static struct sysfs_ops sysfs_ops = {
	.show = dkp_generic_show,
	.store = dkp_generic_store,
};

static struct kobj_type ktype_dkp = {
	.sysfs_ops = &sysfs_ops,
};

static int __init register_dkp_kobject(void) {
	dkp_global_kobject = kobject_create();
	if (!dkp_global_kobject)
		return -ENOMEM;
	if (kobject_init_and_add(dkp_global_kobject,
		&ktype_dkp, kernel_kobj, "dkp"))
		return -ENOMEM;

	return 0;
}
core_initcall(register_dkp_kobject);
