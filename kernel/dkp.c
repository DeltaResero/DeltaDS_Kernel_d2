/*
 * dkp.c: dkp kobject and generic handlers
 *
 * Copyright (C) 2013 Ryan Pennucci
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
EXPORT_SYMBOL(_dkp_register);

/* Pro tip: don't touch kobj.  It's probably not a kobject. */
ssize_t dkp_generic_store(struct kobject *kobj, struct attribute *attr,
		const char *buf, size_t count) {
	int i, l, t, r;
	int *v;
	void *p;
	struct dkp_gattr *gattr = (struct dkp_gattr *)attr;
	if (gattr->store && gattr->store != dkp_generic_store)
		return gattr->store(kobj, attr, buf, count);
	v = kmalloc(gattr->cnt * sizeof(int), GFP_KERNEL);
	if (!v)
		return -ENOMEM;
	for (i = 0, t = 0; i < gattr->cnt; i++) {
		r = sscanf(buf + t, " %i%n", &v[i], &l);
		if (!r || v[i] < gattr->min || v[i] > gattr->max) break;
		t += l;
	}
	while (buf[t] == ' ' && t < count) t++;
	if (i != gattr->cnt || t < count - 1) {
		count = -EINVAL;
		goto out;
	}
	p = gattr->ptr;
	for (i = 0; i < gattr->cnt; i++) {
		if (gattr->set)
			gattr->set(p, v[i]);
		else
			*((int *)p) = v[i];
		p += gattr->stride;
	}
	if (gattr->cb)
		gattr->cb();
out:
	kfree(v);
	return count;
}
EXPORT_SYMBOL(dkp_generic_store);

ssize_t dkp_generic_show(struct kobject *kobj, struct attribute *attr, char *buf) {
	char *fmt;
	int i, l, v;
	void *p;
	struct dkp_gattr *gattr = (struct dkp_gattr *)attr;
	if (gattr->show && gattr->show != dkp_generic_show)
		return gattr->show(kobj, attr, buf);
	fmt = gattr->fmt ? : "%i ";
	p = gattr->ptr;
	for (i = 0, l = 0; i < gattr->cnt; i++) {
		if (gattr->get)
			v = gattr->get(p);
		else
			v = *((int *)p);
		l += sprintf(buf + l, fmt, v);
		p += gattr->stride;
	}
	buf[l-1] = '\n';
	return l;
}
EXPORT_SYMBOL(dkp_generic_show);

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
