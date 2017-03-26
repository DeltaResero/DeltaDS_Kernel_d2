/*
 * skatter.c: skatter kobject and handlers
 *
 * Copyright (c) 2013 Ryan Pennucci <decimalman@gmail.com>
 *
 * All credits to decimalman, just renamed to skatter.
 *
 */
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/gen_attr.h>
#include <linux/export.h>

struct kobject *skatter_global_kobject;
EXPORT_SYMBOL(skatter_global_kobject);

void _skatter_register(struct attribute *gattr) {
	if (!skatter_global_kobject) {
		printk(KERN_ERR "%s: called before kobject initialized (%s)!\n",
			__func__, gattr->name);
		return;
	}
	if (sysfs_create_file(skatter_global_kobject, gattr))
		printk(KERN_ERR "Couldn't register skatter entry: %s\n", gattr->name);
}
EXPORT_SYMBOL(_skatter_register);

static int __init register_skatter_kobject(void) {
        skatter_global_kobject = kobject_create();
        if (!skatter_global_kobject)
                return -ENOMEM;
        if (kobject_add(skatter_global_kobject, kernel_kobj, "skatter"))
                return -ENOMEM;

	return 0;
}
core_initcall(register_skatter_kobject);
