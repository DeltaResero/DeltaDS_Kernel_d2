/*
 * dkp.c: dkp kobject and handlers
 *
 * Copyright (c) 2013 Ryan Pennucci <decimalman@gmail.com>
 */
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/gen_attr.h>
#include <linux/export.h>

struct kobject *dkp_global_kobject;
EXPORT_SYMBOL(dkp_global_kobject);

void _dkp_register(struct attribute *gattr) {
        if (sysfs_create_file(dkp_global_kobject, gattr))
                printk(KERN_ERR "Couldn't register dkp entry: %s\n", gattr->name);
}
EXPORT_SYMBOL(_dkp_register);

static int __init register_dkp_kobject(void) {
        dkp_global_kobject = kobject_create();
        if (!dkp_global_kobject)
                return -ENOMEM;
        if (kobject_add(dkp_global_kobject, kernel_kobj, "dkp"))
                return -ENOMEM;

        return 0;
}
core_initcall(register_dkp_kobject);
