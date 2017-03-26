/*
 * linux/include/linux/skatter.h
 *
 * Copyright (c) 2013 Ryan Pennucci <decimalman@gmail.com>
 * 
 * All credits got to decimalman, just renamed to fit skatter
 *
 * A kobject is placed in /sys/kernel/skatter.  It is intended to collect
 * attributes that don't logically fit elsewhere, but should still be
 * available.
 */

#ifndef _LINUX_SKATTER_H
#define _LINUX_SKATTER_H

#include <linux/gen_attr.h>
#include <linux/sysfs.h>

extern struct kobject *skatter_global_kobject;

extern void _skatter_register(struct attribute *gattr);
#define skatter_register(attr) \
	_skatter_register(&gen_attr(attr))

static inline void _skatter_unregister(struct attribute *attr) {
        sysfs_remove_file(skatter_global_kobject, attr);
}
#define skatter_unregister(attr) \
	_skatter_unregister(&gen_attr(attr))

#endif
