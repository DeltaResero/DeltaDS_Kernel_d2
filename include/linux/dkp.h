/*
 * linux/include/linux/dkp.h
 *
 * dkp global stuff.  This is equal parts a handy place for lots of dkp
 * features and an easy way to get to a kobject to debug.
 *
 * kobject appears at /sys/kernel/dkp
 *
 */
#ifndef _LINUX_DKP_H
#define _LINUX_DKP_H

#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/string.h>

// This structure is mostly compatible with other atttribute structs
struct dkp_gattr {
        struct attribute attr;
	ssize_t (*show)(struct kobject *, struct attribute *, char *);
	ssize_t (*store)(struct kobject *, struct attribute *, const char *, size_t);
        int min, max, cnt;
        int *ptr;
	void (*cb)(void);
        char *fmt;
};

extern struct kobject *dkp_global_kobject;

extern ssize_t dkp_generic_store(struct kobject *kobj, struct attribute *attr,
	const char *buf, size_t count);
extern ssize_t dkp_generic_show(struct kobject *kobj, struct attribute *attr,
	char *buf);
extern void _dkp_register(struct dkp_gattr *gattr);

#define dkp_register(attr) \
	_dkp_register((struct dkp_gattr *)&dkp_##attr)

/* TODO: unregister funcs */

#define dkp_attrp(type, gattr) ((struct type##_attribute *)&dkp_##gattr)
#define dkp_attr(gattr) ((dkp_##gattr).attr)
#define dkp_gattr(attr) ((struct dkp_gattr *)&attr)

#define __DKPATTR(_name,_mode,_min,_max,_cnt,_ptr,_cb,_fmt) { \
	.attr = {.name = __stringify(_name), .mode = _mode}, \
	.store = dkp_generic_store, .show = dkp_generic_show, \
	.min = _min, .max = _max, .cnt = _cnt, \
	.ptr = _ptr, .cb = _cb, .fmt = _fmt, \
}

#define __DKP_NAME(var,name,min,max,cb) \
struct dkp_gattr dkp_##name = __DKPATTR( \
	name, 0666, min, max, 1, &var, cb, NULL)

#define __DKP(name,min,max,cb) __DKP_NAME(name,name,min,max,cb)

#define __DKP_ARR_NAME(var,name,min,max,cb) \
struct dkp_gattr dkp_##name = __DKPATTR( \
	name, 0666, min, max, (sizeof(name)/sizeof(int)), var, cb, NULL)

#define __DKP_ARR(name,min,max,cb) __DKP_ARR_NAME(name,name,min,max,cb)

#endif
