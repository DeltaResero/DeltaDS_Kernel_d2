/*
 * linux/include/linux/dkp.h
 *
 * dkp global stuff.  This is equal parts a handy place for lots of dkp
 * features and an easy way to get to a kobject to debug.  The structure used
 * is intended to be compatible with most existing kobjects.
 *
 * A generic handler is provided for int types.  It can handle (arrays of) ints
 * and int elements in (arrays of) structs, or can call arbitrary get/set
 * functions.  If provided, a callback function will be called after store().
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
	// Mostly standard xxx_attr struct (kobject type not guaranteed)
	struct attribute attr;
	ssize_t (*show)(struct kobject *, struct attribute *, char *);
	ssize_t (*store)(struct kobject *, struct attribute *, const char *, size_t);
	// Generic function information
	int min, max;			// min < val < max
	int cnt, stride;		// nr elements, bytes until next element
	void *ptr;			// ptr to first element or target for get/set
	int (*get)(void *);		// NULL or called to get *ptr
	void (*set)(void *, int);	// NULL or called to set *ptr = int
	void (*cb)(void);		// NULL or called after setting
	char *fmt;			// NULL or per-element printf format
};

extern struct kobject *dkp_global_kobject;

extern ssize_t dkp_generic_store(struct kobject *kobj, struct attribute *attr,
	const char *buf, size_t count);
extern ssize_t dkp_generic_show(struct kobject *kobj, struct attribute *attr,
	char *buf);
extern void _dkp_register(struct dkp_gattr *gattr);

#define dkp_register(attr) \
	_dkp_register((struct dkp_gattr *)&dkp_##attr)

static inline void _dkp_unregister(struct attribute *attr) {
	sysfs_remove_file(dkp_global_kobject, attr);
}
#define dkp_unregister(attr) \
	_dkp_unregister((struct attribute *)&dkp_##attr)

#define dkp_attrp(type, gattr) ((struct type##_attribute *)&dkp_##gattr)
#define dkp_attr(gattr) ((dkp_##gattr).attr)
#define dkp_gattr(attr) ((struct dkp_gattr *)&attr)

#define __DKPATTR(_name,_mode,_min,_max,_cnt,_stride,_ptr,_set,_get,_cb,_fmt) { \
	.attr = {.name = __stringify(_name), .mode = _mode}, \
	.store = dkp_generic_store, .show = dkp_generic_show, \
	.min = _min, .max = _max, .cnt = _cnt, .stride = _stride, \
	.ptr = (void *) _ptr, .set = _set, .get = _get, .cb = _cb, .fmt = _fmt, \
}

// single-int attr
#define __DKP_NAME(var,name,min,max,cb) \
struct dkp_gattr dkp_##name = __DKPATTR( \
	name, 0666, min, max, 1, 0, &var, NULL, NULL, cb, NULL)

#define __DKP(name,min,max,cb) __DKP_NAME(name,name,min,max,cb)

// int-array attr
#define __DKP_ARR_NAME(var,name,min,max,cb) \
struct dkp_gattr dkp_##name = __DKPATTR( \
	name, 0666, min, max, (sizeof(name)/sizeof(int)), sizeof(int), \
	var, NULL, NULL, cb, NULL)

#define __DKP_ARR(name,min,max,cb) __DKP_ARR_NAME(name,name,min,max,cb)

// single-struct attr
#define __DKP_STR_NAME(name,elm,str,min,max,cb) \
struct dkp_gattr dkp##name = __DKPATTR( \
	name, 0666, min, max, 1, 0, &str.elm, NULL, NULL, cb, NULL)

#define __DKP_STR(name,str,min,max,cb) __DKP_STR_NAME(name,name,str,min,max,cb)

// struct-array attr
#define __DKP_STR_ARR_NAME(name,elm,str,min,max,cb) \
struct dkp_gattr dkp##name = __DKPATTR( \
	name, 0666, min, max, (sizeof(str)/sizeof(str[0])), sizeof(str[0]), \
	&str.elm, NULL, NULL, cb, NULL)

#define __DKP_STR_ARR(name,str,min,max,cb) __DKP_STR_NAME(name,name,str,min,max,cb)

#endif
