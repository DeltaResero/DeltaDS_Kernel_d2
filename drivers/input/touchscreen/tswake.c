/* tswake.c: logic for touchscreen wakeup
 * Copyright (C) 2015, Ryan Pennucci <decimalman@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/tswake.h>
#include <linux/wakelock.h>

#define SWEEP_SIZE (screen_x / 2)

static struct wake_lock force_wake;

static struct input_dev *pwrkey_dev;
static struct device *ts_i2c;
static int screen_x, screen_y;
static enum tswake_type {
	TSMODE_OFF	= 0,
	TSMODE_S2W	= 1,
	TSMODE_DT2W	= 2,
	TSMODE_DT2W_FS	= 4,
	TSMODE_DT2W_MASK = TSMODE_DT2W | TSMODE_DT2W_FS,
} tswake_mode = TSMODE_OFF;

void tswake_notify_pwrkey(struct input_dev *dev) {
	pwrkey_dev = dev;
}

void tswake_notify_resolution(int x, int y) {
	screen_x = x;
	screen_y = y;
}

void tswake_notify_i2c(struct device *dev) {
	ts_i2c = dev;
}

int tswake_active(void) {
	return ((tswake_mode) != TSMODE_OFF);
}

void tswake_i2c_enable(void) {
	if (!ts_i2c)
		return;

	/* NB: for QUP, this will wake the bus and request an autosuspend.
	 * Other drivers may need additional management.
	 */
	platform_pm_resume(ts_i2c);
}

static int handler_s2w(int x, int y) {
	enum s2w_state {
		S2W_NIL, // not touching screen
		S2W_DOWN, // pressed, but not yet valid
		S2W_VALID, // complete swipe
		S2W_INVAL, // bogus swipe
	};
	static enum s2w_state oldstate = S2W_NIL, newstate = S2W_NIL;
	static int init_x = -1, init_y = -1;

	int ret = 0;

	newstate = (x != -1) ? S2W_DOWN : S2W_NIL;

	switch (oldstate) {
	case S2W_NIL:
		if (newstate == S2W_DOWN) {
			init_x = x;
			init_y = y;
		}
		break;

	case S2W_VALID:
		if (newstate == S2W_NIL)
			ret = 1;
		// fall through
	case S2W_DOWN:
		if (newstate == S2W_DOWN) {
			if (abs(y - init_y) > 200)
				newstate = S2W_INVAL;
			else if (abs(x - init_x) > SWEEP_SIZE)
				newstate = S2W_VALID;
		}
		break;

	case S2W_INVAL:
		if (newstate == S2W_DOWN)
			newstate = S2W_INVAL;
	}

	oldstate = newstate;
	return ret;
}

static int tap_time_valid(ktime_t now, ktime_t then)
{
	return (ktime_us_delta(now, then) > 5 * USEC_PER_MSEC) &&
	       (ktime_us_delta(now, then) < 250 * USEC_PER_MSEC);
}

static int tap_loc_valid(int x, int y, int last_x, int last_y)
{
	return (abs(x - last_x) < 150) && (abs(y - last_y) < 150);
}

static int handler_dt2w(int x, int y) {
	enum dt2w_state {
		DT2W_NIL, // not touching screen
		DT2W_DOWN1, // touching screen
		DT2W_UP, // first tap is released
		DT2W_DOWN2, // touching screen again before timeout
	};
	static enum dt2w_state oldstate = DT2W_NIL, newstate = DT2W_NIL;
	static ktime_t last_kt;
	static int init_x = -1, init_y = -1;

	int ret = 0;
	ktime_t kt = ktime_get();

	newstate = (x != -1) ? DT2W_DOWN1 : DT2W_NIL;

	if (newstate == DT2W_DOWN1) {
		if (!(tswake_mode & TSMODE_DT2W_FS) &&
		    y < screen_y / 2)
			newstate = DT2W_NIL;
	}

	switch (oldstate) {
	case DT2W_NIL:
		break;

	case DT2W_DOWN1:
		if (newstate == DT2W_NIL) {
			if (tap_time_valid(kt, last_kt))
				newstate = DT2W_UP;
		}
		break;

	case DT2W_UP:
		if (newstate == DT2W_DOWN1) {
			if (tap_time_valid(kt, last_kt) &&
			    tap_loc_valid(x, y, init_x, init_y))
				newstate = DT2W_DOWN2;
		} else {
			newstate = DT2W_UP;
		}
		break;

	case DT2W_DOWN2:
		if (newstate == DT2W_NIL) {
			if (tap_time_valid(kt, last_kt))
				ret = 1;
			else
				newstate = DT2W_UP;
		} else {
			newstate = DT2W_DOWN2;
		}
		break;
	}

	if (newstate != oldstate) {
		last_kt = kt;
		if (newstate == DT2W_DOWN1) {
			init_x = x;
			init_y = y;
		}
	}
	oldstate = newstate;
	return ret;
}

static void request_wakeup(void) {
	wake_lock_timeout(&force_wake, HZ * 2);

	input_report_key(pwrkey_dev, KEY_POWER, 1);
	input_sync(pwrkey_dev);
	input_report_key(pwrkey_dev, KEY_POWER, 0);
	input_sync(pwrkey_dev);
}

void tswake_touch_event(int x, int y) {
	int do_wake = 0;

	if ((tswake_mode & TSMODE_S2W) && handler_s2w(x, y))
		do_wake = 1;
	if ((tswake_mode & TSMODE_DT2W) && handler_dt2w(x, y))
		do_wake = 1;

	if (do_wake)
		request_wakeup();
}

static struct param_map {
	const char *name;
	const char mask;
} params[] = {
	// Fix for Kernel Adiutor: use s2w_s2sonly to avoid sweep2sleep option
	//{ "sweep2wake", TSMODE_S2W },
	{ "s2w_s2sonly", TSMODE_S2W },
	{ "doubletap2wake", TSMODE_DT2W_MASK },
};

static enum tswake_type find_param_type(struct kobj_attribute *attr)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(params); i++)
		if (!strcmp(attr_name(*attr), params[i].name))
			return params[i].mask;
	return 0;
}

static ssize_t show_enabled(struct kobject *kobj, struct kobj_attribute *attr,
                            char *buf)
{
	int param = find_param_type(attr);
	int val;

	val = !!(tswake_mode & param);
	if (param == TSMODE_DT2W_MASK && (tswake_mode & TSMODE_DT2W_FS))
		val = 2;

	return sprintf(buf, "%i\n", val);
}

static ssize_t store_enabled(struct kobject *kobj, struct kobj_attribute *attr,
                           const char *buf, size_t count)
{
	enum tswake_type param = find_param_type(attr);
	enum tswake_type set = param;
	int val, ret = sscanf(buf, "%u", &val);
	if (ret != 1)
		return -EINVAL;

	if (!param)
		return -ENOENT;

	if (param == TSMODE_DT2W_MASK && val == 1)
		set = TSMODE_DT2W;

	tswake_mode = (tswake_mode & ~param) | set;

	return count;
}

#define mode(n) __ATTR(n, 0644, show_enabled, store_enabled)
static struct kobj_attribute ts_attrs[] = {
	//mode(sweep2wake),
	mode(s2w_s2sonly),
	mode(doubletap2wake),
};

static int __init tswake_init(void) {
	struct kobject *kobj;
	wake_lock_init(&force_wake, WAKE_LOCK_SUSPEND, "tswake");

	if (kobj = kobject_create_and_add("android_touch", NULL)) {
		int i;
		for (i = 0; i < ARRAY_SIZE(ts_attrs); i++) {
			if (sysfs_create_file(kobj, &ts_attrs[i].attr)) {
				printk(KERN_ERR "%s: error registering %s\n",
					__func__, attr_name(ts_attrs[i]));
			}
		}
	} else {
		printk(KERN_ERR "%s: error creating kobject\n", __func__);
	}

	return 0;
}
late_initcall(tswake_init);
