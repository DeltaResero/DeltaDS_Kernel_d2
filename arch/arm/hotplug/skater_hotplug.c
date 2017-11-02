/*
 * drivers/soc/qcom/skater_hotplug.c
 *
 * New hotplug based off of AutoSMP. Limit the number of cores online while screen is off.
 * online cores on screen interaction as well. By Lonelyoneskatter <threesixoh.skater_hotplug@yahoo.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version. For more details, see the GNU
 * General Public License included with the Linux kernel or available
 * at www.gnu.org/licenses
 */

#include <linux/moduleparam.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/state_notifier.h>

#define SKATER_TAG			"skater_hotplug:"
#define SKATER_ENABLED			true
#define DEFAULT_BOOST_LOCK_DUR		1500 * 1000L
#define DEFAULT_MAX_CPUS                NR_CPUS
#define DEFAULT_NR_CPUS_BOOSTED		DEFAULT_MAX_CPUS
#define DEFAULT_UPDATE_RATE		30
#define MIN_INPUT_INTERVAL		150 * 1000L
#define DEFAULT_MIN_BOOST_FREQ		1242000

static struct delayed_work skater_hotplug_work;
static struct workqueue_struct *skater_hotplug_workq;
static bool enabled_switch = SKATER_ENABLED;

static struct skater_hotplug_struct {
	unsigned int enabled;
	unsigned int delay;
	unsigned int max_cpus;
	unsigned int min_cpus;
	unsigned int cpufreq_up;
	unsigned int cpufreq_down;
	unsigned int cycle_up;
	unsigned int cycle_down;
	unsigned int cpus_boosted;
	unsigned int min_boost_freq;
	u64 boost_lock_dur;
	struct notifier_block notif;
} skater_hotplug = {
	.enabled = SKATER_ENABLED,
	.delay = DEFAULT_UPDATE_RATE,
	.max_cpus = DEFAULT_MAX_CPUS,
	.min_cpus = DEFAULT_MAX_CPUS,
	.cpufreq_up = 95,
	.cpufreq_down = 80,
	.cycle_up = 1,
	.cycle_down = 1,
	.min_boost_freq = DEFAULT_MIN_BOOST_FREQ,
	.cpus_boosted = DEFAULT_NR_CPUS_BOOSTED,
	.boost_lock_dur = DEFAULT_BOOST_LOCK_DUR,
};

static u64 last_boost_time;
static unsigned int cycle = 0;

static void reschedule_hotplug_work(void)
{
	queue_delayed_work(skater_hotplug_workq, &skater_hotplug_work,
			msecs_to_jiffies(skater_hotplug.delay));
}

static void max_min_check(void)
{
	skater_hotplug.max_cpus = max((unsigned int)1, skater_hotplug.max_cpus);
	skater_hotplug.min_cpus = max((unsigned int)1, skater_hotplug.min_cpus);

	if (skater_hotplug.max_cpus > NR_CPUS)
		skater_hotplug.max_cpus = NR_CPUS;
	if (skater_hotplug.min_cpus > skater_hotplug.max_cpus)
		skater_hotplug.min_cpus = skater_hotplug.max_cpus;
}

static void __cpuinit skater_hotplug_work_fn(struct work_struct *work)
{
	unsigned int cpu = 0, slow_cpu = 0;
	unsigned int rate, cpu0_rate, slow_rate = UINT_MAX, fast_rate;
	unsigned int max_rate, up_rate, down_rate;
	unsigned int nr_cpu_online;
	unsigned int min_boost_freq = skater_hotplug.min_boost_freq;
	u64 now;
	
	if (!skater_hotplug.enabled)
		return;

	/* get maximum possible freq for cpu0 and
	   calculate up/down limits */
	max_rate  = cpufreq_quick_get_max(cpu);
	up_rate   = (max_rate / 100) * skater_hotplug.cpufreq_up;
	down_rate = (max_rate / 100) * skater_hotplug.cpufreq_down;

	/* find current max and min cpu freq to estimate load */
	nr_cpu_online = num_online_cpus();
	cpu0_rate = cpufreq_quick_get(cpu);
	fast_rate = cpu0_rate;

	for_each_online_cpu(cpu) {
		if (cpu) {
			rate = cpufreq_quick_get(cpu);
			if (rate <= slow_rate) {
				slow_cpu = cpu;
				slow_rate = rate;
			} else if (rate > fast_rate)
				fast_rate = rate;
		}
	}

	if (cpu0_rate < slow_rate)
		slow_rate = cpu0_rate;

	if (max_rate <= skater_hotplug.min_boost_freq)
		min_boost_freq = max_rate;

	now = ktime_to_us(ktime_get());
	/* hotplug one core if all online cores are over up_rate limit */
	if (slow_rate > up_rate && fast_rate >= min_boost_freq) {
		if (nr_cpu_online < skater_hotplug.max_cpus &&
				cycle >= skater_hotplug.cycle_up) {
			cpu = cpumask_next_zero(0, cpu_online_mask);
			if (cpu_is_offline(cpu))
				cpu_up(cpu);
			cycle = 0;
		}
	/* check if boost required */
	} else if (nr_cpu_online <= skater_hotplug.cpus_boosted &&
			now - last_boost_time <= skater_hotplug.boost_lock_dur) {
		if (nr_cpu_online < skater_hotplug.cpus_boosted &&
			nr_cpu_online < skater_hotplug.max_cpus) {
			cpu = cpumask_next_zero(0, cpu_online_mask);
			if (cpu_is_offline(cpu))
				cpu_up(cpu);
			// cycle = 0;
		}
	/* unplug slowest core if all online cores are under down_rate limit */
	} else if (slow_cpu && (fast_rate < down_rate)) {
		if (nr_cpu_online > skater_hotplug.min_cpus &&
				cycle >= skater_hotplug.cycle_down) {

			if (cpu_online(slow_cpu))
	 			cpu_down(slow_cpu);
			cycle = 0;
		}
	} /* else do nothing */

	cycle++;
	reschedule_hotplug_work();
}

static void skater_hotplug_suspend(void)
{
	/* Flush hotplug workqueue */
	flush_workqueue(skater_hotplug_workq);
	cancel_delayed_work_sync(&skater_hotplug_work);
}

static void __ref skater_hotplug_resume(void)
{
	unsigned int cpu;

	/* Fire up all CPUs */
	for_each_possible_cpu(cpu)
		if (cpu_is_offline(cpu))
			cpu_up(cpu);

	last_boost_time = ktime_to_us(ktime_get());

	/* Resume hotplug workqueue */
	INIT_DELAYED_WORK(&skater_hotplug_work, skater_hotplug_work_fn);
	reschedule_hotplug_work();
}

static int state_notifier_callback(struct notifier_block *this,
				unsigned long event, void *data)
{
	if (!skater_hotplug.enabled)
                return NOTIFY_OK;

	switch (event) {
		case STATE_NOTIFIER_ACTIVE:
			skater_hotplug_resume();
			break;
		case STATE_NOTIFIER_SUSPEND:
			skater_hotplug_suspend();
			break;
		default:
			break;
	}

	return NOTIFY_OK;
}

static void skater_hotplug_input_event(struct input_handle *handle, unsigned int type,
				unsigned int code, int value)
{
	u64 now;

	if (!skater_hotplug.enabled || state_suspended)
		return;

	now = ktime_to_us(ktime_get());
	if (now - last_boost_time < MIN_INPUT_INTERVAL)
		return;

	if (skater_hotplug.cpus_boosted <= skater_hotplug.min_cpus)
		return;

	last_boost_time = ktime_to_us(ktime_get());
}

static int skater_hotplug_input_connect(struct input_handler *handler,
				 struct input_dev *dev,
				 const struct input_device_id *id)
{
	struct input_handle *handle;
	int err;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = handler->name;

	err = input_register_handle(handle);
	if (err)
		goto err_register;

	err = input_open_device(handle);
	if (err)
		goto err_open;

	return 0;
err_open:
	input_unregister_handle(handle);
err_register:
	kfree(handle);
	return err;
}

static void skater_hotplug_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id skater_hotplug_ids[] = {
	/* multi-touch touchscreen */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			BIT_MASK(ABS_MT_POSITION_X) |
			BIT_MASK(ABS_MT_POSITION_Y) },
	},
	/* touchpad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_X)] =
			BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) },
	},
	/* Keypad */
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};

static struct input_handler skater_hotplug_input_handler = {
	.event		= skater_hotplug_input_event,
	.connect	= skater_hotplug_input_connect,
	.disconnect	= skater_hotplug_input_disconnect,
	.name		= SKATER_TAG,
	.id_table	= skater_hotplug_ids,
};

static int hotplug_start(void)
{
	int ret = 0;

	skater_hotplug_workq =
		alloc_workqueue("skater_hotplug_wq",
			WQ_HIGHPRI | WQ_UNBOUND | WQ_FREEZABLE, 0);

	if (!skater_hotplug_workq) {
		pr_err("%s: Failed to allocate hotplug workqueue\n",
					SKATER_TAG);
		ret = -ENOMEM;
		goto err_wq;
	}

	skater_hotplug.notif.notifier_call = state_notifier_callback;
	if (state_register_client(&skater_hotplug.notif)) {
		pr_err("%s: Failed to register State notifier callback\n",
			SKATER_TAG);
		goto err_notif;
	}

	ret = input_register_handler(&skater_hotplug_input_handler);
	if (ret) {
		pr_err("%s: Failed to register input handler: %d\n",
		       SKATER_TAG, ret);
		goto err;
	}

	INIT_DELAYED_WORK(&skater_hotplug_work, skater_hotplug_work_fn);
	max_min_check();
	reschedule_hotplug_work();
	return ret;

err:
	state_unregister_client(&skater_hotplug.notif);
err_notif:
	skater_hotplug.notif.notifier_call = NULL;
	destroy_workqueue(skater_hotplug_workq);
err_wq:
	skater_hotplug.enabled = false;
	return ret;
}

static void __ref hotplug_stop(void)
{
	int cpu;

	input_unregister_handler(&skater_hotplug_input_handler);
	state_unregister_client(&skater_hotplug.notif);
	skater_hotplug.notif.notifier_call = NULL;
	flush_workqueue(skater_hotplug_workq);
	cancel_delayed_work_sync(&skater_hotplug_work);
	destroy_workqueue(skater_hotplug_workq);

	/* Wake up all the sibling cores */
	for_each_possible_cpu(cpu)
		if (cpu_is_offline(cpu))
			cpu_up(cpu);
}

/***************************** SYSFS START *****************************/
#define define_one_global_ro(_name)					\
static struct global_attr _name =					\
__ATTR(_name, 0444, show_##_name, NULL)

#define define_one_global_rw(_name)					\
static struct global_attr _name =					\
__ATTR(_name, 0644, show_##_name, store_##_name)

struct kobject *skater_hotplug_kobject;

#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%u\n", skater_hotplug.object);			\
}
show_one(delay, delay);
show_one(min_cpus, min_cpus);
show_one(max_cpus, max_cpus);
show_one(cpufreq_up, cpufreq_up);
show_one(cpufreq_down, cpufreq_down);
show_one(cycle_up, cycle_up);
show_one(cycle_down, cycle_down);

#define store_one(file_name, object)					\
static ssize_t store_##file_name					\
(struct kobject *a, struct attribute *b, const char *buf, size_t count)	\
{									\
	unsigned int input;						\
	int ret;							\
	ret = sscanf(buf, "%u", &input);				\
	if (ret != 1)							\
		return -EINVAL;						\
	skater_hotplug.object = input;					\
	max_min_check();						\
	if (skater_hotplug.enabled)						\
		reschedule_hotplug_work();				\
	return count;							\
}									\
define_one_global_rw(file_name);
store_one(delay, delay);
store_one(min_cpus, min_cpus);
store_one(max_cpus, max_cpus);
store_one(cpufreq_up, cpufreq_up);
store_one(cpufreq_down, cpufreq_down);
store_one(cycle_up, cycle_up);
store_one(cycle_down, cycle_down);

static int __cpuinit set_enabled(const char *val, const struct kernel_param *kp)
{
	int ret = 0;

	ret = param_set_bool(val, kp);
	if (ret)
		return -EINVAL;

	if (enabled_switch == skater_hotplug.enabled)
		return ret;

	enabled_switch = skater_hotplug.enabled;

	if (skater_hotplug.enabled)
		hotplug_start();
	else
		hotplug_stop();

	return ret;
}

static struct kernel_param_ops module_ops = {
	.set = set_enabled,
	.get = param_get_bool,
};
module_param_cb(enabled, &module_ops, &skater_hotplug.enabled, 0644);
MODULE_PARM_DESC(enabled, "hotplug/unplug cpu cores based on cpu load");

static ssize_t show_boost_lock_duration(struct device *dev,
				        struct device_attribute
				        *skater_hotplug_attributes, char *buf)
{
	return sprintf(buf, "%llu\n", div_u64(skater_hotplug.boost_lock_dur, 1000));
}

static ssize_t store_boost_lock_duration(struct device *dev,
					 struct device_attribute
					 *skater_hotplug_attributes, const char *buf,
					 size_t count)
{
	int ret;
	u64 val;

	ret = sscanf(buf, "%llu", &val);
	if (ret != 1)
		return -EINVAL;

	skater_hotplug.boost_lock_dur = val * 1000;

	return count;
}

static ssize_t show_cpus_boosted(struct device *dev,
				 struct device_attribute *skater_hotplug_attributes,
				 char *buf)
{
	return sprintf(buf, "%u\n", skater_hotplug.cpus_boosted);
}

static ssize_t store_cpus_boosted(struct device *dev,
				  struct device_attribute *skater_hotplug_attributes,
				  const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	ret = sscanf(buf, "%u", &val);
	if (ret != 1 || val < 1 || val > CONFIG_NR_CPUS)
		return -EINVAL;

	skater_hotplug.cpus_boosted = val;

	return count;
}


static ssize_t show_min_boost_freq(struct device *dev,
				   struct device_attribute *skater_hotplug_attributes,
				   char *buf)
{
	return sprintf(buf, "%u\n", skater_hotplug.min_boost_freq);
}

static ssize_t store_min_boost_freq(struct device *dev,
				    struct device_attribute *skater_hotplug_attributes,
				    const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	ret = sscanf(buf, "%u", &val);
	if (ret != 1)
		return -EINVAL;

	skater_hotplug.min_boost_freq = val;

	return count;
}

static DEVICE_ATTR(boost_lock_duration, 644, show_boost_lock_duration,
		   store_boost_lock_duration);
static DEVICE_ATTR(cpus_boosted, 644, show_cpus_boosted, store_cpus_boosted);
static DEVICE_ATTR(min_boost_freq, 644, show_min_boost_freq,
		   store_min_boost_freq);

static struct attribute *skater_hotplug_attributes[] = {
	&delay.attr,
	&min_cpus.attr,
	&max_cpus.attr,
	&cpufreq_up.attr,
	&cpufreq_down.attr,
	&cycle_up.attr,
	&cycle_down.attr,
	&dev_attr_boost_lock_duration.attr,
	&dev_attr_cpus_boosted.attr,
	&dev_attr_min_boost_freq.attr,
	NULL
};

static struct attribute_group skater_hotplug_attr_group = {
	.attrs = skater_hotplug_attributes,
};

/****************************** SYSFS END ******************************/

static int __init skater_hotplug_init(void)
{
	int ret = 0;

	skater_hotplug.max_cpus = NR_CPUS;

	skater_hotplug_kobject = kobject_create_and_add("skater_hotplug", kernel_kobj);

	if (skater_hotplug_kobject) {
		ret = sysfs_create_group(skater_hotplug_kobject, &skater_hotplug_attr_group);
		if (ret) {
			pr_warn(SKATER_TAG "sysfs: ERROR, create sysfs group.");
			goto err_dev;
		}
	} else {
		pr_warn(SKATER_TAG "sysfs: ERROR, create sysfs kobj");
		goto err_dev;
	}

	if (skater_hotplug.enabled)
		hotplug_start();

	pr_info(SKATER_TAG "Init complete.\n");
	return ret;

err_dev:
	skater_hotplug.enabled = false;
	return ret;
}

late_initcall(skater_hotplug_init);
