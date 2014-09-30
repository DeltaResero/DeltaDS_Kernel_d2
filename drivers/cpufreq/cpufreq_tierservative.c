/* drivers/cpufreq/cpufreq_tierservative.c -- yet another cpufreq governor
 * Copyright (C) 2014, Ryan Pennucci <decimalman@gmail.com>
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

/*
 * tierservative ("ts") is conceptually different from any other CPU frequency
 * governor that I'm aware of.  Rather than step through frequencies gradually
 * (cf. conservative) or quickly jumping to a preset/calculated frequency (cf.
 * ondemand), ts maintains an extensive CPU usage history intended to predict
 * future usage.
 *
 * The exact details of how the usage history is maintained and used isn't
 * terribly important, and is subject to dramatic change at any moment.  In
 * general, usage is categorized into "tiers" of increasing demand, and the
 * average demand for a tier is used to estimate future demand.
 */

/*
 * TODO: add support for multi-CPU cpufreq policies
 * TODO: increase accuracy of usage_sample accessors
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/list.h>

#define USAGE_DIV	(100) // Use percent as user-facing usage unit
#define MAX_TIER	(16)

/* struct ts_global_priv:
 * Shared state & configuration.
 */
struct ts_global_priv {
	struct mutex		global_mutex;
	unsigned int		enabled;
	struct notifier_block	hotplug_notifier;
	struct kmem_cache	*sample_cache;
	bool			notifiers_registered;

	unsigned int		tier_count;
	unsigned int		extra_mhz;
	unsigned int		delay_up[MAX_TIER];
	unsigned int		delay_down[MAX_TIER];
	unsigned int		sample_time;
	unsigned int		max_sample;
	unsigned int		active_sample;
	unsigned int		int_timeout;
};

#define MS(x) DIV_ROUND_UP(x*HZ, 1000)
static struct ts_global_priv ts_global __read_mostly = {
	.tier_count = 8,
	.extra_mhz = 200,
	.sample_time = MS(16),
	.max_sample = 333,
	.active_sample = 100,
	.int_timeout = MS(50),
	.delay_up = { MS( 16), MS( 16), MS( 16), MS( 20), MS( 30), MS( 50),
		      MS(100), MS(200), MS(200), MS(200), MS(200), MS(200),
		      MS(200), MS(200), MS(200), MS(200) },
	.delay_down = { MS(  0), MS(100), MS( 75), MS( 50), MS( 40), MS( 30),
			MS( 75), MS(100), MS(100), MS(100), MS(100), MS(100),
			MS(100), MS(100), MS(100), MS(100) },
};

static void rebuild_all_privs(void);

#define TKNOB(fn, obj, min, max, mul)			\
static ssize_t show_##fn				\
(struct kobject *k, struct attribute *a, char *buf)	\
{							\
	int i, ret = 0;					\
	for (i = 0; i < MAX_TIER; i++)			\
		ret += sprintf(buf + ret, "%u ",	\
			ts_global.obj[i] * mul);	\
	buf[ret - 1] = '\n';				\
	return ret;					\
}							\
static ssize_t store_##fn				\
(struct kobject *k, struct attribute *a,		\
const char *buf, size_t count)				\
{							\
	int i, v[MAX_TIER], ret = 0, len, pos = 0;	\
	for (i = 0; i < MAX_TIER; i++) {		\
		ret = sscanf(buf + pos,			\
			"%u%n", v + i, &len);		\
		if (!ret)				\
			return -EINVAL;			\
		v[i] /= mul;				\
		if (v[i] < min || v[i] > max)		\
			return -EINVAL;			\
		pos += len + 1;				\
	}						\
	if (pos != count)				\
		return -EINVAL;				\
	for (i = 0; i < MAX_TIER; i++)			\
		ts_global.obj[i] = v[i];		\
	return pos;					\
}							\
define_one_global_rw(fn);

#define KNOB(fn, obj, min, max, mul, rb)		\
static ssize_t show_##fn				\
(struct kobject *k, struct attribute *a, char *buf)	\
{ return sprintf(buf, "%u\n", (ts_global.obj) * mul ); }\
static ssize_t store_##fn				\
(struct kobject *k, struct attribute *a,		\
const char *buf, size_t count)				\
{							\
	int v, ret;					\
	ret = sscanf(buf, "%i", &v);			\
	v = v / mul;					\
	if (ret != 1 || v < min || v > max)		\
		return -EINVAL;				\
	mutex_lock(&ts_global.global_mutex);		\
	ts_global.obj = v;				\
	if (rb) rebuild_all_privs();			\
	mutex_unlock(&ts_global.global_mutex);		\
	return count;					\
}							\
define_one_global_rw(fn);

TKNOB(delay_up_ms, delay_up, 1, 100, jiffies_to_msecs(1));
TKNOB(delay_down_ms, delay_down, 1, 100, jiffies_to_msecs(1));
KNOB(tier_count, tier_count, 2, MAX_TIER, 1, 1);
KNOB(extra_mhz, extra_mhz, 75, 500, 1, 0);
// Need at least 2 jiffies to keep get_idle_ktime well-behaved.
KNOB(sample_time_ms, sample_time, 2, MS(100), jiffies_to_msecs(1), 0);
KNOB(max_sample_ms, max_sample, 10, 1000, 1, 1);
KNOB(active_sample_ms, active_sample, 1, 1000, 1, 1);
KNOB(interaction_ms, int_timeout, 0, MS(500), jiffies_to_msecs(1), 0);
static struct attribute *ts_attrs[] = {
	&delay_up_ms.attr,
	&delay_down_ms.attr,
	&tier_count.attr,
	&extra_mhz.attr,
	&sample_time_ms.attr,
	&max_sample_ms.attr,
	&active_sample_ms.attr,
	&interaction_ms.attr,
	NULL
};
static struct attribute_group ts_attr_grp = {
	.attrs = ts_attrs,
	.name = "tierservative"
};

/* struct usage_sample:
 * Holds the KHz usage & duration for a sample.  Usage is stored as KHz * nsec
 * to allow easier accumulation of time-weighted averages: this is robust to
 * both uneven sample sizes and mid-sample frequency changes.
 */
struct usage_sample {
	s64			usage;
	s64			nsec;
};

/* calc_sample_usage:
 * Calculates average KHz of a usage_sample, shifting by a few bits to avoid
 * overflowing the (32-bit) do_div divisor.
 */
static unsigned long calc_sample_usage(struct usage_sample *usage)
{
	s64 sum, div;
	sum = usage->usage >> 8;
	div = usage->nsec >> 8;
	if (unlikely(!div))
		return 0;
	div++;
	do_div(sum, div);
	return sum;
}

/* sample_reduce:
 * Scales a sample to a given length in nsec.
 */
static void sample_reduce(struct usage_sample *usage, s64 len)
{
	s64 num, den;
	num = usage->usage;
	den = usage->nsec >> 8;
	if (unlikely(!den || len < NSEC_PER_MSEC)) {
		usage->usage = usage->nsec = 0;
		return;
	}
	den++;
	do_div(num, den);

	den = len >> 8;
	den++;
	usage->usage = num * (len >> 8);
	usage->nsec = len;
}

/* struct usage_sample_list:
 * A usage_sample with a list_head.  I'm sure there's a more efficient way to
 * do this.
 */
struct usage_sample_list {
	struct list_head	list;
	struct usage_sample	sample;
};
/* struct usage_average:
 * A time-weighted usage average.  The sample list is automatically maintained
 * by the insertion function.
 */
struct usage_average {
	u64			max_sample;
	struct usage_sample	avg;
	unsigned long		avg_khz;
	struct list_head	samples;
};

/* alloc_usage_average:
 * Allocate and initialize a usage_average.
 */
static struct usage_average *alloc_usage_average(void)
{
	struct usage_average *ua;
	ua = kzalloc(sizeof(struct usage_average), GFP_KERNEL);

	if (!ua)
		return NULL;

	INIT_LIST_HEAD(&ua->samples);

	return ua;
}

/* free_usage_average:
 * Free a usage_average and all its samples.
 */
static void free_usage_average(struct usage_average *ua)
{
	struct usage_sample_list *samp, *tmp;
	list_for_each_entry_safe(samp, tmp, &ua->samples, list) {
		list_del(&samp->list);
		kmem_cache_free(ts_global.sample_cache, samp);
	}
	kfree(ua);
}

/* __usage_average_insert:
 * Directly inserts a usage_sample_list.  Does not execute usage_average_trim
 * or recalculate average usage.
 */
static void __usage_average_insert(struct usage_average *ua,
				   struct usage_sample_list *usage)
{
	ua->avg.usage += usage->sample.usage;
	ua->avg.nsec += usage->sample.nsec;
	list_add_tail(&usage->list, &ua->samples);
}

/* usage_average_trim:
 * Reduces the samples to at most max_samples nsec, freeing as needed.
 */
static void usage_average_trim(struct usage_average *ua)
{
	struct usage_sample_list *samp, *tmp;
	s64 rem;

	rem = ua->avg.nsec - ua->max_sample;
	if (rem < 0)
		return;

	list_for_each_entry_safe(samp, tmp, &ua->samples, list) {
		ua->avg.usage -= samp->sample.usage;
		ua->avg.nsec -= samp->sample.nsec;
		rem -= samp->sample.nsec;

		if (rem <= 0) {
			sample_reduce(&samp->sample, -rem);
			ua->avg.usage += samp->sample.usage;
			ua->avg.nsec += samp->sample.nsec;
			break;
		} else {
			list_del(&samp->list);
			kmem_cache_free(ts_global.sample_cache, samp);
		}
	}
}

/* usage_average_insert:
 * Copies a sample into the average, removes stale samples, and recalculates
 * the average usage.
 */
static void usage_average_insert(struct usage_average *ua,
				 struct usage_sample *usage)
{
	struct usage_sample_list *samp;

	samp = kmem_cache_alloc(ts_global.sample_cache, GFP_KERNEL);
	if (!samp)
		return;

	samp->sample = *usage;
	__usage_average_insert(ua, samp);

	usage_average_trim(ua);
	ua->avg_khz = calc_sample_usage(&ua->avg);
}

/* ts_cpu_priv:
 * Per-CPU state information.
 */
struct ts_cpu_priv {
	struct mutex		cpu_mutex;
	bool			enabled;

	struct cpufreq_policy	*policy;
	struct delayed_work	work;

	ktime_t			prev_wall;
	ktime_t			prev_idle;
	struct usage_sample	current_usage;
	unsigned long		current_usage_khz;

	struct usage_average	*usage;
	struct usage_average	*tiers[MAX_TIER];
	unsigned int		active_tier;
	unsigned int		stats_tier;
	unsigned int		max_tier;

	unsigned int		interaction;
	unsigned long		up_timeout;
	unsigned long		down_timeout;
	unsigned long		interaction_timeout;
};

static DEFINE_PER_CPU(struct ts_cpu_priv, ts_cpu);

/* rebuild_priv:
 * Rebuilds ts, including creating/destroying usage_averages, adjusting tier
 * parameters, and creating ->usage if necessary.  Locking is performed by the
 * caller.
 */
static void rebuild_priv(struct ts_cpu_priv *ts)
{
	int i;
	struct usage_average *ua;

	// Allocate and initialize usage average
	if (!ts->usage) {
		ts->usage = alloc_usage_average();
		if (!ts->usage)
			return;
	}
	ts->usage->max_sample = ts_global.active_sample;
	ts->usage->max_sample *= NSEC_PER_MSEC;
	usage_average_trim(ts->usage);
	ts->usage->avg_khz = calc_sample_usage(&ts->usage->avg);

	// Add/remove new tiers as needed
	for (i = 0; i < MAX_TIER; i++) {
		if (i < ts_global.tier_count && !ts->tiers[i]) {
			ts->tiers[i] = alloc_usage_average();
			if (!ts->tiers[i]) {
				ts->max_tier = i - 1;
				goto alloc_fail;
			}
		} else if (i >= ts_global.tier_count && ts->tiers[i]) {
			free_usage_average(ts->tiers[i]);
			ts->tiers[i] = NULL;
		}
	}
	ts->max_tier = ts_global.tier_count - 1;
	if (ts->active_tier > ts->max_tier)
		ts->active_tier = ts->max_tier;
	if (ts->stats_tier > ts->max_tier)
		ts->stats_tier = ts->max_tier;

alloc_fail:
	// Update max_sample and trim tiers
	for (i = 0; i <= ts->max_tier; i++) {
		ua = ts->tiers[i];
		ua->max_sample = ts_global.max_sample;
		ua->max_sample *= NSEC_PER_MSEC;
		usage_average_trim(ua);
		ua->avg_khz = calc_sample_usage(&ua->avg);
	}
}

/* destroy_priv:
 * Frees all allocated data used by ts.
 */
static void destroy_priv(struct ts_cpu_priv *ts)
{
	int i;

	for (i = 0; i <= ts->max_tier; i++) {
		if (!ts->tiers[i])
			break;

		free_usage_average(ts->tiers[i]);
		ts->tiers[i] = NULL;
	}
	ts->max_tier = 0;

	if (ts->usage)
		free_usage_average(ts->usage);
	ts->usage = NULL;
}

/* rebuild_all_privs:
 * Calls rebuild_all_tiers on each priv.  Locking is performed by
 * the caller.
 */
static void rebuild_all_privs(void)
{
	int i;
	for_each_possible_cpu(i) {
		struct ts_cpu_priv *ts = &per_cpu(ts_cpu, i);
		mutex_lock(&ts->cpu_mutex);
		rebuild_priv(ts);
		mutex_unlock(&ts->cpu_mutex);
	}
}

/* insert_current_sample:
 * Insert the current usage average into the appropriate tiers.  Ideally, we
 * want to maintain a smooth upward curve across the tiers.  Occasionally,
 * we'll end up with an unordered table, but it's not generally an issue.
 */
static void insert_current_sample(struct ts_cpu_priv *ts)
{
	int i;
	struct usage_sample samp = ts->current_usage;

	if (ts->current_usage_khz > ts->tiers[ts->stats_tier]->avg_khz) {
		if (ts->stats_tier < ts->max_tier)
			ts->stats_tier++;
	} else {
		if (ts->stats_tier)
			ts->stats_tier--;
	}

	for (i = ts->stats_tier; i >= 0; i--) {
		usage_average_insert(ts->tiers[i], &samp);
		samp.usage >>= 1;
		samp.nsec >>= 1;
	}
}

/* get_usage:
 * Accumulates CPU usage samples, and returns the running average in KHz.
 *
 * NB: get_usage must be called before every frequency change to keep usage
 *     samples accurate.
 */
static unsigned long get_usage(struct ts_cpu_priv *ts)
{
	struct usage_sample samp;
	ktime_t idle, wall;

	wall = ktime_get();
	idle = get_idle_ktime(ts->policy->cpu);

	samp.nsec = ktime_to_ns(ktime_sub(wall, ts->prev_wall));
	ts->prev_wall = wall;

	samp.usage = samp.nsec - ktime_to_ns(ktime_sub(idle, ts->prev_idle));
	ts->prev_idle = idle;

	if (samp.usage <= 0)
		samp.usage = 0;
	else
		samp.usage *= ts->policy->cur;

	usage_average_insert(ts->usage, &samp);

	// Save the current sample for insert_current_sample
	ts->current_usage = samp;
	ts->current_usage_khz = calc_sample_usage(&samp);

	return ts->usage->avg_khz;
}

/* find_tier_up/_down:
 * Applies logic related to switching tiers up/down, and selects an appropriate
 * tier to switch to.
 */
#define usage_suitable(tier, usage) \
	(ts->tiers[tier]->avg_khz >= usage)
static int find_tier_up(struct ts_cpu_priv *ts, unsigned long usage)
{
	int i, max;

	if (usage_suitable(ts->active_tier, usage))
		return -1;

	max = ts->max_tier;
	if (ts->active_tier == max)
		return ts->active_tier;

	for (i = ts->active_tier + 1; i < max; i++) {
		if (usage_suitable(i, usage))
			break;
	}

	return i;
}

static int find_tier_down(struct ts_cpu_priv *ts, unsigned long usage)
{
	int i;

	if (ts->active_tier && !usage_suitable(ts->active_tier, usage))
		return -1;

	if (ts->active_tier == 0) {
		if (ts->interaction == 1 &&
		    time_after(jiffies, ts->interaction_timeout))
			ts->interaction = 0;
		return ts->active_tier;
	}

	/* We can only ramp a limited distance.  If we've been at elevated
	 * frequency for a while, all the lower tiers' averages have been
	 * obliterated, and we'll ramp all the way down.  Ideally, we should
	 * rebuild the frequency curve on the way back down.
	 */
	i = ts->active_tier - 1;
	if (!usage_suitable(i, usage))
		return -1;

	return i;
}

/* ts_sample_worker:
 * The core ts routine: samples usage, handles ramping, updates averages.
 */
static void ts_sample_worker(struct work_struct *work)
{
	struct ts_cpu_priv *ts =
		container_of(work, struct ts_cpu_priv, work.work);
	int next = -1;
	unsigned long usage_khz;
	unsigned long boost_khz;

	mutex_lock(&ts->cpu_mutex);

	if (!ts->enabled)
		goto out_nosched;

	usage_khz = get_usage(ts);

#ifndef MODULE
	cpufreq_notify_utilization(ts->policy,
		usage_khz / ts->policy->user_policy.max);
#endif

	if (time_after_eq(jiffies, ts->up_timeout)) {
		next = find_tier_up(ts, usage_khz);
		if (next >= 0)
			goto set_tier;
	}
	if (time_after_eq(jiffies, ts->down_timeout)) {
		next = find_tier_down(ts, usage_khz);
		if (next >= 0)
			goto set_tier;
	}

	goto set_freq;

set_tier:
	// Kick off the interaction timeout when we switch to tier 0
	if (ts->active_tier && !next && ts->interaction == 1)
		ts->interaction_timeout = jiffies + ts_global.int_timeout;

	ts->active_tier = next;
	ts->up_timeout = jiffies + ts_global.delay_up[ts->active_tier];
	ts->down_timeout = jiffies + ts_global.delay_down[ts->active_tier];

set_freq:
	insert_current_sample(ts);

	boost_khz = (ts->current_usage_khz * 1000) / ts->policy->cur *
		ts_global.extra_mhz;
	if (usage_khz < ts->current_usage_khz)
		usage_khz = ts->current_usage_khz;
	usage_khz += boost_khz;
	if ((ts->active_tier || ts->interaction) &&
	    usage_khz < ts->tiers[ts->active_tier]->avg_khz)
		usage_khz = ts->tiers[ts->active_tier]->avg_khz;
	__cpufreq_driver_target(ts->policy, usage_khz, CPUFREQ_RELATION_L);

	schedule_delayed_work_on(ts->policy->cpu, &ts->work,
		ts_global.sample_time);

out_nosched:
	mutex_unlock(&ts->cpu_mutex);
}

/* hotplug_handler:
 * During hot-remove, enables interaction on all online instances to rapidly
 * compensate for suddenly increasing demand.
 */
static int hotplug_handler(struct notifier_block *nb,
			   unsigned long val, void *data)
{
	int i;

	if (val != CPU_DOWN_PREPARE)
		return NOTIFY_OK;
	if (!ts_global.enabled)
		return NOTIFY_OK;

	for_each_online_cpu(i) {
		struct ts_cpu_priv *ts = &per_cpu(ts_cpu, i);
		if (!ts->enabled)
			continue;

		mutex_lock(&ts->cpu_mutex);
		if (ts->interaction != 2) {
			ts->interaction = 1;
			if (!ts->active_tier)
				ts->interaction_timeout =
					jiffies + ts_global.int_timeout;
		}
		mutex_unlock(&ts->cpu_mutex);
	}

	return NOTIFY_OK;
}

/* ts_(un)register_notifiers:
 * (Un)register the hotcpu notifier used for hotplug compensation.
 */
static void ts_register_notifiers(void)
{
	int ret;
	if (ts_global.notifiers_registered)
		return;

	ts_global.hotplug_notifier.notifier_call = hotplug_handler;
	ret = register_hotcpu_notifier(&ts_global.hotplug_notifier);
	if (ret) {
		printk(KERN_WARNING "%s: register notifier failed (%i)!\n",
			__func__, ret);
		return;
	}
	ts_global.notifiers_registered = 1;
}

static void ts_unregister_notifiers(void)
{
	if (!ts_global.notifiers_registered)
		return;

	unregister_hotcpu_notifier(&ts_global.hotplug_notifier);
	ts_global.notifiers_registered = 0;
}

// cpufreq boilerplate from here down
static int ts_governor(struct cpufreq_policy *policy, unsigned int event)
{
	struct ts_cpu_priv *ts = &per_cpu(ts_cpu, policy->cpu);
	int ret = 0;

	if (likely(event & CPUFREQ_GOV_NOINTERACT)) {
		mutex_lock(&ts->cpu_mutex);
		if (event == CPUFREQ_GOV_INTERACT) {
			ts->interaction = 2;
		} else {
			ts->interaction = 1;
			if (!ts->active_tier)
				ts->interaction_timeout =
					jiffies + ts_global.int_timeout;
		}
		mutex_unlock(&ts->cpu_mutex);
		return 0;
	}

	mutex_lock(&ts_global.global_mutex);

	switch (event) {
	case CPUFREQ_GOV_START:
		if (!cpu_online(policy->cpu) || !policy->cur) {
			ret = -EINVAL;
			goto out;
		}

		if (!ts_global.enabled++) {
			ts_register_notifiers();
			ret = sysfs_create_group(cpufreq_global_kobject,
				&ts_attr_grp);
			if (ret)
				goto out;
		}

		mutex_lock(&ts->cpu_mutex);

		ts->policy = policy;
		ts->interaction = 0;
		ts->active_tier = 0;
		ts->up_timeout = jiffies + ts_global.delay_up[0];
		ts->down_timeout = jiffies + ts_global.delay_down[0];
		ts->interaction_timeout = jiffies;

		ts->prev_wall = ktime_get();
		ts->prev_idle = get_idle_ktime(policy->cpu);

		rebuild_priv(ts);
		if (!ts->usage || !ts->tiers[0]) {
			ret = -ENOMEM;
			goto out;
		}

		ts->enabled = 1;
		INIT_DELAYED_WORK(&ts->work, ts_sample_worker);
		schedule_delayed_work_on(policy->cpu, &ts->work,
			ts_global.sample_time);

		mutex_unlock(&ts->cpu_mutex);
		break;

	case CPUFREQ_GOV_STOP:
		ts->enabled = 0;
		cancel_delayed_work_sync(&ts->work);

		mutex_lock(&ts->cpu_mutex);
		if (ts->usage && ts->tiers[ts->active_tier]) {
			get_usage(ts);
			insert_current_sample(ts);
		}
		mutex_unlock(&ts->cpu_mutex);

		if (!--ts_global.enabled) {
			ts_unregister_notifiers();
			sysfs_remove_group(cpufreq_global_kobject,
				&ts_attr_grp);
			kmem_cache_shrink(ts_global.sample_cache);
		}

		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&ts->cpu_mutex);

		if (ts->usage)
			get_usage(ts);

		if (policy->max < ts->policy->cur)
			__cpufreq_driver_target(ts->policy,
				policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > ts->policy->cur)
			__cpufreq_driver_target(ts->policy,
				policy->min, CPUFREQ_RELATION_L);

		mutex_unlock(&ts->cpu_mutex);

		break;
	}

out:
	mutex_unlock(&ts_global.global_mutex);

	return ret;
}

static struct cpufreq_governor ts_gov_info = {
	.name		= "tierservative",
	.governor	= ts_governor,
	.max_transition_latency = 10000000,
	.owner		= THIS_MODULE,
	.flags		= BIT(GOVFLAGS_ALLCPUS),
};

static int __init ts_init(void)
{
	int i;
	mutex_init(&ts_global.global_mutex);
	for_each_possible_cpu(i) {
		struct ts_cpu_priv *ts;
		ts = &per_cpu(ts_cpu, i);
		mutex_init(&ts->cpu_mutex);
	}
	ts_global.sample_cache = KMEM_CACHE(usage_sample_list, 0);
	if (!ts_global.sample_cache)
		return -ENOMEM;

	return cpufreq_register_governor(&ts_gov_info);
}

static void __exit ts_exit(void)
{
	int i;
	for_each_possible_cpu(i) {
		struct ts_cpu_priv *ts;
		ts = &per_cpu(ts_cpu, i);
		destroy_priv(ts);
		mutex_destroy(&ts->cpu_mutex);
	}
	kmem_cache_destroy(ts_global.sample_cache);
	cpufreq_unregister_governor(&ts_gov_info);
}

module_init(ts_init);
module_exit(ts_exit);

MODULE_AUTHOR("Ryan Pennucci <decimalman@gmail.com>");
MODULE_LICENSE("GPL");
