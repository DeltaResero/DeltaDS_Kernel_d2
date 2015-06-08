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
 * TODO: consider moving to a timer rather than workqueue?
 * TODO: add support for multi-CPU cpufreq policies (no longer possible?)
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
#include <linux/wmavg.h>

// Fixed maximum tier count
#define MAX_TIER	(16)
// msecs_to_jiffies apparently doesn't work at compile time
#define MS(x) DIV_ROUND_UP(x*HZ, 1000)

/* struct ts_global_priv:
 * Shared state & configuration.
 */
struct ts_global_priv {
	struct mutex		global_mutex;
	unsigned int		enabled;
	struct workqueue_struct	*wq;
	struct notifier_block	idle_nb;

	unsigned int		tier_count;
	unsigned int		extra_mhz;
	unsigned int		sample_time;
	unsigned int		max_sample;
	unsigned int		active_sample;
	unsigned int		saved_timeout;
};

static struct ts_global_priv ts_global __read_mostly = {
	.tier_count = 8,
	.extra_mhz = 75,
	.sample_time = MS(16),
	.max_sample = 333,
	.active_sample = 100,
	.saved_timeout = MS(83),
};

static void rebuild_all_privs(void);

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
	v = DIV_ROUND_UP(v, mul);			\
	if (ret != 1) return -EINVAL;			\
	if (v < min) v = min;				\
	if (v > max) v = max;				\
	mutex_lock(&ts_global.global_mutex);		\
	ts_global.obj = v;				\
	if (rb) rebuild_all_privs();			\
	mutex_unlock(&ts_global.global_mutex);		\
	return count;					\
}							\
define_one_global_rw(fn);

KNOB(tier_count, tier_count, 2, MAX_TIER, 1, 1);
KNOB(extra_mhz, extra_mhz, 25, 500, 1, 0);
KNOB(sample_time_ms, sample_time, 1, MS(100), jiffies_to_msecs(1), 0);
KNOB(max_sample_ms, max_sample, 10, 1000, 1, 1);
KNOB(active_sample_ms, active_sample, 1, 1000, 1, 1);
KNOB(saved_expire_ms, saved_timeout, 0, MS(500), jiffies_to_msecs(1), 0);
static struct attribute *ts_attrs[] = {
	&tier_count.attr,
	&extra_mhz.attr,
	&sample_time_ms.attr,
	&max_sample_ms.attr,
	&active_sample_ms.attr,
	&saved_expire_ms.attr,
	NULL
};
static struct attribute_group ts_attr_grp = {
	.attrs = ts_attrs,
	.name = "tierservative"
};

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
	ktime_t			idle_enter;
	struct wmavg_sample	current_usage;
	unsigned long		current_usage_khz;

	struct wmavg_average	*usage;
	struct wmavg_average	*tiers[MAX_TIER];
	unsigned int		active_tier;
	unsigned int		saved_tier;
	unsigned int		max_tier;

	unsigned long		saved_timeout;
};

static DEFINE_PER_CPU(struct ts_cpu_priv, ts_cpu);

/* rebuild_priv:
 * Rebuilds ts, including creating/destroying wmavg_averages, adjusting tier
 * parameters, and creating ->usage if necessary.  Locking is performed by the
 * caller.
 */
static void rebuild_priv(struct ts_cpu_priv *ts)
{
	int i;

	// Allocate and initialize usage average
	if (!ts->usage) {
		ts->usage = wmavg_alloc();
		if (!ts->usage)
			return;
	}
	wmavg_set_max_weight(ts->usage,
		ts_global.active_sample * NSEC_PER_MSEC >> 8);
	wmavg_set_min_weight(ts->usage, NSEC_PER_MSEC >> 8);

	// Add/remove new tiers as needed
	for (i = 0; i < MAX_TIER; i++) {
		if (i < ts_global.tier_count && !ts->tiers[i]) {
			ts->tiers[i] = wmavg_alloc();
			if (!ts->tiers[i]) {
				ts->max_tier = i - 1;
				goto alloc_fail;
			}
			// Pre-populate a small sample into new tiers
			wmavg_insert(ts->tiers[i],
				ts->policy->max * i / ts_global.tier_count,
				50000);
		} else if (i >= ts_global.tier_count && ts->tiers[i]) {
			wmavg_free(ts->tiers[i]);
			ts->tiers[i] = NULL;
		}
	}
	ts->max_tier = ts_global.tier_count - 1;
	if (ts->active_tier > ts->max_tier)
		ts->active_tier = ts->max_tier;
	if (ts->saved_tier > ts->max_tier)
		ts->saved_tier = ts->max_tier;

alloc_fail:
	// Update max_sample and trim tiers
	for (i = 0; i <= ts->max_tier; i++) {
		wmavg_set_max_weight(ts->tiers[i],
			ts_global.active_sample * NSEC_PER_MSEC >> 8);
		wmavg_set_min_weight(ts->usage, NSEC_PER_MSEC >> 8);
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

		wmavg_free(ts->tiers[i]);
		ts->tiers[i] = NULL;
	}
	ts->max_tier = 0;

	if (ts->usage)
		wmavg_free(ts->usage);
	ts->usage = NULL;
}

/* rebuild_all_privs:
 * Calls rebuild_priv on each priv.  Locking is performed by the caller.
 */
static void rebuild_all_privs(void)
{
	int i;
	for_each_possible_cpu(i) {
		struct ts_cpu_priv *ts = &per_cpu(ts_cpu, i);
		mutex_lock(&ts->cpu_mutex);
		if (ts->enabled)
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
	struct wmavg_sample samp = ts->current_usage;

	if (!ts->active_tier) {
		wmavg_insert_sample(ts->tiers[0], &samp);
		return;
	}

	if (ts->current_usage_khz < ts->policy->min / 4)
		return;

	i = ts->active_tier;
	if (ts->current_usage_khz < ts->policy->min &&
	    i > DIV_ROUND_UP(ts->max_tier + 1, 4))
		i = DIV_ROUND_UP(ts->max_tier + 1, 4);

	for (; i >= 0; i--) {
		wmavg_insert_sample(ts->tiers[i], &samp);
		samp.value >>= 1;
		samp.weight >>= 1;
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
	struct wmavg_sample samp;
	ktime_t idle, wall;
	s64 delta;

	wall = ktime_get();
	idle = get_idle_ktime(ts->policy->cpu);

	delta = ktime_to_ns(ktime_sub(wall, ts->prev_wall));
	ts->prev_wall = wall;
	if (delta < 0)
		delta = 0;
	samp.weight = delta;

	delta = ktime_to_ns(ktime_sub(idle, ts->prev_idle));
	ts->prev_idle = idle;
	if (delta < 0)
		delta = 0;
	samp.value = delta;

	samp.value >>= 8;
	samp.weight >>= 8;
	if (likely(samp.value < samp.weight)) {
		samp.value = samp.weight - samp.value;
		samp.value *= ts->policy->cur;
	} else {
		samp.value = 0;
	}

	// Save the current sample for insert_current_sample
	ts->current_usage.value += samp.value;
	ts->current_usage.weight += samp.weight;
	ts->current_usage_khz = wmavg_sample_calc(&samp);

	/* Allow no-demand conditions to bypass the long-term average to reduce
	 * frequency more quickly.  Also, don't pollute the average with low
	 * samples during idle.
	 */
	if (ts->active_tier &&
	    ts->current_usage_khz > ts->policy->min * 2 / 3) {
		wmavg_insert_sample(ts->usage, &samp);
		return ts->usage->avg;
	}

	return ts->current_usage_khz;
}

/* find_tier_up/_down:
 * Applies logic related to switching tiers up/down, and selects an appropriate
 * tier to switch to.
 */
#define usage_suitable(tier, usage) \
	(ts->tiers[tier]->avg >= usage)
static int find_tier_up(struct ts_cpu_priv *ts, unsigned long usage)
{
	int i;

	if (usage_suitable(ts->active_tier, usage) ||
	    ts->active_tier == ts->max_tier)
		return -1;

	i = max(ts->active_tier + 1, ts->saved_tier);
	for (; i < ts->max_tier; i++) {
		if (usage_suitable(i, usage))
			break;
	}

	if (i > ts->saved_tier) {
		ts->saved_tier = i;
		ts->saved_timeout = jiffies + ts_global.saved_timeout;
	}

	return i;
}

static int find_tier_down(struct ts_cpu_priv *ts, unsigned long usage)
{
	int i;

	if (!ts->active_tier || !usage_suitable(ts->active_tier, usage))
		return -1;

	if (ts->active_tier == 1) {
		if (!usage_suitable(0, usage))
			return -1;
		return 0;
	}

	for (i = ts->active_tier - 2; i >= 0; i--) {
		if (!usage_suitable(i, usage))
			return (i + 2 == ts->active_tier) ? -1 : i + 2;
	}

	return 0;
}

/* ts_sample_worker:
 * The core ts routine: samples usage, handles ramping, updates averages.
 */
static void ts_sample_worker(struct work_struct *work)
{
	struct ts_cpu_priv *ts =
		container_of(work, struct ts_cpu_priv, work.work);
	int next = -1;
	unsigned long usage_khz, boost_tmp;

	mutex_lock(&ts->cpu_mutex);

	if (unlikely(!ts->enabled))
		goto out_nosched;

	usage_khz = get_usage(ts);

#ifndef MODULE
	cpufreq_notify_utilization(ts->policy,
		usage_khz / ts->policy->user_policy.max);
#endif

	if (ts->saved_tier > 0 &&
	    time_after_eq(jiffies, ts->saved_timeout) &&
	    usage_khz < ts->tiers[ts->saved_tier]->avg) {
		ts->saved_tier--;
		ts->saved_timeout = jiffies + ts_global.saved_timeout;
	}
	next = find_tier_up(ts, usage_khz);
	if (next == -1)
		next = find_tier_down(ts, usage_khz);
	if (next != -1)
		ts->active_tier = next;

	insert_current_sample(ts);

	/* Boost only according to momentary usage to reduce aggressiveness
	 * when coming out of idle.  The long-term average will remain
	 * significantly elevated during idle.
	 */
	boost_tmp = ts->current_usage_khz * 1000 / ts->policy->cur;
	boost_tmp = boost_tmp * boost_tmp / 1000;
	boost_tmp = ts->current_usage_khz + boost_tmp * ts_global.extra_mhz
		* min(6, (2 + ilog2(nr_running())));
	if (usage_khz < boost_tmp)
		usage_khz = boost_tmp;
	if (ts->active_tier && usage_khz < ts->tiers[ts->active_tier]->avg)
		usage_khz = ts->tiers[ts->active_tier]->avg;

	__cpufreq_driver_target(ts->policy, usage_khz, CPUFREQ_RELATION_L);

	queue_delayed_work_on(ts->policy->cpu, ts_global.wq, &ts->work,
		ts_global.sample_time);

	ts->current_usage.value = ts->current_usage.weight = 0;

out_nosched:
	mutex_unlock(&ts->cpu_mutex);
}

/* ts_idle_notify:
 * Handles idle-time logic.
 *
 * We use this callback to filter out long periods of idleness.  This results
 * in more responsive and more "correct" behavior.  Including the idle period
 * in demand sampling is incorrect in the sense that it doesn't predict future
 * demand well, except in the case of very light load at resume (e.g. a one-off
 * interrupt).
 *
 * We don't care what frequency the core is running at during idle.  It's
 * reasonable to expect that any task that was running prior to idle will
 * resume at its previous demand, and it doesn't make sense to ramp down due to
 * disk/net/whatever latency.
 */
static int ts_idle_notify(struct notifier_block *nb, unsigned long val,
			  void *data)
{
	struct ts_cpu_priv *ts = &per_cpu(ts_cpu, smp_processor_id());
	ktime_t now;
	s64 idle_ns;

	// Not strictly needed (everything is cpu-bound), but lock anyway.
	if (unlikely(!mutex_trylock(&ts->cpu_mutex)))
		return 0;
	// By default, all cores use the same governor in dkp
	if (unlikely(!ts->enabled))
		goto out;

	now = ktime_get();

	switch (val) {
	case IDLE_START:
		ts->idle_enter = now;
		break;
	case IDLE_END:
		idle_ns = ktime_to_ns(ktime_sub(now, ts->idle_enter));
		idle_ns -= jiffies_to_usecs(1) * 500;
		if (idle_ns < 0)
			break;

		ts->prev_wall = ktime_add_ns(ts->prev_wall, idle_ns);
		ts->prev_idle = ktime_add_ns(ts->prev_idle, idle_ns);

		queue_delayed_work_on(ts->policy->cpu, ts_global.wq, &ts->work,
				      1);
		break;
	}

out:
	mutex_unlock(&ts->cpu_mutex);
	return 0;
}

static int ts_governor(struct cpufreq_policy *policy, unsigned int event)
{
	struct ts_cpu_priv *ts = &per_cpu(ts_cpu, policy->cpu);
	int ret = 0;

#ifdef CONFIG_INTERACTION_HINTS
	if (likely(event & CPUFREQ_GOV_NOINTERACT))
		return 0;
#endif

	mutex_lock(&ts_global.global_mutex);

	switch (event) {
	case CPUFREQ_GOV_START:
		if (!cpu_online(policy->cpu) || !policy->cur) {
			ret = -EINVAL;
			goto out;
		}

		if (!ts_global.enabled++) {
			ret = sysfs_create_group(cpufreq_global_kobject,
				&ts_attr_grp);
			if (ret)
				goto out;
		}

		mutex_lock(&ts->cpu_mutex);

		ts->policy = policy;
		ts->saved_tier =
		ts->active_tier = 0;

		ts->prev_wall = ktime_get();
		ts->prev_idle = get_idle_ktime(policy->cpu);

		rebuild_priv(ts);
		if (!ts->usage || !ts->tiers[0]) {
			ret = -ENOMEM;
			goto out;
		}

		ts->enabled = 1;
		INIT_DELAYED_WORK_DEFERRABLE(&ts->work, ts_sample_worker);
		queue_delayed_work_on(ts->policy->cpu, ts_global.wq, &ts->work,
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
			sysfs_remove_group(cpufreq_global_kobject,
				&ts_attr_grp);
			idle_notifier_unregister(&ts_global.idle_nb);
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
	.name			= "tierservative",
	.governor		= ts_governor,
	.max_transition_latency = 10000000,
	.owner			= THIS_MODULE,
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
	ts_global.idle_nb.notifier_call = ts_idle_notify;
	ts_global.wq = alloc_workqueue("tierservative", WQ_HIGHPRI, NR_CPUS);
	if (!ts_global.wq)
		return -ENOMEM;
	return cpufreq_register_governor(&ts_gov_info);
}

static void __exit ts_exit(void)
{
	int i;
	destroy_workqueue(ts_global.wq);
	for_each_possible_cpu(i) {
		struct ts_cpu_priv *ts;
		ts = &per_cpu(ts_cpu, i);
		destroy_priv(ts);
		mutex_destroy(&ts->cpu_mutex);
	}
	cpufreq_unregister_governor(&ts_gov_info);
}

module_init(ts_init);
module_exit(ts_exit);

MODULE_AUTHOR("Ryan Pennucci <decimalman@gmail.com>");
MODULE_LICENSE("GPL");
