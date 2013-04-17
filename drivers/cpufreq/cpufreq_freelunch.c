/*
 *  drivers/cpufreq/cpufreq_freelunch.c
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *            (C)  2009 Alexander Clouter <alex@digriz.org.uk>
 *            (C)  2013 Ryan Pennucci <decimalman@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/slab.h>

//#define DEFER_STATS

enum interaction_flags {
	IFLAG_PRESSED = 1,
	IFLAG_RUNNING = 2,
	IFLAG_ENABLED = 1 | 2,
};
#define IGF(x) (this_dbs_info->is_interactive & IFLAG_##x)
#define ISF(x) this_dbs_info->is_interactive |= IFLAG_##x
#define IUF(x) this_dbs_info->is_interactive &= ~IFLAG_##x

// {{{1 tuner crap
static void do_dbs_timer(struct work_struct *work);

struct cpu_dbs_info_s {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_wall;
	cputime64_t prev_cpu_nice;
	struct cpufreq_policy *cur_policy;
	struct delayed_work work;
	unsigned int requested_freq;
	int cpu;
	unsigned int enable:1;
	/*
	 * percpu mutex that serializes governor limit change with
	 * do_dbs_timer invocation. We do not want do_dbs_timer to run
	 * when user is changing the governor or limits.
	 */
	struct mutex timer_mutex;

	int hotplug_cycle;

	/* Interaction hack stuff */
	int is_interactive;
	unsigned int defer_cycles;
#ifdef DEFER_STATS
	unsigned int deferred_return;
#endif
	unsigned int max_freq;
};
static DEFINE_PER_CPU(struct cpu_dbs_info_s, cs_cpu_dbs_info);

static unsigned int dbs_enable;	/* number of CPUs using this policy */

/* Hotplug stuff */
static struct work_struct cpu_up_work;
static struct work_struct cpu_down_work;

/*
 * dbs_mutex protects dbs_enable in governor start/stop.
 */
static DEFINE_MUTEX(dbs_mutex);

static struct dbs_tuners {
	unsigned int sampling_rate;
	unsigned int ignore_nice;

	unsigned int hotplug_up_cycles;
	unsigned int hotplug_down_cycles;
	unsigned int hotplug_up_load;
	unsigned int hotplug_up_usage;
	unsigned int hotplug_down_usage;

	unsigned int overestimate_khz;
	unsigned int hispeed_thresh;
	unsigned int hispeed_decrease;

	unsigned int interaction_sampling_rate;
	unsigned int interaction_overestimate_khz;
	unsigned int interaction_return_usage;
	unsigned int interaction_return_cycles;

	unsigned int interaction_hispeed;
} dbs_tuners_ins = {
	.sampling_rate = 35000, /* 2 vsyncs */
	.ignore_nice = 0,
	.hotplug_up_cycles = 3,
	.hotplug_down_cycles = 5,
	.hotplug_up_load = 3,
	.hotplug_up_usage = 600000,
	.hotplug_down_usage = 225000,
	.overestimate_khz = 35000,
	.hispeed_thresh = 25000,
	.hispeed_decrease = 75000,
	.interaction_sampling_rate = 10000,
	.interaction_overestimate_khz = 175000,
	.interaction_return_usage = 75000, /* crazy low, but XDA sucks otherwise */
	.interaction_return_cycles = 4, /* 3 vsyncs */
	.interaction_hispeed = 1188000,
};
// }}}
// {{{2 support function crap
static inline cputime64_t get_cpu_idle_time_jiffy(unsigned int cpu,
							cputime64_t *wall)
{
	cputime64_t idle_time;
	cputime64_t cur_wall_time;
	cputime64_t busy_time;

	cur_wall_time = jiffies64_to_cputime64(get_jiffies_64());
	busy_time =  kcpustat_cpu(cpu).cpustat[CPUTIME_USER];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_NICE];

	idle_time = cur_wall_time - busy_time;
	if (wall)
		*wall = (cputime64_t)jiffies_to_usecs(cur_wall_time);

	return (cputime64_t)jiffies_to_usecs(idle_time);
}

static inline cputime64_t get_cpu_idle_time(unsigned int cpu, cputime64_t *wall)
{
	u64 idle_time = get_cpu_idle_time_us(cpu, wall);

	if (idle_time == -1ULL)
		return get_cpu_idle_time_jiffy(cpu, wall);

	return idle_time;
}

/* keep track of frequency transitions */
static int
dbs_cpufreq_notifier(struct notifier_block *nb, unsigned long val,
		     void *data)
{
	struct cpufreq_freqs *freq = data;
	struct cpu_dbs_info_s *this_dbs_info = &per_cpu(cs_cpu_dbs_info,
							freq->cpu);

	struct cpufreq_policy *policy;

	if (!this_dbs_info->enable)
		return 0;

	policy = this_dbs_info->cur_policy;

	/*
	 * we only care if our internally tracked freq moves outside
	 * the 'valid' ranges of freqency available to us otherwise
	 * we do not change it
	*/
	if (this_dbs_info->requested_freq > policy->max
			|| this_dbs_info->requested_freq < policy->min)
		this_dbs_info->requested_freq = freq->new;

	return 0;
}

static struct notifier_block dbs_cpufreq_notifier_block = {
	.notifier_call = dbs_cpufreq_notifier
};
// }}}
// {{{2 sysfs crap
/************************** sysfs interface ************************/
static ssize_t show_sampling_rate_min(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", 10000);
}

define_one_global_ro(sampling_rate_min);

/* cpufreq_freelunch Governor Tunables */
#define show_one(file_name, object)							\
static ssize_t show_##file_name								\
(struct kobject *kobj, struct attribute *attr, char *buf)	\
{															\
	return sprintf(buf, "%u\n", dbs_tuners_ins.object);		\
}
#define i_am_lazy(f, min, max)						\
show_one(f,f)										\
static ssize_t store_##f							\
(struct kobject *a, struct attribute *b,			\
				   const char *buf, size_t count)	\
{													\
	unsigned int input; int ret;					\
	ret = sscanf(buf, "%u", &input);				\
	if (ret != 1 || input < min || input > max) return -EINVAL;	\
	dbs_tuners_ins.f = input;						\
	return count;									\
}													\
define_one_global_rw(f);

show_one(sampling_rate, sampling_rate);
show_one(ignore_nice_load, ignore_nice);
i_am_lazy(hotplug_up_cycles, 0, 100)
i_am_lazy(hotplug_down_cycles, 0, 100)
i_am_lazy(hotplug_up_load, 0, 100)
i_am_lazy(hotplug_up_usage, 0, 4000000)
i_am_lazy(hotplug_down_usage, 0, 4000000)
i_am_lazy(overestimate_khz, 0, 4000000)
i_am_lazy(hispeed_thresh, 0, 4000000)
i_am_lazy(hispeed_decrease, 0, 4000000)
i_am_lazy(interaction_sampling_rate, 10000, 1000000)
i_am_lazy(interaction_overestimate_khz, 0, 4000000)
i_am_lazy(interaction_return_usage, 0, 4000000)
i_am_lazy(interaction_return_cycles, 0, 100)
i_am_lazy(interaction_hispeed, 0, 4000000)

static ssize_t store_sampling_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.sampling_rate = max(input, (unsigned int)10000);
	return count;
}

static ssize_t store_ignore_nice_load(struct kobject *a, struct attribute *b,
				      const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == dbs_tuners_ins.ignore_nice) /* nothing to do */
		return count;

	dbs_tuners_ins.ignore_nice = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(cs_cpu_dbs_info, j);
		dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&dbs_info->prev_cpu_wall);
		if (dbs_tuners_ins.ignore_nice)
			dbs_info->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];
	}
	return count;
}

define_one_global_rw(sampling_rate);
define_one_global_rw(ignore_nice_load);

static struct attribute *dbs_attributes[] = {
	&sampling_rate_min.attr,
	&sampling_rate.attr,
	&ignore_nice_load.attr,
	&hotplug_up_cycles.attr,
	&hotplug_down_cycles.attr,
	&hotplug_up_load.attr,
	&hotplug_up_usage.attr,
	&hotplug_down_usage.attr,
	&overestimate_khz.attr,
	&hispeed_thresh.attr,
	&hispeed_decrease.attr,
	&interaction_sampling_rate.attr,
	&interaction_overestimate_khz.attr,
	&interaction_return_usage.attr,
	&interaction_return_cycles.attr,
	&interaction_hispeed.attr,
	NULL
};

static struct attribute_group dbs_attr_group = {
	.attrs = dbs_attributes,
	.name = "freelunch",
};

/************************** sysfs end ************************/
// }}}
// {{{1 useful crap
static void __cpuinit do_cpu_up(struct work_struct *work) {
	if (num_online_cpus() == 1) cpu_up(1);
}
static void do_cpu_down(struct work_struct *work) {
	if (num_online_cpus() > 1) cpu_down(1);
}

static void dbs_check_cpu(struct cpu_dbs_info_s *this_dbs_info)
{
	unsigned int load;
	cputime64_t cur_wall_time, cur_idle_time;
	unsigned int idle_time, wall_time;

	struct cpufreq_policy *policy;

	unsigned int overestimate, fml;

	/* Calculate load for this processor only.  The assumption is that we're
	 * running on an aSMP processor where each core has its own instance.
	 *
	 * XXX This will not work on processors with linked frequencies, since they
	 * have multiple cores per policy.
	 */
	policy = this_dbs_info->cur_policy;

	cur_idle_time = get_cpu_idle_time(policy->cpu, &cur_wall_time);

	wall_time = (unsigned int)
		(cur_wall_time - this_dbs_info->prev_cpu_wall);
	this_dbs_info->prev_cpu_wall = cur_wall_time;

	idle_time = (unsigned int)
		(cur_idle_time - this_dbs_info->prev_cpu_idle);
	this_dbs_info->prev_cpu_idle = cur_idle_time;

	if (dbs_tuners_ins.ignore_nice) {
		cputime64_t cur_nice;
		unsigned long cur_nice_jiffies;

		cur_nice = kcpustat_cpu(policy->cpu).cpustat[CPUTIME_NICE] -
			this_dbs_info->prev_cpu_nice;
		cur_nice_jiffies = (unsigned long)
			cputime64_to_jiffies64(cur_nice);

		this_dbs_info->prev_cpu_nice = kcpustat_cpu(policy->cpu).cpustat[CPUTIME_NICE];
		idle_time += jiffies_to_usecs(cur_nice_jiffies);
	}

	/* Apparently, this happens. */
	if (unlikely(idle_time > wall_time)) return;

	/* KHz/us is not a good idea.  Increase precision a bit. */
	while (wall_time > 2000) {
		wall_time /= 10;
		idle_time /= 10;
	}

	/* Assume we've only run for a small fraction of a second. */
	load = policy->cur / wall_time * (wall_time - idle_time);

	/* Hotplug? */
	if (num_online_cpus() == 1) {
#if 1
		// RAWR!
		if (nr_running() >= dbs_tuners_ins.hotplug_up_load) {
			if ((this_dbs_info->hotplug_cycle++ >= dbs_tuners_ins.hotplug_up_cycles) &&
				load > dbs_tuners_ins.hotplug_up_usage) {
#else
		// meow.
		if (nr_running() >= dbs_tuners_ins.hotplug_up_load &&
			load > dbs_tuners_ins.hotplug_up_usage) {
			if (this_dbs_info->hotplug_cycle++ >= dbs_tuners_ins.hotplug_up_cycles) {
#endif
				schedule_work_on(0, &cpu_up_work);
				this_dbs_info->hotplug_cycle = 0;
			}
		} else this_dbs_info->hotplug_cycle = 0;
	} else {
		/* TODO: verify other core isn't loaded enough that we'll immediately
		 * bring core 2 back up?
		 */
		if (load < dbs_tuners_ins.hotplug_down_usage) {
			if (this_dbs_info->hotplug_cycle++ >= dbs_tuners_ins.hotplug_down_cycles) {
				schedule_work_on(0, &cpu_down_work);
				this_dbs_info->hotplug_cycle = 0;
			}
		} else this_dbs_info->hotplug_cycle = 0;
	}

	/* Return from interaction? */
	if (IGF(ENABLED)) {
		if (!IGF(PRESSED)) {
#ifdef DEFER_STATS
			this_dbs_info->deferred_return++;
#endif
			if (load < dbs_tuners_ins.interaction_return_usage) {
				if (this_dbs_info->defer_cycles++ >= dbs_tuners_ins.interaction_return_cycles) {
					IUF(ENABLED);
#ifdef DEFER_STATS
					printk(KERN_DEBUG "freelunch: deferred noninteractive %u cycles.\n",
						this_dbs_info->deferred_return);
#endif
				}
			} else this_dbs_info->defer_cycles = 0;
		}
		overestimate = dbs_tuners_ins.interaction_overestimate_khz;
	} else {
		overestimate = dbs_tuners_ins.overestimate_khz;
	}

	/* Update max_freq: will always be >= load */
	if (!dbs_tuners_ins.hispeed_decrease) {
		this_dbs_info->max_freq = load + overestimate;
	} else {
		if (IGF(PRESSED)) {
			this_dbs_info->max_freq = max(dbs_tuners_ins.interaction_hispeed,
				this_dbs_info->max_freq - dbs_tuners_ins.hispeed_decrease);
		} else {
			this_dbs_info->max_freq -= min(this_dbs_info->max_freq,
				dbs_tuners_ins.hispeed_decrease);
		}
		if (load + overestimate > this_dbs_info->max_freq)
			this_dbs_info->max_freq = load + overestimate;
	}

	/* Set frequency */
	this_dbs_info->requested_freq += load + dbs_tuners_ins.hispeed_thresh > policy->cur ?
		this_dbs_info->max_freq : load + overestimate;
	/* Bound request just outside available range
	 * Ideally, this helps stabilize idle @ min, load @ max
	 */
	fml = (policy->min < overestimate ? 0 : policy->min - overestimate);
	if (this_dbs_info->requested_freq > policy->cur) {
		this_dbs_info->requested_freq -= policy->cur;
		if (this_dbs_info->requested_freq < fml)
			this_dbs_info->requested_freq = fml;
		else if (this_dbs_info->requested_freq > policy->max + overestimate)
			this_dbs_info->requested_freq = policy->max + overestimate;
	} else {
		this_dbs_info->requested_freq = fml;
	}

	__cpufreq_driver_target(policy, this_dbs_info->requested_freq,
		CPUFREQ_RELATION_H);
}
// }}}
// {{{2 cpufreq crap
static void do_dbs_timer(struct work_struct *work)
{
	struct cpu_dbs_info_s *this_dbs_info =
		container_of(work, struct cpu_dbs_info_s, work.work);
	unsigned int cpu = this_dbs_info->cpu;

	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay;
	if (IGF(ENABLED))
		delay = usecs_to_jiffies(dbs_tuners_ins.interaction_sampling_rate);
	else
		delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);

	delay -= jiffies % delay;

	mutex_lock(&this_dbs_info->timer_mutex);

	dbs_check_cpu(this_dbs_info);

	schedule_delayed_work_on(cpu, &this_dbs_info->work, delay);
	mutex_unlock(&this_dbs_info->timer_mutex);
}

static inline void dbs_timer_init(struct cpu_dbs_info_s *dbs_info)
{
	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);
	delay -= jiffies % delay;

	dbs_info->enable = 1;
	INIT_DELAYED_WORK_DEFERRABLE(&dbs_info->work, do_dbs_timer);
	schedule_delayed_work_on(dbs_info->cpu, &dbs_info->work, delay);
}

static inline void dbs_timer_exit(struct cpu_dbs_info_s *dbs_info)
{
	dbs_info->enable = 0;
	cancel_delayed_work_sync(&dbs_info->work);
}

static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				   unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpu_dbs_info_s *this_dbs_info;
	unsigned int j;
	int rc;

	this_dbs_info = &per_cpu(cs_cpu_dbs_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		mutex_lock(&dbs_mutex);

		for_each_cpu(j, policy->cpus) {
			struct cpu_dbs_info_s *j_dbs_info;
			j_dbs_info = &per_cpu(cs_cpu_dbs_info, j);
			j_dbs_info->cur_policy = policy;

			j_dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&j_dbs_info->prev_cpu_wall);
			if (dbs_tuners_ins.ignore_nice) {
				j_dbs_info->prev_cpu_nice =
						kcpustat_cpu(j).cpustat[CPUTIME_NICE];
			}
		}
		this_dbs_info->requested_freq = policy->cur;
		this_dbs_info->hotplug_cycle = 0;
		this_dbs_info->defer_cycles = 0;
#ifdef DEFER_STATS
		this_dbs_info->deferred_return = 0;
#endif
		/* Dirty hack */
		if (cpu > 0)
			this_dbs_info->is_interactive = per_cpu(cs_cpu_dbs_info, 0).is_interactive;

		mutex_init(&this_dbs_info->timer_mutex);
		dbs_enable++;
		/*
		 * Start the timerschedule work, when this governor
		 * is used for first time
		 */
		if (dbs_enable == 1) {
			rc = sysfs_create_group(cpufreq_global_kobject,
						&dbs_attr_group);
			if (rc) {
				mutex_unlock(&dbs_mutex);
				return rc;
			}

			cpufreq_register_notifier(
					&dbs_cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);
		}
		mutex_unlock(&dbs_mutex);

		dbs_timer_init(this_dbs_info);

		break;

	case CPUFREQ_GOV_STOP:
		dbs_timer_exit(this_dbs_info);

		mutex_lock(&dbs_mutex);
		dbs_enable--;
		mutex_destroy(&this_dbs_info->timer_mutex);

		/*
		 * Stop the timerschedule work, when this governor
		 * is used for first time
		 */
		if (dbs_enable == 0)
			cpufreq_unregister_notifier(
					&dbs_cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);

		mutex_unlock(&dbs_mutex);
		if (!dbs_enable)
			sysfs_remove_group(cpufreq_global_kobject,
					   &dbs_attr_group);

		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&this_dbs_info->timer_mutex);
		if (policy->max < this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(
					this_dbs_info->cur_policy,
					policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(
					this_dbs_info->cur_policy,
					policy->min, CPUFREQ_RELATION_L);
		mutex_unlock(&this_dbs_info->timer_mutex);

		break;

	case CPUFREQ_GOV_INTERACT:
		mutex_lock(&this_dbs_info->timer_mutex);
		this_dbs_info->max_freq = max(this_dbs_info->max_freq, dbs_tuners_ins.interaction_hispeed);
		if (!IGF(ENABLED)) {
			ISF(ENABLED);
			if (cancel_delayed_work_sync(&this_dbs_info->work)) {
				this_dbs_info->prev_cpu_idle =
					get_cpu_idle_time(policy->cpu, &this_dbs_info->prev_cpu_wall);
				schedule_delayed_work_on(this_dbs_info->cpu, &this_dbs_info->work,
					usecs_to_jiffies(dbs_tuners_ins.interaction_sampling_rate));
			}
		}
		mutex_unlock(&this_dbs_info->timer_mutex);

		break;

	case CPUFREQ_GOV_NOINTERACT:
		mutex_lock(&this_dbs_info->timer_mutex);
		if (IGF(PRESSED)) {
			IUF(PRESSED);
			this_dbs_info->defer_cycles = 0;
#ifdef DEFER_STATS
			this_dbs_info->deferred_return = 0;
#endif
		}
		mutex_unlock(&this_dbs_info->timer_mutex);

		break;
	}
	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_FREELUNCH
static
#endif
struct cpufreq_governor cpufreq_gov_freelunch = {
	.name			= "freelunch",
	.governor		= cpufreq_governor_dbs,
	.max_transition_latency	= 10000000,
	.owner			= THIS_MODULE,
	.flags			= BIT(GOVFLAGS_HOTPLUG) | BIT(GOVFLAGS_ALLCPUS),
};

static int __init cpufreq_gov_dbs_init(void)
{
	INIT_WORK(&cpu_up_work, do_cpu_up);
	INIT_WORK(&cpu_down_work, do_cpu_down);
	return cpufreq_register_governor(&cpufreq_gov_freelunch);
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_freelunch);
}

MODULE_AUTHOR("Ryan Pennucci <decimalman@gmail.com>");
MODULE_DESCRIPTION("'cpufreq_freelunch' -- an incredibly simple hotplugging governor.");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_FREELUNCH
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
// }}}
// vim:ts=4:sw=4:fdm=marker:fdl=1
