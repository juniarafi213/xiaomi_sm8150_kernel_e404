// SPDX-License-Identifier: GPL-2.0
/*
 * Schedhorizon CPUFreq Governor
 *
 * Based on Schedutil, created by xzr (libxzr) and optimized for heterogeneous
 * multi-core / tri-cluster architectures (such as Qualcomm SM8150).
 *
 * Introduces dynamic efficient frequency stepping and up-delay hold times
 * to eliminate transient thermal spikes from short bursts while sustaining
 * full performance during continuous workloads.
 *
 * Copyright (C) 2016, Intel Corporation
 * Copyright (C) 2020-2022, xzr <xzr467706992>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpufreq.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <uapi/linux/sched/types.h>

#include "sched.h"

#define IOWAIT_BOOST_MIN	(SCHED_CAPACITY_SCALE / 8)

/* Per-cluster default efficient frequencies & up-delays (in ns) */
static unsigned int default_efficient_freq_lp[] = {0};
static u64 default_up_delay_lp[] = {0};

static unsigned int default_efficient_freq_hp[] = {1766400};
static u64 default_up_delay_hp[] = {100 * NSEC_PER_MSEC};

static unsigned int default_efficient_freq_pr[] = {2073600};
static u64 default_up_delay_pr[] = {100 * NSEC_PER_MSEC};

struct sh_tunables {
	struct gov_attr_set	attr_set;
	unsigned int		rate_limit_us;
	unsigned int		*efficient_freq;
	int			nefficient_freq;
	u64			*up_delay;
	int			nup_delay;
	int			current_step;
};

struct sh_policy {
	struct cpufreq_policy	*policy;

	struct sh_tunables	*tunables;
	struct list_head	tunables_hook;

	raw_spinlock_t		update_lock;
	u64			last_freq_update_time;
	s64			freq_update_delay_ns;
	unsigned int		next_freq;
	unsigned int		cached_raw_freq;
	u64			first_hp_request_time;

	/* Slow path worker */
	struct irq_work		irq_work;
	struct kthread_work	work;
	struct mutex		work_lock;
	struct kthread_worker	worker;
	struct task_struct	*thread;
	bool			work_in_progress;

	bool			limits_changed;
	bool			need_freq_update;
};

struct sh_cpu {
	struct update_util_data	update_util;
	struct sh_policy	*sg_policy;
	unsigned int		cpu;

	bool			iowait_boost_pending;
	unsigned int		iowait_boost;
	u64			last_update;

	unsigned long		util;
	unsigned long		bw_min;
};

static DEFINE_PER_CPU(struct sh_cpu, sh_cpu);

/************************ Governor internals ***********************/

static bool sh_should_rate_limit(struct sh_policy *sg_policy, u64 time)
{
	s64 delta_ns = time - sg_policy->last_freq_update_time;

	return delta_ns < sg_policy->freq_update_delay_ns;
}

static bool sh_should_update_freq(struct sh_policy *sg_policy, u64 time)
{
	if (!cpufreq_this_cpu_can_update(sg_policy->policy))
		return false;

	if (unlikely(READ_ONCE(sg_policy->limits_changed))) {
		WRITE_ONCE(sg_policy->limits_changed, false);
		sg_policy->need_freq_update = true;
		smp_mb();
		return true;
	} else if (sg_policy->need_freq_update) {
		return true;
	}

	if (arch_scale_freq_invariant())
		return true;

	return !sh_should_rate_limit(sg_policy, time);
}

static bool sh_update_next_freq(struct sh_policy *sg_policy, u64 time,
				unsigned int next_freq)
{
	if (sg_policy->need_freq_update) {
		sg_policy->need_freq_update = false;
		if (cpufreq_driver_test_flags(CPUFREQ_NEED_UPDATE_LIMITS))
			goto must_update;
	}

	if (next_freq == sg_policy->next_freq ||
	    (next_freq < sg_policy->next_freq &&
	     sh_should_rate_limit(sg_policy, time)))
		return false;

must_update:
	sg_policy->next_freq = next_freq;
	sg_policy->last_freq_update_time = time;

	return true;
}

static void sh_deferred_update(struct sh_policy *sg_policy)
{
	if (!sg_policy->work_in_progress) {
		sg_policy->work_in_progress = true;
		irq_work_queue(&sg_policy->irq_work);
	}
}

static inline int sh_match_nearest_efficient_step(unsigned int freq, int maxstep,
						  const unsigned int *freq_table)
{
	int i;

	for (i = 0; i < maxstep; i++) {
		if (freq_table[i] >= freq)
			break;
	}

	return (i >= maxstep) ? (maxstep - 1) : i;
}

static inline void sh_do_freq_limit(struct sh_policy *sg_policy, unsigned int *freq, u64 time)
{
	struct sh_tunables *tunables = sg_policy->tunables;
	int step;

	if (!tunables || !tunables->nefficient_freq || !tunables->efficient_freq)
		return;

	/* If efficient frequency is {0}, no stepping limit is enforced */
	if (tunables->nefficient_freq == 1 && tunables->efficient_freq[0] == 0)
		return;

	step = tunables->current_step;
	if (step >= tunables->nefficient_freq)
		step = tunables->nefficient_freq - 1;

	/* 1. Request exceeds current efficient step: gate and record start time */
	if (*freq > tunables->efficient_freq[step] && !sg_policy->first_hp_request_time) {
		*freq = tunables->efficient_freq[step];
		sg_policy->first_hp_request_time = time;
		return;
	}

	/* 2. Load dropped below current step: step down immediately */
	if (*freq < tunables->efficient_freq[step]) {
		tunables->current_step = sh_match_nearest_efficient_step(*freq,
					tunables->nefficient_freq, tunables->efficient_freq);
		sg_policy->first_hp_request_time = 0;
		return;
	}

	/* 3. Within up_delay duration: clamp to current efficient step */
	if (sg_policy->first_hp_request_time &&
	    step < tunables->nup_delay &&
	    time < sg_policy->first_hp_request_time + tunables->up_delay[step]) {
		*freq = tunables->efficient_freq[step];
		return;
	}

	/* 4. Delay expired and higher step exists: unlock higher tier */
	if (step + 1 < tunables->nefficient_freq && step + 1 < tunables->nup_delay) {
		tunables->current_step++;
		sg_policy->first_hp_request_time = time;
		if (*freq > tunables->efficient_freq[tunables->current_step])
			*freq = tunables->efficient_freq[tunables->current_step];
		return;
	}
}

static unsigned int sh_get_next_freq(struct sh_policy *sg_policy,
				     unsigned long util, unsigned long max,
				     u64 time)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned int freq;
	unsigned int idx, l_freq, h_freq;

	if (arch_scale_freq_invariant())
		freq = policy->cpuinfo.max_freq;
	else
		freq = policy->cur + (policy->cur >> 2);

	freq = map_util_freq(util, freq, max);

	/* Apply Schedhorizon efficient frequency stepping limit */
	sh_do_freq_limit(sg_policy, &freq, time);

	if (freq == sg_policy->cached_raw_freq && !sg_policy->need_freq_update)
		return sg_policy->next_freq;

	sg_policy->cached_raw_freq = freq;
	l_freq = cpufreq_driver_resolve_freq(policy, freq);
	idx = cpufreq_frequency_table_target(policy, freq, CPUFREQ_RELATION_H);
	h_freq = policy->freq_table[idx].frequency;
	h_freq = clamp(h_freq, policy->min, policy->max);
	if (l_freq <= h_freq || l_freq == policy->min)
		return l_freq;

	if (mult_frac(100, freq - h_freq, l_freq - h_freq) < 20)
		return h_freq;

	return l_freq;
}

static void sh_get_util(struct sh_cpu *sg_cpu, unsigned long boost)
{
	unsigned long min, max, util = cpu_util_cfs(cpu_rq(sg_cpu->cpu));

	util = effective_cpu_util(sg_cpu->cpu, util, &min, &max);
	util = max(util, boost);
	sg_cpu->bw_min = min;
	sg_cpu->util = sugov_effective_cpu_perf(sg_cpu->cpu, util, min, max);
}

static bool sh_iowait_reset(struct sh_cpu *sg_cpu, u64 time,
			    bool set_iowait_boost)
{
	s64 delta_ns = time - sg_cpu->last_update;

	if (delta_ns <= TICK_NSEC)
		return false;

	sg_cpu->iowait_boost = set_iowait_boost ? IOWAIT_BOOST_MIN : 0;
	sg_cpu->iowait_boost_pending = set_iowait_boost;

	return true;
}

static void sh_iowait_boost(struct sh_cpu *sg_cpu, u64 time,
			    unsigned int flags)
{
	bool set_iowait_boost = flags & SCHED_CPUFREQ_IOWAIT;

	if (sg_cpu->iowait_boost &&
	    sh_iowait_reset(sg_cpu, time, set_iowait_boost))
		return;

	if (!set_iowait_boost)
		return;

	if (sg_cpu->iowait_boost_pending)
		return;
	sg_cpu->iowait_boost_pending = true;

	if (sg_cpu->iowait_boost) {
		sg_cpu->iowait_boost =
			min_t(unsigned int, sg_cpu->iowait_boost << 1, SCHED_CAPACITY_SCALE);
		return;
	}

	sg_cpu->iowait_boost = IOWAIT_BOOST_MIN;
}

static unsigned long sh_iowait_apply(struct sh_cpu *sg_cpu, u64 time,
				     unsigned long max_cap)
{
	if (!sg_cpu->iowait_boost)
		return 0;

	if (sh_iowait_reset(sg_cpu, time, false))
		return 0;

	if (!sg_cpu->iowait_boost_pending) {
		sg_cpu->iowait_boost >>= 1;
		if (sg_cpu->iowait_boost < IOWAIT_BOOST_MIN) {
			sg_cpu->iowait_boost = 0;
			return 0;
		}
	}

	sg_cpu->iowait_boost_pending = false;

	return (sg_cpu->iowait_boost * max_cap) >> SCHED_CAPACITY_SHIFT;
}

static inline void sh_ignore_dl_rate_limit(struct sh_cpu *sg_cpu)
{
	if (cpu_bw_dl(cpu_rq(sg_cpu->cpu)) > sg_cpu->bw_min)
		sg_cpu->sg_policy->need_freq_update = true;
}

static inline bool sh_update_single_common(struct sh_cpu *sg_cpu,
					   u64 time, unsigned long max_cap,
					   unsigned int flags)
{
	unsigned long boost;

	sh_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	sh_ignore_dl_rate_limit(sg_cpu);

	if (!sh_should_update_freq(sg_cpu->sg_policy, time))
		return false;

	boost = sh_iowait_apply(sg_cpu, time, max_cap);
	sh_get_util(sg_cpu, boost);

	return true;
}

static void sh_update_single_freq(struct update_util_data *hook, u64 time,
				  unsigned int flags)
{
	struct sh_cpu *sg_cpu = container_of(hook, struct sh_cpu, update_util);
	struct sh_policy *sg_policy = sg_cpu->sg_policy;
	unsigned long max_cap;
	unsigned int next_f;

	max_cap = arch_scale_cpu_capacity(sg_cpu->cpu);

	if (!sh_update_single_common(sg_cpu, time, max_cap, flags))
		return;

	next_f = sh_get_next_freq(sg_policy, sg_cpu->util, max_cap, time);

	if (!sh_update_next_freq(sg_policy, time, next_f))
		return;

	if (sg_policy->policy->fast_switch_enabled) {
		cpufreq_driver_fast_switch(sg_policy->policy, next_f);
	} else {
		raw_spin_lock(&sg_policy->update_lock);
		sh_deferred_update(sg_policy);
		raw_spin_unlock(&sg_policy->update_lock);
	}
}

static void sh_update_single_perf(struct update_util_data *hook, u64 time,
				  unsigned int flags)
{
	struct sh_cpu *sg_cpu = container_of(hook, struct sh_cpu, update_util);
	unsigned long max_cap;

	if (!arch_scale_freq_invariant()) {
		sh_update_single_freq(hook, time, flags);
		return;
	}

	max_cap = arch_scale_cpu_capacity(sg_cpu->cpu);

	if (!sh_update_single_common(sg_cpu, time, max_cap, flags))
		return;

	cpufreq_driver_adjust_perf(sg_cpu->cpu, sg_cpu->bw_min,
				   sg_cpu->util, max_cap);

	sg_cpu->sg_policy->last_freq_update_time = time;
}

static unsigned int sh_next_freq_shared(struct sh_cpu *sg_cpu, u64 time)
{
	struct sh_policy *sg_policy = sg_cpu->sg_policy;
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util = 0, max_cap;
	unsigned int j;

	max_cap = arch_scale_cpu_capacity(sg_cpu->cpu);

	for_each_cpu(j, policy->cpus) {
		struct sh_cpu *j_sg_cpu = &per_cpu(sh_cpu, j);
		unsigned long boost;

		boost = sh_iowait_apply(j_sg_cpu, time, max_cap);
		sh_get_util(j_sg_cpu, boost);

		util = max(j_sg_cpu->util, util);
	}

	return sh_get_next_freq(sg_policy, util, max_cap, time);
}

static void sh_update_shared(struct update_util_data *hook, u64 time,
			     unsigned int flags)
{
	struct sh_cpu *sg_cpu = container_of(hook, struct sh_cpu, update_util);
	struct sh_policy *sg_policy = sg_cpu->sg_policy;
	unsigned int next_f;

	raw_spin_lock(&sg_policy->update_lock);

	sh_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	sh_ignore_dl_rate_limit(sg_cpu);

	if (sh_should_update_freq(sg_policy, time)) {
		next_f = sh_next_freq_shared(sg_cpu, time);

		if (!sh_update_next_freq(sg_policy, time, next_f))
			goto unlock;

		if (sg_policy->policy->fast_switch_enabled)
			cpufreq_driver_fast_switch(sg_policy->policy, next_f);
		else
			sh_deferred_update(sg_policy);
	}
unlock:
	raw_spin_unlock(&sg_policy->update_lock);
}

static void sh_work(struct kthread_work *work)
{
	struct sh_policy *sg_policy = container_of(work, struct sh_policy, work);
	unsigned int freq;
	unsigned long flags;

	raw_spin_lock_irqsave(&sg_policy->update_lock, flags);
	freq = sg_policy->next_freq;
	sg_policy->work_in_progress = false;
	raw_spin_unlock_irqrestore(&sg_policy->update_lock, flags);

	mutex_lock(&sg_policy->work_lock);
	__cpufreq_driver_target(sg_policy->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&sg_policy->work_lock);
}

static void sh_irq_work(struct irq_work *irq_work)
{
	struct sh_policy *sg_policy;

	sg_policy = container_of(irq_work, struct sh_policy, irq_work);

	kthread_queue_work(&sg_policy->worker, &sg_policy->work);
}

/************************** sysfs interface ************************/

static struct sh_tunables *global_tunables;
static DEFINE_MUTEX(global_tunables_lock);

static inline struct sh_tunables *to_sh_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct sh_tunables, attr_set);
}

static ssize_t rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sh_tunables *tunables = to_sh_tunables(attr_set);

	return sprintf(buf, "%u\n", tunables->rate_limit_us);
}

static ssize_t rate_limit_us_store(struct gov_attr_set *attr_set, const char *buf,
				   size_t count)
{
	struct sh_tunables *tunables = to_sh_tunables(attr_set);
	struct sh_policy *sg_policy;
	unsigned int rate_limit_us;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;

	tunables->rate_limit_us = rate_limit_us;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook)
		sg_policy->freq_update_delay_ns = (u64)rate_limit_us * NSEC_PER_USEC;

	return count;
}

static unsigned int *sh_resolve_data_freq(const char *buf, int *num_ret, size_t count)
{
	const char *cp;
	unsigned int *output;
	int num = 1, i = 0;

	cp = buf;
	while ((cp = strpbrk(cp + 1, " \t\n")))
		num++;

	output = kmalloc_array(num, sizeof(unsigned int), GFP_KERNEL);
	if (!output)
		return NULL;

	cp = buf;
	while (i < num && (size_t)(cp - buf) < count) {
		unsigned int val;

		while (*cp == ' ' || *cp == '\t' || *cp == '\n')
			cp++;
		if (!*cp)
			break;

		if (sscanf(cp, "%u", &val) != 1)
			goto err_kfree;

		output[i++] = val;
		cp = strpbrk(cp, " \t\n");
		if (!cp)
			break;
	}

	if (!i)
		goto err_kfree;

	*num_ret = i;
	return output;

err_kfree:
	kfree(output);
	return NULL;
}

static u64 *sh_resolve_data_delay(const char *buf, int *num_ret, size_t count)
{
	const char *cp;
	u64 *output;
	int num = 1, i = 0;

	cp = buf;
	while ((cp = strpbrk(cp + 1, " \t\n")))
		num++;

	output = kmalloc_array(num, sizeof(u64), GFP_KERNEL);
	if (!output)
		return NULL;

	cp = buf;
	while (i < num && (size_t)(cp - buf) < count) {
		unsigned long long val;

		while (*cp == ' ' || *cp == '\t' || *cp == '\n')
			cp++;
		if (!*cp)
			break;

		if (sscanf(cp, "%llu", &val) != 1)
			goto err_kfree;

		output[i++] = (u64)val * NSEC_PER_MSEC;
		cp = strpbrk(cp, " \t\n");
		if (!cp)
			break;
	}

	if (!i)
		goto err_kfree;

	*num_ret = i;
	return output;

err_kfree:
	kfree(output);
	return NULL;
}

static ssize_t efficient_freq_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sh_tunables *tunables = to_sh_tunables(attr_set);
	ssize_t ret = 0;
	int i;

	for (i = 0; i < tunables->nefficient_freq; i++)
		ret += sprintf(buf + ret, "%u ", tunables->efficient_freq[i]);

	if (ret > 0 && buf[ret - 1] == ' ')
		buf[ret - 1] = '\n';
	else
		ret += sprintf(buf + ret, "\n");

	return ret;
}

static ssize_t efficient_freq_store(struct gov_attr_set *attr_set,
				    const char *buf, size_t count)
{
	struct sh_tunables *tunables = to_sh_tunables(attr_set);
	unsigned int *new_efficient_freq, *old;
	int new_num = 0;

	new_efficient_freq = sh_resolve_data_freq(buf, &new_num, count);
	if (!new_efficient_freq)
		return -EINVAL;

	old = tunables->efficient_freq;
	tunables->efficient_freq = new_efficient_freq;
	tunables->nefficient_freq = new_num;
	tunables->current_step = 0;

	if (old != default_efficient_freq_lp &&
	    old != default_efficient_freq_hp &&
	    old != default_efficient_freq_pr)
		kfree(old);

	return count;
}

static ssize_t up_delay_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sh_tunables *tunables = to_sh_tunables(attr_set);
	ssize_t ret = 0;
	int i;

	for (i = 0; i < tunables->nup_delay; i++)
		ret += sprintf(buf + ret, "%llu ", tunables->up_delay[i] / NSEC_PER_MSEC);

	if (ret > 0 && buf[ret - 1] == ' ')
		buf[ret - 1] = '\n';
	else
		ret += sprintf(buf + ret, "\n");

	return ret;
}

static ssize_t up_delay_store(struct gov_attr_set *attr_set,
			      const char *buf, size_t count)
{
	struct sh_tunables *tunables = to_sh_tunables(attr_set);
	u64 *new_up_delay, *old;
	int new_num = 0;

	new_up_delay = sh_resolve_data_delay(buf, &new_num, count);
	if (!new_up_delay)
		return -EINVAL;

	old = tunables->up_delay;
	tunables->up_delay = new_up_delay;
	tunables->nup_delay = new_num;
	tunables->current_step = 0;

	if (old != default_up_delay_lp &&
	    old != default_up_delay_hp &&
	    old != default_up_delay_pr)
		kfree(old);

	return count;
}

static struct governor_attr rate_limit_us = __ATTR_RW(rate_limit_us);
static struct governor_attr efficient_freq = __ATTR_RW(efficient_freq);
static struct governor_attr up_delay = __ATTR_RW(up_delay);

static struct attribute *sh_attributes[] = {
	&rate_limit_us.attr,
	&efficient_freq.attr,
	&up_delay.attr,
	NULL
};

static void sh_tunables_free(struct kobject *kobj)
{
	struct gov_attr_set *attr_set = container_of(kobj, struct gov_attr_set, kobj);
	struct sh_tunables *tunables = to_sh_tunables(attr_set);

	if (tunables->efficient_freq != default_efficient_freq_lp &&
	    tunables->efficient_freq != default_efficient_freq_hp &&
	    tunables->efficient_freq != default_efficient_freq_pr)
		kfree(tunables->efficient_freq);

	if (tunables->up_delay != default_up_delay_lp &&
	    tunables->up_delay != default_up_delay_hp &&
	    tunables->up_delay != default_up_delay_pr)
		kfree(tunables->up_delay);

	kfree(tunables);
}

static struct kobj_type sh_tunables_ktype = {
	.default_attrs = sh_attributes,
	.sysfs_ops = &governor_sysfs_ops,
	.release = &sh_tunables_free,
};

/********************** cpufreq governor interface *********************/

static struct cpufreq_governor schedhorizon_gov;

static struct sh_policy *sh_policy_alloc(struct cpufreq_policy *policy)
{
	struct sh_policy *sg_policy;

	sg_policy = kzalloc(sizeof(*sg_policy), GFP_KERNEL);
	if (!sg_policy)
		return NULL;

	sg_policy->policy = policy;
	raw_spin_lock_init(&sg_policy->update_lock);
	return sg_policy;
}

static void sh_policy_free(struct sh_policy *sg_policy)
{
	kfree(sg_policy);
}

static int sh_kthread_create(struct sh_policy *sg_policy)
{
	struct task_struct *thread;
	struct sched_attr attr = {
		.size		= sizeof(struct sched_attr),
		.sched_policy	= SCHED_DEADLINE,
		.sched_flags	= SCHED_FLAG_SUGOV,
		.sched_nice	= 0,
		.sched_priority	= 0,
		.sched_runtime	=  1000000,
		.sched_deadline = 10000000,
		.sched_period	= 10000000,
	};
	struct cpufreq_policy *policy = sg_policy->policy;
	int ret;

	if (policy->fast_switch_enabled)
		return 0;

	kthread_init_work(&sg_policy->work, sh_work);
	kthread_init_worker(&sg_policy->worker);
	thread = kthread_create(kthread_worker_fn, &sg_policy->worker,
				"shgov:%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("failed to create shgov thread: %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = sched_setattr_nocheck(thread, &attr);
	if (ret) {
		kthread_stop(thread);
		pr_warn("%s: failed to set SCHED_DEADLINE\n", __func__);
		return ret;
	}

	sg_policy->thread = thread;
	kthread_bind_mask(thread, policy->related_cpus);
	init_irq_work(&sg_policy->irq_work, sh_irq_work);
	mutex_init(&sg_policy->work_lock);

	wake_up_process(thread);

	return 0;
}

static void sh_kthread_stop(struct sh_policy *sg_policy)
{
	if (sg_policy->policy->fast_switch_enabled)
		return;

	kthread_flush_worker(&sg_policy->worker);
	kthread_stop(sg_policy->thread);
	mutex_destroy(&sg_policy->work_lock);
}

static struct sh_tunables *sh_tunables_alloc(struct sh_policy *sg_policy)
{
	struct sh_tunables *tunables;

	tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
	if (tunables) {
		gov_attr_set_init(&tunables->attr_set, &sg_policy->tunables_hook);
		if (!have_governor_per_policy())
			global_tunables = tunables;
	}
	return tunables;
}

static void sh_clear_global_tunables(void)
{
	if (!have_governor_per_policy())
		global_tunables = NULL;
}

static int sh_init(struct cpufreq_policy *policy)
{
	struct sh_policy *sg_policy;
	struct sh_tunables *tunables;
	int ret = 0;
	int first_cpu;
	unsigned long cap;

	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	sg_policy = sh_policy_alloc(policy);
	if (!sg_policy) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	ret = sh_kthread_create(sg_policy);
	if (ret)
		goto free_sg_policy;

	mutex_lock(&global_tunables_lock);

	if (global_tunables) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL;
			goto stop_kthread;
		}
		policy->governor_data = sg_policy;
		sg_policy->tunables = global_tunables;

		gov_attr_set_get(&global_tunables->attr_set, &sg_policy->tunables_hook);
		goto out;
	}

	tunables = sh_tunables_alloc(sg_policy);
	if (!tunables) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

	policy->governor_data = sg_policy;
	sg_policy->tunables = tunables;

	tunables->rate_limit_us = 2000;

	first_cpu = cpumask_first(policy->cpus);
	cap = arch_scale_cpu_capacity(first_cpu);

	/* Cluster detection for SM8150 (Silver, Gold, Gold Prime) */
	if (cap <= 512 || first_cpu < 4) {
		tunables->efficient_freq = default_efficient_freq_lp;
		tunables->nefficient_freq = ARRAY_SIZE(default_efficient_freq_lp);
		tunables->up_delay = default_up_delay_lp;
		tunables->nup_delay = ARRAY_SIZE(default_up_delay_lp);
	} else if (first_cpu < 7) {
		tunables->efficient_freq = default_efficient_freq_hp;
		tunables->nefficient_freq = ARRAY_SIZE(default_efficient_freq_hp);
		tunables->up_delay = default_up_delay_hp;
		tunables->nup_delay = ARRAY_SIZE(default_up_delay_hp);
	} else {
		tunables->efficient_freq = default_efficient_freq_pr;
		tunables->nefficient_freq = ARRAY_SIZE(default_efficient_freq_pr);
		tunables->up_delay = default_up_delay_pr;
		tunables->nup_delay = ARRAY_SIZE(default_up_delay_pr);
	}

	ret = kobject_init_and_add(&tunables->attr_set.kobj, &sh_tunables_ktype,
				   get_governor_parent_kobj(policy), "%s",
				   schedhorizon_gov.name);
	if (ret)
		goto fail;

out:
	mutex_unlock(&global_tunables_lock);
	return 0;

fail:
	kobject_put(&tunables->attr_set.kobj);
	policy->governor_data = NULL;
	sh_clear_global_tunables();

stop_kthread:
	sh_kthread_stop(sg_policy);
	mutex_unlock(&global_tunables_lock);

free_sg_policy:
	sh_policy_free(sg_policy);

disable_fast_switch:
	cpufreq_disable_fast_switch(policy);

	pr_err("initialization failed (error %d)\n", ret);
	return ret;
}

static void sh_exit(struct cpufreq_policy *policy)
{
	struct sh_policy *sg_policy = policy->governor_data;
	struct sh_tunables *tunables = sg_policy->tunables;
	unsigned int count;

	mutex_lock(&global_tunables_lock);

	count = gov_attr_set_put(&tunables->attr_set, &sg_policy->tunables_hook);
	policy->governor_data = NULL;
	if (!count)
		sh_clear_global_tunables();

	mutex_unlock(&global_tunables_lock);

	sh_kthread_stop(sg_policy);
	sh_policy_free(sg_policy);
	cpufreq_disable_fast_switch(policy);
}

static int sh_start(struct cpufreq_policy *policy)
{
	struct sh_policy *sg_policy = policy->governor_data;
	void (*uu)(struct update_util_data *data, u64 time, unsigned int flags);
	unsigned int cpu;

	sg_policy->freq_update_delay_ns = (u64)sg_policy->tunables->rate_limit_us * NSEC_PER_USEC;
	sg_policy->last_freq_update_time = 0;
	sg_policy->next_freq = 0;
	sg_policy->work_in_progress = false;
	sg_policy->limits_changed = false;
	sg_policy->cached_raw_freq = 0;
	sg_policy->first_hp_request_time = 0;
	sg_policy->tunables->current_step = 0;

	sg_policy->need_freq_update = cpufreq_driver_test_flags(CPUFREQ_NEED_UPDATE_LIMITS);

	if (policy_is_shared(policy))
		uu = sh_update_shared;
	else if (policy->fast_switch_enabled && cpufreq_driver_has_adjust_perf())
		uu = sh_update_single_perf;
	else
		uu = sh_update_single_freq;

	for_each_cpu(cpu, policy->cpus) {
		struct sh_cpu *sg_cpu = &per_cpu(sh_cpu, cpu);

		memset(sg_cpu, 0, sizeof(*sg_cpu));
		sg_cpu->cpu = cpu;
		sg_cpu->sg_policy = sg_policy;
	}

	for_each_cpu(cpu, policy->cpus) {
		struct sh_cpu *sg_cpu = &per_cpu(sh_cpu, cpu);

		cpufreq_add_update_util_hook(cpu, &sg_cpu->update_util, uu);
	}

	return 0;
}

static void sh_stop(struct cpufreq_policy *policy)
{
	struct sh_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_rcu();

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&sg_policy->irq_work);
		kthread_cancel_work_sync(&sg_policy->work);
	}
}

static void sh_limits(struct cpufreq_policy *policy)
{
	struct sh_policy *sg_policy = policy->governor_data;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&sg_policy->work_lock);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&sg_policy->work_lock);
	}

	WRITE_ONCE(sg_policy->limits_changed, true);
}

static struct cpufreq_governor schedhorizon_gov = {
	.name			= "schedhorizon",
	.init			= sh_init,
	.exit			= sh_exit,
	.start			= sh_start,
	.stop			= sh_stop,
	.limits			= sh_limits,
	.owner			= THIS_MODULE,
};

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDHORIZON
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return &schedhorizon_gov;
}
#endif

static int __init schedhorizon_gov_init(void)
{
	return cpufreq_register_governor(&schedhorizon_gov);
}

static void __exit schedhorizon_gov_exit(void)
{
	cpufreq_unregister_governor(&schedhorizon_gov);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("xzr <xzr467706992>");
MODULE_DESCRIPTION("Schedhorizon CPUFreq Governor");

module_init(schedhorizon_gov_init);
module_exit(schedhorizon_gov_exit);
