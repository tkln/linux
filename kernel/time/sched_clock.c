/*
 * sched_clock.c: support for extending counters to full 64-bit ns counter
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clocksource.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/syscore_ops.h>
#include <linux/hrtimer.h>
#include <linux/sched_clock.h>
#include <linux/seqlock.h>
#include <linux/bitops.h>

struct clock_data_banked {
	u64 epoch_ns;
	u64 epoch_cyc;
	u64 (*read_sched_clock)(void);
	u64 sched_clock_mask;
	u32 mult;
	u32 shift;
	bool suspended;
};

struct clock_data {
	ktime_t wrap_kt;
	seqcount_t seq;
	unsigned long rate;
	struct clock_data_banked bank[2];
};

static struct hrtimer sched_clock_timer;
static int irqtime = -1;

core_param(irqtime, irqtime, int, 0400);

static u64 notrace jiffy_sched_clock_read(void)
{
	/*
	 * We don't need to use get_jiffies_64 on 32-bit arches here
	 * because we register with BITS_PER_LONG
	 */
	return (u64)(jiffies - INITIAL_JIFFIES);
}

static struct clock_data cd = {
	.bank = {
		[0] = {
			.mult	= NSEC_PER_SEC / HZ,
			.read_sched_clock = jiffy_sched_clock_read,
		},
	},
};

static inline u64 notrace cyc_to_ns(u64 cyc, u32 mult, u32 shift)
{
	return (cyc * mult) >> shift;
}

unsigned long long notrace sched_clock(void)
{
	u64 cyc;
	unsigned long seq;
	struct clock_data_banked *b;
	u64 res;

	do {
		seq = raw_read_seqcount(&cd.seq);
		b = cd.bank + (seq & 1);
		if (b->suspended) {
			res = b->epoch_ns;
		} else {
			cyc = b->read_sched_clock();
			cyc = (cyc - b->epoch_cyc) & b->sched_clock_mask;
			res = b->epoch_ns + cyc_to_ns(cyc, b->mult, b->shift);
		}
	} while (read_seqcount_retry(&cd.seq, seq));

	return res;
}

/*
 * Start updating the banked clock data.
 *
 * sched_clock will never observe mis-matched data even if called from
 * an NMI. We do this by maintaining an odd/even copy of the data and
 * steering sched_clock to one or the other using a sequence counter.
 * In order to preserve the data cache profile of sched_clock as much
 * as possible the system reverts back to the even copy when the update
 * completes; the odd copy is used *only* during an update.
 *
 * The caller is responsible for avoiding simultaneous updates.
 */
static struct clock_data_banked *update_bank_begin(void)
{
	/* update the backup (odd) bank and steer readers towards it */
	memcpy(cd.bank + 1, cd.bank, sizeof(struct clock_data_banked));
	raw_write_seqcount_latch(&cd.seq);

	return cd.bank;
}

/*
 * Finalize update of banked clock data.
 *
 * This is just a trivial switch back to the primary (even) copy.
 */
static void update_bank_end(void)
{
	raw_write_seqcount_latch(&cd.seq);
}

/*
 * Atomically update the sched_clock epoch.
 */
static void notrace update_sched_clock(bool suspended)
{
	struct clock_data_banked *b;
	u64 cyc;
	u64 ns;

	b = update_bank_begin();

	cyc = b->read_sched_clock();
	ns = b->epoch_ns + cyc_to_ns((cyc - b->epoch_cyc) & b->sched_clock_mask,
				     b->mult, b->shift);

	b->epoch_ns = ns;
	b->epoch_cyc = cyc;
	b->suspended = suspended;

	update_bank_end();
}

static enum hrtimer_restart sched_clock_poll(struct hrtimer *hrt)
{
	update_sched_clock(false);
	hrtimer_forward_now(hrt, cd.wrap_kt);
	return HRTIMER_RESTART;
}

void __init sched_clock_register(u64 (*read)(void), int bits,
				 unsigned long rate)
{
	u64 res, wrap, new_mask, new_epoch, cyc, ns;
	u32 new_mult, new_shift;
	unsigned long r;
	char r_unit;
	struct clock_data_banked *b;

	if (cd.rate > rate)
		return;

	WARN_ON(!irqs_disabled());

	/* calculate the mult/shift to convert counter ticks to ns. */
	clocks_calc_mult_shift(&new_mult, &new_shift, rate, NSEC_PER_SEC, 3600);
	cd.rate = rate;

	new_mask = CLOCKSOURCE_MASK(bits);

	/* calculate how many ns until we wrap */
	wrap = clocks_calc_max_nsecs(new_mult, new_shift, 0, new_mask);
	cd.wrap_kt = ns_to_ktime(wrap - (wrap >> 3));

	b = update_bank_begin();

	/* update epoch for new counter and update epoch_ns from old counter*/
	new_epoch = read();
	cyc = b->read_sched_clock();
	ns = b->epoch_ns + cyc_to_ns((cyc - b->epoch_cyc) & b->sched_clock_mask,
				     b->mult, b->shift);

	b->read_sched_clock = read;
	b->sched_clock_mask = new_mask;
	b->mult = new_mult;
	b->shift = new_shift;
	b->epoch_cyc = new_epoch;
	b->epoch_ns = ns;

	update_bank_end();

	r = rate;
	if (r >= 4000000) {
		r /= 1000000;
		r_unit = 'M';
	} else if (r >= 1000) {
		r /= 1000;
		r_unit = 'k';
	} else
		r_unit = ' ';

	/* calculate the ns resolution of this counter */
	res = cyc_to_ns(1ULL, new_mult, new_shift);

	pr_info("sched_clock: %u bits at %lu%cHz, resolution %lluns, wraps every %lluns\n",
		bits, r, r_unit, res, wrap);

	/* Enable IRQ time accounting if we have a fast enough sched_clock */
	if (irqtime > 0 || (irqtime == -1 && rate >= 1000000))
		enable_sched_clock_irqtime();

	pr_debug("Registered %pF as sched_clock source\n", read);
}

void __init sched_clock_postinit(void)
{
	/*
	 * If no sched_clock function has been provided at that point,
	 * make it the final one one.
	 */
	if (cd.bank[0].read_sched_clock == jiffy_sched_clock_read)
		sched_clock_register(jiffy_sched_clock_read, BITS_PER_LONG, HZ);

	update_sched_clock(false);

	/*
	 * Start the timer to keep sched_clock() properly updated and
	 * sets the initial epoch.
	 */
	hrtimer_init(&sched_clock_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	sched_clock_timer.function = sched_clock_poll;
	hrtimer_start(&sched_clock_timer, cd.wrap_kt, HRTIMER_MODE_REL);
}

static int sched_clock_suspend(void)
{
	update_sched_clock(true);
	hrtimer_cancel(&sched_clock_timer);
	return 0;
}

static void sched_clock_resume(void)
{
	struct clock_data_banked *b;

	b = update_bank_begin();
	b->epoch_cyc = b->read_sched_clock();
	hrtimer_start(&sched_clock_timer, cd.wrap_kt, HRTIMER_MODE_REL);
	b->suspended = false;
	update_bank_end();
}

static struct syscore_ops sched_clock_ops = {
	.suspend = sched_clock_suspend,
	.resume = sched_clock_resume,
};

static int __init sched_clock_syscore_init(void)
{
	register_syscore_ops(&sched_clock_ops);
	return 0;
}
device_initcall(sched_clock_syscore_init);
