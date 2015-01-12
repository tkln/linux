/*
 *  HW NMI watchdog support
 *
 *  started by Don Zickus, Copyright (C) 2010 Red Hat, Inc.
 *
 *  Arch specific calls to support NMI watchdog
 *
 *  Bits copied from original nmi.c file
 *
 */
#include <asm/apic.h>
#include <asm/nmi.h>

#include <linux/cpumask.h>
#include <linux/kdebug.h>
#include <linux/notifier.h>
#include <linux/kprobes.h>
#include <linux/nmi.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/seq_buf.h>

#ifdef CONFIG_HARDLOCKUP_DETECTOR
u64 hw_nmi_get_sample_period(int watchdog_thresh)
{
	return (u64)(cpu_khz) * 1000 * watchdog_thresh;
}
#endif

#ifdef arch_trigger_all_cpu_backtrace
/* For reliability, we're prepared to waste bits here. */
static DECLARE_BITMAP(backtrace_mask, NR_CPUS) __read_mostly;
static cpumask_t printtrace_mask;

void arch_trigger_all_cpu_backtrace(bool include_self)
{
	int i;
	int this_cpu = get_cpu();

	if (0 != prepare_nmi_printk(to_cpumask(backtrace_mask))) {
		/*
		 * If there is already an nmi printk sequence in
		 * progress then just give up...
		 */
		put_cpu();
		return;
	}

	if (!include_self)
		cpumask_clear_cpu(this_cpu, to_cpumask(backtrace_mask));
	cpumask_copy(&printtrace_mask, to_cpumask(backtrace_mask));

	if (!cpumask_empty(to_cpumask(backtrace_mask))) {
		pr_info("sending NMI to %s CPUs:\n",
			(include_self ? "all" : "other"));
		apic->send_IPI_mask(to_cpumask(backtrace_mask), NMI_VECTOR);
	}

	/* Wait for up to 10 seconds for all CPUs to do the backtrace */
	for (i = 0; i < 10 * 1000; i++) {
		if (cpumask_empty(to_cpumask(backtrace_mask)))
			break;
		mdelay(1);
		touch_softlockup_watchdog();
	}

	complete_nmi_printk(&printtrace_mask);
	put_cpu();
}

static int
arch_trigger_all_cpu_backtrace_handler(unsigned int cmd, struct pt_regs *regs)
{
	int cpu;
	printk_func_t orig;

	cpu = smp_processor_id();

	if (cpumask_test_cpu(cpu, to_cpumask(backtrace_mask))) {
		orig = this_cpu_begin_nmi_printk();
		printk(KERN_WARNING "NMI backtrace for cpu %d\n", cpu);
		show_regs(regs);
		this_cpu_end_nmi_printk(orig);

		cpumask_clear_cpu(cpu, to_cpumask(backtrace_mask));
		return NMI_HANDLED;
	}

	return NMI_DONE;
}
NOKPROBE_SYMBOL(arch_trigger_all_cpu_backtrace_handler);

static int __init register_trigger_all_cpu_backtrace(void)
{
	register_nmi_handler(NMI_LOCAL, arch_trigger_all_cpu_backtrace_handler,
				0, "arch_bt");
	return 0;
}
early_initcall(register_trigger_all_cpu_backtrace);
#endif
