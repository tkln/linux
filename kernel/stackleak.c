// SPDX-License-Identifier: GPL-2.0
/*
 * This code fills the used part of the kernel stack with a poison value
 * before returning to the userspace. It's a part of the STACKLEAK feature
 * ported from grsecurity/PaX.
 *
 * Author: Alexander Popov <alex.popov@linux.com>
 *
 * STACKLEAK reduces the information which kernel stack leak bugs can
 * reveal and blocks some uninitialized stack variable attacks. Moreover,
 * STACKLEAK blocks stack depth overflow caused by alloca() (aka Stack Clash
 * attack).
 */

#include <linux/stackleak.h>

asmlinkage void stackleak_erase_kstack(void)
{
	/* It would be nice not to have 'kstack_ptr' and 'boundary' on stack */
	unsigned long kstack_ptr = current->lowest_stack;
	unsigned long boundary = kstack_ptr & ~(THREAD_SIZE - 1);
	unsigned int poison_count = 0;
	const unsigned int check_depth =
			STACKLEAK_POISON_CHECK_DEPTH / sizeof(unsigned long);

	/* Search for the poison value in the kernel stack */
	while (kstack_ptr > boundary && poison_count <= check_depth) {
		if (*(unsigned long *)kstack_ptr == STACKLEAK_POISON)
			poison_count++;
		else
			poison_count = 0;

		kstack_ptr -= sizeof(unsigned long);
	}

	/*
	 * One 'long int' at the bottom of the thread stack is reserved and
	 * should not be poisoned (see CONFIG_SCHED_STACK_END_CHECK).
	 */
	if (kstack_ptr == boundary)
		kstack_ptr += sizeof(unsigned long);

#ifdef CONFIG_STACKLEAK_METRICS
	current->prev_lowest_stack = kstack_ptr;
#endif

	/*
	 * Now write the poison value to the kernel stack. Start from
	 * 'kstack_ptr' and move up till the new 'boundary'. We assume that
	 * the stack pointer doesn't change when we write poison.
	 */
	if (on_thread_stack())
		boundary = current_stack_pointer;
	else
		boundary = current_top_of_stack();

	BUG_ON(boundary - kstack_ptr >= THREAD_SIZE);

	while (kstack_ptr < boundary) {
		*(unsigned long *)kstack_ptr = STACKLEAK_POISON;
		kstack_ptr += sizeof(unsigned long);
	}

	/* Reset the 'lowest_stack' value for the next syscall */
	current->lowest_stack = current_top_of_stack() - THREAD_SIZE / 64;
}

void __used stackleak_track_stack(void)
{
	/*
	 * N.B. stackleak_erase_kstack() fills the kernel stack with the poison
	 * value, which has the register width. That code assumes that
	 * the value of lowest_stack is aligned on the register width boundary.
	 *
	 * That is true for x86 and x86_64 because of the kernel stack
	 * alignment on these platforms (for details, see cc_stack_align in
	 * arch/x86/Makefile). Take care of that when you port STACKLEAK to
	 * new platforms.
	 */
	unsigned long sp = (unsigned long)&sp;

	/*
	 * Having CONFIG_STACKLEAK_TRACK_MIN_SIZE larger than
	 * STACKLEAK_POISON_CHECK_DEPTH makes the poison search in
	 * stackleak_erase_kstack() unreliable. Let's prevent that.
	 */
	BUILD_BUG_ON(CONFIG_STACKLEAK_TRACK_MIN_SIZE >
						STACKLEAK_POISON_CHECK_DEPTH);

	if (sp < current->lowest_stack &&
	    sp >= (unsigned long)task_stack_page(current) +
						sizeof(unsigned long)) {
		current->lowest_stack = sp;
	}
}
EXPORT_SYMBOL(stackleak_track_stack);
