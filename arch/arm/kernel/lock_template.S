/*
 * arch/arm/kernel/lock_template.S
 * Generic spinlock slowpath implementation
 *
 * Copyright (C) 2014-2017, Ryan Pennucci <decimalman@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

/* __arch_XXX_lock_slowpath(*lockp, impl_defined, saved_lr) */

	stmdb	sp!, {r2, lr}
	spinhead

#ifdef CONFIG_MSM_KRAIT_WFE_FIXUP
	/* Check whether WFE fixup is needed */
	ldr	lr, =msm_krait_need_wfe_fixup
	ldr	lr, [lr]
	teq	lr, #0
	beq	21f

	/* Check whether optimistic spinning makes sense */
11:	spinskipoptim 13f

	/* Optimistic spinning */
	mov	lr, #256
12:	subs	lr, lr, #1
	ble	13f
	dmb
	spintest	12b

	ldmia	sp!, {lr, pc}

	/* Fixup needed, set it up */
13:	mrs	lr, cpsr
	// FIQ state on entry is preserved in Z flag and/or lr #0x10000.
	tst	lr, #PSR_F_BIT
	mrc	p15, 7, lr, c15, c0, 5
#ifndef SPIN_OPTIM_AFTER_WFE
	orrne	lr, lr, #0x10000
	biceq	lr, lr, #0x10000

	// Disable FIQs during WFE
14:	tst	lr, #0x10000
#endif
	bne	15f
	cpsid	f

15:	bic	r2, lr, #0x10000
	mcr	p15, 7, r2, c15, c0, 5
	isb

	wfe

	orr	r2, r2, #0x10000
	mcr	p15, 7, r2, c15, c0, 5
	isb
#ifdef FIXUP_NEEDS_CLREX
	clrex
#endif

	// Restore FIQs if appropriate
	// Z preserved from above cpsid
	bne	18f
	cpsie	f
18:

#ifdef SPIN_OPTIM_AFTER_WFE
	spintest 11b
#else
	spintest 14b
#endif

	ldmia	sp!, {lr, pc}
#endif /* CONFIG_MSM_KRAIT_WFE_FIXUP */

	/* No fixup needed, just call WFE */
21:
#ifdef CONFIG_CPU_32v6K
	wfe
#endif
	spintest 21b

	ldmia	sp!, {lr, pc}
