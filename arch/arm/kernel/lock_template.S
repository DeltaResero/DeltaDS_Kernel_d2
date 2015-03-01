/*
 * arch/arm/kernel/lock_template.S
 * Generic spinlock slowpath implementation
 *
 * Copyright (C) 2014, Ryan Pennucci <decimalman@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

/* __arch_XXX_lock_slowpath(*lockp, unused, saved_lr) */

#ifndef CONFIG_MSM_KRAIT_WFE_FIXUP

	stmdb	sp!, {r2, lr}
	spinhead

1:
#ifdef CONFIG_CPU_32v6K
	wfe
#endif
	spintest 1b

	spintail
	ldmia	sp!, {lr, pc}

#else /* CONFIG_MSM_KRAIT_WFE_FIXUP */

	stmdb	sp!, {r2, lr}
	spinhead

	/* Check whether optimistic spinning makes sense */
	spinskipoptim 2f

	/* Optimistic spinning */
	mov	r2, #256
1:	subs	r2, r2, #1
	ble	2f
	dmb
	spintest	1b

	spintail
	ldmia	sp!, {lr, pc}

	/* Check whether WFE fixup is needed */
2:	ldr	r2, =msm_krait_need_wfe_fixup
	ldr	r2, [r2]
	teq	r2, #0
	beq	21f
	b	12f

	/* Fixup needed, set it up */
11:	mrs	r2, cpsr
	cpsid	f
	mrc	p15, 7, lr, c15, c0, 5
	bic	lr, lr, #0x10000
	mcr	p15, 7, lr, c15, c0, 5
	isb

	wfe

	orr	lr, lr, #0x10000
	mcr	p15, 7, lr, c15, c0, 5
	isb
	msr	cpsr_cf, r2

12:	spintest 11b

	spintail
	ldmia	sp!, {lr, pc}

	/* No fixup needed, just call WFE */
21:	wfe
	spintest 21b

	spintail
	ldmia	sp!, {lr, pc}

#endif /* CONFIG_MSM_KRAIT_WFE_FIXUP */
