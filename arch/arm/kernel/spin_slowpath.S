#include <linux/linkage.h>
#include <asm/assembler.h>

#define SPIN_OPTIM_AFTER_WFE

.macro spinhead
#ifdef CONFIG_MSM_KRAIT_WFE_FIXUP
	mov	r2, r1
#endif
	bfc	r1, #0, #16
.endm

.macro spintest loop
	ldr	r2, [r0]
	teq	r1, r2, lsl #16
	bne	\loop
.endm

.macro spinskipoptim skip
	//ldr	r2, [r0] // preserved from spinhead/spintest
	sub	r2, r1, r2, lsl #16
	cmp	r2, #0x10000
	bgt	\skip
.endm

ENTRY(__arch_spin_lock_slowpath)

#include "lock_template.S"

ENDPROC(__arch_spin_lock_slowpath)
