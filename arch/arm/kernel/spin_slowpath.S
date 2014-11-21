#include <linux/linkage.h>
#include <asm/assembler.h>

.macro spinhead
	uxth	r1, r1
.endm

.macro spintest loop
	ldrh	r3, [r0, #2]
	teq	r3, r1
	bne	\loop
.endm

.macro spintail
.endm

ENTRY(__arch_spin_lock_slowpath)

#include "lock_template.S"

ENDPROC(__arch_spin_lock_slowpath)
