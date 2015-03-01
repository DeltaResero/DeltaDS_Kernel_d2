#include <linux/linkage.h>
#include <asm/assembler.h>

.macro spinhead
	eor	r2, r1, r1, ror #16 // Prepare r2 for spinskipoptim
	uxth	r1, r1
.endm

.macro spintest loop
	ldrh	r3, [r0, #2]
	teq	r3, r1
	bne	\loop
.endm

.macro spintail
.endm

.macro spinskipoptim skip
	cmp	r2, #0x20000 // If we're next, r2 == 0x10001
	bhi	\skip
.endm

ENTRY(__arch_spin_lock_slowpath)

#include "lock_template.S"

ENDPROC(__arch_spin_lock_slowpath)
