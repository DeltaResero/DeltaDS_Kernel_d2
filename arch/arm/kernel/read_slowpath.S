#include <linux/linkage.h>
#include <asm/assembler.h>

.macro spinhead
.endm

.macro spintest loop
	ldr	r1, [r0]
	cmp	r1, #0
	bmi	\loop
.endm

.macro spintail
.endm

.macro spinskipoptim skip
.endm

ENTRY(__arch_read_lock_slowpath)

#include "lock_template.S"

ENDPROC(__arch_read_lock_slowpath)
