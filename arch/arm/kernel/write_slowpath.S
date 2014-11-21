#include <linux/linkage.h>
#include <asm/assembler.h>

.macro spinhead
.endm

.macro spintest loop
	ldr	r1, [r0]
	teq	r1, #0
	bne	\loop
.endm

.macro spintail
.endm

ENTRY(__arch_write_lock_slowpath)

#include "lock_template.S"

ENDPROC(__arch_write_lock_slowpath)
