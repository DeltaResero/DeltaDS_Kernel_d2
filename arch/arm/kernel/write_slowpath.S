#include <linux/linkage.h>
#include <asm/assembler.h>

// FIXME: str after stmdb is awful.

.macro spinhead
	str	r3, [sp, #-4]!
.endm

.macro spintest loop
	ldrex	r1, [r0]
	teq	r1, #0
	bne	\loop

	orr	r1, r1, #0x80000000
	strex	r3, r1, [r0]
	teq	r3, #0
	bne	\loop
.endm

.macro spintail
	ldr	r3, [sp], #4
.endm

.macro spinskipoptim skip
	bics	r1, r1, #0x80000001
	bne	\skip
.endm

ENTRY(__arch_write_lock_slowpath)

#include "lock_template.S"

ENDPROC(__arch_write_lock_slowpath)
