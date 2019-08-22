#include <linux/linkage.h>
#include <asm/assembler.h>

// Appears harmful
//#define SPIN_OPTIM_AFTER_WFE

.macro spinhead
.endm

#define FIXUP_NEEDS_CLREX
.macro spintest loop
101:	ldrex	r1, [r0]
	lsls	r2, r1, #15
	bne	\loop
	sub	r1, r1, #0x10000
	strex	r2, r1, [r0]
	teq	r2, #0
#ifdef SPIN_OPTIM_AFTER_WFE
	add	r1, r1, #0x10000
#endif
	/* If the lock was previously unlocked but our store failed, someone
	 * else probably acquired it first.  However, if we were e.g. scheduled
	 * out between the ldrex and strex, the store will also fail, even
	 * though the lock is still unlocked.  Retry to avoid hanging.
	 */
	bne	101b
.endm

.macro spinskipoptim skip
	//ldr	r1, [r0] // preserved from fastpath/spintest
	lsr	r1, r1, #1
	lsls	r1, r1, #17
	bne	\skip
.endm

ENTRY(__arch_write_lock_slowpath)

#include "lock_template.S"

ENDPROC(__arch_write_lock_slowpath)
