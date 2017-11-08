#ifndef __ASM_SPINLOCK_H
#define __ASM_SPINLOCK_H

#if __LINUX_ARM_ARCH__ < 6
#error SMP not supported on pre-ARMv6 CPUs
#endif

#include <asm/processor.h>

#ifdef CONFIG_MSM_KRAIT_WFE_FIXUP
extern int msm_krait_need_wfe_fixup;
#endif

/*
 * sev and wfe are ARMv6K extensions.  Uniprocessor ARMv6 may not have the K
 * extensions, so when running on UP, we have to patch these instructions away.
 */
#define ALT_SMP(smp, up)					\
	"9998:	" smp "\n"					\
	"	.pushsection \".alt.smp.init\", \"a\"\n"	\
	"	.long	9998b\n"				\
	"	" up "\n"					\
	"	.popsection\n"

#ifdef CONFIG_THUMB2_KERNEL
#define SEV		ALT_SMP("sev.w", "nop.w")
/*
 * Both instructions given to the ALT_SMP macro need to be the same size, to
 * allow the SMP_ON_UP fixups to function correctly. Hence the explicit encoding
 * specifications.
 */
#define WFE()		ALT_SMP(		\
	"wfe.w",				\
	"nop.w"					\
)
#else
#define SEV		ALT_SMP("sev", "nop")
#define WFE()		ALT_SMP("wfe", "nop")
#endif

/*
 * The fixup involves disabling FIQs during execution of the WFE instruction.
 * This could potentially lead to deadlock if a thread is trying to acquire a
 * spinlock which is being released from an FIQ. This should not be a problem
 * because FIQs are handled by the secure environment and do not directly
 * manipulate spinlocks.
 */
#ifdef CONFIG_MSM_KRAIT_WFE_FIXUP
#define WFE_SAFE(fixup, tmp) 				\
"	mrs	" tmp ", cpsr\n"			\
"	cmp	" fixup ", #0\n"			\
"	wfeeq\n"					\
"	beq	10f\n"					\
"	cpsid   f\n"					\
"	mrc	p15, 7, " fixup ", c15, c0, 5\n"	\
"	bic	" fixup ", " fixup ", #0x10000\n"	\
"	mcr	p15, 7, " fixup ", c15, c0, 5\n"	\
"	isb\n"						\
"	wfe\n"						\
"	orr	" fixup ", " fixup ", #0x10000\n"	\
"	mcr	p15, 7, " fixup ", c15, c0, 5\n"	\
"	isb\n"						\
"10:	msr	cpsr_cf, " tmp "\n"
#else
#define WFE_SAFE(fixup, tmp)	"	wfe\n"
#endif

static inline void dsb_sev(void)
{
	dsb();
	sev();
}

static inline void wfe_safe(void)
{
#ifdef CONFIG_MSM_KRAIT_WFE_FIXUP
	unsigned long tmp, fixup = msm_krait_need_wfe_fixup;
	__asm__ __volatile__ (
		WFE_SAFE("%0", "%1")
		: "+r" (fixup), "=&r" (tmp)
		: : "memory");
#else
	wfe();
#endif
}

#ifndef CONFIG_ARM_TICKET_LOCKS
/*
 * ARMv6 Spin-locking.
 *
 * We exclusively read the old value.  If it is zero, we may have
 * won the lock, so we try exclusively storing it.  A memory barrier
 * is required after we get a lock, and before we release it, because
 * V6 CPUs are assumed to have weakly ordered memory.
 *
 * Unlocked value: 0
 * Locked value: 1
 */

#define arch_spin_is_locked(x)		((x)->lock != 0)
#define arch_spin_unlock_wait(lock) \
	do { while (arch_spin_is_locked(lock)) cpu_relax(); } while (0)

#define arch_spin_lock_flags(lock, flags) arch_spin_lock(lock)

static inline void arch_spin_lock(arch_spinlock_t *lock)
{
	unsigned long tmp, fixup = msm_krait_need_wfe_fixup;

	__asm__ __volatile__(
"1:	ldrex	%[tmp], [%[lock]]\n"
"	teq	%[tmp], #0\n"
"	beq	2f\n"
	WFE_SAFE("%[fixup]", "%[tmp]")
"2:\n"
"	strexeq	%[tmp], %[bit0], [%[lock]]\n"
"	teqeq	%[tmp], #0\n"
"	bne	1b"
	: [tmp] "=&r" (tmp), [fixup] "+r" (fixup)
	: [lock] "r" (&lock->lock), [bit0] "r" (1)
	: "cc");

	smp_mb();
}

static inline int arch_spin_trylock(arch_spinlock_t *lock)
{
	unsigned long tmp;

	__asm__ __volatile__(
"	ldrex	%0, [%1]\n"
"	teq	%0, #0\n"
"	strexeq	%0, %2, [%1]"
	: "=&r" (tmp)
	: "r" (&lock->lock), "r" (1)
	: "cc");

	if (tmp == 0) {
		smp_mb();
		return 1;
	} else {
		return 0;
	}
}

static inline void arch_spin_unlock(arch_spinlock_t *lock)
{
	smp_mb();

	__asm__ __volatile__(
"	str	%1, [%0]\n"
	:
	: "r" (&lock->lock), "r" (0)
	: "cc");

	dsb_sev();
}
#else
/*
 * ARM Ticket spin-locking
 *
 * Ticket locks are conceptually two parts, one indicating the current head of
 * the queue, and the other indicating the current tail. The lock is acquired
 * by atomically noting the tail and incrementing it by one (thus adding
 * ourself to the queue and noting our position), then waiting until the head
 * becomes equal to the the initial value of the tail.
 *
 * 0xffff0000: next_ticket
 * 0x0000ffff: now_serving
 *
 * Unlocked value: now_serving == next_ticket
 * Locked value: now_serving != next_ticket
 */

#define arch_spin_lock_flags(lock, flags) arch_spin_lock(lock)

static inline void arch_spin_lock(arch_spinlock_t *lock)
{
	register volatile unsigned int *lockp asm ("r0") = &lock->lock;
	__asm__ __volatile__(
"1:	ldrex	r1, [r0]\n"
"	add	r1, r1, #0x10000\n"
"	strex	r2, r1, [r0]\n"

"	sub	r1, r1, #0x10000\n"
"	teq	r2, #0\n"
"	bne	1b\n"

"	teq	r1, r1, ror #16\n"
"	mov	r2, lr\n"
"	blne	__arch_spin_lock_slowpath\n"
	:
	: "r" (lockp)
	: "r1", "r2", "cc");

	smp_mb();
}

static inline int arch_spin_trylock(arch_spinlock_t *lock)
{
	register unsigned long ticket, tmp;

	/* Grab lock if now_serving == next_ticket and access is exclusive */
	__asm__ __volatile__(
"	ldrex	%[ticket], [%[lockaddr]]\n"
"	eors	%[tmp], %[ticket], %[ticket], ror #16\n"
"	add	%[ticket], %[ticket], #0x10000\n"
"	bne	1f\n"
"	strex	%[tmp], %[ticket], [%[lockaddr]]\n"
"1:\n"
	: [ticket]"=&r" (ticket), [tmp]"=&r" (tmp)
	: [lockaddr]"r" (&lock->lock)
	: "cc");

	if (!tmp)
		smp_mb();
	return !tmp;
}

static inline void arch_spin_unlock(arch_spinlock_t *lock)
{
	register unsigned long ticket, tmp = 1;

	smp_mb();

	__asm__ __volatile__(
"1:	ldrex	%[ticket], [%[lockaddr]]\n"
"	uadd16	%[ticket], %[ticket], %[tmp]\n"
"	strex	%[tmp], %[ticket], [%[lockaddr]]\n"
"	teq	%[tmp], #0\n"
"	bne	1b\n"
	: [ticket]"=&r" (ticket), [tmp]"+r" (tmp)
	: [lockaddr]"r" (&lock->lock)
	: "cc");

	if (ticket ^ ror32(ticket, 16))
		dsb_sev();
}

static inline int arch_spin_is_locked(arch_spinlock_t *lock)
{
	unsigned long tmp = ACCESS_ONCE(lock->lock);
	return !!(tmp ^ ror32(tmp, 16));
}

static inline int arch_spin_is_contended(arch_spinlock_t *lock)
{
	unsigned long tmp = ACCESS_ONCE(lock->lock);
	return (tmp - (tmp << 16)) >= 0x20000;
}

static inline void arch_spin_unlock_wait(arch_spinlock_t *lock)
{
	// We can't safely take the lock, so we can't get a SEV.
	while (arch_spin_is_locked(lock))
		cpu_relax();
}

#endif

/*
 * RWLOCKS
 *
 * rwlocks are three conceptual parts: a single bit indicating an active write
 * lock, a 15-bit counter for waiting writers, and a 16-bit counter for readers
 * (active or waiting).  This allows the dsb_sev() to be conditionalized,
 * reducing wakeups across locks.
 *
 * 0xfffe0000: writer counter
 * 0x00010000: write-lock bit
 * 0x0000ffff: reader counter
 *
 * Acquiring a read lock is simple: increment the reader counter; if the
 * write-lock bit is set, spin until it is clear.  To unlock, decrement the
 * reader counter.
 *
 * Acquiring a write lock is slightly more complex: if the reader counter is
 * zero and the write-lock bit is clear, set the write-lock bit.  Otherwise,
 * increment the writer counter, spin until the write-lock bit and the reader
 * counter are zero, then decrement the writer counter and set the write-lock
 * bit.  To unlock, clear the write-lock bit.
 */

static inline void arch_write_lock(arch_rwlock_t *rw)
{
	register volatile unsigned int *lockp asm ("r0") = &rw->lock;
	__asm__ __volatile__(
"1:	ldrex	r1, [r0]\n"
"	lsls	r2, r1, #15\n"
"	orreq	r1, r1, #0x10000\n"
"	addne	r1, r1, #0x20000\n"
"	strex	r2, r1, [r0]\n"

"	orreq	r2, r2, #2\n"
"	lsrs	r2, r2, #1\n"
"	bcs	1b\n"
"	mov	r2, lr\n"
"	bleq	__arch_write_lock_slowpath\n"
	:
	: "r" (lockp)
	: "r1", "r2", "cc");

	smp_mb();
}

static inline int arch_write_trylock(arch_rwlock_t *rw)
{
	unsigned long tmp, tmp2;

	__asm__ __volatile__(
"1:	ldrex	%1, [%2]\n"
"	lsls	%0, %1, #15\n"
"	bne	5f\n"

"	orr	%1, %1, #0x10000\n"
"	strex	%0, %1, [%2]\n"

"5:\n"
	: "=&r" (tmp), "=&r" (tmp2)
	: "r" (&rw->lock)
	: "cc");

	if (!tmp)
		smp_mb();
	return !tmp;
}

static inline void arch_write_unlock(arch_rwlock_t *rw)
{
	unsigned long tmp, tmp2;

	smp_mb();

	__asm__ __volatile__(
"1:	ldrex	%0, [%2]\n"
"	bic	%0, %0, #0x10000\n"
"	strex	%1, %0, [%2]\n"
"	teq	%1, #0\n"
"	bne	1b\n"
	: "=&r" (tmp), "=&r" (tmp2)
	: "r" (&rw->lock)
	: "cc");

	// dsb_sev() if there are other readers or writers waiting
	if (tmp)
		dsb_sev();
}

/* write_can_lock - would write_trylock() succeed? */
#define arch_write_can_lock(x)			(!((x)->lock << 15))

static inline void arch_read_lock(arch_rwlock_t *rw)
{
	register volatile unsigned int *lockp asm ("r0") = &rw->lock;
	__asm__ __volatile__(
"1:	ldrex	r1, [r0]\n"
"	add	r1, r1, #1\n"
"	strex	r2, r1, [r0]\n"
"	teq	r2, #0\n"
"	bne	1b\n"

"	tst	r1, #0x10000\n"
"	mov	r2, lr\n"
"	blne	__arch_read_lock_slowpath\n"
	:
	: "r" (lockp)
	: "r1", "r2", "cc");

	smp_mb();
}

static inline void arch_read_unlock(arch_rwlock_t *rw)
{
	unsigned long tmp, tmp2;

	smp_mb();

	__asm__ __volatile__(
"1:	ldrex	%0, [%2]\n"
"	sub	%0, %0, #1\n"
"	strex	%1, %0, [%2]\n"
"	teq	%1, #0\n"
"	bne	1b"
	: "=&r" (tmp), "=&r" (tmp2)
	: "r" (&rw->lock)
	: "cc");

	// dsb_sev() if we were the last reader and there are writers waiting
	if (!(tmp << 16) && (tmp >> 17))
		dsb_sev();
}

static inline int arch_read_trylock(arch_rwlock_t *rw)
{
	unsigned long tmp, tmp2;

	__asm__ __volatile__(
"1:	ldrex	%0, [%2]\n"
"	ands	%1, %0, #0x10000\n"
"	add	%0, %0, #1\n"
"	strexeq	%1, %0, [%2]\n"
	: "=&r" (tmp), "=&r" (tmp2)
	: "r" (&rw->lock)
	: "cc");

	if (!tmp2)
		smp_mb();
	return !tmp2;
}

/* read_can_lock - would read_trylock() succeed? */
#define arch_read_can_lock(x)		(!((x)->lock & 0x10000))

#define arch_read_lock_flags(lock, flags) arch_read_lock(lock)
#define arch_write_lock_flags(lock, flags) arch_write_lock(lock)

#define arch_spin_relax(lock)	cpu_relax()
#define arch_read_relax(lock)	cpu_relax()
#define arch_write_relax(lock)	cpu_relax()

#endif /* __ASM_SPINLOCK_H */
