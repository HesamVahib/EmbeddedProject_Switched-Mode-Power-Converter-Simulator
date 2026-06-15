#include "semaphore.h"
#include "stm32f4xx.h"   /* for __disable_irq / __enable_irq */

/* =========================================================
 * Local critical section helpers
 * =========================================================
 * These helpers temporarily disable interrupts so that
 * shared variables can be modified safely.
 *
 * They are used to protect semaphore state against
 * race conditions between main code and interrupts.
 */
static inline void crit_enter(void)
{
    __disable_irq();
}

static inline void crit_exit(void)
{
    __enable_irq();
}

/* =========================================================
 * Binary semaphore
 * =========================================================
 * Simple lock with two states:
 *   taken = 0 -> free
 *   taken = 1 -> locked
 *
 * Used when only one owner is allowed at a time
 * (for example: UART exclusively owning CONFIG mode).
 */

/* Try to take the semaphore.
 * Returns 1 if successful, 0 if already taken.
 *
 * The check-and-set is done inside a critical section
 * so it cannot be interrupted.
 */
uint8_t bin_sem_try_take(bin_sem_t *s)
{
    uint8_t ok = 0;

    crit_enter();
    if (!s->taken) {
        s->taken = 1;
        ok = 1;
    }
    crit_exit();

    return ok;
}

/* Release the semaphore.
 * After this call, another owner can take it.
 */
void bin_sem_give(bin_sem_t *s)
{
    crit_enter();
    s->taken = 0;
    crit_exit();
}

/* Check whether the semaphore is currently taken.
 * This is a read-only helper, no critical section needed
 * since 'taken' is a single-byte value.
 */
uint8_t bin_sem_is_taken(bin_sem_t *s)
{
    return s->taken;
}

/* =========================================================
 * Timed semaphore
 * =========================================================
 * Lock that automatically expires after a given time.
 *
 * The semaphore is considered locked while ms_left > 0.
 * A 1 kHz tick function must decrement the counter.
 *
 * Used to temporarily block actions (e.g. UART parameter
 * changes after button usage).
 */

/* Lock the semaphore for a given number of milliseconds.
 * This overwrites any previous remaining time.
 */
void timed_sem_lock_ms(timed_sem_t *t, uint32_t ms)
{
    crit_enter();
    t->ms_left = ms;
    crit_exit();
}

/* Returns 1 if the semaphore is currently locked,
 * otherwise returns 0.
 */
uint8_t timed_sem_is_locked(timed_sem_t *t)
{
    return (t->ms_left > 0U);
}

/* Must be called from a 1 kHz task or timer.
 * Decrements the remaining lock time by 1 ms.
 */
void timed_sem_tick_1ms(timed_sem_t *t)
{
    if (t->ms_left == 0U) return;
    t->ms_left--;
}
