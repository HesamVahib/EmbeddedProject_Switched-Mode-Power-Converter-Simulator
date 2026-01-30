#ifndef SEMAPHORE_H
#define SEMAPHORE_H

#include <stdint.h>

/* ===== Binary semaphore ===== */
typedef struct {
    volatile uint8_t taken;   /* 0 = free, 1 = taken */
} bin_sem_t;

/* ===== Timed semaphore (ms resolution) ===== */
typedef struct {
    volatile uint32_t ms_left;
} timed_sem_t;

/* ===== Binary semaphore API ===== */
uint8_t bin_sem_try_take(bin_sem_t *s);
void    bin_sem_give(bin_sem_t *s);
uint8_t bin_sem_is_taken(bin_sem_t *s);

/* ===== Timed semaphore API ===== */
void    timed_sem_lock_ms(timed_sem_t *t, uint32_t ms);
uint8_t timed_sem_is_locked(timed_sem_t *t);
void    timed_sem_tick_1ms(timed_sem_t *t);

#endif
