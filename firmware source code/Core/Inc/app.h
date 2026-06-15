#ifndef APP_H
#define APP_H

#include <stdint.h>

/* ===== App modes ===== */
typedef enum { MODE_CONFIG=0, MODE_IDLE=1, MODE_MOD=2 } mode_t;

/* ===== Config constants ===== */
#define TS_SEC        (0.001f)   /* 1 ms tick */
#define Y_FULL_SCALE  (1.5f)
#define STATE_LIMIT   (50.0f)
#define Y_LIMIT       (15.0f)

/* ===== Public app API ===== */
void app_init(void);
void app_tick_1khz(void);

void app_button_event(void);              /* mode cycle */
void app_cli_process_line(char *line);    /* UART CLI */

const char* mode_str(mode_t m);

void app_pwm_period_start(void);   // called at start of each PWM period (TIM2 update)
void app_pwm_pulse_end(void);      // called at end of ON-time (TIM2 CC1)


#endif
