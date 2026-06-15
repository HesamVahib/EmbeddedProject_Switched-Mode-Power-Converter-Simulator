/* =========================
 * app.c  (all application logic)
 *
 * Big picture:
 * - This file owns the whole “application layer”:
 *     modes, buttons, UART CLI, control loop, plant model, PWM output, LEDs.
 *
 * Key features inside:
 * - 3 modes: IDLE / MOD / CONFIG
 * - Debounced buttons with consistent behavior
 * - UART CLI (help/status/mode/kp/ki/ref/bonus1/bonus2/freq/amp)
 * - PI controller with anti-windup
 * - Discrete plant model (A,B matrices)
 * - Two protection mechanisms (“semaphores”):
 *     1) UART can exclusively own CONFIG mode (buttons ignored if UART owns it)
 *     2) After any button action, UART parameter changes are blocked for 5 seconds
 * - Short critical sections for shared variables (avoid race conditions)
 * ========================= */

#include "app.h"
#include "main.h"
#include "semaphore.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* HAL handles are defined in main.c */
extern TIM_HandleTypeDef  htim2;
extern UART_HandleTypeDef huart2;

/* =========================================================
 * Local critical section helpers
 * =========================================================
 * These are tiny wrappers around IRQ disable/enable.
 * I use them when I update shared globals that can be touched
 * by interrupts (tick ISR, UART ISR, timer callbacks).
 *
 * Keep the critical sections short: update variable(s) then exit.
 */
static inline void crit_enter(void) { __disable_irq(); }
static inline void crit_exit(void)  { __enable_irq();  }

/* =========================================================
 * Semaphores / locks used by the app
 * =========================================================
 * g_sem_uart_cfg_owner:
 *   - Binary semaphore meaning: "UART owns CONFIG"
 *   - If taken => ALL button actions are ignored
 *
 * g_sem_uart_block:
 *   - Timed lock (ms counter)
 *   - If locked => UART parameter changes are blocked temporarily
 *     (help/status/mode still allowed)
 */
static bin_sem_t   g_sem_uart_cfg_owner = {0}; /* 1 = UART owns CONFIG */
static timed_sem_t g_sem_uart_block     = {0}; /* ms_left > 0 => UART parameter changes blocked */

/* =========================================================
 * App state (globals)
 * =========================================================
 * Marked volatile because they can change in time-critical code paths.
 */
volatile mode_t  g_mode = MODE_IDLE;
volatile uint8_t g_uart_config_lock = 0;  /* mostly for status printing / UI */

/* Signals */
volatile float g_ref    = 0.50f;  /* 0..1 reference shown in status */
volatile float g_uin    = 0.00f;  /* 0..1 (PWM duty) */
volatile float g_uout   = 0.00f;  /* plant output raw */
volatile float g_meas01 = 0.00f;  /* 0..1 filtered measurement */

/* PI controller parameters and integrator */
volatile float g_kp = 0.50f;
volatile float g_ki = 30.0f;
volatile float g_i  = 0.00f;

/* Internal measurement filter state + CONFIG selection */
static float   g_meas_f = 0.00f;
static uint8_t g_sel_param = 0; /* 0=KP, 1=KI */

/* =========================================================
 * Bonus 1: sine reference generator
 * =========================================================
 * When enabled, the reference becomes a sine wave mapped to 0..1.
 * The user can change frequency and amplitude via CLI.
 */
static volatile uint8_t g_bonus1_inverter = 0;  /* 0=off, 1=on */
static volatile float   g_inv_freq_hz = 50.0f;  /* Hz */
static volatile float   g_inv_amp01   = 0.45f;  /* 0..0.49 */
static float            g_inv_phase   = 0.0f;   /* rad */

/* =========================================================
 * Bonus 2: PWM-driven model input (plant is stepped by PWM edges)
 * =========================================================
 * When enabled:
 *  - plant is stepped at PWM start (period start) with +1 or -1
 *  - plant is stepped at pulse end with 0
 *
 * So plant input is basically: +1/0 or -1/0 depending on polarity.
 */
static volatile uint8_t g_bonus2_pwm_model = 0; /* 0=off, 1=on */
static volatile int8_t  g_pwm_polarity = +1;    /* +1 or -1 */
static volatile int8_t  g_upwm_state = 0;       /* -1, 0, +1 */

/* =========================================================
 * Plant model
 * =========================================================
 * Discrete-time state-space:
 *   x[k+1] = A x[k] + B u[k]
 * Output chosen as x[5]
 */
static float xplant[6] = {0};

static const float A[6][6] = {
  { 0.9652f, -0.0172f,  0.0057f, -0.0058f,  0.0052f, -0.0251f },
  { 0.7732f,  0.1252f,  0.2315f,  0.0700f,  0.1282f,  0.7754f },
  { 0.8278f, -0.7522f, -0.0956f,  0.3299f, -0.4855f,  0.3915f },
  { 0.9948f,  0.2655f, -0.3848f,  0.4212f,  0.3927f,  0.2899f },
  { 0.7648f, -0.4165f, -0.4855f, -0.3366f, -0.0986f,  0.7281f },
  { 1.1056f,  0.7587f, -0.1179f,  0.0748f, -0.2192f,  0.1491f },
};
static const float B[6] = { 0.0471f, 0.0377f, 0.0404f, 0.0485f, 0.0373f, 0.0539f };

/* =========================================================
 * Small helper functions
 * ========================================================= */

/* Simple float sanity check (NaN / huge values) */
static int finite_f(float v)
{
  if (v != v) return 0;                  /* NaN check: NaN != NaN */
  if (v > 1e30f || v < -1e30f) return 0; /* “Inf-ish” protection */
  return 1;
}

/* Clamp to [0,1] since many signals here are normalized */
static float clamp01(float x)
{
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

/* UART print helpers (always CRLF for lines) */
static void uart_print(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 200);
}
static void uart_println(const char *s)
{
  uart_print(s);
  uart_print("\r\n");
}

/* If the timed lock is active, UART is not allowed to change parameters */
static uint8_t uart_changes_blocked(void)
{
  return timed_sem_is_locked(&g_sem_uart_block);
}

/* =========================================================
 * PWM wrapper
 * =========================================================
 * Convert a normalized duty [0..1] into CCR value using timer ARR.
 */
static void pwm_set(float duty01)
{
  duty01 = clamp01(duty01);
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(duty01 * (float)(arr + 1U)));
}

/* =========================================================
 * LEDs
 * =========================================================
 * Only 2 pins used:
 *  - PA6 on in MOD
 *  - PA7 on in CONFIG
 *  - both off in IDLE
 */
static void leds_update_from_mode(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,
                    (g_mode == MODE_MOD) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,
                    (g_mode == MODE_CONFIG) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* =========================================================
 * Reset function
 * =========================================================
 * This is the “safe clean state” for control + plant:
 * - clears integrator, outputs, filter
 * - clears plant state vector
 * - resets bonus generator internal states
 *
 * Called when:
 * - changing modes
 * - sanity check fails
 * - initialization
 */
static void reset_control(void)
{
  g_i = 0.0f;
  g_uin  = 0.0f;
  g_uout = 0.0f;
  g_meas_f = 0.0f;
  g_meas01 = 0.0f;
  for (int k = 0; k < 6; k++) xplant[k] = 0.0f;

  g_inv_phase = 0.0f;
  g_pwm_polarity = +1;
  g_upwm_state = 0;
}

/* =========================================================
 * Plant step (discrete model)
 * =========================================================
 * Computes one update of the state vector and returns output y.
 * Includes safety:
 * - if any state becomes NaN or too large => reset plant, output 0
 * - if output becomes NaN or too large => reset plant, output 0
 */
static float plant_step(float uin)
{
  float xn[6];

  /* Compute next state */
  for (int r = 0; r < 6; r++)
  {
    float sum = 0.0f;
    for (int c = 0; c < 6; c++) sum += A[r][c] * xplant[c];
    sum += B[r] * uin;
    xn[r] = sum;
  }

  /* State safety limits */
  for (int i = 0; i < 6; i++)
  {
    if (!finite_f(xn[i]) || xn[i] > STATE_LIMIT || xn[i] < -STATE_LIMIT)
    {
      for (int k = 0; k < 6; k++) xplant[k] = 0.0f;
      return 0.0f;
    }
  }

  /* Commit the state update */
  for (int i = 0; i < 6; i++) xplant[i] = xn[i];

  /* Output is chosen as state x[5] */
  float y = xplant[5];
  if (!finite_f(y) || y > Y_LIMIT || y < -Y_LIMIT)
  {
    for (int k = 0; k < 6; k++) xplant[k] = 0.0f;
    return 0.0f;
  }
  return y;
}

/* =========================================================
 * PI control step with basic anti-windup
 * =========================================================
 * - Computes proportional part: up = kp * e
 * - Proposes new integrator: ui_new = i + ki*Ts*e
 * - Uses conditional integration:
 *     integrate only when not saturated, or when error pushes back
 * - Final output clamped to [0..1]
 */
static float pi_step(float ref, float meas01)
{
  float e  = ref - meas01;
  float up = g_kp * e;

  float ui_new = g_i + (g_ki * TS_SEC) * e;
  float u_try  = up + ui_new;

  /* Integrate if:
   * - output would be inside limits, or
   * - output saturated high but error is negative (wants to reduce), or
   * - output saturated low but error is positive (wants to increase)
   */
  if ((u_try < 1.0f && u_try > 0.0f) ||
      (u_try >= 1.0f && e < 0.0f) ||
      (u_try <= 0.0f && e > 0.0f))
  {
    g_i = ui_new;
  }

  float u = up + g_i;
  if (u > 1.0f) u = 1.0f;
  if (u < 0.0f) u = 0.0f;
  return u;
}

/* =========================================================
 * Bonus 1: sine reference mapped to 0..1
 * =========================================================
 * Output:
 *   0.5 + amp*sin(phase)
 * - Frequency limited to avoid crazy values
 * - Amp limited to max 0.49 so it can’t exceed [0..1]
 */
static float inv_ref_step_01(void)
{
  const float TWO_PI = 6.283185307f;

  float f = g_inv_freq_hz;
  if (f < 0.1f) f = 0.1f;
  if (f > 200.0f) f = 200.0f;

  float a = g_inv_amp01;
  if (a < 0.0f) a = 0.0f;
  if (a > 0.49f) a = 0.49f;

  g_inv_phase += TWO_PI * f * TS_SEC;
  if (g_inv_phase >= TWO_PI) g_inv_phase -= TWO_PI;

  float s = sinf(g_inv_phase);
  return clamp01(0.5f + a * s);
}

/* Mode name string for printing */
const char* mode_str(mode_t m)
{
  switch (m) {
    case MODE_CONFIG: return "CONFIG";
    case MODE_IDLE:   return "IDLE";
    case MODE_MOD:    return "MOD";
    default:          return "?";
  }
}

/* =========================================================
 * Robust button handling (debounced)
 * =========================================================
 * Hardware assumption:
 * - Buttons are active-low, with pull-up.
 *
 * Debounce method:
 * - Integrator counter:
 *     pressed => integrator counts up to BTN_DEB_MAX
 *     released => integrator counts down to 0
 * - stable state is updated when integrator hits extremes
 *
 * Event generation:
 * - return 1 exactly once on a press (stable falling edge)
 */
typedef struct {
  GPIO_TypeDef *port;
  uint16_t      pin;

  uint8_t stable;      /* 1=not pressed, 0=pressed (stable decision) */
  uint8_t last_stable; /* previous stable decision */
  uint8_t integrator;  /* 0..DEB_MAX */
} btn_t;

#define BTN_DEB_MS   10u
#define BTN_DEB_MAX  BTN_DEB_MS

static void btn_init(btn_t *b, GPIO_TypeDef *port, uint16_t pin)
{
  b->port = port;
  b->pin  = pin;

  /* start as “not pressed” */
  b->stable = 1;
  b->last_stable = 1;
  b->integrator = 0;
}

/* returns 1 exactly once per PRESS (stable falling edge) */
static uint8_t btn_update_press_event(btn_t *b)
{
  /* raw read: GPIO high means not pressed (pull-up), low means pressed */
  uint8_t raw = (HAL_GPIO_ReadPin(b->port, b->pin) != GPIO_PIN_RESET); /* 1=high */
  uint8_t raw_level = raw; /* active-low: pressed => 0 */

  /* Integrator debounce */
  if (raw_level == 0) {
    if (b->integrator < BTN_DEB_MAX) b->integrator++;
  } else {
    if (b->integrator > 0) b->integrator--;
  }

  /* Decide stable state from integrator extremes */
  if (b->integrator == 0)                b->stable = 1;
  else if (b->integrator >= BTN_DEB_MAX) b->stable = 0;

  /* Press event on stable transition 1 -> 0 */
  uint8_t event = 0;
  if (b->stable != b->last_stable) {
    if (b->last_stable == 1 && b->stable == 0) event = 1;
    b->last_stable = b->stable;
  }
  return event;
}

/* Buttons mapping */
static btn_t btn_mode;    /* PA0 */
static btn_t btn_inc;     /* PA1 */
static btn_t btn_dec;     /* PB0 */
static btn_t btn_select;  /* PA4 */

/* =========================================================
 * Button actions for INC/DEC
 * =========================================================
 * Behavior depends on mode:
 * - MODE_MOD:
 *     INC/DEC changes reference (unless bonus1 sine is active)
 * - MODE_CONFIG:
 *     INC/DEC changes selected parameter (KP or KI)
 *
 * For controller stability:
 * - whenever KP/KI/ref changes, reset integrator g_i to 0
 */
static void step_ref_or_gain(float d)
{
  if (g_mode == MODE_MOD)
  {
    /* If sine reference is enabled, ignore manual ref changes */
    if (g_bonus1_inverter) return;

    crit_enter();
    g_ref = clamp01(g_ref + d);
    g_i = 0.0f;
    crit_exit();

    uart_println(d > 0 ? "REF increase" : "REF decrease");
    return;
  }

  if (g_mode == MODE_CONFIG)
  {
    if (g_sel_param == 0)
    {
      /* KP change */
      crit_enter();
      g_kp += d;
      if (g_kp < 0.0f) g_kp = 0.0f;
      g_i = 0.0f;
      crit_exit();

      uart_println(d > 0 ? "KP increase" : "KP decrease");
    }
    else
    {
      /* KI change (scaled so it moves faster than KP) */
      crit_enter();
      g_ki += 50.0f * d;
      if (g_ki < 0.0f) g_ki = 0.0f;
      g_i = 0.0f;
      crit_exit();

      uart_println(d > 0 ? "KI increase" : "KI decrease");
    }
  }
}

/* =========================================================
 * Bonus 2 ISR entry points
 * =========================================================
 * These are called by the timer callbacks (TIM2).
 *
 * Idea:
 * - At PWM period start, input is +1 or -1 (based on polarity)
 * - At PWM pulse end, input becomes 0
 *
 * This makes the model react to PWM edges instead of averaged duty.
 */
void app_pwm_period_start(void)
{
  if (g_bonus2_pwm_model && (g_mode == MODE_MOD))
  {
    g_upwm_state = g_pwm_polarity;
    g_uout = plant_step((float)g_upwm_state);
  }
}

void app_pwm_pulse_end(void)
{
  if (g_bonus2_pwm_model && (g_mode == MODE_MOD))
  {
    g_upwm_state = 0;
    g_uout = plant_step(0.0f);
  }
}

/* =========================================================
 * Initialization
 * ========================================================= */
void app_init(void)
{
  reset_control();

  /* start in IDLE (safe state) */
  crit_enter();
  g_mode = MODE_IDLE;
  g_uart_config_lock = 0;
  crit_exit();

  /* UART does NOT own CONFIG at boot */
  bin_sem_give(&g_sem_uart_cfg_owner);

  /* UART is not blocked at boot */
  timed_sem_lock_ms(&g_sem_uart_block, 0);

  /* init buttons (active-low pullups) */
  btn_init(&btn_mode,   GPIOA, GPIO_PIN_0);
  btn_init(&btn_inc,    GPIOA, GPIO_PIN_1);
  btn_init(&btn_dec,    GPIOB, GPIO_PIN_0);
  btn_init(&btn_select, GPIOA, GPIO_PIN_4);

  leds_update_from_mode();
}

/* =========================================================
 * Main application tick: runs at 1 kHz
 * =========================================================
 * This is the “heartbeat” of the app.
 * It does:
 * - updates timed semaphore
 * - reads buttons and triggers actions
 * - updates LEDs
 * - sanity checks
 * - runs behavior based on mode (IDLE/MOD/CONFIG)
 */
void app_tick_1khz(void)
{
  /* tick timed semaphore (UART blocked after button actions) */
  timed_sem_tick_1ms(&g_sem_uart_block);

  /* If UART owns CONFIG, ALL buttons do nothing */
  if (bin_sem_is_taken(&g_sem_uart_cfg_owner))
  {
    /* still update debounce so events don't pile up */
    (void)btn_update_press_event(&btn_mode);
    (void)btn_update_press_event(&btn_select);
    (void)btn_update_press_event(&btn_inc);
    (void)btn_update_press_event(&btn_dec);
  }
  else
  {
    /* normal button handling */

    /* MODE button cycles modes and starts a 5s UART lock */
    if (btn_update_press_event(&btn_mode)) {
      timed_sem_lock_ms(&g_sem_uart_block, 5000);
      app_button_event();
    }

    /* SELECT toggles which parameter is edited in CONFIG */
    if (btn_update_press_event(&btn_select)) {
      timed_sem_lock_ms(&g_sem_uart_block, 5000);
      g_sel_param ^= 1;
      uart_println(g_sel_param ? "Select -> KI" : "Select -> KP");
    }

    /* INC/DEC change either ref (MOD) or gains (CONFIG) */
    if (btn_update_press_event(&btn_inc)) {
      timed_sem_lock_ms(&g_sem_uart_block, 5000);
      step_ref_or_gain(+0.05f);
    }
    if (btn_update_press_event(&btn_dec)) {
      timed_sem_lock_ms(&g_sem_uart_block, 5000);
      step_ref_or_gain(-0.05f);
    }
  }

  leds_update_from_mode();

  /* =======================================================
   * Safety / sanity check
   * =======================================================
   * If something becomes NaN/Inf or output explodes, reset.
   * This prevents the system from “running away”.
   */
  if (!finite_f(g_uout) || !finite_f(g_i) ||
      !finite_f(g_kp)  || !finite_f(g_ki) || !finite_f(g_ref) ||
      g_uout > 20.0f || g_uout < -20.0f)
  {
    reset_control();
    pwm_set(0.0f);
    return;
  }

  /* =======================================================
   * MODE_IDLE behavior
   * =======================================================
   * - PWM off
   * - measurement forced to zero
   * - if bonus2 is off, plant still advances with u=0 (optional behavior)
   */
  if (g_mode == MODE_IDLE)
  {
    g_uin = 0.0f;
    pwm_set(0.0f);

    if (!g_bonus2_pwm_model) g_uout = plant_step(0.0f);

    g_meas01 = 0.0f;
    g_upwm_state = 0;
    return;
  }

  /* =======================================================
   * MODE_MOD behavior (closed-loop control)
   * =======================================================
   * Steps:
   * 1) map plant output to normalized measurement 0..1
   * 2) low-pass filter measurement
   * 3) choose reference (manual or sine)
   * 4) choose polarity for bonus2 (+1/-1 depending on ref)
   * 5) run PI controller to get duty
   * 6) set PWM
   * 7) update plant:
   *      - normal: averaged input u
   *      - bonus2: plant updated inside PWM callbacks
   */
  if (g_mode == MODE_MOD)
  {
    float meas01;

    if (g_bonus1_inverter)
    {
      /* signed mapping [-Y_FULL_SCALE..+Y_FULL_SCALE] -> [0..1] */
      float y = g_uout;
      meas01 = clamp01(0.5f + 0.5f * (y / Y_FULL_SCALE));
    }
    else
    {
      /* magnitude mapping: |y| limited then scaled to [0..1] */
      float y = g_uout;
      if (y < 0.0f) y = -y;
      if (y > Y_FULL_SCALE) y = Y_FULL_SCALE;
      meas01 = clamp01(y / Y_FULL_SCALE);
    }

    /* 1st-order low-pass filter to smooth measurement */
    const float alpha = 0.02f;
    g_meas_f += alpha * (meas01 - g_meas_f);
    g_meas01 = g_meas_f;

    /* reference: either fixed g_ref or sine output */
    float ref01 = g_ref;
    if (g_bonus1_inverter) {
      ref01 = inv_ref_step_01();
      g_ref = ref01; /* store so status shows current sine value */
    }

    /* polarity decision for bonus2:
     * this just chooses +1 or -1 based on whether ref is above or below 0.5
     */
    g_pwm_polarity = (ref01 >= 0.5f) ? +1 : -1;

    /* PI controller generates duty 0..1 */
    float u = pi_step(ref01, g_meas_f);
    g_uin = u;
    pwm_set(u);

    /* If bonus2 is OFF, plant uses averaged duty directly */
    if (!g_bonus2_pwm_model) {
      g_uout = plant_step(u);
    }
    return;
  }

  /* =======================================================
   * MODE_CONFIG behavior
   * =======================================================
   * - apply a small fixed duty (0.10) so output is “alive”
   * - measurement is magnitude-based, normalized
   * - user can edit KP/KI using INC/DEC and SELECT
   */
  {
    const float u = 0.10f;
    g_uin = u;
    pwm_set(u);

    if (!g_bonus2_pwm_model) {
      g_uout = plant_step(u);
    }

    float y = g_uout;
    if (y < 0.0f) y = -y;
    if (y > Y_FULL_SCALE) y = Y_FULL_SCALE;
    g_meas01 = clamp01(y / Y_FULL_SCALE);
  }
}

/* =========================================================
 * Mode button cycle:
 *   IDLE -> MOD -> CONFIG -> IDLE
 *
 * Important detail:
 * - If UART owns CONFIG, we ignore all button actions.
 * - If CONFIG was entered by button, UART does NOT own it,
 *   so buttons keep working normally.
 * ========================================================= */
void app_button_event(void)
{
  /* If UART owns CONFIG => ignore ALL button actions */
  if (bin_sem_is_taken(&g_sem_uart_cfg_owner)) {
    return;
  }

  /* Mode change is a “hard reset” of control/plant so it starts clean */
  crit_enter();
  reset_control();

  if (g_mode == MODE_IDLE) {
    g_mode = MODE_MOD;
    g_uart_config_lock = 0;
    crit_exit();
    uart_println("Mode -> MOD");
  }
  else if (g_mode == MODE_MOD) {
    g_mode = MODE_CONFIG;
    g_uart_config_lock = 1;
    crit_exit();
    uart_println("Mode -> CONFIG");
  }
  else {
    g_mode = MODE_IDLE;
    g_uart_config_lock = 0;
    crit_exit();
    uart_println("Mode -> IDLE");
  }

  leds_update_from_mode();
}

/* =========================================================
 * UART CLI: parse and handle one input line
 * =========================================================
 * Format examples:
 *   help
 *   status
 *   mode idle / mode mod / mode cfg
 *   kp 0.6
 *   ki 20
 *   ref 0.7
 *   bonus1 on/off
 *   freq 50
 *   amp 0.3
 *   bonus2 on/off
 *
 * Two protections here:
 * - Mode commands always allowed (even during timed UART lock)
 * - Parameter commands blocked while timed lock is active
 */
void app_cli_process_line(char *line)
{
  /* trim leading spaces */
  while (*line == ' ') line++;

  /* trim trailing newline/spaces */
  size_t n = strlen(line);
  while (n && (line[n-1] == '\r' || line[n-1] == '\n' || line[n-1] == ' '))
    line[--n] = 0;

  if (*line == '\0') return;

  /* quick help */
  if (strcmp(line, "help") == 0) {
    uart_println("help | status | mode idle|mod|cfg | kp <f> | ki <f> | ref <f> | bonus1 on|off | freq <f> | amp <f> | bonus2 on|off");
    return;
  }

  /* status print: snapshot all values atomically */
  if (strcmp(line, "status") == 0)
  {
    __disable_irq();
    mode_t  m    = g_mode;
    uint8_t lock = g_uart_config_lock;
    float kp = g_kp, ki = g_ki, ref = g_ref, meas = g_meas01, uin = g_uin, uout = g_uout;
    uint8_t b1 = g_bonus1_inverter;
    uint8_t b2 = g_bonus2_pwm_model;
    float f = g_inv_freq_hz, a = g_inv_amp01;
    int upwm = (int)g_upwm_state;
    int pol  = (int)g_pwm_polarity;
    __enable_irq();

    char buf[260];
    snprintf(buf, sizeof(buf),
      "mode=%s lock=%u kp=%.3f ki=%.3f ref=%.3f meas=%.3f uin=%.3f uout=%.3f bonus1=%u f=%.1f amp=%.2f bonus2=%u upwm=%d pol=%d",
      mode_str(m), (unsigned)lock,
      (double)kp, (double)ki, (double)ref, (double)meas, (double)uin, (double)uout,
      (unsigned)b1, (double)f, (double)a,
      (unsigned)b2, upwm, pol);
    uart_println(buf);
    return;
  }

  /* =======================================================
   * MODE commands (always allowed)
   * ======================================================= */
  if (strncmp(line, "mode", 4) == 0)
  {
    char *arg = line + 4; while (*arg == ' ') arg++;

    /* entering CONFIG via UART */
    if (strncmp(arg, "cfg", 3) == 0)
    {
      /* prevent “double entering” cfg (keeps ownership logic clean) */
      if (g_mode == MODE_CONFIG)
      {
        if (bin_sem_is_taken(&g_sem_uart_cfg_owner)) uart_println("ERR cfg busy");
        else                                         uart_println("ERR already cfg");
        return;
      }

      /* take UART ownership (exclusive) */
      if (!bin_sem_try_take(&g_sem_uart_cfg_owner)) {
        uart_println("ERR cfg busy");
        return;
      }

      crit_enter();
      reset_control();
      g_mode = MODE_CONFIG;
      g_uart_config_lock = 1;
      crit_exit();

      uart_println("Mode -> CONFIG");
      leds_update_from_mode();
      return;
    }

    /* for idle/mod: reset then set mode, and release UART ownership if needed */
    crit_enter();
    reset_control();
    crit_exit();

    if (strncmp(arg, "idle", 4) == 0) {
      crit_enter();
      g_mode = MODE_IDLE;
      g_uart_config_lock = 0;
      crit_exit();

      /* if UART owned CONFIG, release it when leaving */
      if (bin_sem_is_taken(&g_sem_uart_cfg_owner)) {
        bin_sem_give(&g_sem_uart_cfg_owner);
      }

      uart_println("Mode -> IDLE");
      leds_update_from_mode();
      return;
    }

    if (strncmp(arg, "mod", 3) == 0) {
      crit_enter();
      g_mode = MODE_MOD;
      g_uart_config_lock = 0;
      crit_exit();

      /* if UART owned CONFIG, release it when leaving */
      if (bin_sem_is_taken(&g_sem_uart_cfg_owner)) {
        bin_sem_give(&g_sem_uart_cfg_owner);
      }

      uart_println("Mode -> MOD");
      leds_update_from_mode();
      return;
    }

    uart_println("ERR mode arg");
    return;
  }

  /* =======================================================
   * Timed lock check:
   * After buttons are used, UART cannot change parameters for 5s.
   * ======================================================= */
  if (uart_changes_blocked())
  {
    uart_println("ERR uart locked");
    return;
  }

  /* =======================================================
   * Parameter / bonus commands
   * ======================================================= */

  if (strncmp(line, "bonus2", 6) == 0)
  {
    char *arg = line + 6; while (*arg == ' ') arg++;
    if (strncmp(arg, "on", 2) == 0)  { crit_enter(); g_bonus2_pwm_model = 1; crit_exit(); uart_println("OK bonus2 on"); return; }
    if (strncmp(arg, "off", 3) == 0) { crit_enter(); g_bonus2_pwm_model = 0; crit_exit(); uart_println("OK bonus2 off"); return; }
    uart_println("ERR bonus2 on|off");
    return;
  }

  if (strncmp(line, "bonus1", 6) == 0)
  {
    char *arg = line + 6; while (*arg == ' ') arg++;
    if (strncmp(arg, "on", 2) == 0)  { crit_enter(); g_bonus1_inverter = 1; crit_exit(); uart_println("OK bonus1 on"); return; }
    if (strncmp(arg, "off", 3) == 0) { crit_enter(); g_bonus1_inverter = 0; crit_exit(); uart_println("OK bonus1 off"); return; }
    uart_println("ERR bonus1 on|off");
    return;
  }

  if (strncmp(line, "freq", 4) == 0)
  {
    char *arg = line + 4; while (*arg == ' ') arg++;
    if (*arg == '\0') { uart_println("ERR missing value"); return; }
    crit_enter();
    g_inv_freq_hz = (float)atof(arg);
    crit_exit();
    uart_println("OK freq");
    return;
  }

  if (strncmp(line, "amp", 3) == 0)
  {
    char *arg = line + 3; while (*arg == ' ') arg++;
    if (*arg == '\0') { uart_println("ERR missing value"); return; }
    float v = (float)atof(arg);
    if (v < 0.0f) v = 0.0f;
    if (v > 0.49f) v = 0.49f;

    crit_enter();
    g_inv_amp01 = v;
    crit_exit();

    uart_println("OK amp");
    return;
  }

  if (strncmp(line, "kp", 2) == 0)
  {
    char *arg = line + 2; while (*arg == ' ') arg++;
    if (*arg == '\0') { uart_println("ERR missing value"); return; }
    crit_enter();
    g_kp = (float)atof(arg);
    g_i = 0.0f; /* reset integrator after changing gain */
    crit_exit();
    uart_println("OK kp");
    return;
  }

  if (strncmp(line, "ki", 2) == 0)
  {
    char *arg = line + 2; while (*arg == ' ') arg++;
    if (*arg == '\0') { uart_println("ERR missing value"); return; }
    crit_enter();
    g_ki = (float)atof(arg);
    g_i = 0.0f; /* reset integrator after changing gain */
    crit_exit();
    uart_println("OK ki");
    return;
  }

  if (strncmp(line, "ref", 3) == 0)
  {
    char *arg = line + 3; while (*arg == ' ') arg++;
    if (*arg == '\0') { uart_println("ERR missing value"); return; }
    crit_enter();
    g_ref = clamp01((float)atof(arg));
    g_i = 0.0f; /* reset integrator after changing reference */
    crit_exit();
    uart_println("OK ref");
    return;
  }

  /* fallback for unknown command */
  uart_println("ERR unknown (type help)");
}
