#include "motor.h"
#include "board_pins.h"
#include "main.h"

#include <stddef.h>

/* PID defaults.
 *
 * The Coding Day 2 values (3.0 / 0.15 / 0) were carried over untuned; on THIS
 * chassis they were measured on 2026-11-17 to be unusable, and the reason is
 * arithmetic, not taste:
 *
 *   plant (loaded, measured): 168 duty counts -> 17 rpm  =>  ~0.1 rpm per count
 *   P-only equilibrium     : speed = 0.1 * kp * (target - speed)
 *                            with kp = 3 that is 23 % of the target speed
 *                            (measured: 57 mm/s for a 200 mm/s command = 28 %)
 *
 * A P controller alone can never hold this plant at the setpoint; the integral
 * term has to supply the steady-state duty.  It could not, because
 * MOTOR_INTEGRAL_LIMIT was 120 and ki 0.15, i.e. a maximum contribution of
 * 18 counts out of 840 - the integrator was effectively disabled.
 *
 * New values: keep P for the fast response, give the integrator enough headroom
 * to supply the full duty range (ki * limit with limit 4000 covers 840 counts
 * with margin), and let the output clamp do the anti-windup that matters.
 * Still to be tuned further under load - this is a first-order fix based on the
 * measurement above, not a validated tuning. */
#define MOTOR_KP_DEFAULT          5.0f
#define MOTOR_KI_DEFAULT          0.50f
#define MOTOR_KD_DEFAULT          0.0f
#define MOTOR_INTEGRAL_LIMIT      4000.0f
#define MOTOR_TARGET_RAMP_STEP    4
#define MOTOR_LEFT_SCALE          1.08f
#define MOTOR_RIGHT_SCALE         1.0f

static motor_handle_t s_motor;

/* Describes the left-wheel PWM timer for the calibration helper.  Registers are
 * driven directly, so only Instance/Init.Period/Init.Prescaler are meaningful. */
static TIM_HandleTypeDef s_htim_motor_l = {0};

/* ------------------------------------------------------------------------- */
/* small helpers                                                             */
/* ------------------------------------------------------------------------- */
static int16_t clamp_s16(int32_t v, int16_t lo, int16_t hi)
{
  if (v < (int32_t)lo)
  {
    return lo;
  }
  if (v > (int32_t)hi)
  {
    return hi;
  }
  return (int16_t)v;
}

static int16_t round_f_to_s16(float v)
{
  return (int16_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

static uint16_t abs_s16(int16_t v)
{
  return (uint16_t)((v >= 0) ? v : -(int32_t)v);
}

static float clamp_f(float v, float lo, float hi)
{
  if (v < lo)
  {
    return lo;
  }
  if (v > hi)
  {
    return hi;
  }
  return v;
}

static int16_t ramp_step_towards(int16_t cur, int16_t target, int16_t step)
{
  if (cur < target)
  {
    cur = (int16_t)(cur + step);
    if (cur > target)
    {
      cur = target;
    }
  }
  else if (cur > target)
  {
    cur = (int16_t)(cur - step);
    if (cur < target)
    {
      cur = target;
    }
  }
  return cur;
}

/* ------------------------------------------------------------------------- */
/* PID                                                                       */
/* ------------------------------------------------------------------------- */
static void pid_reset(motor_pid_t *pid)
{
  pid->integral   = 0.0f;
  pid->last_error = 0.0f;
  pid->output     = 0.0f;
}

static void pid_init(motor_pid_t *pid, float kp, float ki, float kd)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid_reset(pid);
}

static int16_t pid_step(motor_pid_t *pid, int16_t target, int16_t feedback,
                        int16_t pwm_limit)
{
  float error;
  float derivative;
  float output;

  /* A zero target is a stop request: drop the integrator so the next command
   * does not start from stale windup. */
  if (target == 0)
  {
    pid_reset(pid);
    return 0;
  }

  error = (float)target - (float)feedback;
  pid->integral = clamp_f(pid->integral + error,
                          -MOTOR_INTEGRAL_LIMIT, MOTOR_INTEGRAL_LIMIT);
  derivative = error - pid->last_error;
  output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
  output = clamp_f(output, (float)(-pwm_limit), (float)pwm_limit);

  pid->last_error = error;
  pid->output = output;

  return clamp_s16(round_f_to_s16(output), (int16_t)(-pwm_limit), pwm_limit);
}

/* ------------------------------------------------------------------------- */
/* H-bridge output                                                           */
/* ------------------------------------------------------------------------- */
/* Write one wheel's two H-bridge inputs.  Only the active input is PWM'd and
 * the inactive one is forced to 0 first, so the bridge is never briefly
 * commanded to short the supply (CODING_STANDARDS 3.3). */
static void apply_wheel(TIM_TypeDef *tim, uint8_t fwd_ch, uint8_t rev_ch,
                        int16_t pwm, int16_t pwm_limit)
{
  int16_t clamped = clamp_s16(pwm, (int16_t)(-pwm_limit), pwm_limit);
  uint16_t duty;

  if (clamped == 0)
  {
    if (fwd_ch == 1U) { tim->CCR1 = 0U; } else { tim->CCR2 = 0U; }
    if (rev_ch == 1U) { tim->CCR1 = 0U; } else { tim->CCR2 = 0U; }
    return;
  }

  duty = (uint16_t)(((uint32_t)abs_s16(clamped) * BOARD_MOTOR_PWM_PERIOD +
                     (uint16_t)pwm_limit / 2U) / (uint16_t)pwm_limit);

  if (clamped > 0)
  {
    if (rev_ch == 1U) { tim->CCR1 = 0U; } else { tim->CCR2 = 0U; }
    if (fwd_ch == 1U) { tim->CCR1 = duty; } else { tim->CCR2 = duty; }
  }
  else
  {
    if (fwd_ch == 1U) { tim->CCR1 = 0U; } else { tim->CCR2 = 0U; }
    if (rev_ch == 1U) { tim->CCR1 = duty; } else { tim->CCR2 = duty; }
  }
}

static void apply_outputs(int16_t left, int16_t right)
{
  int16_t right_signed = (BOARD_MOTOR_RIGHT_INVERT != 0) ? (int16_t)(-right) : right;

  apply_wheel(BOARD_MOTOR_TIMER, BOARD_MOTOR_L_FORWARD_CH, BOARD_MOTOR_L_REVERSE_CH,
              left, s_motor.left_pwm_limit);
  apply_wheel(BOARD_MOTOR2_TIMER, BOARD_MOTOR2_FORWARD_CH, BOARD_MOTOR2_REVERSE_CH,
              right_signed, s_motor.right_pwm_limit);

  s_motor.output_l = left;
  s_motor.output_r = right;
}

/* ------------------------------------------------------------------------- */
/* timer bring-up                                                            */
/* ------------------------------------------------------------------------- */
/* PWM channel configuration shared by both timers:
 *   CCMR1: CH1 and CH2 in PWM mode 1, both with preload enabled
 *   CCER : both outputs enabled, active high, no complementary output
 * PSC/ARR are set by the caller so that the right-wheel timer can inherit the
 * calibrated prescaler of the left one. */
static void config_pwm_channels(TIM_TypeDef *tim)
{
  tim->CCMR1 = (0x6UL << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
               (0x6UL << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
  tim->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;
}

static void config_pwm_timebase(TIM_TypeDef *tim, uint16_t psc)
{
  tim->CR1  = 0U;
  tim->PSC  = psc;
  tim->ARR  = (uint32_t)(BOARD_MOTOR_PWM_PERIOD - 1U);
  tim->CCR1 = 0U;
  tim->CCR2 = 0U;
  tim->EGR  = TIM_EGR_UG;
  tim->SR   = 0U;
  tim->CR1  = TIM_CR1_ARPE | TIM_CR1_CEN;
}

static void gpio_pins_low(GPIO_TypeDef *port, uint16_t pins)
{
  GPIO_InitTypeDef g = {0};

  HAL_GPIO_WritePin(port, pins, GPIO_PIN_RESET);
  g.Pin   = pins;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(port, &g);
}

void motor_init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* ---- 1. safe state: every H-bridge input driven low as plain GPIO ----- */
  gpio_pins_low(BOARD_MOTOR_L_PORT,
                (uint16_t)(BOARD_MOTOR_L_PIN | BOARD_MOTOR_R_PIN));
  gpio_pins_low(BOARD_MOTOR2_FWD_PORT,
                (uint16_t)(BOARD_MOTOR2_FWD_PIN | BOARD_MOTOR2_REV_PIN));

  /* ---- 2. timers at 0 % duty, before the pins become outputs ----------- */
  __HAL_RCC_TIM4_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  config_pwm_channels(BOARD_MOTOR_TIMER);
  config_pwm_channels(BOARD_MOTOR2_TIMER);
  config_pwm_timebase(BOARD_MOTOR_TIMER,  BOARD_MOTOR_PWM_PSC);
  config_pwm_timebase(BOARD_MOTOR2_TIMER, BOARD_MOTOR_PWM_PSC);

  /* ---- 3. pins to alternate function ----------------------------------- */
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_PULLDOWN;
  g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = BOARD_MOTOR_AF;
  g.Pin       = (uint16_t)(BOARD_MOTOR_L_PIN | BOARD_MOTOR_R_PIN);
  HAL_GPIO_Init(BOARD_MOTOR_L_PORT, &g);

  g.Alternate = BOARD_MOTOR2_AF;
  g.Pin       = BOARD_MOTOR2_FWD_PIN;
  HAL_GPIO_Init(BOARD_MOTOR2_FWD_PORT, &g);
  g.Pin       = BOARD_MOTOR2_REV_PIN;
  HAL_GPIO_Init(BOARD_MOTOR2_REV_PORT, &g);

  /* ---- 4. state ---------------------------------------------------------- */
  s_motor.target_limit   = BOARD_MOTOR_TARGET_LIMIT;
  s_motor.ramp_step      = MOTOR_TARGET_RAMP_STEP;
  s_motor.left_scale     = MOTOR_LEFT_SCALE;
  s_motor.right_scale    = MOTOR_RIGHT_SCALE;
  s_motor.left_pwm_limit = clamp_s16(
      round_f_to_s16((float)BOARD_MOTOR_PWM_MAX_LIMIT * MOTOR_LEFT_SCALE),
      0, (int16_t)BOARD_MOTOR_PWM_PERIOD);
  s_motor.right_pwm_limit = clamp_s16(
      round_f_to_s16((float)BOARD_MOTOR_PWM_MAX_LIMIT * MOTOR_RIGHT_SCALE),
      0, (int16_t)BOARD_MOTOR_PWM_PERIOD);
  s_motor.requested_l = 0;
  s_motor.requested_r = 0;
  s_motor.ramped_l    = 0;
  s_motor.ramped_r    = 0;
  s_motor.output_l    = 0;
  s_motor.output_r    = 0;
  pid_init(&s_motor.pid_l, MOTOR_KP_DEFAULT, MOTOR_KI_DEFAULT, MOTOR_KD_DEFAULT);
  pid_init(&s_motor.pid_r, MOTOR_KP_DEFAULT, MOTOR_KI_DEFAULT, MOTOR_KD_DEFAULT);

  s_htim_motor_l.Instance           = BOARD_MOTOR_TIMER;
  s_htim_motor_l.Init.Period        = (uint32_t)(BOARD_MOTOR_PWM_PERIOD - 1U);
  s_htim_motor_l.Init.Prescaler     = BOARD_MOTOR_PWM_PSC;

  motor_stop();
}

TIM_HandleTypeDef *motor_left_timer_handle(void)
{
  return &s_htim_motor_l;
}

void motor_sync_pwm_clock(void)
{
  /* TIM4 has just been calibrated; TIM2 shares its kernel clock, so inherit
   * the prescaler instead of measuring the same clock twice. */
  BOARD_MOTOR2_TIMER->CR1 &= ~TIM_CR1_CEN;
  BOARD_MOTOR2_TIMER->PSC  = BOARD_MOTOR_TIMER->PSC;
  BOARD_MOTOR2_TIMER->EGR  = TIM_EGR_UG;
  BOARD_MOTOR2_TIMER->SR   = 0U;
  BOARD_MOTOR2_TIMER->CR1 |= TIM_CR1_CEN;
}

/* ------------------------------------------------------------------------- */
/* public control API                                                        */
/* ------------------------------------------------------------------------- */
void motor_stop(void)
{
  s_motor.requested_l = 0;
  s_motor.requested_r = 0;
  s_motor.ramped_l    = 0;
  s_motor.ramped_r    = 0;
  pid_reset(&s_motor.pid_l);
  pid_reset(&s_motor.pid_r);
  apply_outputs(0, 0);
}

void motor_set_duty_percent(int16_t left_percent, int16_t right_percent)
{
  left_percent  = clamp_s16(left_percent, -100, 100);
  right_percent = clamp_s16(right_percent, -100, 100);

  apply_outputs((int16_t)((int32_t)left_percent  * BOARD_MOTOR_PWM_PERIOD / 100),
                (int16_t)((int32_t)right_percent * BOARD_MOTOR_PWM_PERIOD / 100));
}

void motor_set_target_rpm(int16_t left_rpm, int16_t right_rpm)
{
  int16_t l_lim = clamp_s16(
      round_f_to_s16((float)s_motor.target_limit * s_motor.left_scale),
      0, s_motor.left_pwm_limit);
  int16_t r_lim = clamp_s16(
      round_f_to_s16((float)s_motor.target_limit * s_motor.right_scale),
      0, s_motor.right_pwm_limit);

  s_motor.requested_l = clamp_s16(
      round_f_to_s16((float)left_rpm * s_motor.left_scale),
      (int16_t)(-l_lim), l_lim);
  s_motor.requested_r = clamp_s16(
      round_f_to_s16((float)right_rpm * s_motor.right_scale),
      (int16_t)(-r_lim), r_lim);
}

void motor_update(int16_t left_rpm_feedback, int16_t right_rpm_feedback)
{
  /* A zero request snaps to zero instead of ramping down: "stop" must be
   * immediate, only accelerations are ramped. */
  if (s_motor.requested_l == 0)
  {
    s_motor.ramped_l = 0;
  }
  else
  {
    s_motor.ramped_l = ramp_step_towards(s_motor.ramped_l, s_motor.requested_l,
                                         s_motor.ramp_step);
  }

  if (s_motor.requested_r == 0)
  {
    s_motor.ramped_r = 0;
  }
  else
  {
    s_motor.ramped_r = ramp_step_towards(s_motor.ramped_r, s_motor.requested_r,
                                         s_motor.ramp_step);
  }

  apply_outputs(pid_step(&s_motor.pid_l, s_motor.ramped_l, left_rpm_feedback,
                         s_motor.left_pwm_limit),
                pid_step(&s_motor.pid_r, s_motor.ramped_r, right_rpm_feedback,
                         s_motor.right_pwm_limit));
}

void motor_get_output(int16_t *left, int16_t *right)
{
  if (left != NULL)
  {
    *left = s_motor.output_l;
  }
  if (right != NULL)
  {
    *right = s_motor.output_r;
  }
}

void motor_set_pid_gains(float kp, float ki, float kd)
{
  s_motor.pid_l.kp = kp;
  s_motor.pid_l.ki = ki;
  s_motor.pid_l.kd = kd;
  s_motor.pid_r.kp = kp;
  s_motor.pid_r.ki = ki;
  s_motor.pid_r.kd = kd;
  pid_reset(&s_motor.pid_l);
  pid_reset(&s_motor.pid_r);
}
