/* -----------------------------------------------------------------------------
 * motor.h - two-wheel H-bridge driver with per-wheel speed PID.
 *
 * PORTED FROM lessonProgram/Core/Src/Motor.c (Coding Day 2), with three
 * deliberate changes:
 *   1. Board_SetMotorPwm() replaced by direct CCR writes on the owning timer,
 *      so the module does not depend on the Coding Day 2 board layer.
 *   2. PWM period is BOARD_MOTOR_PWM_PERIOD (840) instead of the hard-coded
 *      799, which came from a "16 MHz / 800" assumption that is wrong on this
 *      168 MHz board (see board_pins.h section 3).
 *   3. The right wheel gets its own timer (section 3 of board_pins.h explains
 *      why one timer cannot drive both wheels here).
 *
 * UNITS: speeds are "RPM" as produced by encoder.c (counts/ms scaled by
 * 60/1320).  Targets are clamped to BOARD_MOTOR_TARGET_LIMIT.
 *
 * PID GAINS are the Coding Day 2 values and are NOT validated on this chassis:
 * they must be re-tuned on hardware (documented as an open item).  With ki
 * non-zero and no anti-windup beyond an integral clamp, a large step command
 * will overshoot until tuned.
 *
 * THREADING: every function here is called from the single combined
 * "PID + Motor" 10 ms task (see CODING_STANDARDS 1.2).  No locking is provided
 * or needed; this module must never be called from an ISR.
 * -------------------------------------------------------------------------- */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

/* PID state, one per wheel. */
typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral;
  float last_error;
  float output;
} motor_pid_t;

typedef struct
{
  int16_t  target_limit;      /* |target| clamp, RPM-ish units            */
  int16_t  ramp_step;         /* per 10 ms cycle, same units              */
  float    left_scale;        /* mechanical trim, left wheel              */
  float    right_scale;       /* mechanical trim, right wheel             */
  int16_t  left_pwm_limit;    /* |PWM| clamp in CCR counts                */
  int16_t  right_pwm_limit;
  int16_t  requested_l;
  int16_t  requested_r;
  int16_t  ramped_l;
  int16_t  ramped_r;
  int16_t  output_l;          /* last applied CCR value, signed           */
  int16_t  output_r;
  motor_pid_t pid_l;
  motor_pid_t pid_r;
} motor_handle_t;

/* Bring the two PWM timers and the four H-bridge pins up in the safe order
 * required by CODING_STANDARDS 3.3: outputs low first, then PWM at 0 % duty,
 * then the pins are switched to their alternate function.  Ends with both
 * wheels stopped.  Call once, before the scheduler starts. */
void motor_init(void);

/* Copy the calibrated prescaler from the left-wheel timer to the right-wheel
 * timer.  Both are APB1 timers, so they share one kernel clock and one
 * calibration; this keeps them phase-identical instead of calibrating twice.
 * Call immediately after hal_tim_calibrate() on the left timer. */
void motor_sync_pwm_clock(void);

/* Stop both wheels and reset both PID states.  Safe to call from any task. */
void motor_stop(void);

/* ---- open loop, for bring-up only ---------------------------------------- */
/* duty in percent, -100..+100, negative = reverse. */
void motor_set_duty_percent(int16_t left_percent, int16_t right_percent);

/* ---- closed loop --------------------------------------------------------- */
/* Target wheel speeds.  Zeros mean "stop that wheel and reset its PID". */
void motor_set_target_rpm(int16_t left_rpm, int16_t right_rpm);

/* Run one PID step with the measured feedback and apply the result.
 * Call at exactly BOARD_PERIOD_FAST_MS (10 ms). */
void motor_update(int16_t left_rpm_feedback, int16_t right_rpm_feedback);

/* Last applied signed PWM, for the observability line. */
void motor_get_output(int16_t *left, int16_t *right);

/* Retune at run time (serial console).  Gains are copied to both wheels. */
void motor_set_pid_gains(float kp, float ki, float kd);

/* Handle describing the left-wheel PWM timer, for hal_tim_calibrate().
 * The module drives its timers through registers, so this handle exists only
 * to carry Instance/Period/Prescaler into the calibration helper; it is not
 * used for any HAL timer operation.  Valid after motor_init(). */
TIM_HandleTypeDef *motor_left_timer_handle(void);

#endif /* MOTOR_H */
