/* -----------------------------------------------------------------------------
 * encoder.h - wheel encoders via the TIM1/TIM8 hardware quadrature decoders.
 *
 * THERE IS NO ENCODER TASK, BY DESIGN.
 * TIM1 and TIM8 run in encoder mode (SMCR = 3, x4 decoding): the counting is
 * done by the timer peripheral itself, in the background, with no software
 * involvement.  Reading them is one 16-bit register read per wheel, which is
 * what "Hardware interrupt/capture - counts processed every 10 ms" means in
 * the reference architecture.  Pose Estimation therefore samples both counters
 * at the top of its own 10 ms slot; a separate encoder thread would only add a
 * phase offset between the counts and the pose that consumes them.
 *
 * WHY TIM1 IS "RIGHT" AND TIM8 IS "LEFT": carried over from
 * lessonProgram/Encoder.c, together with the mirrored right-channel sign.
 * Both mapping and sign are UNVERIFIED on this chassis - if they are swapped
 * the car spins on the spot instead of driving, which is the first thing to
 * check on hardware.
 *
 * COUNTER OVERFLOW: the delta is computed in 16-bit arithmetic
 * (int16_t)(now - last), which is exact for any movement of fewer than 32768
 * counts between two samples.  At 1320 counts/rev that is 24 rev in 10 ms,
 * i.e. 145 km/h on a 65 mm wheel - not reachable, so no explicit wrap handling
 * beyond the 16-bit subtraction is needed.  The counters themselves are never
 * cleared while running; the absolute total is accumulated in software.
 *
 * THREADING: Pose Estimation is the only caller.  No locking.
 * -------------------------------------------------------------------------- */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* Configure TIM1 (right) and TIM8 (left) as x4 quadrature decoders and zero
 * both counters.  Call once, before the scheduler starts. */
void encoder_init(void);

/* Zero both software totals and both filter states.  The hardware counters are
 * re-baselined on the next sample, so this may be called at run time. */
void encoder_zero(void);

/* Read both wheels: delta in counts since the previous call (signed, forward
 * positive), and update the filtered RPM estimate.
 *
 * period_ms is the time since the previous call and must be the real elapsed
 * period (10 ms in the control loop), because it scales both the RPM estimate
 * and, downstream, the millimetres travelled.
 *
 * Either output pointer may be NULL.  Returns 0 on success, -1 if period_ms is
 * zero (estimates are then left unchanged but the totals still advance). */
int encoder_sample(uint32_t period_ms, int16_t *delta_left, int16_t *delta_right);

/* Filtered wheel speed in RPM, as of the last encoder_sample(). */
int16_t encoder_left_rpm(void);
int16_t encoder_right_rpm(void);

/* Cumulative signed counts since encoder_zero().  Feeds odometry. */
int32_t encoder_left_total(void);
int32_t encoder_right_total(void);

#endif /* ENCODER_H */
