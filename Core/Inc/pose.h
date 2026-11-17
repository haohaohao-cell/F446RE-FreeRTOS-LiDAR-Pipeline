/* -----------------------------------------------------------------------------
 * pose.h - dead-reckoned pose (x, y, heading) from encoders + gyro.
 *
 * OWNERSHIP (CODING_STANDARDS 0.3): the Pose task is the ONLY caller of
 * pose_update() and the only reader of the hardware encoder counters.  Other
 * tasks read a snapshot through pose_get(), which copies under a short critical
 * section - the same "snapshot a few words" pattern already used for the LiDAR
 * scan map, and the reason no mutex is needed here.
 *
 * UNITS: millimetres and degrees.  Floating point is acceptable here because
 * this is an estimate that is recomputed from scratch every 10 ms, not a value
 * accumulated bit-exactly for a long time.
 *
 * CALIBRATION STATUS: every scale factor used here
 *   - BOARD_MM_PER_COUNT  (wheel diameter, counts/rev)
 *   - BOARD_TRACK_WIDTH_MM
 *   - the encoder direction signs
 * is an UNVERIFIED assumption from board_pins.h.  The maths is exercised by the
 * build, but the numbers coming out of it mean nothing until the wheel diameter
 * and a 1.000 m push test are measured on the real chassis.  Do not quote a
 * distance from this module as a measurement before that is done.
 * -------------------------------------------------------------------------- */

#ifndef POSE_H
#define POSE_H

#include <stdint.h>

typedef struct
{
  float    x_mm;          /* + right                                        */
  float    y_mm;          /* + forward                                      */
  float    heading_deg;   /* + counter-clockwise, 0 = +x axis of the car     */
  float    v_mm_s;        /* forward speed of the wheel midpoint             */
  float    w_dps;         /* yaw rate                                        */
  uint32_t stamp_ms;      /* HAL_GetTick() of the last update                */
  uint8_t  heading_src;   /* 0 = gyro, 1 = wheel odometry (fallback)          */
  uint8_t  valid;         /* at least one update has happened                */
} pose_t;

typedef struct
{
  uint32_t updates;
  uint32_t imu_missing;   /* updates where the gyro was stale                */
  float    odo_mm;        /* cumulative path length (sign lost)              */
  float    left_mm;
  float    right_mm;
} pose_stats_t;

/* Zero the pose.  Call once before the scheduler starts. */
void pose_init(void);

/* One 10 ms step: sample both encoders, update the heading, integrate x/y.
 * period_ms must be the real elapsed period.  Pose task only. */
void pose_update(uint32_t period_ms);

/* Copy the current pose.  Safe from any task. */
void pose_get(pose_t *out);

/* Define the current position as the origin (same orientation). */
void pose_reset(void);

void pose_get_stats(pose_stats_t *out);

#endif /* POSE_H */
