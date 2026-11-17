/* -----------------------------------------------------------------------------
 * path.h - Q_Path message, waypoint generation and the tracking law.
 *
 * THIS IS THE ABSTRACTION SEAM (CODING_STANDARDS 9.1)
 * --------------------------------------------------
 * Q_Path carries WORLD COORDINATE waypoints, never "grid cells" or "the next
 * Bluetooth command".  Two consequences worth keeping in mind:
 *   - today PathGen produces a straight line from the current pose to the target
 *     (no planner yet), and
 *   - the final product's maze solver can replace exactly that producer without
 *     touching the PID/motor task, because both speak Q_Path.
 * The PID/motor task therefore never asks where a waypoint came from.
 *
 * MESSAGE SIZE: 8 bytes, fixed, POD, no pointers (CODING_STANDARDS 2.1).
 * -------------------------------------------------------------------------- */

#ifndef PATH_H
#define PATH_H

#include <stdint.h>

#include "pose.h"

/* Capacity of Q_Path.  A 1 m move at PATH_STEP_MM = 100 mm needs 10 points, so
 * 16 leaves room for one command queued behind another.  The producer blocks
 * (with a timeout) rather than dropping points: a missing waypoint is a car that
 * stops in the wrong place (CODING_STANDARDS 2.2). */
#define Q_PATH_CAPACITY   16U

/* Waypoint spacing for the straight-line generator. */
#define PATH_STEP_MM      100

/* ---- tracking limits (first-order values, 待整定 under load) -------------- */
/* 200 mm/s was the design target but needs ~70 % duty on this chassis (measured:
 * 20 % duty -> 17 rpm loaded), which leaves no margin for battery sag and makes
 * the 30 mm arrival radius easy to overshoot.  120 mm/s needs ~40 % duty, so it
 * is the value used for the first on-ground runs; raise it once the closed loop
 * is proven.  These are control-LOOP limits, not the motor PID gains in motor.h
 * - both still need a proper tuning pass with the car loaded. */
#define PATH_V_MAX_MM_S     120.0f   /* cruise speed, mm/s                    */
#define PATH_V_MIN_MM_S      40.0f   /* slowest forward speed while turning   */
#define PATH_KP_HEADING       2.0f   /* (deg/s) per deg of heading error      */
#define PATH_W_MAX_DPS       90.0f   /* yaw rate clamp                        */
#define PATH_ARRIVE_MM       30.0f   /* waypoint considered reached           */
#define PATH_TURN_ONLY_DEG   45.0f   /* turn in place above this error        */

typedef struct
{
  int16_t  x_mm;      /* + right                                     */
  int16_t  y_mm;      /* + forward                                   */
  uint8_t  flags;     /* PATH_FLAG_*                                 */
  uint8_t  reserved;  /* 0                                           */
  uint16_t index;     /* sequence number, for logs                   */
} path_point_t;

/* Final waypoint of a path: the PID task stops the car on reaching it. */
#define PATH_FLAG_FINAL   0x01U

_Static_assert(sizeof(path_point_t) == 8U, "Q_Path message must stay 8 bytes");

typedef struct
{
  uint32_t generated;      /* waypoints produced                            */
  uint32_t reached;        /* waypoints consumed by the controller          */
  uint32_t finished;       /* paths completed                               */
  uint32_t put_timeouts;   /* Q_Path full for longer than the put timeout    */
  uint32_t aborted;        /* STOP commands that cleared a path             */
} path_stats_t;

/* Fill `out` with waypoints from (x0,y0) to (x1,y1), at most `max_points`.
 * Returns the number written (0 if the target is the start point), or a negative
 * value if it does not fit.  The last point is the exact target and carries
 * PATH_FLAG_FINAL. */
int path_generate_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                       path_point_t *out, uint16_t max_points);

/* Tracking law: given the pose and the active waypoint, produce wheel RPM
 * targets.  Pure function, no state, no hardware. */
void path_track_step(const pose_t *pose, const path_point_t *wp,
                     int16_t *rpm_left, int16_t *rpm_right);

/* 1 if the waypoint is within PATH_ARRIVE_MM. */
uint8_t path_arrived(const pose_t *pose, const path_point_t *wp);

/* 1 if this waypoint ends the path. */
uint8_t path_is_final(const path_point_t *wp);

/* Stats bookkeeping.  Small counters advanced by the tasks that own the
 * respective events, so the 1 Hz line does not have to know about them. */
void path_note_generated(uint32_t count);
void path_note_reached(void);
void path_note_finished(void);
void path_note_put_timeout(void);
void path_note_aborted(void);

void path_get_stats(path_stats_t *out);

#endif /* PATH_H */
