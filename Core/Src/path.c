/* -----------------------------------------------------------------------------
 * path.c - straight-line waypoint generation + the heading controller.
 *
 * STATUS OF THIS FILE: the geometry and the control law are COMPLETE and
 * self-consistent, the numbers are NOT validated.  Everything in path.h tagged
 * 待上板整定 is a guess that must be tuned on the real chassis.  What is proven
 * today is only that the code compiles and that the maths has no division by
 * zero / atan2(0,0) traps on the degenerate cases.
 *
 * WHY ONLY A STRAIGHT LINE / AN L: a real planner (A*, ICP/OGM) belongs to the
 * final development block.  Part 2 asks for "Bluetooth target -> path -> PID",
 * and the three acceptance trials are three arms of a cross-shaped maze whose
 * corridors run along the world axes (Part 2 slides 2-3).  The honest minimum
 * is therefore: emit the two axis-aligned legs to the target, mark the last
 * point, and let the controller track them.  Replacing this function with a
 * planner does not touch the controller or the motor layer.
 *
 * THE CONTROL LAW (pure pursuit reduced to heading + speed):
 *   1. bearing to the waypoint = atan2(dy, dx)
 *   2. heading error = bearing - pose.heading, wrapped to +-180
 *   3. yaw rate demand = KP * error, clamped
 *   4. forward speed = V_MAX * cos(error), clamped to [V_MIN, V_MAX], and 0 when
 *      the error is large enough that turning in place is better
 *   5. differential drive inverse kinematics -> wheel speeds -> RPM
 * This is a P controller on heading with a speed schedule; it is deliberately
 * the simplest thing that can track a polyline, because every extra term is
 * another constant that cannot be tuned without hardware.
 * -------------------------------------------------------------------------- */

#include "path.h"
#include "board_pins.h"

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#define PATH_DEG_PER_RAD  57.2957795f
#define PATH_RAD_PER_DEG   0.0174533f

/* Wheel circumference in mm, from the same (unverified) diameter the odometry
 * uses.  RPM = mm/s / circumference * 60. */
#define PATH_WHEEL_CIRC_MM  (BOARD_PI_F * BOARD_WHEEL_DIAMETER_MM)

static path_stats_t s_stats;

/* Stats are written from two different tasks (PathGen and PID/Motor), so the
 * counters are updated under the same short memory-only critical section the
 * pose snapshot uses. */
static void stats_add(uint32_t *counter, uint32_t delta)
{
  taskENTER_CRITICAL();
  *counter += delta;
  taskEXIT_CRITICAL();
}

static float path_sqrt(float v)
{
  return sqrtf(v);
}

static float path_abs(float v)
{
  return (v < 0.0f) ? -v : v;
}

/* Wrap an angle to (-180, 180]. */
static float wrap180(float deg)
{
  while (deg > 180.0f)
  {
    deg -= 360.0f;
  }
  while (deg <= -180.0f)
  {
    deg += 360.0f;
  }
  return deg;
}

int path_generate_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                       path_point_t *out, uint16_t max_points)
{
  float dx = (float)(x1 - x0);
  float dy = (float)(y1 - y0);
  float dist = path_sqrt((dx * dx) + (dy * dy));
  float steps_f;
  uint16_t steps;
  uint16_t i;

  if ((out == NULL) || (max_points == 0U))
  {
    return -1;
  }

  if (dist < 0.5f)
  {
    /* Already there: no waypoints.  The caller must not leave the car thinking
     * it is tracking something. */
    return 0;
  }

  steps_f = ceilf(dist / (float)PATH_STEP_MM);

  /* A pure ceil() can exceed what the caller can hold (dist > 16 * 100 mm). */
  if (steps_f > (float)max_points)
  {
    return -2;
  }
  steps = (uint16_t)steps_f;
  if (steps == 0U)
  {
    steps = 1U;
  }

  for (i = 0U; i < steps; i++)
  {
    float t = (float)(i + 1U) / (float)steps;

    out[i].x_mm = (int16_t)lroundf((float)x0 + (dx * t));
    out[i].y_mm = (int16_t)lroundf((float)y0 + (dy * t));
    out[i].flags = (i == (uint16_t)(steps - 1U)) ? (uint8_t)PATH_FLAG_FINAL : 0U;
    out[i].reserved = 0U;
    out[i].index = i;
  }

  /* Intentionally no stats update here: this is a pure geometry function, and
   * the caller already knows how many points it got (path_note_generated). */
  return (int)steps;
}

/* ---------------------------------------------------------------------------
 * Axis-aligned paths (see path.h).  Written as two legs so that the corner is
 * an explicit waypoint: the controller's "turn in place above 45 degrees" rule
 * then does exactly one 90 degree pivot at the junction and nothing else.
 * ------------------------------------------------------------------------ */

/* Append `steps` points along one axis, from (xa,ya) towards (xb,yb), starting
 * at out[first].  The last point is forced to the exact coordinate: rounding a
 * cumulative float can leave it a millimetre short, and a corner that is 1 mm
 * off is invisible in the log but ruins the tape-measure check. */
static uint16_t manhattan_leg(int16_t xa, int16_t ya, int16_t xb, int16_t yb,
                              uint16_t steps, path_point_t *out, uint16_t first)
{
  float fx = (float)(xb - xa);
  float fy = (float)(yb - ya);
  uint16_t i;

  for (i = 0U; i < steps; i++)
  {
    float t = (float)(i + 1U) / (float)steps;
    uint16_t k = (uint16_t)(first + i);

    out[k].x_mm = (int16_t)lroundf((float)xa + (fx * t));
    out[k].y_mm = (int16_t)lroundf((float)ya + (fy * t));
    out[k].flags = 0U;
    out[k].reserved = 0U;
    out[k].index = k;
  }

  out[(uint16_t)(first + steps - 1U)].x_mm = xb;
  out[(uint16_t)(first + steps - 1U)].y_mm = yb;

  return steps;
}

int path_generate_manhattan(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                            path_point_t *out, uint16_t max_points)
{
  float dx = (float)(x1 - x0);
  float dy = (float)(y1 - y0);
  uint16_t n_y = 0U;
  uint16_t n_x = 0U;
  uint16_t total;

  if ((out == NULL) || (max_points == 0U))
  {
    return -1;
  }

  if ((path_abs(dx) < 0.5f) && (path_abs(dy) < 0.5f))
  {
    /* Already there. */
    return 0;
  }

  if (path_abs(dy) >= 0.5f)
  {
    n_y = (uint16_t)ceilf(path_abs(dy) / (float)PATH_STEP_MM);
  }
  if (path_abs(dx) >= 0.5f)
  {
    n_x = (uint16_t)ceilf(path_abs(dx) / (float)PATH_STEP_MM);
  }

  if (((uint32_t)n_y + (uint32_t)n_x) > (uint32_t)max_points)
  {
    return -2;
  }

  total = (uint16_t)(n_y + n_x);
  if (total == 0U)
  {
    return 0;
  }

  if (n_y != 0U)
  {
    (void)manhattan_leg(x0, y0, x0, y1, n_y, out, 0U);
  }
  if (n_x != 0U)
  {
    (void)manhattan_leg(x0, y1, x1, y1, n_x, out, n_y);
  }

  out[(uint16_t)(total - 1U)].flags = (uint8_t)PATH_FLAG_FINAL;

  return (int)total;
}

uint8_t path_arrived(const pose_t *pose, const path_point_t *wp)
{
  float dx;
  float dy;

  if ((pose == NULL) || (wp == NULL))
  {
    return 0U;
  }

  dx = (float)wp->x_mm - pose->x_mm;
  dy = (float)wp->y_mm - pose->y_mm;

  return (path_sqrt((dx * dx) + (dy * dy)) <= PATH_ARRIVE_MM) ? 1U : 0U;
}

uint8_t path_is_final(const path_point_t *wp)
{
  if (wp == NULL)
  {
    return 0U;
  }
  return ((wp->flags & (uint8_t)PATH_FLAG_FINAL) != 0U) ? 1U : 0U;
}

void path_track_step(const pose_t *pose, const path_point_t *wp,
                     int16_t *rpm_left, int16_t *rpm_right)
{
  float dx;
  float dy;
  float bearing_deg;
  float err_deg;
  float err_rad;
  float v_mm_s;
  float w_dps;
  float w_rad_s;
  float half_track;
  float v_left;
  float v_right;

  if ((pose == NULL) || (wp == NULL) || (rpm_left == NULL) || (rpm_right == NULL))
  {
    return;
  }

  dx = (float)wp->x_mm - pose->x_mm;
  dy = (float)wp->y_mm - pose->y_mm;

  /* atan2(0,0) is defined as 0 by C, but "bearing to a point I am standing on"
   * is meaningless; the arrive test should have caught it.  Treat it as "keep
   * the current heading" instead of turning to an arbitrary direction. */
  if ((path_abs(dx) < 0.001f) && (path_abs(dy) < 0.001f))
  {
    bearing_deg = pose->heading_deg;
  }
  else
  {
    bearing_deg = atan2f(dy, dx) * PATH_DEG_PER_RAD;
  }

  err_deg = wrap180(bearing_deg - pose->heading_deg);
  err_rad = err_deg * PATH_RAD_PER_DEG;

  w_dps = PATH_KP_HEADING * err_deg;
  if (w_dps > PATH_W_MAX_DPS)
  {
    w_dps = PATH_W_MAX_DPS;
  }
  else if (w_dps < -PATH_W_MAX_DPS)
  {
    w_dps = -PATH_W_MAX_DPS;
  }

  if (path_abs(err_deg) > PATH_TURN_ONLY_DEG)
  {
    /* Too far off to drive forward meaningfully: rotate on the spot. */
    v_mm_s = 0.0f;
  }
  else
  {
    v_mm_s = PATH_V_MAX_MM_S * cosf(err_rad);
    if (v_mm_s < PATH_V_MIN_MM_S)
    {
      v_mm_s = PATH_V_MIN_MM_S;
    }
  }

  /* Differential drive inverse kinematics.
   *   v_left  = v - w * track/2
   *   v_right = v + w * track/2
   * with w in rad/s.  Positive w (counter-clockwise) slows the left wheel. */
  w_rad_s = w_dps * PATH_RAD_PER_DEG;
  half_track = BOARD_TRACK_WIDTH_MM * 0.5f;
  v_left = v_mm_s - (w_rad_s * half_track);
  v_right = v_mm_s + (w_rad_s * half_track);

  /* mm/s -> RPM.  The PID in motor.c speaks RPM, so the conversion must happen
   * here, in one place. */
  *rpm_left = (int16_t)lroundf((v_left / PATH_WHEEL_CIRC_MM) * 60.0f);
  *rpm_right = (int16_t)lroundf((v_right / PATH_WHEEL_CIRC_MM) * 60.0f);
}

/* ========================================================================== */
/* stats                                                                      */
/* ========================================================================== */
void path_note_generated(uint32_t count) { stats_add(&s_stats.generated, count); }
void path_note_reached(void)            { stats_add(&s_stats.reached, 1U); }
void path_note_finished(void)           { stats_add(&s_stats.finished, 1U); }
void path_note_put_timeout(void)        { stats_add(&s_stats.put_timeouts, 1U); }
void path_note_aborted(void)            { stats_add(&s_stats.aborted, 1U); }

void path_get_stats(path_stats_t *out)
{
  if (out == NULL)
  {
    return;
  }
  taskENTER_CRITICAL();
  memcpy(out, &s_stats, sizeof(*out));
  taskEXIT_CRITICAL();
}
