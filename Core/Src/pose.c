/* -----------------------------------------------------------------------------
 * pose.c - odometry from TIM1/TIM8 counts + gyro heading.
 *
 * WHY THE GYRO IS PREFERRED FOR HEADING AND THE WHEELS ARE THE FALLBACK:
 * differential drive odometry loses heading accuracy from any wheel slip or
 * unequal tyre diameter, and that error grows without bound.  The gyro is
 * absolute-world referenced (relative to boot), so it is used whenever it is
 * fresh.  When it is stale (I2C failure, IMU absent) the wheel difference is
 * integrated instead so the estimate degrades instead of freezing - and the
 * snapshot says which source was used (heading_src), so a bad run can be
 * explained afterwards instead of guessed at.
 * -------------------------------------------------------------------------- */

#include "pose.h"
#include "board_pins.h"
#include "encoder.h"
#include "imu.h"

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#define POSE_DEG_PER_RAD  57.2957795f
#define POSE_RAD_PER_DEG   0.0174533f

/* The gyro integrates from 0 deg at boot along the car's own forward axis.
 * bt_protocol.h defines +y as forward, and this module measures heading from +x,
 * so "facing forward at boot" is +90 deg in the pose frame.  One constant, one
 * place: get this wrong and the car drives 90 degrees off every waypoint. */
#define POSE_IMU_TO_NAV_DEG  90.0f

/* Guard against a division by zero if a caller passes period_ms = 0. */
#define POSE_MIN_PERIOD_MS 1U

static pose_t      s_pose;
static pose_stats_t s_stats;

static void pose_lock(void)
{
  /* Short, memory-only critical section (CODING_STANDARDS 1.5): a few float
   * copies, no calls.  Before the scheduler starts there is only one thread, and
   * taskENTER_CRITICAL() is still valid there. */
  taskENTER_CRITICAL();
}

static void pose_unlock(void)
{
  taskEXIT_CRITICAL();
}

void pose_init(void)
{
  memset(&s_pose, 0, sizeof(s_pose));
  memset(&s_stats, 0, sizeof(s_stats));

  /* BUG TRAP: heading 0 means "facing +y" in the navigation convention used by
   * bt_protocol.h (+y is forward).  The pose maths measures heading from +x, so
   * the car's initial orientation is +90 deg, not 0.  Keeping the internal
   * convention in one place (here) is what makes path.c's atan2() comparison
   * correct; changing this line alone will make the car drive sideways. */
  s_pose.heading_deg = 90.0f;
  s_pose.heading_src = 1U;
  s_pose.valid = 0U;
}

void pose_reset(void)
{
  pose_lock();
  s_pose.x_mm = 0.0f;
  s_pose.y_mm = 0.0f;
  s_pose.heading_deg = 90.0f;
  s_pose.v_mm_s = 0.0f;
  s_pose.w_dps = 0.0f;
  pose_unlock();

  encoder_zero();
  imu_reset_heading();
}

void pose_update(uint32_t period_ms)
{
  int16_t d_left = 0;
  int16_t d_right = 0;
  imu_sample_t imu;
  float dt_s;
  float dl_mm;
  float dr_mm;
  float heading_rad;
  float dtheta_deg;
  uint32_t period = (period_ms < POSE_MIN_PERIOD_MS) ? POSE_MIN_PERIOD_MS : period_ms;

  (void)encoder_sample(period, &d_left, &d_right);

  dl_mm = (float)d_left * BOARD_MM_PER_COUNT;
  dr_mm = (float)d_right * BOARD_MM_PER_COUNT;
  dt_s = (float)period / 1000.0f;

  pose_lock();
  s_pose.v_mm_s = ((dl_mm + dr_mm) * 0.5f) / dt_s;

  imu_get(&imu);
  if (imu.valid != 0U)
  {
    /* Gyro heading is absolute (relative to boot), so it is assigned directly
     * rather than integrated - that is why no drift accumulates here.  The
     * offset converts "gyro zero = car forward" into the pose frame. */
    float gyro_nav = imu.heading_deg + POSE_IMU_TO_NAV_DEG;

    while (gyro_nav > 180.0f)  { gyro_nav -= 360.0f; }
    while (gyro_nav < -180.0f) { gyro_nav += 360.0f; }

    dtheta_deg = gyro_nav - s_pose.heading_deg;
    while (dtheta_deg > 180.0f)  { dtheta_deg -= 360.0f; }
    while (dtheta_deg < -180.0f) { dtheta_deg += 360.0f; }

    s_pose.heading_deg = gyro_nav;
    s_pose.heading_src = 0U;
  }
  else
  {
    /* Fallback: the wheel difference over the track width. */
    dtheta_deg = ((dr_mm - dl_mm) / BOARD_TRACK_WIDTH_MM) * POSE_DEG_PER_RAD;
    s_pose.heading_deg += dtheta_deg;
    s_pose.heading_src = 1U;
    s_stats.imu_missing++;
  }
  while (s_pose.heading_deg > 180.0f)  { s_pose.heading_deg -= 360.0f; }
  while (s_pose.heading_deg < -180.0f) { s_pose.heading_deg += 360.0f; }

  heading_rad = s_pose.heading_deg * POSE_RAD_PER_DEG;
  s_pose.x_mm += s_pose.v_mm_s * dt_s * cosf(heading_rad);
  s_pose.y_mm += s_pose.v_mm_s * dt_s * sinf(heading_rad);

  s_pose.w_dps = dtheta_deg / dt_s;
  s_pose.stamp_ms = HAL_GetTick();
  s_pose.valid = 1U;
  pose_unlock();

  s_stats.updates++;
  s_stats.left_mm += dl_mm;
  s_stats.right_mm += dr_mm;
  s_stats.odo_mm += (dl_mm + dr_mm) * 0.5f;
}

void pose_get(pose_t *out)
{
  if (out == NULL)
  {
    return;
  }
  pose_lock();
  memcpy(out, &s_pose, sizeof(*out));
  pose_unlock();
}

void pose_get_stats(pose_stats_t *out)
{
  if (out == NULL)
  {
    return;
  }
  pose_lock();
  memcpy(out, &s_stats, sizeof(*out));
  pose_unlock();
}
