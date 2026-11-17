/* Host-side unit test for pose.c: odometry integration and the heading source
 * switch.  The point is the SIGN conventions and the dead-reckoning fallback,
 * not the (uncalibrated) scale factors. */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pose.h"
#include "encoder.h"
#include "imu.h"

/* ---- stubs controlled by the test ---------------------------------------- */
static int16_t stub_dl;
static int16_t stub_dr;
static imu_sample_t stub_imu;
static uint32_t stub_tick;

uint32_t HAL_GetTick(void) { return stub_tick; }

void encoder_init(void) { }
void encoder_zero(void) { }
int encoder_sample(uint32_t period_ms, int16_t *dl, int16_t *dr)
{
  (void)period_ms;
  if (dl) *dl = stub_dl;
  if (dr) *dr = stub_dr;
  return 0;
}
int16_t encoder_left_rpm(void) { return 0; }
int16_t encoder_right_rpm(void) { return 0; }
int32_t encoder_left_total(void) { return 0; }
int32_t encoder_right_total(void) { return 0; }

void imu_get(imu_sample_t *out) { *out = stub_imu; }
void imu_reset_heading(void) { stub_imu.heading_deg = 0.0f; }

/* ---- helpers ------------------------------------------------------------- */
static int failures;
static int checks;

static void check(int cond, const char *what)
{
  checks++;
  if (!cond)
  {
    printf("FAIL  %s\n", what);
    failures++;
  }
}

static void step(int n, int16_t dl, int16_t dr)
{
  int i;
  stub_dl = dl;
  stub_dr = dr;
  for (i = 0; i < n; i++)
  {
    stub_tick += 10U;
    pose_update(10U);
  }
}

int main(void)
{
  pose_t p;
  const float mm_per_count = (3.14159265f * 65.0f) / 1320.0f;

  memset(&stub_imu, 0, sizeof(stub_imu));
  stub_imu.valid = 1U;          /* gyro present, zero rotation */

  pose_init();
  pose_get(&p);
  check(fabsf(p.heading_deg - 90.0f) < 0.01f, "boot heading is +90 deg (forward = +y)");
  check(p.x_mm == 0.0f && p.y_mm == 0.0f, "boot position is the origin");

  /* Forward: equal counts on both wheels. */
  step(10, 100, 100);
  pose_get(&p);
  printf("forward 10x100 counts: x=%.3f y=%.3f h=%.3f\n", p.x_mm, p.y_mm, p.heading_deg);
  check(p.x_mm > -0.01f && p.x_mm < 0.01f, "driving forward does not move x");
  check(fabsf(p.y_mm - (10.0f * 100.0f * mm_per_count)) < 0.5f,
        "driving forward advances +y by counts*mm_per_count");
  check(fabsf(p.heading_deg - 90.0f) < 0.01f, "heading unchanged while driving straight");

  /* Reverse: -y, still no rotation. */
  pose_reset();
  step(5, -100, -100);
  pose_get(&p);
  check(p.y_mm < 0.0f, "reverse drives -y");
  check(fabsf(p.heading_deg - 90.0f) < 0.01f, "reverse does not rotate");

  /* Gyro heading is absolute: 10 deg of gyro => 100 deg in the pose frame. */
  pose_reset();
  stub_imu.heading_deg = 10.0f;
  step(1, 0, 0);
  pose_get(&p);
  printf("gyro 10 deg -> pose heading %.3f\n", p.heading_deg);
  check(fabsf(p.heading_deg - 100.0f) < 0.01f, "gyro heading is offset by +90 deg");
  check(p.heading_src == 0U, "heading source reported as gyro");

  /* Gyro invalid: fall back to the wheel difference.  dr > dl => +yaw. */
  pose_reset();
  stub_imu.valid = 0U;
  stub_imu.heading_deg = 0.0f;
  step(1, 0, 100);
  pose_get(&p);
  printf("wheel fallback: h=%.3f src=%u\n", p.heading_deg, p.heading_src);
  check(p.heading_src == 1U, "heading source reported as wheel odometry");
  check(p.heading_deg > 90.0f, "right wheel faster turns counter-clockwise (+heading)");

  /* Backwards wheel difference. */
  pose_reset();
  step(1, 100, 0);
  pose_get(&p);
  check(p.heading_deg < 90.0f, "left wheel faster turns clockwise (-heading)");

  /* Turning in place must not translate. */
  pose_reset();
  step(1, 100, -100);
  pose_get(&p);
  printf("in-place turn: x=%.3f y=%.3f h=%.3f\n", p.x_mm, p.y_mm, p.heading_deg);
  check(fabsf(p.x_mm) < 0.01f && fabsf(p.y_mm) < 0.01f, "in-place turn keeps position");
  check(fabsf(p.heading_deg - 90.0f) > 0.01f, "in-place turn changes heading");

  /* Heading wraps and stays in (-180, 180]. */
  pose_reset();
  stub_imu.valid = 1U;
  stub_imu.heading_deg = 170.0f;
  step(1, 0, 0);
  pose_get(&p);
  check(p.heading_deg > -180.0f && p.heading_deg <= 180.0f, "heading stays wrapped");

  /* pose_reset clears position. */
  step(3, 50, 50);
  pose_reset();
  pose_get(&p);
  check(p.x_mm == 0.0f && p.y_mm == 0.0f, "pose_reset restores the origin");
  check(fabsf(p.heading_deg - 90.0f) < 0.01f, "pose_reset restores the boot heading");

  /* period_ms == 0 must not divide by zero. */
  {
    pose_t before;
    pose_get(&before);
    stub_dl = 10;
    stub_dr = 10;
    pose_update(0U);
    pose_get(&p);
    check(isfinite(p.v_mm_s) && isfinite(p.x_mm), "zero period does not produce inf/nan");
  }

  printf("\n%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
