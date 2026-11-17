/* Host-side unit test for path.c: waypoint generation + the tracking law.
 * Checks the geometry and the kinematics sign conventions, not the tuning. */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "path.h"

static int failures;
static int checks;

#define CIRC_MM (3.14159265f * 65.0f)

static int rpm_of(float mm_s)
{
  return (int)lroundf((mm_s / CIRC_MM) * 60.0f);
}

static void check(int cond, const char *what)
{
  checks++;
  if (!cond)
  {
    printf("FAIL  %s\n", what);
    failures++;
  }
}

static void test_generate(void)
{
  path_point_t pts[Q_PATH_CAPACITY];
  int n;
  int i;

  memset(pts, 0, sizeof(pts));
  n = path_generate_line(0, 0, 700, -700, pts, Q_PATH_CAPACITY);
  printf("gen (0,0)->(700,-700): n=%d", n);
  for (i = 0; i < n; i++) printf(" (%d,%d,f%d)", pts[i].x_mm, pts[i].y_mm, pts[i].flags);
  printf("\n");

  check(n == 10, "700,-700 is 989.9 mm -> 10 points of ~99 mm");
  check(pts[n - 1].x_mm == 700 && pts[n - 1].y_mm == -700, "last point is the exact target");
  check((pts[n - 1].flags & PATH_FLAG_FINAL) != 0, "last point carries PATH_FLAG_FINAL");
  for (i = 0; i < n - 1; i++)
  {
    check((pts[i].flags & PATH_FLAG_FINAL) == 0, "intermediate points are not final");
  }
  /* monotonic progress along the line */
  for (i = 1; i < n; i++)
  {
    check(pts[i].x_mm >= pts[i - 1].x_mm, "x advances monotonically");
    check(pts[i].y_mm <= pts[i - 1].y_mm, "y decreases monotonically");
  }
  /* spacing sanity: no gap larger than the step plus rounding */
  for (i = 1; i < n; i++)
  {
    float dx = (float)(pts[i].x_mm - pts[i - 1].x_mm);
    float dy = (float)(pts[i].y_mm - pts[i - 1].y_mm);
    float d = sqrtf(dx * dx + dy * dy);
    check(d <= (float)PATH_STEP_MM + 1.5f, "step does not exceed PATH_STEP_MM");
  }

  /* degenerate: target == start */
  n = path_generate_line(100, 100, 100, 100, pts, Q_PATH_CAPACITY);
  check(n == 0, "zero-length path yields no waypoints");

  /* too long for the queue */
  n = path_generate_line(0, 0, 2000, 0, pts, Q_PATH_CAPACITY);
  check(n == -2, "20 x 100 mm does not fit in 16 slots -> refused");

  /* exactly at capacity: 1600 mm / 100 mm = 16 points */
  n = path_generate_line(0, 0, 1600, 0, pts, Q_PATH_CAPACITY);
  check(n == 16, "1600 mm fits exactly in 16 slots");
  check(pts[15].x_mm == 1600, "last point still exact at capacity");

  /* NULL / zero capacity */
  check(path_generate_line(0, 0, 10, 10, NULL, 4) == -1, "NULL output refused");
  check(path_generate_line(0, 0, 10, 10, pts, 0) == -1, "zero capacity refused");
}

static void test_arrive(void)
{
  pose_t p;

  memset(&p, 0, sizeof(p));
  p.heading_deg = 90.0f;

  {
    path_point_t wp = { 0, 20, 0, 0, 0 };
    check(path_arrived(&p, &wp) == 1, "20 mm away counts as arrived (<=30)");
  }
  {
    path_point_t wp = { 0, 31, 0, 0, 0 };

    check(path_arrived(&p, &wp) == 0, "31 mm away is not arrived");
  }
  {
    path_point_t wp = { 30, 0, 0, 0, 0 };
    check(path_arrived(&p, &wp) == 1, "exactly 30 mm counts as arrived (boundary)");
  }
  {
    path_point_t wp = { -100, -100, 0, 0, 0 };
    check(path_arrived(&p, &wp) == 0, "141 mm away is not arrived");
  }
}

static void test_track(void)
{
  pose_t p;
  int16_t l;
  int16_t r;

  /* Straight ahead: facing +y (heading 90), waypoint 1 m straight ahead. */
  memset(&p, 0, sizeof(p));
  p.heading_deg = 90.0f;

  {
    path_point_t wp = { 0, 1000, 0, 0, 0 };
    l = r = 0;
    path_track_step(&p, &wp, &l, &r);

    check(l == r, "zero heading error => equal wheel targets");
    check(l == (int16_t)rpm_of(PATH_V_MAX_MM_S),
          "straight ahead runs at PATH_V_MAX_MM_S");
    printf("straight: rpm l=%d r=%d (expected %d)\n", l, r, rpm_of(PATH_V_MAX_MM_S));
  }

  /* Target 90 deg to the right: must rotate in place, left wheel forward. */
  {
    path_point_t wp = { 1000, 0, 0, 0, 0 };
    l = r = 0;
    path_track_step(&p, &wp, &l, &r);

    printf("right turn: rpm l=%d r=%d\n", l, r);
    check(l > 0, "clockwise turn drives the left wheel forward");
    check(r < 0, "clockwise turn drives the right wheel backward");
    check(l == -r, "in-place turn is symmetric");
    check(l == (int16_t)rpm_of((PATH_W_MAX_DPS * 3.14159265f / 180.0f) * 65.0f),
          "yaw rate is clamped to PATH_W_MAX_DPS");
  }

  /* Target 90 deg to the left. */
  {
    path_point_t wp = { -1000, 0, 0, 0, 0 };
    l = r = 0;
    path_track_step(&p, &wp, &l, &r);
    printf("left turn:  rpm l=%d r=%d\n", l, r);
    check(l < 0, "counter-clockwise turn drives the left wheel backward");
    check(r > 0, "counter-clockwise turn drives the right wheel forward");
  }

  /* Small heading error: forward speed must stay above the minimum. */
  {
    path_point_t wp = { -50, 1000, 0, 0, 0 };
    l = r = 0;
    path_track_step(&p, &wp, &l, &r);
    check(l > 0, "small error still drives forward");
  }

  /* Degenerate: standing exactly on the waypoint must not produce NaN. */
  {
    path_point_t wp = { 0, 0, 0, 0, 0 };
    p.x_mm = 0.0f;
    p.y_mm = 0.0f;
    l = r = -999;
    path_track_step(&p, &wp, &l, &r);
    check(l > -30000 && l < 30000 && r > -30000 && r < 30000,
          "zero-distance waypoint produces finite wheel targets");
    printf("degenerate: rpm l=%d r=%d\n", l, r);
  }

  /* 180 deg error must be clamped, not spin at 360 deg/s. */
  {
    path_point_t wp = { 0, -1000, 0, 0, 0 };
    p.x_mm = 0.0f;
    p.y_mm = 0.0f;
    l = r = 0;
    path_track_step(&p, &wp, &l, &r);
    printf("180 deg:    rpm l=%d r=%d\n", l, r);
    check(labs((long)l) <= (long)rpm_of((PATH_W_MAX_DPS * 3.14159265f / 180.0f) * 65.0f),
          "180 deg error is clamped to the yaw limit");
  }

  /* NULL guards */
  {
    path_point_t wp = { 0, 1000, 0, 0, 0 };
    path_track_step(NULL, &wp, &l, &r);
    path_track_step(&p, NULL, &l, &r);
    path_track_step(&p, &wp, NULL, &r);
    path_track_step(&p, &wp, &l, NULL);
    check(1, "NULL arguments do not crash");
  }
}

int main(void)
{
  test_generate();
  test_arrive();
  test_track();

  printf("\n%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
