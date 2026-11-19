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

static void test_manhattan(void)
{
  path_point_t pts[Q_PATH_CAPACITY];
  int n;
  int i;

  /* ------------------------------------------------------------------------
   * Trial 1 of the marking scheme, (0.00, 1.40): no turn at all, so the L
   * generator must agree with the straight-line one waypoint for waypoint.
   * ---------------------------------------------------------------------- */
  memset(pts, 0, sizeof(pts));
  n = path_generate_manhattan(0, 0, 0, 1400, pts, Q_PATH_CAPACITY);
  printf("manhattan (0,0)->(0,1400): n=%d\n", n);
  check(n == 14, "(0,0)->(0,1400) is 14 x 100 mm");
  check(pts[n - 1].x_mm == 0 && pts[n - 1].y_mm == 1400, "axial last point is exact");
  for (i = 0; i < n; i++)
  {
    check(pts[i].x_mm == 0, "axial leg keeps x = 0");
    check(pts[i].y_mm == (i + 1) * 100, "axial leg advances 100 mm per point");
    check((pts[i].flags & PATH_FLAG_FINAL) == ((i == n - 1) ? PATH_FLAG_FINAL : 0U),
          "only the last axial point is final");
  }

  /* ------------------------------------------------------------------------
   * Trial 3 of the marking scheme, (0.70, 0.70) 右前: 先直行 700 mm 沿 +y,
   * 再右转 90 度, 再沿 +x 走 700 mm.  This is the shape on Part 2 slide 3 and
   * the shape the teacher asked for ("先走直线，再转弯").
   * ---------------------------------------------------------------------- */
  memset(pts, 0, sizeof(pts));
  n = path_generate_manhattan(0, 0, 700, 700, pts, Q_PATH_CAPACITY);
  printf("manhattan (0,0)->(700,700): n=%d\n", n);
  check(n == 14, "(0,0)->(700,700) is 7 + 7 points");
  for (i = 0; i < 7; i++)
  {
    check(pts[i].x_mm == 0, "leg 1 stays on the y corridor x = 0");
    check(pts[i].y_mm == (i + 1) * 100, "leg 1 advances 100 mm per point");
  }
  check(pts[6].x_mm == 0 && pts[6].y_mm == 700, "corner (0,700) ends leg 1");
  for (i = 7; i < n; i++)
  {
    check(pts[i].y_mm == 700, "leg 2 stays on the x corridor y = 700");
    check(pts[i].x_mm == (i - 6) * 100, "leg 2 advances 100 mm per point");
  }
  check(pts[n - 1].x_mm == 700 && pts[n - 1].y_mm == 700, "last point is the exact target");
  check((pts[n - 1].flags & PATH_FLAG_FINAL) != 0, "last point carries PATH_FLAG_FINAL");
  for (i = 0; i < n - 1; i++)
  {
    check((pts[i].flags & PATH_FLAG_FINAL) == 0, "intermediate points are not final");
  }
  /* the corner must not be emitted twice: a duplicate would make the
   * controller "arrive" at a point it is already on and skip the turn */
  check(!(pts[6].x_mm == pts[7].x_mm && pts[6].y_mm == pts[7].y_mm),
        "corner point is not duplicated");
  /* every step is exactly the configured resolution, on the axis */
  for (i = 1; i < n; i++)
  {
    int ddx = abs(pts[i].x_mm - pts[i - 1].x_mm);
    int ddy = abs(pts[i].y_mm - pts[i - 1].y_mm);
    check((ddx == 0) != (ddy == 0), "each step moves on exactly one axis");
    check((ddx + ddy) <= (PATH_STEP_MM + 1), "step does not exceed PATH_STEP_MM");
  }

  /* ------------------------------------------------------------------------
   * Trial 2, (0.70 to the LEFT, 0.70): same shape, opposite sign.
   * ---------------------------------------------------------------------- */
  memset(pts, 0, sizeof(pts));
  n = path_generate_manhattan(0, 0, -700, 700, pts, Q_PATH_CAPACITY);
  printf("manhattan (0,0)->(-700,700): n=%d\n", n);
  check(n == 14, "(0,0)->(-700,700) is 7 + 7 points");
  check(pts[6].x_mm == 0 && pts[6].y_mm == 700, "left trial corner is (0,700)");
  check(pts[n - 1].x_mm == -700 && pts[n - 1].y_mm == 700, "left trial target is exact");
  for (i = 7; i < n; i++)
  {
    check(pts[i].x_mm == -(i - 6) * 100, "leg 2 runs towards -x");
  }

  /* A target purely to the side: no y leg at all, one 90 degree turn. */
  memset(pts, 0, sizeof(pts));
  n = path_generate_manhattan(0, 0, 300, 0, pts, Q_PATH_CAPACITY);
  check(n == 3, "lateral-only target gives 3 points");
  for (i = 0; i < n; i++)
  {
    check(pts[i].y_mm == 0, "lateral-only path keeps y = 0");
  }

  /* Capacity: 8 + 8 = 16 fits exactly, 9 + 9 = 18 is refused. */
  n = path_generate_manhattan(0, 0, 800, 800, pts, Q_PATH_CAPACITY);
  check(n == 16, "800 + 800 mm fits exactly in 16 slots");
  n = path_generate_manhattan(0, 0, 900, 900, pts, Q_PATH_CAPACITY);
  check(n == -2, "900 + 900 mm does not fit in 16 slots -> refused");

  /* Degenerate and NULL cases. */
  n = path_generate_manhattan(100, 100, 100, 100, pts, Q_PATH_CAPACITY);
  check(n == 0, "zero-length Manhattan path yields no waypoints");
  check(path_generate_manhattan(0, 0, 10, 10, NULL, 4) == -1, "NULL output refused");
  check(path_generate_manhattan(0, 0, 10, 10, pts, 0) == -1, "zero capacity refused");
}

int main(void)
{
  test_generate();
  test_manhattan();
  test_arrive();
  test_track();

  printf("\n%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
