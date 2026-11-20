#include "lidar_guard.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_PI 3.14159265358979323846
static unsigned long checks;
static lidar_guard_scan_t scan;
static uint32_t seed = UINT32_C(0x17ad84);
#define CHECK(e) do { checks++; if (!(e)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #e); exit(EXIT_FAILURE); \
} } while (0)

static double random_unit(void)
{
    seed = seed * UINT32_C(1664525) + UINT32_C(1013904223);
    return (double)(seed >> 8) / 16777216.0;
}

static void clear_scan(float range)
{
    unsigned int i;
    for (i = 0U; i < LIDAR_GUARD_BINS; i++) {
        scan.range_mm[i] = range;
        scan.age_ms[i] = 0U;
        scan.state[i] = (uint8_t)LIDAR_GUARD_CLEAR;
    }
}

static lidar_guard_config_t geometry_config(void)
{
    lidar_guard_config_t config;
    lidar_guard_default_config(&config);
    config.clearance_mm = 0.0f;
    config.range_error_mm = 0.0f;
    config.reaction_ms = 0.0f;
    config.max_age_ms = 1U;
    return config;
}

static void hit(unsigned int angle, float range)
{
    scan.state[angle] = (uint8_t)LIDAR_GUARD_HIT;
    scan.range_mm[angle] = range;
}

static void test_geometry(void)
{
    lidar_guard_config_t config = geometry_config();
    lidar_guard_motion_t motion = {0};
    clear_scan(5000.0f);
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_ALLOW);
    hit(0U, 145.0f);
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_STOP_OBSTACLE);
    hit(0U, 148.0f);
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_ALLOW);
    clear_scan(5000.0f);
    hit(180U, 55.0f);
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_STOP_OBSTACLE);
    hit(180U, 58.0f);
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_ALLOW);
    clear_scan(5000.0f);
    hit(90U, 87.0f);
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_STOP_OBSTACLE);
    hit(90U, 95.0f);
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_ALLOW);
    /* Full turn: backside shell is swept all the way out to radius 145. */
    hit(180U, 140.0f);
    motion.requested_angular_deg_s = 360.0f;
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_STOP_OBSTACLE);
    motion.requested_angular_deg_s = -360.0f;
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_STOP_OBSTACLE);
    clear_scan(5000.0f);
    hit(180U, 148.0f);
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_ALLOW);
    motion.requested_angular_deg_s = 0.0f;
    motion.requested_linear_mm_s = 200.0f;
    clear_scan(5000.0f);
    hit(0U, 230.0f);
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_STOP_OBSTACLE);
    hit(0U, 260.0f);
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_ALLOW);
    motion.requested_linear_mm_s = -200.0f;
    clear_scan(5000.0f);
    hit(180U, 140.0f);
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_STOP_OBSTACLE);
    /* Reverse request cannot hide the old forward speed. */
    clear_scan(5000.0f);
    hit(0U, 230.0f);
    motion.measured_linear_mm_s = 200.0f;
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_STOP_OBSTACLE);
    motion.requested_linear_mm_s = 0.0f;
    motion.requested_angular_deg_s = 90.0f;
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_STOP_OBSTACLE);
    motion.measured_linear_mm_s = 0.0f;
    motion.requested_angular_deg_s = 0.0f;
    config.clearance_mm = 20.0f;
    config.range_error_mm = 30.0f;
    clear_scan(5000.0f);
    hit(0U, 190.0f);
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_STOP_OBSTACLE);
}

static void test_evidence_and_errors(void)
{
    lidar_guard_config_t config = geometry_config();
    lidar_guard_motion_t motion = {0};
    unsigned int i;
    clear_scan(5000.0f);
    CHECK(lidar_guard_evaluate(NULL, &scan, &motion) == LIDAR_GUARD_ERR_ARGUMENT);
    CHECK(lidar_guard_evaluate(&config, NULL, &motion) == LIDAR_GUARD_ERR_ARGUMENT);
    CHECK(lidar_guard_evaluate(&config, &scan, NULL) == LIDAR_GUARD_ERR_ARGUMENT);
    lidar_guard_default_config(NULL);
    for (i = 0U; i < LIDAR_GUARD_BINS; i++) {
        scan.state[i] = (uint8_t)LIDAR_GUARD_MISSING;
        CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_STOP_COVERAGE);
        scan.state[i] = (uint8_t)LIDAR_GUARD_CLEAR;
        scan.age_ms[i] = 2U;
        CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_STOP_STALE);
        scan.age_ms[i] = 1U;
        CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_ALLOW);
        scan.age_ms[i] = 0U;
    }
    scan.range_mm[0] = 50.0f;
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_STOP_COVERAGE);
    scan.range_mm[0] = 0.0f;
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_ERR_ARGUMENT);
    scan.range_mm[0] = (float)NAN;
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_ERR_ARGUMENT);
    scan.range_mm[0] = (float)INFINITY;
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_ERR_ARGUMENT);
    clear_scan(5000.0f);
    motion.measured_linear_mm_s = (float)NAN;
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_ERR_ARGUMENT);
    motion.measured_linear_mm_s = 1001.0f;
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_ERR_ARGUMENT);
    motion.measured_linear_mm_s = 0.0f;
    motion.requested_angular_deg_s = (float)INFINITY;
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_ERR_ARGUMENT);
    motion.requested_angular_deg_s = 0.0f;
#define BAD(field, value) do { \
    config = geometry_config(); config.field = (value); \
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_ERR_ARGUMENT); \
} while (0)
    BAD(body_radius_mm, 0.0f);
    BAD(body_radius_mm, (float)NAN);
    BAD(center_forward_mm, 100.0f);
    BAD(center_forward_mm, (float)INFINITY);
    BAD(clearance_mm, -1.0f);
    BAD(range_error_mm, (float)NAN);
    BAD(reaction_ms, -1.0f);
    BAD(brake_mm_s2, 0.0f);
    BAD(brake_deg_s2, 0.0f);
    BAD(max_linear_mm_s, (float)NAN);
    BAD(max_angular_deg_s, 0.0f);
    BAD(max_age_ms, 0U);
#undef BAD
    config = geometry_config();
    scan.state[1] = 99U;
    hit(0U, 100.0f);
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_STOP_OBSTACLE);
    clear_scan(5000.0f);
    motion.requested_linear_mm_s = 200.0f;
    hit(0U, 300.0f);
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_ALLOW);
    config.reaction_ms = 1000.0f;
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_STOP_OBSTACLE);
    config = geometry_config();
    config.max_age_ms = 1000U;
    CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_STOP_OBSTACLE);
}

/* Independent physical integration: generate points ON the real moving body
 * while linear and angular velocities decelerate independently. Every such
 * point must be rejected, including mixed-sign commands and bin quantization.
 */
static void test_swept_body(void)
{
    unsigned int run;
    lidar_guard_config_t config = geometry_config();
    config.reaction_ms = 100.0f;
    config.max_age_ms = 100U;
    for (run = 0U; run < 120U; run++) {
        lidar_guard_motion_t motion = {0};
        double v = (random_unit() * 2.0 - 1.0) * 900.0;
        double w = (random_unit() * 2.0 - 1.0) * 350.0;
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        double time = 0.0;
        const double dt = 0.005;
        unsigned int sample;
        motion.measured_linear_mm_s = (float)v;
        motion.measured_angular_deg_s = (float)w;
        motion.requested_linear_mm_s = (float)(-v * 0.5);
        motion.requested_angular_deg_s = (float)(-w * 0.5);
        /* Integration starts from exactly the values passed to the API. */
        v = motion.measured_linear_mm_s;
        w = motion.measured_angular_deg_s;
        for (sample = 0U; sample < 1300U; sample++) {
            double speed = copysign(fmax(0.0, fabs(v) - (double)config.brake_mm_s2 * fmax(0.0, time - 0.2)), v);
            double turn = copysign(fmax(0.0, fabs(w) - (double)config.brake_deg_s2 * fmax(0.0, time - 0.2)), w);
            /* Midpoint integration, fine enough to bound numerical drift. */
            double mid_yaw = yaw + turn * TEST_PI / 180.0 * dt / 2.0;
            x += speed * cos(mid_yaw) * dt;
            y += speed * sin(mid_yaw) * dt;
            yaw += turn * TEST_PI / 180.0 * dt;
            time += dt;
            if (sample % 25U == 0U) {
                double surface = random_unit() * 2.0 * TEST_PI;
                double px = x + 45.0 * cos(yaw) + 99.0 * cos(surface);
                double py = y + 45.0 * sin(yaw) + 99.0 * sin(surface);
                double range = hypot(px, py);
                int bin = (int)lround(atan2(py, px) * 180.0 / TEST_PI);
                unsigned int angle = (unsigned int)((bin + 360) % 360);
                if (range < 0.01) { continue; }
                clear_scan(50000.0f);
                hit(angle, (float)range);
                CHECK(lidar_guard_evaluate(&config, &scan, &motion) == LIDAR_GUARD_STOP_OBSTACLE);
            }
            if (speed == 0.0 && turn == 0.0) { break; }
        }
    }
}

int main(void)
{
    test_geometry();
    test_evidence_and_errors();
    test_swept_body();
    printf("PASS: %lu checks; offset body, stopping distance, freshness, 120 integrated braking trajectories.\n", checks);
    return EXIT_SUCCESS;
}
