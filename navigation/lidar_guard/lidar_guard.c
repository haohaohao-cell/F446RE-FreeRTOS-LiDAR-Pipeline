#include "lidar_guard.h"

#include <math.h>
#include <stddef.h>

#define LIDAR_GUARD_RAD 0.01745329251994329577
#define LIDAR_GUARD_PI 3.14159265358979323846

typedef struct {
    double travel;
    double yaw;
    double base_radius;
    double offset;
    double padding;
    double extent;
    double sign;
} lidar_guard_envelope_t;

void lidar_guard_default_config(lidar_guard_config_t *config)
{
    if (config == NULL) { return; }
    config->body_radius_mm = 100.0f;
    config->center_forward_mm = 45.0f;
    config->clearance_mm = 20.0f;
    config->range_error_mm = 30.0f;
    config->reaction_ms = 200.0f;
    config->brake_mm_s2 = 200.0f;
    config->brake_deg_s2 = 180.0f;
    config->max_linear_mm_s = 1000.0f;
    config->max_angular_deg_s = 360.0f;
    config->max_age_ms = 200U;
}

static int lidar_guard_config_valid(const lidar_guard_config_t *c)
{
    return isfinite(c->body_radius_mm) && c->body_radius_mm > 0.0f && c->body_radius_mm <= 10000.0f &&
        isfinite(c->center_forward_mm) && fabsf(c->center_forward_mm) < c->body_radius_mm &&
        isfinite(c->clearance_mm) && c->clearance_mm >= 0.0f && c->clearance_mm <= 10000.0f &&
        isfinite(c->range_error_mm) && c->range_error_mm >= 0.0f && c->range_error_mm <= 10000.0f &&
        isfinite(c->reaction_ms) && c->reaction_ms >= 0.0f && c->reaction_ms <= 60000.0f &&
        isfinite(c->brake_mm_s2) && c->brake_mm_s2 >= 0.001f && c->brake_mm_s2 <= 1000000.0f &&
        isfinite(c->brake_deg_s2) && c->brake_deg_s2 >= 0.001f && c->brake_deg_s2 <= 1000000.0f &&
        isfinite(c->max_linear_mm_s) && c->max_linear_mm_s > 0.0f && c->max_linear_mm_s <= 100000.0f &&
        isfinite(c->max_angular_deg_s) && c->max_angular_deg_s > 0.0f && c->max_angular_deg_s <= 100000.0f &&
        c->max_age_ms > 0U && c->max_age_ms <= 60000U;
}

static lidar_guard_envelope_t lidar_guard_envelope(const lidar_guard_config_t *c,
                                                   double v, double w)
{
    lidar_guard_envelope_t e;
    double delay = ((double)c->reaction_ms + (double)c->max_age_ms) / 1000.0;
    double speed = fabs(v);
    double turn = fabs(w);
    double error;
    e.travel = speed * delay + speed * speed / (2.0 * (double)c->brake_mm_s2);
    e.yaw = (turn * delay + turn * turn / (2.0 * (double)c->brake_deg_s2)) * LIDAR_GUARD_RAD;
    e.base_radius = (double)c->body_radius_mm + c->clearance_mm + c->range_error_mm;
    e.offset = c->center_forward_mm;
    e.sign = v < 0.0 ? -1.0 : 1.0;
    /* Two bounding envelopes: a forward/reverse capsule plus yaw uncertainty,
     * and an axle-centered disk bounding all translations and orientations.
     * Their intersection still contains every possible swept body position.
     * Deviation from a straight path <= 2*D*sin(min(yaw,pi)/2).
     */
    error = 2.0 * (e.travel + fabs(e.offset)) * sin(fmin(e.yaw, LIDAR_GUARD_PI) / 2.0);
    e.padding = e.base_radius + error;
    e.extent = e.travel + fabs(e.offset) + e.base_radius;
    /* Half-bin angular footprint. Includes gaps between adjacent 1-deg rays. */
    error = e.extent * sin(LIDAR_GUARD_RAD / 2.0);
    e.padding += error;
    e.extent += error;
    return e;
}

static int lidar_guard_inside(const lidar_guard_envelope_t *e, double x, double y)
{
    double end = e->offset + e->sign * e->travel;
    double nearest = fmax(fmin(e->offset, end), fmin(fmax(e->offset, end), x));
    double dx = x - nearest;
    return x*x + y*y <= e->extent*e->extent &&
           dx*dx + y*y <= e->padding*e->padding;
}

lidar_guard_result_t lidar_guard_evaluate(const lidar_guard_config_t *config,
                                        const lidar_guard_scan_t *scan,
                                        const lidar_guard_motion_t *motion)
{
    lidar_guard_envelope_t measured;
    lidar_guard_envelope_t requested;
    unsigned int i;
    double max_v;
    double max_w;
    lidar_guard_result_t result = LIDAR_GUARD_ALLOW;
    if (config == NULL || scan == NULL || motion == NULL ||
        !lidar_guard_config_valid(config) ||
        !isfinite(motion->measured_linear_mm_s) || !isfinite(motion->requested_linear_mm_s) ||
        !isfinite(motion->measured_angular_deg_s) || !isfinite(motion->requested_angular_deg_s)) {
        return LIDAR_GUARD_ERR_ARGUMENT;
    }
    max_v = fmax(fabs((double)motion->measured_linear_mm_s), fabs((double)motion->requested_linear_mm_s));
    max_w = fmax(fabs((double)motion->measured_angular_deg_s), fabs((double)motion->requested_angular_deg_s));
    if (max_v > config->max_linear_mm_s || max_w > config->max_angular_deg_s) {
        return LIDAR_GUARD_ERR_ARGUMENT;
    }
    /* Use maximum yaw for both directions: a turn command while still moving
     * forward must not erase the forward stopping corridor (or vice versa). */
    measured = lidar_guard_envelope(config, motion->measured_linear_mm_s, max_w);
    requested = lidar_guard_envelope(config, motion->requested_linear_mm_s, max_w);
    for (i = 0U; i < LIDAR_GUARD_BINS; i++) {
        double range = scan->range_mm[i];
        double angle = (double)i * LIDAR_GUARD_RAD;
        double x;
        double y;
        if (scan->state[i] != (uint8_t)LIDAR_GUARD_HIT &&
            scan->state[i] != (uint8_t)LIDAR_GUARD_CLEAR) {
            if (result != LIDAR_GUARD_STOP_OBSTACLE) { result = LIDAR_GUARD_STOP_COVERAGE; }
            continue;
        }
        if (!isfinite(range) || range <= 0.0 || range > 1000000.0) {
            return LIDAR_GUARD_ERR_ARGUMENT;
        }
        if (scan->age_ms[i] > config->max_age_ms) {
            if (result == LIDAR_GUARD_ALLOW) { result = LIDAR_GUARD_STOP_STALE; }
            continue;
        }
        x = range * cos(angle);
        y = range * sin(angle);
        if (lidar_guard_inside(&measured, x, y) || lidar_guard_inside(&requested, x, y)) {
            if (scan->state[i] == (uint8_t)LIDAR_GUARD_HIT) {
                result = LIDAR_GUARD_STOP_OBSTACLE;
            } else if (result != LIDAR_GUARD_STOP_OBSTACLE) {
                result = LIDAR_GUARD_STOP_COVERAGE;
            }
        }
    }
    return result;
}
