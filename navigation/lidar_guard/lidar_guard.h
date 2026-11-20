#ifndef LIDAR_GUARD_H
#define LIDAR_GUARD_H

#include <stdint.h>

#define LIDAR_GUARD_BINS 360U

/* Body frame: origin = lidar = axle midpoint; x FORWARD, y LEFT.
 * Beam i points i degrees counterclockwise from the vehicle's front.
 * Sensor mounting rotation/direction must be converted by the caller.
 * Standalone C11, no hardware/RTOS/global state; caller owns all objects.
 */
typedef struct {
    float body_radius_mm;        /* Confirmed chassis: 100. */
    float center_forward_mm;     /* Confirmed chassis: +45. */
    float clearance_mm;          /* Extra body clearance. */
    float range_error_mm;        /* Bound on range/registration error. */
    float reaction_ms;           /* Additional processing/actuation delay. */
    float brake_mm_s2;           /* Guaranteed minimum linear deceleration. */
    float brake_deg_s2;          /* Guaranteed minimum angular deceleration. */
    float max_linear_mm_s;
    float max_angular_deg_s;
    uint32_t max_age_ms;
} lidar_guard_config_t;

typedef enum {
    LIDAR_GUARD_MISSING = 0,
    LIDAR_GUARD_HIT = 1,
    /* Explicit verified free ray to range_mm; zero return is NOT free space. */
    LIDAR_GUARD_CLEAR = 2
} lidar_guard_ray_state_t;

typedef struct {
    float range_mm[LIDAR_GUARD_BINS];
    uint32_t age_ms[LIDAR_GUARD_BINS]; /* Age at evaluate time, per ray. */
    uint8_t state[LIDAR_GUARD_BINS];
} lidar_guard_scan_t;

typedef struct {
    float measured_linear_mm_s;    /* Signed: forward +, reverse -. */
    float measured_angular_deg_s;  /* Signed: counterclockwise +. */
    float requested_linear_mm_s;
    float requested_angular_deg_s;
} lidar_guard_motion_t;

typedef enum {
    LIDAR_GUARD_ALLOW = 0,
    LIDAR_GUARD_STOP_OBSTACLE = 1,
    LIDAR_GUARD_STOP_COVERAGE = 2,
    LIDAR_GUARD_STOP_STALE = 3,
    LIDAR_GUARD_ERR_ARGUMENT = -1
} lidar_guard_result_t;

/* Defaults include engineering PLACEHOLDERS for delay/deceleration/error.
 * Must calibrate on hardware before permission is used to command motors. */
void lidar_guard_default_config(lidar_guard_config_t *config);

/* Checks a conservative envelope for measured AND requested motion, including
 * changing direction. Any non-ALLOW result means reject motion/request stop.
 * Does not stop a motor itself. All 360 bins require valid, fresh evidence.
 * Rays must be registered into the current body frame by the caller.
 * Ray ranges must cover the envelope; angular gaps are conservatively padded
 * for 1 degree bins. Fixed work: 360 iterations, no waiting, allocation or I/O.
 * Configured deceleration must remain achievable, speeds must not exceed the
 * larger measured/requested magnitudes during braking. Caller must check new
 * scans every control step; this is not an autonomous safety controller.
 */
lidar_guard_result_t lidar_guard_evaluate(const lidar_guard_config_t *config,
                                        const lidar_guard_scan_t *scan,
                                        const lidar_guard_motion_t *motion);

#endif
