#ifndef FLOOD_NAV_H
#define FLOOD_NAV_H

#include <stdint.h>

/* Independent C11 module. No firmware, RTOS, allocation or hardware dependency.
 * Single owner only: callers must serialize all calls (including getters).
 * Maximum storage is fixed; configured dimensions can be any size up to 32x32.
 */
#define FLOOD_NAV_MAX_COLS 32U
#define FLOOD_NAV_MAX_ROWS 32U

typedef enum {
    FLOOD_NAV_NORTH = 0,
    FLOOD_NAV_EAST = 1,
    FLOOD_NAV_SOUTH = 2,
    FLOOD_NAV_WEST = 3
} flood_nav_direction_t;

typedef enum {
    FLOOD_NAV_UNKNOWN = 0,
    FLOOD_NAV_OPEN = 1,
    FLOOD_NAV_WALL = 2
} flood_nav_edge_t;

typedef struct {
    uint16_t col; /* 1-based, increases RIGHT (east). */
    uint16_t row; /* 1-based, increases DOWN (south). */
} flood_nav_cell_t;

typedef struct {
    uint16_t cols;
    uint16_t rows;
    uint16_t cell_mm; /* Square cells; must be nonzero. */
    flood_nav_cell_t start;
    flood_nav_cell_t goal;
    flood_nav_direction_t initial_heading; /* Actual vehicle heading at start. */
} flood_nav_config_t;

typedef struct {
    flood_nav_cell_t cell;
    flood_nav_direction_t heading;
    uint8_t at_center; /* Exactly 0/1; caller establishes physical tolerance. */
    uint8_t stopped;   /* Exactly 0/1; caller verifies wheel speed has settled. */
} flood_nav_location_t;

typedef struct {
    flood_nav_cell_t cell;
    flood_nav_direction_t direction;
    int32_t east_mm;  /* Relative to START CELL CENTER; positive RIGHT/east. */
    int32_t north_mm; /* Relative to START CELL CENTER; positive UP/north. */
    uint8_t is_goal;
} flood_nav_step_t;

typedef enum {
    FLOOD_NAV_READY = 0,
    FLOOD_NAV_NEED_LOCATION = 1,
    FLOOD_NAV_WAIT_CENTER = 2,
    FLOOD_NAV_WAIT_STOP = 3,
    FLOOD_NAV_NEED_OBSERVATION = 4,
    FLOOD_NAV_NO_PATH = 5,
    FLOOD_NAV_FINISHED = 6,
    FLOOD_NAV_ERR_ARGUMENT = -1,
    FLOOD_NAV_ERR_NOT_INITIALIZED = -2,
    FLOOD_NAV_ERR_BOUNDARY = -3
} flood_nav_result_t;

/* Reinitializes all state. Invalid configuration leaves prior state untouched.
 * Perimeter is WALL, internal edges UNKNOWN. Goal behavior is always stop at
 * goal center. No location is assumed valid until set_location succeeds.
 */
flood_nav_result_t flood_nav_init(const flood_nav_config_t *config);

/* One CONFIRMED absolute-map edge observation, mirrored to its neighbor.
 * Raw lidar ranges/voting/freshness belong to a future external adapter.
 * Confirmed walls can be corrected; perimeter cannot be opened or forgotten.
 */
flood_nav_result_t flood_nav_observe(flood_nav_cell_t cell,
                                   flood_nav_direction_t direction,
                                   flood_nav_edge_t edge);

flood_nav_result_t flood_nav_set_location(const flood_nav_location_t *location);

/* Recomputes reverse BFS using OPEN and UNKNOWN edges, selects a strictly
 * decreasing distance, preferring heading for equal distances then N/E/S/W.
 * READY is the ONLY permission to execute the returned step. UNKNOWN next
 * edges return NEED_OBSERVATION with a candidate to inspect, never permission
 * to drive. Output is written only for READY or NEED_OBSERVATION.
 * Planning requires stopped at current cell center. FINISHED requires goal
 * center AND stopped, not merely entering the goal cell. No waiting or I/O.
 */
flood_nav_result_t flood_nav_next(flood_nav_step_t *out);

#endif
