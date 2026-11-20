#include "flood_nav.h"

#include <stddef.h>
#include <string.h>

#define FLOOD_NAV_CAPACITY (FLOOD_NAV_MAX_COLS * FLOOD_NAV_MAX_ROWS)
#define FLOOD_NAV_INFINITY UINT16_MAX

static flood_nav_config_t flood_nav_config;
static flood_nav_location_t flood_nav_location;
static uint8_t flood_nav_edges[FLOOD_NAV_CAPACITY][4];
static uint16_t flood_nav_distance[FLOOD_NAV_CAPACITY];
static uint16_t flood_nav_queue[FLOOD_NAV_CAPACITY];
static uint8_t flood_nav_initialized;
static uint8_t flood_nav_location_valid;

_Static_assert(FLOOD_NAV_CAPACITY < UINT16_MAX, "BFS indices must fit uint16_t");
_Static_assert((FLOOD_NAV_MAX_COLS - 1U) * UINT16_MAX <= INT32_MAX,
               "Coordinates must fit int32_t");
_Static_assert((FLOOD_NAV_MAX_ROWS - 1U) * UINT16_MAX <= INT32_MAX,
               "Coordinates must fit int32_t");

static uint8_t flood_nav_cell_valid(flood_nav_cell_t cell,
                                   const flood_nav_config_t *config)
{
    return (uint8_t)(cell.col >= 1U && cell.col <= config->cols &&
                     cell.row >= 1U && cell.row <= config->rows);
}

static uint16_t flood_nav_index(flood_nav_cell_t cell)
{
    return (uint16_t)((cell.row - 1U) * flood_nav_config.cols + cell.col - 1U);
}

static flood_nav_cell_t flood_nav_cell(uint16_t index)
{
    flood_nav_cell_t cell;
    cell.col = (uint16_t)(index % flood_nav_config.cols + 1U);
    cell.row = (uint16_t)(index / flood_nav_config.cols + 1U);
    return cell;
}

static uint8_t flood_nav_neighbor(flood_nav_cell_t cell,
                                  flood_nav_direction_t direction,
                                  flood_nav_cell_t *neighbor)
{
    *neighbor = cell;
    switch (direction) {
    case FLOOD_NAV_NORTH:
        if (cell.row == 1U) { return 0U; }
        neighbor->row--;
        break;
    case FLOOD_NAV_EAST:
        if (cell.col == flood_nav_config.cols) { return 0U; }
        neighbor->col++;
        break;
    case FLOOD_NAV_SOUTH:
        if (cell.row == flood_nav_config.rows) { return 0U; }
        neighbor->row++;
        break;
    case FLOOD_NAV_WEST:
        if (cell.col == 1U) { return 0U; }
        neighbor->col--;
        break;
    default:
        return 0U;
    }
    return 1U;
}

flood_nav_result_t flood_nav_init(const flood_nav_config_t *config)
{
    uint16_t i;
    uint16_t cells;
    if (config == NULL || config->cols == 0U || config->rows == 0U ||
        config->cols > FLOOD_NAV_MAX_COLS || config->rows > FLOOD_NAV_MAX_ROWS ||
        config->cell_mm == 0U ||
        (unsigned int)config->initial_heading > (unsigned int)FLOOD_NAV_WEST ||
        !flood_nav_cell_valid(config->start, config) ||
        !flood_nav_cell_valid(config->goal, config)) {
        return FLOOD_NAV_ERR_ARGUMENT;
    }
    flood_nav_config = *config;
    memset(flood_nav_edges, 0, sizeof(flood_nav_edges));
    cells = (uint16_t)(config->cols * config->rows);
    for (i = 0U; i < cells; i++) {
        unsigned int d;
        for (d = 0U; d < 4U; d++) {
            flood_nav_cell_t neighbor;
            if (!flood_nav_neighbor(flood_nav_cell(i),
                                    (flood_nav_direction_t)d, &neighbor)) {
                flood_nav_edges[i][d] = (uint8_t)FLOOD_NAV_WALL;
            }
        }
    }
    memset(&flood_nav_location, 0, sizeof(flood_nav_location));
    flood_nav_location.heading = config->initial_heading;
    flood_nav_location_valid = 0U;
    flood_nav_initialized = 1U;
    return FLOOD_NAV_READY;
}

flood_nav_result_t flood_nav_observe(flood_nav_cell_t cell,
                                   flood_nav_direction_t direction,
                                   flood_nav_edge_t edge)
{
    flood_nav_cell_t neighbor;
    if (!flood_nav_initialized) { return FLOOD_NAV_ERR_NOT_INITIALIZED; }
    if (!flood_nav_cell_valid(cell, &flood_nav_config) ||
        (unsigned int)direction > (unsigned int)FLOOD_NAV_WEST ||
        (unsigned int)edge > (unsigned int)FLOOD_NAV_WALL) {
        return FLOOD_NAV_ERR_ARGUMENT;
    }
    if (!flood_nav_neighbor(cell, direction, &neighbor)) {
        return edge == FLOOD_NAV_WALL ? FLOOD_NAV_READY : FLOOD_NAV_ERR_BOUNDARY;
    }
    flood_nav_edges[flood_nav_index(cell)][direction] = (uint8_t)edge;
    flood_nav_edges[flood_nav_index(neighbor)][((unsigned int)direction + 2U) % 4U]
        = (uint8_t)edge;
    return FLOOD_NAV_READY;
}

flood_nav_result_t flood_nav_set_location(const flood_nav_location_t *location)
{
    if (!flood_nav_initialized) { return FLOOD_NAV_ERR_NOT_INITIALIZED; }
    if (location == NULL ||
        !flood_nav_cell_valid(location->cell, &flood_nav_config) ||
        (unsigned int)location->heading > (unsigned int)FLOOD_NAV_WEST ||
        location->at_center > 1U || location->stopped > 1U) {
        return FLOOD_NAV_ERR_ARGUMENT;
    }
    flood_nav_location = *location;
    flood_nav_location_valid = 1U;
    return FLOOD_NAV_READY;
}

flood_nav_result_t flood_nav_next(flood_nav_step_t *out)
{
    uint16_t head = 0U;
    uint16_t tail = 0U;
    uint16_t cells;
    uint16_t i;
    uint16_t current;
    uint16_t goal;
    uint16_t best = FLOOD_NAV_INFINITY;
    unsigned int best_direction = 4U;
    unsigned int d;
    flood_nav_step_t step;

    if (!flood_nav_initialized) { return FLOOD_NAV_ERR_NOT_INITIALIZED; }
    if (out == NULL) { return FLOOD_NAV_ERR_ARGUMENT; }
    if (!flood_nav_location_valid) { return FLOOD_NAV_NEED_LOCATION; }
    if (!flood_nav_location.at_center) { return FLOOD_NAV_WAIT_CENTER; }
    if (!flood_nav_location.stopped) { return FLOOD_NAV_WAIT_STOP; }
    current = flood_nav_index(flood_nav_location.cell);
    goal = flood_nav_index(flood_nav_config.goal);
    if (current == goal) { return FLOOD_NAV_FINISHED; }

    cells = (uint16_t)(flood_nav_config.cols * flood_nav_config.rows);
    for (i = 0U; i < cells; i++) { flood_nav_distance[i] = FLOOD_NAV_INFINITY; }
    flood_nav_distance[goal] = 0U;
    flood_nav_queue[tail++] = goal;
    /* Each cell is enqueued once; at most cells dequeues and 4*cells edges. */
    while (head < tail) {
        uint16_t index = flood_nav_queue[head++];
        for (d = 0U; d < 4U; d++) {
            flood_nav_cell_t neighbor;
            uint16_t next;
            if (flood_nav_edges[index][d] == (uint8_t)FLOOD_NAV_WALL ||
                !flood_nav_neighbor(flood_nav_cell(index),
                                    (flood_nav_direction_t)d, &neighbor)) {
                continue;
            }
            next = flood_nav_index(neighbor);
            if (flood_nav_distance[next] != FLOOD_NAV_INFINITY) { continue; }
            flood_nav_distance[next] = (uint16_t)(flood_nav_distance[index] + 1U);
            flood_nav_queue[tail++] = next;
        }
    }
    if (flood_nav_distance[current] == FLOOD_NAV_INFINITY) {
        return FLOOD_NAV_NO_PATH;
    }
    for (d = 0U; d < 4U; d++) {
        flood_nav_cell_t neighbor;
        uint16_t distance;
        if (flood_nav_edges[current][d] == (uint8_t)FLOOD_NAV_WALL ||
            !flood_nav_neighbor(flood_nav_location.cell,
                                (flood_nav_direction_t)d, &neighbor)) {
            continue;
        }
        distance = flood_nav_distance[flood_nav_index(neighbor)];
        if (distance >= flood_nav_distance[current]) { continue; }
        if (distance < best || (distance == best &&
            d == (unsigned int)flood_nav_location.heading)) {
            best = distance;
            best_direction = d;
            step.cell = neighbor;
        }
    }
    if (best_direction == 4U) { return FLOOD_NAV_NO_PATH; }
    step.direction = (flood_nav_direction_t)best_direction;
    step.east_mm = ((int32_t)step.cell.col - (int32_t)flood_nav_config.start.col)
                    * (int32_t)flood_nav_config.cell_mm;
    step.north_mm = ((int32_t)flood_nav_config.start.row - (int32_t)step.cell.row)
                     * (int32_t)flood_nav_config.cell_mm;
    step.is_goal = (uint8_t)(flood_nav_index(step.cell) == goal);
    *out = step;
    return flood_nav_edges[current][best_direction] == (uint8_t)FLOOD_NAV_OPEN
        ? FLOOD_NAV_READY : FLOOD_NAV_NEED_OBSERVATION;
}
