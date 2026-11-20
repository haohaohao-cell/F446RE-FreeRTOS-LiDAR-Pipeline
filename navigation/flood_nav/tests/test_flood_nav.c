#include "flood_nav.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned long checks;
#define CHECK(expression) do { \
    checks++; \
    if (!(expression)) { \
        fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #expression); \
        exit(EXIT_FAILURE); \
    } \
} while (0)

#define TEST_MAX (FLOOD_NAV_MAX_COLS * FLOOD_NAV_MAX_ROWS)
static uint8_t truth[TEST_MAX][4];
static int reference_distance[TEST_MAX];
static unsigned int reference_queue[TEST_MAX];
static uint32_t random_state = UINT32_C(0x3b9a71);
static const int dc[4] = {0, 1, 0, -1};
static const int dr[4] = {-1, 0, 1, 0};

static uint32_t test_random(void)
{
    random_state = random_state * UINT32_C(1664525) + UINT32_C(1013904223);
    return random_state;
}

static flood_nav_config_t config_of(uint16_t cols, uint16_t rows)
{
    flood_nav_config_t config = {0};
    config.cols = cols;
    config.rows = rows;
    config.cell_mm = 600U;
    config.start.col = 1U;
    config.start.row = rows;
    config.goal.col = cols;
    config.goal.row = 1U;
    config.initial_heading = FLOOD_NAV_NORTH;
    return config;
}

static flood_nav_cell_t cell_of(unsigned int index, unsigned int cols)
{
    flood_nav_cell_t cell;
    cell.col = (uint16_t)(index % cols + 1U);
    cell.row = (uint16_t)(index / cols + 1U);
    return cell;
}

static void locate(flood_nav_cell_t cell, flood_nav_direction_t heading,
                   uint8_t center, uint8_t stopped)
{
    flood_nav_location_t location = {0};
    location.cell = cell;
    location.heading = heading;
    location.at_center = center;
    location.stopped = stopped;
    CHECK(flood_nav_set_location(&location) == FLOOD_NAV_READY);
}

static void open_map(const flood_nav_config_t *config)
{
    unsigned int i;
    for (i = 0U; i < (unsigned int)config->cols * config->rows; i++) {
        flood_nav_cell_t cell = cell_of(i, config->cols);
        if (cell.col < config->cols) {
            CHECK(flood_nav_observe(cell, FLOOD_NAV_EAST, FLOOD_NAV_OPEN) == FLOOD_NAV_READY);
        }
        if (cell.row < config->rows) {
            CHECK(flood_nav_observe(cell, FLOOD_NAV_SOUTH, FLOOD_NAV_OPEN) == FLOOD_NAV_READY);
        }
    }
}

static void test_api_and_directions(void)
{
    flood_nav_config_t config = config_of(3U, 3U);
    flood_nav_step_t step;
    flood_nav_step_t sentinel;
    flood_nav_location_t invalid = {0};
    unsigned int d;
    memset(&sentinel, 0x5a, sizeof(sentinel));
    step = sentinel;
    CHECK(flood_nav_next(&step) == FLOOD_NAV_ERR_NOT_INITIALIZED);
    CHECK(flood_nav_observe(config.start, FLOOD_NAV_NORTH, FLOOD_NAV_OPEN) == FLOOD_NAV_ERR_NOT_INITIALIZED);
    CHECK(flood_nav_set_location(&invalid) == FLOOD_NAV_ERR_NOT_INITIALIZED);
    CHECK(flood_nav_init(NULL) == FLOOD_NAV_ERR_ARGUMENT);
    CHECK(flood_nav_init(&config) == FLOOD_NAV_READY);
    CHECK(flood_nav_next(NULL) == FLOOD_NAV_ERR_ARGUMENT);
    CHECK(flood_nav_next(&step) == FLOOD_NAV_NEED_LOCATION);
    CHECK(memcmp(&step, &sentinel, sizeof(step)) == 0);
    CHECK(flood_nav_set_location(NULL) == FLOOD_NAV_ERR_ARGUMENT);
    CHECK(flood_nav_set_location(&invalid) == FLOOD_NAV_ERR_ARGUMENT);
    CHECK(flood_nav_observe(config.start, (flood_nav_direction_t)-1, FLOOD_NAV_OPEN) == FLOOD_NAV_ERR_ARGUMENT);
    CHECK(flood_nav_observe(config.start, FLOOD_NAV_NORTH, (flood_nav_edge_t)-1) == FLOOD_NAV_ERR_ARGUMENT);
    CHECK(flood_nav_observe(config.start, FLOOD_NAV_WEST, FLOOD_NAV_OPEN) == FLOOD_NAV_ERR_BOUNDARY);
    CHECK(flood_nav_observe(config.start, FLOOD_NAV_SOUTH, FLOOD_NAV_UNKNOWN) == FLOOD_NAV_ERR_BOUNDARY);
    CHECK(flood_nav_observe(config.start, FLOOD_NAV_SOUTH, FLOOD_NAV_WALL) == FLOOD_NAV_READY);
    locate(config.start, config.initial_heading, 0U, 0U);
    CHECK(flood_nav_next(&step) == FLOOD_NAV_WAIT_CENTER);
    locate(config.start, config.initial_heading, 1U, 0U);
    CHECK(flood_nav_next(&step) == FLOOD_NAV_WAIT_STOP);
    locate(config.start, config.initial_heading, 1U, 1U);
    CHECK(flood_nav_next(&step) == FLOOD_NAV_NEED_OBSERVATION);
    CHECK(step.direction == FLOOD_NAV_NORTH);
    CHECK(flood_nav_observe(config.start, step.direction, FLOOD_NAV_OPEN) == FLOOD_NAV_READY);
    CHECK(flood_nav_next(&step) == FLOOD_NAV_READY);
    CHECK(step.north_mm == 600 && step.east_mm == 0);
    /* Invalid init is transactional: last valid map and location survive. */
    config.cols = 0U;
    CHECK(flood_nav_init(&config) == FLOOD_NAV_ERR_ARGUMENT);
    CHECK(flood_nav_next(&step) == FLOOD_NAV_READY);
    config = config_of(3U, 3U);
    config.rows = (uint16_t)(FLOOD_NAV_MAX_ROWS + 1U);
    CHECK(flood_nav_init(&config) == FLOOD_NAV_ERR_ARGUMENT);
    config = config_of(3U, 3U);
    config.cols = (uint16_t)(FLOOD_NAV_MAX_COLS + 1U);
    CHECK(flood_nav_init(&config) == FLOOD_NAV_ERR_ARGUMENT);
    config = config_of(3U, 3U);
    config.cell_mm = 0U;
    CHECK(flood_nav_init(&config) == FLOOD_NAV_ERR_ARGUMENT);
    config = config_of(3U, 3U);
    config.goal.col = 0U;
    CHECK(flood_nav_init(&config) == FLOOD_NAV_ERR_ARGUMENT);
    config.goal.col = 4U;
    CHECK(flood_nav_init(&config) == FLOOD_NAV_ERR_ARGUMENT);
    config = config_of(3U, 3U);
    config.initial_heading = (flood_nav_direction_t)-1;
    CHECK(flood_nav_init(&config) == FLOOD_NAV_ERR_ARGUMENT);

    for (d = 0U; d < 4U; d++) {
        config = config_of(3U, 3U);
        config.start.col = 2U;
        config.start.row = 2U;
        config.goal.col = (uint16_t)(2 + dc[d]);
        config.goal.row = (uint16_t)(2 + dr[d]);
        config.initial_heading = (flood_nav_direction_t)d;
        CHECK(flood_nav_init(&config) == FLOOD_NAV_READY);
        locate(config.start, config.initial_heading, 1U, 1U);
        CHECK(flood_nav_observe(config.start, (flood_nav_direction_t)d, FLOOD_NAV_OPEN) == FLOOD_NAV_READY);
        CHECK(flood_nav_next(&step) == FLOOD_NAV_READY);
        CHECK(step.direction == (flood_nav_direction_t)d);
        CHECK(step.cell.col == config.goal.col && step.cell.row == config.goal.row);
        CHECK(step.east_mm == dc[d] * 600 && step.north_mm == -dr[d] * 600);
        CHECK(step.is_goal == 1U);
        locate(config.goal, config.initial_heading, 0U, 0U);
        CHECK(flood_nav_next(&step) == FLOOD_NAV_WAIT_CENTER);
        locate(config.goal, config.initial_heading, 1U, 0U);
        CHECK(flood_nav_next(&step) == FLOOD_NAV_WAIT_STOP);
        locate(config.goal, config.initial_heading, 1U, 1U);
        step = sentinel;
        CHECK(flood_nav_next(&step) == FLOOD_NAV_FINISHED);
        CHECK(memcmp(&step, &sentinel, sizeof(step)) == 0);
    }
    config = config_of(1U, 1U);
    CHECK(flood_nav_init(&config) == FLOOD_NAV_READY);
    locate(config.start, FLOOD_NAV_NORTH, 1U, 1U);
    CHECK(flood_nav_next(&step) == FLOOD_NAV_FINISHED);
}

static void test_symmetry_and_replanning(void)
{
    flood_nav_config_t config = config_of(2U, 1U);
    flood_nav_step_t step;
    CHECK(flood_nav_init(&config) == FLOOD_NAV_READY);
    locate(config.start, FLOOD_NAV_EAST, 1U, 1U);
    CHECK(flood_nav_next(&step) == FLOOD_NAV_NEED_OBSERVATION);
    /* Observe from the OTHER side; both directions must change. */
    CHECK(flood_nav_observe(config.goal, FLOOD_NAV_WEST, FLOOD_NAV_WALL) == FLOOD_NAV_READY);
    CHECK(flood_nav_next(&step) == FLOOD_NAV_NO_PATH);
    CHECK(flood_nav_observe(config.goal, FLOOD_NAV_WEST, FLOOD_NAV_OPEN) == FLOOD_NAV_READY);
    CHECK(flood_nav_next(&step) == FLOOD_NAV_READY);
    CHECK(flood_nav_observe(config.goal, FLOOD_NAV_WEST, FLOOD_NAV_UNKNOWN) == FLOOD_NAV_READY);
    CHECK(flood_nav_next(&step) == FLOOD_NAV_NEED_OBSERVATION);
    config = config_of(2U, 2U);
    CHECK(flood_nav_init(&config) == FLOOD_NAV_READY);
    open_map(&config);
    locate(config.start, FLOOD_NAV_NORTH, 1U, 1U);
    CHECK(flood_nav_next(&step) == FLOOD_NAV_READY && step.direction == FLOOD_NAV_NORTH);
    CHECK(flood_nav_observe(config.start, FLOOD_NAV_NORTH, FLOOD_NAV_WALL) == FLOOD_NAV_READY);
    CHECK(flood_nav_next(&step) == FLOOD_NAV_READY && step.direction == FLOOD_NAV_EAST);
    locate(config.start, FLOOD_NAV_EAST, 1U, 1U);
    CHECK(flood_nav_next(&step) == FLOOD_NAV_READY && step.direction == FLOOD_NAV_EAST);
    CHECK(flood_nav_init(&config) == FLOOD_NAV_READY);
    locate(config.start, FLOOD_NAV_EAST, 1U, 1U);
    CHECK(flood_nav_next(&step) == FLOOD_NAV_NEED_OBSERVATION);
}

/* Independent reverse BFS in row/column arithmetic, against a truth graph. */
static void reference_bfs(const flood_nav_config_t *config)
{
    unsigned int count = (unsigned int)config->rows * config->cols;
    unsigned int goal = (unsigned int)(config->goal.row - 1U) * config->cols + config->goal.col - 1U;
    unsigned int head = 0U;
    unsigned int tail = 0U;
    unsigned int i;
    for (i = 0U; i < count; i++) { reference_distance[i] = -1; }
    reference_distance[goal] = 0;
    reference_queue[tail++] = goal;
    while (head < tail) {
        unsigned int current = reference_queue[head++];
        unsigned int d;
        for (d = 0U; d < 4U; d++) {
            int col = (int)(current % config->cols) + dc[d];
            int row = (int)(current / config->cols) + dr[d];
            unsigned int next;
            if (col < 0 || col >= config->cols || row < 0 || row >= config->rows ||
                truth[current][d] != (uint8_t)FLOOD_NAV_OPEN) { continue; }
            next = (unsigned int)row * config->cols + (unsigned int)col;
            if (reference_distance[next] >= 0) { continue; }
            reference_distance[next] = reference_distance[current] + 1;
            reference_queue[tail++] = next;
        }
    }
}

static void build_truth(const flood_nav_config_t *config, uint8_t snake)
{
    unsigned int count = (unsigned int)config->rows * config->cols;
    unsigned int i;
    memset(truth, (int)FLOOD_NAV_WALL, sizeof(truth));
    for (i = 0U; i < count; i++) {
        flood_nav_cell_t cell = cell_of(i, config->cols);
        if (cell.col < config->cols) {
            uint8_t edge = (uint8_t)(snake || test_random() % 100U < 62U
                                     ? FLOOD_NAV_OPEN : FLOOD_NAV_WALL);
            truth[i][FLOOD_NAV_EAST] = edge;
            truth[i + 1U][FLOOD_NAV_WEST] = edge;
        }
        if (cell.row < config->rows) {
            uint8_t edge = (uint8_t)(snake
                ? (((cell.row % 2U == 1U && cell.col == config->cols) ||
                    (cell.row % 2U == 0U && cell.col == 1U)) ? FLOOD_NAV_OPEN : FLOOD_NAV_WALL)
                : (test_random() % 100U < 62U ? FLOOD_NAV_OPEN : FLOOD_NAV_WALL));
            truth[i][FLOOD_NAV_SOUTH] = edge;
            truth[i + config->cols][FLOOD_NAV_NORTH] = edge;
        }
    }
}

static void reveal(const flood_nav_config_t *config, flood_nav_cell_t cell)
{
    unsigned int index = (unsigned int)(cell.row - 1U) * config->cols + cell.col - 1U;
    unsigned int d;
    for (d = 0U; d < 4U; d++) {
        CHECK(flood_nav_observe(cell, (flood_nav_direction_t)d,
                                (flood_nav_edge_t)truth[index][d]) == FLOOD_NAV_READY);
    }
}

static void test_reference_maps(void)
{
    unsigned int run;
    for (run = 0U; run < 100U; run++) {
        flood_nav_config_t config = config_of((uint16_t)(test_random() % 16U + 1U),
                                              (uint16_t)(test_random() % 13U + 1U));
        unsigned int count = (unsigned int)config.cols * config.rows;
        unsigned int i;
        build_truth(&config, 0U);
        reference_bfs(&config);
        CHECK(flood_nav_init(&config) == FLOOD_NAV_READY);
        for (i = 0U; i < count; i++) { reveal(&config, cell_of(i, config.cols)); }
        for (i = 0U; i < count; i++) {
            flood_nav_step_t step;
            flood_nav_cell_t current = cell_of(i, config.cols);
            flood_nav_result_t result;
            locate(current, (flood_nav_direction_t)(i % 4U), 1U, 1U);
            result = flood_nav_next(&step);
            if (reference_distance[i] == 0) {
                CHECK(result == FLOOD_NAV_FINISHED);
            } else if (reference_distance[i] < 0) {
                CHECK(result == FLOOD_NAV_NO_PATH);
            } else {
                unsigned int next = (unsigned int)(step.cell.row - 1U) * config.cols + step.cell.col - 1U;
                CHECK(result == FLOOD_NAV_READY);
                CHECK(next < count);
                CHECK(truth[i][step.direction] == (uint8_t)FLOOD_NAV_OPEN);
                CHECK(reference_distance[next] == reference_distance[i] - 1);
            }
        }
    }
}

static void test_online_and_maximum(void)
{
    unsigned int run;
    for (run = 0U; run < 21U; run++) {
        uint8_t maximum = (uint8_t)(run == 20U);
        flood_nav_config_t config = config_of(maximum ? FLOOD_NAV_MAX_COLS : 7U,
                                              maximum ? FLOOD_NAV_MAX_ROWS : 5U);
        flood_nav_cell_t current;
        unsigned int steps;
        unsigned int limit = (unsigned int)config.cols * config.rows * 8U;
        flood_nav_result_t result = FLOOD_NAV_READY;
        if (maximum) {
            config.start.col = 1U;
            config.start.row = 1U;
            config.goal.col = 1U;
            config.goal.row = config.rows;
            config.cell_mm = UINT16_MAX;
        }
        current = config.start;
        build_truth(&config, maximum);
        CHECK(flood_nav_init(&config) == FLOOD_NAV_READY);
        for (steps = 0U; steps < limit; steps++) {
            flood_nav_step_t step;
            unsigned int index;
            reveal(&config, current);
            locate(current, FLOOD_NAV_NORTH, 1U, 1U);
            result = flood_nav_next(&step);
            if (result == FLOOD_NAV_FINISHED || result == FLOOD_NAV_NO_PATH) { break; }
            CHECK(result == FLOOD_NAV_READY);
            index = (unsigned int)(current.row - 1U) * config.cols + current.col - 1U;
            CHECK(truth[index][step.direction] == (uint8_t)FLOOD_NAV_OPEN);
            CHECK(step.cell.col == (uint16_t)((int)current.col + dc[step.direction]));
            CHECK(step.cell.row == (uint16_t)((int)current.row + dr[step.direction]));
            CHECK(step.east_mm == ((int32_t)step.cell.col - config.start.col) * config.cell_mm);
            CHECK(step.north_mm == ((int32_t)config.start.row - step.cell.row) * config.cell_mm);
            current = step.cell;
        }
        CHECK(steps < limit);
        reference_bfs(&config);
        {
            unsigned int start = (unsigned int)(config.start.row - 1U) * config.cols + config.start.col - 1U;
            CHECK((result == FLOOD_NAV_FINISHED) == (reference_distance[start] >= 0));
        }
        if (maximum) { CHECK(steps == TEST_MAX - 1U); }
    }
    {
        flood_nav_config_t config = config_of(FLOOD_NAV_MAX_COLS, FLOOD_NAV_MAX_ROWS);
        flood_nav_step_t step;
        CHECK(flood_nav_init(&config) == FLOOD_NAV_READY);
        open_map(&config); /* Entire BFS queue capacity is exercised. */
        locate(config.start, FLOOD_NAV_NORTH, 1U, 1U);
        CHECK(flood_nav_next(&step) == FLOOD_NAV_READY);
    }
}

int main(void)
{
    test_api_and_directions();
    test_symmetry_and_replanning();
    test_reference_maps();
    test_online_and_maximum();
    printf("PASS: %lu checks; 100 reference maps, 21 online runs, maximum 32x32 snake.\n", checks);
    return EXIT_SUCCESS;
}
