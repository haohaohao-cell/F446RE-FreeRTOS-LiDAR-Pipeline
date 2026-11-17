#ifndef BOARD_PINS_H
#define BOARD_PINS_H
/* Host-side stand-in for Core/Inc/board_pins.h, used ONLY by the unit tests in
 * this directory (it is found first through -I ordering and never reaches the
 * firmware build).  It carries just the constants pose.c / path.c use, with the
 * same - still unverified - values as the real header. */
#include <stdint.h>

#define BOARD_PI_F                 3.14159265f
#define BOARD_WHEEL_DIAMETER_MM    65.0f
#define BOARD_TRACK_WIDTH_MM       130.0f
#define BOARD_ENC_COUNTS_PER_REV   1320.0f
#define BOARD_MM_PER_COUNT \
  ((BOARD_PI_F * BOARD_WHEEL_DIAMETER_MM) / BOARD_ENC_COUNTS_PER_REV)

/* Provided by the test that needs it (test_pose.c). */
uint32_t HAL_GetTick(void);

#endif
