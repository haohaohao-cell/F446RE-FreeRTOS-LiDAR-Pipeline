/* -----------------------------------------------------------------------------
 * oled.h - SSD1306 128x64 status display on the shared I2C1 bus.
 *
 * TWO LAYERS, ON PURPOSE
 * ---------------------
 *   upper  : a plain 1024-byte framebuffer + text drawing.  No bus access at
 *            all, so it is free and testable everywhere.
 *   lower  : oled_bus_write(), the ONE function that touches hardware.
 *
 * The reason is hardware uncertainty, not elegance: the panel may turn out to be
 * a 7-pin SPI version instead of the expected 4-pin I2C one (open item).  The
 * SSD1306 initialisation sequence is identical for both transports, so if the
 * module is SPI only oled_bus_write() has to be rewritten - the drawing code,
 * the page scheduler and every caller stay untouched.  Do not inline HAL calls
 * into the drawing functions.
 *
 * REFRESH MODEL: ONE PAGE PER CALL
 * --------------------------------
 * At 100 kHz a 128-byte page costs about 1.5 ms while a full frame costs about
 * 12 ms.  Refreshing a whole frame in one 50 ms slot would hold the I2C mutex
 * for 12 ms, long enough to push an IMU sample (100 Hz) past its deadline.  So
 * the 50 ms display task calls oled_refresh_next() once per slot and the frame
 * walks through its 8 pages.
 *
 *   consequence: a full frame takes 8 x 50 ms = 400 ms.  That is fine for a
 *   slowly changing status screen and is stated here so nobody is surprised.
 *   Raising I2C1 to 400 kHz (both devices support it) would cut it to ~100 ms,
 *   but it invalidates the IMU timing that was measured on hardware.
 *
 * WHAT TO DRAW
 * ------------
 * Fixed 6x8 character cells: 21 columns x 8 rows.  ASCII 0x20..0x7E only;
 * anything else is rendered as '?'  (the font table is not a Unicode renderer).
 *
 * THREADING: the OLED task owns the framebuffer.  Nothing else may call into
 * this module.  If a future task needs to display something, it posts to the
 * OLED task rather than drawing directly.
 * -------------------------------------------------------------------------- */

#ifndef OLED_H_INCLUDED
#define OLED_H_INCLUDED

#include <stdint.h>

/* NOTE: the guard is deliberately NOT "OLED_H" - that name is the panel height
 * macro below, and using it for both would make the guard and the height clash. */
#define OLED_W        128U
#define OLED_H         64U
#define OLED_PAGES      8U    /* 64 / 8 rows per page                     */
#define OLED_COLS      21U    /* 128 / 6 px per character cell            */
#define OLED_ROWS       8U    /* one text row per page                    */
#define OLED_CELL_W     6U
#define OLED_CELL_H     8U

/* Return codes.  Everything is negative on failure so `if (st < 0)` is enough. */
typedef enum
{
  OLED_OK         =  0,
  OLED_ERR_BUS    = -1,   /* I2C transfer failed (mutex or HAL)            */
  OLED_ERR_PARAM  = -2,   /* page/row/column out of range                  */
  OLED_ERR_ABSENT = -3    /* panel did not acknowledge - not fitted        */
} oled_status_t;

/* ---- life cycle ---------------------------------------------------------- */
/* Bring the panel up.  Returns OLED_OK, OLED_ERR_ABSENT (nothing at
 * BOARD_OLED_I2C_ADDR_7BIT) or OLED_ERR_BUS.  Never blocks indefinitely: the
 * bus timeout is bounded, so a missing panel costs a few milliseconds, not a
 * hang.  Call once, before the scheduler starts.
 *
 * A missing panel is NOT fatal: the framebuffer still works and refresh calls
 * return OLED_ERR_ABSENT, so the caller needs no special case. */
int oled_init(void);

/* 1 if oled_init() found the panel, 0 otherwise. */
uint8_t oled_is_present(void);

/* ---- drawing (framebuffer only, no bus access) --------------------------- */
void oled_clear(void);

/* Write text at a text cell.  row 0..OLED_ROWS-1, col 0..OLED_COLS-1.  Text that
 * would run past the right edge is clipped, not wrapped.  Returns OLED_OK or
 * OLED_ERR_PARAM; never fails because the panel is absent. */
int oled_show_text(uint8_t row, uint8_t col, const char *text);

/* Same, positioned in pixels (rounded down to a cell). */
int oled_show_text_xy(uint8_t x_px, uint8_t y_px, const char *text);

/* Invert a whole text row, for a selection cursor in the setup UI. */
int oled_invert_row(uint8_t row, uint8_t inverted);

/* ---- transferring the framebuffer --------------------------------------- */
/* Push one page (8 pixel rows) to the panel.  About 1.5 ms. */
int oled_refresh_page(uint8_t page);

/* Push the next page in round-robin order and report which one went out.
 * This is what the 50 ms OLED task calls; *page_out may be NULL.
 * Returns OLED_OK, OLED_ERR_ABSENT or OLED_ERR_BUS. */
int oled_refresh_next(uint8_t *page_out);

/* Push all 8 pages back to back (~12 ms, holds the bus throughout).  Only for
 * one-off setup screens - never from the periodic display task. */
int oled_refresh_all(void);

/* ---- transport seam ------------------------------------------------------ */
/* The only function that touches the panel.  `ctrl` is the SSD1306 control
 * byte: 0x00 for a command stream, 0x40 for pixel data.  Implemented in oled.c
 * on top of i2c_bus_write(); a SPI panel would replace just this.
 * Declared here so the seam is documented, not because callers use it. */
int oled_bus_write(uint8_t ctrl, const uint8_t *buf, uint16_t len);

/* ---- observability ------------------------------------------------------- */
typedef struct
{
  uint32_t pages;        /* pages successfully transferred                */
  uint32_t errors;       /* failed transfers                              */
  uint32_t last_status;  /* last oled_status_t, for the 1 Hz line         */
} oled_stats_t;

void oled_get_stats(oled_stats_t *out);

#endif /* OLED_H_INCLUDED */
