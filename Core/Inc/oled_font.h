/* -----------------------------------------------------------------------------
 * oled_font.h - 6x8 ASCII font table used by oled.c.
 *
 * Kept in its own translation unit only so that the ~600 bytes of glyph data
 * do not bury the display logic.  See oled_font.c for the layout contract:
 *   OLED_FONT_COLS bytes per glyph, one byte per column, LSB = top pixel row,
 *   last column always 0 (inter-character gap).
 *
 * Covered range: 0x20 (space) .. 0x7E ('~'), dense.  oled.c maps every other
 * byte to '?' before indexing, so an index out of range is impossible.
 * -------------------------------------------------------------------------- */

#ifndef OLED_FONT_H
#define OLED_FONT_H

#include <stdint.h>

#define OLED_FONT_FIRST  0x20U   /* ' '  */
#define OLED_FONT_LAST   0x7EU   /* '~'  */
#define OLED_FONT_ROWS   95U     /* LAST - FIRST + 1 */
#define OLED_FONT_COLS    6U     /* 5 glyph columns + 1 gap column  */

extern const uint8_t oled_font_6x8[OLED_FONT_ROWS][OLED_FONT_COLS];

#endif /* OLED_FONT_H */
