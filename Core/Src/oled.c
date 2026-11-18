/* -----------------------------------------------------------------------------
 * oled.c - SSD1306 128x64 panel on the shared I2C1 bus.
 *
 * Two layers, as oled.h requires: a 1 KiB framebuffer that never touches
 * hardware, and oled_bus_write() as the single transport seam.  Everything
 * except oled_bus_write()/oled_init() is pure memory work, which is why the
 * panel can be missing without breaking anything except the transfer itself.
 *
 * FRAMEBUFFER LAYOUT = the panel's own layout
 * -------------------------------------------
 *   fb[page][column], page 0..7 = 8 pixel rows, bit 0 = topmost row of the page.
 * That is exactly what SSD1306 expects in page addressing mode, so a page
 * refresh is a straight 128-byte copy and a glyph is a straight 6-byte copy.
 * No bit rotation anywhere: the alternative (a linear pixel buffer) would burn
 * CPU in the 50 ms slot for no benefit on a fixed 6x8 font.
 *
 * TIMING AND WHY A PAGE IS SENT IN CHUNKS
 * ---------------------------------------
 * CODING_STANDARDS 2.4 sets a < 3 ms mutex hold target and 5.1 caps a single
 * transaction at 10 ms.  Do the arithmetic for this bus before believing any
 * estimate:
 *
 *     100 kHz = 100 000 bit/s = 11 111 byte/s (9 bit times per byte, ACK incl.)
 *     128 data bytes + 1 control byte = 129 bytes  ->  ~11.6 ms   TOO LONG
 *      32 data bytes + 1 control byte =  33 bytes  ->  ~2.97 ms   fits
 *
 * So oled_refresh_page() sends the page as OLED_CHUNK_BYTES-byte transactions:
 * still one page per call (the oled.h contract), but no single mutex hold can
 * exceed the 3 ms target, and an IMU acquisition waiting on the bus waits at
 * most one chunk, not a whole page.
 *
 *   This also means the "128 B ~= 1.5 ms" figure in oled.h/CODING_STANDARDS 3.4
 *   does not hold for a 100 kHz bus (1.5 ms would be ~16 data bytes; at 400 kHz
 *   a page is still ~2.9 ms).  The real hold time is measured in software
 *   (i2c_bus_stats_t.hold_us_max) and printed in the 1 Hz line; the estimate
 *   above is labelled an estimate.  Both docs need correcting on hardware day.
 *
 *   Chunking is safe because the SSD1306 keeps its own column pointer: a second
 *   device (the MPU-6500) using the bus in between does not disturb it, and the
 *   next refresh rewrites the whole page anyway.
 * -------------------------------------------------------------------------- */

#include "oled.h"
#include "oled_font.h"
#include "board_pins.h"
#include "i2c_bus.h"

#include <string.h>

/* SSD1306 control byte: Co=0, D/C#=0 -> a command stream; D/C#=1 -> pixel data. */
#define OLED_CTRL_CMD     0x00U
#define OLED_CTRL_DATA    0x40U

/* One page = 128 data bytes + 1 control byte. */
#define OLED_TX_MAX       (OLED_W + 1U)

/* Largest number of pixel bytes per I2C transaction.  32 data + 1 control byte
 * is ~2.97 ms at 100 kHz, just under the 3 ms mutex hold target.  See the
 * timing note at the top of the file before changing this. */
#define OLED_CHUNK_BYTES  16U

/* Bounded: an absent panel must cost milliseconds at init, never a hang. */
#define OLED_TMO_MS       10U
#define OLED_PROBE_TMO_MS  5U

static uint8_t fb[OLED_PAGES][OLED_W];
static uint8_t invert_mask;      /* bit p set -> page p is displayed inverted */
static uint8_t panel_present;
static uint8_t next_page;
static oled_stats_t stats;

/* Single static transmit buffer: the OLED task is the only caller, so no lock
 * is needed for it (the bus itself is locked inside i2c_bus_*). */
static uint8_t tx_buf[OLED_TX_MAX];

static uint8_t to_ascii(char c)
{
  uint8_t u = (uint8_t)c;
  if ((u < OLED_FONT_FIRST) || (u > OLED_FONT_LAST))
  {
    return (uint8_t)'?';
  }
  return u;
}

static void status_note(int st)
{
  stats.last_status = (uint32_t)(int32_t)st;
  if (st != OLED_OK)
  {
    stats.errors++;
  }
}

/* ========================================================================== */
/* transport seam: the ONLY function that touches the panel                   */
/* ========================================================================== */
int oled_bus_write(uint8_t ctrl, const uint8_t *buf, uint16_t len)
{
  int st;

  if ((buf == NULL) || (len == 0U) || (len >= OLED_TX_MAX))
  {
    return OLED_ERR_PARAM;
  }

  /* SSD1306 wants the control byte in the same transaction as its payload. */
  tx_buf[0] = ctrl;
  memcpy(&tx_buf[1], buf, len);

  st = i2c_bus_write(BOARD_OLED_I2C_ADDR_7BIT, tx_buf, (uint16_t)(len + 1U),
                     OLED_TMO_MS);

  if (st == I2C_BUS_OK)
  {
    return OLED_OK;
  }
  return (st == I2C_BUS_NACK) ? OLED_ERR_ABSENT : OLED_ERR_BUS;
}

/* ========================================================================== */
/* life cycle                                                                 */
/* ========================================================================== */
int oled_init(void)
{
  /* Standard SSD1306 power-up sequence.  Only Page addressing mode (0x20 0x00)
   * is usable here: it is what makes "one page = one 128-byte write" true. */
  static const uint8_t init_seq[] =
  {
    0xAEU,              /* display off while configuring                     */
    0xD5U, 0x80U,       /* clock divide / oscillator frequency               */
    0xA8U, 0x3FU,       /* multiplex ratio = 64 rows                         */
    0xD3U, 0x00U,       /* display offset 0                                  */
    0x40U,              /* start line 0                                      */
    0x8DU, 0x14U,       /* charge pump on (module has no external VCC)        */
    0x20U, 0x00U,       /* memory addressing mode = PAGE                     */
    0xA1U,              /* segment remap: column 127 -> SEG0 (module wiring)   */
    0xC8U,              /* COM scan direction remapped                       */
    0xDAU, 0x12U,       /* COM pins: alternative, no left/right remap        */
    0x81U, 0xCFU,       /* contrast                                          */
    0xD9U, 0xF1U,       /* pre-charge period                                 */
    0xDBU, 0x40U,       /* VCOMH deselect level                              */
    0xA4U,              /* display follows RAM (not all-on)                  */
    0xA6U,              /* normal (non-inverted)                             */
    0x2EU,              /* deactivate scrolling                              */
    0xAFU               /* display on                                        */
  };

  panel_present = 0U;
  next_page = 0U;
  invert_mask = 0U;
  memset(&stats, 0, sizeof(stats));
  oled_clear();

  /* Probe by sending the first command (display off).  If nothing acknowledges
   * at 0x3C the whole sequence is pointless, so bail out instead of pushing 25
   * more bytes at an empty address.  The bus timeout bounds this to a few ms. */
  {
    int st = oled_bus_write(OLED_CTRL_CMD, &init_seq[0], 1U);

    if (st != OLED_OK)
    {
      /* OLED_ERR_ABSENT and OLED_ERR_BUS are both reported as "absent" only
       * when the panel never answered at all; a broken bus is still a real
       * error and is returned as such. */
      status_note(st);
      return st;
    }

    /* Panel answered: push the rest.  One transaction, one control byte. */
    st = oled_bus_write(OLED_CTRL_CMD, &init_seq[1],
                        (uint16_t)(sizeof(init_seq) - 1U));
    if (st != OLED_OK)
    {
      status_note(st);
      return st;
    }
  }

  panel_present = 1U;

  /* A blank screen is the defined power-up state; this also proves the panel
   * accepts page writes before anything relies on it. */
  (void)oled_refresh_all();
  return OLED_OK;
}

uint8_t oled_is_present(void)
{
  return panel_present;
}

/* ========================================================================== */
/* drawing - framebuffer only, no bus access                                  */
/* ========================================================================== */
void oled_clear(void)
{
  memset(fb, 0, sizeof(fb));
}

int oled_show_text(uint8_t row, uint8_t col, const char *text)
{
  if ((row >= OLED_ROWS) || (col >= OLED_COLS) || (text == NULL))
  {
    return OLED_ERR_PARAM;
  }

  /* Clip at the right edge rather than wrap: a status line that overflows must
   * not scribble over the next row. */
  while ((*text != '\0') && (col < OLED_COLS))
  {
    uint8_t idx = (uint8_t)(to_ascii(*text) - OLED_FONT_FIRST);
    uint8_t x = (uint8_t)(col * OLED_CELL_W);
    uint8_t i;

    for (i = 0U; i < OLED_FONT_COLS; i++)
    {
      fb[row][x + i] = oled_font_6x8[idx][i];
    }

    col++;
    text++;
  }

  return OLED_OK;
}

int oled_show_text_xy(uint8_t x_px, uint8_t y_px, const char *text)
{
  /* Round down to the enclosing cell: the panel has no sub-cell positioning. */
  if ((x_px >= OLED_W) || (y_px >= OLED_H))
  {
    return OLED_ERR_PARAM;
  }
  return oled_show_text((uint8_t)(y_px / OLED_CELL_H), (uint8_t)(x_px / OLED_CELL_W), text);
}

int oled_invert_row(uint8_t row, uint8_t inverted)
{
  if (row >= OLED_ROWS)
  {
    return OLED_ERR_PARAM;
  }

  /* Kept as a display attribute, not by XOR-ing the framebuffer: drawing after
   * the inversion must not come out inverted, and un-inverting must restore the
   * original pixels exactly. */
  if (inverted != 0U)
  {
    invert_mask = (uint8_t)(invert_mask | (uint8_t)(1U << row));
  }
  else
  {
    invert_mask = (uint8_t)(invert_mask & (uint8_t)~(1U << row));
  }
  return OLED_OK;
}

/* ========================================================================== */
/* transferring the framebuffer                                               */
/* ========================================================================== */
int oled_refresh_page(uint8_t page)
{
  uint8_t chunk[OLED_CHUNK_BYTES];
  uint8_t cmd[3];
  uint8_t offset;
  uint8_t i;
  int st;

  if (page >= OLED_PAGES)
  {
    return OLED_ERR_PARAM;
  }

  /* No panel: the framebuffer is still valid, so this is not a drawing error.
   * Report it honestly instead of pretending the page went out. */
  if (panel_present == 0U)
  {
    status_note(OLED_ERR_ABSENT);
    return OLED_ERR_ABSENT;
  }

  /* Point the panel at (page, column 0).  Page addressing mode keeps the column
   * pointer moving forward across transactions, which is what makes the chunked
   * data phase below contiguous. */
  cmd[0] = (uint8_t)(0xB0U | page);   /* page start address      */
  cmd[1] = 0x00U;                     /* lower column nibble 0   */
  cmd[2] = 0x10U;                     /* higher column nibble 0  */
  st = oled_bus_write(OLED_CTRL_CMD, cmd, (uint16_t)sizeof(cmd));
  if (st != OLED_OK)
  {
    status_note(st);
    return st;
  }

  /* Data phase, one bounded transaction per chunk. */
  for (offset = 0U; offset < OLED_W; offset = (uint8_t)(offset + OLED_CHUNK_BYTES))
  {
    for (i = 0U; i < OLED_CHUNK_BYTES; i++)
    {
      chunk[i] = ((invert_mask & (uint8_t)(1U << page)) != 0U)
                   ? (uint8_t)(fb[page][offset + i] ^ 0xFFU)
                   : fb[page][offset + i];
    }

    st = oled_bus_write(OLED_CTRL_DATA, chunk, OLED_CHUNK_BYTES);
    if (st != OLED_OK)
    {
      /* Partial page: reported, counted, and rewritten on the next refresh of
       * this page.  Not silently treated as success. */
      status_note(st);
      return st;
    }
  }

  stats.pages++;
  status_note(OLED_OK);
  return OLED_OK;
}

int oled_refresh_next(uint8_t *page_out)
{
  uint8_t page = next_page;
  int st;

  next_page = (uint8_t)((next_page + 1U) % OLED_PAGES);
  if (page_out != NULL)
  {
    *page_out = page;
  }

  st = oled_refresh_page(page);
  return st;
}

int oled_refresh_all(void)
{
  int st = OLED_OK;
  uint8_t page;

  for (page = 0U; page < OLED_PAGES; page++)
  {
    int one = oled_refresh_page(page);
    if ((one != OLED_OK) && (st == OLED_OK))
    {
      st = one;
    }
  }
  return st;
}

void oled_get_stats(oled_stats_t *out)
{
  if (out == NULL)
  {
    return;
  }
  memcpy(out, &stats, sizeof(*out));
}
