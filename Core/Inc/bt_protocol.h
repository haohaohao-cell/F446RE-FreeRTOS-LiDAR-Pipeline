/* -----------------------------------------------------------------------------
 * bt_protocol.h - Bluetooth command protocol (RX path) for Block 4.5 Part 2.
 *
 * TWO DIRECTIONS, TWO FORMATS - DELIBERATELY ASYMMETRIC
 * -----------------------------------------------------
 *   RX  (phone/PC -> car) : ASCII line text, '\n' terminated ('\r\n' tolerated),
 *                           integer millimetres.  Chosen so the car can be
 *                           driven from any serial terminal app during the demo.
 *   TX  (car -> PC)       : unchanged binary telemetry frames (0xAA 0x55 ...,
 *                           see telemetry.c).  The PC viewer depends on it.
 *
 * Do not "unify" them.  The asymmetry is a feature: the command channel stays
 * human-readable and debuggable, the telemetry channel stays compact and is
 * already validated on hardware.
 *
 * COORDINATE FRAME
 * ----------------
 *   Origin  = the pose at power-up (place the car squarely before resetting).
 *   +x      = right,  +y = forward   (matches pc_app's "X right / Y forward")
 *   Units   = millimetres, integers.  Angles are never sent by hand.
 *
 * GRAMMAR (case-insensitive command letter, spaces may repeat)
 * -----------------------------------------------------------
 *   G <x_mm> <y_mm>\n   absolute target;  e.g. "G 700 -700\n"
 *   S\n                 stop: clear Q_Path, PWM -> 0, keep the current pose
 *   P\n                 query pose (debug)
 *   ?\n                 help
 *
 * DOWNLINK REPLIES (built by BT TX, parsed by the PC app)
 * ------------------------------------------------------
 *   A G <x> <y>\n       accepted
 *   E <code> <text>\n   rejected; code: 1 = syntax, 2 = out of range, 3 = busy
 *
 * STATUS: interface only.  The implementation file (bt_protocol.c) lands in
 * the BT phase; nothing calls these yet, and no caller may assume a reply.
 * -------------------------------------------------------------------------- */

#ifndef BT_PROTOCOL_H
#define BT_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>   /* offsetof, used by the layout asserts below */

/* ------------------------------------------------------------------------- */
/* Q_Cmd message                                                             */
/* ------------------------------------------------------------------------- */
/* Exactly 8 bytes so one message is a single 64-bit copy; POD, no padding
 * surprises.  Field order fixed: type/reserved/x_mm/y_mm/reserved2. */
typedef enum
{
  CMD_TYPE_GO    = 0,   /* move to (x_mm, y_mm)                       */
  CMD_TYPE_STOP  = 1,   /* stop and clear the remaining path          */
  CMD_TYPE_QUERY = 2,   /* report pose; carries no target             */
  CMD_TYPE_ZERO  = 3    /* clear path AND zero the pose ("Z")         */
} cmd_type_t;

/* Why CMD_TYPE_ZERO exists: the pose is relative to boot and the console key
 * that zeroes it ('c') is only on USART2, i.e. only reachable over the ST-Link
 * USB cable.  An untethered run - which is what the marking asks for - then has
 * no way to re-zero between runs except a power cycle, and that drops the
 * Bluetooth link and costs a re-pair in front of the examiner.  'Z' over the
 * Bluetooth link derives the same effect: clear Q_Path, raise the STOP event,
 * and request the pose reset that the Pose task already owns. */

typedef struct
{
  uint8_t  type;       /* cmd_type_t                                  */
  uint8_t  reserved;   /* 0; keeps x_mm 2-byte aligned                */
  int16_t  x_mm;       /* + right, valid for CMD_TYPE_GO only         */
  int16_t  y_mm;       /* + forward, valid for CMD_TYPE_GO only       */
  uint16_t reserved2;  /* 0; pads the message to 8 bytes              */
} cmd_msg_t;

/* The 8-byte size is an interface contract, not an accident: one Q_Cmd message
 * is a pair of 32-bit words, cheap to copy in and out of the queue, and the
 * reserved fields exist only to keep it there.  Adding a field without
 * thinking will silently change what a queue copy costs and which bytes the
 * producer and consumer agree on, so make it a build error instead. */
_Static_assert(sizeof(cmd_msg_t) == 8, "Q_Cmd message must stay 8 bytes");
_Static_assert(offsetof(cmd_msg_t, x_mm) == 2, "x_mm must be 2-byte aligned");

/* Queue depth for Q_Cmd (see CODING_STANDARDS 2.2). */
#define Q_CMD_CAPACITY        4U

/* Parser return codes.  0 = accepted, negative = rejected. */
#define BT_OK                 0
#define BT_ERR_SYNTAX        (-1)   /* not one of G/S/P/?, or bad tokens  */
#define BT_ERR_RANGE         (-2)   /* coordinate outside +/-NAV_MAX_MM    */
#define BT_ERR_BUSY          (-3)   /* accepted but could not be queued   */

/* Sanity bound on |coordinate|, NOT the real field size: it exists so that a
 * typo such as "G 700000 0" is rejected instead of wrapping a int16.  The
 * 4 x 4 m field discussed for the final product fits comfortably. */
#define NAV_MAX_MM            5000

/* Longest line the parser will look at, including the terminator.  Longer
 * lines are rejected with BT_ERR_SYNTAX rather than truncated. */
#define BT_LINE_MAX           32U

/* ------------------------------------------------------------------------- */
/* Parser                                                                    */
/* ------------------------------------------------------------------------- */
/* Parse one received line into *out.
 *
 *   line : NUL-terminated text.  A trailing '\n' and/or '\r' is accepted and
 *          ignored; leading/trailing spaces are ignored; empty lines are
 *          BT_ERR_SYNTAX.
 *   out  : written only when 0 is returned.  Untouched otherwise, so the
 *          caller may reuse a buffer across calls.
 *
 * Returns BT_OK, BT_ERR_SYNTAX or BT_ERR_RANGE.  BT_ERR_BUSY is produced by
 * the queueing layer (BT RX task), not by this function.
 *
 * Thread context: pure function, no state, no HAL, safe from any task.  It is
 * deliberately not ISR-callable by contract - the BT RX task owns parsing. */
int bt_parse_line(const char *line, cmd_msg_t *out);

/* Human-readable name of a parser/Q_Cmd error code, for the "E <code> <text>"
 * reply.  Never returns NULL. */
const char *bt_error_text(int code);

#endif /* BT_PROTOCOL_H */
