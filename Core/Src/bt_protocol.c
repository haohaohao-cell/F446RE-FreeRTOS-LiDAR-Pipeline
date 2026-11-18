/* -----------------------------------------------------------------------------
 * bt_protocol.c - command parser for the text downlink described in bt_protocol.h.
 *
 * Deliberately a pure function: no HAL, no queue, no state.  The BT RX task
 * feeds it one assembled line at a time, which is what makes every rejection
 * path (syntax, range, missing parameter, junk characters) testable without
 * a radio and without a scheduler.
 *
 * ACCEPTED GRAMMAR (case-insensitive letter, runs of spaces allowed)
 *     G <x_mm> <y_mm>     absolute target
 *     S                   stop
 *     P                   report pose
 *     ?                   help
 * A trailing '\n' and/or '\r' is ignored, leading/trailing spaces are ignored,
 * and NOTHING else is allowed after the last token - trailing junk is a syntax
 * error rather than being silently ignored, so a typo is reported instead of
 * driving the car somewhere unexpected.
 *
 * "?" IS PARSED AS A QUERY, not as a fourth message type.  Q_Cmd has exactly
 * three types and the layout is a locked 8-byte contract; inventing a HELP type
 * would change that contract to carry an enum nobody consumes.  The reply layer
 * (BT TX) is what turns a query into "here is the pose" plus "here is the help
 * text", so the distinction lives where it is actually used.
 * -------------------------------------------------------------------------- */

#include "bt_protocol.h"

static int bt_is_space(char c)
{
  return ((c == ' ') || (c == '\t')) ? 1 : 0;
}

static int bt_is_digit(char c)
{
  return ((c >= '0') && (c <= '9')) ? 1 : 0;
}

/* Skip any run of spaces. */
static void bt_skip_spaces(const char **pp)
{
  const char *p = *pp;
  while (bt_is_space(*p) != 0)
  {
    p++;
  }
  *pp = p;
}

/* Is the rest of the line empty (only terminator / spaces left)?
 *
 * Everything AFTER the terminator must be empty too: the line contract is "one
 * command per NUL-terminated string", and accepting "G 1 2\nXYZ" would mean the
 * parser silently ignores a second command instead of reporting it.  The BT RX
 * layer splits on '\n' anyway, so this only ever rejects genuinely malformed
 * input - which is the point. */
static int bt_at_end(const char *p)
{
  bt_skip_spaces(&p);

  if (*p == '\0')
  {
    return 1;
  }

  if ((*p == '\n') || (*p == '\r'))
  {
    p++;
    while ((*p == '\n') || (*p == '\r') || (bt_is_space(*p) != 0))
    {
      p++;
    }
    return (*p == '\0') ? 1 : 0;
  }

  return 0;
}

/* Parse one signed decimal integer.  Rejects an empty or non-numeric token, and
 * stops at the first non-digit so the caller can check the separator.
 *
 * The accumulator is wide on purpose: "G 99999999 0" must be caught by the range
 * check and reported as BT_ERR_RANGE, never wrap into a valid-looking int16. */
static int bt_parse_int(const char **pp, int32_t *out)
{
  const char *p = *pp;
  int32_t value = 0;
  int32_t sign = 1;
  uint8_t digits = 0U;

  if (*p == '-')
  {
    sign = -1;
    p++;
  }
  else if (*p == '+')
  {
    p++;
  }

  while (bt_is_digit(*p) != 0)
  {
    /* Saturate instead of overflowing: sign and magnitude are preserved well
     * past NAV_MAX_MM, which is all the range check needs. */
    if (value < 100000000)
    {
      value = (value * 10) + (int32_t)(*p - '0');
    }
    digits++;
    p++;
  }

  if (digits == 0U)
  {
    return BT_ERR_SYNTAX;
  }

  /* A token glued to the next one ("G 1x 2") is not a number. */
  if ((*p != '\0') && (*p != '\n') && (*p != '\r') && (bt_is_space(*p) == 0))
  {
    return BT_ERR_SYNTAX;
  }

  *out = value * sign;
  *pp = p;
  return BT_OK;
}

int bt_parse_line(const char *line, cmd_msg_t *out)
{
  const char *p = line;
  char cmd;

  if ((line == NULL) || (out == NULL))
  {
    return BT_ERR_SYNTAX;
  }

  bt_skip_spaces(&p);

  if ((*p == '\0') || (*p == '\n') || (*p == '\r'))
  {
    /* Empty line: a terminal sends one for a bare Enter.  Reject it rather than
     * treating it as a command. */
    return BT_ERR_SYNTAX;
  }

  cmd = *p;
  p++;

  /* Case-insensitive command letter. */
  if ((cmd >= 'a') && (cmd <= 'z'))
  {
    cmd = (char)(cmd - 'a' + 'A');
  }

  switch (cmd)
  {
    case 'G':
    {
      int32_t x = 0;
      int32_t y = 0;
      int st;

      bt_skip_spaces(&p);
      st = bt_parse_int(&p, &x);
      if (st != BT_OK)
      {
        return st;              /* missing or malformed x */
      }

      if (bt_is_space(*p) == 0)
      {
        return BT_ERR_SYNTAX;   /* missing second parameter */
      }
      bt_skip_spaces(&p);

      st = bt_parse_int(&p, &y);
      if (st != BT_OK)
      {
        return st;              /* missing or malformed y */
      }

      if (bt_at_end(p) == 0)
      {
        return BT_ERR_SYNTAX;   /* trailing junk */
      }

      /* Range is checked after the whole line parses, so "G 99999 abc" is a
       * syntax error while "G 99999 0" is a range error - the two are reported
       * as what they are instead of depending on token order. */
      if ((x > NAV_MAX_MM) || (x < -NAV_MAX_MM) ||
          (y > NAV_MAX_MM) || (y < -NAV_MAX_MM))
      {
        return BT_ERR_RANGE;
      }

      out->type = (uint8_t)CMD_TYPE_GO;
      out->reserved = 0U;
      out->x_mm = (int16_t)x;
      out->y_mm = (int16_t)y;
      out->reserved2 = 0U;
      return BT_OK;
    }

    case 'S':
      if (bt_at_end(p) == 0)
      {
        return BT_ERR_SYNTAX;
      }
      out->type = (uint8_t)CMD_TYPE_STOP;
      out->reserved = 0U;
      out->x_mm = 0;
      out->y_mm = 0;
      out->reserved2 = 0U;
      return BT_OK;

    case 'Z':
      if (bt_at_end(p) == 0)
      {
        return BT_ERR_SYNTAX;
      }
      out->type = (uint8_t)CMD_TYPE_ZERO;
      out->reserved = 0U;
      out->x_mm = 0;
      out->y_mm = 0;
      out->reserved2 = 0U;
      return BT_OK;

    case 'P':
    case '?':
      if (bt_at_end(p) == 0)
      {
        return BT_ERR_SYNTAX;
      }
      out->type = (uint8_t)CMD_TYPE_QUERY;
      out->reserved = 0U;
      out->x_mm = 0;
      out->y_mm = 0;
      out->reserved2 = 0U;
      return BT_OK;

    default:
      return BT_ERR_SYNTAX;
  }
}

const char *bt_error_text(int code)
{
  switch (code)
  {
    case BT_OK:          return "ok";
    case BT_ERR_SYNTAX:  return "syntax";
    case BT_ERR_RANGE:   return "range";
    case BT_ERR_BUSY:    return "busy";
    default:             return "unknown";
  }
}
