/* Host-side unit test for bt_parse_line / bt_error_text.
 * Compiled with the host gcc, NOT part of the firmware build. */
#include <stdio.h>
#include <string.h>

#include "bt_protocol.h"

static int failures;
static int checks;

static const char *type_name(uint8_t t)
{
  switch (t)
  {
    case CMD_TYPE_GO:    return "GO";
    case CMD_TYPE_STOP:  return "STOP";
    case CMD_TYPE_QUERY: return "QUERY";
    default:             return "?";
  }
}

static void expect_ok(const char *line, cmd_type_t type, int x, int y)
{
  cmd_msg_t m;
  int r;

  memset(&m, 0xA5, sizeof(m));   /* out must be untouched on failure */
  r = bt_parse_line(line, &m);
  checks++;
  if (r != BT_OK)
  {
    printf("FAIL  %-28s expected OK, got %d\n", line, r);
    failures++;
    return;
  }
  if ((m.type != (uint8_t)type) ||
      (type == CMD_TYPE_GO && (m.x_mm != x || m.y_mm != y)) ||
      (type != CMD_TYPE_GO && (m.x_mm != 0 || m.y_mm != 0)))
  {
    printf("FAIL  %-28s expected %s %d %d, got %s %d %d\n", line,
           type_name((uint8_t)type), x, y, type_name(m.type), m.x_mm, m.y_mm);
    failures++;
    return;
  }
  if ((m.reserved != 0U) || (m.reserved2 != 0U))
  {
    printf("FAIL  %-28s reserved fields not zeroed\n", line);
    failures++;
  }
}

static void expect_err(const char *line, int want)
{
  cmd_msg_t m;
  cmd_msg_t before;
  int r;

  memset(&m, 0xA5, sizeof(m));
  memcpy(&before, &m, sizeof(m));
  r = bt_parse_line(line, &m);
  checks++;
  if (r != want)
  {
    printf("FAIL  %-28s expected %d, got %d\n", line, want, r);
    failures++;
    return;
  }
  if (memcmp(&m, &before, sizeof(m)) != 0)
  {
    printf("FAIL  %-28s wrote to *out on failure\n", line);
    failures++;
  }
}

static void show(const char *s)
{
  /* print with escapes so trailing CR/LF cases are readable */
  for (; *s; s++)
  {
    if (*s == '\n') printf("\\n");
    else if (*s == '\r') printf("\\r");
    else if (*s == '\t') printf("\\t");
    else putchar(*s);
  }
}

int main(void)
{
  /* ---- accepted -------------------------------------------------------- */
  expect_ok("G 700 -700", CMD_TYPE_GO, 700, -700);
  expect_ok("G 700 -700\n", CMD_TYPE_GO, 700, -700);
  expect_ok("G 700 -700\r\n", CMD_TYPE_GO, 700, -700);
  expect_ok("g 700 -700\n", CMD_TYPE_GO, 700, -700);
  expect_ok("  G   700   -700   \r\n", CMD_TYPE_GO, 700, -700);
  expect_ok("G +1400 +700\n", CMD_TYPE_GO, 1400, 700);
  expect_ok("G 0 1400\n", CMD_TYPE_GO, 0, 1400);
  expect_ok("G 5000 -5000\n", CMD_TYPE_GO, 5000, -5000);   /* boundary */
  expect_ok("G1 2\n", CMD_TYPE_GO, 1, 2);                  /* lenient */
  expect_ok("S", CMD_TYPE_STOP, 0, 0);
  expect_ok("s\n", CMD_TYPE_STOP, 0, 0);
  expect_ok("S \r\n", CMD_TYPE_STOP, 0, 0);
  expect_ok("P\n", CMD_TYPE_QUERY, 0, 0);
  expect_ok("?\n", CMD_TYPE_QUERY, 0, 0);
  /* 'Z' is the untethered counterpart of the console 'c' key: clear the path
   * and zero the pose without a power cycle (which would drop the BT link). */
  expect_ok("Z", CMD_TYPE_ZERO, 0, 0);
  expect_ok("z\n", CMD_TYPE_ZERO, 0, 0);
  expect_ok("Z \r\n", CMD_TYPE_ZERO, 0, 0);

  /* ---- rejected: syntax ------------------------------------------------ */
  expect_err("", BT_ERR_SYNTAX);
  expect_err("\n", BT_ERR_SYNTAX);
  expect_err("\r\n", BT_ERR_SYNTAX);
  expect_err("    ", BT_ERR_SYNTAX);
  expect_err("   \n", BT_ERR_SYNTAX);
  expect_err("G", BT_ERR_SYNTAX);
  expect_err("G ", BT_ERR_SYNTAX);
  expect_err("G 700", BT_ERR_SYNTAX);              /* missing y */
  expect_err("G 700 ", BT_ERR_SYNTAX);
  expect_err("G 1 2 3", BT_ERR_SYNTAX);            /* trailing junk */
  expect_err("G 1x 2", BT_ERR_SYNTAX);             /* glued token */
  expect_err("G - 0", BT_ERR_SYNTAX);              /* bare sign */
  expect_err("X 1 2", BT_ERR_SYNTAX);              /* unknown command */
  expect_err("S x", BT_ERR_SYNTAX);
  expect_err("Z x", BT_ERR_SYNTAX);                /* 'Z' takes no argument */
  expect_err("?x", BT_ERR_SYNTAX);
  expect_err("P 1", BT_ERR_SYNTAX);
  expect_err("G 0x10 2", BT_ERR_SYNTAX);           /* hex not accepted */
  expect_err("G 1.5 2", BT_ERR_SYNTAX);            /* mm are integers */
  expect_err("G 1 2\nXYZ", BT_ERR_SYNTAX);         /* junk after terminator */

  /* ---- rejected: range ------------------------------------------------- */
  expect_err("G 5001 0", BT_ERR_RANGE);
  expect_err("G 0 -5001", BT_ERR_RANGE);
  expect_err("G 99999 0", BT_ERR_RANGE);
  expect_err("G 700000 0", BT_ERR_RANGE);          /* the typo the bound is for */
  expect_err("G 999999999999999999999 0", BT_ERR_RANGE); /* no int32 overflow */
  expect_err("G 1 999999999999999999999", BT_ERR_RANGE);

  /* ---- NULL arguments -------------------------------------------------- */
  {
    cmd_msg_t m;
    checks++;
    if (bt_parse_line(NULL, &m) != BT_ERR_SYNTAX) { printf("FAIL  NULL line\n"); failures++; }
    checks++;
    if (bt_parse_line("S", NULL) != BT_ERR_SYNTAX) { printf("FAIL  NULL out\n"); failures++; }
  }

  /* ---- error text ------------------------------------------------------ */
  {
    const char *codes[] = { "ok", "syntax", "range", "busy", "unknown" };
    int vals[] = { BT_OK, BT_ERR_SYNTAX, BT_ERR_RANGE, BT_ERR_BUSY, -999 };
    int i;
    for (i = 0; i < 5; i++)
    {
      const char *t = bt_error_text(vals[i]);
      checks++;
      if (t == NULL || strcmp(t, codes[i]) != 0)
      {
        printf("FAIL  bt_error_text(%d) = %s\n", vals[i], t ? t : "(null)");
        failures++;
      }
    }
  }

  (void)show;
  printf("\n%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
