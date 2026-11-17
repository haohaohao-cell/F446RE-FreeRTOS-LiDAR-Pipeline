/* -----------------------------------------------------------------------------
 * bt_rx.c - UART5 receive path: DMA idle-line bursts -> ring -> lines -> sink.
 *
 * Split of work (CODING_STANDARDS 0.4):
 *   ISR  : copy the N bytes that the DMA window already holds into the ring,
 *          drop what does not fit, re-arm, return.  A handful of microseconds.
 *   task : assemble lines, parse with bt_parse_line(), hand to the sink.
 * Nothing in this file calls a CMSIS-RTOS2 API: the task wake-up is issued by
 * app.c, which owns the semaphore, keeping this module free of kernel headers
 * (see bt_rx.h).
 *
 * WHY THE DMA IS NORMAL-MODE AND 64 BYTES WIDE
 * --------------------------------------------
 * The window has to be bounded, otherwise "how many bytes arrived" never has an
 * answer.  Idle-line detection closes the burst; a 64-byte window is twice the
 * longest legal command, so a complete command always fits, and the transfer
 * complete event (64 bytes with no gap at all) is handled by the same callback.
 *
 * HALF-TRANSFER IS DELIBERATELY DISABLED.  On a half-transfer the HAL calls the
 * same callback but leaves reception running, so the callback cannot re-arm (the
 * UART is still busy) and the following IDLE would report byte 0..N again -
 * every command in a >32-byte burst would be delivered twice.  Re-arming from
 * the HT event is therefore impossible, and the only safe options are "disable
 * HT" (chosen) or "buffer the second half separately" (more state, no benefit at
 * 64 bytes).  __HAL_DMA_DISABLE_IT() is re-applied after every re-arm because
 * HAL_UARTEx_ReceiveToIdle_DMA() re-enables it.
 *
 * SELF-HEALING (CODING_STANDARDS 5.2): a UART error does not try to re-arm from
 * the ISR.  bt_rx_on_error() only raises a flag; bt_rx_process() performs the
 * abort + re-arm in task context, exactly like lidar.c does.
 * -------------------------------------------------------------------------- */

#include "bt_rx.h"
#include "board_pins.h"

#include <string.h>

extern UART_HandleTypeDef huart5;

#define BT_RING_MASK  (BT_RX_RING_SIZE - 1U)

/* Compile-time proof of the power-of-two assumption the mask relies on. */
_Static_assert((BT_RX_RING_SIZE & BT_RING_MASK) == 0U,
               "BT_RX_RING_SIZE must be a power of two");

static DMA_HandleTypeDef hdma_bt_rx;

static uint8_t rx_dma[BOARD_BT_RX_DMA_SIZE];

/* volatile: written by the ISR, read by the task.  The indices alone being
 * volatile is not enough - without it the compiler is free to move the byte copy
 * across the index update, and the task could read a stale slot. */
static volatile uint8_t ring[BT_RX_RING_SIZE];
static volatile uint16_t ring_head;   /* advanced by the ISR only   */
static volatile uint16_t ring_tail;   /* advanced by the task only  */

static char line_buf[BT_LINE_MAX];
static uint8_t line_len;
static uint8_t line_overlong;         /* discarding until the next '\n' */

static volatile uint8_t rearm_pending;
static uint8_t initialised;

static bt_rx_sink_t sink;
static bt_rx_stats_t stats;

/* ========================================================================== */
/* DMA plumbing                                                               */
/* ========================================================================== */
static int bt_rx_arm(void)
{
  /* Normal (not circular) mode: one bounded window per burst. */
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_dma, (uint16_t)sizeof(rx_dma)) != HAL_OK)
  {
    return -1;
  }

  /* See the header comment: a half-transfer event cannot be re-armed and would
   * duplicate bytes, so it is switched off after every arm. */
  __HAL_DMA_DISABLE_IT(&hdma_bt_rx, DMA_IT_HT);
  return 0;
}

static void bt_rx_dma_setup(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_bt_rx.Instance = BOARD_BT_RX_DMA;
  hdma_bt_rx.Init.Channel = BOARD_BT_RX_DMA_CHANNEL;
  hdma_bt_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_bt_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_bt_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_bt_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_bt_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_bt_rx.Init.Mode = DMA_NORMAL;
  hdma_bt_rx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_bt_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

  if (HAL_DMA_Init(&hdma_bt_rx) != HAL_OK)
  {
    return;
  }

  __HAL_LINKDMA(&huart5, hdmarx, hdma_bt_rx);

  /* Priority 5 == configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY: this IRQ leads
   * to the BT RX task wake-up, so it must never sit above a kernel critical
   * section.  app_init() re-asserts the whole table at the end anyway. */
  HAL_NVIC_SetPriority(BOARD_BT_RX_DMA_IRQn, BOARD_PRIO_BT_RX, 0U);
  HAL_NVIC_EnableIRQ(BOARD_BT_RX_DMA_IRQn);
}

/* Called from DMA1_Stream0_IRQHandler in stm32f4xx_it.c. */
void bt_rx_dma_irq_handler(void)
{
  HAL_DMA_IRQHandler(&hdma_bt_rx);
}

/* ========================================================================== */
/* public API                                                                 */
/* ========================================================================== */
void bt_rx_set_sink(bt_rx_sink_t fn)
{
  sink = fn;
}

int bt_rx_init(void)
{
  memset(&stats, 0, sizeof(stats));
  ring_head = 0U;
  ring_tail = 0U;
  line_len = 0U;
  line_overlong = 0U;
  rearm_pending = 0U;
  initialised = 0U;

  bt_rx_dma_setup();

  if (huart5.hdmarx == NULL)
  {
    return -1;
  }

  if (bt_rx_arm() != 0)
  {
    return -2;
  }

  initialised = 1U;
  return 0;
}

void bt_rx_isr_event(uint16_t size)
{
  uint16_t i;

  if ((size == 0U) || (size > BOARD_BT_RX_DMA_SIZE))
  {
    return;
  }

  stats.bytes += size;

  /* Single producer (this ISR) / single consumer (bt_rx_process): head and tail
   * are 16-bit aligned words, so the modular indices need no critical section. */
  for (i = 0U; i < size; i++)
  {
    uint16_t next = (uint16_t)((ring_head + 1U) & BT_RING_MASK);

    if (next == ring_tail)
    {
      /* Ring full: drop the rest of this burst and count it.  A lost command is
       * recoverable; an ISR that waits for the task is not. */
      stats.ring_drops += (uint32_t)(size - i);
      break;
    }

    ring[ring_head] = rx_dma[i];
    ring_head = next;
  }

  /* Re-arm for the next burst.  If it fails the task repairs it. */
  if (bt_rx_arm() != 0)
  {
    rearm_pending = 1U;
    stats.uart_errors++;
  }
}

void bt_rx_on_error(void)
{
  /* Interrupt context: no HAL calls, no RTOS calls.  Just ask the task to fix
   * the reception. */
  rearm_pending = 1U;
  stats.uart_errors++;
}

int bt_rx_process(void)
{
  int queued = 0;

  if (initialised == 0U)
  {
    return 0;
  }

  /* Error recovery belongs here, in task context (CODING_STANDARDS 5.2). */
  if (rearm_pending != 0U)
  {
    rearm_pending = 0U;
    (void)HAL_UART_AbortReceive(&huart5);
    if (bt_rx_arm() != 0)
    {
      rearm_pending = 1U;
    }
  }

  while (ring_tail != ring_head)
  {
    char c = (char)ring[ring_tail];

    ring_tail = (uint16_t)((ring_tail + 1U) & BT_RING_MASK);

    if (c == '\n')
    {
      if (line_overlong != 0U)
      {
        /* The tail of an over-long line ends here; the line itself was already
         * discarded as a whole. */
        stats.overlong++;
        line_overlong = 0U;
        line_len = 0U;
        continue;
      }

      line_buf[line_len] = '\0';
      stats.lines++;

      {
        cmd_msg_t msg;
        int st = bt_parse_line(line_buf, &msg);

        line_len = 0U;

        if (st == BT_OK)
        {
          if (sink == NULL)
          {
            /* No sink installed: the command is dropped, and counted by the
             * parser category so it does not look like a success. */
            stats.syntax_err++;
          }
          else
          {
            int r = sink(&msg);

            if (r == BT_OK)
            {
              stats.queued++;
              queued++;
            }
            else if (r == BT_ERR_BUSY)
            {
              stats.busy++;
            }
            else
            {
              stats.syntax_err++;
            }
          }
        }
        else if (st == BT_ERR_SYNTAX)
        {
          stats.syntax_err++;
        }
        else if (st == BT_ERR_RANGE)
        {
          stats.range_err++;
        }
        else
        {
          /* bt_parse_line() cannot return anything else; count it rather than
           * falling through silently. */
          stats.syntax_err++;
        }
      }
      continue;
    }

    if (line_overlong != 0U)
    {
      continue;   /* still discarding the rest of an over-long line */
    }

    if (c == '\r')
    {
      continue;   /* CRLF tolerated; a lone CR is simply not a terminator */
    }

    if (line_len >= (uint8_t)(BT_LINE_MAX - 1U))
    {
      /* Buffer is full and this is not the terminator: the line exceeds
       * BT_LINE_MAX, so reject it as a whole instead of truncating it into a
       * different (valid-looking) command. */
      line_overlong = 1U;
      line_len = 0U;
      continue;
    }

    line_buf[line_len] = c;
    line_len++;
  }

  return queued;
}

void bt_rx_get_stats(bt_rx_stats_t *out)
{
  if (out == NULL)
  {
    return;
  }
  memcpy(out, &stats, sizeof(*out));
}
