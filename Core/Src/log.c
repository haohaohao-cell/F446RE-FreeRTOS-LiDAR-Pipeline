/* -----------------------------------------------------------------------------
 * log.c - USART2 TX ring buffer + DMA.  See log.h for why.
 *
 * Ring discipline: the producer (log_write, task context) moves `head`, the
 * consumer (kick, task or DMA-completion ISR) moves `tail`.  The only real race
 * is "task is copying a chunk while the completion ISR fires", and that cannot
 * happen: a completion ISR can only fire while `busy != 0`, and kick() returns
 * immediately when busy is set.  The task-side critical section still covers
 * head/tail together so two logging tasks cannot interleave a copy.
 * -------------------------------------------------------------------------- */

#include "log.h"
#include "board_pins.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

extern UART_HandleTypeDef huart2;

#define LOG_RING_SIZE   2048U
#define LOG_RING_MASK   (LOG_RING_SIZE - 1U)

/* Bytes per DMA transfer.  ~11 ms at 115200 baud; small enough that the ring
 * absorbs it, large enough that per-transfer overhead is irrelevant. */
#define LOG_CHUNK_MAX   128U

static uint8_t ring[LOG_RING_SIZE];
static volatile uint16_t head;
static volatile uint16_t tail;

/* DMA needs one contiguous buffer, so a chunk is copied out of the (wrapping)
 * ring first.  It must outlive the transfer, hence static. */
static uint8_t chunk[LOG_CHUNK_MAX];
static volatile uint8_t busy;

static DMA_HandleTypeDef hdma_usart2_tx;
static log_stats_t stats;

static void log_dma_setup(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_usart2_tx.Instance = DMA1_Stream6;   /* USART2_TX is DMA1 ch4 stream 6 */
  hdma_usart2_tx.Init.Channel = DMA_CHANNEL_4;
  hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart2_tx.Init.Mode = DMA_NORMAL;
  hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

  if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
  {
    stats.dma_errors++;
    return;
  }

  __HAL_LINKDMA(&huart2, hdmatx, hdma_usart2_tx);

  /* 6: this ISR calls no RTOS API, and it must stay numerically >= 5 so it can
   * never preempt a kernel critical section. */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, BOARD_PRIO_BT_TX_DMA, 0U);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

int log_init(void)
{
  memset(&stats, 0, sizeof(stats));
  head = 0U;
  tail = 0U;
  busy = 0U;

  log_dma_setup();
  return (huart2.hdmatx != NULL) ? 0 : -1;
}

/* Copy the next chunk out of the ring and start its DMA transfer.
 * Must be called with `busy == 0`.
 *
 * `from_isr` selects the locking: with the scheduler running, a task needs a
 * critical section around head/tail, but an ISR must NOT take one
 * (taskENTER_CRITICAL() manipulates the kernel nesting counter and is only
 * legal from task context).  The ISR does not need it either: it runs at NVIC
 * priority 6, and the task-side critical section raises BASEPRI to mask
 * everything >= 5, so a completion ISR can never fire inside the producer's
 * critical section.  Returns the number of bytes started. */
static uint16_t log_kick(int from_isr)
{
  uint16_t n = 0U;
  uint16_t avail;

  if (busy != 0U)
  {
    return 0U;
  }

  /* Two preconditions, both learned from the F4 HAL's own behaviour:
   *
   *  - hdmatx != NULL: if log_init() could not wire the stream, the HAL would
   *    dereference a NULL handle inside HAL_UART_Transmit_DMA.
   *
   *  - gState must be READY.  HAL_UART_TxCpltCallback is invoked TWICE per
   *    transfer: once from the DMA complete ISR (which only enables the UART
   *    "transmission complete" interrupt - the last byte is still shifting out,
   *    gState is still BUSY_TX) and once from the UART TC ISR (gState READY).
   *    Starting the next chunk at the first one would fail with HAL_BUSY *after*
   *    the bytes had already been removed from the ring, i.e. silently drop
   *    them.  Checking here means the second invocation starts the chunk. */
  if ((huart2.hdmatx == NULL) || (huart2.gState != HAL_UART_STATE_READY))
  {
    return 0U;
  }

  if (from_isr == 0)
  {
    taskENTER_CRITICAL();
  }

  avail = (uint16_t)((head - tail) & LOG_RING_MASK);
  while ((n < avail) && (n < LOG_CHUNK_MAX))
  {
    chunk[n] = ring[(uint16_t)((tail + n) & LOG_RING_MASK)];
    n++;
  }
  tail = (uint16_t)((tail + n) & LOG_RING_MASK);
  if (n != 0U)
  {
    busy = 1U;        /* claim the DMA before releasing the ring */
  }

  if (from_isr == 0)
  {
    taskEXIT_CRITICAL();
  }

  if (n == 0U)
  {
    return 0U;
  }

  if (HAL_UART_Transmit_DMA(&huart2, chunk, n) != HAL_OK)
  {
    /* Could not start: release the claim and count it.  The bytes are already
     * out of the ring, i.e. lost - hence an error counter rather than a retry
     * (a retry would reorder output). */
    busy = 0U;
    stats.dma_errors++;
    return 0U;
  }

  stats.chunks++;
  return n;
}

int log_write(const char *buf, uint16_t len)
{
  uint16_t i;
  uint16_t dropped = 0U;

  if ((buf == NULL) || (len == 0U))
  {
    return 0;
  }

  taskENTER_CRITICAL();
  for (i = 0U; i < len; i++)
  {
    uint16_t next = (uint16_t)((head + 1U) & LOG_RING_MASK);

    if (next == tail)
    {
      /* Ring full: drop the remainder.  A truncated boot log is a diagnostic
       * problem; blocking the caller would be a scheduling problem. */
      dropped = (uint16_t)(len - i);
      break;
    }
    ring[head] = (uint8_t)buf[i];
    head = next;
  }
  if (dropped == 0U)
  {
    stats.bytes += len;
  }
  else
  {
    stats.bytes += (uint32_t)(len - dropped);
    stats.drops += dropped;
  }
  taskEXIT_CRITICAL();

  (void)log_kick(0);
  return (dropped == 0U) ? 0 : -1;
}

void log_on_tx_complete(void)
{
  /* Called from the DMA TX-complete path (HAL_UART_TxCpltCallback in app.c),
   * i.e. interrupt context. */
  busy = 0U;
  (void)log_kick(1);
}

void log_dma_irq_handler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

void log_get_stats(log_stats_t *out)
{
  if (out == NULL)
  {
    return;
  }
  memcpy(out, &stats, sizeof(*out));
}
