/* -----------------------------------------------------------------------------
 * log.h - non-blocking debug console output on USART2 (115200, ST-Link VCP).
 *
 * WHY THIS EXISTS: dbg_printf() used to be a blocking HAL_UART_Transmit with a
 * 100 ms timeout.  One 200-character status line at 115200 baud occupies the
 * UART for ~17 ms, and CODING_STANDARDS 5.4 forbids that in a system with 10 ms
 * deadlines: even from the lowest-priority task it delays the display slot and
 * every other print behind it.  So the text goes into a ring buffer and leaves
 * through TX DMA; a full ring drops bytes and counts them instead of waiting.
 *
 * Callers keep using dbg_printf() (app.h) for formatting.  This module only owns
 * the transport.
 * -------------------------------------------------------------------------- */

#ifndef LOG_H
#define LOG_H

#include <stdint.h>

/* Set up DMA1_Stream6 (USART2_TX, channel 4) and clear the ring.  Call once,
 * before the scheduler starts.  Returns 0 on success. */
int log_init(void);

/* Queue len bytes for transmission.  Never blocks and never waits for the UART:
 * returns 0 if the bytes were queued, -1 if they were dropped because the ring
 * was full (counted in log_stats_t.drops). */
int log_write(const char *buf, uint16_t len);

/* DMA completion entry point, called from DMA1_Stream6_IRQHandler. */
void log_dma_irq_handler(void);

/* USART2 TX-complete notification, called from HAL_UART_TxCpltCallback (app.c).
 * Interrupt context: it starts the next queued chunk and returns. */
void log_on_tx_complete(void);

typedef struct
{
  uint32_t bytes;       /* bytes accepted into the ring                    */
  uint32_t drops;       /* bytes discarded because the ring was full        */
  uint32_t chunks;      /* DMA transfers started                            */
  uint32_t dma_errors;  /* HAL_UART_Transmit_DMA refused to start           */
} log_stats_t;

void log_get_stats(log_stats_t *out);

#endif /* LOG_H */
