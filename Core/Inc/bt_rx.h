/* -----------------------------------------------------------------------------
 * bt_rx.h - Bluetooth receive path: UART5 bytes -> lines -> Q_Cmd.
 *
 * WHY IDLE-LINE DETECTION AND NOT A BYTE INTERRUPT
 * ------------------------------------------------
 * The command channel is ASCII text of unknown length, so "how many bytes came"
 * is only known when the line goes quiet.  HAL_UARTEx_ReceiveToIdle_DMA() gives
 * exactly that: DMA collects the burst, the IDLE event says it ended.  One
 * interrupt per command instead of one per byte, and no per-byte ISR to keep
 * short - which is what CODING_STANDARDS 0.4 asks for.
 *
 * ISR DOES NOTHING BUT COPY
 * -------------------------
 * The DMA window (BOARD_BT_RX_DMA_SIZE, 64 bytes) is about to be re-armed, so
 * the ISR copies the received bytes into a ring buffer and returns.  Line
 * assembly, parsing and queueing all happen in the BT RX task.  If the ring
 * overflows the bytes are dropped and counted - a command channel that loses a
 * burst is recoverable, one that blocks an ISR is not.
 *
 * THE SINK IS A CALLBACK, NOT A QUEUE HANDLE
 * ------------------------------------------
 * This module does not include cmsis_os2.h and does not know that Q_Cmd exists.
 * The BT RX task installs a sink once (bt_rx_set_sink) which is expected to be a
 * thin wrapper around osMessageQueuePut().  Two reasons: the module stays
 * testable on a PC, and the queueing policy (CODING_STANDARDS 2.2: overwrite
 * the oldest when full) lives next to the queue, not inside the parser.
 *
 * REPLIES: the "A G <x> <y>" / "E <code> <text>" downlink is the BT TX task's
 * job.  bt_rx_process() only reports what it did through its stats; wiring the
 * replies to the transmitter is a later step.
 * -------------------------------------------------------------------------- */

#ifndef BT_RX_H
#define BT_RX_H

#include <stdint.h>

#include "bt_protocol.h"

/* Ring buffer size.  One command is <= BT_LINE_MAX (32) bytes, so 256 bytes
 * holds several commands plus slack for a burst that arrives while the task is
 * not scheduled yet.  Must be a power of two (mask arithmetic is used). */
#define BT_RX_RING_SIZE   256U

/* Return value of the sink: BT_OK, or BT_ERR_BUSY when the command could not be
 * queued.  Anything else is treated as a rejection and counted. */
typedef int (*bt_rx_sink_t)(const cmd_msg_t *msg);

/* Install the command sink.  Call before bt_rx_process() is first used; passing
 * NULL removes it (then parsed commands are discarded and counted). */
void bt_rx_set_sink(bt_rx_sink_t sink);

/* Start the receive path: arming DMA1_Stream0 with idle-line detection and
 * clearing the ring.  Call once, before the scheduler starts.
 * Returns 0 on success, negative if the UART refused the DMA. */
int bt_rx_init(void);

/* Interrupt-context entry point, called from the HAL receive-to-idle callback
 * with the number of bytes now sitting in the DMA window (1..DMA_SIZE).
 *
 * Copies those bytes into the ring, drops them if the ring is full, re-arms
 * reception and returns.  No malloc, no parsing, no queueing. */
void bt_rx_isr_event(uint16_t size);

/* Task-context worker: drain whole lines out of the ring, parse them with
 * bt_parse_line() and hand the accepted ones to the sink.  Safe to call even
 * when no complete line has arrived yet (returns quickly).
 *
 * Returns the number of commands successfully passed to the sink (0 or more). */
int bt_rx_process(void);

/* ---- observability ------------------------------------------------------- */
typedef struct
{
  uint32_t bytes;      /* bytes received from the UART                     */
  uint32_t lines;      /* complete lines assembled                         */
  uint32_t ring_drops; /* bytes lost because the ring was full             */
  uint32_t overlong;   /* lines longer than BT_LINE_MAX, discarded         */
  uint32_t syntax_err; /* BT_ERR_SYNTAX from the parser                   */
  uint32_t range_err;  /* BT_ERR_RANGE from the parser                    */
  uint32_t queued;     /* commands accepted by the sink                    */
  uint32_t busy;       /* sink returned BT_ERR_BUSY                        */
} bt_rx_stats_t;

void bt_rx_get_stats(bt_rx_stats_t *out);

#endif /* BT_RX_H */
