/* -----------------------------------------------------------------------------
 * lidar.c - RPLidar C1 driver, RTOS-side block parser
 *
 * Block 4.5 split of responsibilities:
 *
 *   [USART3 + DMA1_Stream1]  circular buffer of LIDAR_DMA_SIZE bytes
 *            |
 *            |-- DMA half-complete IRQ --> lidar_on_half()   --+
 *            |-- DMA complete IRQ      --> lidar_on_full()   --+--> Queue 1
 *            |-- USART3 error IRQ      --> lidar_on_uart_error()+
 *            |
 *            v  (raw bytes STAY here in memory)
 *   lidar_parser_process()   <- LiDAR Parser Task, reads offset/length
 *            |
 *            +--> persistent 360-bin scan  (used by the telemetry frame)
 *            +--> lidar_result_t           --> Queue 2 --> Output/Debug Task
 *
 * Nothing in this file is allowed to parse LiDAR data inside an interrupt.
 * -------------------------------------------------------------------------- */

#include "lidar.h"
#include "main.h"
#include "app_rtos.h"
#include <string.h>

extern UART_HandleTypeDef huart3;

/* Raw circular DMA destination.  Deliberately a plain global so that the
 * parser task can read it directly from any block offset. */
uint8_t lidar_dma_buf[LIDAR_DMA_SIZE];

/* ---- persistent scan map (written only by the parser task) --------------- */
static uint16_t scan_dist[LIDAR_BINS];
static uint8_t  scan_qual[LIDAR_BINS];

/* ---- cumulative diagnostics --------------------------------------------- */
static lidar_stats_t stats;

/* ---- per-block accumulator (parser task only) ---------------------------- */
static lidar_result_t blk;
static uint8_t blk_touched[(LIDAR_BINS + 7U) / 8U];

/* ---- stream synchroniser state (parser task only) ------------------------ */
static uint8_t pend[5];
static uint8_t pend_n;
static uint8_t desc_n;
static uint8_t in_scan;
static uint32_t last_cmd_ms;

/* ---- stream watchdog ---------------------------------------------------- */
/* The RPLidar C1 does not stream after it is power-cycled: it needs the SCAN
 * command (0xA5 0x20) again.  So "no completed block for a while" must be
 * treated as "the stream is gone, unlock and ask for SCAN again", otherwise a
 * LiDAR that was unplugged and replugged (or that reset itself) would stay
 * silent forever.  This is what makes the demonstrate-the-recovery step work. */
#define LIDAR_STREAM_TIMEOUT_MS   1000U   /* no block for this long = lost     */
#define LIDAR_RESYNC_BLOCKS       50U     /* blocks without descriptor = mid-stream */

static volatile uint32_t last_block_ms;   /* updated in the ISR                */
static uint16_t sync_wait_blocks;         /* blocks seen while still unlocked  */
static uint8_t  stream_lost;

/* ---- debug-console requests -------------------------------------------- */
/* Set by the Output task when 's' / 'g' is typed on USART2; acted on by the
 * parser task so that in_scan / pend / desc_n keep exactly one owner. */
static volatile uint8_t req_stop;
static volatile uint8_t req_resume;
static uint8_t paused;                    /* deliberately stopped by the user  */

/* ---- ISR <-> task handshake --------------------------------------------- */
static volatile uint8_t rearm_pending;   /* set while a restart is queued      */
static volatile uint16_t block_seq;      /* incremented for every block        */

static const uint8_t scan_cmd[2] = {0xA5U, 0x20U};
static const uint8_t stop_cmd[2] = {0xA5U, 0x25U};

/* RPLidar SCAN-mode response descriptor: A5 5A 05 00 00 40 81 */
static const uint8_t desc[7] = {0xA5U, 0x5AU, 0x05U, 0x00U, 0x00U, 0x40U, 0x81U};

/* ========================================================================== */
/* helpers                                                                    */
/* ========================================================================== */

static uint32_t cycles_per_us(void)
{
  uint32_t f = SystemCoreClock / 1000000U;
  return (f == 0U) ? 1U : f;
}

static void send_cmd(const uint8_t *cmd, uint16_t len)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)cmd, len, 20U);
  last_cmd_ms = HAL_GetTick();
  stats.commands++;
}

/* ========================================================================== */
/* public: start / stop / restart                                             */
/* ========================================================================== */

void lidar_init(void)
{
  memset(&stats, 0, sizeof(stats));
  memset(scan_dist, 0, sizeof(scan_dist));
  memset(scan_qual, 0, sizeof(scan_qual));
  pend_n = 0U;
  desc_n = 0U;
  in_scan = 0U;
  rearm_pending = 0U;
  block_seq = 0U;
  sync_wait_blocks = 0U;
  stream_lost = 1U;                 /* nothing received yet */
  req_stop = 0U;
  req_resume = 0U;
  paused = 0U;
  last_block_ms = HAL_GetTick();
  HAL_UART_Receive_DMA(&huart3, lidar_dma_buf, LIDAR_DMA_SIZE);
  send_cmd(scan_cmd, sizeof(scan_cmd));
}

void lidar_stop(void)
{
  HAL_UART_AbortReceive(&huart3);
  send_cmd(stop_cmd, sizeof(stop_cmd));
}

/* Re-arm the circular DMA reception and restart the parser state machine.
 * Runs in task context only (see lidar_parser_service). */
void lidar_restart(void)
{
  stats.restarts++;
  HAL_UART_AbortReceive(&huart3);
  pend_n = 0U;
  desc_n = 0U;
  in_scan = 0U;
  sync_wait_blocks = 0U;
  stream_lost = 1U;
  last_block_ms = HAL_GetTick();
  if (HAL_UART_Receive_DMA(&huart3, lidar_dma_buf, LIDAR_DMA_SIZE) != HAL_OK)
  {
    stats.restart_fail++;
  }
  send_cmd(scan_cmd, sizeof(scan_cmd));
  rearm_pending = 0U;
}

/* ========================================================================== */
/* interrupt context - deliberately trivial                                   */
/* ========================================================================== */

static void post_block(uint16_t offset)
{
  lidar_block_desc_t d;

  if (rearm_pending != 0U)
  {
    return;   /* a re-arm is already queued, do not describe stale data */
  }

  d.offset = offset;
  d.length = LIDAR_HALF_SIZE;
  d.block_id = block_seq++;
  d.kind = (uint8_t)LIDAR_BLOCK_DATA;
  d.reserved = 0U;
  d.isr_cycles = DWT->CYCCNT;

  /* feed the stream watchdog (plain 32-bit store, ISR-safe) */
  last_block_ms = HAL_GetTick();
  if (in_scan == 0U)
  {
    if (sync_wait_blocks < 0xFFFFU)
    {
      sync_wait_blocks++;
    }
  }

  stats.blocks++;
  if (app_rtos_post_block(&d) != 0)
  {
    stats.q_full++;   /* parser task fell behind: drop, the block is lost */
  }
}

void lidar_on_half(void)
{
  stats.ht_count++;
  post_block(0U);
}

void lidar_on_full(void)
{
  stats.tc_count++;
  post_block(LIDAR_HALF_SIZE);
}

void lidar_on_uart_error(void)
{
  lidar_block_desc_t d;

  stats.uart_errors++;
  if (rearm_pending != 0U)
  {
    return;
  }
  rearm_pending = 1U;

  d.offset = 0U;
  d.length = 0U;
  d.block_id = block_seq++;
  d.kind = (uint8_t)LIDAR_BLOCK_UART_ERROR;
  d.reserved = 0U;
  d.isr_cycles = DWT->CYCCNT;

  (void)app_rtos_post_block(&d);
}

/* ========================================================================== */
/* parser task side: byte-stream decoding                                    */
/* ========================================================================== */

static void touch_bin(uint16_t bin)
{
  blk_touched[bin >> 3] |= (uint8_t)(1U << (bin & 7U));
}

static void decode_packet(const uint8_t *p)
{
  uint16_t angle_q6 = (uint16_t)(((uint16_t)p[1] >> 1) | ((uint16_t)p[2] << 7));
  uint16_t dist_q2  = (uint16_t)((uint16_t)p[3] | ((uint16_t)p[4] << 8));
  uint8_t  quality  = (uint8_t)(p[0] >> 2);
  uint8_t  valid    = (uint8_t)(p[1] & 0x01U);

  blk.packets++;
  stats.packets++;

  if ((valid == 0U) || (angle_q6 >= (LIDAR_BINS * 64U)))
  {
    blk.invalid++;
    stats.invalid++;
    return;
  }
  if (dist_q2 == 0U)
  {
    blk.no_return++;
    stats.no_return++;
    return;
  }

  uint16_t bin = (uint16_t)(angle_q6 >> 6);
  uint16_t mm  = (uint16_t)(dist_q2 >> 2);
  uint8_t  sector = (uint8_t)(bin / (LIDAR_BINS / LIDAR_SECTORS));

  blk.valid++;
  scan_dist[bin] = mm;
  scan_qual[bin] = quality;
  touch_bin(bin);

  if (mm < blk.min_mm) { blk.min_mm = mm; }
  if (mm > blk.max_mm) { blk.max_mm = mm; }

  {
    uint16_t cm = (uint16_t)(mm / 10U);
    if (cm > 255U) { cm = 255U; }
    if ((blk.sector_min_cm[sector] == LIDAR_SECTOR_EMPTY) ||
        (cm < blk.sector_min_cm[sector]))
    {
      blk.sector_min_cm[sector] = (uint8_t)cm;
    }
  }
}

/* Walk one DMA block of the continuous LiDAR byte stream.
 * The synchroniser state (desc_n / pend_n / in_scan) is intentionally kept
 * across blocks, because a LiDAR packet may straddle two DMA blocks. */
static void process_block(const uint8_t *buf, uint16_t len)
{
  for (uint16_t i = 0U; i < len; i++)
  {
    uint8_t b = buf[i];
    stats.bytes++;

    if (in_scan == 0U)
    {
      /* Hunting for the SCAN-mode descriptor */
      if (b == desc[desc_n])
      {
        desc_n++;
        if (desc_n == sizeof(desc))
        {
          desc_n = 0U;
          in_scan = 1U;
          pend_n = 0U;
          stats.syncs++;
          blk.syncs++;
        }
      }
      else
      {
        desc_n = (uint8_t)((b == desc[0]) ? 1U : 0U);
      }
      continue;
    }

    pend[pend_n++] = b;

    /* 5-byte packet framing: bit0 of byte0 is the check bit, bit0 of
     * byte1 is the "start flag" - both alternate for a valid frame. */
    if ((pend_n == 1U) && ((((pend[0] ^ (pend[0] >> 1)) & 0x01U)) == 0U))
    {
      pend_n = 0U;
      stats.slips++;
      blk.slips++;
      continue;
    }
    if ((pend_n == 2U) && ((pend[1] & 0x01U) == 0U))
    {
      pend[0] = pend[1];
      pend_n = (uint8_t)(((((pend[0] ^ (pend[0] >> 1)) & 0x01U)) != 0U) ? 1U : 0U);
      stats.slips++;
      blk.slips++;
      continue;
    }
    if (pend_n == 5U)
    {
      pend_n = 0U;
      decode_packet(pend);
    }
  }
}

void lidar_parser_process(const lidar_block_desc_t *desc_in, lidar_result_t *out)
{
  uint32_t t_entry = DWT->CYCCNT;   /* task woken up: handover ends here   */
  uint32_t t0;
  uint32_t dt;
  uint32_t cpu;

  /* ISR -> task wake-up latency.  Must be sampled BEFORE any parsing work,
   * otherwise it would also contain the parse time. */
  cpu = cycles_per_us();

  memset(&blk, 0, sizeof(blk));
  memset(blk_touched, 0, sizeof(blk_touched));
  for (uint8_t s = 0U; s < LIDAR_SECTORS; s++)
  {
    blk.sector_min_cm[s] = LIDAR_SECTOR_EMPTY;
  }
  blk.min_mm = 0xFFFFU;
  blk.max_mm = 0U;
  blk.block_id = desc_in->block_id;
  blk.length = desc_in->length;

  blk.handover_us = (t_entry - desc_in->isr_cycles) / cpu;

  t0 = DWT->CYCCNT;

  if (desc_in->kind == (uint8_t)LIDAR_BLOCK_DATA)
  {
    process_block(&lidar_dma_buf[desc_in->offset], desc_in->length);
  }

  /* counted nodes = popcount of touched bins */
  {
    uint16_t nodes = 0U;
    for (uint8_t i = 0U; i < sizeof(blk_touched); i++)
    {
      uint8_t v = blk_touched[i];
      while (v != 0U)
      {
        nodes = (uint16_t)(nodes + (v & 1U));
        v >>= 1;
      }
    }
    blk.nodes = nodes;
  }

  if (blk.min_mm == 0xFFFFU)
  {
    blk.min_mm = 0U;   /* no return at all in this block */
  }

  dt = DWT->CYCCNT - t0;
  blk.parse_us = dt / cpu;

  if (blk.parse_us + blk.handover_us > stats.proc_us_max)
  {
    stats.proc_us_max = blk.parse_us + blk.handover_us;
  }
  blk.synced = in_scan;

  *out = blk;
}

void lidar_parser_service(void)
{
  uint32_t now = HAL_GetTick();

  /* (a) operator asked for a deliberate stop / restart from the console.
   * Handled here, in the parser task, so the stream state machine still has
   * exactly one owner. */
  if (req_stop != 0U)
  {
    req_stop = 0U;
    paused = 1U;
    HAL_UART_AbortReceive(&huart3);
    send_cmd(stop_cmd, sizeof(stop_cmd));
    in_scan = 0U;
    pend_n = 0U;
    desc_n = 0U;
    sync_wait_blocks = 0U;
    return;
  }
  if (req_resume != 0U)
  {
    req_resume = 0U;
    paused = 0U;
    lidar_restart();
    return;
  }
  if (paused != 0U)
  {
    return;   /* stopped on purpose: keep every watchdog quiet */
  }

  /* (0) a UART/DMA error was signalled by the ISR: re-arm in task context */
  if (rearm_pending != 0U)
  {
    lidar_restart();
    return;
  }

  /* (1) Stream watchdog.  No completed DMA block for LIDAR_STREAM_TIMEOUT_MS
   * means the sensor is gone, was power-cycled, or reset itself.  Drop the
   * lock so the SCAN command is re-issued - the C1 does NOT stream by itself
   * after power-up, so without this a replugged LiDAR would stay silent. */
  if ((uint32_t)(now - last_block_ms) > LIDAR_STREAM_TIMEOUT_MS)
  {
    if (stream_lost == 0U)
    {
      stats.stream_losses++;
    }
    stream_lost = 1U;
    in_scan = 0U;
    pend_n = 0U;
    desc_n = 0U;
    sync_wait_blocks = 0U;
    last_block_ms = now;      /* do not retrigger on every call */
  }
  else
  {
    stream_lost = 0U;
  }

  if (stream_lost != 0U)
  {
    /* Retry the SCAN command once per second until data comes back. */
    if ((uint32_t)(now - last_cmd_ms) >= LIDAR_STREAM_TIMEOUT_MS)
    {
      send_cmd(scan_cmd, sizeof(scan_cmd));
    }
  }
  else if ((in_scan == 0U) && (sync_wait_blocks >= LIDAR_RESYNC_BLOCKS))
  {
    /* Data IS arriving but the A5 5A descriptor never showed up: this is a
     * mid-stream start (e.g. the sensor was already scanning).  Assume the
     * frame alignment rather than keep the stream locked out forever. */
    in_scan = 1U;
    pend_n = 0U;
  }
  else
  {
    /* data flowing, still hunting for the descriptor - nothing to do */
  }
}

/* ========================================================================== */
/* shared with the Output/Debug task                                          */
/* ========================================================================== */

void lidar_get_scan(uint16_t *dist_mm, uint8_t *quality)
{
  memcpy(dist_mm, scan_dist, sizeof(scan_dist));
  memcpy(quality, scan_qual, sizeof(scan_qual));
}

void lidar_get_stats(lidar_stats_t *out)
{
  memcpy(out, &stats, sizeof(stats));
}

/* --------------------------------------------------------------------------
 * Debug-console requests.  Called from the Output task (or any task); they
 * only raise a flag, the parser task does the work.  Single-byte volatile
 * stores are atomic on Cortex-M, so no lock is needed.
 * -------------------------------------------------------------------------- */
void lidar_request_stop(void)
{
  req_stop = 1U;
}

void lidar_request_resume(void)
{
  req_resume = 1U;
}
