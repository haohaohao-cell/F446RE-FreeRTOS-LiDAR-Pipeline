/* -----------------------------------------------------------------------------
 * app_rtos.c - Block 4.5 Coding Day: the FreeRTOS side of the LiDAR pipeline
 *
 *                       ISR (USART3 / DMA1_Stream1)
 *                                |
 *                                | osMessageQueuePut(timeout = 0, ISR ctx)
 *                                v
 *                        +---------------+
 *                        |    Queue 1    |  8 x lidar_block_desc_t (12 bytes)
 *                        +---------------+
 *                                |
 *                                | osMessageQueueGet(osWaitForever)
 *                                v
 *                     LiDAR Parser Task   (osPriorityAboveNormal)
 *                                |
 *                                | osMessageQueuePut(timeout = 0)
 *                                v
 *                        +---------------+
 *                        |    Queue 2    |  4 x lidar_result_t (32 bytes)
 *                        +---------------+
 *                                |
 *                                | osMessageQueueGet(100 ms guard timeout)
 *                                v
 *                    Output / Debug Task   (osPriorityNormal)
 *                                |
 *                 +--------------+--------------+
 *                 |                             |
 *          USART2 text (115200)          UART5 binary (230400)
 *
 * Priority rationale: the parser must always be able to drain Queue 1 before
 * DMA wraps around and overwrites the next block, so it runs above the Output
 * task.  The Output task is deliberately the lower one - it only formats and
 * pushes bytes out and may legitimately be preempted.
 * -------------------------------------------------------------------------- */

#include "app_rtos.h"
#include "app.h"
#include "main.h"
#include "lidar.h"
#include "imu.h"
#include "telemetry.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <string.h>

extern UART_HandleTypeDef huart2;   /* debug console, polled for 's' / 'g' */

/* -------------------------------------------------------------------------- */
/* RTOS objects                                                               */
/* -------------------------------------------------------------------------- */
static osMessageQueueId_t q_blocks;    /* Queue 1: ISR      -> Parser Task     */
static osMessageQueueId_t q_results;   /* Queue 2: Parser   -> Output Task     */
static osThreadId_t       t_parser;
static osThreadId_t       t_output;

static app_rtos_stats_t rtos_stats;

/* Snapshot of the rolling 360-bin scan, assembled by the Output task */
static uint16_t scan_frame[LIDAR_BINS];
static uint8_t  qual_frame[LIDAR_BINS];

/* -------------------------------------------------------------------------- */
/* Task attributes                                                            */
/* -------------------------------------------------------------------------- */
static const osThreadAttr_t parser_attr = {
  .name       = "lidarParser",
  .stack_size = APP_PARSER_STACK,
  .priority   = (osPriority_t)osPriorityAboveNormal,
};

static const osThreadAttr_t output_attr = {
  .name       = "outputDebug",
  .stack_size = APP_OUTPUT_STACK,
  .priority   = (osPriority_t)osPriorityNormal,
};

static const osMessageQueueAttr_t q_blocks_attr  = { .name = "qBlocks" };
static const osMessageQueueAttr_t q_results_attr = { .name = "qResults" };

/* ========================================================================== */
/* ISR side                                                                   */
/* ========================================================================== */

/* CMSIS-RTOS v2 has no "FromISR" variants.  ST's cmsis_os2.c detects interrupt
 * context itself and switches to xQueueSendToBackFromISR() + portYIELD_FROM_ISR().
 * Consequence: from an ISR the timeout argument MUST be 0.
 * Precondition: the calling ISR's NVIC priority must be numerically >=
 * configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, otherwise it triggers
 * configASSERT (vPortValidateInterruptPriority). */
int app_rtos_post_block(const lidar_block_desc_t *desc)
{
  if (osMessageQueuePut(q_blocks, desc, 0U, 0U) != osOK)
  {
    rtos_stats.blocks_dropped++;
    return -1;
  }
  rtos_stats.blocks_posted++;
  return 0;
}

/* ========================================================================== */
/* Task 1: LiDAR Parser Task                                                  */
/* ========================================================================== */
static void parser_task(void *argument)
{
  lidar_block_desc_t desc;
  lidar_result_t     result;

  (void)argument;

  for (;;)
  {
    osStatus_t st;

    /* ---- BLOCKED here until Queue 1 receives a block descriptor.
     *
     * The timeout is NOT used to poll for data - the data path is purely
     * event driven.  It only exists so that the stream watchdog inside
     * lidar_parser_service() still runs when the sensor is silent (e.g. the
     * SCAN command was lost at power-up): with osWaitForever the parser would
     * block forever and never re-issue the command, and the pipeline would
     * deadlock silently.  100 ms is ~4 block periods, so in normal operation
     * the timeout never fires. ------------------------------------------- */
    st = osMessageQueueGet(q_blocks, &desc, NULL, APP_PARSER_IDLE_MS);

    if (st == osOK)
    {
      rtos_stats.blocks_consumed++;
      {
        uint32_t used = osMessageQueueGetCount(q_blocks);
        if (used > rtos_stats.q1_peak)
        {
          rtos_stats.q1_peak = used;
        }
      }

      /* ---- READY -> RUNNING: parse the block straight out of the DMA buf */
      lidar_parser_process(&desc, &result);
      result.tick = osKernelGetTickCount();

      if (result.parse_us > rtos_stats.max_parse_us)
      {
        rtos_stats.max_parse_us = result.parse_us;
      }
      if (result.handover_us > rtos_stats.max_handover_us)
      {
        rtos_stats.max_handover_us = result.handover_us;
      }

      /* ---- hand the processed result to the Output task ---------------- */
      if (osMessageQueuePut(q_results, &result, 0U, 0U) != osOK)
      {
        rtos_stats.results_dropped++;
      }
      else
      {
        rtos_stats.results_posted++;
      }
      {
        uint32_t used = osMessageQueueGetCount(q_results);
        if (used > rtos_stats.q2_peak)
        {
          rtos_stats.q2_peak = used;
        }
      }
    }
    else
    {
      rtos_stats.parser_idle_wakeups++;
    }

    /* Re-arm / resync work that is not allowed in interrupt context.
     * Runs on every wake-up, including the "no block arrived" case. */
    lidar_parser_service();
  }
}

/* ========================================================================== */
/* Task 2: Output / Debug Task                                                */
/* ========================================================================== */
static void output_task(void *argument)
{
  lidar_result_t  last;
  uint8_t         have_result = 0U;
  uint32_t        t_frame;
  uint32_t        t_stats;
  uint32_t        prev_bytes = 0U;
  uint32_t        prev_ms = 0U;
  uint32_t        prev_ktick = 0U;
  uint32_t        prev_blocks = 0U;

  (void)argument;

  t_frame = osKernelGetTickCount();
  t_stats = t_frame;
  prev_ms = HAL_GetTick();
  prev_ktick = t_frame;

  for (;;)
  {
    /* ---- BLOCKED here until the parser posts a result.
     * The 100 ms guard timeout still lets this task service the IMU and the
     * 1 Hz statistics line if the LiDAR stops producing blocks. ---------- */
    if (osMessageQueueGet(q_results, &last, NULL, APP_FRAME_PERIOD_MS) == osOK)
    {
      have_result = 1U;
    }

    /* ---- debug console: one-shot non-blocking read of USART2 RX ---------
     *   s  -> stop the LiDAR   (the cheap, cable-free way to demonstrate
     *                           "sensor goes silent" without touching wiring)
     *   g  -> restart the LiDAR
     *   h  -> help                                                          */
    {
      uint8_t ch = 0U;
      if (HAL_UART_Receive(&huart2, &ch, 1U, 0U) == HAL_OK)
      {
        if ((ch == (uint8_t)'s') || (ch == (uint8_t)'S'))
        {
          lidar_request_stop();
          dbg_printf("\r\n[cmd] s -> stopping LiDAR (watch blk / idle)\r\n");
        }
        else if ((ch == (uint8_t)'g') || (ch == (uint8_t)'G'))
        {
          lidar_request_resume();
          dbg_printf("\r\n[cmd] g -> restarting LiDAR\r\n");
        }
        else if ((ch == (uint8_t)'h') || (ch == (uint8_t)'H') || (ch == (uint8_t)'?'))
        {
          dbg_printf("\r\n[cmd] s = stop LiDAR, g = restart LiDAR\r\n");
        }
        else
        {
          /* ignore anything else (including the CR/LF of the terminal) */
        }
      }
    }

    imu_poll();

    /* ---- 10 Hz: binary telemetry frame + one text summary -------------- */
    if ((uint32_t)(osKernelGetTickCount() - t_frame) >= APP_FRAME_PERIOD_MS)
    {
      imu_sample_t sample;

      t_frame = osKernelGetTickCount();

      /* The parser task is the only writer of the 360-bin map.  A short
       * critical section guarantees a consistent snapshot: it stops the
       * kernel from switching back to the parser halfway through. */
      taskENTER_CRITICAL();
      lidar_get_scan(scan_frame, qual_frame);
      taskEXIT_CRITICAL();

      imu_get(&sample);
      /* Non-blocking: if the UART5 DMA is still busy this frame is dropped and
       * counted in telemetry_stats_t.drops. */
      (void)telemetry_send_data(&sample, scan_frame);

      if (have_result != 0U)
      {
        dbg_printf("#%04u p%u ok%u nr%u iv%u n%u d[%u..%u] lat%lu p%lu "
                   "S %u %u %u %u %u %u %u %u\r\n",
                   (unsigned)last.block_id,
                   (unsigned)last.packets,
                   (unsigned)last.valid,
                   (unsigned)last.no_return,
                   (unsigned)last.invalid,
                   (unsigned)last.nodes,
                   (unsigned)last.min_mm,
                   (unsigned)last.max_mm,
                   (unsigned long)last.handover_us,
                   (unsigned long)last.parse_us,
                   (unsigned)last.sector_min_cm[0],
                   (unsigned)last.sector_min_cm[1],
                   (unsigned)last.sector_min_cm[2],
                   (unsigned)last.sector_min_cm[3],
                   (unsigned)last.sector_min_cm[4],
                   (unsigned)last.sector_min_cm[5],
                   (unsigned)last.sector_min_cm[6],
                   (unsigned)last.sector_min_cm[7]);
      }
    }

    /* ---- 1 Hz: statistics --------------------------------------------- */
    if ((uint32_t)(osKernelGetTickCount() - t_stats) >= APP_STATS_PERIOD_MS)
    {
      lidar_stats_t ls;
      imu_stats_t   is;
      telemetry_stats_t ts;
      app_rtos_stats_t  rs;
      uint32_t now = HAL_GetTick();
      uint32_t now_k = osKernelGetTickCount();
      uint32_t span = now - prev_ms;
      uint32_t span_k = now_k - prev_ktick;
      uint32_t bps;

      lidar_get_stats(&ls);
      imu_get_stats(&is);
      telemetry_get_stats(&ts);
      app_rtos_get_stats(&rs);

      if (span == 0U)
      {
        span = 1U;
      }
      bps = (uint32_t)(((uint64_t)(ls.bytes - prev_bytes) * 1000U) / span);
      prev_bytes = ls.bytes;
      prev_ms = now;
      prev_ktick = now_k;

      (void)telemetry_send_stats(ts.frames, bps, &ls, &is);

      dbg_printf("[1s] blk %lu/%lu q1 %lu/%u q2 %lu/%u idle %lu latmax %lu "
                 "parsemax %lu | L %luB/s pkt %lu inv %lu nr %lu sync %lu "
                 "slip %lu err %lu cmd %lu/%lu | IMU %lu err %lu | BT f %lu dr %lu | "
                 "heap %lu min %lu | stk %lu/%lu | hal %lu k %lu\r\n",
                 (unsigned long)(ls.blocks - prev_blocks),
                 (unsigned long)ls.blocks,
                 (unsigned long)rs.q1_peak,
                 (unsigned)APP_Q_BLOCKS_LEN,
                 (unsigned long)rs.q2_peak,
                 (unsigned)APP_Q_RESULTS_LEN,
                 (unsigned long)rs.parser_idle_wakeups,
                 (unsigned long)rs.max_handover_us,
                 (unsigned long)rs.max_parse_us,
                 (unsigned long)bps,
                 (unsigned long)ls.packets,
                 (unsigned long)ls.invalid,
                 (unsigned long)ls.no_return,
                 (unsigned long)ls.syncs,
                 (unsigned long)ls.slips,
                 (unsigned long)ls.uart_errors,
                 (unsigned long)ls.commands,
                 (unsigned long)ls.stream_losses,
                 (unsigned long)is.samples,
                 (unsigned long)is.errors,
                 (unsigned long)ts.frames,
                 (unsigned long)ts.drops,
                 (unsigned long)rs.heap_free,
                 (unsigned long)rs.heap_min_free,
                 (unsigned long)rs.parser_stack_free,
                 (unsigned long)rs.output_stack_free,
                 (unsigned long)span,
                 (unsigned long)span_k);

      prev_blocks = ls.blocks;
      t_stats = osKernelGetTickCount();
    }
  }
}

/* ========================================================================== */
/* init / start                                                               */
/* ========================================================================== */

void app_rtos_init(void)
{
  memset(&rtos_stats, 0, sizeof(rtos_stats));

  if (osKernelInitialize() != osOK)
  {
    Error_Handler();
  }

  /* Queue 1: small block descriptors, ISR -> parser. */
  q_blocks = osMessageQueueNew(APP_Q_BLOCKS_LEN, sizeof(lidar_block_desc_t), &q_blocks_attr);
  /* Queue 2: processed results, parser -> output. */
  q_results = osMessageQueueNew(APP_Q_RESULTS_LEN, sizeof(lidar_result_t), &q_results_attr);

  if ((q_blocks == NULL) || (q_results == NULL))
  {
    Error_Handler();
  }

  t_parser = osThreadNew(parser_task, NULL, &parser_attr);
  t_output = osThreadNew(output_task, NULL, &output_attr);

  if ((t_parser == NULL) || (t_output == NULL))
  {
    Error_Handler();
  }
}

void app_rtos_start(void)
{
  if (osKernelStart() != osOK)
  {
    /* The scheduler must never return. */
    Error_Handler();
  }
}

void app_rtos_get_stats(app_rtos_stats_t *out)
{
  memset(out, 0, sizeof(*out));
  *out = rtos_stats;

  out->heap_free = (uint32_t)xPortGetFreeHeapSize();
  out->heap_min_free = (uint32_t)xPortGetMinimumEverFreeHeapSize();

  if (t_parser != NULL)
  {
    out->parser_stack_free =
        (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)t_parser) * sizeof(StackType_t);
  }
  if (t_output != NULL)
  {
    out->output_stack_free =
        (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)t_output) * sizeof(StackType_t);
  }
}

/* ========================================================================== */
/* FreeRTOS hooks                                                             */
/* ========================================================================== */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;

  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}

void vApplicationMallocFailedHook(void)
{
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}
