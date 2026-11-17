/* -----------------------------------------------------------------------------
 * app_rtos.h - the 8-thread architecture for Block 4.5 Part 2.
 *
 * Thread table (CODING_STANDARDS 1.2, which is the authoritative copy of these
 * numbers; this header restates them so the code and the report cannot drift):
 *
 *   #  thread          schedule            period/trigger   prio  stack  WCET
 *   1  PID + Motor     osDelayUntil         10 ms            40   1024   <700us
 *   2  IMU             semaphore (TIM6)     10 ms            37   1024   <300us
 *   3  Pose            osDelayUntil         10 ms            36    768   <400us
 *   4  LiDAR           Q_Lidar (100ms guard) ~22 ms           34   1536   <900us
 *   5  PathGen         Q_Cmd                on command       30   1024   <500us
 *   6  BT RX           UART5 semaphore      on bytes         28    768   <200us
 *   7  BT TX           osDelayUntil         50 ms            22   1024   <2 ms
 *   8  OLED            osDelayUntil         50 ms            20   2048   <3 ms
 *
 * Queues, exactly three (CODING_STANDARDS 2.2 sets the overflow policy):
 *   Q_Lidar  8 x 12 B  ISR -> LiDAR task         full: drop,      q_lidar_drop
 *   Q_Cmd    4 x  8 B  BT RX -> PathGen          full: overwrite, q_cmd_overwrite
 *   Q_Path  16 x  8 B  PathGen -> PID+Motor      full: block,     q_path_timeout
 * No Q_Enc (hardware QEP counters) and no Q_Set (PID and motor are one task).
 *
 * PRIORITY NUMBERS: the table above uses the project's own numbering.  Note that
 * CMSIS-RTOS2's osPriorityAboveNormal is 32, not 37 - the table's numbers are
 * used verbatim (they are what the report quotes) and the ORDER is enforced by
 * the _Static_asserts below, which is the property that actually matters.
 * -------------------------------------------------------------------------- */

#ifndef APP_RTOS_H
#define APP_RTOS_H

#include <stdint.h>

#include "lidar.h"
#include "bt_protocol.h"
#include "path.h"

/* ---- queue depths -------------------------------------------------------- */
#define APP_Q_LIDAR_LEN  8U
#define APP_Q_CMD_LEN    Q_CMD_CAPACITY   /* 4, from bt_protocol.h */
#define APP_Q_PATH_LEN   Q_PATH_CAPACITY  /* 16, from path.h       */

#define APP_Q_PATH_PUT_TMO_MS  50U   /* producer blocks at most this long */
#define APP_Q_LIDAR_GUARD_MS  100U   /* Q_Lidar get timeout, keeps watchdog alive */
#define APP_BT_RX_GUARD_MS    100U   /* BT RX wake-up even if no byte arrives */
#define APP_IMU_WAIT_MS       100U   /* IMU semaphore wait (detects dead TIM6) */

/* Counting semaphore for the TIM6 -> IMU hand-off.  2 allows one tick to be
 * consumed while the next is pending (jitter), but not a backlog: a third tick
 * without a sample is a real overrun and is counted as imu_skip
 * (CODING_STANDARDS 2.5). */
#define APP_IMU_SEM_MAX       2U
#define APP_BT_RX_SEM_MAX     4U

/* ---- task cadence -------------------------------------------------------- */
#define APP_PERIOD_FAST_MS    10U
#define APP_PERIOD_SLOW_MS    50U
/* Binary telemetry keeps the 10 Hz rate the PC viewer expects: one frame every
 * second 50 ms slot (see the slot policy in bt_tx_task). */
#define APP_STATUS_PERIOD_MS  1000U

/* ---- priorities ---------------------------------------------------------- */
#define APP_PRIO_PID_MOTOR    40U
#define APP_PRIO_IMU          37U
#define APP_PRIO_POSE         36U
#define APP_PRIO_LIDAR        34U
#define APP_PRIO_PATH         30U
#define APP_PRIO_BT_RX        28U
#define APP_PRIO_BT_TX        22U
#define APP_PRIO_OLED         20U

_Static_assert(APP_PRIO_PID_MOTOR > APP_PRIO_IMU,   "execuator must outrank sensing");
_Static_assert(APP_PRIO_IMU       > APP_PRIO_POSE,  "sensor must outrank fusion");
_Static_assert(APP_PRIO_POSE      > APP_PRIO_LIDAR, "control loop must outrank LiDAR");
_Static_assert(APP_PRIO_LIDAR     > APP_PRIO_PATH,  "LiDAR deadline is 40 ms");
_Static_assert(APP_PRIO_PATH      > APP_PRIO_BT_RX, "planning before radio");
_Static_assert(APP_PRIO_BT_RX     > APP_PRIO_BT_TX, "receive before transmit");
_Static_assert(APP_PRIO_BT_TX     > APP_PRIO_OLED,  "display is always last");
_Static_assert(APP_PRIO_OLED      > 0U,             "must be above idle");

/* ---- stack sizes, bytes (CMSIS-RTOS2 osThreadAttr_t semantics) ----------- */
#define APP_STK_PID_MOTOR     1024U
/* MEASURED ON HARDWARE 2026-11-17: 512 was not enough.  With the task's stack
 * base at 0x20003AC0, the saved stack pointer was 0x20003A9C - 36 bytes BELOW
 * the base - and FreeRTOS' overflow hook (configCHECK_FOR_STACK_OVERFLOW=2)
 * fired on the first context switch after the first I2C sample.  Cause: an -O0
 * build with the whole HAL_I2C_Mem_Read -> I2C_WaitOnFlagUntilTimeout chain on
 * the stack.  1024 leaves ~2x margin over the ~550 bytes actually needed; the
 * high-water mark is printed in the 1 Hz STK line, so raise it further if that
 * number drops below 30 % of 1024.  (CODING_STANDARDS 1.4: grow, never shave.) */
#define APP_STK_IMU           1024U
#define APP_STK_POSE           768U
#define APP_STK_LIDAR         1536U
#define APP_STK_PATH          1024U
#define APP_STK_BT_RX          768U
#define APP_STK_BT_TX         1024U
/* MEASURED ON HARDWARE 2026-11-17: with 1024 the OLED task had only 40 bytes of
 * headroom left (STK line: "... 636 40"), i.e. it was one function frame away
 * from the same overflow-hook freeze that the IMU task hit.  Blame the 1 Hz
 * status block: eight fixed-point dbg_printf() calls each carry a 128-byte
 * format buffer plus the vsnprintf frame, on top of eight oled_row() calls.
 * 2048 leaves ~50 % free.  This task is the last one in the priority order, so
 * spending another 1 KiB of heap here costs nothing else. */
#define APP_STK_OLED          2048U

/* ---- task identifiers, used for the STK line (order is fixed & documented) */
#define APP_TASK_COUNT        8U
enum
{
  APP_TASK_PID_MOTOR = 0,
  APP_TASK_IMU,
  APP_TASK_POSE,
  APP_TASK_LIDAR,
  APP_TASK_PATH,
  APP_TASK_BT_RX,
  APP_TASK_BT_TX,
  APP_TASK_OLED
};

/* ---- event bits shared between PathGen and PID+Motor --------------------- */
/* Event flags, not a queue: a STOP is a one-shot notification with no payload,
 * and Q_Path may legitimately contain waypoints that must all be discarded. */
#define APP_NAV_EVT_STOP      0x0001U
#define APP_NAV_EVT_NEW_PATH  0x0002U

/* ---- per-task timing (measured with DWT, never estimated) ---------------- */
typedef struct
{
  uint32_t activations;
  uint32_t period_us_min;
  uint32_t period_us_max;
  uint32_t jitter_us_max;   /* |actual - nominal|, worst case            */
  uint32_t wcet_us_max;     /* worst observed execution time             */
  uint32_t overruns;        /* wake-ups more than one period late        */
} app_timing_t;

/* Indices into app_rtos_stats_t.timing[] */
enum
{
  APP_TIMING_PID_MOTOR = 0,
  APP_TIMING_POSE,
  APP_TIMING_BT_TX,
  APP_TIMING_OLED,
  APP_TIMING_COUNT
};

typedef struct
{
  /* Q_Lidar (queue 1) */
  uint32_t blocks_posted;      /* descriptors pushed by the ISRs         */
  uint32_t blocks_consumed;    /* descriptors taken by the LiDAR task    */
  uint32_t blocks_dropped;     /* Q_Lidar was full -> dropped, counted   */
  uint32_t q_lidar_peak;
  uint32_t parser_idle_wakeups;/* guard-timeout wake-ups (sensor silent) */
  uint32_t max_parse_us;
  uint32_t max_handover_us;

  /* Q_Cmd (queue 2) */
  uint32_t q_cmd_peak;
  uint32_t q_cmd_overwrite;    /* oldest command discarded to make room  */

  /* Q_Path (queue 3) */
  uint32_t q_path_peak;
  uint32_t q_path_timeout;     /* producer gave up after APP_Q_PATH_PUT_TMO_MS */

  /* IMU hand-off */
  uint32_t imu_ticks_unused;   /* semaphore was full: task fell behind   */
  uint32_t imu_wait_timeouts;  /* no tick for APP_IMU_WAIT_MS (TIM6 dead)*/

  /* periodic task timing */
  app_timing_t timing[APP_TIMING_COUNT];

  /* resources */
  uint32_t heap_free;
  uint32_t heap_min_free;
  uint32_t heap_total;
  uint32_t stack_free[APP_TASK_COUNT];
} app_rtos_stats_t;

/* Creates queues, semaphore, event flags and all 8 tasks.  Called from
 * app_init(), i.e. before the scheduler starts; nothing runs yet.
 * Also installs the Q_Cmd sink for bt_rx. */
void app_rtos_init(void);

/* Starts the scheduler.  Never returns. */
void app_rtos_start(void);

/* Interrupt-context entry point used by lidar.c.
 * Returns 0 on success, -1 if Q_Lidar was full.
 * MUST only be called from an ISR whose NVIC priority is numerically >= 5. */
int app_rtos_post_block(const lidar_block_desc_t *desc);

/* Interrupt context: one TIM6 tick -> one IMU semaphore credit.
 * Returns 0 if released, -1 if the semaphore was already full (task behind). */
int app_rtos_imu_tick(void);

/* Interrupt context: UART5 received something, wake the BT RX task. */
void app_rtos_bt_rx_event(void);

void app_rtos_get_stats(app_rtos_stats_t *out);

#endif /* APP_RTOS_H */
