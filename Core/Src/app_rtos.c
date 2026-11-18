/* -----------------------------------------------------------------------------
 * app_rtos.c - the 8-thread architecture (see app_rtos.h for the table).
 *
 * DATA FLOW
 * ---------
 *   USART3/DMA1_S1 ISR --Q_Lidar--> LiDAR task --> lidar.c 360-bin map
 *   UART5/DMA1_S0 ISR  --sem-----> BT RX task  --sink--> Q_Cmd
 *   Q_Cmd --> PathGen task --Q_Path--> PID+Motor task --> motor.c --> TIM4/TIM2
 *   TIM6 ISR --sem--> IMU task --> i2c_bus --> MPU-6500
 *   Pose task (10 ms) samples TIM1/TIM8 + the IMU heading -> pose.c
 *   BT TX task (50 ms) pushes the binary telemetry frame the PC viewer needs
 *   OLED task (50 ms) draws one page per slot and prints the 1 Hz status block
 *
 * WHY THE 1 Hz BLOCK IS PRINTED FROM THE OLED TASK: it is the lowest-priority
 * task in the system, so formatting and queueing those characters can never
 * delay a control deadline (CODING_STANDARDS 5.4).  The output itself is
 * non-blocking (log.c: ring + DMA), so even there it does not stall.
 *
 * NO QUEUE FOR ENCODER DATA AND NONE FOR SETPOINTS: the encoders are hardware
 * QEP counters read by the Pose task, and the PID/motor pair shares one task so
 * the setpoint is a local variable.  That is what "Q_Enc / Q_Set are not
 * introduced" means in practice (CODING_STANDARDS 1.2).
 * -------------------------------------------------------------------------- */

#include "app_rtos.h"
#include "app.h"
#include "main.h"
#include "board_pins.h"

#include "lidar.h"
#include "imu.h"
#include "telemetry.h"
#include "oled.h"
#include "bt_rx.h"
#include "bt_protocol.h"
#include "i2c_bus.h"
#include "pose.h"
#include "path.h"
#include "motor.h"
#include "encoder.h"
#include "log.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart2;   /* debug console, polled for 's' / 'g' */

/* ========================================================================== */
/* RTOS objects                                                               */
/* ========================================================================== */
static osMessageQueueId_t q_lidar;   /* ISR      -> LiDAR task              */
static osMessageQueueId_t q_cmd;     /* BT RX    -> PathGen                 */
static osMessageQueueId_t q_path;    /* PathGen  -> PID + Motor             */
static osSemaphoreId_t    sem_imu;   /* TIM6 ISR -> IMU task                */
static osSemaphoreId_t    sem_bt_rx; /* UART5 ISR -> BT RX task             */
static osEventFlagsId_t   evt_nav;   /* STOP / NEW_PATH notifications       */

static osThreadId_t t_pid;
static osThreadId_t t_imu;
static osThreadId_t t_pose;
static osThreadId_t t_lidar;
static osThreadId_t t_path;
static osThreadId_t t_bt_rx;
static osThreadId_t t_bt_tx;
static osThreadId_t t_oled;

/* Defined with the demo itself, near the bottom of this file; the console
 * handler lives above it and only needs the prototype. */
static void demo_prio_start(void);

static app_rtos_stats_t rtos_stats;

/* Timing state is kept separate from the published copy so a 1 Hz snapshot can
 * never tear a value that a task is updating. */
typedef struct
{
  uint32_t last_cyc;
  uint32_t start_cyc;
} timing_int_t;

static timing_int_t   timing_int[APP_TIMING_COUNT];
static app_timing_t   timing_pub[APP_TIMING_COUNT];

/* Snapshot of the rolling 360-bin scan, assembled by the BT TX task. */
static uint16_t scan_frame[LIDAR_BINS];
static uint8_t  qual_frame[LIDAR_BINS];

/* ========================================================================== */
/* task attributes                                                            */
/* ========================================================================== */
static const osThreadAttr_t pid_motor_attr = {
  .name = "pidMotor", .stack_size = APP_STK_PID_MOTOR, .priority = (osPriority_t)APP_PRIO_PID_MOTOR,
};
static const osThreadAttr_t imu_attr = {
  .name = "imuAcq", .stack_size = APP_STK_IMU, .priority = (osPriority_t)APP_PRIO_IMU,
};
static const osThreadAttr_t pose_attr = {
  .name = "pose", .stack_size = APP_STK_POSE, .priority = (osPriority_t)APP_PRIO_POSE,
};
static const osThreadAttr_t lidar_attr = {
  .name = "lidarProc", .stack_size = APP_STK_LIDAR, .priority = (osPriority_t)APP_PRIO_LIDAR,
};
static const osThreadAttr_t path_attr = {
  .name = "pathGen", .stack_size = APP_STK_PATH, .priority = (osPriority_t)APP_PRIO_PATH,
};
static const osThreadAttr_t bt_rx_attr = {
  .name = "btRx", .stack_size = APP_STK_BT_RX, .priority = (osPriority_t)APP_PRIO_BT_RX,
};
static const osThreadAttr_t bt_tx_attr = {
  .name = "btTx", .stack_size = APP_STK_BT_TX, .priority = (osPriority_t)APP_PRIO_BT_TX,
};
static const osThreadAttr_t oled_attr = {
  .name = "oled", .stack_size = APP_STK_OLED, .priority = (osPriority_t)APP_PRIO_OLED,
};

static const osMessageQueueAttr_t q_lidar_attr = { .name = "qLidar" };
static const osMessageQueueAttr_t q_cmd_attr   = { .name = "qCmd"   };
static const osMessageQueueAttr_t q_path_attr  = { .name = "qPath"  };
static const osSemaphoreAttr_t    sem_imu_attr = { .name = "semImu"    };
static const osSemaphoreAttr_t    sem_bt_attr  = { .name = "semBtRx"   };
static const osEventFlagsAttr_t   evt_nav_attr = { .name = "evtNav"    };

/* ========================================================================== */
/* timing helpers (DWT, 纰宻 resolution)                                        */
/* ========================================================================== */
static uint32_t cyc_to_us(uint32_t cycles)
{
  return (uint32_t)(((uint64_t)cycles * 1000000ULL) / (uint64_t)SystemCoreClock);
}

/* Call at the top of a periodic task, before doing the work. */
static void timing_begin(uint32_t slot, uint32_t nominal_us)
{
  app_timing_t *t = &timing_pub[slot];
  timing_int_t *s = &timing_int[slot];
  uint32_t now = DWT->CYCCNT;

  if (t->activations != 0U)
  {
    uint32_t period_us = cyc_to_us(now - s->last_cyc);
    uint32_t jitter_us;

    if ((t->period_us_min == 0U) || (period_us < t->period_us_min))
    {
      t->period_us_min = period_us;
    }
    if (period_us > t->period_us_max)
    {
      t->period_us_max = period_us;
    }

    jitter_us = (period_us > nominal_us) ? (period_us - nominal_us)
                                         : (nominal_us - period_us);
    if (jitter_us > t->jitter_us_max)
    {
      t->jitter_us_max = jitter_us;
    }
  }

  s->last_cyc = now;
  s->start_cyc = now;
  t->activations++;
}

/* Call after the work, before osDelayUntil(). */
static void timing_end(uint32_t slot)
{
  uint32_t wcet_us = cyc_to_us(DWT->CYCCNT - timing_int[slot].start_cyc);

  if (wcet_us > timing_pub[slot].wcet_us_max)
  {
    timing_pub[slot].wcet_us_max = wcet_us;
  }
}

/* CODING_STANDARDS 1.3: a wake-up more than one period late is an overrun and
 * must be visible, never silently absorbed by osDelayUntil(). */
static void timing_check_late(uint32_t slot, uint32_t target_tick, uint32_t period_ms)
{
  if ((uint32_t)(osKernelGetTickCount() - target_tick) >= period_ms)
  {
    timing_pub[slot].overruns++;
  }
}

/* ========================================================================== */
/* small display helper                                                       */
/* ========================================================================== */
/* Fixed 6x8 cells, 21 columns.  A longer string is clipped by oled.c, which is
 * the behaviour we want for a status screen (no wrapping onto the next row). */
static void oled_row(uint8_t row, const char *fmt, ...)
{
  char buf[24];
  va_list ap;

  va_start(ap, fmt);
  (void)vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  (void)oled_show_text(row, 0U, buf);
}

/* ========================================================================== */
/* Q_Cmd sink (installed into bt_rx by app_rtos_init)                         */
/* ========================================================================== */
/* Overflow policy for Q_Cmd: overwrite the OLDEST command, because when a user
 * taps "go" twice the newest target is the one they mean
 * (CODING_STANDARDS 2.2).  The overwrite is counted, never silent. */
static int cmd_sink(const cmd_msg_t *msg)
{
  uint32_t depth;

  if (msg == NULL)
  {
    return BT_ERR_SYNTAX;
  }

  if (osMessageQueuePut(q_cmd, msg, 0U, 0U) != osOK)
  {
    cmd_msg_t discarded;

    if (osMessageQueueGet(q_cmd, &discarded, 0U, 0U) != osOK)
    {
      return BT_ERR_BUSY;
    }
    rtos_stats.q_cmd_overwrite++;

    if (osMessageQueuePut(q_cmd, msg, 0U, 0U) != osOK)
    {
      return BT_ERR_BUSY;
    }
  }

  depth = osMessageQueueGetCount(q_cmd);
  if (depth > rtos_stats.q_cmd_peak)
  {
    rtos_stats.q_cmd_peak = depth;
  }
  return BT_OK;
}

/* ========================================================================== */
/* ISR-side entry points                                                      */
/* ========================================================================== */
int app_rtos_post_block(const lidar_block_desc_t *desc)
{
  if (osMessageQueuePut(q_lidar, desc, 0U, 0U) != osOK)
  {
    rtos_stats.blocks_dropped++;
    return -1;
  }
  rtos_stats.blocks_posted++;
  return 0;
}

int app_rtos_imu_tick(void)
{
  /* osSemaphoreRelease() from an ISR is legal: ST's cmsis_os2.c detects the
   * context itself (CODING_STANDARDS 2.3).  A full semaphore means the IMU task
   * has not consumed the previous tick - that is an overrun, reported upward. */
  if (osSemaphoreRelease(sem_imu) != osOK)
  {
    rtos_stats.imu_ticks_unused++;
    return -1;
  }
  return 0;
}

void app_rtos_bt_rx_event(void)
{
  (void)osSemaphoreRelease(sem_bt_rx);
}

/* ---- bench (open-loop) mode, for hardware bring-up only ------------------- */
/* 'f' / 'b' apply a fixed duty to both wheels for one short burst; 'x' stops.
 * Only ever used with the wheels OFF the ground.
 *
 * Why a timeout: the console is a single keypress.  If the byte is dropped, the
 * terminal window is closed or the operator's hand slips, a duty left applied
 * would drive the car away.  BENCH_HOLD_MS bounds that to 1.5 s.
 *
 * Why the PID task applies it: the motor has exactly one owner
 * (CODING_STANDARDS 0.3).  While a burst is active the task forces
 * have_wp = 0 and stopped = 1, which makes both branches of the tracking code
 * no-ops, so the open-loop write is the only motor write in that cycle.  On the
 * cycle the burst ends, bench_latched makes the task call motor_stop() once -
 * without it the last CCR value would keep the wheels turning after 'x'. */
/* Bench duty.  Was 20 %, which is BELOW THIS PACK'S DEAD BAND: measured
 * 2026-11-18 by driving the H-bridge inputs straight over SWD, the wheels do not
 * start at all at 10-20 % and only just move at 30 %; the same 20 % duty did turn
 * them at 83 rpm on 2026-11-17 (HANDOVER 12.4).  A jog that produces no motion
 * cannot tell "weak pack" from "dead channel", which is what it exists for, so
 * the bench burst now runs at 40 % - clearly above the threshold and still only
 * meant to be used with the wheels off the ground. */
#define BENCH_DUTY_PCT     40      /* open-loop duty, percent              */
#define BENCH_HOLD_MS    1500U     /* burst length, milliseconds           */

static volatile int8_t   bench_duty_l;      /* 0 = off, else -100..+100     */
static volatile int8_t   bench_duty_r;      /* per wheel, so a single channel
                                             * can be exercised on its own   */
static volatile uint32_t bench_deadline_ms; /* HAL tick when the burst ends */
static uint8_t           bench_latched;     /* duty is currently applied    */

/* Set by the 'c' console key, consumed by the Pose task - the only owner of the
 * encoder counters (see the single-owner comment at the 'c' handler). */
static volatile uint8_t  g_pose_reset_req;

/* TEMPORARY bring-up counters (read over SWD; not printed anywhere).  They
 * exist to answer "which branch of the tracking loop actually ran" without
 * guessing from the 1 Hz line.  Remove once the path tracking is validated. */
static volatile uint32_t dbg_cycles;     /* PID task iterations               */
static volatile uint32_t dbg_stop_ev;    /* STOP event consumed               */
static volatile uint32_t dbg_newpath_ev; /* NEW_PATH event consumed           */
static volatile uint32_t dbg_pops;       /* successful q_path gets            */
static volatile uint32_t dbg_arrived;    /* waypoint judged already reached   */
static volatile uint32_t dbg_track;      /* tracking law executed             */

/* ========================================================================== */
/* Task 1: PID + Motor (10 ms)                                                */
/* ========================================================================== */
static void pid_motor_task(void *argument)
{
  uint32_t tick = osKernelGetTickCount();
  path_point_t wp;
  uint8_t  have_wp = 0U;
  uint8_t  stopped = 1U;
  (void)argument;

  dbg_task_started("pidMotor", APP_PERIOD_FAST_MS, APP_PRIO_PID_MOTOR, APP_STK_PID_MOTOR);

  for (;;)
  {
    uint32_t nav_flags;

    tick += APP_PERIOD_FAST_MS;
    timing_begin(APP_TIMING_PID_MOTOR, APP_PERIOD_FAST_MS * 1000U);

    /* Consume the one-shot navigation notifications.
     *
     * THE TRAP: osEventFlagsWait() returns the flag word on success but a
     * NEGATIVE osStatus_t on "nothing pending" (osErrorResource = -3 =
     * 0xFFFFFFFD, or osErrorTimeout = -2 = 0xFFFFFFFE when a timeout was given).
     * Those error codes live in the same uint32_t as the flags, and
     * 0xFFFFFFFD & 0x0001 == 1, so testing the result with & alone reports a
     * STOP on every single call - which is exactly what happened on hardware:
     * stop_ev == cycles, the motor was stopped every cycle and the car never
     * moved.  Any CMSIS-RTOS2 error code has bit31 set, while event flags only
     * use the low 24 bits, so mask the error case away first. */
    nav_flags = osEventFlagsWait(evt_nav,
                                 (uint32_t)APP_NAV_EVT_STOP | (uint32_t)APP_NAV_EVT_NEW_PATH,
                                 osFlagsWaitAny, 0U);

    if ((nav_flags & 0x80000000U) != 0U)
    {
      nav_flags = 0U;   /* error code, not a flag word: nothing happened */
    }

    if ((nav_flags & (uint32_t)APP_NAV_EVT_STOP) != 0U)
    {
      have_wp = 0U;
      stopped = 1U;
      motor_stop();
      dbg_stop_ev++;
    }
    else if ((nav_flags & (uint32_t)APP_NAV_EVT_NEW_PATH) != 0U)
    {
      /* A new path replaced the old one.  Drop the waypoint this task is
       * currently following, otherwise the car would finish tracking the OLD
       * path before noticing the new command.  `stopped` is left alone and the
       * queue get below takes the new first waypoint in this same cycle, so
       * there is no unnecessary brake between retargets. */
      have_wp = 0U;
      dbg_newpath_ev++;
    }

    /* ---- bench burst: open-loop drive, wheels off the ground -------------- */
    if (((bench_duty_l != 0) || (bench_duty_r != 0)) &&
        ((int32_t)(bench_deadline_ms - HAL_GetTick()) > 0))
    {
      have_wp = 0U;   /* cancel tracking; the queue keeps its waypoints */
      stopped = 1U;
      motor_set_duty_percent(bench_duty_l, bench_duty_r);
      bench_latched = 1U;
    }
    else if (bench_latched != 0U)
    {
      /* Burst finished (timeout or 'x').  Stop here, once, so the PWM really goes
       * to zero instead of holding the last CCR value. */
      bench_duty_l = 0;
      bench_duty_r = 0;
      bench_latched = 0U;
      have_wp = 0U;
      stopped = 1U;
      motor_stop();
      dbg_printf("[bench] burst done, motor stopped\r\n");
    }

    {
      pose_t pose;

      pose_get(&pose);
      dbg_cycles++;

      /* 1. Is the waypoint we are following already reached? */
      if ((have_wp != 0U) && (path_arrived(&pose, &wp) != 0U))
      {
        path_note_reached();
        have_wp = 0U;
        dbg_arrived++;

        if (path_is_final(&wp) != 0U)
        {
          path_note_finished();
        }
      }

      /* 2. Take the next waypoint.  The loop matters: a 100 mm waypoint can
       * already be inside PATH_ARRIVE_MM right after the previous one, and
       * braking for one cycle at every step would make the car judder along the
       * whole path.  Consecutive waypoints are consumed in the SAME cycle, so
       * the motor outputs are never zeroed between two pieces of one path. */
      if (have_wp == 0U)
      {
        while (osMessageQueueGet(q_path, &wp, NULL, 0U) == osOK)
        {
          uint32_t depth = osMessageQueueGetCount(q_path);

          dbg_pops++;
          if (depth > rtos_stats.q_path_peak)
          {
            rtos_stats.q_path_peak = depth;
          }

          if (path_arrived(&pose, &wp) == 0U)
          {
            have_wp = 1U;
            break;
          }

          /* Already inside the arrival radius: skip it. */
          path_note_reached();
          if (path_is_final(&wp) != 0U)
          {
            path_note_finished();
            break;
          }
        }
      }

      if (have_wp != 0U)
      {
        int16_t rpm_l = 0;
        int16_t rpm_r = 0;

        path_track_step(&pose, &wp, &rpm_l, &rpm_r);
        motor_set_target_rpm(rpm_l, rpm_r);
        stopped = 0U;
        dbg_track++;

        /* One PID step, one CCR write - never split across a blocking call
         * (CODING_STANDARDS 1.2.2).  The feedback is the value the Pose task
         * published on the previous 10 ms edge, i.e. at most one period old.
         *
         * motor_update() is deliberately NOT called while idle: with a zero
         * target the PID would drive the PWM against the coasting wheels
         * (feedback != 0), i.e. actively brake instead of leaving the outputs
         * at 0.  "PWM to 0 within one period" (CODING_STANDARDS 3.3) requires
         * the outputs to be written by motor_stop() and left alone. */
        motor_update(encoder_left_rpm(), encoder_right_rpm());
      }
      else if (stopped == 0U)
      {
        /* No waypoint left: stop, once. */
        motor_stop();
        stopped = 1U;
      }
    }

    timing_end(APP_TIMING_PID_MOTOR);
    osDelayUntil(tick);
    timing_check_late(APP_TIMING_PID_MOTOR, tick, APP_PERIOD_FAST_MS);
  }
}

/* ========================================================================== */
/* Task 2: IMU acquisition (event: TIM6 semaphore)                            */
/* ========================================================================== */
static void imu_task(void *argument)
{
  (void)argument;

  dbg_task_started("imuAcq", 0U, APP_PRIO_IMU, APP_STK_IMU);

  for (;;)
  {
    /* The ONLY reason this task exists in this shape: the sample must happen in
     * task context so the bus mutex can be held across the transfer. */
    if (osSemaphoreAcquire(sem_imu, APP_IMU_WAIT_MS) == osOK)
    {
      (void)imu_sample_once();
    }
    else
    {
      /* No tick for 100 ms: TIM6 has stopped.  Counted so it cannot hide. */
      rtos_stats.imu_wait_timeouts++;
    }
  }
}

/* ========================================================================== */
/* Task 3: Pose estimation (10 ms)                                            */
/* ========================================================================== */
static void pose_task(void *argument)
{
  uint32_t tick = osKernelGetTickCount();
  (void)argument;

  dbg_task_started("pose", APP_PERIOD_FAST_MS, APP_PRIO_POSE, APP_STK_POSE);

  for (;;)
  {
    tick += APP_PERIOD_FAST_MS;
    timing_begin(APP_TIMING_POSE, APP_PERIOD_FAST_MS * 1000U);

    /* Console 'c' asked for a zero.  Done HERE, not in the console handler,
     * because this task is the only owner of the encoder counters: zeroing them
     * from another task could tear a 32-bit total that is being accumulated. */
    if (g_pose_reset_req != 0U)
    {
      g_pose_reset_req = 0U;
      pose_reset();
      dbg_printf("[enc] zeroed at this pose\r\n");
    }

    pose_update(APP_PERIOD_FAST_MS);

    timing_end(APP_TIMING_POSE);
    osDelayUntil(tick);
    timing_check_late(APP_TIMING_POSE, tick, APP_PERIOD_FAST_MS);
  }
}

/* ========================================================================== */
/* Task 4: LiDAR processing (event: Q_Lidar)                                  */
/* ========================================================================== */
static void lidar_task(void *argument)
{
  lidar_block_desc_t desc;
  lidar_result_t     result;
  (void)argument;

  dbg_task_started("lidarProc", 0U, APP_PRIO_LIDAR, APP_STK_LIDAR);

  for (;;)
  {
    /* The 100 ms guard is NOT a poll: it exists so the stream watchdog inside
     * lidar_parser_service() still runs when the sensor goes silent (e.g. the
     * SCAN command was lost at power-up).  In normal operation it never fires.
     */
    osStatus_t st = osMessageQueueGet(q_lidar, &desc, NULL, APP_Q_LIDAR_GUARD_MS);

    if (st == osOK)
    {
      uint32_t depth = osMessageQueueGetCount(q_lidar);

      rtos_stats.blocks_consumed++;
      if (depth > rtos_stats.q_lidar_peak)
      {
        rtos_stats.q_lidar_peak = depth;
      }

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
    }
    else
    {
      rtos_stats.parser_idle_wakeups++;
    }

    lidar_parser_service();
  }
}

/* ========================================================================== */
/* Task 5: Path generation (event: Q_Cmd)                                     */
/* ========================================================================== */
static void path_task(void *argument)
{
  static path_point_t pts[Q_PATH_CAPACITY];
  cmd_msg_t cmd;
  (void)argument;

  dbg_task_started("pathGen", 0U, APP_PRIO_PATH, APP_STK_PATH);

  for (;;)
  {
    if (osMessageQueueGet(q_cmd, &cmd, NULL, osWaitForever) != osOK)
    {
      continue;
    }

    if (cmd.type == (uint8_t)CMD_TYPE_GO)
    {
      pose_t pose;
      int n;
      int i;
      uint8_t put_failed = 0U;

      pose_get(&pose);

      /* Until odometry is calibrated, pose is (0,0) at boot, so this is a
       * straight line from "where we think we are" to the commanded point.  No
       * obstacle avoidance - that belongs to the final development block
       * (CODING_STANDARDS 9.1). */
      n = path_generate_line((int16_t)lroundf(pose.x_mm),
                             (int16_t)lroundf(pose.y_mm),
                             cmd.x_mm, cmd.y_mm, pts, Q_PATH_CAPACITY);
      if (n < 0)
      {
        /* Does not fit in Q_Path: refuse rather than emit a partial path that
         * would silently stop the car in the middle of the field. */
        rtos_stats.q_path_timeout++;
        dbg_printf("[nav] G %d %d: path does not fit in Q_Path (%d)\r\n",
                   (int)cmd.x_mm, (int)cmd.y_mm, n);
        continue;
      }

      /* A new target replaces the old one entirely.  The queue is filled FIRST
       * and the notification is raised afterwards, so the PID task can never
       * wake up to "new path" and find Q_Path still empty (it is the higher
       * priority of the two). */
      (void)osMessageQueueReset(q_path);
      path_note_generated((uint32_t)n);

      for (i = 0; i < n; i++)
      {
        /* Producer blocks with a timeout: waypoints must not be dropped, but a
         * stuck consumer must not hang PathGen either (CODING_STANDARDS 2.2). */
        if (osMessageQueuePut(q_path, &pts[i], 0U, APP_Q_PATH_PUT_TMO_MS) != osOK)
        {
          path_note_put_timeout();
          rtos_stats.q_path_timeout++;
          put_failed = 1U;
          break;
        }
      }

      if (put_failed == 0U)
      {
        (void)osEventFlagsSet(evt_nav, APP_NAV_EVT_NEW_PATH);
      }

      if (put_failed != 0U)
      {
        dbg_printf("[nav] Q_Path full after %d/%d points, path abandoned\r\n", i, n);
        (void)osMessageQueueReset(q_path);
        (void)osEventFlagsSet(evt_nav, APP_NAV_EVT_STOP);
      }
      else
      {
        dbg_printf("[nav] G %d %d accepted, %d waypoints\r\n",
                   (int)cmd.x_mm, (int)cmd.y_mm, n);
      }
    }
    else if (cmd.type == (uint8_t)CMD_TYPE_STOP)
    {
      (void)osMessageQueueReset(q_path);
      (void)osEventFlagsSet(evt_nav, APP_NAV_EVT_STOP);
      path_note_aborted();
      dbg_printf("[nav] STOP: Q_Path cleared, PWM to 0\r\n");
    }
    else if (cmd.type == (uint8_t)CMD_TYPE_ZERO)
    {
      /* Bluetooth counterpart of the console 'c' key, so an untethered run can
       * be re-zeroed between attempts without a power cycle (which would cost
       * a Bluetooth re-pair in front of the examiner).  Same order as 'c':
       * stop first, then ask the Pose task - the only owner of the encoder
       * totals - to zero, so a torn 32-bit total cannot happen. */
      (void)osMessageQueueReset(q_path);
      (void)osEventFlagsSet(evt_nav, APP_NAV_EVT_STOP);
      path_note_aborted();
      g_pose_reset_req = 1U;
      dbg_printf("[nav] ZERO: Q_Path cleared, pose + encoders will zero\r\n");
    }
    else
    {
      /* CMD_TYPE_QUERY.  The "A G ..." / "E ..." reply belongs to the BT TX
       * task; bt_rx.h states that wiring the replies is a later step, and this
       * task must not print without being asked.  Nothing to do here yet. */
    }
  }
}

/* ========================================================================== */
/* Task 6: BT RX (event: UART5 semaphore)                                     */
/* ========================================================================== */
static void bt_rx_task(void *argument)
{
  (void)argument;

  dbg_task_started("btRx", 0U, APP_PRIO_BT_RX, APP_STK_BT_RX);

  for (;;)
  {
    /* The guard timeout also drives the "re-arm request" repair path inside
     * bt_rx_process() when the UART reported an error (CODING_STANDARDS 5.2). */
    (void)osSemaphoreAcquire(sem_bt_rx, APP_BT_RX_GUARD_MS);
    (void)bt_rx_process();
  }
}

/* ========================================================================== */
/* Task 7: BT TX + debug console (50 ms)                                      */
/* ========================================================================== */
static void console_poll(void)
{
  uint8_t ch = 0U;

  if (HAL_UART_Receive(&huart2, &ch, 1U, 0U) != HAL_OK)
  {
    return;
  }

  if ((ch == (uint8_t)'s') || (ch == (uint8_t)'S'))
  {
    lidar_request_stop();
    dbg_printf("[cmd] s -> stopping LiDAR\r\n");
  }
  else if ((ch == (uint8_t)'g') || (ch == (uint8_t)'G'))
  {
    lidar_request_resume();
    dbg_printf("[cmd] g -> restarting LiDAR\r\n");
  }
  else if ((ch == (uint8_t)'f') || (ch == (uint8_t)'F'))
  {
    bench_duty_l = (int8_t)BENCH_DUTY_PCT;
    bench_duty_r = (int8_t)BENCH_DUTY_PCT;
    bench_deadline_ms = HAL_GetTick() + BENCH_HOLD_MS;
    dbg_printf("[bench] both forward %d%% for %lu ms\r\n",
               (int)BENCH_DUTY_PCT, (unsigned long)BENCH_HOLD_MS);
  }
  else if ((ch == (uint8_t)'b') || (ch == (uint8_t)'B'))
  {
    bench_duty_l = (int8_t)(-BENCH_DUTY_PCT);
    bench_duty_r = (int8_t)(-BENCH_DUTY_PCT);
    bench_deadline_ms = HAL_GetTick() + BENCH_HOLD_MS;
    dbg_printf("[bench] both reverse %d%% for %lu ms\r\n",
               (int)BENCH_DUTY_PCT, (unsigned long)BENCH_HOLD_MS);
  }
  else if ((ch == (uint8_t)'l') || (ch == (uint8_t)'L'))
  {
    /* Left channel ONLY.  Which physical wheel moves tells us which timer
     * channel is wired, which is what decides whether the fix for an inverted
     * wheel is BOARD_MOTOR_RIGHT_INVERT or the left channel's pin swap. */
    bench_duty_l = (int8_t)BENCH_DUTY_PCT;
    bench_duty_r = 0;
    bench_deadline_ms = HAL_GetTick() + BENCH_HOLD_MS;
    dbg_printf("[bench] LEFT channel only %d%% for %lu ms\r\n",
               (int)BENCH_DUTY_PCT, (unsigned long)BENCH_HOLD_MS);
  }
  else if ((ch == (uint8_t)'r') || (ch == (uint8_t)'R'))
  {
    bench_duty_l = 0;
    bench_duty_r = (int8_t)BENCH_DUTY_PCT;
    bench_deadline_ms = HAL_GetTick() + BENCH_HOLD_MS;
    dbg_printf("[bench] RIGHT channel only %d%% for %lu ms\r\n",
               (int)BENCH_DUTY_PCT, (unsigned long)BENCH_HOLD_MS);
  }
  else if ((ch == (uint8_t)'x') || (ch == (uint8_t)'X'))
  {
    /* Only clears the request: the PID task performs the actual motor_stop()
     * on its next 10 ms cycle, so the motor keeps a single owner. */
    bench_duty_l = 0;
    bench_duty_r = 0;
    dbg_printf("[bench] stop requested\r\n");
  }
  else if ((ch == (uint8_t)'c') || (ch == (uint8_t)'C'))
  {
    /* Zero the odometry.  Deliberately NOT done here: the Pose task owns the
     * encoder counters (CODING_STANDARDS 0.3), so this only raises a request and
     * the Pose task performs pose_reset() at the top of its next cycle.  That is
     * the difference between a clean zero and a torn 32-bit total. */
    g_pose_reset_req = 1U;
    dbg_printf("[enc] zero requested (pose + both encoder totals)\r\n");
  }
  else if ((ch == (uint8_t)'e') || (ch == (uint8_t)'E'))
  {
    /* Raw encoder readout for the 1.000 m calibration and the direction check.
     * Reads the software totals, which only the Pose task writes; a single
     * aligned 32-bit read cannot tear, so no lock is needed for a display. */
    pose_t p;
    imu_sample_t s;

    pose_get(&p);
    imu_get(&s);
    dbg_printf("[enc] L=%ld R=%ld cnt | rpm %d/%d | x=%ld y=%ld | h=%ld gyro=%ld (x0.1deg)\r\n",
               (long)encoder_left_total(), (long)encoder_right_total(),
               (int)encoder_left_rpm(), (int)encoder_right_rpm(),
               (long)lroundf(p.x_mm), (long)lroundf(p.y_mm),
               (long)lroundf(p.heading_deg * 10.0f),
               (long)lroundf(s.heading_deg * 10.0f));
  }
  else if ((ch == (uint8_t)'h') || (ch == (uint8_t)'H') || (ch == (uint8_t)'?'))
  {
    dbg_printf("[cmd] s=stop lidar g=restart lidar f/b=both fwd/rev "
               "l/r=left/right only x=stop c=zero enc e=read enc i=prio demo h=help\r\n");
  }
  else if ((ch == (uint8_t)'i') || (ch == (uint8_t)'I'))
  {
    /* Role 5 evidence.  Raise-and-return: the demo runs in its own low-priority
     * tasks, so the console (and everyone else) keeps running. */
    demo_prio_start();
  }
  else
  {
    /* ignore everything else, including terminal CR/LF */
  }
}

static void bt_tx_task(void *argument)
{
  uint32_t tick = osKernelGetTickCount();
  uint32_t slot = 0U;
  uint32_t stats_prev_bytes = 0U;
  uint32_t stats_prev_ms = HAL_GetTick();
  (void)argument;

  dbg_task_started("btTx", APP_PERIOD_SLOW_MS, APP_PRIO_BT_TX, APP_STK_BT_TX);

  for (;;)
  {
    tick += APP_PERIOD_SLOW_MS;
    timing_begin(APP_TIMING_BT_TX, APP_PERIOD_SLOW_MS * 1000U);

    console_poll();
    slot++;

    /* Slot policy: odd slots carry the 10 Hz binary data frame (the PC viewer
     * depends on that rate), even slots are free, and every 20th slot - always
     * even, so it can never collide - carries the 1 Hz statistics frame.
     * telemetry.c allows one UART5 transfer at a time, so a collision would
     * silently drop one of the two frames. */
    if ((slot % 2U) != 0U)
    {
      imu_sample_t sample;

      /* The LiDAR task is the only writer of the 360-bin map; a short critical
       * section gives a consistent snapshot (CODING_STANDARDS 1.5). */
      taskENTER_CRITICAL();
      lidar_get_scan(scan_frame, qual_frame);
      taskEXIT_CRITICAL();

      imu_get(&sample);
      /* Non-blocking: if UART5 TX is still busy this frame is dropped and
       * counted by telemetry.c. */
      (void)telemetry_send_data(&sample, scan_frame);
    }

    if ((slot % (APP_STATUS_PERIOD_MS / APP_PERIOD_SLOW_MS)) == 0U)
    {
      static lidar_stats_t ls;
      static imu_stats_t   is;
      static telemetry_stats_t ts;
      uint32_t now_ms = HAL_GetTick();
      uint32_t span_ms = now_ms - stats_prev_ms;
      uint32_t bps;

      if (span_ms == 0U)
      {
        span_ms = 1U;
      }

      lidar_get_stats(&ls);
      imu_get_stats(&is);
      telemetry_get_stats(&ts);

      bps = (uint32_t)(((uint64_t)(ls.bytes - stats_prev_bytes) * 1000U) / span_ms);
      stats_prev_bytes = ls.bytes;
      stats_prev_ms = now_ms;

      (void)telemetry_send_stats(ts.frames, bps, &ls, &is);
    }

    timing_end(APP_TIMING_BT_TX);
    osDelayUntil(tick);
    timing_check_late(APP_TIMING_BT_TX, tick, APP_PERIOD_SLOW_MS);
  }
}

/* ========================================================================== */
/* Task 8: OLED display + 1 Hz status block (50 ms)                           */
/* ========================================================================== */
static uint32_t status_prev_blocks;
static uint32_t status_prev_ms;
static uint32_t status_prev_samples;

/* "period_ms/jitter_us/wcet_us" for one periodic task, as CODING_STANDARDS 1.3
 * requires for EVERY periodic task (the 鎼? example only shows the motor one). */
static void fmt_timing(char *dst, uint32_t dst_len, uint32_t slot)
{
  uint32_t per_us = timing_pub[slot].period_us_max;

  (void)snprintf(dst, dst_len, "%lu.%02lu/%lu/%lu",
                 (unsigned long)(per_us / 1000U),
                 (unsigned long)((per_us % 1000U) / 10U),
                 (unsigned long)timing_pub[slot].jitter_us_max,
                 (unsigned long)timing_pub[slot].wcet_us_max);
}

static void status_line_1hz(void)
{
  static lidar_stats_t   ls;
  static imu_stats_t     is;
  static i2c_bus_stats_t bs;
  static bt_rx_stats_t   br;
  static path_stats_t    ps;
  static log_stats_t     lg;
  static app_rtos_stats_t rs;
  pose_t p;

  uint32_t now_ms = HAL_GetTick();
  uint32_t span_ms = now_ms - status_prev_ms;
  uint32_t blk = 0U;
  uint32_t imu_hz = 0U;
  uint32_t hold_us;
  uint32_t per_us;
  int32_t  h10;

  if (span_ms == 0U)
  {
    span_ms = 1U;
  }

  lidar_get_stats(&ls);
  imu_get_stats(&is);
  i2c_bus_get_stats(&bs);
  bt_rx_get_stats(&br);
  path_get_stats(&ps);
  log_get_stats(&lg);
  app_rtos_get_stats(&rs);
  pose_get(&p);

  blk = (uint32_t)(((uint64_t)(ls.blocks - status_prev_blocks) * 1000U) / span_ms);
  status_prev_blocks = ls.blocks;

  /* CODING_STANDARDS 2.5: the IMU sample rate must be measurable (~100 Hz).
   * Measured as a delta, never assumed. */
  imu_hz = (uint32_t)(((uint64_t)(is.samples - status_prev_samples) * 1000U) / span_ms);
  status_prev_samples = is.samples;

  status_prev_ms = now_ms;

  /* ---- line 1: LiDAR, motor task, Q_Cmd -------------------------------- */
  per_us = rs.timing[APP_TIMING_PID_MOTOR].period_us_max;
  dbg_printf("[1s] L: blk%lu q%lu/%u p%lu | M: per%lu.%02lu jit%lu wc%lu | C: q%lu/%u\r\n",
             (unsigned long)blk,
             (unsigned long)rs.q_lidar_peak,
             (unsigned)APP_Q_LIDAR_LEN,
             (unsigned long)rs.max_parse_us,
             (unsigned long)(per_us / 1000U),
             (unsigned long)((per_us % 1000U) / 10U),
             (unsigned long)rs.timing[APP_TIMING_PID_MOTOR].jitter_us_max,
             (unsigned long)rs.timing[APP_TIMING_PID_MOTOR].wcet_us_max,
             (unsigned long)osMessageQueueGetCount(q_cmd),
             (unsigned)APP_Q_CMD_LEN);

  /* ---- line 2: pose, Q_Path, path progress ----------------------------- */
  h10 = (int32_t)lroundf(p.heading_deg * 10.0f);
  dbg_printf("[1s] P: eh%ld.%ld ex%ld ey%ld | QP: %lu | PATH: g%lu r%lu\r\n",
             (long)(h10 / 10),
             (long)((h10 % 10 < 0) ? -(h10 % 10) : (h10 % 10)),
             (long)lroundf(p.x_mm),
             (long)lroundf(p.y_mm),
             (unsigned long)osMessageQueueGetCount(q_path),
             (unsigned long)ps.generated,
             (unsigned long)ps.reached);

  /* ---- line 3: I2C, heap, watchdog, IMU rate --------------------------- */
  hold_us = bs.hold_us_max;
  dbg_printf("[1s] I2C: ok%lu err%lu hold%lu.%lums | H: %lu/%u | WD: %u | IMU: %lu/s\r\n",
             (unsigned long)bs.ok,
             (unsigned long)bs.err,
             (unsigned long)(hold_us / 1000U),
             (unsigned long)((hold_us % 1000U) / 100U),
             (unsigned long)rs.heap_free,
             (unsigned)rs.heap_total,
             (unsigned)0U,   /* IWDG is not enabled yet - see HANDOVER section 7 */
             (unsigned long)imu_hz);

  /* ---- line 4: stacks, in the documented order ------------------------- */
  dbg_printf("[1s] STK: %lu %lu %lu %lu %lu %lu %lu %lu\r\n",
             (unsigned long)rs.stack_free[APP_TASK_PID_MOTOR],
             (unsigned long)rs.stack_free[APP_TASK_IMU],
             (unsigned long)rs.stack_free[APP_TASK_POSE],
             (unsigned long)rs.stack_free[APP_TASK_LIDAR],
             (unsigned long)rs.stack_free[APP_TASK_PATH],
             (unsigned long)rs.stack_free[APP_TASK_BT_RX],
             (unsigned long)rs.stack_free[APP_TASK_BT_TX],
             (unsigned long)rs.stack_free[APP_TASK_OLED]);

  /* ---- line 5+6: timing of all four periodic tasks (1.3) --------------- */
  {
    char t_motor[32];
    char t_pose[32];
    char t_bttx[32];
    char t_oled[32];

    fmt_timing(t_motor, sizeof(t_motor), APP_TIMING_PID_MOTOR);
    fmt_timing(t_pose, sizeof(t_pose), APP_TIMING_POSE);
    fmt_timing(t_bttx, sizeof(t_bttx), APP_TIMING_BT_TX);
    fmt_timing(t_oled, sizeof(t_oled), APP_TIMING_OLED);

    /* format per row: <key>period_ms/jitter_us/wcet_us */
    dbg_printf("[1s] T10: m%s p%s | ov m%lu p%lu\r\n",
               t_motor, t_pose,
               (unsigned long)rs.timing[APP_TIMING_PID_MOTOR].overruns,
               (unsigned long)rs.timing[APP_TIMING_POSE].overruns);

    dbg_printf("[1s] T50: b%s o%s | ov b%lu o%lu\r\n",
               t_bttx, t_oled,
               (unsigned long)rs.timing[APP_TIMING_BT_TX].overruns,
               (unsigned long)rs.timing[APP_TIMING_OLED].overruns);
  }

  /* ---- line 7: the failure counters CODING_STANDARDS 5.5 requires ------ */
  /* Label note: the old label "imu2" printed as "imu20" when the value was 0,
   * which reads like the number 20.  Renamed to iw (IMU wait timeouts). */
  dbg_printf("[1s] CNT: ql%lu qc%lu qpt%lu imu%lu bt(s%lu r%lu o%lu u%lu) log%lu iw%lu\r\n",
             (unsigned long)rs.blocks_dropped,
             (unsigned long)rs.q_cmd_overwrite,
             (unsigned long)rs.q_path_timeout,
             (unsigned long)rs.imu_ticks_unused,
             (unsigned long)br.syntax_err,
             (unsigned long)br.range_err,
             (unsigned long)br.overlong,
             (unsigned long)br.uart_errors,
             (unsigned long)lg.drops,
             (unsigned long)rs.imu_wait_timeouts);
}

static void oled_task(void *argument)
{
  uint32_t tick = osKernelGetTickCount();
  uint32_t slot = 0U;
  (void)argument;

  dbg_task_started("oled", APP_PERIOD_SLOW_MS, APP_PRIO_OLED, APP_STK_OLED);

  for (;;)
  {
    uint8_t page = 0U;
    pose_t p;
    i2c_bus_stats_t bs;

    tick += APP_PERIOD_SLOW_MS;
    timing_begin(APP_TIMING_OLED, APP_PERIOD_SLOW_MS * 1000U);

    pose_get(&p);
    i2c_bus_get_stats(&bs);

    /* One page per slot: 8 slots = 400 ms for a full frame, which is why the
     * content must be a status screen and not a live trace. */
    oled_row(0U, "PART2 NAV v0.3");
    oled_row(1U, "X%6ld Y%6ld", (long)lroundf(p.x_mm), (long)lroundf(p.y_mm));
    oled_row(2U, "H%6ld QP%3lu", (long)lroundf(p.heading_deg),
             (unsigned long)osMessageQueueGetCount(q_path));
    oled_row(3U, "CQ%2lu LQ%2lu", (unsigned long)osMessageQueueGetCount(q_cmd),
             (unsigned long)osMessageQueueGetCount(q_lidar));
    oled_row(4U, "I2C ok%lu e%lu", (unsigned long)bs.ok, (unsigned long)bs.err);
    oled_row(5U, "OLED %s", (oled_is_present() != 0U) ? "ok" : "absent");
    oled_row(6U, "H src %s", (p.heading_src == 0U) ? "gyro" : "wheel");
    oled_row(7U, "t%lu s%lu", (unsigned long)osKernelGetTickCount(),
             (unsigned long)slot);

    /* A missing panel is not an error here: oled.c returns OLED_ERR_ABSENT and
     * the framebuffer stays usable, so the display code is identical either way. */
    (void)oled_refresh_next(&page);

    slot++;

    if ((slot % (APP_STATUS_PERIOD_MS / APP_PERIOD_SLOW_MS)) == 0U)
    {
      status_line_1hz();
    }

    timing_end(APP_TIMING_OLED);
    osDelayUntil(tick);
    timing_check_late(APP_TIMING_OLED, tick, APP_PERIOD_SLOW_MS);
  }
}

/* ========================================================================== */
/* ========================================================================== */
/* Role 5 demo: priority inheritance, measured on the holder itself           */
/* ========================================================================== */
/* Scoring role 5 wants "the mutex plus a demonstration of priority
 * inheritance".  A hold-time number on the 1 Hz line is not that: it shows the
 * mutex works, not that the inversion it prevents would otherwise happen.
 *
 * This measures the defining property directly.  With two mutexes that differ
 * in exactly one attribute:
 *
 *   low  (11) takes the mutex and burns CPU while holding it
 *   high (13) blocks on the same mutex
 *   -> if the mutex was created with osMutexPrioInherit, the kernel raises
 *      low to 13 for as long as high waits; the plain mutex leaves it at 11
 *
 * low reads its OWN priority while high is blocked, which is the boost itself,
 * so the demonstration does not depend on any scheduler ordering assumption
 * (an earlier version tried to show it through a third "medium" task and could
 * not get the medium task scheduled inside the hold at all - the boost is the
 * mechanism, the medium task is only a story about the consequence).
 *
 * Deliberate choices:
 *  - all demo priorities are BELOW the lowest real task (OLED = 20), so the
 *    demo can never delay the control loop; it runs in slack time only;
 *  - it touches only its own two mutexes, never I2C1, so a bug here cannot
 *    deadlock the real bus;
 *  - one-shot per 'i' keypress; nothing runs until asked.
 */
#define DEMO_HOLD_US       60000U
#define DEMO_PRIO_LOW        11U
#define DEMO_PRIO_HIGH       13U
#define DEMO_STK           1024U
#define DEMO_TIMEOUT_MS    3000U

static osMutexId_t     mx_demo;           /* created WITH prio inheritance   */
static osSemaphoreId_t sem_demo_plain;    /* binary semaphore used as a lock */
static osSemaphoreId_t sem_demo_low;      /* 'i' -> start a run              */
static osSemaphoreId_t sem_demo_high;     /* low -> high: mutex is held      */
static osSemaphoreId_t sem_demo_high_done;

static osThreadId_t t_demo_low;
static osThreadId_t t_demo_high;

static const osMutexAttr_t mx_demo_inh_attr = {
  .name = "mxDemoInherit", .attr_bits = osMutexPrioInherit,
};
static const osThreadAttr_t demo_low_attr = {
  .name = "demoLow", .stack_size = DEMO_STK, .priority = (osPriority_t)DEMO_PRIO_LOW,
};
static const osThreadAttr_t demo_high_attr = {
  .name = "demoHigh", .stack_size = DEMO_STK, .priority = (osPriority_t)DEMO_PRIO_HIGH,
};

static volatile uint32_t demo_mode;              /* which mutex is being timed */
static volatile uint32_t demo_hold_us[2];
static volatile uint32_t demo_wait_us[2];
static volatile uint32_t demo_prio_seen[2];      /* holder priority while high waits */
static volatile uint32_t demo_prio_after[2];     /* holder priority after release    */

static uint32_t demo_cycles_to_us(uint32_t cyc)
{
  return (uint32_t)(((uint64_t)cyc * 1000000ULL) / (uint64_t)SystemCoreClock);
}

static void demo_busy_us(uint32_t us)
{
  uint32_t start  = DWT->CYCCNT;
  uint32_t cycles = (uint32_t)(((uint64_t)us * (uint64_t)SystemCoreClock) / 1000000ULL);

  while ((uint32_t)(DWT->CYCCNT - start) < cycles)
  {
  }
}

/* The two primitives under test.  FreeRTOS mutexes ALWAYS carry priority
 * inheritance (xSemaphoreCreateMutex() has no non-inheriting variant), so the
 * honest contrast is mutex versus binary semaphore used as a lock - which is
 * also the reason CODING_STANDARDS 2.4 mandates a mutex for the I2C bus. */
static osStatus_t demo_lock_acquire(uint32_t which)
{
  if (which == 0U)
  {
    return osMutexAcquire(mx_demo, osWaitForever);
  }
  return osSemaphoreAcquire(sem_demo_plain, osWaitForever);
}

static void demo_lock_release(uint32_t which)
{
  if (which == 0U)
  {
    (void)osMutexRelease(mx_demo);
  }
  else
  {
    (void)osSemaphoreRelease(sem_demo_plain);
  }
}

static void demo_print_ms(const char *label, uint32_t us)
{
  dbg_printf("%s%lu.%02lums", label, (unsigned long)(us / 1000U),
             (unsigned long)((us % 1000U) / 10U));
}

static void demo_low_task(void *argument)
{
  (void)argument;

  for (;;)
  {
    uint32_t m;

    if (osSemaphoreAcquire(sem_demo_low, osWaitForever) != osOK)
    {
      continue;
    }

    dbg_printf("[demo] priority inheritance: mutex vs binary semaphore\r\n");

    for (m = 0U; m < 2U; m++)
    {
      uint32_t t0;
      uint32_t prio_seen = 0U;

      demo_mode        = m;
      demo_wait_us[m]  = 0xFFFFFFFFU;
      demo_hold_us[m]  = 0U;
      demo_prio_seen[m]  = 0U;
      demo_prio_after[m] = 0U;

      if (demo_lock_acquire(m) != osOK)
      {
        continue;
      }

      /* Released only with the mutex already held, so high's wait is honestly
       * "blocked by low".  high preempts here and blocks on the mutex. */
      (void)osSemaphoreRelease(sem_demo_high);

      t0 = DWT->CYCCNT;

      /* Sample the holder's priority across the hold: with inheritance it must
       * read 13, without it 11.  Sampling repeatedly is what makes this robust
       * against the exact moment high gets queued. */
      {
        uint32_t chunk;
        for (chunk = 0U; chunk < 10U; chunk++)
        {
          uint32_t p;

          demo_busy_us(DEMO_HOLD_US / 10U);
          p = (uint32_t)osThreadGetPriority(osThreadGetId());
          if (p > prio_seen)
          {
            prio_seen = p;
          }
          taskYIELD();
        }
      }

      demo_hold_us[m]   = demo_cycles_to_us(DWT->CYCCNT - t0);
      demo_prio_seen[m] = prio_seen;

      (void)demo_lock_release(m);

      demo_prio_after[m] = (uint32_t)osThreadGetPriority(osThreadGetId());

      (void)osSemaphoreAcquire(sem_demo_high_done, DEMO_TIMEOUT_MS);
    }

    /* ---- report ------------------------------------------------------- */
    {
      uint32_t i;

      for (i = 0U; i < 2U; i++)
      {
        demo_print_ms((i == 0U) ? "[demo] mutex   : high waited "
                                : "[demo] semaphore: high waited ", demo_wait_us[i]);
        dbg_printf(", holder prio %lu while high waited, %lu after release\r\n",
                   (unsigned long)demo_prio_seen[i], (unsigned long)demo_prio_after[i]);
      }

      if ((demo_prio_seen[0] == DEMO_PRIO_HIGH) && (demo_prio_seen[1] == DEMO_PRIO_LOW))
      {
        dbg_printf("[demo] result: inheritance confirmed (holder 11->13 only with osMutexPrioInherit)\r\n");
      }
      else if ((demo_prio_seen[0] == 0U) || (demo_prio_seen[1] == 0U))
      {
        dbg_printf("[demo] result: inconclusive (high never blocked - check priorities)\r\n");
      }
      else
      {
        dbg_printf("[demo] result: unexpected holder priorities %lu / %lu\r\n",
                   (unsigned long)demo_prio_seen[0], (unsigned long)demo_prio_seen[1]);
      }
    }
  }
}

static void demo_high_task(void *argument)
{
  (void)argument;

  for (;;)
  {
    if (osSemaphoreAcquire(sem_demo_high, osWaitForever) != osOK)
    {
      continue;
    }

    {
      uint32_t t0 = DWT->CYCCNT;

      if (demo_lock_acquire(demo_mode) == osOK)
      {
        demo_wait_us[demo_mode] = demo_cycles_to_us(DWT->CYCCNT - t0);
        (void)demo_lock_release(demo_mode);
      }
      (void)osSemaphoreRelease(sem_demo_high_done);
    }
  }
}

/* Called from the console ('i').  Only raises the start semaphore; the demo
 * runs in its own low-priority task, so the console never blocks. */
static void demo_prio_start(void)
{
  if (sem_demo_low == NULL)
  {
    dbg_printf("[demo] not initialised\r\n");
    return;
  }
  if (osSemaphoreRelease(sem_demo_low) != osOK)
  {
    dbg_printf("[demo] already running, press i again after it prints\r\n");
    return;
  }
  dbg_printf("[demo] starting (60 ms hold, priorities low=11 high=13)\r\n");
}
/* ========================================================================== */
/* init / start                                                               */
/* ========================================================================== */
void app_rtos_init(void)
{
  memset(&rtos_stats, 0, sizeof(rtos_stats));
  memset(timing_int, 0, sizeof(timing_int));
  memset(timing_pub, 0, sizeof(timing_pub));

  if (osKernelInitialize() != osOK)
  {
    Error_Handler();
  }

  q_lidar = osMessageQueueNew(APP_Q_LIDAR_LEN, sizeof(lidar_block_desc_t), &q_lidar_attr);
  q_cmd   = osMessageQueueNew(APP_Q_CMD_LEN, sizeof(cmd_msg_t), &q_cmd_attr);
  q_path  = osMessageQueueNew(APP_Q_PATH_LEN, sizeof(path_point_t), &q_path_attr);

  sem_imu   = osSemaphoreNew(APP_IMU_SEM_MAX, 0U, &sem_imu_attr);
  sem_bt_rx = osSemaphoreNew(APP_BT_RX_SEM_MAX, 0U, &sem_bt_attr);

  evt_nav = osEventFlagsNew(&evt_nav_attr);

  if ((q_lidar == NULL) || (q_cmd == NULL) || (q_path == NULL) ||
      (sem_imu == NULL) || (sem_bt_rx == NULL) || (evt_nav == NULL))
  {
    Error_Handler();
  }

  t_pid  = osThreadNew(pid_motor_task, NULL, &pid_motor_attr);
  t_imu  = osThreadNew(imu_task, NULL, &imu_attr);
  t_pose = osThreadNew(pose_task, NULL, &pose_attr);
  t_lidar = osThreadNew(lidar_task, NULL, &lidar_attr);
  t_path = osThreadNew(path_task, NULL, &path_attr);
  t_bt_rx = osThreadNew(bt_rx_task, NULL, &bt_rx_attr);
  t_bt_tx = osThreadNew(bt_tx_task, NULL, &bt_tx_attr);
  t_oled = osThreadNew(oled_task, NULL, &oled_attr);

  if ((t_pid == NULL) || (t_imu == NULL) || (t_pose == NULL) || (t_lidar == NULL) ||
      (t_path == NULL) || (t_bt_rx == NULL) || (t_bt_tx == NULL) || (t_oled == NULL))
  {
    Error_Handler();
  }

  /* The queue-ownership policy lives next to the queue: bt_rx only knows it has
   * a sink to call (bt_rx.h). */
  bt_rx_set_sink(cmd_sink);

  /* ---- role 5 demo objects -------------------------------------------- */
  /* Two mutexes that differ in exactly one attribute: that is what makes the
   * comparison a controlled experiment rather than two unrelated numbers. */
  mx_demo        = osMutexNew(&mx_demo_inh_attr);
  sem_demo_plain = osSemaphoreNew(1U, 1U, NULL);   /* 1 = free, like a mutex */

  sem_demo_low       = osSemaphoreNew(1U, 0U, NULL);
  sem_demo_high      = osSemaphoreNew(1U, 0U, NULL);
  sem_demo_high_done = osSemaphoreNew(1U, 0U, NULL);

  t_demo_low  = osThreadNew(demo_low_task,  NULL, &demo_low_attr);
  t_demo_high = osThreadNew(demo_high_task, NULL, &demo_high_attr);

  if ((mx_demo == NULL) || (sem_demo_plain == NULL) ||
      (sem_demo_low == NULL) || (sem_demo_high == NULL) ||
      (sem_demo_high_done == NULL) ||
      (t_demo_low == NULL) || (t_demo_high == NULL))
  {
    Error_Handler();
  }
}

void app_rtos_start(void)
{
  if (osKernelStart() != osOK)
  {
    Error_Handler();
  }
}

void app_rtos_get_stats(app_rtos_stats_t *out)
{
  osThreadId_t ids[APP_TASK_COUNT];
  uint32_t i;

  if (out == NULL)
  {
    return;
  }

  ids[APP_TASK_PID_MOTOR] = t_pid;
  ids[APP_TASK_IMU]       = t_imu;
  ids[APP_TASK_POSE]      = t_pose;
  ids[APP_TASK_LIDAR]     = t_lidar;
  ids[APP_TASK_PATH]      = t_path;
  ids[APP_TASK_BT_RX]     = t_bt_rx;
  ids[APP_TASK_BT_TX]     = t_bt_tx;
  ids[APP_TASK_OLED]      = t_oled;

  /* Copy the counters under a short critical section so a 1 Hz snapshot cannot
   * mix two different moments (memory only, no calls inside - 1.5). */
  taskENTER_CRITICAL();
  memcpy(out, &rtos_stats, sizeof(*out));
  memcpy(out->timing, timing_pub, sizeof(timing_pub));
  taskEXIT_CRITICAL();

  out->heap_total = (uint32_t)configTOTAL_HEAP_SIZE;
  out->heap_free = (uint32_t)xPortGetFreeHeapSize();
  out->heap_min_free = (uint32_t)xPortGetMinimumEverFreeHeapSize();

  for (i = 0U; i < APP_TASK_COUNT; i++)
  {
    out->stack_free[i] = (ids[i] != NULL)
        ? (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)ids[i]) * sizeof(StackType_t)
        : 0U;
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