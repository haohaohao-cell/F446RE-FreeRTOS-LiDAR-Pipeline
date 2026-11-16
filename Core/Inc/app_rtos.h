#ifndef APP_RTOS_H
#define APP_RTOS_H

#include <stdint.h>
#include "lidar.h"

/* -------------------------------------------------------------------------- */
/* FreeRTOS object sizing                                                      */
/*                                                                            */
/* Queue 1 holds 12-byte block descriptors only.  Depth 8 gives the parser     */
/* task ~8 x 22 ms of slack before a descriptor would be dropped, which is     */
/* far more than the ~40 us it needs per block.                                */
/* Queue 2 holds 32-byte results; depth 4 is plenty because the Output task    */
/* runs at 10 Hz on the same 100 ms cadence as the telemetry frame.            */
/* -------------------------------------------------------------------------- */
#define APP_Q_BLOCKS_LEN     8U
#define APP_Q_RESULTS_LEN    4U

/* Task stacks, in BYTES (CMSIS-RTOS v2 osThreadAttr_t semantics) */
#define APP_PARSER_STACK     1536U
#define APP_OUTPUT_STACK     2048U

/* Output cadence */
#define APP_FRAME_PERIOD_MS  100U
#define APP_STATS_PERIOD_MS  1000U

/* Guard timeout on Queue 1: only there to keep the stream watchdog alive when
 * the sensor goes silent.  Must stay comfortably above one DMA block period
 * (~22 ms) so it never fires during normal operation. */
#define APP_PARSER_IDLE_MS   100U

/* -------------------------------------------------------------------------- */
/* Run-time statistics collected for the report / demonstration                */
/* -------------------------------------------------------------------------- */
typedef struct
{
  uint32_t blocks_posted;        /* descriptors pushed by the ISRs           */
  uint32_t blocks_consumed;      /* descriptors taken by the parser task     */
  uint32_t blocks_dropped;       /* Queue 1 was full                         */
  uint32_t results_posted;       /* results handed to Queue 2                */
  uint32_t results_dropped;      /* Queue 2 was full                         */
  uint32_t q1_peak;              /* max Queue 1 occupancy (overwrite risk)   */
  uint32_t q2_peak;              /* max Queue 2 occupancy                    */
  uint32_t parser_idle_wakeups;  /* Queue 1 get timed out (sensor silent)    */
  uint32_t max_parse_us;         /* worst-case task-side parse time          */
  uint32_t max_handover_us;      /* worst-case ISR -> task wake-up latency   */
  uint32_t parser_stack_free;    /* high-water mark, bytes                   */
  uint32_t output_stack_free;    /* high-water mark, bytes                   */
  uint32_t heap_free;            /* current FreeRTOS heap free bytes         */
  uint32_t heap_min_free;        /* lowest ever FreeRTOS heap free bytes     */
} app_rtos_stats_t;

/* Called from app_init(), i.e. before the scheduler is started.  Creates both
 * queues and both tasks; nothing is running yet. */
void app_rtos_init(void);

/* Creates nothing, starts the scheduler.  Never returns. */
void app_rtos_start(void);

/* Interrupt-context entry point used by lidar.c.
 * Returns 0 on success, -1 if Queue 1 was full.
 * MUST only be called from an ISR whose NVIC priority is numerically >=
 * configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5 in this project). */
int app_rtos_post_block(const lidar_block_desc_t *desc);

void app_rtos_get_stats(app_rtos_stats_t *out);

#endif /* APP_RTOS_H */
