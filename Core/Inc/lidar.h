#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* DMA block strategy                                                          */
/* -------------------------------------------------------------------------- */
/* One circular DMA buffer, logically split into two halves that DMA fills
 * alternately (ping-pong).  A half-complete interrupt means half 0 is ready,
 * a complete interrupt means half 1 is ready. */
#define LIDAR_DMA_SIZE   2048U
#define LIDAR_HALF_SIZE  (LIDAR_DMA_SIZE / 2U)

/* Persistent 360-entry polar map used by the telemetry frame */
#define LIDAR_BINS       360U

/* Sector summary resolution for the per-block result (8 x 45 degrees) */
#define LIDAR_SECTORS    8U

/* No valid return marks an empty sector slot */
#define LIDAR_SECTOR_EMPTY  0xFFU

/* -------------------------------------------------------------------------- */
/* Queue 1 message: a *small descriptor* of a completed DMA block.             */
/*                                                                            */
/* The LiDAR bytes are NOT copied into the queue - they stay in lidar_dma_buf  */
/* and the parser task reads them there using offset/length.  This keeps the    */
/* ISR short and the queue traffic tiny (12 bytes per block).                   */
/* -------------------------------------------------------------------------- */
typedef enum
{
  LIDAR_BLOCK_DATA = 0,      /* a DMA half-buffer has been filled            */
  LIDAR_BLOCK_UART_ERROR = 1 /* UART/DMA error: parser task must re-arm      */
} lidar_block_kind_t;

typedef struct
{
  uint16_t offset;      /* byte offset of the block inside lidar_dma_buf     */
  uint16_t length;      /* block length in bytes                             */
  uint16_t block_id;    /* monotonically increasing block counter            */
  uint8_t  kind;        /* lidar_block_kind_t                                */
  uint8_t  reserved;    /* keeps the struct 12 bytes / 4-byte aligned        */
  uint32_t isr_cycles;  /* DWT->CYCCNT captured in the ISR (latency probe)   */
} lidar_block_desc_t;

/* -------------------------------------------------------------------------- */
/* Queue 2 message: the processed result of one DMA block.                     */
/* -------------------------------------------------------------------------- */
typedef struct
{
  uint16_t block_id;
  uint16_t length;                          /* bytes parsed this block      */
  uint16_t packets;                         /* 5-byte LiDAR packets decoded */
  uint16_t valid;                           /* packets with a real distance */
  uint16_t no_return;                       /* distance == 0                */
  uint16_t invalid;                         /* check-bit / angle rejected   */
  uint16_t nodes;                           /* distinct angle bins touched  */
  uint16_t min_mm;                          /* nearest return in this block */
  uint16_t max_mm;                          /* farthest return              */
  uint8_t  sector_min_cm[LIDAR_SECTORS];    /* 0xFF = no return in sector   */
  uint8_t  synced;                          /* stream descriptor locked     */
  uint8_t  syncs;                           /* descriptor found this block  */
  uint8_t  slips;                           /* resync events this block     */
  uint32_t parse_us;                        /* task-side parse duration     */
  uint32_t handover_us;                     /* ISR -> task wake-up latency  */
  uint32_t tick;                            /* kernel tick when dequeued    */
} lidar_result_t;

/* -------------------------------------------------------------------------- */
/* Cumulative diagnostics (kept for the existing telemetry/statistics path)    */
/* -------------------------------------------------------------------------- */
typedef struct
{
  uint32_t bytes;
  uint32_t packets;
  uint32_t invalid;
  uint32_t syncs;
  uint32_t commands;
  uint32_t ht_count;        /* DMA half-complete events                      */
  uint32_t tc_count;        /* DMA complete events                           */
  uint32_t proc_us_max;     /* worst-case task-side parse time               */
  uint32_t restarts;
  uint32_t restart_fail;
  uint32_t uart_errors;
  uint32_t slips;
  uint32_t no_return;
  uint32_t blocks;          /* blocks handed to the parser task              */
  uint32_t q_full;          /* descriptors dropped because Queue 1 was full  */
  uint32_t stream_losses;   /* times the stream watchdog lost the sensor      */
} lidar_stats_t;

extern uint8_t lidar_dma_buf[LIDAR_DMA_SIZE];

/* Called once from app_init(), before the scheduler starts */
void lidar_init(void);
void lidar_stop(void);
void lidar_restart(void);

/* --- interrupt context ---------------------------------------------------- */
/* These run inside the USART3/DMA1_Stream1 ISRs.  They must stay short: they
 * only build a 12-byte descriptor and post it to Queue 1.  No parsing here. */
void lidar_on_half(void);
void lidar_on_full(void);
void lidar_on_uart_error(void);

/* --- task context (LiDAR Parser Task) ------------------------------------ */
/* Reads the block described by *desc directly out of lidar_dma_buf, updates the
 * persistent 360-bin scan, and fills *out with the per-block result. */
void lidar_parser_process(const lidar_block_desc_t *desc, lidar_result_t *out);
/* Housekeeping that must not run in interrupt context: SCAN command re-issue,
 * stream watchdog, restart after an error.  Called every time the parser task
 * wakes up. */
void lidar_parser_service(void);

/* --- debug-console requests (safe from any task) -------------------------- */
/* 's' / 'g' on the USART2 console.  They only raise a flag; the parser task
 * performs the actual stop/restart, so its stream state stays single-owner. */
void lidar_request_stop(void);
void lidar_request_resume(void);

/* --- shared data for the Output/Debug Task ------------------------------- */
void lidar_get_scan(uint16_t *dist_mm, uint8_t *quality);
void lidar_get_stats(lidar_stats_t *out);

#endif /* LIDAR_H */
