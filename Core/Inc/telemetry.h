#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "imu.h"
#include "lidar.h"

#define TELEMETRY_SYNC0  0xAAU
#define TELEMETRY_SYNC1  0x55U
#define TELEMETRY_DATA   0x01U
#define TELEMETRY_STATS  0x02U

typedef struct
{
  uint32_t frames;
  uint32_t bytes;
  uint32_t drops;
  uint32_t errors;
} telemetry_stats_t;

void telemetry_init(void);
uint32_t telemetry_bt_baud(void);
int telemetry_send_data(const imu_sample_t *s, const uint16_t *dist);
int telemetry_send_stats(uint32_t frames_sent, uint32_t bytes_per_s,
                         const lidar_stats_t *ls, const imu_stats_t *is);
void telemetry_on_error(void);

/* UART5 TX-complete notification.  Called from HAL_UART_TxCpltCallback in app.c
 * (Phase C moved that callback there so USART2's log DMA and UART5's telemetry
 * DMA share one dispatcher instead of fighting over the weak symbol). */
void telemetry_on_tx_complete(void);

void telemetry_get_stats(telemetry_stats_t *out);

#endif
