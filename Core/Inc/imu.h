#ifndef IMU_H
#define IMU_H

#include <stdint.h>

typedef struct
{
  float heading_deg;
  float gz_dps;
  float ax_g;
  float ay_g;
  uint8_t valid;
} imu_sample_t;

typedef struct
{
  uint32_t ticks;
  uint32_t samples;
  uint32_t skipped;
  uint32_t errors;
  uint32_t first_sample_ms;
  uint32_t recoveries;
} imu_stats_t;

int imu_init(void);
extern uint32_t imu_diag_status;
extern uint32_t imu_diag_err;
extern uint32_t imu_diag_who;

/* Start the TIM6 10 ms cadence.  The ISR releases a semaphore; it does not
 * sample.  Call once, before osKernelStart(). */
void imu_start(void);

/* Perform ONE sample + heading integration in the calling task's context.
 * The IMU task calls this after osSemaphoreAcquire(), which is the whole point
 * of the Phase C change: no I2C traffic from interrupt context, and the shared
 * bus mutex is held across the transfer.
 *
 * Returns 0 on success, -1 if no IMU was detected at init, -2 on a bus error
 * (already counted; recovery is handled by i2c_bus_*). */
int imu_sample_once(void);

void imu_reset_heading(void);

/* Interrupt-context error notification from HAL_I2C_ErrorCallback.  It only
 * counts: the synchronous API reports the failure through the return value, and
 * bus recovery must run in task context. */
void imu_on_error(void);

void imu_get(imu_sample_t *out);
void imu_get_stats(imu_stats_t *out);

#endif
