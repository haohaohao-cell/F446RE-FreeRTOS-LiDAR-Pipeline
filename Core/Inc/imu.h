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
void imu_start(void);
void imu_poll(void);
void imu_reset_heading(void);
void imu_on_error(void);
void imu_get(imu_sample_t *out);
void imu_get_stats(imu_stats_t *out);

#endif
