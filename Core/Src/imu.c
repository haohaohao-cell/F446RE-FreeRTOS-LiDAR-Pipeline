/* -----------------------------------------------------------------------------
 * imu.c - MPU-6500 on I2C1, sampled from TASK context.
 *
 * WHAT CHANGED IN PHASE C (and why)
 * ---------------------------------
 * Before: the TIM6 100 Hz interrupt issued HAL_I2C_Mem_Read_IT() and the result
 * was integrated in HAL_I2C_MemRxCpltCallback.  That is exactly what
 * CODING_STANDARDS 0.4 forbids - an ISR starting a bus transaction and a second
 * ISR doing the maths - and it made the shared-bus mutex impossible to hold
 * across the transfer, because an interrupt cannot own a mutex.
 *
 * Now: TIM6 releases a counting semaphore; the IMU task acquires it and calls
 * imu_sample_once(), which performs one blocking-but-bounded read through
 * i2c_bus_mem_read() (mutex held for the whole transaction) and integrates the
 * heading in the same task context.  No HAL I2C call remains in this file's
 * interrupt path; the only ISR work left is "count the tick, release the
 * semaphore".
 *
 * The TIM6 HAL callback stays in this file because it belongs to the sampling
 * cadence, but it does not touch the bus any more - it calls
 * app_rtos_imu_tick(), which is a bare osSemaphoreRelease().
 *
 * HEADING INTEGRATION: dt comes from DWT (not from the nominal 10 ms) so a
 * late task or a preempted sample does not skew the gyro integration.  dt is
 * clamped to 200 ms so a long stall (debugger, bus recovery) cannot produce a
 * huge heading jump; that clamp is a deliberate, visible approximation.
 * -------------------------------------------------------------------------- */

#include "imu.h"
#include "app_rtos.h"
#include "board_pins.h"
#include "i2c_bus.h"

#include <string.h>

extern TIM_HandleTypeDef htim6;

#define IMU_ADDR7      BOARD_IMU_I2C_ADDR_7BIT
#define IMU_WHO_AM_I   0x75U
#define IMU_PWR_MGMT_1 0x6BU
#define IMU_CONFIG     0x1AU
#define IMU_GYRO_CFG   0x1BU
#define IMU_ACCEL_CFG  0x1CU
#define IMU_ACCEL_XOUT 0x3BU
#define IMU_GYRO_SENS  131.0f
#define IMU_ACCEL_SENS 16384.0f

/* Bounded per CODING_STANDARDS 5.1: one sample must never wait on the bus for
 * longer than this, whatever the panel is doing. */
#define IMU_TXN_TMO_MS 10U

/* dt clamp for the gyro integration (see the header comment). */
#define IMU_DT_MAX_S   0.2f

static uint32_t last_cycles;
static uint32_t last_ms;
static float    bias_gz;
static float    heading_deg;
static float    gz_dps;
static float    ax_g;
static float    ay_g;
static imu_stats_t stats;
static uint8_t  imu_present;

uint32_t imu_diag_status;
uint32_t imu_diag_err;
uint32_t imu_diag_who;

static int16_t be16(const uint8_t *p)
{
  return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static int imu_wr(uint8_t reg, uint8_t val)
{
  return (i2c_bus_mem_write(IMU_ADDR7, reg, &val, 1U, IMU_TXN_TMO_MS) == I2C_BUS_OK) ? 0 : -1;
}

static int imu_rd(uint8_t reg, uint8_t *buf, uint16_t len)
{
  return (i2c_bus_mem_read(IMU_ADDR7, reg, buf, len, IMU_TXN_TMO_MS) == I2C_BUS_OK) ? 0 : -1;
}

/* ========================================================================== */
/* init (boot context, scheduler not running yet)                             */
/* ========================================================================== */
int imu_init(void)
{
  uint8_t who = 0U;
  uint8_t ok = 0U;
  uint8_t raw[14];

  imu_present = 0U;

  /* HOW LONG TO WAIT FOR THE CHIP (fixed 2026-11-19)
   * ------------------------------------------------
   * This used to be 5 attempts x 20 ms = a 100 ms window, and it was too
   * short.  On 2026-11-19 22:05 the board rebooted on battery power and the
   * MPU-6500 never answered WHO_AM_I inside that window: it ACKed its address
   * on an I2C scan (`d` found 0x68) but every register read failed, so the
   * firmware declared the sensor absent.  That costs the gyro AND it stopped
   * TIM6 (see imu_start), which made the whole 1 Hz line look broken.
   *
   * A battery cold start raises 3V3 more slowly than the USB-powered bench
   * setup this was developed on, and the MPU-6500 wants up to ~100 ms of
   * stable supply on top of that.  So: wait up to ~1 s, and poke the device
   * out of sleep halfway through in case it is sitting in a state where it
   * ACKs but does not serve reads.
   *
   * This only costs time when the sensor really is absent (boot, no
   * scheduler, HAL_Delay is legal here - see the note above the bias loop). */
  for (uint16_t attempt = 0U; attempt < 50U; attempt++)
  {
    who = 0U;
    imu_diag_status = (uint32_t)imu_rd(IMU_WHO_AM_I, &who, 1U);
    imu_diag_who = who;

    if ((imu_diag_status == 0U) &&
        ((who == 0x70U) || (who == 0x71U) || (who == 0x73U)))
    {
      ok = 1U;
      break;
    }

    /* Halfway: release the device from sleep and its own self-test.  If the
     * chip is merely sluggish this is what brings it back; if it is absent the
     * write fails harmlessly. */
    if (attempt == 25U)
    {
      (void)imu_wr(IMU_PWR_MGMT_1, 0x80U);
      HAL_Delay(50U);
    }

    /* Boot only (no scheduler yet), so a blocking delay is acceptable here and
     * nowhere else in this file. */
    HAL_Delay(20U);
  }

  if (ok == 0U)
  {
    return (who == 0U) ? -1 : -2;
  }

  if (imu_wr(IMU_PWR_MGMT_1, 0x80U) != 0) { return -3; }   /* reset            */
  HAL_Delay(100U);
  if (imu_wr(IMU_PWR_MGMT_1, 0x01U) != 0) { return -4; }   /* clock = PLL      */
  if (imu_wr(IMU_CONFIG, 0x03U) != 0)     { return -5; }   /* DLPF             */
  if (imu_wr(IMU_GYRO_CFG, 0x00U) != 0)   { return -6; }   /* +-250 dps        */
  if (imu_wr(IMU_ACCEL_CFG, 0x00U) != 0)  { return -7; }   /* +-2 g            */
  HAL_Delay(50U);

  /* Gyro zero bias: the car must stand still during boot.  200 samples at
   * 10 ms = 2 s.  This is a boot-time calibration, not a measurement result. */
  {
    float accum = 0.0f;
    uint16_t n = 0U;

    for (uint16_t i = 0U; i < 200U; i++)
    {
      if (imu_rd(IMU_ACCEL_XOUT, raw, 14U) == 0)
      {
        accum += (float)be16(&raw[12]);
        n++;
      }
      HAL_Delay(10U);
    }
    bias_gz = (n > 0U) ? ((accum / (float)n) / IMU_GYRO_SENS) : 0.0f;
  }

  heading_deg = 0.0f;
  gz_dps = 0.0f;
  ax_g = 0.0f;
  ay_g = 0.0f;
  last_cycles = DWT->CYCCNT;
  last_ms = HAL_GetTick();

  {
    uint32_t keep = stats.recoveries;
    memset(&stats, 0, sizeof(stats));
    stats.recoveries = keep;
  }

  imu_present = 1U;
  return 0;
}

void imu_start(void)
{
  if (imu_present != 0U)
  {
    last_cycles = DWT->CYCCNT;
    last_ms = HAL_GetTick();
  }

  /* TIM6 starts even when the sensor was not found.
   *
   * It used to return here, which meant one failed init also stopped the 10 ms
   * cadence: the IMU task then timed out once per 100 ms (iw climbing at 10/s)
   * and the 1 Hz line looked like a second, unrelated fault.  The cadence is a
   * system resource, not a property of the sensor; with the sensor absent the
   * task simply wakes, gets -1 from imu_sample_once() and returns. */
  HAL_TIM_Base_Start_IT(&htim6);
}

void imu_reset_heading(void)
{
  heading_deg = 0.0f;
}

/* Interrupt context: count the tick and hand one credit to the IMU task.
 * No bus access, no parsing, no integration (CODING_STANDARDS 0.4). */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM6)
  {
    return;
  }

  stats.ticks++;

  if (app_rtos_imu_tick() != 0)
  {
    /* The previous tick had not been consumed yet: the semaphore was full.
     * That is the "IMU task fell behind" signal and must be visible. */
    stats.skipped++;
  }
}

void imu_on_error(void)
{
  /* Called from HAL_I2C_ErrorCallback, i.e. interrupt context.  The synchronous
   * transfer API already reports the failure through its return value, and the
   * bus recovery must run in task context, so all this does is count. */
  stats.errors++;
}

/* ========================================================================== */
/* task context                                                               */
/* ========================================================================== */
int imu_sample_once(void)
{
  uint8_t raw[14];
  int16_t axr;
  int16_t ayr;
  int16_t gzr;
  uint32_t now;
  float dt;

  if (imu_present == 0U)
  {
    return -1;
  }

  if (imu_rd(IMU_ACCEL_XOUT, raw, 14U) != 0)
  {
    /* i2c_bus_* already counted the error and triggers bus recovery after
     * repeated real failures.  Nothing to repair here. */
    stats.errors++;
    return -2;
  }

  axr = be16(&raw[0]);
  ayr = be16(&raw[2]);
  gzr = be16(&raw[12]);

  now = DWT->CYCCNT;
  dt = (float)(uint32_t)(now - last_cycles) / (float)SystemCoreClock;
  last_cycles = now;

  if (dt > IMU_DT_MAX_S)
  {
    dt = IMU_DT_MAX_S;
  }

  ax_g = (float)axr / IMU_ACCEL_SENS;
  ay_g = (float)ayr / IMU_ACCEL_SENS;
  gz_dps = (float)gzr / IMU_GYRO_SENS;

  heading_deg += (gz_dps - bias_gz) * dt;
  while (heading_deg > 180.0f)
  {
    heading_deg -= 360.0f;
  }
  while (heading_deg < -180.0f)
  {
    heading_deg += 360.0f;
  }

  if (stats.samples == 0U)
  {
    stats.first_sample_ms = HAL_GetTick();
  }
  stats.samples++;
  last_ms = HAL_GetTick();
  return 0;
}

void imu_get(imu_sample_t *out)
{
  if (out == NULL)
  {
    return;
  }
  out->heading_deg = heading_deg;
  out->gz_dps = gz_dps;
  out->ax_g = ax_g;
  out->ay_g = ay_g;
  out->valid = ((stats.samples > 0U) && ((HAL_GetTick() - last_ms) < 100U)) ? 1U : 0U;
}

void imu_get_stats(imu_stats_t *out)
{
  if (out == NULL)
  {
    return;
  }
  memcpy(out, &stats, sizeof(*out));
}
