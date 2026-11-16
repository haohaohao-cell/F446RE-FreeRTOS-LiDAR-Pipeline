#include "imu.h"
#include "main.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim6;

#define IMU_ADDR       (0x68U << 1)
#define IMU_WHO_AM_I   0x75U
#define IMU_PWR_MGMT_1 0x6BU
#define IMU_CONFIG     0x1AU
#define IMU_GYRO_CFG   0x1BU
#define IMU_ACCEL_CFG  0x1CU
#define IMU_ACCEL_XOUT 0x3BU
#define IMU_GYRO_SENS  131.0f
#define IMU_ACCEL_SENS 16384.0f

static uint8_t raw[14];
static volatile uint8_t busy;
static volatile uint32_t busy_since;
static uint32_t last_cycles;
static uint32_t last_ms;
static float bias_gz;
static float heading_deg;
static float gz_dps;
static float ax_g;
static float ay_g;
static imu_stats_t stats;
static uint8_t imu_present;
static volatile uint8_t need_recover;
static uint8_t busy_ticks;
uint32_t imu_diag_status;
uint32_t imu_diag_err;
uint32_t imu_diag_who;

static void delay_us(uint32_t us)
{
  uint32_t t0 = DWT->CYCCNT;
  uint32_t n = us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - t0) < n)
  {
  }
}

static void i2c_bus_recover(void)
{
  GPIO_InitTypeDef g = {0};

  HAL_I2C_DeInit(&hi2c1);
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
  g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  g.Mode = GPIO_MODE_OUTPUT_OD;
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &g);
  delay_us(10U);

  for (uint8_t i = 0U; i < 9U; i++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    delay_us(10U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    delay_us(10U);
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
  delay_us(10U);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
  delay_us(10U);

  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();
  hi2c1.Init.ClockSpeed = 50000U;
  HAL_I2C_Init(&hi2c1);
  g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  g.Mode = GPIO_MODE_AF_OD;
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &g);
  /* Keep the I2C ISRs at the same priority the RTOS layer defines (6).  These
   * ISRs never call FreeRTOS API, but they must NOT sit above
   * configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5): an interrupt that can
   * preempt a kernel critical section must not be able to touch RTOS objects. */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 6U, 0U);
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 6U, 0U);
  stats.recoveries++;
}

void imu_poll(void)
{
  if ((need_recover != 0U) && (imu_present != 0U))
  {
    i2c_bus_recover();
    busy = 0U;
    busy_ticks = 0U;
    need_recover = 0U;
  }
}

static HAL_StatusTypeDef imu_wr(uint8_t reg, uint8_t val)
{
  return HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1U, 100U);
}

static HAL_StatusTypeDef imu_rd(uint8_t reg, uint8_t *buf, uint16_t len)
{
  return HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100U);
}

static int16_t be16(const uint8_t *p)
{
  return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

int imu_init(void)
{
  uint8_t who = 0U;
  uint8_t ok = 0U;

  imu_present = 0U;
  for (uint8_t attempt = 0U; attempt < 5U; attempt++)
  {
    HAL_StatusTypeDef st;
    i2c_bus_recover();
    who = 0U;
    st = imu_rd(IMU_WHO_AM_I, &who, 1U);
    imu_diag_status = (uint32_t)st;
    imu_diag_err = hi2c1.ErrorCode;
    imu_diag_who = who;
    if ((st == HAL_OK) &&
        ((who == 0x70U) || (who == 0x71U) || (who == 0x73U)))
    {
      ok = 1U;
      break;
    }
    HAL_Delay(20U);
  }
  if (ok == 0U)
  {
    return (who == 0U) ? -1 : -2;
  }

  if (imu_wr(IMU_PWR_MGMT_1, 0x80U) != HAL_OK)
  {
    return -3;
  }
  HAL_Delay(100U);
  if (imu_wr(IMU_PWR_MGMT_1, 0x01U) != HAL_OK)
  {
    return -4;
  }
  if (imu_wr(IMU_CONFIG, 0x03U) != HAL_OK)
  {
    return -5;
  }
  if (imu_wr(IMU_GYRO_CFG, 0x00U) != HAL_OK)
  {
    return -6;
  }
  if (imu_wr(IMU_ACCEL_CFG, 0x00U) != HAL_OK)
  {
    return -7;
  }
  HAL_Delay(50U);

  float accum = 0.0f;
  uint16_t n = 0U;
  for (uint16_t i = 0U; i < 200U; i++)
  {
    if (imu_rd(IMU_ACCEL_XOUT, raw, 14U) == HAL_OK)
    {
      accum += (float)be16(&raw[12]);
      n++;
    }
    HAL_Delay(10U);
  }
  bias_gz = (n > 0U) ? ((accum / (float)n) / IMU_GYRO_SENS) : 0.0f;

  heading_deg = 0.0f;
  gz_dps = 0.0f;
  ax_g = 0.0f;
  ay_g = 0.0f;
  busy = 0U;
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
  if (imu_present == 0U)
  {
    return;
  }
  last_cycles = DWT->CYCCNT;
  last_ms = HAL_GetTick();
  HAL_TIM_Base_Start_IT(&htim6);
}

void imu_reset_heading(void)
{
  heading_deg = 0.0f;
}

void imu_on_error(void)
{
  busy = 0U;
  stats.errors++;
  need_recover = 1U;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM6)
  {
    return;
  }

  stats.ticks++;

  if (need_recover != 0U)
  {
    stats.skipped++;
    return;
  }

  if (busy != 0U)
  {
    busy_ticks++;
    if (busy_ticks > 2U)
    {
      busy = 0U;
      busy_ticks = 0U;
      stats.errors++;
      need_recover = 1U;
    }
    else
    {
      stats.skipped++;
    }
    return;
  }

  if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET)
  {
    stats.errors++;
    need_recover = 1U;
    return;
  }
  busy = 1U;
  busy_ticks = 0U;
  busy_since = HAL_GetTick();
  if (HAL_I2C_Mem_Read_IT(&hi2c1, IMU_ADDR, IMU_ACCEL_XOUT, I2C_MEMADD_SIZE_8BIT, raw, 14U) != HAL_OK)
  {
    busy = 0U;
    stats.errors++;
    need_recover = 1U;
  }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C1)
  {
    return;
  }

  int16_t axr = be16(&raw[0]);
  int16_t ayr = be16(&raw[2]);
  int16_t gzr = be16(&raw[12]);

  uint32_t now = DWT->CYCCNT;
  float dt = (float)(now - last_cycles) / (float)SystemCoreClock;
  last_cycles = now;
  if (dt > 0.2f)
  {
    dt = 0.2f;
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
  busy = 0U;
}

void imu_get(imu_sample_t *out)
{
  out->heading_deg = heading_deg;
  out->gz_dps = gz_dps;
  out->ax_g = ax_g;
  out->ay_g = ay_g;
  out->valid = ((stats.samples > 0U) && ((HAL_GetTick() - last_ms) < 100U)) ? 1U : 0U;
}

void imu_get_stats(imu_stats_t *out)
{
  memcpy(out, &stats, sizeof(stats));
}
