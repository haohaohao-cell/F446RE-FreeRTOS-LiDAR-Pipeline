/**
  ******************************************************************************
  * @file    stm32f4xx_hal_timebase_tim.c
  * @brief   HAL time base moved from SysTick to a hardware timer (TIM7).
  *
  *          WHY THIS FILE EXISTS
  *          --------------------
  *          As soon as FreeRTOS is started it owns SysTick: the kernel uses the
  *          SysTick exception to generate its 1 kHz tick and to trigger context
  *          switches (PendSV).  SysTick_Handler is therefore provided by
  *          cmsis_os2.c and no longer calls HAL_IncTick().
  *
  *          If HAL kept using SysTick, HAL_GetTick() would freeze the moment the
  *          scheduler starts, breaking HAL timeouts (I2C, UART, LiDAR resync).
  *          So the HAL tick is re-based on TIM7, a basic timer that needs no pin
  *          and is otherwise unused in this project.
  *
  *          TIM6 is NOT available for this purpose: it is already used by imu.c
  *          as the 100 Hz IMU sampling trigger.
  *
  *          The HAL tick interrupt is given the lowest possible priority (15)
  *          because it calls no FreeRTOS API at all - it must never preempt a
  *          kernel critical section.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOSConfig.h"

/* Private define ------------------------------------------------------------*/
#define HAL_TIMEBASE_TIM           TIM7
#define HAL_TIMEBASE_TIM_IRQn      TIM7_IRQn
#define HAL_TIMEBASE_TIM_CLK_EN()  __HAL_RCC_TIM7_CLK_ENABLE()

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim7_timebase;

/* Set to 1 if RCC_DCKCFGR.TIMPRE was found asserted - reported in the banner.
 * When TIMPRE == 1 the APBx timers run from HCLK instead of 2 x PCLKx, which
 * would silently double the HAL tick rate. */
uint32_t hal_timebase_timpre_found;

/* Number of TIM7 update interrupts actually taken - diagnostic only. */
volatile uint32_t hal_timebase_irq_count;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Configure TIM7 as the HAL time base (1 ms tick).
  * @note   Called by HAL_Init() at reset and again by HAL_RCC_ClockConfig()
  *         every time the system clock changes, so the prescaler is always
  *         recomputed from the current APB1 timer clock.
  * @param  TickPriority: ignored on purpose, the HAL tick always runs at the
  *         lowest NVIC priority (see note above).
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef clkconfig;
  uint32_t uwTimclock;
  uint32_t uwPrescalerValue;
  uint32_t pFLatency;

  (void)TickPriority;

  /* TIM7 interrupt at the HIGHEST priority (0), NOT the lowest.
   *
   * This looks backwards but is mandatory here.  BASEPRI (used by every
   * FreeRTOS critical section) masks interrupts whose NVIC priority is
   * numerically >= BASEPRI.  A SysTick based HAL tick never cares, because
   * SysTick is a core exception and core exceptions ignore BASEPRI.  A TIM
   * based tick is an ordinary peripheral IRQ and *is* masked.
   *
   * That matters in practice: this FreeRTOS port initialises
   * uxCriticalNesting to 0xaaaaaaaa and only clears it in xPortStartScheduler().
   * Every osThreadNew()/osMessageQueueNew() call before the scheduler starts
   * therefore leaves BASEPRI raised (0x50) forever.  With TIM7 at priority 15
   * the HAL tick simply stops and every HAL_GetTick() based timeout is dead.
   *
   * Priority 0 is safe because this ISR calls no FreeRTOS API - it only does
   * uwTick += uwTickFreq, a single 32-bit store. */
  HAL_NVIC_SetPriority(HAL_TIMEBASE_TIM_IRQn, 0U, 0U);
  HAL_NVIC_EnableIRQ(HAL_TIMEBASE_TIM_IRQn);
  uwTickPrio = 0U;

  HAL_TIMEBASE_TIM_CLK_EN();

  /* ---- CRITICAL: pin down the APB1 timer clock -------------------------
   * On STM32F4 the TIMx kernel clock is NOT simply 2 x PCLK1:
   *
   *   RCC_DCKCFGR.TIMPRE = 0 : APB1 prescaler 1 -> TIMxCLK = PCLK1
   *                            otherwise        -> TIMxCLK = 2 x PCLK1
   *   RCC_DCKCFGR.TIMPRE = 1 : APB1 prescaler 1,2,4 -> TIMxCLK = HCLK
   *                            otherwise           -> TIMxCLK = 4 x PCLK1
   *
   * With APB1 = HCLK/4 this codebase expects 2 x PCLK1 = 84 MHz.  If some
   * other code (or a CubeMX regeneration) leaves TIMPRE asserted, TIM7 would
   * actually run at HCLK = 168 MHz and the HAL tick would be exactly 2x too
   * fast - which silently corrupts every HAL_GetTick() based timeout and any
   * measured byte rate.  Force TIMPRE to 0 so the arithmetic below is valid. */
  if ((RCC->DCKCFGR & RCC_DCKCFGR_TIMPRE) != 0U)
  {
    hal_timebase_timpre_found = 1U;
    RCC->DCKCFGR &= ~RCC_DCKCFGR_TIMPRE;
  }

  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

  /* APB1 timer clock is 2 x PCLK1 unless APB1 prescaler is /1 */
  if (clkconfig.APB1CLKDivider == RCC_HCLK_DIV1)
  {
    uwTimclock = HAL_RCC_GetPCLK1Freq();
  }
  else
  {
    uwTimclock = 2U * HAL_RCC_GetPCLK1Freq();
  }

  /* 1 MHz counter clock -> prescaler value */
  uwPrescalerValue = (uint32_t)((uwTimclock / 1000000U) - 1U);

  htim7_timebase.Instance = HAL_TIMEBASE_TIM;
  htim7_timebase.Init.Period = (1000000U / 1000U) - 1U;   /* 1 kHz update event */
  htim7_timebase.Init.Prescaler = uwPrescalerValue;
  htim7_timebase.Init.ClockDivision = 0U;
  htim7_timebase.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7_timebase.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim7_timebase) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_TIM_Base_Start_IT(&htim7_timebase);
}

/* ---------------------------------------------------------------------------
 * One-shot calibration of the HAL tick against DWT.
 *
 * The prescaler computed in HAL_InitTick() relies on knowing the TIM7 kernel
 * clock.  Per the reference manual that is 2 x PCLK1 = 84 MHz here
 * (APB1 = HCLK/4 = 42 MHz, RCC_DCKCFGR.TIMPRE = 0), and yet TIM7 measured on
 * this board runs at 168 MHz = HCLK, i.e. the HAL tick came out exactly 2x too
 * fast while SysTick/FreeRTOS was correct.
 *
 * Rather than trust the multiplier, measure it: run TIM7 for a known number of
 * HCLK cycles (DWT->CYCCNT) and count how many update interrupts arrive.  The
 * prescaler is then corrected so the tick is exactly 1 kHz.  This is immune to
 * any APB1 timer-clock rule and to clock-tree changes.
 *
 * Must be called once, after the system clock is final and with DWT enabled,
 * and before anything depends on HAL_GetTick().
 * -------------------------------------------------------------------------- */
uint32_t hal_timebase_calib_irqs;    /* measured, for the boot banner */
uint32_t hal_timebase_calib_expect;  /* expected in the same window  */

void hal_timebase_calibrate(void)
{
  uint32_t i0;
  uint32_t c0;
  uint32_t cycles;
  uint32_t irqs;
  uint32_t expect;
  uint32_t psc;

  /* ~20 ms of HCLK cycles: expected ~20 ticks, i.e. 5 % resolution */
  const uint32_t span = SystemCoreClock / 50U;

  i0 = hal_timebase_irq_count;
  c0 = DWT->CYCCNT;
  while ((uint32_t)(DWT->CYCCNT - c0) < span)
  {
  }
  cycles = (uint32_t)(DWT->CYCCNT - c0);
  irqs   = (uint32_t)(hal_timebase_irq_count - i0);

  expect = (uint32_t)(((uint64_t)cycles * 1000U) / SystemCoreClock);
  if (expect == 0U)
  {
    expect = 1U;
  }
  hal_timebase_calib_irqs = irqs;
  hal_timebase_calib_expect = expect;

  if ((irqs == 0U) || (irqs == expect))
  {
    return;   /* already correct (or the timer is not running at all) */
  }

  /* new_prescaler = (old_prescaler + 1) * measured / expected - 1 */
  psc = (uint32_t)((((uint64_t)(TIM7->PSC + 1U)) * irqs) / expect);

  if (psc == 0U)
  {
    return;
  }
  psc -= 1U;

  /* Apply while stopped, force an update event so PSC takes effect immediately,
   * then run again. */
  TIM7->CR1 &= ~TIM_CR1_CEN;
  TIM7->PSC  = psc;
  TIM7->EGR  = TIM_EGR_UG;
  TIM7->SR   = 0U;
  TIM7->CR1 |= TIM_CR1_CEN;
}

/**
  * @brief  Suspend the HAL tick (stop the TIM7 update interrupt).
  */void HAL_SuspendTick(void)
{
  __HAL_TIM_DISABLE_IT(&htim7_timebase, TIM_IT_UPDATE);
}

/**
  * @brief  Resume the HAL tick (restart the TIM7 update interrupt).
  */
void HAL_ResumeTick(void)
{
  __HAL_TIM_ENABLE_IT(&htim7_timebase, TIM_IT_UPDATE);
}

/**
  * @brief  TIM7 global interrupt handler - increments the HAL tick.
  * @note   Deliberately calls HAL_IncTick() directly instead of going through
  *         HAL_TIM_IRQHandler()/HAL_TIM_PeriodElapsedCallback(), because that
  *         callback is already claimed by imu.c for TIM6.  This keeps the tick
  *         path short and completely independent of the RTOS.
  */
void TIM7_IRQHandler(void)
{
  hal_timebase_irq_count++;
  HAL_IncTick();
  __HAL_TIM_CLEAR_FLAG(&htim7_timebase, TIM_FLAG_UPDATE);
}
