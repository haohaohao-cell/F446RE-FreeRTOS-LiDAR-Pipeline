/* -----------------------------------------------------------------------------
 * tim_cal.h - measure a timer's kernel clock with DWT and fix its prescaler.
 *
 * WHY: this board already proved that the APB1 timer clock does not follow the
 * reference manual - TIM7 measured exactly 2x the documented 2 x PCLK1 value,
 * which is why hal_timebase_calibrate() had to move PSC from 83 to 167
 * (HANDOVER 5.2).  TIM6 (10 ms cadence) and TIM4/TIM2 (20 kHz PWM) are on the
 * same bus, so hard-coding a prescaler for them would bury the same trap again.
 *
 * Same idea as the TIM7 fix, but generalised: instead of counting interrupts
 * (which would need an ISR for the PWM timers), read the free-running counter
 * across a DWT-measured window.  No interrupt is involved, so this works for a
 * PWM generator whose update interrupt is not enabled.
 * -------------------------------------------------------------------------- */

#ifndef TIM_CAL_H
#define TIM_CAL_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct
{
  uint32_t nominal_hz;        /* update rate that was asked for            */
  uint32_t measured_clk_hz;   /* measured TIMx kernel clock                */
  uint32_t psc_before;        /* prescaler that was programmed             */
  uint32_t psc_after;         /* prescaler in effect afterwards            */
  uint32_t update_hz_before;  /* implied update rate with psc_before       */
  uint32_t update_hz_after;   /* implied update rate with psc_after        */
  uint8_t  corrected;         /* 1 = prescaler was changed                 */
  uint8_t  ok;                /* 1 = measurement was valid                 */
} tim_cal_result_t;

/* Measure the timer clock behind *htim, and correct its prescaler if the
 * resulting update rate is more than 5 % away from nominal_hz.
 *
 * The timer's ARR (htim->Init.Period) is preserved and used as the period
 * divisor, so the caller must have initialised the handle first.
 *
 * Preconditions:
 *   - DWT cycle counter enabled (dwt_init() already does this in app_init()).
 *   - For a PWM timer, all compare outputs inactive (CCR = 0), because the
 *     measurement temporarily raises ARR to 0xFFFF.
 *   - Interrupts need not be masked; the update interrupt is disabled for the
 *     duration, so no ISR fires from the measurement itself.
 *
 * Returns 0 on success, negative on failure (timer not initialised, or the
 * measurement window produced no counts).  *out may be NULL.
 *
 * MUST be called before osKernelStart(): it rewrites hardware registers that
 * the first scheduler tick would otherwise race with. */
int hal_tim_calibrate(TIM_HandleTypeDef *htim, uint32_t nominal_hz,
                      tim_cal_result_t *out);

#endif /* TIM_CAL_H */
