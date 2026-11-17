#include "tim_cal.h"
#include "main.h"

#include <stddef.h>

/* Measurement window, expressed as a fraction of SystemCoreClock.
 * 8000 -> 125 us at 168 MHz.  The window must stay well below the 16-bit
 * counter wrap time at the fastest plausible timer clock:
 *   65536 counts / 168 MHz = 390 us, so 125 us is a 3x margin.
 * At 168 MHz the window yields ~21000 counts, i.e. ~0.005 % resolution -
 * far better than the 5 % decision threshold it feeds. */
#define TIM_CAL_WINDOW_DIV        8000U

/* Correction is attempted only when the implied update rate is further than
 * this from nominal.  Below the threshold the existing prescaler is kept, so a
 * correct timer is never perturbed. */
#define TIM_CAL_TOLERANCE_PCT     5U

static uint32_t abs_diff_u32(uint32_t a, uint32_t b)
{
  return (a > b) ? (a - b) : (b - a);
}

int hal_tim_calibrate(TIM_HandleTypeDef *htim, uint32_t nominal_hz,
                      tim_cal_result_t *out)
{
  TIM_TypeDef *tim;
  uint32_t saved_cr1;
  uint32_t saved_psc;
  uint32_t saved_arr;
  uint32_t saved_dier;
  uint32_t c0;
  uint32_t c1;
  uint32_t n0;
  uint32_t n1;
  uint32_t span_cycles;
  uint32_t counts;
  uint32_t clk;
  uint32_t arr_plus1;
  uint32_t upd_before;
  uint32_t psc_new;
  uint32_t upd_after;
  uint64_t tmp;

  if (out != NULL)
  {
    *out = (tim_cal_result_t){0};
  }
  if ((htim == NULL) || (htim->Instance == NULL) || (nominal_hz == 0U))
  {
    return -1;
  }
  tim = htim->Instance;

  saved_cr1  = tim->CR1;
  saved_psc  = tim->PSC;
  saved_arr  = tim->ARR;
  saved_dier = tim->DIER;

  /* ---- measurement pass: prescaler 1, full 16-bit range, no interrupts ---- */
  tim->CR1  &= ~TIM_CR1_CEN;
  tim->DIER  = 0U;
  tim->PSC   = 0U;
  tim->ARR   = 0xFFFFU;
  tim->EGR   = TIM_EGR_UG;
  tim->SR    = 0U;
  tim->CR1  |= TIM_CR1_CEN;

  span_cycles = (uint32_t)(SystemCoreClock / TIM_CAL_WINDOW_DIV);
  if (span_cycles == 0U)
  {
    span_cycles = 1U;
  }

  c0 = DWT->CYCCNT;
  n0 = tim->CNT;
  while ((uint32_t)(DWT->CYCCNT - c0) < span_cycles)
  {
  }
  n1 = tim->CNT;
  c1 = DWT->CYCCNT;

  /* ---- restore, forcing an update so PSC/ARR take effect immediately ----- */
  tim->CR1 &= ~TIM_CR1_CEN;
  tim->PSC  = (uint16_t)saved_psc;
  tim->ARR  = (uint16_t)saved_arr;
  tim->DIER = saved_dier;
  tim->EGR  = TIM_EGR_UG;
  tim->SR   = 0U;
  tim->CR1  = saved_cr1;

  span_cycles = (uint32_t)(c1 - c0);
  counts      = (uint32_t)(uint16_t)((uint16_t)n1 - (uint16_t)n0);

  if ((counts == 0U) || (span_cycles == 0U))
  {
    return -2;   /* timer not running, or window too short to resolve */
  }

  /* f_tim = counts / (span_cycles / SystemCoreClock) */
  tmp = ((uint64_t)counts * (uint64_t)SystemCoreClock) / (uint64_t)span_cycles;
  clk = (uint32_t)tmp;

  arr_plus1   = (uint32_t)((uint16_t)saved_arr) + 1U;
  upd_before  = (uint32_t)((uint64_t)clk /
                           ((uint64_t)((uint16_t)saved_psc + 1U) * (uint64_t)arr_plus1));
  psc_new     = (uint16_t)saved_psc;
  upd_after   = upd_before;

  if (((uint64_t)abs_diff_u32(upd_before, nominal_hz) * 100U) >
      ((uint64_t)nominal_hz * (uint64_t)TIM_CAL_TOLERANCE_PCT))
  {
    uint64_t denom = (uint64_t)nominal_hz * (uint64_t)arr_plus1;
    uint64_t want  = ((uint64_t)clk + (denom / 2U)) / denom;   /* rounded */

    if (want == 0U)
    {
      want = 1U;
    }
    if (want > 65536U)
    {
      want = 65536U;
    }
    psc_new = (uint32_t)(want - 1U);

    tim->CR1 &= ~TIM_CR1_CEN;
    tim->PSC  = (uint16_t)psc_new;
    tim->ARR  = (uint16_t)saved_arr;
    tim->EGR  = TIM_EGR_UG;
    tim->SR   = 0U;
    tim->CR1  = saved_cr1;

    /* Keep the HAL handle consistent with the hardware, so a later
     * HAL_TIM_Base_Start()/re-init does not resurrect the old value. */
    htim->Init.Prescaler = (uint16_t)psc_new;

    upd_after = (uint32_t)((uint64_t)clk /
                           ((uint64_t)(psc_new + 1U) * (uint64_t)arr_plus1));
  }

  if (out != NULL)
  {
    out->nominal_hz       = nominal_hz;
    out->measured_clk_hz  = clk;
    out->psc_before       = saved_psc;
    out->psc_after        = psc_new;
    out->update_hz_before = upd_before;
    out->update_hz_after  = upd_after;
    out->corrected        = (psc_new != saved_psc) ? 1U : 0U;
    out->ok               = 1U;
  }
  return 0;
}
