#include "encoder.h"
#include "board_pins.h"
#include "main.h"

#include <stddef.h>

/* First-order low-pass on the speed estimate, as in lessonProgram/Encoder.c.
 * 0.45 keeps roughly the same damping the Coding Day 2 code had. */
#define ENCODER_FILTER_ALPHA   0.45f

typedef struct
{
  TIM_TypeDef *tim;
  int8_t       forward_sign;
  int16_t      last_count;     /* previous CNT value, 16-bit wrap is fine  */
  int32_t      total;          /* signed counts since zero                */
  int16_t      rpm;            /* filtered, exposed to the control loop   */
  float        filtered;
  uint8_t      primed;         /* 1 once last_count holds a real sample   */
} encoder_ch_t;

static encoder_ch_t s_left;
static encoder_ch_t s_right;

static float clamp_f(float v, float lo, float hi)
{
  if (v < lo) { return lo; }
  if (v > hi) { return hi; }
  return v;
}

static int16_t round_f_to_s16(float v)
{
  float r = (v >= 0.0f) ? (v + 0.5f) : (v - 0.5f);

  if (r > 32767.0f) { r = 32767.0f; }
  if (r < -32768.0f) { r = -32768.0f; }
  return (int16_t)r;
}

/* ------------------------------------------------------------------------- */
/* timer configuration                                                       */
/* ------------------------------------------------------------------------- */
/* Encoder mode 3 (SMCR = 3): the counter is clocked by both TI1 and TI2 edges,
 * which is the x4 decoding that makes 11 x 30 x 4 = 1320 counts/rev.
 *
 * Input filtering (IC1F/IC2F) is NOT the lessonProgram value any more.
 * lessonProgram used 0011 (fDTS/2, N=8 ~= 190 ns), which is far too short to
 * reject anything a brushed motor injects.  Measured 2026-11-18, right encoder
 * on PA8/PA9 (TIM1):
 *
 *   right channel driven alone, 2 s        dR = +2625  (+~45 rpm)
 *   right channel + left channel, 2 s      dR = -11697 (-~265 rpm)
 *   left channel alone, 2 s                dR =      0
 *
 * The right wheel's own PWM was identical in the first two cases, so its
 * direction cannot have changed: those counts are edges that were never
 * produced by the wheel.  Changing 190 ns to 1111 (fDTS/32, N=8 ~= 3 us) keeps
 * real edges (2200 Hz per channel at 400 rpm, i.e. a 450 us period) and rejects
 * motor noise.  Raise the filter, do not "scale" the wheel in software. */
#define ENCODER_IC_FILTER  15UL   /* ICxF = 1111: fDTS/32, N=8 */

static void qep_config(TIM_TypeDef *tim)
{
  tim->CR1   = 0U;
  tim->PSC   = 0U;
  tim->ARR   = 0xFFFFU;
  tim->CCMR1 = (1UL << TIM_CCMR1_CC1S_Pos) | (1UL << TIM_CCMR1_CC2S_Pos) |
               (ENCODER_IC_FILTER << TIM_CCMR1_IC1F_Pos) |
               (ENCODER_IC_FILTER << TIM_CCMR1_IC2F_Pos);
  tim->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;
  tim->SMCR  = 3UL;
  tim->EGR   = TIM_EGR_UG;
  tim->CNT   = 0U;
  tim->CR1   = TIM_CR1_CEN;
}

static void gpio_af(GPIO_TypeDef *port, uint16_t pins, uint32_t af)
{
  GPIO_InitTypeDef g = {0};

  g.Pin       = pins;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_PULLUP;      /* open-collector hall/optical outputs */
  g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = af;
  HAL_GPIO_Init(port, &g);
}

void encoder_init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM8_CLK_ENABLE();

  gpio_af(BOARD_ENC_R_PORT,
          (uint16_t)(BOARD_ENC_R_CH_A_PIN | BOARD_ENC_R_CH_B_PIN),
          BOARD_ENC_R_AF);
  gpio_af(BOARD_ENC_L_PORT,
          (uint16_t)(BOARD_ENC_L_CH_A_PIN | BOARD_ENC_L_CH_B_PIN),
          BOARD_ENC_L_AF);

  qep_config(BOARD_ENC_R_TIMER);
  qep_config(BOARD_ENC_L_TIMER);

  s_left.tim          = BOARD_ENC_L_TIMER;
  s_left.forward_sign = BOARD_ENC_L_FORWARD_SIGN;
  s_right.tim         = BOARD_ENC_R_TIMER;
  s_right.forward_sign = BOARD_ENC_R_FORWARD_SIGN;

  encoder_zero();
}

void encoder_zero(void)
{
  s_left.last_count  = 0;
  s_left.total       = 0;
  s_left.rpm         = 0;
  s_left.filtered    = 0.0f;
  s_left.primed      = 0U;

  s_right.last_count  = 0;
  s_right.total       = 0;
  s_right.rpm         = 0;
  s_right.filtered    = 0.0f;
  s_right.primed      = 0U;
}

/* ------------------------------------------------------------------------- */
/* sampling                                                                  */
/* ------------------------------------------------------------------------- */
static int16_t read_delta(encoder_ch_t *ch, uint32_t period_ms)
{
  uint16_t now = (uint16_t)ch->tim->CNT;
  int16_t  raw_delta;
  int16_t  signed_delta;

  if (ch->primed == 0U)
  {
    /* First sample after a zero: the counter may be anywhere, so take the
     * current value as the baseline and report no movement. */
    ch->last_count = (int16_t)now;
    ch->primed     = 1U;
    return 0;
  }

  raw_delta    = (int16_t)(now - (uint16_t)ch->last_count);
  ch->last_count = (int16_t)now;
  signed_delta = (int16_t)(raw_delta * ch->forward_sign);

  ch->total += signed_delta;

  if (period_ms != 0U)
  {
    /* counts/ms -> RPM:  (delta / period_ms) * 1000 / 1320 * 60 */
    float scale = 60000.0f / ((float)period_ms * BOARD_ENC_COUNTS_PER_REV);
    float inst  = (float)signed_delta * scale;

    ch->filtered = clamp_f(ch->filtered + (ENCODER_FILTER_ALPHA *
                                           (inst - ch->filtered)),
                           -32768.0f, 32767.0f);
    ch->rpm = round_f_to_s16(ch->filtered);
  }

  return signed_delta;
}

int encoder_sample(uint32_t period_ms, int16_t *delta_left, int16_t *delta_right)
{
  int16_t dl = read_delta(&s_left, period_ms);
  int16_t dr = read_delta(&s_right, period_ms);

  if (delta_left != NULL)
  {
    *delta_left = dl;
  }
  if (delta_right != NULL)
  {
    *delta_right = dr;
  }
  return (period_ms == 0U) ? -1 : 0;
}

int16_t encoder_left_rpm(void)
{
  return s_left.rpm;
}

int16_t encoder_right_rpm(void)
{
  return s_right.rpm;
}

int32_t encoder_left_total(void)
{
  return s_left.total;
}

int32_t encoder_right_total(void)
{
  return s_right.total;
}
