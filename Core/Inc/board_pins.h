/* -----------------------------------------------------------------------------
 * board_pins.h - Block 4.5 Coding Day Part 2: single source of truth for the
 *                physical resource map (pins, timers, DMA, NVIC, timing).
 *
 * WHY THIS FILE EXISTS
 * --------------------
 * Part 2 merges two codebases that were written independently:
 *   - this project (LiDAR pipeline, I2C1 IMU, UART5 telemetry, TIM7 HAL tick)
 *   - lessonProgram (Coding Day 2: TIM4 PWM, TIM1/TIM8 encoders, OLED)
 * Their pin and timer assumptions overlap in places and silently disagree in
 * others (PB6/PB7 are GPIO outputs here but TIM4_CH1/2 there; the OLED there
 * is a bit-banged I2C on PA4/PB0, here it must share I2C1).  Every such fact is
 * declared once, here, with its evidence, so that a conflict is a compile-time
 * or review-time error rather than a run-time mystery.
 *
 * RULES
 *  1. No .c file may configure a pin or timer without taking the constant from
 *     this header.  No bare GPIO_PIN_x, TIMx, or priority number in driver code.
 *  2. Constants marked ASSUMPTION are unverified: they must be confirmed on
 *     hardware before they are trusted, and they are listed in the Phase A
 *     open-questions list.
 *  3. Nothing here changes behaviour on its own; it is a declaration layer.
 *     It is not yet included by any .c file in Phase A.
 *
 * PA13/PA14 (SWDIO/SWCLK) are deliberately absent from this map except as a
 * "do not touch" note: see CODING_STANDARDS.md 0.1.
 * -------------------------------------------------------------------------- */

#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#include "stm32f4xx_hal.h"

/* ========================================================================== */
/* 1. Clock tree (SystemClock_Config in main.c)                               */
/* ========================================================================== */

/* HSI 16 MHz -> PLLM 16 -> 1 MHz ; x PLLN 336 -> 336 MHz ; / PLLP 2 -> 168 MHz
 * AHB /1 -> 168 MHz, APB1 /4 -> 42 MHz, APB2 /2 -> 84 MHz. */
#define BOARD_SYSCLK_HZ       168000000UL
#define BOARD_HCLK_HZ         168000000UL
#define BOARD_PCLK1_HZ         42000000UL
#define BOARD_PCLK2_HZ         84000000UL

/* ---- timer kernel clock --------------------------------------------------
 * Reference manual rule with RCC_DCKCFGR.TIMPRE = 0 and APB1 = HCLK/4:
 *     TIM2..TIM7  <- 2 x PCLK1 =  84 MHz
 *     TIM1, TIM8  <- 2 x PCLK2 = 168 MHz
 *
 * MEASURED on this board, against DWT, during the Block 4.5 bring-up: TIM7
 * ran at exactly 2x the value above, i.e. 168 MHz = HCLK.  That is why
 * hal_timebase_calibrate() had to move PSC from 83 to 167 (see HANDOVER 5.2).
 *
 * The single hypothesis that explains the measurement for both buses is
 * "every TIMx kernel clock is HCLK": APB1 timers would be 168 MHz (measured),
 * APB2 timers would be 168 MHz (their nominal value too).  Therefore ONE
 * constant is used for all timers, and every derived prescaler below assumes it.
 *
 * ASSUMPTION (strongly evidenced, still to be re-confirmed on hardware):
 * the value below is what the boot self-check must print and agree with.
 * Do not silently "fix" a timer that misbehaves by editing this number -
 * measure it, as hal_timebase_calibrate() does for TIM7. */
#define BOARD_TIM_CLK_HZ      168000000UL

/* ========================================================================== */
/* 2. Task cadence                                                            */
/* ========================================================================== */

#define BOARD_PERIOD_FAST_MS        10U   /* 100 Hz: motor, PID, pose, IMU   */
#define BOARD_PERIOD_SLOW_MS        50U   /*  20 Hz: OLED, BT TX             */

/* TIM6 drives the 10 ms cadence.  Prescaler/period are derived, never typed:
 *   f_update = BOARD_TIM_CLK_HZ / (PSC+1) / (ARR+1) = 100 Hz
 * ARR = 99 gives a 100-count resolution and PSC a round number. */
#define BOARD_TIM6_PRESCALER   ((uint16_t)((BOARD_TIM_CLK_HZ / 100U / 100U) - 1U))
#define BOARD_TIM6_PERIOD      ((uint16_t)99U)

/* ========================================================================== */
/* 3. Motor PWM: two H-bridge channels                                        */
/* ========================================================================== */
/* lessonProgram/Board.c wrote PSC=0, ARR=799 with the comment
 * "16 MHz / 800 = 20 kHz" - that arithmetic assumes a 16 MHz timer clock and
 * is wrong for this board.  At 168 MHz it yields 210 kHz.
 * Chosen here: 168 MHz / 10 / 840 = 20 kHz exactly, with 840 duty steps. */
#define BOARD_MOTOR_PWM_HZ         20000UL
#define BOARD_MOTOR_PWM_PSC        ((uint16_t)9U)
#define BOARD_MOTOR_PWM_PERIOD     ((uint16_t)840U)   /* ARR + 1, i.e. CCR max+1 */

/* Compile-time proof that the split above really is BOARD_MOTOR_PWM_HZ. */
_Static_assert((BOARD_TIM_CLK_HZ / ((uint32_t)BOARD_MOTOR_PWM_PSC + 1UL) /
                (uint32_t)BOARD_MOTOR_PWM_PERIOD) == BOARD_MOTOR_PWM_HZ,
               "TIM4 prescaler/period do not produce the requested PWM frequency");
_Static_assert(BOARD_MOTOR_PWM_PERIOD <= 65535U, "TIM4 ARR does not fit");

/* ---------------------------------------------------------------------------
 * WHEEL-TO-CHANNEL MAPPING WAS MEASURED ON HARDWARE 2026-11-17, and it is the
 * opposite of what this file used to assume.
 *
 * Method: single-channel open-loop jogs (`l` / `r` on the console) while reading
 * the two hardware encoder totals over SWD:
 *      left channel  (TIM4/PB6/PB7) +20 %  ->  right encoder moved -4778 counts
 *      right channel (TIM2/PA0/PA1) +20 %  ->  nothing moved
 * Conclusion:
 *   1. the one motor that is fitted is on PB6/PB7 (TIM4), and it is the RIGHT
 *      wheel of the car, not the left one;
 *   2. nothing is connected to PA0/PA1 (TIM2) yet;
 *   3. that motor's polarity is reversed - a positive applied duty turned it
 *      backwards - which is exactly what BOARD_MOTOR_RIGHT_INVERT compensates.
 *
 * So the two groups are swapped below relative to the old assumption: the
 * logical LEFT wheel now owns TIM2/PA0/PA1 and the logical RIGHT wheel owns
 * TIM4/PB6/PB7.  Do NOT also swap the motor's two leads: the inversion is
 * handled here, and doing both would invert twice and make the car spin.
 *
 * The left wheel's pins remain an ASSUMPTION (nothing is wired there yet):
 * TIM2_CH1 = PA0, TIM2_CH2 = PA1.  PA0 was an ADC input in lessonProgram, so
 * confirm before connecting anything to it.
 * ------------------------------------------------------------------------- */

/* ---- LEFT wheel: TIM2_CH1/CH2 -> PA0/PA1   (ASSUMPTION, not yet wired) ---- */
#define BOARD_MOTOR_TIMER          TIM2
#define BOARD_MOTOR_TIMER_IRQn     TIM2_IRQn
#define BOARD_MOTOR_AF             GPIO_AF1_TIM2
#define BOARD_MOTOR_L_FORWARD_CH   1U   /* TIM2_CH1 -> PA0 */
#define BOARD_MOTOR_L_REVERSE_CH   2U   /* TIM2_CH2 -> PA1 */
#define BOARD_MOTOR_L_PORT         GPIOA
#define BOARD_MOTOR_L_PIN          GPIO_PIN_0
#define BOARD_MOTOR_R_PORT         GPIOA
#define BOARD_MOTOR_R_PIN          GPIO_PIN_1

/* ---- RIGHT wheel: TIM4_CH1/CH2 -> PB6/PB7  (MEASURED: motor is here) ------ */
#define BOARD_MOTOR2_TIMER         TIM4
#define BOARD_MOTOR2_TIMER_IRQn    TIM4_IRQn
#define BOARD_MOTOR2_FORWARD_CH    1U   /* TIM4_CH1 -> PB6 */
#define BOARD_MOTOR2_REVERSE_CH    2U   /* TIM4_CH2 -> PB7 */
#define BOARD_MOTOR2_FWD_PORT      GPIOB
#define BOARD_MOTOR2_FWD_PIN       GPIO_PIN_6
#define BOARD_MOTOR2_REV_PORT      GPIOB
#define BOARD_MOTOR2_REV_PIN       GPIO_PIN_7
#define BOARD_MOTOR2_AF            GPIO_AF2_TIM4

/* Right-wheel electrical polarity: MEASURED, not assumed any more.  A positive
 * duty on TIM4/PB6/PB7 turned the physical right wheel backwards, so the sign is
 * flipped here.  Keep it at 1 unless the motor wiring itself changes. */
#define BOARD_MOTOR_RIGHT_INVERT   1

/* ---------------------------------------------------------------------------
 * 3b. DRIVE FRAME: which end of the chassis is "forward"
 * ---------------------------------------------------------------------------
 * 2026-11-19: the car is run the other way round - it travels towards the end
 * that carries the DRIVEN wheels, so the caster trails behind ("rear drive
 * becomes front drive").  Nothing has to be unscrewed for that; what changes is
 * which way a positive wheel command has to turn.
 *
 * Two things flip together, and they must stay in step:
 *
 *   1. DIRECTION.  A positive command used to send the car towards the caster
 *      end; it must now send it towards the driven-wheel end.  So both wheel
 *      outputs are NEGATED (motor.c apply_outputs).
 *   2. LEFT/RIGHT.  Seen from the new travel direction, the channel wired as
 *      "left" (TIM2/PA0/PA1) is on the car's right.  So the two channels are
 *      SWAPPED - on the motor outputs AND on the encoder inputs, because the
 *      PID feedback must describe the same wheels the controller commands.
 *
 * Negating without swapping (or the reverse) gives the classically confusing
 * "it drives forwards but every turn is mirrored" - which is why both live
 * behind this single switch, with the derivation written out in motor.c.
 *
 * WHAT DOES NOT CHANGE: POSE_IMU_TO_NAV_DEG (pose.c).  Heading is measured from
 * the gyro's zero at power-up, and the operator places the car with its travel
 * direction along +y; rotating the chassis about the VERTICAL axis leaves the
 * gyro's yaw measurement unchanged.
 *
 *   0 = original layout (travel towards the caster).  Measured, demo-proven.
 *   1 = reversed layout  (travel towards the driven wheels, caster trails).
 *       COMPILES, and the signs are derived in motor.c, but it has NEVER been
 *       driven on the floor - validate wheels-up before trusting it.          */
#define BOARD_DRIVE_FRAME_REVERSED 0

/* lessonProgram MOTOR_DEFAULT_* values are kept as the TARGET clamp (they limit
 * the requested speed, which is a different thing from the duty).
 *
 * The PWM limit, however, had to be raised: measured 2026-11-17 with the car on
 * the floor, a 20 % duty (168/840 counts) produced only 17 rpm, so reaching the
 * commanded 59 rpm needs ~70 % duty.  The old 325 count (38.7 %) ceiling made
 * the requested speed physically unreachable and the car crawled at 28 % of the
 * target.  Full scale is allowed now; the PID and its integral clamp are what
 * keep it sane, and a stalled wheel is protected by the 10 ms loop, not by this
 * number. */
#define BOARD_MOTOR_TARGET_LIMIT      325
#define BOARD_MOTOR_PWM_MAX_LIMIT     840   /* = the full PWM period */

/* ========================================================================== */
/* 4. Wheel encoders: TIM1 (right) and TIM8 (left), hardware x4 decoding     */
/* ========================================================================== */

/* 11 pole pairs x 30 : 1 gearbox x 4 edges = 1320 counts per wheel revolution
 * (from lessonProgram/Encoder.h).  Direction signs come from the same file:
 * the right channel is mounted mirrored. */
#define BOARD_ENC_COUNTS_PER_REV      1320.0f
#define BOARD_ENC_L_TIMER             TIM8
#define BOARD_ENC_L_PORT              GPIOC
#define BOARD_ENC_L_CH_A_PIN          GPIO_PIN_6
#define BOARD_ENC_L_CH_B_PIN          GPIO_PIN_7
#define BOARD_ENC_L_AF                GPIO_AF3_TIM8
#define BOARD_ENC_L_FORWARD_SIGN      (+1)
#define BOARD_ENC_R_TIMER             TIM1
#define BOARD_ENC_R_PORT              GPIOA
#define BOARD_ENC_R_CH_A_PIN          GPIO_PIN_8
#define BOARD_ENC_R_CH_B_PIN          GPIO_PIN_9
#define BOARD_ENC_R_AF                GPIO_AF1_TIM1
#define BOARD_ENC_R_FORWARD_SIGN      (-1)

/* ASSUMPTION (unverified): wheel diameter of the kit chassis.  Every metric
 * conversion (counts -> mm) depends on it, so odometry is meaningless until
 * this is measured.  Alternative to measuring: push the car 1.000 m by hand
 * and divide.  Kept as a named constant so it is impossible to miss. */
#define BOARD_WHEEL_DIAMETER_MM       65.0f
#define BOARD_PI_F                    3.14159265f
#define BOARD_MM_PER_COUNT \
  ((BOARD_PI_F * BOARD_WHEEL_DIAMETER_MM) / BOARD_ENC_COUNTS_PER_REV)

/* Track width is needed by the differential-drive pose update.  ASSUMPTION. */
#define BOARD_TRACK_WIDTH_MM          130.0f

/* Wheel speed sanity limit used to bound the 10 ms integration step: at
 * 1320 counts/rev and the maximum speed the chassis reaches, a 10 ms window
 * cannot overflow the 16-bit CNT delta (would need > 240 rev/s). */
#define BOARD_ENC_MAX_COUNTS_PER_PERIOD  16000

/* ========================================================================== */
/* 5. Timers: one row per timer, no overlaps                                  */
/* ========================================================================== */

#define BOARD_TIM_HAL_TICK         TIM7   /* HAL 1 ms tick, DO NOT TOUCH (HANDOVER 5.1) */
#define BOARD_TIM_FAST_TICK        TIM6   /* 10 ms cadence -> IMU semaphore              */
#define BOARD_TIM_MOTOR_L          TIM2   /* left  wheel PWM, 20 kHz (see section 3)     */
#define BOARD_TIM_MOTOR_R          TIM4   /* right wheel PWM, 20 kHz - motor fitted here */
#define BOARD_TIM_MICROS           TIM5   /* free-running 1 MHz timestamp (32-bit)       */
#define BOARD_TIM_MICROS_PSC       ((uint16_t)((BOARD_TIM_CLK_HZ / 1000000UL) - 1U))

/* Spare, declared so nobody grabs them silently:
 *   TIM3  - PA6/PA7: input capture in lessonProgram Track.c.  PA7 is wired to
 *           the MC520 A phase on that board, so it is NOT used for PWM here.
 *           Not ported in Part 2 (line following is not a Part 2 item).
 *   TIM9..TIM14 - unused.
 *
 * TIM5 replaced TIM2 as the microsecond counter when TIM2 was taken for the
 * right-wheel PWM.  TIM5 is 32-bit and needs no pins. */

_Static_assert(BOARD_TIM_MICROS_PSC <= 65535U, "TIM5 prescaler does not fit");

/* ========================================================================== */
/* 6. Serial ports                                                            */
/* ========================================================================== */

/* USART2: debug console, 115200, PA2/PA3 -> ST-Link VCP (COM8). Polled TX,
 *         single-byte RX polling for the 's'/'g' console commands. */
#define BOARD_DBG_UART             USART2
#define BOARD_DBG_BAUD             115200UL
#define BOARD_DBG_TX_PORT          GPIOA
#define BOARD_DBG_TX_PIN           GPIO_PIN_2
#define BOARD_DBG_RX_PORT          GPIOA
#define BOARD_DBG_RX_PIN           GPIO_PIN_3
#define BOARD_DBG_AF               GPIO_AF7_USART2

/* USART3: C1 LiDAR, 460800, PC10/PC11, circular RX on DMA1_Stream1. */
#define BOARD_LIDAR_UART           USART3
#define BOARD_LIDAR_BAUD           460800UL
#define BOARD_LIDAR_TX_PORT        GPIOC
#define BOARD_LIDAR_TX_PIN         GPIO_PIN_10
#define BOARD_LIDAR_RX_PORT        GPIOC
#define BOARD_LIDAR_RX_PIN         GPIO_PIN_11
#define BOARD_LIDAR_AF             GPIO_AF7_USART3
#define BOARD_LIDAR_DMA            DMA1_Stream1
#define BOARD_LIDAR_DMA_CHANNEL    DMA_CHANNEL_4
#define BOARD_LIDAR_DMA_IRQn       DMA1_Stream1_IRQn

/* UART5: HC-04 Bluetooth, 230400, PC12 (TX) / PD2 (RX).
 * TX already uses DMA1_Stream7 for telemetry.
 * RX is NEW in Part 2: DMA1_Stream0, channel 4 - the only free UART5_RX
 * mapping on DMA1 and not used by anything else in this project. */
#define BOARD_BT_UART              UART5
#define BOARD_BT_BAUD              230400UL
#define BOARD_BT_TX_PORT           GPIOC
#define BOARD_BT_TX_PIN            GPIO_PIN_12
#define BOARD_BT_RX_PORT           GPIOD
#define BOARD_BT_RX_PIN            GPIO_PIN_2
#define BOARD_BT_AF                GPIO_AF8_UART5
#define BOARD_BT_TX_DMA            DMA1_Stream7
#define BOARD_BT_TX_DMA_CHANNEL    DMA_CHANNEL_4
#define BOARD_BT_TX_DMA_IRQn       DMA1_Stream7_IRQn
#define BOARD_BT_RX_DMA            DMA1_Stream0
#define BOARD_BT_RX_DMA_CHANNEL    DMA_CHANNEL_4
#define BOARD_BT_RX_DMA_IRQn       DMA1_Stream0_IRQn
#define BOARD_BT_RX_DMA_SIZE       64U    /* >= 2 x longest command frame */

/* Windows SPP tops out at 115200: the 230400 above is nominal only and does
 * not throttle the link (HANDOVER 5, last note). */
#define BOARD_BT_SPP_MAX_BAUD      115200UL

/* ========================================================================== */
/* 7. I2C1: MPU-6500 and SSD1306 share one bus (this is why a mutex exists)   */
/* ========================================================================== */

#define BOARD_I2C                  I2C1
#define BOARD_I2C_SCL_PORT         GPIOB
#define BOARD_I2C_SCL_PIN          GPIO_PIN_8
#define BOARD_I2C_SDA_PORT         GPIOB
#define BOARD_I2C_SDA_PIN          GPIO_PIN_9
#define BOARD_I2C_AF               GPIO_AF4_I2C1
#define BOARD_I2C_EV_IRQn          I2C1_EV_IRQn
#define BOARD_I2C_ER_IRQn          I2C1_ER_IRQn

/* Targets on the bus.  MPU-6500 accepts 0x70/0x71/0x73 in WHO_AM_I at 0x75;
 * the SSD1306 answers at 7-bit 0x3C (lessonProgram/Oled.h). */
#define BOARD_IMU_I2C_ADDR_7BIT    0x68U
#define BOARD_OLED_I2C_ADDR_7BIT   0x3CU

/* Bus speed.  100 kHz keeps the existing, verified IMU timing but makes a
 * full 1 KiB OLED frame take ~12 ms; Part 2 therefore refreshes one page per
 * 50 ms slot (128 B ~ 1.5 ms) - see CODING_STANDARDS 3.4.
 * Raising to 400 kHz is possible (both devices support it) but changes the
 * IMU timing that was validated on hardware, so it is left as a later step. */
#define BOARD_I2C_SPEED_HZ         100000UL
#define BOARD_I2C_LOCK_TIMEOUT_MS  10U

/* ========================================================================== */
/* 8. NVIC priorities: the ONLY place these numbers are written               */
/* ========================================================================== */

/* Cortex-M: a larger number is a logically lower priority.
 * Hard rule (CODING_STANDARDS 4.1): any ISR that touches a CMSIS-RTOS2 object
 * must have a numerically >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY.
 * The value is repeated here rather than included from FreeRTOSConfig.h so
 * that this header stays independent of the kernel headers. */
#define BOARD_MAX_SYSCALL_PRIO     5U

#define BOARD_PRIO_HAL_TICK        0U   /* TIM7: must stay 0 (HANDOVER 5.1)        */
#define BOARD_PRIO_LIDAR           5U   /* USART3 + DMA1_Stream1: post Q_Lidar     */
#define BOARD_PRIO_FAST_TICK       5U   /* TIM6: osSemaphoreRelease                */
#define BOARD_PRIO_BT_RX           5U   /* UART5 + DMA1_Stream0: signal BT RX task */
#define BOARD_PRIO_I2C             6U   /* I2C1 EV/ER: no RTOS call, kept below 5  */
#define BOARD_PRIO_BT_TX_DMA       6U   /* DMA1_Stream7: no RTOS call              */
#define BOARD_PRIO_CONSOLE         7U   /* USART2: polled only                     */
#define BOARD_PRIO_KERNEL          15U  /* SysTick / PendSV                        */

/* The HAL tick must be the highest-priority peripheral IRQ: BASEPRI raised by
 * the kernel would otherwise mask it and HAL_GetTick() would stop (HANDOVER 5.1). */
_Static_assert(BOARD_PRIO_HAL_TICK == 0U, "TIM7 must stay at priority 0");
_Static_assert(BOARD_PRIO_LIDAR    >= BOARD_MAX_SYSCALL_PRIO, "LiDAR ISR calls RTOS API");
_Static_assert(BOARD_PRIO_FAST_TICK >= BOARD_MAX_SYSCALL_PRIO, "TIM6 ISR calls RTOS API");
_Static_assert(BOARD_PRIO_BT_RX    >= BOARD_MAX_SYSCALL_PRIO, "BT RX ISR calls RTOS API");
_Static_assert(BOARD_PRIO_I2C      >= BOARD_MAX_SYSCALL_PRIO, "I2C1 must not preempt a critical section");
_Static_assert(BOARD_PRIO_BT_TX_DMA >= BOARD_MAX_SYSCALL_PRIO, "DMA TX must not preempt a critical section");
_Static_assert(BOARD_PRIO_KERNEL   == 15U, "SysTick/PendSV must stay lowest");

/* ========================================================================== */
/* 9. Reserved / do-not-touch                                                */
/* ========================================================================== */

/* PA13/PA14  SWDIO/SWCLK.  Never reconfigure (CODING_STANDARDS 0.1).
 * PA5        LD2, optional status LED (Board_SetLed in lessonProgram).
 * PB1,PB13/14/15  lessonProgram buttons + EXTI.  NOT brought up in Part 2:
 *            no Part 2 acceptance item uses them, and every extra EXTI IRQ
 *            is another chance to violate the priority rule.
 * PA0        ADC1_IN0 in lessonProgram (battery/pot).  Not in Part 2.
 * PB10       buzzer in lessonProgram.  Not in Part 2.
 * PA4 / PB0  lessonProgram's bit-banged OLED I2C.  Freed once the OLED is
 *            moved onto I2C1; must NOT be initialised as GPIO any more.
 * PA7        TIM3_CH2 input capture (Track.c).  Free in Part 2.
 *
 * NOTE: the encoder pins are PA8/PA9 (TIM1) while the IMU bus is PB8/PB9
 * (I2C1).  Same numbers, different ports - a swap here is silent and total. */

#endif /* BOARD_PINS_H */
