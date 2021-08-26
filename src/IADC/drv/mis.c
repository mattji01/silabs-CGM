/***************************************************************************//**
 * @file emuTemperature.c
 * @brief emu, clock and other.
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *******************************************************************************
 *
 * # Evaluation Quality
 * This code has been minimally tested to ensure that it builds and is suitable
 * as a demonstration for evaluation purposes only. This code will be maintained
 * at the sole discretion of Silicon Labs.
 ******************************************************************************/

#include "em_emu.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_letimer.h"
#include <mis.h>

#ifdef __cplusplus
extern "C" {
#endif

// button
#define EM4WU_PORT          gpioPortC
#define EM4WU_PIN           5
#define EM4WU_EM4WUEN_NUM   (7)                       // PC5 is EM4WUEN pin 7
#define EM4WU_EM4WUEN_MASK  (1 << EM4WU_EM4WUEN_NUM)

// clock output
#define CMUCLOCKOUT_PORT    gpioPortC
#define CMUCLOCKOUT_PIN     3
#define SLEW_RATE           7

volatile uint8_t button_EmMode = 0;

#if 0
/**************************************************************************//**
 * @brief GPIO Interrupt handler for even pins.
 * @comment
 *    use platform->driver->button->simple button component (gpiointerrupt)
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  /* Get and clear all pending GPIO interrupts */
  uint32_t interruptMask = GPIO_IntGet();
  GPIO_IntClear(interruptMask);

  /* Check if button 1 was pressed */
  if (interruptMask & ((1 << EM4WU_PIN) | GPIO_IEN_EM4WUIEN7))
  {
    /* add your code here */
    ;
  }
}
#endif

/**************************************************************************//**
 * @brief
 *    init letimer
 * @param[in]
 *    none
 * @return
 *    none
 * @comment
 *    24-bit down count for delay
 *****************************************************************************/
void initLetimer(void)
{
  LETIMER_Init_TypeDef letimerInit = LETIMER_INIT_DEFAULT;

  /* use LFRCO */
  CMU_ClockSelectSet(cmuClock_EM23GRPACLK, cmuSelect_LFRCO);
  /* Enable clock for letimer */
  CMU_ClockEnable(cmuClock_LETIMER0, true);

  /* field initialize */
  letimerInit.enable = false;
  letimerInit.debugRun = false;

  /* Initialize LETIMER */
  LETIMER_Init(LETIMER0, &letimerInit);
  LETIMER_CounterSet(LETIMER0, 0);
}

/**************************************************************************//**
 * @brief
 *    Delay with letimer
 * @param[in]
 *    usec, with 32.768 clock frequency
 *    unit = 1/32.768 = 30.57 uS
 * @return
 *    none
 * @comment
 *    for delay， em0 only
 *****************************************************************************/
void letimerDelay(uint32_t msec)
{
  /* enable and wait CNT equal */
  uint32_t totalTicks = 0xFFFFFF - msec * 32.768;
  LETIMER_Enable(LETIMER0, true);
  while (LETIMER_CounterGet(LETIMER0) >= totalTicks) ;

  /* reset CNT and disable */
  LETIMER_CounterSet(LETIMER0, 0);
  LETIMER_Enable(LETIMER0, false);
}

/**************************************************************************//**
 * @brief
 *    EMU temperature read back
 * @param[in]
 *    none
 * @return
 *    bg22 die temperature in Celcius unit
 *****************************************************************************/
float getDieTemperature(void)
{
  uint32_t diTemp = 0, diEmu = 0;
  float EMUTempC = 0.0, tempDegC = 0.0, offsetCorrection = 0.0;

  /* Read EMU temperature sensor calibration data from device info page */
  diTemp = (DEVINFO->CALTEMP & _DEVINFO_CALTEMP_TEMP_MASK)
           >> _DEVINFO_CALTEMP_TEMP_SHIFT;
  diEmu =  (DEVINFO->EMUTEMP & _DEVINFO_EMUTEMP_EMUTEMPROOM_MASK)
           >> _DEVINFO_EMUTEMP_EMUTEMPROOM_SHIFT;

  /* EMU temperature raw data in Celcius unit */
  EMUTempC = EMU_TemperatureGet();

  /* compensate the data */
  offsetCorrection = diEmu - 273.15 - diTemp;
  tempDegC = EMUTempC - offsetCorrection;

  return tempDegC;
}

/**************************************************************************//**
 * @brief
 *    clock output
 * @param[in]
 *    none
 * @return
 *    none
 *****************************************************************************/
void initClockOut(void)
{
  /* Enable clock */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Set chosen port pin as output */
  GPIO_PinModeSet(CMUCLOCKOUT_PORT, CMUCLOCKOUT_PIN, gpioModePushPull, 0);
  /* Set slew rate (drive strength) so there is no ringing */
  GPIO_SlewrateSet(CMUCLOCKOUT_PORT, SLEW_RATE, SLEW_RATE);

  /* Enable Low Frequency RC Oscillator (LFRCO) and
  wait until it is stable */
  CMU_OscillatorEnable(cmuOsc_HFXO, true, true);

  /* Force output */

  /* to CLKOUTSEL2 */
  CMU_ClkOutPinConfig(2, cmuSelect_HFXO, 0, CMUCLOCKOUT_PORT, CMUCLOCKOUT_PIN);
  /* max clkDiv is 32 , only applicable to EXPCLK */
  // CMU_ClkOutPinConfig(1, cmuSelect_EXPCLK,
  //                     31, CMUCLOCKOUT_PORT, CMUCLOCKOUT_PIN);
  CMU->EXPORTCLKCTRL = (CMU->EXPORTCLKCTRL & ~_CMU_EXPORTCLKCTRL_PRESC_MASK)
                       | (0x7<<_CMU_EXPORTCLKCTRL_PRESC_SHIFT) ;
}

/**************************************************************************//**
 * @brief
 *    gpio (button) init for em0/1
 * @param[in]
 *    none
 * @return
 *    none
 *****************************************************************************/
void initButtonEM1(void)
{
  /* Enable clock */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure Button PC5 as input and enable interrupt
   * internal pull-up */
  GPIO_PinModeSet(EM4WU_PORT, EM4WU_PIN, gpioModeInputPullFilter, 1);

  /* falling edge interrupt */
  GPIO_ExtIntConfig(EM4WU_PORT,
                    EM4WU_PIN,
                    EM4WU_PIN,
                    false,
                    true,
                    true);

  /* Enable ODD interrupt to catch button press */
  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);

  /* enter EM1 */
  button_EmMode = 1;
  EMU_EnterEM1();
}

/**************************************************************************//**
 * @brief
 *    gpio (button) init for em2
 * @param[in]
 *    none
 * @return
 *    none
 *****************************************************************************/
void initButtonEM2(void)
{
  /* Enable clock */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure Button PC5 as input and enable interrupt */
  GPIO_PinModeSet(EM4WU_PORT, EM4WU_PIN, gpioModeInputPullFilter, 1);

  /* falling edge interrupt */
  GPIO_EM4EnablePinWakeup(GPIO_IEN_EM4WUIEN7, 0 << _GPIO_IEN_EM4WUIEN7_SHIFT);
  GPIO->IEN = 1 << _GPIO_IEN_EM4WUIEN7_SHIFT;
  GPIO->EM4WUEN = 1 << _GPIO_IEN_EM4WUIEN7_SHIFT;

  /* Enable EVEN interrupt to catch button press */
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);

  /* Enable ODD interrupt to catch button press */
  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);

  /* enter EM2 */
  button_EmMode = 2;
  EMU_EnterEM2(false);
}

/**************************************************************************//**
 * @brief
 *    gpio (button) init for em4
 * @param[in]
 *    none
 * @return
 *    none
 *****************************************************************************/
void initButtonEM4(void)
{
  /* Use default settings for EM4 */
  EMU_EM4Init_TypeDef em4Init = EMU_EM4INIT_DEFAULT;

  /* Release pin state */
  //EMU_UnlatchPinRetention();  // removed???

  /* Enable clock */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure Button PC5 as input and EM4 wake-up source */
  /* internal pull high */
  GPIO_PinModeSet(EM4WU_PORT, EM4WU_PIN, gpioModeInputPullFilter, 1);

  /* Enable GPIO pin wake-up from EM4; PC5 is EM4WUEN pin 7 */
  GPIO_EM4EnablePinWakeup(GPIO_IEN_EM4WUIEN7, 0 << _GPIO_IEN_EM4WUIEN7_SHIFT);

  /* Enable Pin Retention through EM4 and wakeup */
  em4Init.pinRetentionMode = emuPinRetentionLatch;
  /* Initialize EM mode 4 */
  EMU_EM4Init(&em4Init);

  /* Enter EM4 */
  //EMU_EnterEM4();
}

/**************************************************************************//**
 * @brief
 *    button de-bounce suppressing
 * @return
 *    none
 *****************************************************************************/
void debounceButton(void)
{
  /* Configure Button PB1 as input and EM4 wake-up source */
  GPIO_PinModeSet(EM4WU_PORT, EM4WU_PIN, gpioModeInputPullFilter, 1);

  for (int i = 0; i < 100; i++)
  {
    letimerDelay(1000);
  }
}

#ifdef __cplusplus
}
#endif
