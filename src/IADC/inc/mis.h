/***************************************************************************//**
 * @file mis.h
 * @brief header file for driver.
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
#ifndef MIS_H
#define MIS_H

// temperature, clock out, button
void initLetimer(void);
void letimerDelay(uint32_t msec);
void initButtonEM2(void);
void initClockOut(void);
void debounceButton(void);
float getDieTemperature(void);

// dac70501
uint16_t dac70501_init(void);
float dac70501_readRef(void);
uint16_t dac70501_setRef(uint8_t dacValueHigh, uint8_t dacValueLow);
uint16_t dac70501_setVolt(float voltValue);

// ads1220
uint32_t ads1220_init(void);
double ads1220_getAdcTemp(void);
double ads1220_getAdcDataVolt(void);
void ads1220_Calibrate(void);

// efr32bg22 adc
void rescaleIADC(uint32_t newScale);
void initIADC(void);
void bg22SaveCalData(uint32_t scale);
void bg22RestoreCalData(void);
double iadcPollSingleResult(void);
uint32_t iadcDifferentialCalibrate();

// global buffer
extern double buffer[1024];
extern double adcGainResult;
extern double adcOffsetresult;

#endif /* MIS_H */
