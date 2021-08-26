/***************************************************************************//**
 * @file efr32bg22_adc.c
 * @brief EFR32BG22 adc routine for calibration and read.
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

#include "em_cmu.h"
#include "em_iadc.h"
#include "em_msc.h"
#include <mis.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/
// Set CLK_ADC to 10MHz (this corresponds to a sample rate of 77K with OSR = 32)
// CLK_SRC_ADC; largest division is by 4
#define CLK_SRC_ADC_FREQ        20000000
// CLK_ADC; IADC_SCHEDx PRESCALE has 10 valid bits
#define CLK_ADC_FREQ            10000000

// adc input channel/pins
#define ADC_POS_INPUT             iadcPosInputPortCPin0
#define ADC_NEG_INPUT             iadcNegInputPortCPin1
// When changing GPIO port/pins above, make sure to change xBUSALLOC macro's
// accordingly.
#define IADC_INPUT_0_BUS          CDBUSALLOC
#define IADC_INPUT_0_BUSALLOC     GPIO_CDBUSALLOC_CDEVEN0_ADC0
#define IADC_INPUT_1_BUS          CDBUSALLOC
#define IADC_INPUT_1_BUSALLOC     GPIO_CDBUSALLOC_CDODD0_ADC0

// for calibration
// How many samples to capture and average
#define NUM_SAMPLES                 1024
#define IADC_SCALE_OFFSET_MAX_NEG   0x00020000UL // 18-bit 2's compliment
#define IADC_SCALE_OFFSET_ZERO      0x00000000UL

/*******************************************************************************
 ***************************   GLOBAL VARIABLES   ******************************
 ******************************************************************************/
// Stores latest ADC sample and converts to volts
static volatile IADC_Result_t sample;
static volatile double singleResult;

// buffer to save adc result
double buffer[1024];
volatile uint32_t loop = 0;
double adcGainResult;
double adcOffsetresult;

/**************************************************************************//**
 * @brief  ADC Handler, never be used/called
 * This code don't use interrupt, instead it use polling mode
 *****************************************************************************/
void IADC_IRQHandler(void)
{
  /* Read data from the FIFO, 16-bit result */
  sample = IADC_pullSingleFifoResult(IADC0);

  /* For differential the result range is -Vref to +Vref, i.e., 16 bits for the
  ** conversion value.
  ** */
  singleResult = sample.data * 1.25 *2 / 0xFFFF;

  /* clear int flag */
  IADC_clearInt(IADC0, IADC_IF_SINGLEFIFODVL);

  /* store result to buffer */
  if (loop < 1024)
  {
    buffer[loop] = singleResult;
    loop++;
    letimerDelay(2);
  }
  else
  {
    ;
  }
}

/**************************************************************************//**
 * @brief re-scale register SCALE
 *    note: IADC must be disabled to change scale
* @param[in]
 *    scale value in CFG[0]
 * @return
 *    none
 *****************************************************************************/
void rescaleIADC(uint32_t newScale)
{
  // Disable the IADC
  IADC0->EN_CLR = IADC_EN_EN;

  // configure new scale settings
  IADC0->CFG[0].SCALE = newScale;

  // Re-enable IADC
  IADC0->EN_SET = IADC_EN_EN;
}

/**************************************************************************//**
 * @brief
 *    poll signle adc result
 * @param[in]
 *    none
 * @return
 *    iadc value in volt unit
 *****************************************************************************/
double iadcPollSingleResult(void)
{
  /* start converting */
  IADC_command(IADC0, iadcCmdStartSingle);

  /* Wait for conversion to be complete
  ** while combined status bits 8 & 6 don't equal 1 and 0 respectively
  ** */
  while((IADC0->STATUS &
        (_IADC_STATUS_CONVERTING_MASK | _IADC_STATUS_SINGLEFIFODV_MASK))
        != IADC_STATUS_SINGLEFIFODV);

  /* Read data from the FIFO, 16-bit result */
  sample = IADC_pullSingleFifoResult(IADC0);

  /* For differential the result range is -Vref to +Vref, i.e., 16 bits for the
  ** conversion value.
  ** */
  singleResult = sample.data * 1.25 * 2 / 0xFFFF;

  return singleResult;
}

/**************************************************************************//**
 * @brief  Take several sequential samples and average the measurement
 * @param[in]
 *    numSamples - number of adc samples
 * @return
 *    adc average raw data
 *****************************************************************************/
double iadcAverageConversion(uint32_t numSamples)
{
  uint32_t i;
  double average;
  IADC_Result_t sample;

  /* Averaging loop, reset accumulator */
  average = 0;
  for(i = 0; i < numSamples; i++)
  {
    /* Start IADC conversion */
    IADC_command(IADC0, iadcCmdStartSingle);

    /* Wait for conversion to be complete
    ** while combined status bits 8 & 6 don't equal 1 and 0 respectively
    ** */
    while((IADC0->STATUS &
        (_IADC_STATUS_CONVERTING_MASK | _IADC_STATUS_SINGLEFIFODV_MASK))
        != IADC_STATUS_SINGLEFIFODV);

    /* Get ADC result and accumulate */
    sample = IADC_pullSingleFifoResult(IADC0);
    average += (int32_t) sample.data;
  }
  /* get average */
  average /= NUM_SAMPLES;

  return average;
}

/**************************************************************************//**
 * @brief
 *    IADC Initializer SCAN+LDMA
 * @param[in]
 *    none
 * @return
 *    none
 *****************************************************************************/
void initIADCScan(void)
{
  /* dummy, may use LDMA */
  ;
}

/**************************************************************************//**
 * @brief
 *    IADC Initializer
 * @param[in]
 *    none
 * @return
 *    none
 *****************************************************************************/
void initIADC(void)
{
  /* Declare init structs */
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
  IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

  /* Enable IADC clock */
  CMU_ClockEnable(cmuClock_IADC0, true);

  /* Reset IADC to reset configuration in case it has been modified */
  IADC_reset(IADC0);

  /* Configure IADC clock source */
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);

  /* Modify init structs and initialize */
  init.warmup = iadcWarmupKeepWarm;
  /* Set the HFSCLK prescale value here */
  /* CLK_SRC_ADC = CLK_SRC_ADC_FREQ, CLK_CMU_ADC = 0 */
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

  /* Configuration 0 is used by both scan and single conversions by default */
  /* Use external reference as reference */
  initAllConfigs.configs[0].reference = iadcCfgReferenceExt1V25;
  /* Divides CLK_SRC_ADC to set the CLK_ADC frequency for desired sample rate */
  initAllConfigs.configs[0].adcClkPrescale =
                            IADC_calcAdcClkPrescale(IADC0,
                            CLK_ADC_FREQ,
                            0,
                            iadcCfgModeNormal,
                            init.srcClkPrescale);

  /* Set over sampling rate to 32x
  ** resolution formula res = 11 + log2(oversampling * digital averaging)
  ** in this case res = 11 + log2(32 * 1) = 16
  ** */
  initAllConfigs.configs[0].osrHighSpeed = iadcCfgOsrHighSpeed32x;
  // initAllConfigs.configs[0].digAvg = iadcDigitalAverage16;

  /* Single initialization */
  initSingle.dataValidLevel = _IADC_SINGLEFIFOCFG_DVL_VALID1;
  /* Set conversions to run once per trigger */
  initSingle.triggerAction = iadcTriggerActionOnce;
  /* Set alignment to right justified with 16 bits for data field */
  initSingle.alignment = iadcAlignRight16;

  /* Configure input sources for differential conversion */
  initSingleInput.posInput = ADC_POS_INPUT;
  initSingleInput.negInput = ADC_NEG_INPUT;

  /* Initialize IADC
  ** This is taken care of in the IADC_init() function in the emlib em_iadc
  ** */
  IADC_init(IADC0, &init, &initAllConfigs);

  /* Initialize single */
  IADC_initSingle(IADC0, &initSingle, &initSingleInput);

  /* Allocate the analog bus for ADC0 inputs */
  GPIO->IADC_INPUT_0_BUS |= IADC_INPUT_0_BUSALLOC;
  GPIO->IADC_INPUT_1_BUS |= IADC_INPUT_1_BUSALLOC;

  /* Enable interrupts on data valid level */
  // IADC_enableInt(IADC0, IADC_IEN_SINGLEFIFODVL);

  /* Enable ADC interrupts */
  // NVIC_ClearPendingIRQ(IADC_IRQn);
  // NVIC_EnableIRQ(IADC_IRQn);
}

/**************************************************************************//**
 * @brief  Initialize IADC function
 * @param[in]
 *    none
 * @return
 *    none
 *****************************************************************************/
void initIADCforCali(void)
{
  /* Declare init structs */
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
  IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

  /* Enable IADC0 clock branch */
  CMU_ClockEnable(cmuClock_IADC0, true);
  /* Select clock for IADC */
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);  // FSRCO - 20MHz

  /* Set warmup mode */
  init.warmup = iadcWarmupKeepWarm;
  /* Set the HFSCLK pre-scale value here */
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

  /* Configuration 0 is used by both scan and single conversions by default */
  /* Use external reference */
  initAllConfigs.configs[0].reference = iadcCfgReferenceExt1V25;
  /* Force IADC to use bipolar inputs for conversion */
  initAllConfigs.configs[0].twosComplement = iadcCfgTwosCompBipolar;
  /* Divides CLK_SRC_ADC to set the CLK_ADC frequency */
  initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                             CLK_ADC_FREQ,
                                             0,
                                             iadcCfgModeNormal,
                                             init.srcClkPrescale);
  /* 32x OVS mode */
  initAllConfigs.configs[0].osrHighSpeed = iadcCfgOsrHighSpeed32x;

  // single initialization
  // initSingle.dataValidLevel = _IADC_SINGLEFIFOCFG_DVL_VALID1;
  initSingle.alignment = iadcAlignRight16;

  // Assign pins to positive and negative inputs in differential mode
  initSingleInput.posInput = iadcPosInputPortCPin0; // PC00
  initSingleInput.negInput = iadcNegInputPortCPin1; // PC01

  // Initialize the IADC
  IADC_init(IADC0, &init, &initAllConfigs);
  // Initialize the Single conversion inputs
  IADC_initSingle(IADC0, &initSingle, &initSingleInput);

  // Allocate the analog bus for ADC0 inputs
  GPIO->IADC_INPUT_0_BUS |= IADC_INPUT_0_BUSALLOC;
  GPIO->IADC_INPUT_1_BUS |= IADC_INPUT_1_BUSALLOC;
}

// DAC output expected (raw), 2^15-1 = 32767
/**************************************************************************//**
 * @brief
 *    Use DAC dac70501 to calibrate the bg22 iadc gain and offset
 * @param[in]
 *    none
 * @return
 *    calbrated scale value
 *****************************************************************************/
uint32_t iadcDifferentialCalibrate(void)
{
  /* please pay attention that offset is dependent to analog gain and OSR  */
  uint32_t scale;                          /* offset is dependent to scale */
  uint32_t caliGain1msb;                   /* msb gain result              */
  double caliGain13lsb;                    /* 0.75 or 1.0 gain             */

  double resultFullscale,                  /* full scale result, 1250mV    */
         resultZero,                       /* zero result, 0v              */
         resultOffset,                     /* zero offset, 0v              */
         resultRange;                      /* resultFullscale - resultZero */
  double gainCorrectionFactor;             /* gain correction factor       */
  int32_t iadcCalculatedOffset;            /* offset from resultZero       */
  /* gain resolution, 0.25 / 8192 = 0.000030517578125                      */
  double IADC_GAIN13LSB_LSB = 0.25 / 8192; /* lsb gain multiply factor     */

  uint32_t iadcCalibratedGain3lsb;         /* gain result value            */
  int32_t iadcCalibratedOffset;            /* offset result value          */

  /* Initialize ADC for calibration */
  initIADCforCali();
  iadcCalibratedGain3lsb = (IADC0->CFG[0].SCALE & _IADC_SCALE_GAIN13LSB_MASK)
                           >> _IADC_SCALE_GAIN13LSB_SHIFT;   /* 13 bit */
  iadcCalibratedOffset = (IADC0->CFG[0].SCALE & _IADC_SCALE_OFFSET_MASK)
                         >> _IADC_SCALE_OFFSET_SHIFT;        /* 18 bit */

  /* range, 0.75x to 1.2499x
  ** bitfield GAIN3MSB (3 MSBs of the 16-bit gain value)
  ** value   0(011)   1(100)
  ** gain    0.75x    1.0x
  ** bitfield GAIN13LSB (13 LSBs of the 16-bit gain value)
  ** value   0x0000          0x1FFF
  ** gain    0.75/1.0000     1.00/1.2499
  ** */

  /* Set initial offset to maximum negative and initial gain to 1.0 */

  scale = IADC_SCALE_GAIN3MSB_GAIN100 | IADC_SCALE_GAIN13LSB_DEFAULT
                                      | IADC_SCALE_OFFSET_MAX_NEG;
  /* here we use config[0] only */
  rescaleIADC(scale);

  /* Apply a full-scale (almost) positive input to the IADC
  ** Wait until differential voltage is applied
  ** Take multiple conversions and average to reduce system-level noise
  ** This is the ADC raw data.
  ** */
  dac70501_setVolt(1.25);
  resultFullscale = iadcAverageConversion(NUM_SAMPLES);

  /* Apply a zero differential input to the IADC (short POS and NEG)
  ** Wait until differential voltage is applied
  ** Take multiple conversions and average to reduce system-level noise
  ** */
  dac70501_setVolt(0.0);
  resultZero = iadcAverageConversion(NUM_SAMPLES);

  /* Calculate gain correction factor
  ** In bipolar mode, expected positive full-scale for IADC is
  ** (2^15) - 1 = 32727
  ** */
  resultRange = resultFullscale - resultZero;
  //gainCorrectionFactor = 32767 / (resultFullscale - resultZero);
  gainCorrectionFactor =  (double)32767.0 / resultRange;
  adcGainResult = gainCorrectionFactor;

  /* calculate the coarse offset here */
  iadcCalculatedOffset = -(IADC_SCALE_OFFSET_MAX_NEG + resultZero * 16);
  iadcCalculatedOffset = iadcCalculatedOffset * gainCorrectionFactor;

  /* Correct gain:
  ** Set IADC correction gain and clear offset in order to calibrate offset
  ** 3 MSB of gain is represented by 1 bit;
  **     a. 1 => 100 representing 1.00x to 1.2499x,
  **     b. 0 => 011 representing 0.75x to 0.9999x
  ** */
  if(gainCorrectionFactor >= 1.0)
  {
      /* need to increase gain */
      caliGain1msb = IADC_SCALE_GAIN3MSB_GAIN100;
      caliGain13lsb = (gainCorrectionFactor - 1.0) / IADC_GAIN13LSB_LSB;
      /* round to the nearest integer */
      iadcCalibratedGain3lsb = (uint32_t) (caliGain13lsb + 0.5);
      scale = IADC_SCALE_GAIN3MSB_GAIN100                           /* 3 msb  */
          | (iadcCalibratedGain3lsb << _IADC_SCALE_GAIN13LSB_SHIFT) /* 13 lsb */
          | IADC_SCALE_OFFSET_ZERO;                                 /* offset */
  }
  else
  {
      /* need to decrease gain */
      caliGain1msb = IADC_SCALE_GAIN3MSB_GAIN011;
      caliGain13lsb = (gainCorrectionFactor - 0.75) / IADC_GAIN13LSB_LSB;
      /* round to the nearest integer */
      iadcCalibratedGain3lsb = (uint32_t) (caliGain13lsb + 0.5);
      scale = IADC_SCALE_GAIN3MSB_GAIN011
          | (iadcCalibratedGain3lsb << _IADC_SCALE_GAIN13LSB_SHIFT)
          | IADC_SCALE_OFFSET_ZERO;
  }

  /* apply the gain correction */
  rescaleIADC(scale);

  /* Correct offset:
  ** Apply a zero differential input to the IADC (short POS and NEG)
  ** Take multiple conversions and average to reduce system-level noise
  ** */
  /* already applied zero when measure resultZero */
  resultOffset = iadcAverageConversion(NUM_SAMPLES);
  adcOffsetresult = resultOffset * 1.25 * 2 / 0xFFFF;

  /* scale and negate offset
  ** OFFSET is encoded as a 2's complement,
  ** 18-bit number with the LSB representing 1 / (2^20) of full scale.
  ** */
  /* 16-bit convert to 20-bit, need gain 2^(20-16) = 16 */
  iadcCalibratedOffset = (int32_t) (resultOffset * -16);

  /* 18-bit boundary check [-2^17 <-> (2^17-1)]
  ** Check if offset is too large to be corrected and
  ** set to maximum correction allowed
  ** */
  if (iadcCalibratedOffset > 131071)
    iadcCalibratedOffset = 131071;
  if (iadcCalibratedOffset < -131072)
    iadcCalibratedOffset = -131072;

  /* offset and gain result */
  scale = caliGain1msb
        | (iadcCalibratedGain3lsb << _IADC_SCALE_GAIN13LSB_SHIFT)
        | (iadcCalibratedOffset & _IADC_SCALE_OFFSET_MASK);

  /* apply gain and offset correction */
  rescaleIADC(scale);

  /* expected to get result close to zero
   * for test purpose*/
  resultOffset = iadcAverageConversion(NUM_SAMPLES);

  /* set dac70501 voltage for test */
  gainCorrectionFactor = 1.00f;
  dac70501_setVolt((float) gainCorrectionFactor);
  letimerDelay(2);

  /* test result, 10 samples */
  for (uint8_t i = 0; i < 20; i++)
  {
    /* store result to buffer */
    buffer[i] = iadcPollSingleResult();
  }

  IADC_reset(IADC0);

  return scale;
}

/***************************************************************************//**
 * @brief
 *    save iadc calibration data to internal flash
 * @param[in]
 *    scale, config scale value
 * @return
 *    none
 ******************************************************************************/
#define USERDATA ((uint32_t*)USERDATA_BASE)
#define POSITION 3
uint32_t cleared_value, write_value;
void bg22SaveCalData(uint32_t scale)
{
  /* Enable MSC Clock */
  CMU_ClockEnable(cmuClock_MSC, true);

  /* Clear the user data page of any previous data stored */
  MSC_ErasePage(USERDATA);

  /* Read the initial value in the cleared page */
  cleared_value = USERDATA[POSITION];

  /* Write the value into the 4th word of the user data portion of the flash */
  MSC_Init();
  MSC_WriteWord((USERDATA + POSITION), &scale, 4);
  MSC_Deinit();

  write_value = USERDATA[POSITION];
}

/***************************************************************************//**
 * @brief
 *    restore iadc calibration data from flash into register
 * @param[in]
 *    none
 * @return
 *    none
 ******************************************************************************/
void bg22RestoreCalData(void)
{
  uint32_t scale;

  scale = USERDATA[POSITION];
  // offset = USERDATA[POSITION+1];

  rescaleIADC(scale);
}

#ifdef __cplusplus
}
#endif
