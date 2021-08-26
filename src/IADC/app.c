/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
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
 ******************************************************************************/
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "sl_sleeptimer.h"
#include "sl_simple_timer.h"
#include "gatt_db.h"
#include "app.h"
#include <mis.h>

// The advertising set handle allocated from Bluetooth stack.
float dacVoltageValue;                            /* dac70501 output voltage */
float bg22DieTemperature;                         /* bg22 emu die temp */
double adsAdcTemperature;                         /* ads1220 adc temp */
double gainCalResult;                             /* bg22 gain cal result */
double offsetCalResult;                           /* bg22 offset cal result */
uint32_t bg22AdcscaleResult;                      /* bg22 scale cal result */
int32_t flashCalPosition;                         /* flash pos to save scale */
double adcMax = 0.0;                              /* for enob calculation */
double adcMin = 1.26;                             /* for enob calculation */
double adcPeak = 0.0;                             /* for enob calculation */
double adcAve = 0.0;                              /* for enob calculation */
uint32_t adcSnr = 0.0;                            /* for enob calculation */
//static sl_sleeptimer_timer_handle_t periodic_timer;
//static sl_simple_timer_t simple_periodic_timer;   /* simple timer callback */
/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////

  app_log("\r\n\r\n\r\n");
  app_log("cgm_iadc \r\n");

  /* Initialize letimer for delay function */
  initLetimer();
  //debounceButton();

  app_log("HFXO clock output\r\n");
  /* Route clock to pin PC3 */
  //initClockOut();

  app_log("button init\r\n");
  /* Initialize button PC5 */
  //initButtonEM2();

  app_log("external dac70501 init\r\n");
  /* Initialize dac70501 */
  dac70501_init();
  /* set dac70501 dac output voltage */
  dacVoltageValue = 1.00f;
  dac70501_setVolt(dacVoltageValue);
  letimerDelay(10);

  app_log("external ads1220 init\r\n");
  /* Initialize ads1220 */
  ads1220_init();
  /* ads1220 gain and offset calibration */
  ads1220_Calibrate();
  /* collect 10 samples */
  for (uint32_t i = 0; i < 10; i++)
    buffer[i] = ads1220_getAdcDataVolt();

  app_log("read ads1220 adc temperature\r\n");
  /* get ads1220 temperature */
  adsAdcTemperature = ads1220_getAdcTemp();
  app_log("read BG22 emu die temperature\r\n");
  bg22DieTemperature = getDieTemperature();

  app_log("iadc calibration\r\n");
  /* efr32bg22 iadc calibration */
  bg22AdcscaleResult = iadcDifferentialCalibrate();
  //bg22SaveCalData(bg22AdcscaleResult);

  app_log("iadc collect data\r\n");
  /* collect adc result and dump it via vcom */
  /* Initialize the IADC */
  initIADC();
  //bg22RestoreCalData();
  rescaleIADC(bg22AdcscaleResult);
  dac70501_setVolt(dacVoltageValue);
  /* collect 1024 samples for ENOB calculation */
  adcMax = 0.0;
  adcMin = 1.26;
  adcAve = 0.0;
  for (uint32_t i = 0; i < 1024; i++)
  {
    buffer[i] = iadcPollSingleResult();
    adcAve += buffer[i];
    if(buffer[i] < adcMin)
      adcMin = buffer[i];
    if(buffer[i] > adcMax)
      adcMax = buffer[i];
  }
  adcPeak = (adcMax - adcMin) * 1000;             /* in mV unit */
  adcAve = adcAve / 1024 * 1000;                  /* in mV unit */
  adcPeak = adcPeak/6.6;                          /* in mV unit */
  adcSnr = (uint32_t)(1250 * 2 / adcPeak);        /* signal to noise ratio */
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

double bg22GlucoseValue;
/**************************************************************************//**
 * simple timer callback
 *****************************************************************************/
/*static void simple_periodic_timeout(sl_simple_timer_t *handle,
                                void *data)
{
  uint32_t adcValueIntmV, adcValueFramV;
  (void)&handle;
  (void)&data;
  bg22GlucoseValue = iadcPollSingleResult();
  adcValueIntmV = (uint32_t) (bg22GlucoseValue * 1000);
  adcValueFramV = (uint32_t) ((bg22GlucoseValue * 1000 - adcValueIntmV) * 1000);
  app_log("adc voltage %d.%d mV\r\n", adcValueIntmV, adcValueFramV);
}*/

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      /* add a periodic (soft) timer to measure blood glucose data */
      //sl_bt_system_set_soft_timer(32768, 0, 0);
      /* Create timer for waking up the system periodically */
     /* app_log("start a period time \r\n");
      sl_simple_timer_start(&simple_periodic_timer,
                            100,
                            simple_periodic_timeout,
                            NULL,
                            1);*/
      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      app_assert_status(sc);

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      app_assert_status(sc);

      // Create an advertising set.
     /* sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      app_assert_status(sc);
      // Start general advertising and enable connections.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);
      app_assert_status(sc);*/
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      // Restart advertising after client has disconnected.
     /* sc = sl_bt_advertiser_start(
        advertising_set_handle,
        sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);
      app_assert_status(sc);*/
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    case sl_bt_evt_system_soft_timer_id:
      /* capture blood glucose data */
      // bg22GlucoseValue = adcPollSingleResult();
      break;
    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}
