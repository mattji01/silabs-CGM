/***************************************************************************//**
 * @file
 * @brief CGM measurement characteristic
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
* # Experimental Quality
* This code has not been formally tested and is provided as-is. It is not
* suitable for production environments. In addition, this code will not be
* maintained and there may be no bug maintenance planned for these resources.
* Silicon Labs may update projects from time to time.
******************************************************************************/#include "cgm.h"

// Glucose measurement characteristic notification enable
bool measure_indicate_enabled = false;

// Periodic timer handle.
sl_simple_timer_t cgm_periodic_timer;


static uint8_t sl_glucose_concentration[] = {0x0F,0xE3,04,00,00,00,01,01,01,00,00,00,00,0xFF,0xFF};

/**************************************************************************//**
 * Send Temperature Measurement characteristic indication to the client.
 *****************************************************************************/
void sl_bt_cgm_measurement_notificate(void)
{
  sl_status_t sc;
  sc = sl_bt_gatt_server_send_notification(
    connection,
    gattdb_cgm_measurement,
    sizeof(sl_glucose_concentration),
    sl_glucose_concentration
    );
  if (sc) {
    app_log_warning("Failed to send measurement notification 0x%04X\n", sc);
    while(sc == SL_STATUS_NO_MORE_RESOURCE){
        sc = sl_bt_gatt_server_send_notification(
          connection,
          gattdb_cgm_measurement,
          sizeof(sl_glucose_concentration),
          sl_glucose_concentration
          );
    }
  }
  sl_glucose_concentration[2] ++ ;
  uint16_t temp = sl_glucose_concentration[4] | sl_glucose_concentration[5] << 8;
  temp = temp + GLUCOSE_DEFAULT_COMM_INTERVAL_SEC;
  sl_glucose_concentration[4] = temp & 0xff;
  sl_glucose_concentration[5] = temp >> 8;
  if (sl_glucose_concentration[2] == 30)
    sl_glucose_concentration[2] = 04;
  if(SL_BT_CGM_E2E_CRC_SUPPORTED){
    uint16_t crc = crc16(0xFFFF, sl_glucose_concentration, sizeof(sl_glucose_concentration) - 2);
    sl_glucose_concentration[sizeof(sl_glucose_concentration) - 2] = UINT16_TO_BYTE0(crc);
    sl_glucose_concentration[sizeof(sl_glucose_concentration) - 1] = UINT16_TO_BYTE1(crc);
  }
}

/**************************************************************************//**
 * Timer callback
 * Called periodically to time periodic measurements and notifications.
 *****************************************************************************/
void sl_bt_cgm_periodic_timer_cb(sl_simple_timer_t *timer, void *data)
{
  (void)data;
  (void)timer;
//  sl_status_t sc = SL_STATUS_OK;
  if(session_started == true){
      sl_bt_cgm_measurement_notificate();
  }
}

/**************************************************************************//**
 * CGM - CGM Measurement
 * notification changed callback
 *
 * Called when notification of CGM measurement is enabled/disabled by
 * the client.
 *****************************************************************************/
void sl_bt_cgm_measurement_notification_handler(sl_bt_msg_t *evt)
{
  // client characteristic configuration changed by remote GATT client
  if (sl_bt_gatt_server_client_config == (sl_bt_gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.status_flags) {
      if (sl_bt_gatt_server_notification == (sl_bt_gatt_server_client_configuration_t)evt->data.evt_gatt_server_characteristic_status.client_config_flags) {
          app_log("measurement enable notification\n");
          measure_indicate_enabled = true;
      }  // notification disabled.
      else if(sl_bt_gatt_server_disable == (sl_bt_gatt_server_client_configuration_t)evt->data.evt_gatt_server_characteristic_status.client_config_flags) {
          app_log("measurement disable notification\n");
          measure_indicate_enabled = false;
          (void)sl_simple_timer_stop(&cgm_periodic_timer);
      }
  }

}

