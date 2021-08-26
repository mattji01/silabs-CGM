/***************************************************************************//**
 * @file
 * @brief CGM specific ops characteristic
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
******************************************************************************/


#include "cgm.h"


bool session_started = false;
bool sops_enabled = false;

uint8_t comm_interval = GLUCOSE_DEFAULT_COMM_INTERVAL_SEC;

bool periodic_comm = false;


/**************************************************************************//**
 * CGM - CGM specific ops characteristic
 * indication changed callback
 *
 * Called when indication of CGM sops is enabled/disabled by
 * the client.
 *****************************************************************************/

void sl_bt_cgm_sops_indication_handler(sl_bt_msg_t *evt)
{
  // client characteristic configuration changed by remote GATT client
  if (sl_bt_gatt_server_client_config == (sl_bt_gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.status_flags) {
      if (sl_bt_gatt_server_indication == (sl_bt_gatt_server_client_configuration_t)evt->data.evt_gatt_server_characteristic_status.client_config_flags) {
          app_log("SOPS enable indicate\n");
          sops_enabled = true;
      }
      else if(sl_bt_gatt_server_disable == (sl_bt_gatt_server_client_configuration_t)evt->data.evt_gatt_server_characteristic_status.client_config_flags) {
          app_log("SOPS disable indicate\n");
          sops_enabled = false;
      }
   }
  // confirmation of indication received from remove GATT client
  else if (sl_bt_gatt_server_confirmation == (sl_bt_gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.status_flags) {
    app_log("SOPS receive indication confirm\n");
  }
}


// general SOPS indication event
void sl_bt_cgm_send_sops_indicate(uint8_t opcode, uint8_t response_code)
{
  uint8_t sl_sops_indication_reply[] = {0x1C,0x00,0x00};
  sl_sops_indication_reply[1] = opcode;
  sl_sops_indication_reply[2] = response_code;
  sl_status_t sc;
  sc = sl_bt_gatt_server_send_indication(
    connection,
    gattdb_cgm_specific_ops_control_point,
    sizeof(sl_sops_indication_reply),
    sl_sops_indication_reply
    );
  if(sc != SL_STATUS_OK){
      app_log("sl_bt_gatt_server_send_indication SOPS 0x%04X.\n", sc);
      while(sc == 0x0182){
          sc = sl_bt_gatt_server_send_indication(
            connection,
            gattdb_cgm_specific_ops_control_point,
            sizeof(sl_sops_indication_reply),
            sl_sops_indication_reply
            );
      }
  }
}

// general SOPS send response value(Hypo/Hyper alert value..)
void sl_bt_cgm_send_sops_response(uint8_t opcode, uint16_t value)
{
  uint8_t sl_sops_indication_reply[] = {0x00,0x00,0x00};
  sl_sops_indication_reply[0] = opcode;
  sl_sops_indication_reply[1] = value & 0xff;
  sl_sops_indication_reply[2] = value >> 8;
  sl_status_t sc;
  sc = sl_bt_gatt_server_send_indication(
    connection,
    gattdb_cgm_specific_ops_control_point,
    sizeof(sl_sops_indication_reply),
    sl_sops_indication_reply
    );
  if(sc != SL_STATUS_OK){
      app_log("sl_bt_gatt_server_send_indication SOPS 0x%04X.\n", sc);
      while(sc == 0x0182){
          sc = sl_bt_gatt_server_send_indication(
            connection,
            gattdb_cgm_specific_ops_control_point,
            sizeof(sl_sops_indication_reply),
            sl_sops_indication_reply
            );
      }
  }
}



void sl_bt_cgm_handle_sops(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  uint8_t operand;
  //uint8_t connection = evt->data.evt_connection_opened.connection;
  uint8_t len = evt->data.evt_gatt_server_attribute_value.value.len;
  for(int i = 0; i < len; i++){
      app_log("0x%02x ", evt->data.evt_gatt_server_attribute_value.value.data[i]);
  }
  app_log("\n");
  //uint16_t crc = 0;
  if(SL_BT_CGM_E2E_CRC_SUPPORTED){
    //crc = evt->data.evt_gatt_server_attribute_value.value.data[len - 2] | evt->data.evt_gatt_server_attribute_value.value.data[len - 1] << 8;
    //if(crc != crc16(evt->data.evt_gatt_server_attribute_value.value.data, len - 2)){
    if(0){
        sl_bt_gatt_server_send_user_write_response(connection, gattdb_cgm_specific_ops_control_point, 0x81);
        return;
    }else{
        sl_bt_gatt_server_send_user_write_response(connection, gattdb_cgm_specific_ops_control_point, 0);
    }
  }else{
      sl_bt_gatt_server_send_user_write_response(connection, gattdb_cgm_specific_ops_control_point, 0);
  }
  uint8_t opcode = evt->data.evt_gatt_server_attribute_value.value.data[0];
  switch (opcode) {
    case 0x1B:
      app_log("stop session\n");
      if(session_started == true){
        session_started = false;
        sc = sl_simple_timer_stop(&cgm_periodic_timer);
        if(sc != SL_STATUS_OK){
            app_log("sl_simple_timer_stop failed 0x%04X\n", sc);
        }
        cgm_status[2] = cgm_status[2] | SL_BT_CGM_STATUS_SESSION_STOPPED;
        sl_bt_cgm_send_sops_indicate(opcode, 0x01);
      }else{
          cgm_status[2] = cgm_status[2] |  SL_BT_CGM_STATUS_SESSION_STOPPED;
          sl_bt_cgm_send_sops_indicate(opcode, 0x01);
      }
      break;
    case 0x1A:
      app_log("start session\n");
      if(SL_BT_CGM_MULTI_BOND_SUPPORTED == false){
          //sl_bt_cgm_send_sops_indicate(opcode, 0x04);
          //break;
      }
      if(session_started == false){
        sc = sl_simple_timer_start(&cgm_periodic_timer,
                                   comm_interval * 1000,
                                   sl_bt_cgm_periodic_timer_cb,
                                   NULL,
                                   true);
        if(sc != SL_STATUS_OK){
            app_log("sl_simple_timer_start failed 0x%04X\n", sc);
        }
        session_started = true;
        cgm_status[2] = cgm_status[2] & ~SL_BT_CGM_STATUS_SESSION_STOPPED;
        sl_bt_cgm_send_sops_indicate(opcode, 0x01);
      }else{
          // session already started, send Response Code for ‘Procedure not completed’ (0x04).
          sl_bt_cgm_send_sops_indicate(opcode, 0x04);
          app_log("return Procedure not completed as the session is already started.\n");
      }
      break;
    // CGM Specific Ops – ‘Set CGM Communication Interval’
    case 0x01:
      app_log("Set CGM Communication Interval procedure.\n");
      operand = evt->data.evt_gatt_server_attribute_value.value.data[1];
      sl_bt_cgm_communication_interval_procedures(operand);
      sl_bt_cgm_send_sops_indicate(opcode, 0x01);
      break;
    // CGMS/SEN/CGMCP/BV-02-C [CGM Specific Ops – ‘Get CGM Communication Interval without E2E-CRC’]
    case 0x02:
      app_log("Get CGM communication interval\n");
      sl_bt_cgm_sops_reply_comm_interval();
      break;
    case 0x03:

      break;
    //  CGMS/SEN/CGMCP/BV-08-C [CGM Specific Ops – ‘Set Glucose Calibration value’]
    case 0x04:
      app_log("set glucose calibration value.\n");
      sl_bt_cgm_set_cal_value(evt);
      break;
    case 0x05:
      // Get Glucose Calibration value
      sl_bt_cgm_get_cal_value(evt);
      break;
    case 0x08:
      sl_bt_cgm_get_alert_level(evt);
      break;
    case 0x07:
      sl_bt_cgm_set_high_alert_level(evt);
      break;
    case 0x0A:
      sl_bt_cgm_set_low_alert_level(evt);
      break;
    case 0x0B:
        sl_bt_cgm_get_alert_level(evt);
      break;
    case SL_BT_CGM_GET_HYPO_ALERT_LEVEL_OPCODE:
      sl_bt_cgm_send_sops_response(SL_BT_CGM_GET_HYPO_ALERT_LEVEL_RSP_OPCODE, cgm_init.hypo_alert_level);
      break;
    case 0x0D:
      sl_bt_cgm_set_hypo_alert_level(evt);
      break;
    case SL_BT_CGM_GET_HYPER_ALERT_LEVEL_OPCODE:
      sl_bt_cgm_send_sops_response(SL_BT_CGM_GET_HYPER_ALERT_LEVEL_RSP_OPCODE, cgm_init.hyper_alert_level);
      break;
    case 0x10:
      sl_bt_cgm_set_hyper_alert_level(evt);
      break;
    case SL_BT_CGM_GET_RATE_DECREASE_ALERT_LEVEL_OPCODE:
      sl_bt_cgm_send_sops_response(SL_BT_CGM_GET_RATE_DECREASE_ALERT_LEVEL_RSP_OPCODE, cgm_init.rate_decrease_alert_value);
      break;
    case 0x13:
      sl_bt_cgm_set_rate_decrease_alert_level(evt);
      break;
    case SL_BT_CGM_GET_RATE_INCREASE_ALERT_LEVEL_OPCODE:
      sl_bt_cgm_send_sops_response(SL_BT_CGM_GET_RATE_INCREASE_ALERT_LEVEL_RSP_OPCODE, cgm_init.rate_increase_alert_value);
      break;
    case 0x16:
      sl_bt_cgm_set_rate_increase_alert_level(evt);
      break;
    case SL_BT_CGM_RESET_DEVICE_SPECIFIC_ALERT_OPCODE:
      sl_bt_cgm_reset_device_special_alert();
      break;
    default:
      break;
  }
}



