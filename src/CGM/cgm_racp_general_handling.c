/***************************************************************************//**
 * @file
 * @brief RACP general handle
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

void sl_bt_cgm_send_racp_indication(uint8_t opcode, uint8_t error)
{
  sl_status_t sc = SL_STATUS_FAIL;
  // Op Code (0x06), Operator of NUll(0x00), Operand representing Request Op Code, Response code value
  uint8_t buf[4] = {0x06, 0x00, 0x01, 0x01};
  buf[2] = opcode;
  buf[3] = error;
  sc = sl_bt_gatt_server_send_indication(
    connection,
    gattdb_record_access_control_point,
    sizeof(buf),
    buf);
  if (sc) {
    app_log_warning("Failed to send indication in RACP 0x%04X\n", sc);
  }
}

void sl_bt_cgm_racp_indication_handler(sl_bt_msg_t *evt)
{
  // client characteristic configuration changed by remote GATT client
  if (sl_bt_gatt_server_client_config == (sl_bt_gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.status_flags) {
      if (sl_bt_gatt_server_indication == (sl_bt_gatt_server_client_configuration_t)evt->data.evt_gatt_server_characteristic_status.client_config_flags) {
          app_log("RACP enable indicate\n");
          racp_indicate_enabled = true;
      }
      else if(sl_bt_gatt_server_disable == (sl_bt_gatt_server_client_configuration_t)evt->data.evt_gatt_server_characteristic_status.client_config_flags) {
          app_log("RACP disable indicate\n");
          racp_indicate_enabled = false;
      }
   }
  // confirmation of indication received from remove GATT client
  else if (sl_bt_gatt_server_confirmation == (sl_bt_gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.status_flags) {
    app_log("RACP receive indication confirm\n");
  }
}

void sl_bt_cgm_racp_handler(sl_bt_msg_t *evt)
{

  uint8_t len = evt->data.evt_gatt_server_attribute_value.value.len;
  for(int i = 0; i < len; i++){
      app_log("0x%02x ", evt->data.evt_gatt_server_attribute_value.value.data[i]);
  }
  app_log("\n");

  uint8_t opcode = evt->data.evt_gatt_server_attribute_value.value.data[0];
  //if(measure_indicate_enabled == false || racp_indicate_enabled == false){
  if(bonding == 0xff &&(measure_indicate_enabled == false || racp_indicate_enabled == false)){
      app_log("client have not enable RACP or measure indication\n");
      sl_bt_gatt_server_send_user_write_response(connection, gattdb_record_access_control_point, 0xFD);
      return;
  }else if(procedure_in_progress == true && opcode == SL_BT_CGM_OPCODE_REPORT_STORED_REPORT){
      sl_bt_gatt_server_send_user_write_response(connection, gattdb_record_access_control_point, 0xFE);
  }else{
      sl_bt_gatt_server_send_user_write_response(connection, gattdb_record_access_control_point, 0);
  }

  switch(opcode)
  {
    case 0x00:
      app_log("op code not supported\n");
      sl_bt_cgm_send_racp_indication(opcode, 0x02);
      break;
    case 0x01:
      sl_bt_cgm_report_record(evt);
      break;
    case 0x02:
      sl_bt_cgm_delete_record(evt);
      break;
    case 0x03:
      app_log("Abort Operation\n");
      sl_bt_cgm_abort_operation(evt);
      break;
    case 0x04:
      app_log("report records number\n");
      sl_bt_cgm_report_num_records(evt);
      break;
    default:
      app_log("unsupported opcode\n");
      //sl_cgm_unsupported_opcode(connection, opcode);
      // CGMS/SEN/CBE/BI-01-C [General Error Handling –‘Op Code Not Supported’]
      sl_bt_cgm_send_racp_indication(opcode, 0x02);
      break;
  }
}

 
