/***************************************************************************//**
 * @file
 * @brief CGM session start timer characteristic
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

static uint8_t session_start_time[11] = {0xE5,0x07,0x07,0x05,0x12,0x30,0x05,0x08,0x00,0x00,0x00};

uint8_t feature[FEATURE_LEN]={0x00};

typedef struct
{
  uint16_t run_time;
  uint16_t start_time;
}sl_bt_cgm_characteristic_t;

sl_bt_cgm_characteristic_t cr = {0x00};

// offset 2 octets, cgm_status 3 octets, CRC 2 octets
typedef struct
{
  uint8_t offset;
  uint8_t warning;
  uint8_t calib_temp;
  uint16_t status;
  uint16_t crc;
}sl_bt_cgm_status_t;

uint8_t cgm_status[STATUS_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void sl_bt_cgm_init_database(void)
{
  cgm_status[2] = SL_BT_CGM_STATUS_DEVICE_SEPCIFIC_ALERT;
  if(SL_BT_CGM_E2E_CRC_SUPPORTED){
    uint16_t crc = crc16(0xFFFF, cgm_status, sizeof(cgm_status) - 2);
    cgm_status[STATUS_LEN - 2] = UINT16_TO_BYTE0(crc);
    cgm_status[STATUS_LEN - 1] = UINT16_TO_BYTE1(crc);
  }

  if(SL_BT_CGM_E2E_CRC_SUPPORTED == true){
  	feature[1] = feature[1] | SL_BT_CGM_E2E_CRC;
    uint16_t crc = crc16(0xFFFF, feature, sizeof(feature) - 2);
    feature[ sizeof(feature) - 2] = UINT16_TO_BYTE0(crc);
    feature[ sizeof(feature) - 1] = UINT16_TO_BYTE1(crc);
  }
  if(SL_BT_CGM_MULTI_BOND_SUPPORTED == true){
  	feature[1]  = feature[1]| SL_BT_CGM_MULTI_BOND;
  }
  if(SL_BT_CGM_MULTI_SESSION_SUPPORTED == true){
  	feature[1] = feature[1] | SL_BT_CGM_MULTI_SESSION;
  }
  feature[3] = SL_BT_CGM_SAMPLE_LOCATION << 4 | SL_BT_CGM_TYPE;
  feature[1] = 0x60;

  cr.run_time = 0x0001;

}


void sl_bt_cgm_handle_sst_write(sl_bt_msg_t *evt)
{
  uint8_t timezone = evt->data.evt_gatt_server_user_write_request.value.data[7];
  uint8_t connection = evt->data.evt_gatt_server_user_write_request.connection;
  uint8_t len = evt->data.evt_gatt_server_user_write_request.value.len;
  uint8_t offset = evt->data.evt_gatt_server_user_write_request.offset;
  app_log("timezone %d, len %d, offset %d\n", timezone, len, offset);
  for(uint8_t i = 0; i < len; i++){
      session_start_time[i] = evt->data.evt_gatt_server_user_write_request.value.data[i];
    app_log("0x%02X ", session_start_time[i]);
  }
  app_log("\n");
  // CGMS/SEN/CBE/BI-06-C [General Error Handling – ‘Missing CRC’]
  if(SL_BT_CGM_E2E_CRC_SUPPORTED){
      if(len != SL_BT_CGM_SST_WITHCRC_LEN){
          sl_bt_gatt_server_send_user_write_response(connection, gattdb_cgm_session_start_time, 0x80);
          return;
      }
  }
  if(timezone > 25){
      sl_bt_gatt_server_send_user_write_response(connection, gattdb_cgm_session_start_time, 0xff);
  }else{
      sl_bt_gatt_server_send_user_write_response(connection, gattdb_cgm_session_start_time, 0);
  }
}

/***************************************************************************//**
 * @brief
 *    4.6 CGMS/SEN/CR/BV-04-C
 *    [Characteristic Read – ‘CGM Session Start Time with E2E-CRC’]
 *
 ******************************************************************************/
void sl_bt_cgm_handle_sst_read(void)
{
  app_log("Read CGM Session Start Time\n");
  uint16_t sent_len = 0;
  sl_status_t sc;
  if(SL_BT_CGM_E2E_CRC_SUPPORTED){
    uint16_t crc = crc16(0xFFFF, session_start_time, sizeof(session_start_time) - 2);
    session_start_time[sizeof(session_start_time) - 2] = crc & 0xFF;
    session_start_time[sizeof(session_start_time) - 1] = crc >> 8;
    sc = sl_bt_gatt_server_send_user_read_response(connection,
                                                   gattdb_cgm_session_start_time,
                                                   0,
                                                   sizeof(session_start_time),
                                                   session_start_time,
                                                   &sent_len);
  }else{
      sc = sl_bt_gatt_server_send_user_read_response(connection,
                                                     gattdb_cgm_session_start_time,
                                                     0,
                                                     sizeof(session_start_time) - 2,
                                                     session_start_time,
                                                     &sent_len);
  }
  if(sc != SL_STATUS_OK){
      app_log("sl_bt_gatt_server_send_user_read_response session start failed 0x%04X\n", sc);
  }

}


void sl_bt_cgm_handle_status_read(void)
{
  app_log("read cgm_status\n");
  uint16_t sent_len = 0;
  sl_status_t sc;
  sc = sl_bt_gatt_server_send_user_read_response(connection, gattdb_cgm_status,
                                                       0, STATUS_LEN,
                                                        cgm_status,
                                                        &sent_len);
  for(uint8_t i = 0; i < STATUS_LEN; i++)
    app_log("0x%02X ", cgm_status[i]);
  app_log("sent length %d\n", sent_len);
  if(sc != SL_STATUS_OK){
      app_log("sl_bt_cgm_handle_status_read cgm_status 0x%04X\n", sc);
  }
}

void sl_bt_cgm_handle_feature_read(void)
{
  app_log("read feature\n");
  uint16_t sent_len = 0;
  sl_status_t sc;
  sc = sl_bt_gatt_server_send_user_read_response(connection,
                                                 gattdb_cgm_feature,
                                                 0,
                                                 FEATURE_LEN,
                                                 feature,
                                                 &sent_len);
  if(sc != SL_STATUS_OK){
      app_log("sl_bt_gatt_server_send_user_read_response feature 0x%04X\n", sc);
  }
}

void sl_bt_cgm_handle_run_time_read(void)
{
  uint16_t sent_len = 0;
  sl_status_t sc;
  uint8_t temp[2];
  temp[0] = cr.run_time & 0xFF;
  temp[1] = cr.run_time >> 8;
  if(SL_BT_CGM_E2E_CRC_SUPPORTED){
      //uint16_t crc = crc16(temp, sizeof(temp));

    sc = sl_bt_gatt_server_send_user_read_response(connection, gattdb_cgm_feature,
                                                         0, 4,
                                                          temp,
                                                          &sent_len);
    if(sc != SL_STATUS_OK){
        app_log("sl_bt_gatt_server_send_user_read_response feature 0x%04X\n", sc);
    }
  }else
    {
      sc = sl_bt_gatt_server_send_user_read_response(connection, gattdb_cgm_feature,
                                                               0, 2,
                                                                temp,
                                                                &sent_len);
          if(sc != SL_STATUS_OK){
              app_log("sl_bt_gatt_server_send_user_read_response feature 0x%04X\n", sc);
          }
    }
}
