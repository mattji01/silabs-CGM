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

/**@brief Glucose Calibration value */
typedef struct
{
    uint16_t  concentration;  /**< Glucose Concentration mg/dL. */
    uint16_t  cal_time;           /**< Calibration Time minutes. */
    uint8_t   sample_location;/**< Sample Location */
    uint16_t  next_cal;       /**< Next Calibration minutes */
    uint16_t  record_num;     /**< Calibration Data Record Number */
    uint8_t   status;         /**< Calibration Status */
} sl_bt_cgms_cal_value_t;

typedef struct
{
  uint8_t opcode;
  sl_bt_cgms_cal_value_t value;
}sl_bt_cgms_cal_value_rsp_t;

sl_bt_cgm_init_value_t cgm_init ={130, 100, 60, 200, 4, 4};
#define CGM_CAL_VALUE_LEN 12
#define MAX_CALIBRATION_NUM 10
typedef struct
{
  uint8_t op_code;
  uint8_t operand;
  uint8_t response_code;
}sl_bt_cgm_sops_t;
sl_bt_cgms_cal_value_t calibration_value[MAX_CALIBRATION_NUM] = \
    {{0x004F, 0x0005, 0x06, 0x0005, 0x0000, 0x00}, {0x00}};
uint16_t cal_num = 0;

#define PARAMETER_OUT_OF_RANGE  0x05


void cgm_dump_cal_value(uint8_t sl_cgm_cal_value[], uint16_t index)
{
  sl_cgm_cal_value[0] = 0x06;
  sl_cgm_cal_value[1] = calibration_value[index].concentration & 0xff;
  sl_cgm_cal_value[2] = calibration_value[index].concentration >> 8;
  sl_cgm_cal_value[3] = calibration_value[index].cal_time & 0xff;
  sl_cgm_cal_value[4] = calibration_value[index].cal_time >> 8;
  sl_cgm_cal_value[5] = calibration_value[index].sample_location;
  sl_cgm_cal_value[6] = calibration_value[index].next_cal & 0xff;
  sl_cgm_cal_value[7] = calibration_value[index].next_cal >> 8;
  sl_cgm_cal_value[8] = calibration_value[index].record_num & 0xff;
  sl_cgm_cal_value[9] = calibration_value[index].record_num >> 8;
  sl_cgm_cal_value[10] = calibration_value[index].status;
  if (SL_BT_CGM_E2E_CRC_SUPPORTED){
      uint16_t crc = crc16(0xFFFF, sl_cgm_cal_value, 11);
      sl_cgm_cal_value[11] = UINT16_TO_BYTE0(crc);
      sl_cgm_cal_value[12] = UINT16_TO_BYTE1(crc);
  }else{
    sl_cgm_cal_value[11] = 0xFF;
    sl_cgm_cal_value[12] = 0xFF;
  }


}
// CGMS/SEN/CGMCP/BV-06-C [CGM Specific Ops – ‘Get Glucose Calibration Value’ Type 1]
void sl_bt_cgm_get_cal_value(sl_bt_msg_t *evt)
{
  uint16_t index = evt->data.evt_gatt_server_attribute_value.value.data[1] | evt->data.evt_gatt_server_attribute_value.value.data[2] << 8;
  //CGMS/SEN/CGMCP/BV-07-C [CGM Specific Ops – ‘Get Glucose Calibration Value’ Type 2]
  if(index == 0xFFFF){
      app_log("get latest calibration data %d\n", cal_num);
      uint8_t sl_cgm_cal_value[1 + CGM_CAL_VALUE_LEN];
      cgm_dump_cal_value(sl_cgm_cal_value, cal_num);
          sl_status_t sc;
          sc = sl_bt_gatt_server_send_indication(
            connection,
            gattdb_cgm_specific_ops_control_point,
            sizeof(sl_cgm_cal_value) - 2,
            sl_cgm_cal_value
            );
          if(sc != SL_STATUS_OK){
              app_log("sl_bt_cgm_get_cal_value 0x%04X\n", sc);
          }
          //CGMS/SEN/CGMCP/BI-01-C [CGM Specific Ops – ‘Get Glucose Calibration Value’ Type 3]
  }else if(index == 0xFFFE){
      //Response Code for ‘Parameter out of Range’ (0x05).
      //sl_bt_cgms_cal_value_t rsp = {0x1C, 0x05, PARAMETER_OUT_OF_RANGE};
      sl_bt_cgm_send_sops_indicate(0x05, 0x05);
  }else{
      sl_status_t sc;
      uint8_t sl_cgm_cal_value[1 + CGM_CAL_VALUE_LEN];
      cgm_dump_cal_value(sl_cgm_cal_value, index);
      sc = sl_bt_gatt_server_send_indication(
      connection,
      gattdb_cgm_specific_ops_control_point,
      sizeof(sl_cgm_cal_value) - 2,
      sl_cgm_cal_value
      );
    if(sc != SL_STATUS_OK){
        app_log("sl_bt_cgm_get_cal_value 0x%04X\n", sc);
    }
  }
}


void sl_bt_cgm_communication_interval_procedures(uint8_t interval)
{
  sl_status_t sc;
  if(interval == 0){
      app_log("interval is 0, disable the periodic communication\n");
      comm_interval = 0;
      if(session_started == true){
          session_started = false;
          sc = sl_simple_timer_stop(&cgm_periodic_timer);
          if(sc != SL_STATUS_OK){
              app_log("sl_simple_timer_stop failed 0x%04X\n", sc);
          }
      }
  }else if(interval == 0xFF){
      app_log("interval is 0xFF, change the communication interval to fastest.\n");
      comm_interval = GLUCOSE_DEFAULT_COMM_INTERVAL_SEC;
  }else{
      app_log("set CGM communication interval %d\n", interval);
      comm_interval = interval;
  }
}

void sl_bt_cgm_sops_reply_comm_interval(void)
{
   uint8_t sl_sops_indication_reply[] = {0x03,0x00,0xff,0xff};
   sl_sops_indication_reply[1] = comm_interval;
   if (SL_BT_CGM_E2E_CRC_SUPPORTED){
       uint16_t crc = crc16(0xFFFF, sl_sops_indication_reply, 2);
       sl_sops_indication_reply[2] = UINT16_TO_BYTE0(crc);
       sl_sops_indication_reply[3] = UINT16_TO_BYTE1(crc);
     }
   sl_status_t sc;
   sc = sl_bt_gatt_server_send_indication(
     connection,
     gattdb_cgm_specific_ops_control_point,
     sizeof(sl_sops_indication_reply),
     sl_sops_indication_reply
     );
   if(sc != SL_STATUS_OK){
       app_log("sl_bt_gatt_server_send_indication SOPS 0x%04X\n", sc);
   }
}


void sl_bt_cgm_send_cal_value(uint8_t cal_value)
{
  uint8_t sl_sops_indication_reply[] = {0x06,0x00,0x00};
  sl_sops_indication_reply[1] = cal_value;
  sl_sops_indication_reply[2] = cal_value >> 8;
  sl_status_t sc;
  sc = sl_bt_gatt_server_send_indication(
    connection,
    gattdb_cgm_specific_ops_control_point,
    sizeof(sl_sops_indication_reply),
    sl_sops_indication_reply
    );
  if(sc != SL_STATUS_OK){
      app_log("sl_bt_gatt_server_send_indication SOPS 0x%04X, connection %d\n", sc, connection);
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

void sl_bt_cgm_set_cal_value(sl_bt_msg_t *evt)
{
  cal_num ++;
  if(cal_num > MAX_CALIBRATION_NUM){
      app_log("out of calibration value store memory\n");
      return;
  }
  calibration_value[cal_num].concentration = evt->data.evt_gatt_server_attribute_value.value.data[1] | evt->data.evt_gatt_server_attribute_value.value.data[2] << 8;
  calibration_value[cal_num].cal_time = evt->data.evt_gatt_server_attribute_value.value.data[3] | evt->data.evt_gatt_server_attribute_value.value.data[4] << 8;
  calibration_value[cal_num].sample_location = evt->data.evt_gatt_server_attribute_value.value.data[5];
  calibration_value[cal_num].next_cal = evt->data.evt_gatt_server_attribute_value.value.data[6] | evt->data.evt_gatt_server_attribute_value.value.data[7] << 8;
  calibration_value[cal_num].record_num = evt->data.evt_gatt_server_attribute_value.value.data[8] | evt->data.evt_gatt_server_attribute_value.value.data[9] << 8;
  calibration_value[cal_num].status = evt->data.evt_gatt_server_attribute_value.value.data[10];
  sl_bt_cgm_send_sops_indicate(0x04, 0x01);
}

void sl_bt_cgm_get_alert_level(sl_bt_msg_t *evt)
{
  uint8_t sl_sops_indication_reply[] = {0x09,0x00,0x00};
  if(evt->data.evt_gatt_server_attribute_value.value.data[0] == 0x08){
    sl_sops_indication_reply[1] = cgm_init.high_alert_value & 0xff;
    sl_sops_indication_reply[2] = cgm_init.high_alert_value >> 8;
  }else if(evt->data.evt_gatt_server_attribute_value.value.data[0] == 0x0B){
      sl_sops_indication_reply[0] = 0x0C;
      sl_sops_indication_reply[1] = cgm_init.low_alert_value & 0xff;
      sl_sops_indication_reply[2] = cgm_init.low_alert_value >> 8;
      app_log("low is %d, first is %d, second is %d\n", cgm_init.low_alert_value, sl_sops_indication_reply[1], sl_sops_indication_reply[2]);
  }
  sl_status_t sc;
  sc = sl_bt_gatt_server_send_indication(
    connection,
    gattdb_cgm_specific_ops_control_point,
    sizeof(sl_sops_indication_reply),
    sl_sops_indication_reply
    );
  if(sc != SL_STATUS_OK){
      app_log("sl_bt_gatt_server_send_indication SOPS 0x%04X, connection %d\n", sc, connection);
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



void sl_bt_cgm_set_high_alert_level(sl_bt_msg_t *evt)
{
  uint16_t high_alert_value = evt->data.evt_gatt_server_attribute_value.value.data[1] | evt->data.evt_gatt_server_attribute_value.value.data[2] << 8;
  if(high_alert_value > 0x700){
      sl_bt_cgm_send_sops_indicate(0x07, 0x05);
  }else{
      cgm_init.high_alert_value = high_alert_value;
      sl_bt_cgm_send_sops_indicate(0x07, 0x01);
  }
}


void sl_bt_cgm_set_low_alert_level(sl_bt_msg_t *evt)
{
  uint16_t low_alert_value = evt->data.evt_gatt_server_attribute_value.value.data[1] | evt->data.evt_gatt_server_attribute_value.value.data[2] << 8;
  if(low_alert_value > 0x700){
      sl_bt_cgm_send_sops_indicate(0x0A, 0x05);
  }else{
      cgm_init.low_alert_value = low_alert_value;
      sl_bt_cgm_send_sops_indicate(0x0A, 0x01);
  }
}

void sl_bt_cgm_set_hypo_alert_level(sl_bt_msg_t *evt)
{
  uint16_t hypo_alert_level = evt->data.evt_gatt_server_attribute_value.value.data[1] | evt->data.evt_gatt_server_attribute_value.value.data[2] << 8;
  if(hypo_alert_level > 100){
      sl_bt_cgm_send_sops_indicate(0x0D, 0x05);
  }else{
      cgm_init.hypo_alert_level = hypo_alert_level;
      sl_bt_cgm_send_sops_indicate(0x0D, 0x01);
  }
}

void sl_bt_cgm_set_hyper_alert_level(sl_bt_msg_t *evt)
{
  uint16_t hyper_alert_level = evt->data.evt_gatt_server_attribute_value.value.data[1] | evt->data.evt_gatt_server_attribute_value.value.data[2] << 8;
  if(hyper_alert_level > 1000){
      sl_bt_cgm_send_sops_indicate(0x10, 0x05);
  }else{
      cgm_init.hyper_alert_level = hyper_alert_level;
      sl_bt_cgm_send_sops_indicate(0x10, 0x01);
  }
}


void sl_bt_cgm_set_rate_decrease_alert_level(sl_bt_msg_t *evt)
{
  uint16_t level = evt->data.evt_gatt_server_attribute_value.value.data[1] | evt->data.evt_gatt_server_attribute_value.value.data[2] << 8;
  if(level > 100){
      sl_bt_cgm_send_sops_indicate(0x13, 0x05);
  }else{
      cgm_init.rate_decrease_alert_value = level;
      sl_bt_cgm_send_sops_indicate(0x13, 0x01);
  }
}

void sl_bt_cgm_set_rate_increase_alert_level(sl_bt_msg_t *evt)
{
  uint16_t level = evt->data.evt_gatt_server_attribute_value.value.data[1] | evt->data.evt_gatt_server_attribute_value.value.data[2] << 8;
  if(level > 1000){
      sl_bt_cgm_send_sops_indicate(0x16, 0x05);
  }else{
      cgm_init.rate_increase_alert_value = level;
      sl_bt_cgm_send_sops_indicate(0x16, 0x01);
  }
}

void sl_bt_cgm_reset_device_special_alert(void)
{
  cgm_status[2] = cgm_status[2] &  ~(SL_BT_CGM_STATUS_DEVICE_SEPCIFIC_ALERT);
  sl_bt_cgm_send_sops_indicate(0x19, 0x01);
}
