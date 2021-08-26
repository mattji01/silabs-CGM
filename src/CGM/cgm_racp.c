/***************************************************************************//**
 * @file
 * @brief CGM Record Access Control Point characteristic
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
/**************************************************************************//**
 * BGM - BGM Record Access Control Point characteristic
 * indication changed callback
 *
 * Called when indication of BGM RCAP is enabled/disabled by
 * the client.
 *****************************************************************************/

#include "cgm.h"
uint16_t records_num = 5;

// Glucose RACP characteristic indication enable
bool racp_indicate_enabled = false;
bool abort_operation = false;


#define SL_CGM_OPERATOR_REPORT_FIRST_RECORD 0x05
#define SL_CGM_OPERATOR_REPORT_LAST_RECORD  0x06
void sl_bt_cgm_report_record(sl_bt_msg_t *evt)
{
  uint8_t len = evt->data.evt_gatt_server_attribute_value.value.len;
  uint8_t opcode = evt->data.evt_gatt_server_attribute_value.value.data[0];
  uint8_t operator = evt->data.evt_gatt_server_attribute_value.value.data[1];
  //uint8_t connection = evt->data.evt_gatt_server_attribute_value.connection;
  uint8_t filter_type;
  uint16_t high;
  switch (operator)
  {
    case 0x00:
      app_log("Invalid Operator 0x%02X\n", operator);
      sl_bt_cgm_send_racp_indication(opcode, 0x03);
      break;
    case 0x01:
      if(len != 2){
          // CGMS/SEN/CBE/BI-04-C [General Error Handling – ‘Invalid Operand’ – Type 1]
          sl_bt_cgm_send_racp_indication(opcode, 0x05);
          break;
      }
      app_log("report all records\n");
      sl_bt_cgm_report_all_records();
      break;
    case 0x02:
      sl_bt_cgm_report_records_less_than(evt);
      break;
    case 0x03:
      filter_type = evt->data.evt_gatt_server_attribute_value.value.data[2];
      if(filter_type != 0x01){
          //CGMS/SEN/RAE/BI-01-C [RACP specific Errors – ‘Unsupported Filter Type’]
            // Response Code Value for ‘Operand not supported’ (0x09).
            sl_bt_cgm_send_racp_indication(opcode, 0x09);
            break;
      }
      high = evt->data.evt_gatt_server_attribute_value.value.data[3] | evt->data.evt_gatt_server_attribute_value.value.data[4] << 8;
      app_log("report greater than or equal to Time offset records num\n");
      if(high > records_num){
          // ‘No Records found’ (0x06).
          app_log("No Records found, greater than max record number\n");
          sl_bt_cgm_send_racp_indication(opcode, 0x06);
          break;
      }
      sl_bt_cgm_report_records_greater_than(evt);
      break;
    case 0x04:
      app_log("report records within range of\n");
      sl_bt_cgm_report_records_within_range(evt);
      break;
    case SL_CGM_OPERATOR_REPORT_FIRST_RECORD:
      app_log("report first record\n");
      sl_bt_cgm_report_first_record();
      break;
    case SL_CGM_OPERATOR_REPORT_LAST_RECORD:
      app_log("report last record\n");
      sl_bt_cgm_report_last_record();
      break;
    default:
      // CGMS/SEN/CBE/BI-03-C [General Error Handling – ‘Unsupported Operator’]
      app_log("Operator 0x%02X not support\n", operator);
      sl_bt_cgm_send_racp_indication(opcode, 0x04);
      break;
  }
}

void sl_bt_cgm_abort_operation(sl_bt_msg_t *evt)
{
  abort_operation = true;
  procedure_in_progress = false;
  uint8_t operator = evt->data.evt_gatt_server_attribute_value.value.data[1];
  if(operator != 0x00){
      return;
  }
  sl_status_t sc = SL_STATUS_FAIL;
  uint8_t cgm_abort_operation_data[] = {0x06, 0x00, 0x03, 0x01};
  sc = sl_bt_gatt_server_send_indication(
      connection,
      gattdb_record_access_control_point,
      sizeof(cgm_abort_operation_data),
      cgm_abort_operation_data
      );
  if (sc) {
    app_log_warning("Failed to send indication in abort operation 0x%04x\n", sc);
    while(sc == SL_STATUS_NO_MORE_RESOURCE){
        sc = sl_bt_gatt_server_send_indication(
              connection,
              gattdb_record_access_control_point,
              sizeof(cgm_abort_operation_data),
              cgm_abort_operation_data
              );
    }
  }
}
void sl_bt_cgm_report_num_records(sl_bt_msg_t *evt)
{
  //uint8_t opcode = evt->data.evt_gatt_server_attribute_value.value.data[0];
  uint8_t operator = evt->data.evt_gatt_server_attribute_value.value.data[1];
  uint8_t connection = evt->data.evt_gatt_server_attribute_value.connection;
  //uint8_t filter_type;
  //uint16_t high;
  switch (operator)
  {
    case 0x01:
      app_log("report all records num\n");
      sl_bt_cgm_report_all_num_records(connection);
      break;
    //CGMS/SEN/RAN/BV-02-C [Report Number of Stored Records –‘Greater than or equal to Time Offset’]
    case 0x03:
      app_log("report number of stored records greater than\n");
      sl_bt_cgm_report_greater_num_records(evt);
      break;
    default:
      break;
  }
}
#define SL_BT_CGM_OPERATOR_DELETE_WITHIN_RANGE 0x04
void sl_bt_cgm_delete_record(sl_bt_msg_t *evt)
{
  uint8_t operator = evt->data.evt_gatt_server_attribute_value.value.data[1];
  uint8_t connection = evt->data.evt_gatt_server_attribute_value.connection;
  switch (operator)
  {
    case 0x01:
      app_log("delete all records\n");
      sl_bt_cgm_delete_all_num_records(connection);
      break;
    case SL_BT_CGM_OPERATOR_DELETE_WITHIN_RANGE:
      app_log("delete records within range of\n");
      sl_bt_cgm_delete_records_within_range(evt);
      break;
    default:
      break;
  }
}
/**************************************************************************//**
 * 4.10 CGMS/SEN/RAN/BV-01-C [Report Number of Stored Records – ‘All records’]
 * 4.10 CGMS/SEN/RAN/BV-03-C [Report Number of Stored Records – ‘No records found’]
 *****************************************************************************/
void sl_bt_cgm_report_all_num_records(uint8_t connection)
{
  sl_status_t sc = SL_STATUS_FAIL;
  uint8_t cgm_report_all_num_records_data[] = {0x05, 0x00, 0x00, 0x00};
  cgm_report_all_num_records_data[2] = records_num & 0xff;
  cgm_report_all_num_records_data[3] = records_num >> 8;
  sc = sl_bt_gatt_server_send_indication(
      connection,
      gattdb_record_access_control_point,
      sizeof(cgm_report_all_num_records_data), cgm_report_all_num_records_data
      );
  if (sc) {
    app_log_warning("Failed to report all records number\n");
  }
}

/**************************************************************************//**
 * CGMS/SEN/RAN/BV-02-C [Report Number of Stored Records –‘Greater than or equal to Time Offset’]
 *****************************************************************************/
void sl_bt_cgm_report_greater_num_records(sl_bt_msg_t *evt)
{
  sl_status_t sc = SL_STATUS_FAIL;
  //uint8_t operator = evt->data.evt_gatt_server_attribute_value.value.data[1];
  //uint8_t operand = evt->data.evt_gatt_server_attribute_value.value.data[2];
  uint8_t offset = evt->data.evt_gatt_server_attribute_value.value.data[3] | evt->data.evt_gatt_server_attribute_value.value.data[4] << 8;
  app_log("report records number greater than %d\n", offset);
  uint8_t reply[] = {0x05, 0x00, 0x00, 0x00};
  records_num = records_num - 1;
  reply[2] = records_num & 0xff;
  reply[3] = records_num  >> 8;
  sc = sl_bt_gatt_server_send_indication(
      connection,
      gattdb_record_access_control_point,
      sizeof(reply), reply
      );
  if (sc) {
    app_log_warning("Failed to report records number greater than %d\n", offset);
  }
}

/**************************************************************************//**
 * 4.11 CGMS/SEN/RAD/BV-01-C [Delete Stored Records – ‘All records’]
 *****************************************************************************/
void sl_bt_cgm_delete_all_num_records(uint8_t connection)
{
  sl_status_t sc = SL_STATUS_FAIL;
  records_num = 0;
  uint8_t cgm_delete_all_num_records_data[] = {0x06, 0x00, 0x02, 0x01};
  sc = sl_bt_gatt_server_send_indication(
      connection,
      gattdb_record_access_control_point,
      sizeof(cgm_delete_all_num_records_data), cgm_delete_all_num_records_data
      );
  if (sc) {
    app_log_warning("Failed to report all records number\n");
  }
}

/**************************************************************************//**
 * 4.11 CGMS/SEN/RAD/BV-02-C
 * [Delete Stored Records – ‘Within range of (inclusive) Time Offset value pair’]
 *****************************************************************************/
void sl_bt_cgm_delete_records_within_range(sl_bt_msg_t *evt)
{
  sl_status_t sc = SL_STATUS_FAIL;
  //uint8_t operator = evt->data.evt_gatt_server_attribute_value.value.data[1];
  //uint8_t operand = evt->data.evt_gatt_server_attribute_value.value.data[2];
  uint8_t offset = evt->data.evt_gatt_server_attribute_value.value.data[3] | evt->data.evt_gatt_server_attribute_value.value.data[4] << 8;
  records_num = records_num - offset;
  uint8_t reply[] = {0x06, 0x00, 0x02, 0x01};

  sc = sl_bt_gatt_server_send_indication(
      connection,
      gattdb_record_access_control_point,
      sizeof(reply), reply
      );
  if (sc) {
    app_log_warning("Failed to delete records within range\n");
  }
}
/**************************************************************************//**
 * 4.12 Record Access – Report Stored Records
 * CGMS/SEN/RAR/BV-01-C [Report Stored Records – ‘All records’]
 *****************************************************************************/
void sl_bt_cgm_report_all_records_bak(uint8_t connection)
{
  sl_status_t sc = SL_STATUS_FAIL;
  for(uint16_t i = 0; i < records_num; i++){
      if(abort_operation == true)
      {
        app_log("abort operation\n");
        break;
      }
      app_log("send %d\n",i);
      sl_bt_cgm_measurement_notificate();
  }
  if(abort_operation == false)
  {
    uint8_t cgm_report_all_num_records_data[] = {0x06, 0x00, 0x01, 0x01};
    sc = sl_bt_gatt_server_send_indication(
        connection,
        gattdb_record_access_control_point,
        sizeof(cgm_report_all_num_records_data), cgm_report_all_num_records_data
        );
    if (sc) {
      app_log_warning("Failed to report all records number\n");
    }
  }else{
      abort_operation = false;
  }
}
sl_simple_timer_t cgm_report_timer;
// CGMS/SEN/RAA/BV-01-C [Abort Operation – ‘Report Stored Records’]
// 100ms
uint16_t report_interval = 500;
bool procedure_in_progress = false;
void sl_bt_cgm_report_timer_cb(sl_simple_timer_t *timer, void *data)
{
  (void)data;
  (void)timer;
  sl_status_t sc = SL_STATUS_FAIL;
  static uint16_t num = 0;
  if(abort_operation == true)
  {
    app_log("abort operation in report records\n");
    sc = sl_simple_timer_stop(&cgm_report_timer);
    if(sc != SL_STATUS_OK){
        app_log("sl_simple_timer_stop failed 0x%04X\n", sc);
    }
    abort_operation = false;
    procedure_in_progress = false;
    num = 0;
    return;
  }
  app_log("send %d\n",num);
  sl_bt_cgm_measurement_notificate();
  num ++;
  if(num == records_num){
      app_log("finished send all records\n");
      num = 0;
      procedure_in_progress = false;
      sc = sl_simple_timer_stop(&cgm_report_timer);
      if(sc != SL_STATUS_OK){
          app_log("sl_simple_timer_stop failed 0x%04X\n", sc);
      }
     uint8_t cgm_report_all_num_records_data[] = {0x06, 0x00, 0x01, 0x01};
     sc = sl_bt_gatt_server_send_indication(
         connection,
         gattdb_record_access_control_point,
         sizeof(cgm_report_all_num_records_data), cgm_report_all_num_records_data
         );
     if (sc) {
       app_log_warning("Failed to report all records number\n");
     }
  }

}

void sl_bt_cgm_report_all_records(void)
{
  if(procedure_in_progress == false){
    procedure_in_progress = true;
  }
  sl_status_t sc = SL_STATUS_FAIL;
  sc = sl_simple_timer_start(&cgm_report_timer,
                             report_interval,
                             sl_bt_cgm_report_timer_cb,
                             NULL,
                             true);
  if(sc != SL_STATUS_OK){
      app_log("cgm_report_timer failed 0x%04X\n", sc);
  }
}

/**************************************************************************//**
 * 4.12
 * CGMS/SEN/RAR/BV-02-C [Report Stored Records – ‘Less than or equal to Time Offset’]
 *****************************************************************************/
void sl_bt_cgm_report_records_less_than(sl_bt_msg_t *evt)
{
  sl_status_t sc = SL_STATUS_FAIL;
  //uint8_t operator = evt->data.evt_gatt_server_attribute_value.value.data[1];
  //uint8_t filter_type = evt->data.evt_gatt_server_attribute_value.value.data[2];
  uint8_t maximum = evt->data.evt_gatt_server_attribute_value.value.data[3] | evt->data.evt_gatt_server_attribute_value.value.data[4] << 8;

  //uint8_t reply[] = {0x06, 0x00, 0x02, 0x01};
  for(uint8_t i = 0; i < maximum; i ++){
      if(abort_operation == true)
      {
          app_log("abort operation\n");
        break;
      }
      sl_bt_cgm_measurement_notificate();
  }
  if(abort_operation == false)
  {
    uint8_t cgm_report_all_num_records_data[] = {0x06, 0x00, 0x01, 0x01};
    sc = sl_bt_gatt_server_send_indication(
        connection,
        gattdb_record_access_control_point,
        sizeof(cgm_report_all_num_records_data), cgm_report_all_num_records_data
        );
    if (sc) {
      app_log_warning("Failed to report all records number\n");
    }
  }else{
      abort_operation = false;
  }
}


/**************************************************************************//**
 * 4.12 CGMS/SEN/RAR/BV-03-C
 * [Report Stored Records – ‘Greater than or equal to Time Offset’]
 *****************************************************************************/
void sl_bt_cgm_report_records_greater_than(sl_bt_msg_t *evt)
{
  sl_status_t sc = SL_STATUS_FAIL;
  //uint8_t operator = evt->data.evt_gatt_server_attribute_value.value.data[1];
  //uint8_t filter_type = evt->data.evt_gatt_server_attribute_value.value.data[2];
  uint8_t minimum = evt->data.evt_gatt_server_attribute_value.value.data[3] | evt->data.evt_gatt_server_attribute_value.value.data[4] << 8;

  //uint8_t reply[] = {0x06, 0x00, 0x02, 0x01};
  for(uint8_t i = minimum - 1; i < records_num; i ++){
      if(abort_operation == true)
      {
          app_log("abort operation\n");
        break;
      }
      sl_bt_cgm_measurement_notificate();
  }
  if(abort_operation == false)
  {
    uint8_t cgm_report_all_num_records_data[] = {0x06, 0x00, 0x01, 0x01};
    sc = sl_bt_gatt_server_send_indication(
        connection,
        gattdb_record_access_control_point,
        sizeof(cgm_report_all_num_records_data), cgm_report_all_num_records_data
        );
    if (sc) {
      app_log_warning("Failed to report all records number\n");
    }
  }else{
      abort_operation = false;
  }
}

/**************************************************************************//**
 * 4.12 CGMS/SEN/RAR/BV-04-C
 * [Report Stored Records – ‘Within range of (inclusive) Time Offset value pair’]
 *****************************************************************************/
void sl_bt_cgm_report_records_within_range(sl_bt_msg_t *evt)
{

  sl_status_t sc = SL_STATUS_FAIL;
  uint8_t len = evt->data.evt_gatt_server_attribute_value.value.len;
  uint8_t opcode = evt->data.evt_gatt_server_attribute_value.value.data[0];
  //uint8_t operator = evt->data.evt_gatt_server_attribute_value.value.data[1];


  if(len != 7){
      app_log("wrong data\n");
      return;
  }
  uint16_t min = evt->data.evt_gatt_server_attribute_value.value.data[3] | evt->data.evt_gatt_server_attribute_value.value.data[4] << 8;
  uint16_t max = evt->data.evt_gatt_server_attribute_value.value.data[5] | evt->data.evt_gatt_server_attribute_value.value.data[6] << 8;
  if(min > max){
      app_log("Invalid Operand Type 2\n");
      sl_bt_cgm_send_racp_indication(opcode, 0x05);
      return;
  }


  //uint8_t reply[] = {0x06, 0x00, 0x02, 0x01};
  for(uint8_t i = min; i < max + 1; i ++){
    if(abort_operation == true)
    {
      app_log("abort operation\n");
      break;
    }
    sl_bt_cgm_measurement_notificate();
  }
  if(abort_operation == false)
  {
    uint8_t cgm_report_all_num_records_data[] = {0x06, 0x00, 0x01, 0x01};
    sc = sl_bt_gatt_server_send_indication(
        connection,
        gattdb_record_access_control_point,
        sizeof(cgm_report_all_num_records_data), cgm_report_all_num_records_data
        );
    if (sc) {
      app_log_warning("Failed to report all records number\n");
    }
  }else{
      abort_operation = false;
  }
}


/**************************************************************************//**
 * 4.12 CGMS/SEN/RAR/BV-05-C [Report Stored Records – ‘First record’]
 *****************************************************************************/
void sl_bt_cgm_report_first_record()
{
  sl_status_t sc = SL_STATUS_FAIL;

  //uint8_t reply[] = {0x06, 0x00, 0x02, 0x01};
  if(abort_operation == true)
  {
    app_log("abort operation\n");
    return;
  }
  uint8_t first_record[] = {0x0D,0xE3,04,00,00,00,01,01,01,00,00,00,00};
  sc = sl_bt_gatt_server_send_notification(
    connection,
    gattdb_cgm_measurement,
    sizeof(first_record),
    first_record
    );
  if(abort_operation == false)
  {
    uint8_t cgm_report_all_num_records_data[] = {0x06, 0x00, 0x01, 0x01};
    sc = sl_bt_gatt_server_send_indication(
        connection,
        gattdb_record_access_control_point,
        sizeof(cgm_report_all_num_records_data), cgm_report_all_num_records_data
        );
    if (sc) {
      app_log_warning("Failed to report all records number\n");
    }
  }else{
      abort_operation = false;
  }
}


/**************************************************************************//**
 * 4.12 CGMS/SEN/RAR/BV-06-C [Report Stored Records – ‘Last record’]
 *****************************************************************************/
void sl_bt_cgm_report_last_record()
{
  sl_status_t sc = SL_STATUS_FAIL;

  //uint8_t reply[] = {0x06, 0x00, 0x02, 0x01};
  if(abort_operation == true)
  {
    app_log("abort operation\n");
    return;
  }
  uint8_t last_record[] = {0x0D,0xE3,0x08,00,0x08,00,01,01,01,00,00,00,00};
  sc = sl_bt_gatt_server_send_notification(
    connection,
    gattdb_cgm_measurement,
    sizeof(last_record),
    last_record
    );
  if(abort_operation == false)
  {
    uint8_t cgm_report_all_num_records_data[] = {0x06, 0x00, 0x01, 0x01};
    sc = sl_bt_gatt_server_send_indication(
        connection,
        gattdb_record_access_control_point,
        sizeof(cgm_report_all_num_records_data), cgm_report_all_num_records_data
        );
    if (sc) {
      app_log_warning("Failed to report all records number\n");
    }
  }else{
      abort_operation = false;
  }
}
