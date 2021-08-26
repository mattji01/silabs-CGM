/***************************************************************************//**
 * @file
 * @brief CGM header file
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


#ifndef CGM_H_
#define CGM_H_

#include "stdbool.h"
#include "gatt_db.h"
#include "sl_status.h"
#include "sl_bt_api.h"
#include "app_log.h"
#include "app_assert.h"
#include "sl_simple_timer.h"

typedef struct
{
  uint16_t high_alert_value;
  uint16_t low_alert_value;
  uint16_t hypo_alert_level;
  uint16_t hyper_alert_level;
  uint16_t rate_decrease_alert_value;
  uint16_t rate_increase_alert_value;
}sl_bt_cgm_init_value_t;

extern sl_bt_cgm_init_value_t cgm_init;
extern bool procedure_in_progress;

// Macros.
#define UINT16_TO_BYTES(n)            ((uint8_t) (n)), ((uint8_t)((n) >> 8))
#define UINT16_TO_BYTE0(n)            ((uint8_t) (n))
#define UINT16_TO_BYTE1(n)            ((uint8_t) ((n) >> 8))
#define BYTES_TO_UINT16(n,m)          (n | m << 8)


// Glucose measurement current record number
extern uint16_t records_num;

extern uint8_t comm_interval;
extern uint8_t bonding;

extern double adcGainResult;
extern double adcOffsetresult;

// <o GLUCOSE_MEAS_INTERVAL_SEC>
// <i> Default: 2s
// <i> The Measurement Interval is the time interval between two CGM Measurement notifications.
#define GLUCOSE_DEFAULT_COMM_INTERVAL_SEC   2

#define MIN_CGM_CONCENTRATION       20

extern bool measure_indicate_enabled;
extern bool session_started;
extern bool racp_indicate_enabled;
extern uint8_t connection;
extern sl_simple_timer_t cgm_periodic_timer;
extern bool abort_operation;
extern uint8_t cgm_status[];


#define STATUS_LEN 7
#define SL_BT_CGM_STATUS_DEVICE_SEPCIFIC_ALERT  (0x01 << 4)
#define SL_BT_CGM_STATUS_SESSION_STOPPED  (0x01 << 0)

#define SL_BT_CGM_OPCODE_REPORT_STORED_REPORT 0x01

//CGM Measurement flags
#define SL_BT_CGM_FLAG_TREND_INFO_PRESENT                 0x01
#define SL_BT_CGM_FLAGS_QUALITY_PRESENT                   0x02
#define SL_BT_CGM_STATUS_ANNUNCIATION_WARNING_PRESENT        0x20
#define SL_BT_CGM_STATUS_ANNUNCIATION_CALTEMP_PRESENT        0x40
#define SL_BT_CGM_STATUS_ANNUNCIATION_STATUS_PRESENT         0x80

// Feature characteristic support
extern uint8_t feature[];
#define FEATURE_LEN 6
#define SL_BT_CGM_E2E_CRC_SUPPORTED 0
#define SL_BT_CGM_MULTI_BOND_SUPPORTED 0
#define SL_BT_CGM_MULTI_SESSION_SUPPORTED 1
#define SL_BT_CGM_E2E_CRC  (0x01 << 12)
#define SL_BT_CGM_MULTI_BOND  (0x01 << 13)
#define SL_BT_CGM_MULTI_SESSION  (0x01 << 14)
#define SL_BT_CGM_SAMPLE_LOCATION 0x02
#define SL_BT_CGM_TYPE 0x03
#define SL_BT_CGM_TREND_INFO_SUPPORTED 0
#define SL_BT_CGM_QUALITY_SUPPORTED 0



#define SL_BT_CGM_GET_HYPO_ALERT_LEVEL_OPCODE 0x0E
#define SL_BT_CGM_GET_HYPO_ALERT_LEVEL_RSP_OPCODE 0x0F
#define SL_BT_CGM_GET_HYPER_ALERT_LEVEL_OPCODE 0x11
#define SL_BT_CGM_GET_HYPER_ALERT_LEVEL_RSP_OPCODE 0x12
#define SL_BT_CGM_GET_RATE_DECREASE_ALERT_LEVEL_OPCODE 0x14
#define SL_BT_CGM_GET_RATE_DECREASE_ALERT_LEVEL_RSP_OPCODE 0x15
#define SL_BT_CGM_GET_RATE_INCREASE_ALERT_LEVEL_OPCODE 0x17
#define SL_BT_CGM_GET_RATE_INCREASE_ALERT_LEVEL_RSP_OPCODE 0x18
#define SL_BT_CGM_RESET_DEVICE_SPECIFIC_ALERT_OPCODE  0x19

/**************************************************************************//**
 * Bluetooth stack event handler.
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_cgm_on_event(sl_bt_msg_t *evt);

/**************************************************************************//**
 * RACP event handler.
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_cgm_racp_indication_handler(sl_bt_msg_t *evt);
void sl_bt_cgm_send_racp_indication(uint8_t opcode, uint8_t error);
void sl_bt_cgm_racp_handler(sl_bt_msg_t *evt);
void sl_bt_cgm_report_record(sl_bt_msg_t *evt);
void sl_bt_cgm_abort_operation(sl_bt_msg_t *evt);
void sl_bt_cgm_report_num_records(sl_bt_msg_t *evt);
void sl_bt_cgm_delete_record(sl_bt_msg_t *evt);
void sl_bt_cgm_report_all_num_records(uint8_t connection);
void sl_bt_cgm_delete_all_num_records(uint8_t connection);
void sl_bt_cgm_report_all_records(void);

/**************************************************************************//**
 * Measurement characteristic event handler.
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_cgm_measurement_notification_handler(sl_bt_msg_t *evt);

/**************************************************************************//**
 * Specific ops event handler.
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_cgm_sops_indication_handler(sl_bt_msg_t *evt);
void sl_bt_cgm_handle_sops(sl_bt_msg_t *evt);
void sl_bt_cgm_send_sops_indicate(uint8_t opcode, uint8_t response_code);
void sl_bt_cgm_communication_interval_procedures(uint8_t interval);
void sl_bt_cgm_sops_reply_comm_interval(void);
void sl_bt_cgm_get_cal_value(sl_bt_msg_t *evt);
void sl_bt_cgm_set_cal_value(sl_bt_msg_t *evt);

// Session start time
#define SL_BT_CGM_SST_LEN 9
#define SL_BT_CGM_SST_WITHCRC_LEN 11
void sl_bt_cgm_handle_sst_write(sl_bt_msg_t *evt);
void sl_bt_cgm_handle_sst_read(void);


// status characteristic
void sl_bt_cgm_handle_status_read(void);
void sl_bt_cgm_handle_status_write(sl_bt_msg_t *evt);

// Measurement characteristic
void sl_bt_cgm_periodic_timer_cb(sl_simple_timer_t *timer, void *data);
void sl_bt_cgm_measurement_notificate(void);

uint16_t crc16(uint16_t crc, uint8_t *data, size_t len);

void sl_bt_cgm_get_alert_level(sl_bt_msg_t *evt);
void sl_bt_cgm_set_high_alert_level(sl_bt_msg_t *evt);
void sl_bt_cgm_set_low_alert_level(sl_bt_msg_t *evt);

void sl_bt_cgm_report_records_less_than(sl_bt_msg_t *evt);
void sl_bt_cgm_report_records_greater_than(sl_bt_msg_t *evt);
void sl_bt_cgm_report_records_within_range(sl_bt_msg_t *evt);
void sl_bt_cgm_report_first_record();
void sl_bt_cgm_report_last_record();

void sl_bt_cgm_report_greater_num_records(sl_bt_msg_t *evt);
void sl_bt_cgm_delete_records_within_range(sl_bt_msg_t *evt);
void sl_bt_cgm_report_greater_num_records(sl_bt_msg_t *evt);
void sl_bt_cgm_report_greater_num_records(sl_bt_msg_t *evt);
void sl_bt_cgm_set_hypo_alert_level(sl_bt_msg_t *evt);
void sl_bt_cgm_set_hyper_alert_level(sl_bt_msg_t *evt);
void sl_bt_cgm_set_rate_decrease_alert_level(sl_bt_msg_t *evt);
void sl_bt_cgm_set_rate_increase_alert_level(sl_bt_msg_t *evt);
void sl_bt_cgm_reset_device_special_alert(void);

void sl_bt_cgm_init_database(void);
void sl_bt_cgm_handle_feature_read(void);
void sl_bt_cgm_handle_run_time_read(void);

void sl_bt_set_dac(sl_bt_msg_t *evt);
void sl_bt_get_dac_level(sl_bt_msg_t *evt);
void sl_bt_get_internal_adc(sl_bt_msg_t *evt);
void sl_bt_get_external_adc(sl_bt_msg_t *evt);
void sl_bt_get_temp(sl_bt_msg_t *evt);
void sl_bt_get_enob(sl_bt_msg_t *evt);
void sl_bt_read_offset(sl_bt_msg_t *evt);
void sl_bt_read_gain_error(sl_bt_msg_t *evt);



#endif /* CGM_H_ */

