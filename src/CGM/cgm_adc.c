/***************************************************************************//**
 * @file
 * @brief handle all ADC GATT operations
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
#include <mis.h>
#include <stdbool.h>
#include <stdio.h>

#include <stdlib.h>

void sl_bt_set_dac(sl_bt_msg_t *evt)
{
  uint8_t connection = evt->data.evt_gatt_server_attribute_value.connection;
//  uint8_t len = evt->data.evt_gatt_server_attribute_value.value.len;
//  uint8_t valueInt = evt->data.evt_gatt_server_attribute_value.value.data[0];
//  uint8_t lenFrac = len -2;
  sl_bt_gatt_server_send_user_write_response(connection, gattdb_dac_adjust, 0);
  float voltValue = atof((char *)evt->data.evt_gatt_server_attribute_value.value.data);
  dac70501_setVolt(voltValue);

}

void sl_bt_get_dac_level(sl_bt_msg_t *evt)
{
  uint8_t connection = evt->data.evt_gatt_server_attribute_value.connection;

  uint32_t valueInt, valueFrac;
  float dacValue = dac70501_readRef();
  valueInt = (uint32_t) (dacValue * 1000);
  valueFrac = (uint32_t) ((dacValue * 1000 - valueInt) * 1000);
  app_log("read dac voltage %d.%dmV\r\n", valueInt, valueFrac);
  uint8_t value[20] = {'.'};
  //adc_to_array(valueInt, valueFram, value);
  uint8_t i = snprintf((char *)value, 10, "%lu", valueInt);
  value[i] = '.';
  uint8_t j = snprintf((char *)&value[i+1], 10, "%lu", valueFrac);
  value[i+j+1] = 'm';
  value[i+j+2] = 'V';
  app_log("%s\n", value);
  uint16_t sent_len;
  sl_bt_gatt_server_send_user_read_response(connection,
                                            gattdb_dac_adjust,
                                            0,
                                            i+j+3,
                                            value,
                                            &sent_len);
}


void sl_bt_get_internal_adc(sl_bt_msg_t *evt)
{
  uint8_t connection = evt->data.evt_gatt_server_attribute_value.connection;

  uint32_t valueInt, valueFram;
  double glucoseValue = iadcPollSingleResult();
  valueInt = (uint32_t) (glucoseValue * 1000);
  valueFram = (uint32_t) ((glucoseValue * 1000 - valueInt) * 1000);
  app_log("iadc %d.%d mV\r\n", valueInt, valueFram);
  uint8_t value[20] = {'.'};
  //adc_to_array(valueInt, valueFram, value);
  uint8_t i = snprintf((char *)value, 10, "%lu", valueInt);
  value[i] = '.';
  uint8_t j = snprintf((char *)&value[i+1], 10, "%lu", valueFram);
  value[i+j+1] = 'm';
  value[i+j+2] = 'V';
  //app_log("%s\n", value);
  uint16_t sent_len;
  sl_bt_gatt_server_send_user_read_response(connection,
                                            gattdb_internal_adc,
                                            0,
                                            i+j+3,
                                            value,
                                            &sent_len);
}



 void sl_bt_get_external_adc(sl_bt_msg_t *evt)
 {
   uint8_t connection = evt->data.evt_gatt_server_attribute_value.connection;

   uint32_t valueInt, valueFram;
   double glucoseValue = ads1220_getAdcDataVolt();
   valueInt = (uint32_t) (glucoseValue * 1000);
   valueFram = (uint32_t) ((glucoseValue * 1000 - valueInt) * 1000);
   app_log("iadc external %d.%dmV\r\n", valueInt, valueFram);
   uint8_t value[20] = {'.'};
   uint8_t i = snprintf((char *)value, 10, "%lu", valueInt);
   value[i] = '.';
   uint8_t j = snprintf((char *)&value[i+1], 10, "%lu", valueFram);
   value[i+j+1] = 'm';
   value[i+j+2] = 'V';
   app_log("%s\n", value);
   uint16_t sent_len;
   sl_bt_gatt_server_send_user_read_response(connection,
                                             gattdb_external_adc,
                                             0,
                                             i+j+3,
                                             value,
                                             &sent_len);
 }

void sl_bt_get_temp(sl_bt_msg_t *evt)
{
  uint8_t connection = evt->data.evt_gatt_server_attribute_value.connection;
  float dieTemp = getDieTemperature();
  uint32_t valueInt, valueFram;
  valueInt = (uint32_t) (dieTemp);
  valueFram = (uint32_t) (dieTemp * 1000 - valueInt*1000);
  app_log("Die temperature %d.%d\r\n", valueInt, valueFram);
  uint8_t value[20] = {'.'};
  uint8_t i = snprintf((char *)value, 10, "%lu", valueInt);
  value[i] = '.';
  uint8_t j = snprintf((char *)&value[i+1], 10, "%lu", valueFram);
  value[i+j+1] = 'C';
  app_log("%s\n", value);
  uint16_t sent_len;
  sl_bt_gatt_server_send_user_read_response(connection,
                                            gattdb_temp,
                                            0,
                                            i+j+2,
                                            value,
                                            &sent_len);
}


void sl_bt_get_enob(sl_bt_msg_t *evt)
{
  uint8_t connection = evt->data.evt_gatt_server_attribute_value.connection;
  float dieTemp = getDieTemperature();
  uint32_t valueInt, valueFram;
  valueInt = (uint32_t) (dieTemp);
  valueFram = (uint32_t) (dieTemp * 1000 - valueInt*1000);
  app_log("Die temperature %d.%d\r\n", valueInt, valueFram);
  uint8_t value[20] = {'.'};
  //adc_to_array(valueInt, valueFram, value);
  uint8_t i = snprintf((char *)value, 10, "%lu", valueInt);
  value[i] = '.';
  uint8_t j = snprintf((char *)&value[i+1], 10, "%lu", valueFram);
  value[i+j+1] = 'C';
  app_log("%s\n", value);
  uint16_t sent_len;
  sl_bt_gatt_server_send_user_read_response(connection,
                                            gattdb_temp,
                                            0,
                                            i+j+2,
                                            value,
                                            &sent_len);
}

void sl_bt_read_offset(sl_bt_msg_t *evt)
{
  uint8_t connection = evt->data.evt_gatt_server_attribute_value.connection;

  uint32_t valueInt, valueFram;
  valueInt = (uint32_t) (adcOffsetresult);
  valueFram = (uint32_t) ((adcOffsetresult - valueInt) * 1000);
  app_log("iadc intenal adc offset %d.%d\r\n", valueInt, valueFram);
  uint8_t value[20] = {'.'};
  uint8_t i = snprintf((char *)value, 10, "%lu", valueInt);
  value[i] = '.';
  uint8_t j = snprintf((char *)&value[i+1], 10, "%lu", valueFram);
  value[i+j+1] = 'm';
  value[i+j+2] = 'V';
  app_log("%s\n", value);
  uint16_t sent_len;
  sl_bt_gatt_server_send_user_read_response(connection,
                                            gattdb_external_adc,
                                            0,
                                            i+j+3,
                                            value,
                                            &sent_len);
}


void sl_bt_read_gain_error(sl_bt_msg_t *evt)
{
  uint8_t connection = evt->data.evt_gatt_server_attribute_value.connection;

  uint32_t valueInt, valueFram;
  valueInt = (uint32_t) (adcGainResult);
  valueFram = (uint32_t) ((adcGainResult - valueInt) * 1000);
  app_log("iadc intenal adc gain error %d.%d\r\n", valueInt, valueFram);
  uint8_t value[20] = {'.'};
  uint8_t i = snprintf((char *)value, 10, "%lu", valueInt);
  value[i] = '.';
  uint8_t j = snprintf((char *)&value[i+1], 10, "%lu", valueFram);
  app_log("%s\n", value);
  uint16_t sent_len;
  sl_bt_gatt_server_send_user_read_response(connection,
                                            gattdb_external_adc,
                                            0,
                                            i+j+1,
                                            value,
                                            &sent_len);
}
