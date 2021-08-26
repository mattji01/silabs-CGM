/***************************************************************************//**
 * @file
 * @brief CGM GATT service
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

#include "sl_bluetooth.h"
#include "cgm.h"
#include "gatt_db.h"
#include "sl_simple_timer.h"
#include "app_log.h"
#include "app_assert.h"


// Connection handle. At any given time, a CGM Sensor shall be connected to only one Collector
uint8_t connection = 0xff;
uint8_t bonding = 0xff;
static uint8_t advertising_set_handle = 0xff;
uint8_t type;
bd_addr address;



void sl_bt_create_user_adv(void)
{
  sl_status_t sc;

  uint8_t temp;
  if(type == 0){
      temp = 0x17;
  }else{
      temp = 0x18;
  }
  const uint8_t value[] = {0x02,0x01,0x06,0x03,0x03,0x1F,0x18,0x0B,0x08,0x73,\
            0x69,0x6C,0x61,0x62,0x73,0x2D,0x63,0x67,0x6D,0x03,0x19,0x0D,0x00,\
        0x07,temp,address.addr[0],address.addr[1],address.addr[2],address.addr[3],address.addr[4],address.addr[5]};
  sc = sl_bt_advertiser_set_data(advertising_set_handle, 0,sizeof(value),value);
   app_assert_status(sc);
   // Start general advertising and enable connections.
   sc = sl_bt_advertiser_start(
     advertising_set_handle,
     //sl_bt_advertiser_general_discoverable,
     sl_bt_advertiser_user_data,
     sl_bt_advertiser_connectable_scannable);
   sc = sl_bt_gap_enable_whitelisting(1);
}


/**************************************************************************//**
 * Callback function of connection close event.
 *
 * @param[in] reason Unused parameter required by the health_thermometer component
 * @param[in] connection Unused parameter required by the health_thermometer component
 *****************************************************************************/
void sl_bt_connection_closed_cb(uint16_t reason, uint8_t connection)
{
  (void)reason;
  (void)connection;
  sl_status_t sc;
  app_log_info("central disconnect\n");
  // Stop timer.
  sc = sl_simple_timer_stop(&cgm_periodic_timer);
  if(sc != SL_STATUS_OK){
      app_log_info("sl_simple_timer_stop failed 0x%04X\n", sc);
  }
  connection = 0xff;
  bonding = 0xff;
  sl_bt_create_user_adv();
}


void list_bondings(void)
{
  uint32_t num_bondings;
  size_t bondings_len;
  uint8_t bondings[10];
  sl_status_t sc;
  uint8_t address_type;
  uint8_t security_mode;
  uint8_t key_size;
  sl_bt_sm_get_bonding_handles(0,
                               &num_bondings,
                               10,
                               &bondings_len,
                               bondings);
  if(num_bondings > 0){
    for(uint8_t i = 0; i< num_bondings;i++){
      sc = sl_bt_sm_get_bonding_details(i,
                                       &address,
                                       &address_type,
                                       &security_mode,
                                       &key_size);
      app_assert_status(sc);
      app_log("address type %d, bonding handler %d: ", address_type, i);
      for(uint8_t i = 0; i < 6; i++){
          app_log("0x%02X ",address.addr[i]);
      }
      app_log("\n");
    }
  }
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 *****************************************************************************/
void sl_bt_cgm_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  // Handle stack events
  uint8_t handler;
  switch (SL_BT_MSG_ID(evt->header)) {
    case sl_bt_evt_system_boot_id:
      {
        sc = sl_bt_advertiser_create_set(&advertising_set_handle);
        app_assert_status(sc);
        uint8_t flags = 0x0E;
        uint8_t io_capabilities = sl_bt_sm_io_capability_noinputnooutput;
        // The advertising set handle allocated from Bluetooth stack.
        app_log_info("system boot in CGM\n");
        sc = sl_bt_sm_set_bondable_mode(1);
        sc = sl_bt_sm_configure(flags, io_capabilities);
        app_log_info("set security\n");
        list_bondings();
        sl_bt_cgm_init_database();
        //sc = sl_bt_system_get_identity_address(&address, &address_type);
        //app_assert_status(sc);
        sl_bt_create_user_adv();
      }
      break;

      // -------------------------------
      // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      {
       connection = evt->data.evt_connection_opened.connection;
       bonding = evt->data.evt_connection_opened.bonding;
       type = evt->data.evt_connection_opened.address_type;
       app_log("client address type is %d\n", type);
       address = evt->data.evt_connection_opened.address;
       for(uint8_t i = 0; i < 6; i++){
           app_log("0x%02X ",address.addr[i]);
       }
       app_log("\n");
       app_log_info("current connection handler is %d， bonding 0x%02X\n", connection, bonding);
       if(bonding == 0xFF){
           sc = sl_bt_sm_increase_security(connection);
           app_log_info("start pairing\n");
       }else{
         app_log_info("already bonded, binding handler 0x%02x\n", bonding);
         sc = sl_bt_sm_increase_security(connection);
       }
      }
      break;

    case sl_bt_evt_connection_closed_id:
      sl_bt_connection_closed_cb(evt->data.evt_connection_closed.reason,
                                 evt->data.evt_connection_closed.connection);
      break;
// RACP SOPS indication handler
    case sl_bt_evt_gatt_server_characteristic_status_id:
    {
      uint16_t handler = evt->data.evt_gatt_server_characteristic_status.characteristic;
      switch(handler){
        case gattdb_cgm_measurement:
          sl_bt_cgm_measurement_notification_handler(evt);
        break;
        case gattdb_cgm_specific_ops_control_point:
          sl_bt_cgm_sops_indication_handler(evt);
          break;
        case gattdb_record_access_control_point:
          sl_bt_cgm_racp_indication_handler(evt);
          break;
        default:
          //app_log_info("GATT handler is %d\n", handler);
          break;
      }
      break;
    }

// characteristic hex type
    case sl_bt_evt_gatt_server_attribute_value_id:
      handler = evt->data.evt_gatt_server_attribute_value.attribute;
      switch(handler){
        case gattdb_cgm_status:
          //cgm_handle_status(evt);
          break;
      }

      break;

 // characteristic user type write

    case sl_bt_evt_gatt_server_user_write_request_id:
      handler = evt->data.evt_gatt_server_user_write_request.characteristic;
      switch(handler){
        case gattdb_cgm_specific_ops_control_point:
          app_log_info("client write specific ops\n");
          sl_bt_cgm_handle_sops(evt);
          break;
        case gattdb_cgm_status:
          //sl_bt_cgm_handle_status_write(evt);
          break;
        case gattdb_cgm_session_start_time:
          sl_bt_cgm_handle_sst_write(evt);
          break;
        case gattdb_record_access_control_point:
          app_log_info("client write RACP: \n");
          sl_bt_cgm_racp_handler(evt);
          break;
        case gattdb_dac_adjust:
          app_log("set dac level\n");
          sl_bt_set_dac(evt);
          break;
        default:
          app_log("wrong write handler\n");
          break;

      }
      break;

// characteristic user type read
    case sl_bt_evt_gatt_server_user_read_request_id:
      handler = evt->data.evt_gatt_server_user_read_request.characteristic;
      switch(handler){
        case gattdb_cgm_session_start_time:
          sl_bt_cgm_handle_sst_read();
        break;
        case gattdb_cgm_status:
           sl_bt_cgm_handle_status_read();
         break;
        case gattdb_cgm_feature:
          sl_bt_cgm_handle_feature_read();
          break;
        case gattdb_cgm_session_run_time:
          sl_bt_cgm_handle_run_time_read();
          break;
        case gattdb_internal_adc:
          sl_bt_get_internal_adc(evt);
          break;
        case gattdb_external_adc:
          sl_bt_get_external_adc(evt);
          break;
        case gattdb_temp:
          sl_bt_get_temp(evt);
          break;
        case gattdb_enob:
          sl_bt_get_enob(evt);
          break;
        case gattdb_dac_adjust:
          sl_bt_get_dac_level(evt);
          break;
        case gattdb_offset:
          sl_bt_read_offset(evt);
          break;
        case gattdb_gain_error:
          sl_bt_read_gain_error(evt);
          break;
        default:
          app_log("wrong read handler %d\n", handler);
          break;
      }
      break;

      // other events
    case sl_bt_evt_connection_parameters_id:
      app_log_info("security level %d\n", evt->data.evt_connection_parameters.security_mode+1);
      break;
    case sl_bt_evt_sm_bonded_id:
      app_log_info("bonded, bonding handler is 0x%02X\n", evt->data.evt_sm_bonded.bonding);
      list_bondings();
      break;
    case sl_bt_evt_sm_bonding_failed_id:
      app_log_info("bonding failed 0x%04X\n", evt->data.evt_sm_bonding_failed.reason);
      break;
    case sl_bt_evt_sm_confirm_passkey_id:
      app_log_info("sl_bt_evt_sm_confirm_passkey_id\n");
      break;
    case sl_bt_evt_gatt_mtu_exchanged_id:
      app_log_info("mtu is %d\n", evt->data.evt_gatt_mtu_exchanged.mtu);
      break;
    case sl_bt_evt_connection_phy_status_id:
      app_log_info("phy is %d\n", evt->data.evt_connection_phy_status.phy);
      break;
    case sl_bt_evt_connection_remote_used_features_id:
      app_log_info("remote phy feature\n");
      break;
    case sl_bt_evt_sm_confirm_bonding_id:
      app_log_info("sl_bt_evt_sm_confirm_bonding_id\n");
      sc = sl_bt_sm_bonding_confirm(evt->data.evt_sm_confirm_bonding.connection, 1);
      if(sc != SL_STATUS_OK){
          app_log_info("sl_bt_sm_bonding_confirm failed\n");
      }
      break;
    default:
      app_log_info("unknown evt 0x%08x\n",evt->header);
      break;
  }
}
