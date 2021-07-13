/**
 * Copyright (c) 2012 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Reinfotribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Reinfotributions of source code must retain the above copyright notice, this
 *    list of conditions and the following infoclaimer.
 *
 * 2. Reinfotributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following infoclaimer in the documentation and/or other
 *    materials provided with the infotribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or infoassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * infoCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/* Attention!
 * To maintain compliance with Nordic Semiconductor ASA's Bluetooth profile
 * qualification listings, this section of source code must not be modified.
 */
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_DIS)
#include "ble_info_service.h"

#include <stdlib.h>
#include <string.h>
#include "app_error.h"
#include "ble_gatts.h"
#include "ble_srv_common.h"

static uint16_t service_handle;
static ble_gatts_char_handles_t sys_id_handles;
static ble_gatts_char_handles_t Firmware_handles;
static ble_gatts_char_handles_t serial_num_handles;
static ble_gatts_char_handles_t hw_rev_handles;
static ble_gatts_char_handles_t fw_rev_handles;
static ble_gatts_char_handles_t date_handles;

static uint8_t address[ADDRESS_LEN]             = {0};
static uint8_t Firmware[FIRMWARE_LEN]           = {'2', '0', '2', '1', '-', '0', '7', '-', '1', '2'};
static uint8_t Serial_Number[SERIAL_NUMBER_LEN] = {'a', '0', '2', 'c', '2', 'f', '1', '5', '1', 'b', '1', '5'};
static uint8_t fw[FIRMWARE_REVISION_LEN]        = {'F', 'W', 'V', 'E', 'R', 'S', 'I', 'O', 'N', '_', '0', '0', '0', '1', '\0'};
static uint8_t hw[HARDWARE_VERSION_LEN]         = {'H', 'W', 'V', 'E', 'R', 'S', 'I', 'O', 'N', '_', '0', '0', '0', '1', '\0'};
static uint8_t date[DATE_LEN]                   = {'2', '0', '2', '1', '-', '0', '7', '-', '1', '1'};

uint32_t
ble_info_service_init(ble_info_init_t const *p_info_init)
{
  uint32_t err_code;
  uint8_t i;
  ble_uuid_t ble_uuid;
  ble_add_char_params_t add_char_params;
  ble_gap_addr_t bleAddr;

  sd_ble_gap_addr_get(&bleAddr);
  // Add service
  BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_DEVICE_INFORMATION_SERVICE);

  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &service_handle);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  for (i = 0; i < 6; i++)
  {
    address[i] = bleAddr.addr[5 - i];
  }
  // Add Firmware characteristics
  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = BLE_UUID_SYSTEM_ID_CHAR;
  add_char_params.max_len = ADDRESS_LEN;
  add_char_params.p_init_value = address;
  add_char_params.init_len = ADDRESS_LEN;
  add_char_params.is_var_len = true;
  add_char_params.char_props.read = 1;

  add_char_params.read_access = SEC_OPEN;
  err_code = characteristic_add(service_handle, &add_char_params, &sys_id_handles);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  // Add Firmware characteristics
  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = BLE_UUID_MODEL_NUMBER_STRING_CHAR;
  add_char_params.max_len = FIRMWARE_LEN;
  add_char_params.p_init_value = Firmware;
  add_char_params.init_len = FIRMWARE_LEN;
  add_char_params.is_var_len = true;
  add_char_params.char_props.read = 1;

  add_char_params.read_access = SEC_OPEN;
  err_code = characteristic_add(service_handle, &add_char_params, &Firmware_handles);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  // Add Serial Number characteristics
  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = BLE_UUID_SERIAL_NUMBER_STRING_CHAR;
  add_char_params.max_len = SERIAL_NUMBER_LEN;
  add_char_params.p_init_value = Serial_Number;
  add_char_params.init_len = SERIAL_NUMBER_LEN;
  add_char_params.is_var_len = true;
  add_char_params.char_props.read = 1;

  add_char_params.read_access = SEC_OPEN;
  err_code = characteristic_add(service_handle, &add_char_params, &serial_num_handles);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  // Add Firmware Revision characteristics
  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = BLE_UUID_FIRMWARE_REVISION_STRING_CHAR;
  add_char_params.max_len = FIRMWARE_REVISION_LEN;
  add_char_params.p_init_value = fw;
  add_char_params.init_len = FIRMWARE_REVISION_LEN;
  add_char_params.is_var_len = true;
  add_char_params.char_props.read = 1;

  add_char_params.read_access = SEC_OPEN;
  err_code = characteristic_add(service_handle, &add_char_params, &fw_rev_handles);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  // Add Hardware Revision characteristics
  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = BLE_UUID_HARDWARE_REVISION_STRING_CHAR;
  add_char_params.max_len = HARDWARE_VERSION_LEN;
  add_char_params.p_init_value = hw;
  add_char_params.init_len = HARDWARE_VERSION_LEN;
  add_char_params.is_var_len = true;
  add_char_params.char_props.read = 1;

  add_char_params.read_access = SEC_OPEN;
  err_code = characteristic_add(service_handle, &add_char_params, &hw_rev_handles);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  // Add Manufacture Date characteristics
  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = BLE_UUID_CURRENT_TIME_CHAR;
  add_char_params.max_len = DATE_LEN;
  add_char_params.p_init_value = date;
  add_char_params.init_len = DATE_LEN;
  add_char_params.is_var_len = true;
  add_char_params.char_props.read = 1;

  add_char_params.read_access = SEC_OPEN;
  err_code = characteristic_add(service_handle, &add_char_params, &date_handles);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  return NRF_SUCCESS;
}

void InfoService_SetParameter(uint8_t param, uint8_t len, void *value)
{
    switch (param)
    {
    case FIRMWARE_CHAR:
        if (len == FIRMWARE_LEN)
        {
            memcpy(Firmware, value, FIRMWARE_LEN);
        }
        break;
    case SERIAL_NUMBER_CHAR:
        if (len == SERIAL_NUMBER_LEN)
        {
            memcpy(Serial_Number, value, SERIAL_NUMBER_LEN);
        }
        break;
    case FIRMWARE_REVISION_CHAR:
        if (len == FIRMWARE_REVISION_LEN)
        {
            memcpy(fw, value, FIRMWARE_REVISION_LEN);
        }
        break;
    case HARDWARE_VERSION_CHAR:
        if (len == HARDWARE_VERSION_LEN)
        {
            memcpy(hw, value, HARDWARE_VERSION_LEN);
        }
        break;
    case DATE_CHAR:
        if (len == DATE_LEN)
        {
            memcpy(date, value, DATE_LEN);
        }
        break;

    default:
        break;
    }
}

#endif // NRF_MODULE_ENABLED(BLE_info)
