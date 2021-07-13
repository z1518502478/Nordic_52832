/**
 * Copyright (c) 2012 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "sdk_common.h"

#if NRF_MODULE_ENABLED(BLE_NUS)
#include "ble.h"
#include "ble_diy_service.h"
#include "ble_srv_common.h"

#define NRF_LOG_MODULE_NAME ble_diy
#if BLE_diy_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL BLE_diy_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR BLE_diy_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR BLE_diy_CONFIG_DEBUG_COLOR
#else // BLE_diy_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL 0
#endif // BLE_diy_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define DIY_BASE_UUID                                                                                \
  {                                                                                                  \
    {                                                                                                \
      0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 \
    }                                                                                                \
  } /**< Used vendor specific UUID. */

static uint8_t ServiceProfileChar1Value[SERVICEPROFILE_CHAR1_LEN] = {0};
static uint8_t ServiceProfileChar2Value[SERVICEPROFILE_CHAR2_LEN] = {0};
static uint8_t ServiceProfileChar3Value = 0;
static uint8_t ServiceProfileChar4Value[SERVICEPROFILE_CHAR4_LEN] = {0};
static uint8_t ServiceProfileChar5Value[SERVICEPROFILE_CHAR5_LEN] = {0};
static uint8_t ServiceProfileChar6Value[SERVICEPROFILE_CHAR6_LEN] = {0};
static uint8_t ServiceProfileChar7Value = {0};
static uint8_t ServiceProfileChar8Value = {0};



/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the SoftDevice.
 *
 * @param[in] p_diy     Nordic UART Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_diy_t *p_diy, ble_evt_t const *p_ble_evt)
{

  ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
  ble_diy_evt_t evt;
  memset(&evt, 0, sizeof(ble_diy_evt_t));
  evt.p_diy = p_diy;
  /*--------------------- 特征1 ---------------------*/
  if ((p_evt_write->handle == p_diy->char1Handle.value_handle) &&
      (p_diy->data_handler != NULL))
  {
    evt.params.service_data.p_data = p_evt_write->data;
    evt.params.service_data.length = p_evt_write->len;
    evt.uuid = p_evt_write->uuid.uuid;
    p_diy->data_handler(&evt);
  }

  /*--------------------- 特征2 ---------------------*/

  else if ((p_evt_write->handle == p_diy->char2Handle.value_handle) &&
           (p_diy->data_handler != NULL))
  {
    evt.params.service_data.p_data = p_evt_write->data;
    evt.params.service_data.length = p_evt_write->len;
    evt.uuid = p_evt_write->uuid.uuid;
    p_diy->data_handler(&evt);
  }

  /*--------------------- 特征3 ---------------------*/

  else if ((p_evt_write->handle == p_diy->char3Handle.value_handle) &&
           (p_diy->data_handler != NULL))
  {
    evt.params.service_data.p_data = p_evt_write->data;
    evt.params.service_data.length = p_evt_write->len;
    evt.uuid = p_evt_write->uuid.uuid;
    p_diy->data_handler(&evt);
  }

  /*--------------------- 特征4 ---------------------*/

  else if ((p_evt_write->handle == p_diy->char4Handle.value_handle) &&
           (p_diy->data_handler != NULL))
  {
    evt.params.service_data.p_data = p_evt_write->data;
    evt.params.service_data.length = p_evt_write->len;
    evt.uuid = p_evt_write->uuid.uuid;
    p_diy->data_handler(&evt);
  }

  /*--------------------- 特征5 ---------------------*/
  else if ((p_evt_write->handle == p_diy->char5Handle.value_handle) &&
           (p_diy->data_handler != NULL))
  {
    evt.params.service_data.p_data = p_evt_write->data;
    evt.params.service_data.length = p_evt_write->len;
    evt.uuid = p_evt_write->uuid.uuid;
    p_diy->data_handler(&evt);
  }

  /*--------------------- 特征6 ---------------------*/
  else if ((p_evt_write->handle == p_diy->char6Handle.value_handle) &&
           (p_diy->data_handler != NULL))
  {
    evt.params.service_data.p_data = p_evt_write->data;
    evt.params.service_data.length = p_evt_write->len;
    evt.uuid = p_evt_write->uuid.uuid;
    p_diy->data_handler(&evt);
  }

  /*--------------------- 特征7 ---------------------*/
  else if ((p_evt_write->handle == p_diy->char7Handle.value_handle) &&
           (p_diy->data_handler != NULL))
  {
    evt.params.service_data.p_data = p_evt_write->data;
    evt.params.service_data.length = p_evt_write->len;
    evt.uuid = p_evt_write->uuid.uuid;
    p_diy->data_handler(&evt);
  }
  /*--------------------- 特征8 ---------------------*/
  else if ((p_evt_write->handle == p_diy->char8Handle.value_handle) &&
           (p_diy->data_handler != NULL))
  {
    evt.params.service_data.p_data = p_evt_write->data;
    evt.params.service_data.length = p_evt_write->len;
    evt.uuid = p_evt_write->uuid.uuid;
    p_diy->data_handler(&evt);
  }
}

void ble_diy_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context)
{
  if ((p_context == NULL) || (p_ble_evt == NULL))
  {
    return;
  }
  ble_diy_t *p_diy = (ble_diy_t *)p_context;

  switch (p_ble_evt->header.evt_id)
  {
  case BLE_GATTS_EVT_WRITE:
    on_write(p_diy, p_ble_evt);

    break;
  default:
    // No implementation needed.
    break;
  }
}

/*********************************************************************
 * @fn      DiyService_SetParameter
 *
 * @brief   Set a Diy Service parameter.
 *          设置“Diy Service”参数。
 *
 * @param   param - service parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  无
 */
void DiyService_SetParameter(uint8_t param, uint8_t len, void *value)
{
  switch (param)
  {
  case SERVICEPROFILE_CHAR1:
    if (len == SERVICEPROFILE_CHAR1_LEN)
    {
       memcpy(ServiceProfileChar1Value, value, SERVICEPROFILE_CHAR1_LEN);
    }
    break;

  case SERVICEPROFILE_CHAR2:
    if (len == SERVICEPROFILE_CHAR2_LEN)
    {
       memcpy(ServiceProfileChar2Value, value, SERVICEPROFILE_CHAR2_LEN);
    }
    break;

  case SERVICEPROFILE_CHAR3:
    if (len == sizeof(uint8_t))
    {
      ServiceProfileChar3Value = *(uint8_t*)value;
    }
    break;

  case SERVICEPROFILE_CHAR4:
    if (len == SERVICEPROFILE_CHAR4_LEN)
    {
       memcpy(ServiceProfileChar4Value, value, SERVICEPROFILE_CHAR4_LEN);
    }
    break;

  case SERVICEPROFILE_CHAR5:
    if (len == SERVICEPROFILE_CHAR5_LEN)
    {
       memcpy(ServiceProfileChar5Value, value, SERVICEPROFILE_CHAR5_LEN);
    }
    break;

  //(New) Char 6
  case SERVICEPROFILE_CHAR6:
    if (len == SERVICEPROFILE_CHAR6_LEN)
    {
       memcpy(ServiceProfileChar6Value, value, SERVICEPROFILE_CHAR6_LEN);
    }
    break;

    //(New) Char 7
  case SERVICEPROFILE_CHAR7:
    if (len == sizeof(uint8_t))
    {
      ServiceProfileChar7Value = *(uint8_t*)value;
    }
    break;

    //(New) Char 8
  case SERVICEPROFILE_CHAR8:
    if (len == sizeof(uint8_t))
    {
      ServiceProfileChar8Value = *(uint8_t*)value;
    }
    break;

  default:
    break;
  }
}

uint32_t ble_diy_service_init(ble_diy_t *p_diy, ble_diy_init_t const *p_diy_init)
{

  uint32_t err_code;
  ble_uuid_t ble_uuid;
  ble_uuid128_t diy_base_uuid = DIY_BASE_UUID;
  ble_add_char_params_t add_char_params;

  VERIFY_PARAM_NOT_NULL(p_diy);
  VERIFY_PARAM_NOT_NULL(p_diy_init);

  // Initialize the service structure.
  p_diy->data_handler = p_diy_init->data_handler;

  /**@snippet [Adding proprietary Service to the SoftDevice] */
  // Add a custom base UUID.
  err_code = sd_ble_uuid_vs_add(&diy_base_uuid, &p_diy->uuid_type);
  VERIFY_SUCCESS(err_code);

  ble_uuid.type = p_diy->uuid_type;
  ble_uuid.uuid = BLE_UUID_DIY_SERVICE;

  // Add the service.
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                      &ble_uuid,
                                      &p_diy->service_handle);
  /**@snippet [Adding proprietary Service to the SoftDevice] */
  VERIFY_SUCCESS(err_code);

  /*--------------------- 特征1(可读、可�?) ---------------------*/
  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = SERVICEPROFILE_UUID_CHAR1;
  add_char_params.uuid_type = p_diy->uuid_type;
  add_char_params.max_len = SERVICEPROFILE_CHAR1_LEN;
  add_char_params.p_init_value = (uint8_t *)&ServiceProfileChar1Value;
  add_char_params.init_len = SERVICEPROFILE_CHAR1_LEN;
  add_char_params.is_var_len = true;
  add_char_params.char_props.read = 1;  //可读属写
  add_char_params.char_props.write = 1; //可写属写

  add_char_params.read_access = SEC_OPEN;
  add_char_params.write_access = SEC_OPEN;

  err_code = characteristic_add(p_diy->service_handle, &add_char_params, &p_diy->char1Handle);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  /*--------------------- 特征2(可读、可�?) ---------------------*/
  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = SERVICEPROFILE_UUID_CHAR2;
  add_char_params.uuid_type = p_diy->uuid_type;
  add_char_params.max_len = SERVICEPROFILE_CHAR2_LEN;
  add_char_params.init_len = SERVICEPROFILE_CHAR2_LEN;
  add_char_params.p_init_value = ServiceProfileChar2Value;
  add_char_params.is_var_len = true;
  add_char_params.char_props.read = 1;  //可读属写
  add_char_params.char_props.write = 1; //可写属写

  add_char_params.read_access = SEC_OPEN;
  add_char_params.write_access = SEC_OPEN;

  err_code = characteristic_add(p_diy->service_handle, &add_char_params, &p_diy->char2Handle);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  /*--------------------- 特征3(可读、可�?) ---------------------*/
  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = SERVICEPROFILE_UUID_CHAR3;
  add_char_params.uuid_type = p_diy->uuid_type;
  add_char_params.max_len = 1;
  add_char_params.p_init_value = &ServiceProfileChar3Value;
  add_char_params.init_len = 1;
  add_char_params.is_var_len = true;
  add_char_params.char_props.read = 1;  //可读属写
  add_char_params.char_props.write = 1; //可写属写

  add_char_params.read_access = SEC_OPEN;
  add_char_params.write_access = SEC_OPEN;

  err_code = characteristic_add(p_diy->service_handle, &add_char_params, &p_diy->char3Handle);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  /*--------------------- 特征4(只读) ---------------------*/
  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = SERVICEPROFILE_UUID_CHAR4;
  add_char_params.uuid_type = p_diy->uuid_type;
  add_char_params.max_len = SERVICEPROFILE_CHAR4_LEN;
  add_char_params.p_init_value = ServiceProfileChar4Value;
  add_char_params.init_len = SERVICEPROFILE_CHAR4_LEN;
  add_char_params.is_var_len = true;
  add_char_params.char_props.read = 1; //可读属写

  add_char_params.read_access = SEC_OPEN;

  err_code = characteristic_add(p_diy->service_handle, &add_char_params, &p_diy->char4Handle);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  /*--------------------- 特征5(可读、可�?) ---------------------*/
  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = SERVICEPROFILE_UUID_CHAR5;
  add_char_params.uuid_type = p_diy->uuid_type;
  add_char_params.max_len = SERVICEPROFILE_CHAR5_LEN;
  add_char_params.p_init_value = ServiceProfileChar5Value;
  add_char_params.init_len = SERVICEPROFILE_CHAR5_LEN;
  add_char_params.is_var_len = true;
  add_char_params.char_props.read = 1;  //可读属写
  add_char_params.char_props.write = 1; //可写属写

  add_char_params.read_access = SEC_OPEN;
  add_char_params.write_access = SEC_OPEN;

  err_code = characteristic_add(p_diy->service_handle, &add_char_params, &p_diy->char5Handle);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  /*--------------------- 特征6(可读、可�?) ---------------------*/
  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = SERVICEPROFILE_UUID_CHAR6;
  add_char_params.uuid_type = p_diy->uuid_type;
  add_char_params.max_len = SERVICEPROFILE_CHAR6_LEN;
  add_char_params.p_init_value = ServiceProfileChar6Value;
  add_char_params.init_len = SERVICEPROFILE_CHAR6_LEN;
  add_char_params.is_var_len = true;
  add_char_params.char_props.read = 1;  //可读属写
  add_char_params.char_props.write = 1; //可写属写

  add_char_params.read_access = SEC_OPEN;
  add_char_params.write_access = SEC_OPEN;

  err_code = characteristic_add(p_diy->service_handle, &add_char_params, &p_diy->char6Handle);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  /*--------------------- 特征7(可读、可�?) ---------------------*/
  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = SERVICEPROFILE_UUID_CHAR7;
  add_char_params.uuid_type = p_diy->uuid_type;
  add_char_params.max_len = 1;
  add_char_params.p_init_value = &ServiceProfileChar7Value;
  add_char_params.init_len = 1;
  add_char_params.is_var_len = true;
  add_char_params.char_props.read = 1;  //可读属写
  add_char_params.char_props.write = 1; //可写属写

  add_char_params.read_access = SEC_OPEN;
  add_char_params.write_access = SEC_OPEN;

  err_code = characteristic_add(p_diy->service_handle, &add_char_params, &p_diy->char7Handle);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }

  /*--------------------- 特征8(只读) ---------------------*/
  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = SERVICEPROFILE_UUID_CHAR8;
  add_char_params.uuid_type = p_diy->uuid_type;
  add_char_params.max_len = 1;
  add_char_params.p_init_value = &ServiceProfileChar8Value;
  add_char_params.init_len = 1;
  add_char_params.is_var_len = true;
  add_char_params.char_props.read = 1; //可读属写

  add_char_params.read_access = SEC_OPEN;

  err_code = characteristic_add(p_diy->service_handle, &add_char_params, &p_diy->char8Handle);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }
  return err_code;
}

#endif // NRF_MODULE_ENABLED(BLE_diy)
