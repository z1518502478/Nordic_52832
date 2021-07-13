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
/**@file
 *
 * @defgroup ble_diy Nordic UART Service
 * @{
 * @ingroup  ble_sdk_srv
 * @brief    Nordic UART Service implementation.
 *
 * @details The Nordic UART Service is a simple GATT-based service with TX and RX characteristics.
 *          Data received from the peer is passed to the application, and the data received
 *          from the application of this service is sent to the peer as Handle Value
 *          Notifications. This module demonstrates how to implement a custom GATT-based
 *          service and characteristics using the SoftDevice. The service
 *          is used by the application to send and receive ASCII text strings to and from the
 *          peer.
 *
 * @note    The application must register this module as BLE event observer using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_diy_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_diy_BLE_OBSERVER_PRIO,
 *                                   ble_diy_on_ble_evt, &instance);
 *          @endcode
 */
#ifndef BLE_DIY_SERVICE_H_
#define BLE_DIY_SERVICE_H_

#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**@brief   Macro for defining a ble_diy instance.
 *
 * @param     _name            Name of the instance.
 * @param[in] _diy_max_clients Maximum number of diy clients connected at a time.
 * @hideinitializer
 */
#define BLE_DIY_DEF(_name)                        \
  static ble_diy_t _name;                         \
  NRF_SDH_BLE_OBSERVER(_name##_obs,               \
                       BLE_HRS_BLE_OBSERVER_PRIO, \
                       ble_diy_on_ble_evt, &_name)
#define SERVICE_DATA                     \
  {                                      \
    {                                    \
      0x27, 0xD6, 0x28, 0x66, 0xE5, 0x01 \
    }                                    \
  }

#define BLE_UUID_DIY_SERVICE 0xFFF0 /**< The UUID of the Diy Service. */
#define SERVICEPROFILE_UUID_CHAR1 0xFFF1
#define SERVICEPROFILE_UUID_CHAR2 0xFFF2
#define SERVICEPROFILE_UUID_CHAR3 0xFFF3
#define SERVICEPROFILE_UUID_CHAR4 0xFFF4
#define SERVICEPROFILE_UUID_CHAR5 0xFFF5
#define SERVICEPROFILE_UUID_CHAR6 0xFFF6
#define SERVICEPROFILE_UUID_CHAR7 0xFFF7
#define SERVICEPROFILE_UUID_CHAR8 0xFF60

// Profile Parameters
#define SERVICEPROFILE_CHAR1 0 // uint8_t - Profile Characteristic 1 value
#define SERVICEPROFILE_CHAR2 1 // uint8_t - Profile Characteristic 2 value
#define SERVICEPROFILE_CHAR3 2 // uint8_t - Profile Characteristic 2 value
#define SERVICEPROFILE_CHAR4 3 // uint8_t - Profile Characteristic 3 value
#define SERVICEPROFILE_CHAR5 4 // uint8_t - Profile Characteristic 4 value
#define SERVICEPROFILE_CHAR6 5 // uint8_t - Profile Characteristic 5 value
#define SERVICEPROFILE_CHAR7 6 // uint8_t - Profile Characteristic 3 value
#define SERVICEPROFILE_CHAR8 7 // uint8_t - Profile Characteristic 4 value

// Length of Characteristic in bytes
#define SERVICEPROFILE_CHAR1_LEN 2  // CHAR1 LEN
#define SERVICEPROFILE_CHAR2_LEN 16 // CHAR2 LEN
#define SERVICEPROFILE_CHAR4_LEN 2  // CHAR4 LEN
#define SERVICEPROFILE_CHAR5_LEN 2  // CHAR5 LEN
#define SERVICEPROFILE_CHAR6_LEN 2  // CHAR6 LEN

  /* Forward declaration of the ble_diy_t type. */
  typedef struct ble_diy_s ble_diy_t;

  typedef struct
  {
    uint8_t const *p_data; /**< A pointer to the buffer with received data. */
    uint16_t length;       /**< Length of received data. */
  } ble_diy_evt_data_t;

  typedef struct
  {
    ble_diy_t *p_diy; /**< A pointer to the instance. */
    uint16_t uuid;

    union
    {
      ble_diy_evt_data_t service_data; /**< @ref BLE_diy_EVT_RX_DATA event data. */
    } params;
  } ble_diy_evt_t;

  /**@brief   Nordic UART Service event structure.
 *
 * @details This structure is passed to an event coming from service.
 */

  /**@brief Nordic UART Service event handler type. */
  typedef void (*ble_diy_data_handler_t)(ble_diy_evt_t *p_evt);

  /**@brief   Nordic UART Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_diy_init
 *          function.
 */
  typedef struct
  {
    ble_diy_data_handler_t data_handler; /**< 为处理接收的数据而调用的事件处理程序. */
  } ble_diy_init_t;

  /**@brief   Nordic UART Service structure.
 *
 * @details This structure contains status information related to the service.
 */
  struct ble_diy_s
  {
    uint8_t uuid_type;                    /**< UUID type for Nordic UART Service Base UUID. */
    uint16_t service_handle;              /**< Handle of Nordic UART Service (as provided by the SoftDevice). */
    uint16_t conn_handle;                 /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    ble_gatts_char_handles_t char1Handle; // Handles related to the Characteristic 1.
    ble_gatts_char_handles_t char2Handle; // Handles related to the Characteristic 2.
    ble_gatts_char_handles_t char3Handle; // Handles related to the Characteristic 3.
    ble_gatts_char_handles_t char4Handle; // Handles related to the Characteristic 4.
    ble_gatts_char_handles_t char5Handle; // Handles related to the Characteristic 5.
    ble_gatts_char_handles_t char6Handle; // Handles related to the Characteristic 6.
    ble_gatts_char_handles_t char7Handle; // Handles related to the Characteristic 7.
    ble_gatts_char_handles_t char8Handle; // Handles related to the Characteristic 8.
    ble_diy_data_handler_t data_handler;  /**< Event handler to be called for handling received data. */
  };

  /*********************************************************************
 * API FUNCTIONS
 */
  uint32_t ble_diy_service_init(ble_diy_t *p_diy, ble_diy_init_t const *p_diy_init);
  void ble_diy_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);
  void DiyService_SetParameter(uint8_t param, uint8_t len, void *value);
#ifdef __cplusplus
}
#endif

#endif // BLE_DIY_SERVICE_H_

/** @} */
