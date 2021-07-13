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
/** @file
 *
 * @defgroup ble_info Device Information Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief Device Information Service module.
 *
 * @details This module implements the Device Information Service.
 *          During initialization it adds the Device Information Service to the BLE stack database.
 *          It then encodes the supplied information, and adds the corresponding characteristics.
 *
 * @note Attention!
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef BLE_INFO_SERVICE_H__
#define BLE_INFO_SERVICE_H__

#include <stdint.h>
#include "ble_gap.h"
#include "ble_srv_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define ADDRESS_LEN 6
#define DATE_LEN 10
#define FIRMWARE_LEN 10
#define SERIAL_NUMBER_LEN 12
#define HARDWARE_VERSION_LEN 15
#define FIRMWARE_REVISION_LEN 15

#define ADDREEE_CHAR 0
#define FIRMWARE_CHAR 1
#define SERIAL_NUMBER_CHAR 2 
#define FIRMWARE_REVISION_CHAR 3
#define HARDWARE_VERSION_CHAR 4
#define DATE_CHAR 5

  /**@brief Device Information Service init structure. This contains all possible characteristics
 *        needed for initialization of the service.
 */
  typedef struct
  {
    ble_srv_utf8_str_t manufact_name_str; /**< Manufacturer Name String. */
    ble_srv_utf8_str_t model_num_str;     /**< Model Number String. */
    ble_srv_utf8_str_t serial_num_str;    /**< Serial Number String. */
    ble_srv_utf8_str_t hw_rev_str;        /**< Hardware Revision String. */
    ble_srv_utf8_str_t fw_rev_str;        /**< Firmware Revision String. */
    ble_srv_utf8_str_t sw_rev_str;        /**< Software Revision String. */
  } ble_info_init_t;

  /**@brief Function for initializing the Device Information Service.
 *
 * @details This call allows the application to initialize the device information service.
 *          It adds the info service and info characteristics to the database, using the initial
 *          values supplied through the p_info_init parameter. Characteristics which are not to be
 *          added, shall be set to NULL in p_info_init.
 *
 * @param[in]   p_info_init   The structure containing the values of characteristics needed by the
 *                           service.
 *
 * @return      NRF_SUCCESS on successful initialization of service.
 */
  uint32_t ble_info_service_init(ble_info_init_t const *p_info_init);
  void InfoService_SetParameter(uint8_t param, uint8_t len, void *value);

#ifdef __cplusplus
}
#endif

#endif // BLE_INFO_SERVICE_H__

/** @} */
