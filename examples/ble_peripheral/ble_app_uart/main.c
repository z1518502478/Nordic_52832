
/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
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
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "ble_diy_service.h"
#include "ble_info_service.h"
#include "config.h"
#include "nvmc.h"
#include "uart_ble.h"
#include "adc.h"
#include "watchdog.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log_ctrl.h"

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "Beelinker"                                 /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_BLE                                     /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                160                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 100 ms). */
#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_BEACON_INFO_LENGTH          0x17
#define APP_ADV_DATA_LENGTH             0x15
#define APP_DEVICE_TYPE                 0x02
#define APP_MEASURED_RISS               0xB5
#define APP_COMPANY_IDENTIFIER          0x004C
#define APP_MAJOR_VALUE                 0x27, 0x14
#define APP_MINOR_VALUE                 0x36, 0xCD
#define APP_BEACON_UUID                 0xFD, 0xA5, 0x06, 0x93, \
                                        0xA4, 0xE2, 0x4F, 0xB1, \
                                        0xAF, 0xCF, 0xC6, 0xEB, \
                                        0x07, 0x64, 0x78, 0x25
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18
#define UICR_ADDRESS                    0x10001080
#define BLE_SERVICE_UUID                0x2578
#define BLE_UUID_DIY_SERVICE            0xFFF0                                      /**< The UUID of the Diy Service. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */


BLE_DIY_DEF(m_diy);                                                                 /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */



#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO 18 /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS 0x10001080          /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif

extern SVNINF_t nvdata;
SYS_CONFIG sys_config_t;
extern Buff_t buff_t;

static ble_gap_adv_params_t m_adv_params;                     /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t      m_adv_handle     = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t      m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];

static uint16_t     m_conn_handle = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint8_t      m_enc_scandata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
static ble_uuid_t   m_adv_uuids[] = /**< Universally unique service identifier. */
   {
        {BLE_UUID_DIY_SERVICE, NUS_SERVICE_UUID_TYPE}};

static uint8_t p_data[] = {0x27, 0xD6, 0x28, 0x66, 0xE5, 0x01};
uint8_t D_FRT[10] = {'2', '0', '2', '0', '-', '0', '9', '-', '2', '4'};                                                
uint8_t D_FR[14] = {'H', 'M', 'V', 'E', 'R', 'S', 'I', 'O', 'N', '_', '0', '0', '0', '8'};                             
uint8_t D_CKey[16] = {0xDE, 0x48, 0x2B, 0x1C, 0x22, 0x1C, 0x6C, 0x30, 0x3C, 0xF0, 0x50, 0xEB, 0x00, 0x20, 0xB0, 0xBD}; 
uint8_t Back_AT[4] = {'O','K','\r','\n'};
uint8_t Back_A[6] = {'O','K','+','1','\r','\n'};
uint8_t Back_T[3] = {'O','K','+'};

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
    {
        .adv_data =
            {
                .p_data = m_enc_advdata,
                .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX},
        .scan_rsp_data =
            {
                .p_data = m_enc_scandata,
                .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

            }};

static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] = /**< Information advertised by the Beacon. */
    {
        APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                             // implementation.
        APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                             // manufacturer specific data in this implementation.
        APP_BEACON_UUID,     // 128 bit UUID value.

        APP_MAJOR_VALUE, // Major arbitrary value that can be used to distinguish between Beacons.

        APP_MINOR_VALUE, // Minor arbitrary value that can be used to distinguish between Beacons.

        APP_MEASURED_RISS // Manufacturer specific information. The Beacon's measured TX power in
                          // this implementation.
};

static void advertising_start(void);
static void advertising_init(void);
void UART_WriteData(uint8_t *pData, uint8_t dataLen); 

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void diy_service_handler(ble_diy_evt_t *p_evt)
{
  uint16_t uuid = p_evt->uuid;
  switch (uuid)
  {
  case SERVICEPROFILE_UUID_CHAR1:
      memcpy((void *)&sys_config_t.charkey, p_evt->params.service_data.p_data, SERVICEPROFILE_CHAR1_LEN);
      break;
  case SERVICEPROFILE_UUID_CHAR2:
      memcpy((void *)&sys_config_t.UUID_value, p_evt->params.service_data.p_data, SERVICEPROFILE_CHAR2_LEN);
      break;
  case SERVICEPROFILE_UUID_CHAR3:
      memcpy((void *)&sys_config_t.txPower, p_evt->params.service_data.p_data, 1);
      break;
  case SERVICEPROFILE_UUID_CHAR4:
      memcpy((void *)&sys_config_t.charbattry, p_evt->params.service_data.p_data, SERVICEPROFILE_CHAR4_LEN);
      break;
  case SERVICEPROFILE_UUID_CHAR5:
      memcpy((void *)&sys_config_t.major_value, p_evt->params.service_data.p_data, SERVICEPROFILE_CHAR5_LEN);
      break;
  case SERVICEPROFILE_UUID_CHAR6:
      memcpy((void *)&sys_config_t.minor_value, p_evt->params.service_data.p_data, SERVICEPROFILE_CHAR6_LEN);
      break;
  case SERVICEPROFILE_UUID_CHAR7:
      memcpy((void *)&sys_config_t.interval, p_evt->params.service_data.p_data, 1);
      break;
  case SERVICEPROFILE_UUID_CHAR8:
      memcpy((void *)&sys_config_t.Rxp, p_evt->params.service_data.p_data, 1);
      break;
  default:
      break;
  }
}

/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_diy_init_t diy_init = {0};
    ble_info_init_t info_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    //Initialize Info Service.
    err_code = ble_info_service_init(&info_init);
    APP_ERROR_CHECK(err_code);

    // Initialize DIY Service.
    memset(&diy_init, 0, sizeof(diy_init));
    diy_init.data_handler = diy_service_handler;

    err_code = ble_diy_service_init(&m_diy, &diy_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:    
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            advertising_init();
            advertising_start();
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {        
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t err_code;
    uint8_t S_interval = APP_ADV_INTERVAL;
    ble_advertising_init_t init;
    ble_advdata_service_data_t sr_data;
    ble_advdata_manuf_data_t manuf_specific_data;

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
    // If USE_UICR_FOR_MAJ_MIN_VALUES is defined, the major and minor values will be read from the
    // UICR instead of using the default values. The major and minor values obtained from the UICR
    // are encoded into advertising data in big endian order (MSB First).
    // To set the UICR used by this example to a desired value, write to the address 0x10001080
    // using the nrfjprog tool. The command to be used is as follows.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val <your major/minor value>
    // For example, for a major value and minor value of 0xabcd and 0x0102 respectively, the
    // the following command should be used.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val 0xabcd0102
    uint16_t major_value = ((*(uint32_t *)UICR_ADDRESS) & 0xFFFF0000) >> 16;
    uint16_t minor_value = ((*(uint32_t *)UICR_ADDRESS) & 0x0000FFFF);

    uint8_t index = MAJ_VAL_OFFSET_IN_BEACON_INFO;

    m_beacon_info[index++] = MSB_16(major_value);
    m_beacon_info[index++] = LSB_16(major_value);

    m_beacon_info[index++] = MSB_16(minor_value);
    m_beacon_info[index++] = LSB_16(minor_value);
#endif

    memcpy(m_beacon_info + 2, sys_config_t.UUID_value, 16);
    memcpy(m_beacon_info + 18, sys_config_t.major_value, 2);
    memcpy(m_beacon_info + 20, sys_config_t.minor_value, 2);
    *(m_beacon_info + 22) = sys_config_t.Rxp;

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data = (uint8_t *)m_beacon_info;
    manuf_specific_data.data.size = APP_BEACON_INFO_LENGTH;

    // 建立和设置广播数据.
    memset(&init, 0, sizeof(init));

    init.advdata.name_type = BLE_ADVDATA_NO_NAME;
    init.advdata.flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
    init.advdata.include_appearance = false;
    init.advdata.p_manuf_specific_data = &manuf_specific_data;

    init.srdata.name_type = BLE_ADVDATA_FULL_NAME;
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids = m_adv_uuids;

    sr_data.service_uuid = BLE_SERVICE_UUID;
    sr_data.data.p_data = p_data;
    sr_data.data.size = sizeof(p_data);

    init.srdata.p_service_data_array = &sr_data;
    init.srdata.service_data_count = 0x01;
    init.srdata.name_type = BLE_ADVDATA_FULL_NAME;

    err_code = ble_advdata_encode(&init.advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);
    err_code = ble_advdata_encode(&init.srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);
    //err_code = ble_advertising_init(&m_advertising, &init);
    //APP_ERROR_CHECK(err_code);

    //初始化发布参数(在开始发布时使用).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    m_adv_params.p_peer_addr = NULL; // Undirected advertisement.
    m_adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval = S_interval;
    m_adv_params.duration = 0; // Never time out.

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}


void Uart_Processing(void)
{
	
    uint8_t restflg;
    restflg = false;
    if (buff_t.w_len == 4 && str_cmp(buff_t.userBuffer, "AT\r\n", 4))
    {
        UART_WriteData(Back_AT,4);
        memset(buff_t.userBuffer, 0, BUFSIZE);
        buff_t.w_len = 0;
    }
    else if ((buff_t.w_len == 49) && str_cmp(buff_t.userBuffer, "AT+1=", 5))
    {
        for (uint8_t i = 0; i < 16; i++)
        {
            nvdata.sys_config_t.UUID_value[i] = buff_t.userBuffer[5 + i];
        }
        for (uint8_t i = 0; i < 2; i++)
        {
            nvdata.sys_config_t.major_value[i] = buff_t.userBuffer[21 + i];
        }
        for (uint8_t i = 0; i < 2; i++)
        {
            nvdata.sys_config_t.minor_value[i] = buff_t.userBuffer[23 + i];
        }
        for (uint8_t i = 0; i < 10; i++)
        {
            nvdata.sys_config_t.date[i] = buff_t.userBuffer[25 + i];
        }
        for (uint8_t i = 0; i < 4; i++)
        {
            nvdata.sys_config_t.HWVR[i] = buff_t.userBuffer[35 + i];
        }
        nvdata.sys_config_t.txPower = buff_t.userBuffer[39];
        switch (nvdata.sys_config_t.txPower)
        {
            case 1:
                nvdata.sys_config_t.Rxp = 0xAB;
                break; //-10dBm
            case 3:
                nvdata.sys_config_t.Rxp = 0xB5;
                break; //0dBm
            case 5:
                nvdata.sys_config_t.Rxp = 0xBD;
                break; //+8dBm
            case 7:
                nvdata.sys_config_t.Rxp = 0xC0;
                break; //+11dBm
            default:
                nvdata.sys_config_t.Rxp = 0xB5;
                break; //0dBm
            }
            nvdata.sys_config_t.interval = buff_t.userBuffer[40];
            for (uint8_t i = 0; i < 6; i++)
            {
                nvdata.sys_config_t.mac_addr[i] = buff_t.userBuffer[41 + i];
            }

            UART_WriteData(Back_A, 6);
            nvdata.sys_config_t.AT_Flag = 0;
            Nvmc_Write((uint8_t *)&nvdata.sys_config_t, sizeof(SYS_CONFIG));

            memset(buff_t.userBuffer, 0, BUFSIZE);
            buff_t.w_len = 0;
    }
    //AT查询
    if ((buff_t.w_len == 6) && str_cmp(buff_t.userBuffer, "AT+?\r\n", 6))
    {
        Nvmc_Read((uint8_t*)&nvdata.sys_config_t, sizeof(SYS_CONFIG), FLASH_ADDR + 4);
        

        UART_WriteData(Back_T, 3);
        UART_WriteData(&nvdata.sys_config_t.mac_addr[0], 43);
        UART_WriteData(D_FRT, 10);
        UART_WriteData(D_FR, 14);
        UART_WriteData(D_CKey, 16);

        nvdata.sys_config_t.AT_Flag = 1;
        sys_config_t.Rxp = 0xB5;

        Nvmc_Write((uint8_t *)&nvdata.sys_config_t, sizeof(SYS_CONFIG));

        restflg = true;
    }
		
    if(true == restflg)
    {
        nrf_delay_ms(1000);
        NVIC_SystemReset();
    }
}

/**@brief Application main function.
 */
int main(void)
{

    // Initialize.
    ADC_Init();
    UART_Init();
#ifdef IWDG_ENABLE
    Watchdog_Init();
#endif
    timers_init();
    Nvcm_Check_init(sizeof(SYS_CONFIG) + sizeof(uint32_t));
    Nvmc_Read((uint8_t *)&nvdata, sizeof(SYS_CONFIG) + sizeof(uint32_t), FLASH_ADDR);

    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    //Start execution.
    ADC_Read();
    nrf_delay_ms(20);
    ADC_Disable();
    advertising_start();

    // Enter main loop.
		
    for (;;)
    {
        if(nvdata.sys_config_t.AT_Flag == 0)
        {
            Uart_Processing();
        }
        idle_state_handle();

#ifdef IWDG_ENABLE
        Watchdog_Clear();
#endif
    }
}


/**
 * @}
 */
