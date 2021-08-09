/*********************************************************************
 * INCLUDES
 */
#include "pca10040.h"
#include "nrf_uart.h"
#include "app_uart.h"
#include "uart_ble.h"
#include "nvmc.h"


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void uart_handleIrqEvent(app_uart_evt_t *pEvent);
Buff_t buff_t;
SVNINF_t nvdata;

/*********************************************************************
 * PUBLIC FUNCTIONS
 */


/**
 @brief 串口驱动初始化
 @param 无
 @return 无
*/
void UART_Init(void)
{
    uint32_t errCode;
    app_uart_comm_params_t const commParams =
    {
    .rx_pin_no = RX_PIN_NUMBER,
    .tx_pin_no = TX_PIN_NUMBER,
    .rts_pin_no = RTS_PIN_NUMBER,
    .cts_pin_no = CTS_PIN_NUMBER,
    .flow_control = APP_UART_FLOW_CONTROL_DISABLED, // 关掉流控
    .use_parity = false,
#if defined(UART_PRESENT)
    .baud_rate = NRF_UART_BAUDRATE_9600 // 波特率
#else
    .baud_rate = NRF_UART_BAUDRATE_9600
#endif
    };

    APP_UART_FIFO_INIT(&commParams,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_handleIrqEvent,
                       APP_IRQ_PRIORITY_LOWEST,
                       errCode);

    APP_ERROR_CHECK(errCode);
}

/**
 @brief 串口写数据函数
 @param pData -[in] 写入数据
 @param dataLen -[in] 写入数据长度
 @return 无
*/
void UART_WriteData(uint8_t *pData, uint8_t dataLen)
{
	uint8_t i;
	for(i = 0; i < dataLen; i++)
	{
		app_uart_put(pData[i]);
	}
}

/**
 @brief 串口读数据函数
 @param pData -[out] 读取数据
 @return 无
*/
static void UART_ReadData(uint8_t *pData)
{
	uint32_t errCode;
	errCode = app_uart_get(pData);
	APP_ERROR_CHECK(errCode);
}


/*********************************************************************
 * LOCAL FUNCTIONS
 */
/**
 @brief 串口读取数据处理函数
 @param pEvent -[in] 串口事件
 @return 无
*/
static void uart_handleIrqEvent(app_uart_evt_t *pEvent)
{
    switch (pEvent->evt_type)
    {
        case APP_UART_DATA_READY: // 已接收到UART数据
            app_uart_get(&buff_t.userBuffer[buff_t.w_len++]);
            break;

        case APP_UART_COMMUNICATION_ERROR: // 接收过程中发生通信错误
            APP_ERROR_HANDLER(pEvent->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR: // app_uart模块使用的FIFO模块中出现错误
            APP_ERROR_HANDLER(pEvent->data.error_code);
            break;

        default:
            break;
    }
}

int str_cmp(uint8_t *buf, char *str, uint8_t len)
{
    uint8_t array[BUFSIZE] = {0};
    memcpy((void *)array, buf, len);
    if (strcmp((void *)array, (void *)str) == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}


/****************************************************END OF FILE****************************************************/
