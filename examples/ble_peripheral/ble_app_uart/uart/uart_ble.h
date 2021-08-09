#ifndef _BOARD_UART_H_
#define _BOARD_UART_H_

/*********************************************************************
 * INCLUDES
 */
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "config.h"
#include "nrf_delay.h"
#include "config.h"
/*********************************************************************
 * DEFINITIONS
 */
#define UART_TX_BUF_SIZE                256		// UART TX buffer size
#define UART_RX_BUF_SIZE                256		// UART RX buffer size
#define BUFSIZE					        256

//串口环形队列缓冲
typedef struct
{
    uint8_t userBuffer[BUFSIZE];
    uint8_t r_len; 
    uint8_t w_len; 
} Buff_t;

/*********************************************************************
 * API FUNCTIONS
 */
void UART_Init(void);
void UART_WriteData(uint8_t *pData, uint8_t dataLen);
int str_cmp(uint8_t *buf, char *str, uint8_t len);

#endif /* _BOARD_UART_H_ */
