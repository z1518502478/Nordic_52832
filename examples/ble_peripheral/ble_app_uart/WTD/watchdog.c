/*********************************************************************
 * INCLUDES
 */
#include "nrf_drv_wdt.h"

static void watchdogHandleEvent(void);

/*********************************************************************
 * LOCAL VARIABLES
 */
static nrf_drv_wdt_channel_id s_wdtChannelId; // 看门狗通道ID

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/**
 @brief 看门狗驱动初始化
 @param 无
 @return 无
*/
void Watchdog_Init(void)
{
    ret_code_t err_code;
    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;       // 配置看门狗
    
    err_code = nrf_drv_wdt_init(&config, watchdogHandleEvent);      // 初始化看门狗和看门狗中断
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_wdt_channel_alloc(&s_wdtChannelId);          // 分配看门狗通道
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_wdt_enable();                                           // 看门狗使能
}
/**
 @brief 清除看门狗计数，“喂狗”
 @param 无
 @return 无
*/
void Watchdog_Clear(void)
{
    nrf_drv_wdt_channel_feed(s_wdtChannelId);
}


/*********************************************************************
 * LOCAL FUNCTIONS
 */
/**
 @brief 看门狗中断事件处理函数
 @param 无
 @return 无
*/
static void watchdogHandleEvent(void)
{
    Watchdog_Clear();
}

/****************************************************END OF FILE****************************************************/
