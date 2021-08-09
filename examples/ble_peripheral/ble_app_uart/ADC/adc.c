/*********************************************************************
 * INCLUDES
 */
#include "nrfx_saadc.h"
#include "nrf_drv_saadc.h"
#include "app_error.h" 
#include "config.h"
#include "adc.h"
#include "nvmc.h"

static void adcCallbackFunc(nrf_drv_saadc_evt_t const *pEvent);
extern SVNINF_t nvdata;

/*********************************************************************
 * LOCAL VARIABLES
 */
static nrf_saadc_value_t s_bufferPool[SAMPLES_IN_BUFFER];
	
/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/**
 @brief ADC的初始化函数
 @param 无
 @return 无
*/
void ADC_Init(void)
{
	ret_code_t errCode;

	// ADC通道配置
	nrf_saadc_channel_config_t channelConfig = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);	// 单端输入
    // ADC初始化
	errCode = nrf_drv_saadc_init(NULL, adcCallbackFunc);
	APP_ERROR_CHECK(errCode);

	// ADC通道初始化
	errCode = nrf_drv_saadc_channel_init(0, &channelConfig);
	APP_ERROR_CHECK(errCode);

	// 缓冲配置
	errCode = nrf_drv_saadc_buffer_convert(s_bufferPool, SAMPLES_IN_BUFFER);
	APP_ERROR_CHECK(errCode);
}

/**
 @brief ADC读取
 @param 无
 @return 结果在回调函数的缓冲区中
*/
void ADC_Read(void)
{
	ret_code_t errCode;
	errCode = nrf_drv_saadc_sample();
	APP_ERROR_CHECK(errCode);
}

/**
 @brief 禁用ADC
 @param 无
 @return 无
*/
void ADC_Disable(void)
{
	nrfx_saadc_uninit();
}


/*********************************************************************
 * LOCAL FUNCTIONS
 */
/**
 @brief ADC中断处理回调函数
 @param 无
 @return 无
*/
static void adcCallbackFunc(nrf_drv_saadc_evt_t const *pEvent)
{
	if(pEvent->type == NRF_DRV_SAADC_EVT_DONE)																	// 采样完成
    {
        nrf_saadc_value_t adcResult;
        float batteryVoltage = 0.00;
        uint16_t battery_16t;
        ret_code_t errCode;

        errCode = nrf_drv_saadc_buffer_convert(pEvent->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(errCode);
        adcResult = pEvent->data.done.p_buffer[0];
        // 电池电压转换计算
        batteryVoltage = ADC_RESULT_IN_MILLI_VOLTS(adcResult) * 1.35;
        battery_16t = (uint16_t)batteryVoltage;
        nvdata.sys_config_t.charbattry[1] = (uint8_t)(battery_16t >> 8 & 0xFF);
        nvdata.sys_config_t.charbattry[0] = (uint8_t)(battery_16t & 0xFF);
    }
}

/****************************************************END OF FILE****************************************************/
