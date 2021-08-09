#ifndef _BOARD_ADC_H_
#define _BOARD_ADC_H_

/*********************************************************************
 * INCLUDES
 */
#include "nordic_common.h"
#include "stdio.h"
#include "string.h"

/*********************************************************************
 * DEFINITIONS
 */
#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600				/**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6					/**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS  1000					/**< Typical forward voltage drop of the diode . */
#define ADC_RES_10BIT                   1024				/**< Maximum digital value for 10-bit ADC conversion. */

// VP = (RESULT * REFERENCE / 2^10) * 6
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) \
    ((((ADC_VALUE)*ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION) 

#define SAMPLES_IN_BUFFER				1

/*********************************************************************
 * API FUNCTIONS
 */
void ADC_Init(void);
void ADC_Read(void);
void ADC_Disable(void);

#endif /* _BOARD_ADC_H_ */
