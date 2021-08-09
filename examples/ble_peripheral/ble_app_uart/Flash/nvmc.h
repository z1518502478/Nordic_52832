#ifndef _NVMC_H_
#define _NVMC_H_

/*********************************************************************
 * INCLUDES
 */
#include "nrf_nvmc.h"
#include "stdint.h"
#include "config.h"
#include "nrf_delay.h"
#include "ble_gap.h"
/*********************************************************************
 * DEFINITIONS
 */
#define FLASH_ADDR 0x2E000
#define BACKUP_ADDR 0x2F000

/*********************外部函数************************/
void Nvmc_Write(uint8_t *Write_data, uint32_t len); //写入Flash函数
void Nvcm_Check_init(uint32_t len);
int Nvmc_Read(uint8_t *Read_data, uint32_t len, uint32_t address);

#endif /* _NVMC_H_ */
