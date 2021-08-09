/*********************************************************************
 * INCLUDES
 */
#include "nvmc.h"

#include "string.h"
#include "stdio.h"
#include "crc.h"

#define BUF_LEN 64
//SNV使用宏

uint8_t Buf_Data[BUF_LEN];
static int default_Init(void);

void Nvcm_Check_init(uint32_t len)
{
    uint32_t crc_nv = 0;

    memset(Buf_Data, 0, BUF_LEN);
    Nvmc_Read(Buf_Data, len, FLASH_ADDR);
    memcpy((void *)&crc_nv, Buf_Data, sizeof(crc_nv));

    if (crc_nv != crc32(0, &Buf_Data[sizeof(crc_nv)], len - sizeof(crc_nv)))
    {
        memset(Buf_Data, 0, BUF_LEN);
        Nvmc_Read(Buf_Data, len, BACKUP_ADDR);
        memcpy((void *)&crc_nv, Buf_Data, sizeof(crc_nv));

        if (crc_nv != crc32(0, &Buf_Data[sizeof(crc_nv)], len - sizeof(crc_nv)))
        {
            default_Init();
        }
    }
}

const uint8_t default_UUID[]   = {0xFD, 0xA5, 0x06, 0x93, 0xA4, 0xE2, 0x4F, 0xB1,
                                  0xAF, 0xCF, 0xC6, 0xEB, 0x07, 0x64, 0x78, 0x25};
const uint8_t default_Major[]  = {0x27, 0x11};
const uint8_t default_Minor[]  = {0x27, 0x12};
const uint8_t default_Battry[] = {0x68, 0x01};
//默认配置
static int default_Init(void)
{
    SVNINF_t nvdata;

    nvdata.sys_config_t.txPower = 6;
    nvdata.sys_config_t.Rxp = 0xB5;
    nvdata.sys_config_t.interval = 1;
    nvdata.sys_config_t.AT_Flag = 0;

    memcpy(nvdata.sys_config_t.UUID_value, default_UUID, 16);
    memcpy(nvdata.sys_config_t.major_value, default_Major, 2);
    memcpy(nvdata.sys_config_t.minor_value, default_Minor, 2);
    memcpy(nvdata.sys_config_t.charbattry, default_Battry, 2);
    memcpy(nvdata.sys_config_t.HWVR, "0110", 4);
    memcpy(nvdata.sys_config_t.date, "2021-07-12", 10);

    nvdata.crc32 = crc32(0, (uint8_t *)(&nvdata.sys_config_t), sizeof(SYS_CONFIG));

    Nvmc_Write((uint8_t *)&nvdata.sys_config_t, sizeof(SYS_CONFIG));

    return 0;
}

//写入Flash参数
void Nvmc_Write(uint8_t *Write_data, uint32_t len)
{
    uint32_t crc_nv = 0;
    memset(Buf_Data, 0, BUF_LEN);

    crc_nv = crc32(0, Write_data, len);
    memcpy(Buf_Data, (void *)&crc_nv, sizeof(crc_nv));
    memcpy(Buf_Data + sizeof(crc_nv), Write_data, len);

    nrf_nvmc_page_erase(BACKUP_ADDR);
    if (NRF_NVMC->READY != NVMC_READY_READY_Busy)
    {
        nrf_nvmc_write_bytes(BACKUP_ADDR, Buf_Data, len + sizeof(uint32_t));
    }
    nrf_nvmc_page_erase(FLASH_ADDR);
    if (NRF_NVMC->READY != NVMC_READY_READY_Busy)
    {
        nrf_nvmc_write_bytes(FLASH_ADDR, Buf_Data, len + sizeof(uint32_t));
    }
    nrf_nvmc_page_erase(BACKUP_ADDR);
}

int Nvmc_Read(uint8_t *Read_data, uint32_t len, uint32_t address)
{
    uint8_t *pdat;
    if (Read_data == NULL && len == 0)
    {
        return -1;
    }
    pdat = (uint8_t *)address;
    memcpy(Read_data, pdat, len);

    return 0;
}
