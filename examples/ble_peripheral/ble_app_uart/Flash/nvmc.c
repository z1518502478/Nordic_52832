/*********************************************************************
 * INCLUDES
 */
#include "nvmc.h"

#include "string.h"
#include "stdio.h"

#define BUF_LEN 64
//SNV使用宏

uint8_t Buf_Data[BUF_LEN];

void Nvcm_Check_init(uint32_t len)
{
    uint32_t crc_nv = 0;
    uint8_t buf_d[4] = {0};

    memset(Buf_Data, 0, BUF_LEN);
    Nvmc_Read(Buf_Data, len);

    for (int i = (len - sizeof(uint32_t)); i < len; i++)
    {
        memcpy((void *)&buf_d[i - (len - 4)], &Buf_Data[i], 1);
    }
    memcpy((void *)&crc_nv, buf_d, sizeof(uint32_t));

    if (crc_nv != crc32(0, Buf_Data, len - sizeof(uint32_t)))
    {
        default_Init();
    }
}

const uint8_t default_UUID[]   = {0xFD, 0xA5, 0x06, 0x93, 0xA4, 0xE2, 0x4F, 0xB1,
                                  0xAF, 0xCF, 0xC6, 0xEB, 0x07, 0x64, 0x78, 0x25};
const uint8_t default_Major[]  = {0x27, 0x11};
const uint8_t default_Minor[]  = {0x27, 0x12};
const uint8_t default_Battry[] = {0x68, 0x01};
//默认配置
int default_Init(void)
{
    uint8_t mac_addr[6] = {0};
    SYS_CONFIG nvdata;
    ble_gap_addr_t bleAddr;
		
    for (int i = 0; i < 6; i++)
    {
        mac_addr[i] = bleAddr.addr[5 - i];
    }

    nvdata.txPower = 6;
    nvdata.Rxp = 0xB5;
    nvdata.interval = 1;
    nvdata.AT_Flag = 0;

    memcpy(nvdata.mac_addr, mac_addr, 6);
    memcpy(nvdata.UUID_value, default_UUID, 16);
    memcpy(nvdata.major_value, default_Major, 2);
    memcpy(nvdata.minor_value, default_Minor, 2);
    memcpy(nvdata.charbattry, default_Battry, 2);
    memcpy(nvdata.HWVR, "0110", 4);
    memcpy(nvdata.date, "2021-07-12", 10);

    Nvmc_Write((uint8_t *)&nvdata, sizeof(SYS_CONFIG));
    nrf_delay_ms(10);

    return 0;
}

//写入Flash参数
void Nvmc_Write(uint8_t *Write_data, uint32_t len)
{
    uint32_t crc_nv = 0;
    memset(Buf_Data, 0, BUF_LEN);
    crc_nv = crc32(0, Write_data, len);

    memcpy(Buf_Data, Write_data, len );
    memcpy(Buf_Data + len, (void *)&crc_nv, sizeof(uint32_t));

    nrf_nvmc_page_erase(BACKUP_ADDR);
    nrf_delay_ms(10);
    nrf_nvmc_write_bytes(BACKUP_ADDR, Buf_Data, len + sizeof(uint32_t));
    nrf_delay_ms(10);
    nrf_nvmc_page_erase(FLASH_ADDR);
    nrf_delay_ms(10);
    nrf_nvmc_write_bytes(FLASH_ADDR, Buf_Data, len + sizeof(uint32_t));
    nrf_delay_ms(10);
    nrf_nvmc_page_erase(BACKUP_ADDR);
}

int Nvmc_Read(uint8_t *Read_data, uint16_t len)
{
    uint8_t *fdat;
    uint8_t *bdat;
    fdat = (uint8_t *)FLASH_ADDR;
    if (fdat == NULL)
    {
        bdat = (uint8_t *)BACKUP_ADDR;
        if (bdat == NULL)
        {
            return -1;
        }
        memcpy(Read_data, bdat, len);
        return 0;
    }
    memcpy(Read_data, fdat, len);
    return 0;
}
