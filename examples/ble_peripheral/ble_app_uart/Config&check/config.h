#ifndef CONFIG_H__
#define CONFIG_H__

#include <stdint.h>

typedef struct
{
    uint8_t charkey[2];     //服务密钥
    uint8_t charbattry[2];  //电池电量
    uint8_t txPower;        //发射信号强度
    uint8_t interval;       //工作间隔
    uint8_t major_value[2]; //Major
    uint8_t minor_value[2]; //Minor
    uint8_t UUID_value[16]; //UUID
    uint8_t date[10];       //生产日期
    uint8_t Rxp;            //RXP
    uint8_t HWVR[4];        //硬件版本
    uint8_t AT_Flag;        //串口配置完成标志位
    uint8_t Flag;           //服务写入标志位
} SYS_CONFIG;

typedef struct
{
    uint32_t crc32;
    SYS_CONFIG sys_config_t;
} SVNINF_t;

#endif // CONFIG_H__

/** @} */
