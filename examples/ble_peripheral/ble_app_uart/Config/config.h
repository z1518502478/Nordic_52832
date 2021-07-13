/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
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
 * @defgroup crc32 CRC32 compute
 * @{
 * @ingroup hci_transport
 *
 * @brief    This module implements the CRC-32 calculation in the blocks.
 */

#ifndef CONFIG_H__
#define CONFIG_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**@brief Function for calculating CRC-32 in blocks.
 *
 * Feed each consecutive data block into this function, along with the current value of p_crc as
 * returned by the previous call of this function. The first call of this function should pass NULL
 * as the initial value of the crc in p_crc.
 *
 * @param[in] p_data The input data block for computation.
 * @param[in] size   The size of the input data block in bytes.
 * @param[in] p_crc  The previous calculated CRC-32 value or NULL if first call.
 *
 * @return The updated CRC-32 value, based on the input supplied.
 */
    uint32_t crc32(uint32_t crc, uint8_t *buf, uint32_t len);
    
    typedef struct
    {
        //
        uint8_t mac_addr[6];    //本机MAC地址
        uint8_t charid[2];      //服务id
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

#ifdef __cplusplus
}
#endif

#endif // CRC_32_H__

/** @} */
