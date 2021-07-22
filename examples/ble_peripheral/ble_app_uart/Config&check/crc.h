#ifndef CRC_H__
#define CRC_H__

#ifdef __cplusplus
extern "C"
{
#endif

    uint32_t crc32(uint32_t crc, uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif // CRC_H__

/** @} */

