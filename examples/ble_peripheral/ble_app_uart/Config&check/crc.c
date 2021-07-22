#include "sdk_common.h"

#include <stdlib.h>

uint32_t crc32(uint32_t crc, uint8_t *buf, uint32_t len)
{
  int i;

  crc = crc ^ 0xFFFFFFFF;

  while (len--)
  {
    crc ^= *buf++;

    for (i = 0; i < 8; i++)
    {
      if ((crc & 0x00000001) != 0)
      {
        crc >>= 1;
        crc ^= 0xEDB88320L;
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc ^ 0xFFFFFFFF;
}

