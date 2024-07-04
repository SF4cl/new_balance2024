#ifndef __CRC_CCITT_H
#define __CRC_CCITT_H

#include <stdint.h>
#include <stddef.h>


uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, size_t len);

#endif
