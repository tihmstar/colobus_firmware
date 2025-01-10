#ifndef CRC_H
#define CRC_H

#include <stdint.h>
#include <stdlib.h>

uint8_t crc(const void *buf, size_t bufSize);

#endif // CRC_H