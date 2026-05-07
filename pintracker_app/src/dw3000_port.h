#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

	void spi_init(void);

	int readfromspi(uint16_t headerLength,
					uint8_t *headerBuffer,
					uint16_t readLength,
					uint8_t *readBuffer);

	int writetospi(uint16_t headerLength,
				   const uint8_t *headerBuffer,
				   uint16_t bodyLength,
				   const uint8_t *bodyBuffer);

#ifdef __cplusplus
}
#endif