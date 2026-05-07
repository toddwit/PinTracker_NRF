#pragma once

#include <stdint.h>
#include "deca_device_api.h"

#ifdef __cplusplus
extern "C"
{
#endif

	/* ===== SPI rate helpers ===== */

	void port_set_dw_ic_spi_slowrate(void);

	void port_set_dw_ic_spi_fastrate(void);

	/* ===== Wakeup ===== */

	void wakeup_device_with_io(void);

	/* ===== SPI write with CRC ===== */

	int writetospiwithcrc(uint16_t headerLength,
						  const uint8_t *headerBuffer,
						  uint16_t bodyLength,
						  const uint8_t *bodyBuffer,
						  uint8_t crc8);

	/* ===== Timing ===== */

	void deca_usleep(unsigned long time_us);

	void deca_sleep(unsigned int time_ms);

	/* ===== Mutex ===== */

	decaIrqStatus_t decamutexon(void);

	void decamutexoff(decaIrqStatus_t s);

	/* ===== RSSI helpers ===== */

	float rsl_calculate_signal_power(float rxpacc,
									 float cir_pwr,
									 uint8_t dgc_dec);

	float rsl_calculate_first_path_power(float fp1,
										 float fp2,
										 float fp3,
										 uint16_t rxpacc);

#ifdef __cplusplus
}
#endif