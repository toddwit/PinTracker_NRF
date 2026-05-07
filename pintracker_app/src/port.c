#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include "deca_device_api.h"

#include <stdint.h>

void deca_usleep(unsigned long time_us)
{
	k_usleep(time_us);
}

void deca_sleep(unsigned int time_ms)
{
	k_msleep(time_ms);
}

decaIrqStatus_t decamutexon(void)
{
	return 0;
}

void decamutexoff(decaIrqStatus_t s)
{
	ARG_UNUSED(s);
}

float rsl_calculate_signal_power(float rxpacc,
								 float cir_pwr,
								 uint8_t dgc_dec)
{
	ARG_UNUSED(rxpacc);
	ARG_UNUSED(cir_pwr);
	ARG_UNUSED(dgc_dec);

	return 0.0f;
}

float rsl_calculate_first_path_power(float fp1,
									 float fp2,
									 float fp3,
									 uint16_t rxpacc)
{
	ARG_UNUSED(fp1);
	ARG_UNUSED(fp2);
	ARG_UNUSED(fp3);
	ARG_UNUSED(rxpacc);

	return 0.0f;
}

/* ===== Existing stubs ===== */

void port_set_dw_ic_spi_slowrate(void)
{
}

void port_set_dw_ic_spi_fastrate(void)
{
}

void wakeup_device_with_io(void)
{
	k_msleep(1);
}

extern int writetospi(uint16_t headerLength,
					  const uint8_t *headerBuffer,
					  uint32_t bodyLength,
					  const uint8_t *bodyBuffer);

int writetospiwithcrc(uint16_t headerLength,
					  const uint8_t *headerBuffer,
					  uint16_t bodyLength,
					  const uint8_t *bodyBuffer,
					  uint8_t crc8)
{
	ARG_UNUSED(crc8);

	return writetospi(headerLength,
					  headerBuffer,
					  bodyLength,
					  bodyBuffer);
}