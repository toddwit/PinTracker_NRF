#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>

#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "dw3000_port.h"

extern void spi_init(void);

int main(void)
{
	printk("DW3000 API test\n");

	spi_init();

	k_msleep(10);

	/* ===== RAW DEV_ID TEST ===== */

	uint8_t dev_hdr[1] = {0x00};
	uint8_t dev_id_raw[4] = {0};

	readfromspi(1, dev_hdr, 4, dev_id_raw);

	printk("RAW DEV_ID: %02X %02X %02X %02X\n",
		   dev_id_raw[0],
		   dev_id_raw[1],
		   dev_id_raw[2],
		   dev_id_raw[3]);

	/* ===== RAW SYS_CFG TEST ===== */

	uint8_t syscfg_hdr[2] = {0x04, 0x00};
	uint8_t sys_cfg[4] = {0};

	readfromspi(2, syscfg_hdr, 4, sys_cfg);

	printk("SYS_CFG: %02X %02X %02X %02X\n",
		   sys_cfg[0],
		   sys_cfg[1],
		   sys_cfg[2],
		   sys_cfg[3]);

	k_msleep(10);

	/* ===== PROBE DRIVER ===== */

	if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) == DWT_ERROR)
	{
		printk("PROBE FAILED\n");

		while (1)
		{
			k_sleep(K_SECONDS(1));
		}
	}

	printk("PROBE OK\n");

	while (1)
	{
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
