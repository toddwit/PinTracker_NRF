#include <zephyr/kernel.h>
#include <stdint.h>

extern void dw3000_port_init(void);
extern int readfromspi(uint16_t headerLength,
					   const uint8_t *headerBuffer,
					   uint32_t readlength,
					   uint8_t *readBuffer);

int main(void)
{
	printk("DW3000 SPI read test\n");

	dw3000_port_init();

	uint8_t header[1] = {0x00}; // register 0x00 (DEV_ID)
	uint8_t id[4] = {0};

	readfromspi(1, header, 4, id);

	printk("DEV_ID: %02X %02X %02X %02X\n",
		   id[0], id[1], id[2], id[3]);

	while (1)
	{
		k_sleep(K_SECONDS(1));
	}
}