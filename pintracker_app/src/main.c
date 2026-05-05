#include <zephyr/kernel.h>

extern void dw3000_port_init(void);

int main(void)
{
	printk("SPI test start\n");

	dw3000_port_init();

	printk("SPI init done\n");

	while (1)
	{
		printk("Alive\n");
		k_sleep(K_SECONDS(1));
	}
}