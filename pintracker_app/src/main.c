#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/spi.h>

#define SPI_NODE DT_NODELABEL(spi3)

const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

int main(void)
{
    printk("PinTracker start\r\n");

    if (!device_is_ready(spi_dev)) {
        printk("SPI NOT READY\r\n");
        return 0;
    }

    printk("SPI READY\r\n");

    while (1) {
        printk("alive\r\n");
        k_sleep(K_SECONDS(1));
    }
}
