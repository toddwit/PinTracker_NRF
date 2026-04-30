#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#define SPI_NODE DT_NODELABEL(spi3)
static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

#define CS_NODE DT_NODELABEL(gpio1)
static const struct device *cs_gpio = DEVICE_DT_GET(CS_NODE);
#define CS_PIN 6

static struct spi_config spi_cfg = {
    .frequency = 2000000,  // start slower for safety
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA | SPI_MODE_CPOL,
    .slave = 0,
};

void cs_select(void) {
    gpio_pin_set(cs_gpio, CS_PIN, 0);
}

void cs_deselect(void) {
    gpio_pin_set(cs_gpio, CS_PIN, 1);
}

int main(void)
{
    printk("PinTracker start\r\n");

    if (!device_is_ready(spi_dev)) {
        printk("SPI not ready\r\n");
        return 0;
    }

    if (!device_is_ready(cs_gpio)) {
        printk("CS GPIO not ready\r\n");
        return 0;
    }

    gpio_pin_configure(cs_gpio, CS_PIN, GPIO_OUTPUT_HIGH);

    printk("SPI + CS ready\r\n");

    while (1)
    {
       	uint8_t tx_buf[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
		uint8_t rx_buf[5] = {0};

		struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
		struct spi_buf rx = { .buf = rx_buf, .len = sizeof(rx_buf) };

		struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
		struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

		cs_select();
		int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
		cs_deselect();

		if (ret == 0) {
			uint32_t dev_id =
				(rx_buf[4] << 24) |
				(rx_buf[3] << 16) |
				(rx_buf[2] << 8)  |
				(rx_buf[1]);

			printk("DEV_ID: 0x%08X\r\n", dev_id);
		}
    }
}
