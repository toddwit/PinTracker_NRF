#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <string.h>

/* ===== SPI ===== */
#define SPI_NODE DT_NODELABEL(spi2)

/* ===== Pins ===== */
#define PIN_CS 6
#define PIN_RST 25
#define PIN_WAKEUP 19

static const struct device *spi_dev;
static const struct device *gpio0_dev;
static const struct device *gpio1_dev;

/* ===== SPI config ===== */
static const struct spi_config spi_cfg = {
	.frequency = 1000000,
	.operation = SPI_WORD_SET(8) |
				 SPI_TRANSFER_MSB |
				 SPI_MODE_CPOL |
				 SPI_MODE_CPHA,
	.slave = 0,
	.cs = {0},
};

/* ===== INIT ===== */
void spi_init(void)
{
	spi_dev = DEVICE_DT_GET(SPI_NODE);

	if (!device_is_ready(spi_dev))
	{
		printk("SPI not ready!\n");
		return;
	}

	gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	gpio1_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));

	if (!device_is_ready(gpio0_dev) ||
		!device_is_ready(gpio1_dev))
	{
		printk("GPIO not ready!\n");
		return;
	}

	/* ===== Configure pins ===== */

	gpio_pin_configure(gpio1_dev, PIN_CS, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set(gpio1_dev, PIN_CS, 1);

	gpio_pin_configure(gpio1_dev, PIN_WAKEUP, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set(gpio1_dev, PIN_WAKEUP, 1);

	gpio_pin_configure(gpio0_dev, PIN_RST, GPIO_OUTPUT_ACTIVE);

	/* ===== Hardware reset ===== */

	gpio_pin_set(gpio0_dev, PIN_RST, 0);
	k_msleep(10);

	gpio_pin_set(gpio0_dev, PIN_RST, 1);
	k_msleep(10);

	printk("SPI init done\n");
}

/* ===== SPI WRITE ===== */
int writetospi(uint16_t headerLength,
			   const uint8_t *headerBuffer,
			   uint16_t bodyLength,
			   const uint8_t *bodyBuffer)
{
	uint8_t tx_buf[256];

	uint32_t total_len = headerLength + bodyLength;

	if (total_len > sizeof(tx_buf))
	{
		return -1;
	}

	memcpy(tx_buf, headerBuffer, headerLength);

	if (bodyLength > 0)
	{
		memcpy(tx_buf + headerLength,
			   bodyBuffer,
			   bodyLength);
	}

	struct spi_buf tx = {
		.buf = tx_buf,
		.len = total_len,
	};

	struct spi_buf_set tx_set = {
		.buffers = &tx,
		.count = 1,
	};

	gpio_pin_set(gpio1_dev, PIN_CS, 0);

	int ret = spi_write(spi_dev, &spi_cfg, &tx_set);

	gpio_pin_set(gpio1_dev, PIN_CS, 1);

	if (ret)
	{
		printk("SPI write error %d\n", ret);
	}

	return ret;
}

/* ===== SPI READ ===== */
int readfromspi(uint16_t headerLength,
				uint8_t *headerBuffer,
				uint16_t readLength,
				uint8_t *readBuffer)
{
	uint8_t tx_buf[256];
	uint8_t rx_buf[256];

	uint32_t total_len = headerLength + readLength;

	if (total_len > sizeof(tx_buf))
	{
		return -1;
	}

	memcpy(tx_buf, headerBuffer, headerLength);
	memset(tx_buf + headerLength, 0x00, readLength);

	struct spi_buf tx = {
		.buf = tx_buf,
		.len = total_len,
	};

	struct spi_buf rx = {
		.buf = rx_buf,
		.len = total_len,
	};

	struct spi_buf_set tx_set = {
		.buffers = &tx,
		.count = 1,
	};

	struct spi_buf_set rx_set = {
		.buffers = &rx,
		.count = 1,
	};

	gpio_pin_set(gpio1_dev, PIN_CS, 0);

	int ret = spi_transceive(spi_dev,
							 &spi_cfg,
							 &tx_set,
							 &rx_set);

	gpio_pin_set(gpio1_dev, PIN_CS, 1);

	if (ret)
	{
		printk("SPI read error %d\n", ret);
		return ret;
	}

	memcpy(readBuffer,
		   rx_buf + headerLength,
		   readLength);

	return 0;
}
