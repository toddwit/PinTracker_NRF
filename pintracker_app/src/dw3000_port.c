#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>

/* ===== SPI1 (from schematic) ===== */
#define SPI_NODE DT_NODELABEL(spi1)

/* ===== Correct DW3000 pins ===== */
/* CS = P0.30 */
static const struct gpio_dt_spec cs_gpio = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
	.pin = 30,
	.dt_flags = GPIO_ACTIVE_LOW,
};

/* RESET = P0.18 */
static const struct gpio_dt_spec rst_gpio = {
	.port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
	.pin = 18,
	.dt_flags = GPIO_ACTIVE_LOW,
};

/* ===== SPI device + config ===== */
static const struct device *spi_dev;
static struct spi_config spi_cfg;

/* ===== Init ===== */
int dw3000_port_init(void)
{
	spi_dev = DEVICE_DT_GET(SPI_NODE);

	if (!device_is_ready(spi_dev))
	{
		printk("SPI not ready!\n");
		return -1;
	}

	if (!device_is_ready(cs_gpio.port))
	{
		printk("CS GPIO not ready!\n");
		return -1;
	}

	if (!device_is_ready(rst_gpio.port))
	{
		printk("RST GPIO not ready!\n");
		return -1;
	}

	/* Configure CS */
	gpio_pin_configure_dt(&cs_gpio, GPIO_OUTPUT_INACTIVE);

	/* Configure RESET */
	gpio_pin_configure_dt(&rst_gpio, GPIO_OUTPUT_ACTIVE);

	/* Reset DW3000 */
	gpio_pin_set_dt(&rst_gpio, 0);
	k_msleep(2);
	gpio_pin_set_dt(&rst_gpio, 1);
	k_msleep(10);

	printk("DW3000 reset done\n");

	/* SPI CS control */
	static struct spi_cs_control cs_ctrl = {
		.gpio = cs_gpio,
		.delay = 0,
	};

	/* SPI config (Mode 3 required for DW3000) */
	spi_cfg.frequency = 1000000;
	spi_cfg.operation =
		SPI_WORD_SET(8) |
		SPI_TRANSFER_MSB |
		SPI_MODE_CPOL |
		SPI_MODE_CPHA;

	spi_cfg.slave = 0;
	spi_cfg.cs = cs_ctrl;

	printk("SPI init done\n");

	return 0;
}

/* ===== SPI WRITE ===== */
int writetospi(uint16_t headerLength,
			   const uint8_t *headerBuffer,
			   uint32_t bodyLength,
			   const uint8_t *bodyBuffer)
{
	struct spi_buf tx_bufs[2];
	struct spi_buf_set tx;

	size_t count = 0;

	if (headerLength > 0 && headerBuffer)
	{
		tx_bufs[count].buf = (void *)headerBuffer;
		tx_bufs[count].len = headerLength;
		count++;
	}

	if (bodyLength > 0 && bodyBuffer)
	{
		tx_bufs[count].buf = (void *)bodyBuffer;
		tx_bufs[count].len = bodyLength;
		count++;
	}

	tx.buffers = tx_bufs;
	tx.count = count;

	return spi_write(spi_dev, &spi_cfg, &tx);
}

/* ===== SPI READ ===== */
int readfromspi(uint16_t headerLength,
				const uint8_t *headerBuffer,
				uint32_t readlength,
				uint8_t *readBuffer)
{
	struct spi_buf tx_buf = {
		.buf = (void *)headerBuffer,
		.len = headerLength};

	struct spi_buf rx_buf = {
		.buf = readBuffer,
		.len = readlength};

	struct spi_buf_set tx_set = {
		.buffers = &tx_buf,
		.count = 1};

	struct spi_buf_set rx_set = {
		.buffers = &rx_buf,
		.count = 1};

	return spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
}