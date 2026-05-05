#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/devicetree.h>
#include <string.h>

/* ===== SPI node (correct for DWM3001CDK) ===== */
#define SPI_NODE DT_NODELABEL(spi3)

static const struct device *spi_dev;

/* ===== SPI config ===== */
static const struct spi_config spi_cfg = {
	.frequency = 1000000,
	.operation = SPI_WORD_SET(8) |
				 SPI_TRANSFER_MSB |
				 SPI_MODE_CPOL |
				 SPI_MODE_CPHA, // MODE 3 (DW3000 requirement)
	.slave = 0,
	.cs = NULL, // CS handled by devicetree
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

	printk("SPI init done\n");
}

/* ===== SPI READ ===== */
int readfromspi(uint16_t headerLength,
				const uint8_t *headerBuffer,
				uint32_t readLength,
				uint8_t *readBuffer)
{
	uint8_t tx_buf[256];
	uint8_t rx_buf[256];

	uint32_t total_len = headerLength + readLength;

	if (total_len > sizeof(tx_buf))
	{
		return -1;
	}

	/* Build TX: header + dummy bytes */
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

	int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);

	if (ret)
	{
		printk("SPI error %d\n", ret);
		return ret;
	}

	/* Copy RX (skip header) */
	memcpy(readBuffer, rx_buf + headerLength, readLength);

	return 0;
}