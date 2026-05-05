#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>

static const struct device *spi_dev;
static struct spi_config spi_cfg;

// Initialize SPI for DW3000
void dw3000_port_init(void)
{
	// ⚠️ This must match your board (we’ll adjust if needed)
	spi_dev = DEVICE_DT_GET_ANY(nordic_nrf_spim);

	if (!device_is_ready(spi_dev))
	{
		printk("SPI device not ready!\n");
		return;
	}

	spi_cfg.frequency = 8000000;
	spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
	spi_cfg.slave = 0;
}

// =========================
// REQUIRED BY DW3000 DRIVER
// =========================

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

void deca_sleep(unsigned int time_ms)
{
	k_msleep(time_ms);
}

void deca_usleep(unsigned long time_us)
{
	k_usleep(time_us);
}

void reset_DWIC(void)
{
	// OK to leave empty for now
}