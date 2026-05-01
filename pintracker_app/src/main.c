#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#define RST_NODE DT_NODELABEL(gpio0)
static const struct device *rst_gpio = DEVICE_DT_GET(RST_NODE);

#define RST_PIN 28   // adjust if needed

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

int dw_write_reg(uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t header[1] = {reg | 0x80}; // write = set MSB

    struct spi_buf tx_bufs[2] = {
        { .buf = header, .len = 1 },
        { .buf = data,   .len = len }
    };

    struct spi_buf_set tx = { .buffers = tx_bufs, .count = 2 };

    cs_select();
    int ret = spi_write(spi_dev, &spi_cfg, &tx);
    cs_deselect();

    return ret;
}

int dw_read_reg(uint8_t reg, uint8_t *buf, size_t len)
{
    uint8_t header[1] = {reg};

    struct spi_buf tx_bufs[2] = {
        { .buf = header, .len = 1 },
        { .buf = NULL,   .len = len }
    };

    struct spi_buf rx_bufs[2] = {
        { .buf = NULL, .len = 1 },
        { .buf = buf,  .len = len }
    };

    struct spi_buf_set tx = { .buffers = tx_bufs, .count = 2 };
    struct spi_buf_set rx = { .buffers = rx_bufs, .count = 2 };

    cs_select();
    int ret = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
    cs_deselect();

    return ret;
}

void dw_reset(void)
{
    if (!device_is_ready(rst_gpio)) {
        return;
    }

    gpio_pin_configure(rst_gpio, RST_PIN, GPIO_OUTPUT_ACTIVE);

    // Drive reset low
    gpio_pin_set(rst_gpio, RST_PIN, 0);
    k_sleep(K_MSEC(2));

    // Release reset (high)
    gpio_pin_set(rst_gpio, RST_PIN, 1);
    k_sleep(K_MSEC(5));
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

    dw_reset();
    k_sleep(K_MSEC(1000));
    printk("DW reset done\r\n");

    while (1)
    {
        uint8_t id[4];

        dw_read_reg(0x00, id, 4);

        uint32_t dev_id =
            (id[3] << 24) |
            (id[2] << 16) |
            (id[1] << 8)  |
            (id[0]);

        printk("DEV_ID: 0x%08X\r\n", dev_id);

        k_sleep(K_SECONDS(1));
    }
}
