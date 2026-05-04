#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include "dw3000/deca_device_api.h"
//#include "dw3000/dw3000_deca_regs.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "deca_device_api.h"

// ===== CONFIG =====
static dwt_config_t config = {
    5,              // channel
    DWT_PLEN_128,
    DWT_PAC8,
    9,              // TX preamble code
    9,              // RX preamble code
    1,              // SFD
    DWT_BR_6M8,
    DWT_PHRMODE_STD,
    DWT_PHRRATE_STD,
    (128 + 1 + 8 - 8),
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,
    DWT_PDOA_M0
};

extern dwt_txconfig_t txconfig_options;

// ===== SIMPLE POLL FRAME =====
static uint8_t tx_poll_msg[] = {
    0x41, 0x88,
    0x00,
    0xCA, 0xDE,
    0x01, 0x02,
    0x03, 0x04,
    'P','O','L','L'
};

static uint8_t rx_buffer[32];



// ─────────────────────────────────────────────
// SYSTEM CONFIGURATION
// ─────────────────────────────────────────────

// ───────────── BUILD CONFIG ─────────────

#define FW_VERSION "v1.0"

#define DEBUG_LEVEL   1   // 0 = off, 1 = normal, 2 = verbose
#define DEBUG_UWB     0
#define DEBUG_RX      0
#define USE_DISPLAY   1

#define DEBUG_PRINT(...)   printk(__VA_ARGS__)
#define DEBUG_PRINTLN(...) printk(__VA_ARGS__)
#define DEBUG_PRINTF(...)  printk(__VA_ARGS__)

// ───────────── HARDWARE PINS ─────────────
const int PIN_RST = 27;
const int PIN_IRQ = 34;
const int PIN_SS  =  4;

#define SYS_STATUS_ID 0x0F
#define RX_FINFO_ID   0x10

#define SYS_STATUS_RXFCG_BIT_MASK (1UL << 7)

// ───────────── SYSTEM SIZES ─────────────
#define MAX_ANCHORS 3
#define MEDIAN_WINDOW 7
#define DISPLAY_BUFFER_LEN 16
#define RX_BUFFER_LEN 22
#define CAL_SAMPLES 50


// ───────────── TIMING ─────────────
const uint32_t RANGE_INTERVAL_MS   = 100;
const uint32_t DISPLAY_INTERVAL_MS = 500;

const uint32_t RX_TIMEOUT_MS      = 20;
const uint32_t RX_EARLY_EXIT_MS   = 12;
const uint32_t TX_TIMEOUT_MS      = 50;


// ───────────── UNIT CONVERSIONS ─────────────
const float METERS_TO_INCHES = 39.3701f;


// ───────────── CALIBRATION ─────────────
const float DISTANCE_SCALE  = 1.0625f;
const float DISTANCE_OFFSET = -0.159f;


// ───────────── ANCHOR PROCESSING ─────────────
const float MAX_ANCHOR_JUMP_METERS   = 1.5f;   // reject extreme spikes
const float ANCHOR_DISAGREE_THRESH   = 0.25f;  // outlier rejection
const float MAX_ANCHOR_ERROR_METERS  = 0.5f;   // weighted fusion cutoff
const float WEIGHT_DECAY             = 6.0f;   // exp falloff strength
const float ANCHOR_REJECT_THRESH     = 0.18f;


// ───────────── CONFIDENCE MODEL ─────────────
const float SPREAD_THRESH_1 = 0.20f;
const float SPREAD_THRESH_2 = 0.40f;
const float SPREAD_THRESH_3 = 0.70f;

const float CONF_SPREAD_PENALTY_1 = 0.7f;
const float CONF_SPREAD_PENALTY_2 = 0.4f;
const float CONF_SPREAD_PENALTY_3 = 0.2f;

const float CONF_ANCHOR_2 = 0.8f;
const float CONF_ANCHOR_1 = 0.5f;

const float CONF_MOTION_PENALTY = 0.7f;
const float CONF_MIN            = 0.05f;


// ───────────── FILTER RESPONSE ─────────────
const float ALPHA_MIN = 0.05f;
const float ALPHA_MAX = 0.35f;

const float VELOCITY_LP_ALPHA = 0.2f;
const float VELOCITY_LP_DECAY = 0.8f;


// ───────────── LOCK STATE MACHINE ─────────────
const float LOCK_ACQUIRE_TOL_METERS = 0.05f;
const float LOCK_BREAK_TOL_METERS   = 0.20f;
const float LOCK_MIN_CONFIDENCE     = 80.0f;

const uint8_t   LOCK_CONFIRM_COUNT  = 5;

// ───────────── VELOCITY ─────────────
const float VELOCITY_BASE = 0.015f;
const float VELOCITY_DISTANCE_SCALE = 0.004f;
const float VELOCITY_LOCK_MULTIPLIER = 1.5f;


// ───────────── FREEZE LOGIC ─────────────
const float FREEZE_MAX_VELOCITY   =  0.03f;

const float FREEZE_MIN_CONFIDENCE = 65.0f;
const float UNFREEZE_DELTA_METERS =  0.2f;
const float VERY_STABLE_RATIO     =  0.8f;


// ───────────── RELIABILITY ─────────────
const float MAX_SPREAD_FOR_RELIABILITY = 1.2f;
const float MIN_CONFIDENCE_FOR_RELIABILITY = 70.0f;


// ───────────── DISPLAY ─────────────
const int DISPLAY_HYSTERESIS_INCHES = 2;


// ───────────── UWB RADIO CONFIG ─────────────
const uint16_t TX_ANT_DLY = 16384;
const uint16_t RX_ANT_DLY = 16384;

const uint16_t POLL_TX_TO_RESP_RX_DLY_UUS = 1000;
const uint16_t RESP_RX_TIMEOUT_UUS        = 9000;
const uint16_t TX_DELAY_UUS               = 3000;

const int ALL_MSG_COMMON_LEN = 10;
const int ALL_MSG_SN_IDX = 2;

const int RESP_MSG_POLL_RX_TS_IDX = 10;
const int RESP_MSG_RESP_TX_TS_IDX = 14;
const int RESP_MSG_ANCHOR_ID_IDX  = 18;

const uint8_t DW3000_XTRIM_VALUE = 0x10;

const float DW3000_RSSI_SCALE_FACTOR = 131072.0f;   // 2^17 per DW3000 RSSI formula
const float DW3000_RSSI_OFFSET_DB    = 113.77f;     // calibration offset for DW3000
const float DW3000_NO_SIGNAL_DBM     = -120.0f;     // fallback when no valid signal


// ───────────── IMU CONFIG ─────────────
const uint8_t IMU_WARMUP_SAMPLES = 5;

const uint8_t IMU_ADDR = 0x6B;

const float MOTION_ON_THRESH  = 0.06f;
const float MOTION_OFF_THRESH = 0.03f;

const float ROT_ON_THRESH  = 25.0f;
const float ROT_OFF_THRESH = 10.0f;

const float IMU_ROTATION_LP_ALPHA = 0.15f;
const float IMU_ROTATION_MAX = 300.0f;

const float IMU_GRAVITY = 1.0f;




const float CUP_DIAMETER_METERS = 0.108f;
const float DEBUG_STABILITY_MAX_SPREAD = 0.4f;



// ───────────── TYPES ─────────────
struct FilterState
{
  float smoothed;
  float confidence;
  float velocity_lp;
  bool  valid;
};

enum LockState
{
  SEARCHING,
  CANDIDATE,
  LOCKED
};

struct AnchorMeasurement
{
  uint8_t id;
  float   distance;
  float   rssi;
  bool    valid;
  bool    rejected;
};

struct UWBState
{
  // Buffers / radio
  uint8_t rx_buffer[RX_BUFFER_LEN];
  uint8_t frame_seq_nb;
  uint32_t status_reg;

  // Anchors
//  AnchorMeasurement anchors[MAX_ANCHORS];
  uint8_t anchor_count;
  float distance_spread;
  bool last_anchor_rejected[MAX_ANCHORS];

  // Signal
  float signal_strength;
  float signal_strength_peak;
};

struct IMUState
{
  // Raw sensor values
  float ax, ay, az;
  float gx, gy, gz;

  // Derived values
  float accel_mag;
  float motion;
  float rotation_rate;

  // Motion state (hysteresis)
  bool moving;

  // Warmup counter
  uint8_t warmup;
};

struct RangeState
{
  // Median filter buffer
  float buffer[MEDIAN_WINDOW];
  uint8_t index;
  uint8_t count;

  // Raw measurement tracking
  float last_raw_distance;
  float raw_velocity;
};
/*
struct DisplayState
{
  TFT_eSPI display;
  bool ready = false;

  bool frozen = false;
  float frozen_distance = 0.0f;
  int stable_count = 0;

  bool lock_displayed = false;

  int displayed_feet = -1;
  int displayed_inches = -1;
  int displayed_total_inches = -1;
};

struct DebugSnapshot
{
  uint32_t dt_ms;
  uint32_t cycle;

  float median_now;
  float display_distance;
  float spread;

  char disp_buf[DISPLAY_BUFFER_LEN];
  char motion_str[6];
  char lock_str[6];
};
*/

// ───────────── GLOBAL STATE ─────────────

// Calibration
float cal_buffer[MAX_ANCHORS][CAL_SAMPLES];
uint16_t cal_index[MAX_ANCHORS] = {0};

// Filter & Lock
//FilterState filter;
//LockState lockState = SEARCHING;
float lock_distance;
int lock_counter;

// UWB
//UWBState  uwb;

// IMU
//IMUState imu;
bool imuPresent;

// Display
//DisplayState displayState;
float current_display_distance;
float display_confidence;

// Ranging
//RangeState rangeState;

// Timing
uint32_t last_range_ms;
uint32_t last_print_ms;

// Debug/Stats
volatile uint16_t rx_attempt_count = 0;
volatile uint16_t rx_valid_count = 0;

int debug_health_valid = 0;
int debug_health_attempt = 0;

// UWB Config
extern dwt_txconfig_t txconfig_options;   // defined in dw3000 library
/*
static dwt_config_t config = 
{
  5,                  // UWB channel 5 = 6.4896 GHz center, commonly used for short-range
  DWT_PLEN_256,       // TX preamble length (256 symbols), short = faster, lower power, long = more robust at long range
  DWT_PAC16,          // RX Preamble Acquisition Chunk size (16 symbols)
  9,                  // TX preamble code index
  9,                  // RX preamble code index (same as TX for symmetric SS-TWR)
  1,                  // SFD type 0 = Standard 8-symbol SFD (IEEE 802.15.4)
  DWT_BR_6M8,         // Data rate = 6.8 Mbps (maximum)
  DWT_PHRMODE_STD,    // PHY header mode = standard (explicit)
  DWT_PHRRATE_STD,    // PHY header rate = standard
  (256 + 1 + 8 - 16), // SFD timeout = preamble length + 1 + SFD length - PAC size
  DWT_STS_MODE_OFF,   // STS (Scrambled Timestamp Sequence) disabled
  DWT_STS_LEN_64,     // STS length (ignored when STS is off)
  DWT_PDOA_M0         // Phase Difference of Arrival (PDOA) mode off
};
*/
// messages
static uint8_t tx_poll_msg1[] = {
  0xC1, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0
};

static uint8_t rx_resp_msg1[] = {
  0xC1, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1,  // 10-byte header doesn't change
  0, 0, 0, 0,       //  4-byte RX timestamp
  0, 0, 0, 0,       //  4-byte TX timestamp
  0,                //  1-byte anchor ID
  0,                //  1-byte spare
  0, 0              //  2-byte crc
};



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
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB
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

void dw_wait_ready(void)
{
    uint32_t dev_id = 0;

    while (dev_id != 0xDECA0302) {
        uint8_t id[4];

        dw_read_reg(0x00, id, 4);

        dev_id =
            (id[3] << 24) |
            (id[2] << 16) |
            (id[1] << 8)  |
            (id[0]);

        k_sleep(K_MSEC(5));
    }

    printk("DW3000 ready\r\n");
}


bool runRangingCycle(float *distance)
{
    // 1. Send poll
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);

	return false;
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
	dwt_rxenable(0);

    // 2. Wait for response
    uint32_t status;
    int timeout = 1000;

    do {
        status = dwt_read_reg(SYS_STATUS_ID);
        timeout--;
    } while (!(status & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)) && timeout > 0);

    if (timeout <= 0) {
        printk("RX TIMEOUT\r\n");
        return false;
    }

    // 3. Check if good frame received
    if (status & SYS_STATUS_RXFCG_BIT_MASK)
    {
        // Read frame length
        uint32_t finfo = dwt_read_reg(RX_FINFO_ID);
        uint32_t frame_len = finfo & 0x7F;

        uint8_t rx_buffer[32];
        dwt_readrxdata(rx_buffer, frame_len, 0);

        // Clear RX good flag
        dwt_write_reg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

        // 4. Compute distance (placeholder)
        *distance = 1.23f;

        return true;
    }
    else
    {
        // Clear RX error flags
        dwt_write_reg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        return false;
    }
}

int writetospi(uint16_t headerLength,
               const uint8_t *headerBuffer,
               uint16_t bodyLength,
               const uint8_t *bodyBuffer)
{
    struct spi_buf bufs[2];
    struct spi_buf_set tx;

    bufs[0].buf = (void *)headerBuffer;
    bufs[0].len = headerLength;

    if (bodyLength > 0) {
        bufs[1].buf = (void *)bodyBuffer;
        bufs[1].len = bodyLength;
        tx.buffers = bufs;
        tx.count = 2;
    } else {
        tx.buffers = bufs;
        tx.count = 1;
    }

    cs_select();
    int ret = spi_write(spi_dev, &spi_cfg, &tx);
    cs_deselect();

    return ret;
}

int readfromspi(uint16_t headerLength,
                const uint8_t *headerBuffer,
                uint16_t readLength,
                uint8_t *readBuffer)
{
    uint8_t dummy[32];  // must be >= max headerLength

    struct spi_buf tx_buf = {
        .buf = (void *)headerBuffer,
        .len = headerLength
    };

    struct spi_buf rx_bufs[2] = {
        { .buf = dummy,      .len = headerLength },  // <-- FIXED
        { .buf = readBuffer, .len = readLength }
    };

    struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1
    };

    struct spi_buf_set rx = {
        .buffers = rx_bufs,
        .count = 2
    };

    cs_select();
    int ret = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
    cs_deselect();

    return ret;
}



// ===== MAIN =====
int main(void)
{
    printk("PinTracker start\r\n");

    printk("A\r\n");

	if (!device_is_ready(spi_dev)) {
		printk("SPI not ready\r\n");
		return 0;
	}

	printk("B\r\n");

	if (!device_is_ready(cs_gpio)) {
		printk("CS GPIO not ready\r\n");
		return 0;
	}

	printk("C\r\n");

	gpio_pin_configure(cs_gpio, CS_PIN, GPIO_OUTPUT_HIGH);

	printk("D\r\n");

    k_sleep(K_MSEC(20));

	dwt_txconfig_t txconfig_options = {
			0x34,
			0x0F0F0F0F
		};

	dw_reset();
	dw_wait_ready();

	printk("DW3000 ready\r\n");


    // =========================
    // 🔥 LOOP
    // =========================
    while (1)
    {
        // Load TX data
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);

        // Start TX
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        printk("TX sent\r\n");

        // Wait for RX
        uint32_t status;
        int timeout = 1000;

        do {
            status = dwt_read_reg(SYS_STATUS_ID);
            timeout--;
        } while (!(status & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)) && timeout > 0);

        if (status & SYS_STATUS_RXFCG_BIT_MASK)
        {
            printk("RX OK\r\n");

            uint32_t frame_len = dwt_read_reg(RX_FINFO_ID) & 0x7F;

            if (frame_len <= sizeof(rx_buffer))
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            // clear RX flag
            dwt_write_reg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        }
        else
        {
            printk("RX TIMEOUT\r\n");

            // clear errors
            dwt_write_reg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }

        k_sleep(K_MSEC(500));
    }
}
/*
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
    printk("DW reset done\r\n");

	dwt_softreset();

// wait until device is ready
while (!dwt_checkidlerc()) {
    k_sleep(K_MSEC(1));
}

printk("DW idle ready\r\n");

	k_sleep(K_MSEC(20));

	if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    printk("dwt_initialise failed\r\n");
    while (1);
}

dwt_config_t config = {
    5,              // channel
    DWT_PLEN_256,
    DWT_PAC16,
    9,              // tx preamble code
    9,              // rx preamble code
    1,              // SFD
    DWT_BR_6M8,
    DWT_PHRMODE_STD,
    DWT_PHRRATE_STD,
    (256 + 1 + 8 - 16),
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,
    DWT_PDOA_M0
};

if (dwt_configure(&config)) {
    printk("dwt_configure failed\r\n");
    while (1);
}

dwt_setrxantennadelay(16384);
dwt_settxantennadelay(16384);

printk("DW3000 configured (dwt)\r\n");

    dw_wait_ready();


	while (1)
	{
		float distance;

		if (runRangingCycle(&distance))
		{
			printk("Distance: %.2f m\r\n", distance);
		}
		else
		{
			printk("Ranging failed\r\n");
		}

		k_sleep(K_MSEC(500));
	}
}

*/