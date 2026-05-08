#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "dw3000_deca_regs.h"
#include "dw3000_port.h"

extern void spi_init(void);

#define FW_VERSION "v1.0-nrf"

#define MAX_ANCHORS 3U
#define MEDIAN_WINDOW 7U
#define RX_BUFFER_LEN 22U
#define CAL_SAMPLES 50U

#define RANGE_INTERVAL_MS 100U
#define RX_TIMEOUT_MS 20U
#define RX_EARLY_EXIT_MS 12U
#define TX_TIMEOUT_MS 50U
#define ANCHOR_POLL_TIMEOUT_MS 1000U

#define DISTANCE_SCALE 1.0625f
#define DISTANCE_OFFSET (-0.159f)
#define MAX_ANCHOR_JUMP_METERS 1.5f
#define ANCHOR_REJECT_THRESH 0.18f
#define WEIGHT_DECAY 6.0f
#define CUP_DIAMETER_METERS 0.108f
#define DEBUG_STABILITY_MAX_SPREAD 0.4f

#define CONF_ANCHOR_2 0.8f
#define CONF_ANCHOR_1 0.5f
#define CONF_MOTION_PENALTY 0.7f
#define CONF_MIN 0.05f

#define ALPHA_MIN 0.05f
#define ALPHA_MAX 0.35f
#define VELOCITY_LP_ALPHA 0.2f
#define VELOCITY_LP_DECAY 0.8f

#define LOCK_ACQUIRE_TOL_METERS 0.05f
#define LOCK_BREAK_TOL_METERS 0.20f
#define LOCK_MIN_CONFIDENCE 80.0f
#define LOCK_CONFIRM_COUNT 5U

#define TX_ANT_DLY 16384U
#define RX_ANT_DLY 16384U
#define POLL_TX_TO_RESP_RX_DLY_UUS 1000U
#define RESP_RX_TIMEOUT_UUS 9000U
#define PREAMBLE_TIMEOUT_PAC 0U
#define POLL_RX_TO_RESP_TX_DLY_UUS 3000U
#define UUS_TO_DWT_TIME 65536ULL

#define ALL_MSG_COMMON_LEN 10U
#define ALL_MSG_SN_IDX 2U
#define RESP_MSG_POLL_RX_TS_IDX 10U
#define RESP_MSG_RESP_TX_TS_IDX 14U
#define RESP_MSG_ANCHOR_ID_IDX 18U
#define DW3000_XTRIM_VALUE 0x10U

#define DW3000_RSSI_SCALE_FACTOR 131072.0f
#define DW3000_RSSI_OFFSET_DB 113.77f
#define DW3000_NO_SIGNAL_DBM (-120.0f)
#define SPEED_OF_LIGHT 299702547.0f

#ifndef PINTRACKER_ANCHOR_ID
#define PINTRACKER_ANCHOR_ID 0U
#endif

typedef struct {
	float smoothed;
	float confidence;
	float velocity_lp;
	bool valid;
} filter_state_t;

typedef enum {
	SEARCHING,
	CANDIDATE,
	LOCKED,
} lock_state_t;

typedef struct {
	uint8_t id;
	float distance;
	float rssi;
	bool valid;
	bool rejected;
} anchor_measurement_t;

typedef struct {
	uint8_t rx_buffer[RX_BUFFER_LEN];
	uint8_t frame_seq_nb;
	uint32_t status_reg;
	anchor_measurement_t anchors[MAX_ANCHORS];
	uint8_t anchor_count;
	float distance_spread;
	float signal_strength;
	float signal_strength_peak;
} uwb_state_t;

typedef struct {
	float buffer[MEDIAN_WINDOW];
	uint8_t index;
	uint8_t count;
	float last_raw_distance;
	float raw_velocity;
} range_state_t;

static filter_state_t filter;
static lock_state_t lock_state = SEARCHING;
static float lock_distance;
static uint8_t lock_counter;
static uwb_state_t uwb;
static range_state_t range_state;

static float cal_buffer[MAX_ANCHORS][CAL_SAMPLES];
static uint16_t cal_index[MAX_ANCHORS];
static uint16_t rx_attempt_count;
static uint16_t rx_valid_count;

static dwt_config_t config = {
	5,
	DWT_PLEN_256,
	DWT_PAC16,
	9,
	9,
	DWT_SFD_DW_8,
	DWT_BR_6M8,
	DWT_PHRMODE_STD,
	DWT_PHRRATE_STD,
	(256 + 1 + 8 - 16),
	DWT_STS_MODE_OFF,
	DWT_STS_LEN_64,
	DWT_PDOA_M0,
};

/* Common Qorvo example defaults for DW3000 channel 5; tune for production hardware. */
static dwt_txconfig_t txconfig = {
	.PGdly = 0x34,
	.power = 0xFDFDFDFD,
	.PGcount = 0x0000,
};

static uint8_t tx_poll_msg[] = {
	0xC1, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0,
};

static uint8_t rx_resp_msg[] = {
	0xC1, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1,
	0, 0, 0, 0,
	0, 0, 0, 0,
	0,
	0,
	0, 0,
};

static uint8_t rx_poll_msg[] = {
	0xC1, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0,
};

static uint8_t tx_resp_msg[] = {
	0xC1, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1,
	0, 0, 0, 0,
	0, 0, 0, 0,
	PINTRACKER_ANCHOR_ID,
	0,
	0, 0,
};

static uint64_t get_tx_timestamp_u64(void)
{
	uint8_t ts[5];
	uint64_t value = 0;

	dwt_readtxtimestamp(ts);

	for (int i = 4; i >= 0; i--) {
		value <<= 8;
		value |= ts[i];
	}

	return value;
}

static uint64_t get_rx_timestamp_u64(void)
{
	uint8_t ts[5];
	uint64_t value = 0;

	dwt_readrxtimestamp(ts, DWT_COMPAT_NONE);

	for (int i = 4; i >= 0; i--) {
		value <<= 8;
		value |= ts[i];
	}

	return value;
}

static void resp_msg_get_ts(const uint8_t *ts_field, uint32_t *ts)
{
	*ts = ((uint32_t)ts_field[0]) |
	      ((uint32_t)ts_field[1] << 8) |
	      ((uint32_t)ts_field[2] << 16) |
	      ((uint32_t)ts_field[3] << 24);
}

static void resp_msg_set_ts(uint8_t *ts_field, uint64_t ts)
{
	for (uint8_t i = 0; i < 4U; i++) {
		ts_field[i] = (uint8_t)ts;
		ts >>= 8;
	}
}

static void clear_anchor_data(void)
{
	uwb.anchor_count = 0;

	for (uint8_t i = 0; i < MAX_ANCHORS; i++) {
		uwb.anchors[i].valid = false;
		uwb.anchors[i].rejected = false;
	}
}

static void restart_receiver(void)
{
	dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
	k_busy_wait(5);
	(void)dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void handle_radio_errors(void)
{
	uint32_t status = dwt_readsysstatuslo();

	if ((status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0U) {
		dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
	}
}

static bool send_poll(void)
{
	tx_poll_msg[ALL_MSG_SN_IDX] = uwb.frame_seq_nb;

	dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK |
			       SYS_STATUS_RXFCG_BIT_MASK |
			       SYS_STATUS_ALL_RX_TO |
			       SYS_STATUS_ALL_RX_ERR);

	dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
	dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);

	if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
		return false;
	}

	int64_t start = k_uptime_get();

	while ((dwt_readsysstatuslo() & SYS_STATUS_TXFRS_BIT_MASK) == 0U) {
		if ((k_uptime_get() - start) > TX_TIMEOUT_MS) {
			printk("TX timeout\n");
			restart_receiver();
			return false;
		}
		k_sleep(K_MSEC(1));
	}

	dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK);
	restart_receiver();
	rx_attempt_count++;
	return true;
}

static void extract_signal_quality(void)
{
	dwt_rxdiag_t diag;

	dwt_readdiagnostics(&diag);

	if ((diag.ipatovAccumCount > 0U) && (diag.ipatovPower > 0U)) {
		float n = (float)diag.ipatovAccumCount;
		float c = (float)diag.ipatovPower;

		uwb.signal_strength = 10.0f * log10f((c * DW3000_RSSI_SCALE_FACTOR) / (n * n)) - DW3000_RSSI_OFFSET_DB;
	} else {
		uwb.signal_strength = DW3000_NO_SIGNAL_DBM;
	}

	uwb.signal_strength_peak = uwb.signal_strength;
}

static bool process_anchor_response(void)
{
	uint8_t rng = 0;
	uint16_t frame_len = dwt_getframelength(&rng);

	if ((frame_len < RX_BUFFER_LEN) || (frame_len > sizeof(uwb.rx_buffer))) {
		return false;
	}

	dwt_readrxdata(uwb.rx_buffer, frame_len, 0);

	uint8_t seq = uwb.rx_buffer[ALL_MSG_SN_IDX];
	uwb.rx_buffer[ALL_MSG_SN_IDX] = 0;
	bool valid = (memcmp(uwb.rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0);
	uwb.rx_buffer[ALL_MSG_SN_IDX] = seq;

	if (!valid) {
		return false;
	}

	uint64_t resp_rx_ts = get_rx_timestamp_u64();
	uint64_t poll_tx_ts = get_tx_timestamp_u64();
	uint32_t poll_rx_ts;
	uint32_t resp_tx_ts;

	resp_msg_get_ts(&uwb.rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
	resp_msg_get_ts(&uwb.rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

	int64_t rtd_init = (int64_t)(resp_rx_ts - poll_tx_ts);
	int64_t rtd_resp = (int64_t)(resp_tx_ts - poll_rx_ts);

	if ((rtd_init <= 0) || (rtd_resp <= 0)) {
		return false;
	}

	float clock_offset_ratio = ((float)dwt_readclockoffset()) / (float)(1UL << 26);
	float tof = ((float)rtd_init - ((float)rtd_resp * (1.0f - clock_offset_ratio))) * 0.5f * (float)DWT_TIME_UNITS;
	float distance = (tof * SPEED_OF_LIGHT * DISTANCE_SCALE) + DISTANCE_OFFSET;

	if (distance < 0.0f) {
		distance = 0.0f;
	}

	extract_signal_quality();

	uint8_t anchor_id = uwb.rx_buffer[RESP_MSG_ANCHOR_ID_IDX];
	static float last_dist[MAX_ANCHORS];
	bool rejected = false;

	if (anchor_id < MAX_ANCHORS) {
		float prev = last_dist[anchor_id];

		if ((prev > 0.0f) && (fabsf(distance - prev) > MAX_ANCHOR_JUMP_METERS)) {
			rejected = true;
			printk("[REJECT] A%u %.2f -> %.2f\n", anchor_id, (double)prev, (double)distance);
		}

		if (!rejected) {
			last_dist[anchor_id] = distance;
		}
	}

	if (!rejected && (uwb.anchor_count < MAX_ANCHORS)) {
		anchor_measurement_t *anchor = &uwb.anchors[uwb.anchor_count++];

		anchor->id = anchor_id;
		anchor->distance = distance;
		anchor->rssi = uwb.signal_strength_peak;
		anchor->valid = true;
		anchor->rejected = false;
		rx_valid_count++;
	}

	return true;
}

static bool collect_anchor_responses(void)
{
	int64_t start = k_uptime_get();
	int64_t first_rx_time = 0;

	while ((k_uptime_get() - start) < RX_TIMEOUT_MS) {
		uwb.status_reg = dwt_readsysstatuslo();

		if ((uwb.status_reg & SYS_STATUS_RXFCG_BIT_MASK) != 0U) {
			if (first_rx_time == 0) {
				first_rx_time = k_uptime_get();
			}

			dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK);
			(void)process_anchor_response();
			restart_receiver();

			if (uwb.anchor_count >= MAX_ANCHORS) {
				break;
			}
		}

		if ((uwb.status_reg & SYS_STATUS_ALL_RX_TO) != 0U) {
			dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO);
			break;
		}

		if ((uwb.status_reg & SYS_STATUS_ALL_RX_ERR) != 0U) {
			dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
		}

		if ((first_rx_time == 0) && ((k_uptime_get() - start) > RX_EARLY_EXIT_MS)) {
			break;
		}

		k_sleep(K_MSEC(1));
	}

	uwb.frame_seq_nb++;
	return true;
}

static float fuse_anchor_distances(void)
{
	if (uwb.anchor_count == 0U) {
		return 0.0f;
	}

	float d[MAX_ANCHORS];
	uint8_t count = 0;

	for (uint8_t i = 0; i < uwb.anchor_count; i++) {
		if (uwb.anchors[i].valid) {
			d[count++] = uwb.anchors[i].distance;
		}
	}

	if (count == 0U) {
		return 0.0f;
	}

	for (uint8_t i = 0; i < (count - 1U); i++) {
		for (uint8_t j = i + 1U; j < count; j++) {
			if (d[j] < d[i]) {
				float tmp = d[i];
				d[i] = d[j];
				d[j] = tmp;
			}
		}
	}

	float median = (count % 2U) == 0U ? 0.5f * (d[count / 2U - 1U] + d[count / 2U]) : d[count / 2U];
	float sum = 0.0f;
	float wsum = 0.0f;
	uint8_t accepted = 0;

	for (uint8_t i = 0; i < uwb.anchor_count; i++) {
		if (!uwb.anchors[i].valid) {
			continue;
		}

		float dist = uwb.anchors[i].distance;
		float err = fabsf(dist - median);

		if (err > ANCHOR_REJECT_THRESH) {
			uwb.anchors[i].rejected = true;
			continue;
		}

		float weight = expf(-err * WEIGHT_DECAY);
		sum += dist * weight;
		wsum += weight;
		accepted++;
	}

	return ((accepted > 0U) && (wsum > 0.0f)) ? (sum / wsum) : median;
}

static float compute_anchor_spread(void)
{
	if (uwb.anchor_count < 2U) {
		return 0.0f;
	}

	float min_d = uwb.anchors[0].distance;
	float max_d = uwb.anchors[0].distance;

	for (uint8_t i = 1; i < uwb.anchor_count; i++) {
		if (uwb.anchors[i].distance < min_d) {
			min_d = uwb.anchors[i].distance;
		}

		if (uwb.anchors[i].distance > max_d) {
			max_d = uwb.anchors[i].distance;
		}
	}

	float spread = (max_d - min_d) - CUP_DIAMETER_METERS;
	return spread < 0.0f ? 0.0f : spread;
}

static float compute_median(const float *values, uint8_t count)
{
	float temp[MEDIAN_WINDOW];

	if (count > MEDIAN_WINDOW) {
		count = MEDIAN_WINDOW;
	}

	memcpy(temp, values, count * sizeof(float));

	for (uint8_t i = 0; i < (count - 1U); i++) {
		for (uint8_t j = i + 1U; j < count; j++) {
			if (temp[j] < temp[i]) {
				float t = temp[i];
				temp[i] = temp[j];
				temp[j] = t;
			}
		}
	}

	return temp[count / 2U];
}

static float get_median_distance(void)
{
	return range_state.count == 0U ? filter.smoothed : compute_median(range_state.buffer, range_state.count);
}

static float compute_confidence(float spread, uint8_t anchor_count, bool moving)
{
	float confidence = 1.0f;
	float spread_penalty = 1.0f - (spread * 2.5f);

	if (spread_penalty < 0.15f) {
		spread_penalty = 0.15f;
	}

	confidence *= spread_penalty;

	if (anchor_count == 2U) {
		confidence *= CONF_ANCHOR_2;
	}

	if (anchor_count <= 1U) {
		confidence *= CONF_ANCHOR_1;
	}

	if (moving) {
		confidence *= CONF_MOTION_PENALTY;
	}

	return confidence < CONF_MIN ? CONF_MIN : confidence;
}

static void update_velocity(float distance)
{
	range_state.raw_velocity = distance - range_state.last_raw_distance;
	range_state.last_raw_distance = distance;
	filter.velocity_lp = (VELOCITY_LP_DECAY * filter.velocity_lp) + (VELOCITY_LP_ALPHA * range_state.raw_velocity);
}

static void update_filter(float measurement, float confidence)
{
	float alpha = ALPHA_MIN + ((ALPHA_MAX - ALPHA_MIN) * confidence);

	if (!filter.valid) {
		filter.smoothed = measurement;
		filter.valid = true;
		return;
	}

	filter.smoothed += alpha * (measurement - filter.smoothed);
}

static void update_lock(float distance, bool moving)
{
	float delta = fabsf(distance - lock_distance);

	switch (lock_state) {
	case SEARCHING:
		lock_counter = 0;
		if (!moving && (filter.confidence > LOCK_MIN_CONFIDENCE)) {
			lock_distance = distance;
			lock_counter = 1;
			lock_state = CANDIDATE;
		}
		break;
	case CANDIDATE:
		if ((delta < LOCK_ACQUIRE_TOL_METERS) && (filter.confidence > LOCK_MIN_CONFIDENCE)) {
			lock_counter++;
			if (lock_counter > LOCK_CONFIRM_COUNT) {
				lock_state = LOCKED;
			}
		} else {
			lock_state = SEARCHING;
		}
		break;
	case LOCKED:
		if ((delta > LOCK_BREAK_TOL_METERS) || moving) {
			lock_state = SEARCHING;
		}
		break;
	}
}

static bool run_ranging_cycle(float *distance)
{
	clear_anchor_data();

	if (!send_poll()) {
		return false;
	}

	if (!collect_anchor_responses()) {
		return false;
	}

	*distance = fuse_anchor_distances();
	uwb.distance_spread = compute_anchor_spread();

	if ((uwb.anchor_count == MAX_ANCHORS) && (uwb.distance_spread < DEBUG_STABILITY_MAX_SPREAD)) {
		for (uint8_t i = 0; i < uwb.anchor_count; i++) {
			uint8_t id = uwb.anchors[i].id;

			if (id < MAX_ANCHORS) {
				cal_buffer[id][cal_index[id]++] = uwb.anchors[i].distance;
				if (cal_index[id] >= CAL_SAMPLES) {
					cal_index[id] = 0;
				}
			}
		}
	}

	return uwb.anchor_count > 0U;
}

static void process_measurement(float distance, float spread, bool moving)
{
	update_velocity(distance);

	range_state.buffer[range_state.index++] = distance;
	if (range_state.index >= MEDIAN_WINDOW) {
		range_state.index = 0;
	}
	if (range_state.count < MEDIAN_WINDOW) {
		range_state.count++;
	}

	float confidence = compute_confidence(spread, uwb.anchor_count, moving);
	filter.confidence = confidence * 100.0f;
	update_filter(distance, confidence);
	update_lock(filter.smoothed, moving);
}

static void print_telemetry(uint32_t cycle)
{
	printk("%6u A0=%5.2f A1=%5.2f A2=%5.2f spread=%4.2f dist=%5.2f med=%5.2f conf=%3.0f%% rx=%u/%u state=%d\n",
	       cycle,
	       uwb.anchors[0].valid ? (double)uwb.anchors[0].distance : -1.0,
	       uwb.anchors[1].valid ? (double)uwb.anchors[1].distance : -1.0,
	       uwb.anchors[2].valid ? (double)uwb.anchors[2].distance : -1.0,
	       (double)uwb.distance_spread,
	       (double)filter.smoothed,
	       (double)get_median_distance(),
	       (double)filter.confidence,
	       rx_valid_count,
	       rx_attempt_count * MAX_ANCHORS,
	       lock_state);
}

static bool anchor_wait_poll_and_respond(uint32_t *poll_count)
{
	uint8_t rx_buffer[RX_BUFFER_LEN];
	int64_t start = k_uptime_get();

	dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS_BIT_MASK);
	(void)dwt_rxenable(DWT_START_RX_IMMEDIATE);

	while ((k_uptime_get() - start) < ANCHOR_POLL_TIMEOUT_MS) {
		uint32_t status = dwt_readsysstatuslo();

		if ((status & SYS_STATUS_RXFCG_BIT_MASK) != 0U) {
			uint8_t rng = 0;
			uint16_t frame_len = dwt_getframelength(&rng);

			dwt_writesysstatuslo(SYS_STATUS_RXFCG_BIT_MASK);

			if ((frame_len == sizeof(rx_poll_msg)) && (frame_len <= sizeof(rx_buffer))) {
				dwt_readrxdata(rx_buffer, frame_len, 0);

				uint8_t seq = rx_buffer[ALL_MSG_SN_IDX];
				rx_buffer[ALL_MSG_SN_IDX] = 0;
				bool valid = (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0);
				rx_buffer[ALL_MSG_SN_IDX] = seq;

				if (valid) {
					uint64_t poll_rx_ts = get_rx_timestamp_u64();
					uint32_t resp_tx_time = (uint32_t)((poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);
					uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

					tx_resp_msg[ALL_MSG_SN_IDX] = seq;
					tx_resp_msg[RESP_MSG_ANCHOR_ID_IDX] = PINTRACKER_ANCHOR_ID;
					resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
					resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

					dwt_setdelayedtrxtime(resp_tx_time);
					dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
					dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);

					if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS) {
						int64_t tx_start = k_uptime_get();

						while ((dwt_readsysstatuslo() & SYS_STATUS_TXFRS_BIT_MASK) == 0U) {
							if ((k_uptime_get() - tx_start) > TX_TIMEOUT_MS) {
								break;
							}
						}

						dwt_writesysstatuslo(SYS_STATUS_TXFRS_BIT_MASK);
						(*poll_count)++;
						return true;
					}
				}
			}
		}

		if ((status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0U) {
			dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
			(void)dwt_rxenable(DWT_START_RX_IMMEDIATE);
		}

		k_sleep(K_MSEC(1));
	}

	dwt_forcetrxoff();
	return false;
}

static void run_anchor(void)
{
	uint32_t poll_count = 0;
	uint32_t idle_count = 0;

	printk("Anchor ID: %u\n", PINTRACKER_ANCHOR_ID);
	printk("Anchor waiting for polls\n");

	while (1) {
		if (anchor_wait_poll_and_respond(&poll_count)) {
			printk("anchor %u responded count=%u\n", PINTRACKER_ANCHOR_ID, poll_count);
		} else {
			idle_count++;
			printk("anchor %u waiting idle=%u responses=%u\n", PINTRACKER_ANCHOR_ID, idle_count, poll_count);
		}
	}
}

static void run_tag(void)
{
	uint32_t last_range_ms = 0;
	uint32_t cycle = 0;

	while (1) {
		uint32_t now = k_uptime_get_32();

		if ((now - last_range_ms) >= RANGE_INTERVAL_MS) {
			float raw_distance = 0.0f;

			last_range_ms = now;
			cycle++;

			if (run_ranging_cycle(&raw_distance)) {
				process_measurement(raw_distance, uwb.distance_spread, false);
				print_telemetry(cycle);
			} else {
				printk("%6u searching rx=%u/%u\n", cycle, rx_valid_count, rx_attempt_count * MAX_ANCHORS);
			}
		}

		handle_radio_errors();
		k_sleep(K_MSEC(5));
	}
}

static void init_uwb(const char *role)
{
	printk("DW3000 %s %s\n", role, FW_VERSION);

	spi_init();
	k_msleep(20);

	uint8_t dev_hdr[1] = { 0x00 };
	uint8_t dev_id_raw[4] = { 0 };

	for (uint8_t i = 0; i < 5; i++) {
		(void)readfromspi(1, dev_hdr, 4, dev_id_raw);
		printk("RAW DEV_ID[%u]: %02X %02X %02X %02X\n",
		       i,
		       dev_id_raw[0],
		       dev_id_raw[1],
		       dev_id_raw[2],
		       dev_id_raw[3]);
		k_msleep(5);
	}

	if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) == DWT_ERROR) {
		printk("PROBE FAILED\n");
		while (1) {
			k_sleep(K_SECONDS(1));
		}
	}

	while (!dwt_checkidlerc()) {
		k_sleep(K_MSEC(1));
	}

	if (dwt_initialise(DWT_READ_OTP_ALL) == DWT_ERROR) {
		printk("INIT FAILED\n");
		while (1) {
			k_sleep(K_SECONDS(1));
		}
	}

	dwt_setxtaltrim(DW3000_XTRIM_VALUE);
	dwt_softreset(DWT_RESET_ALL);
	k_msleep(2);

	while (!dwt_checkidlerc()) {
		k_sleep(K_MSEC(1));
	}

	if (dwt_initialise(DWT_READ_OTP_ALL) == DWT_ERROR) {
		printk("INIT FAILED after XTRIM\n");
		while (1) {
			k_sleep(K_SECONDS(1));
		}
	}

	printk("DEVICE ID: 0x%08X\n", dwt_readdevid());

	dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

	if (dwt_configure(&config) != DWT_SUCCESS) {
		printk("CONFIG FAILED\n");
		while (1) {
			k_sleep(K_SECONDS(1));
		}
	}

	dwt_configciadiag(1);
	dwt_configuretxrf(&txconfig);
	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);
	dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
	dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
	dwt_setpreambledetecttimeout(PREAMBLE_TIMEOUT_PAC);
	dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

	printk("UWB ready\n");
}

int main(void)
{
#if defined(PINTRACKER_ROLE_ANCHOR)
	init_uwb("anchor");
	run_anchor();
#else
	init_uwb("tag");
	run_tag();
#endif

	return 0;
}
