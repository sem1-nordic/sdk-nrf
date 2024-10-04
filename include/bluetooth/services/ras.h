/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/gatt_dm.h>
#include <zephyr/bluetooth/gatt.h>

/** @brief UUID of the Ranging Service. **/
#define BT_UUID_RANGING_SERVICE_VAL (0x185B)

/** @brief UUID of the RAS Features Characteristic. **/
#define BT_UUID_RAS_FEATURES_VAL (0x2C14)

/** @brief UUID of the Real-time Ranging Data Characteristic. **/
#define BT_UUID_RAS_REALTIME_RD_VAL (0x2C15)

/** @brief UUID of the On-demand Ranging Data Characteristic. **/
#define BT_UUID_RAS_ONDEMAND_RD_VAL (0x2C16)

/** @brief UUID of the RAS Control Point Characteristic. **/
#define BT_UUID_RAS_CP_VAL (0x2C17)

/** @brief UUID of the Ranging Data Ready Characteristic. **/
#define BT_UUID_RAS_RD_READY_VAL (0x2C18)

/** @brief UUID of the Ranging Data Overwritten Characteristic. **/
#define BT_UUID_RAS_RD_OVERWRITTEN_VAL (0x2C19)

#define BT_UUID_RANGING_SERVICE    BT_UUID_DECLARE_16(BT_UUID_RANGING_SERVICE_VAL)
#define BT_UUID_RAS_FEATURES       BT_UUID_DECLARE_16(BT_UUID_RAS_FEATURES_VAL)
#define BT_UUID_RAS_REALTIME_RD    BT_UUID_DECLARE_16(BT_UUID_RAS_REALTIME_RD_VAL)
#define BT_UUID_RAS_ONDEMAND_RD    BT_UUID_DECLARE_16(BT_UUID_RAS_ONDEMAND_RD_VAL)
#define BT_UUID_RAS_CP             BT_UUID_DECLARE_16(BT_UUID_RAS_CP_VAL)
#define BT_UUID_RAS_RD_READY       BT_UUID_DECLARE_16(BT_UUID_RAS_RD_READY_VAL)
#define BT_UUID_RAS_RD_OVERWRITTEN BT_UUID_DECLARE_16(BT_UUID_RAS_RD_OVERWRITTEN_VAL)

#define BT_RAS_CURRENT_SEGMENT_COUNTER_NOT_SET 0xFF

typedef void (*bt_ras_rreq_rd_ready_cb_t)(struct bt_conn *conn, uint16_t ranging_counter);
typedef void (*bt_ras_rreq_rd_overwritten_cb_t)(struct bt_conn *conn, uint16_t ranging_counter);
typedef void (*bt_ras_rreq_ranging_data_get_complete_t)(int err, uint16_t ranging_counter);

struct ras_ranging_header {
	uint16_t ranging_counter : 12;
	uint8_t  config_id       : 4;
	int8_t   selected_tx_power;
	uint8_t  antenna_paths_mask;
} __packed;

struct ras_subevent_header {
	uint16_t start_acl_conn_event;
	int16_t freq_compensation;
	uint8_t ranging_done_status   : 4;
	uint8_t subevent_done_status  : 4;
	uint8_t ranging_abort_reason  : 4;
	uint8_t subevent_abort_reason : 4;
	int8_t  ref_power_level;
	uint8_t num_steps_reported;
} __packed;

struct ras_subevent {
	struct ras_subevent_header header;
	uint8_t * data;
} __packed;

int bt_ras_rrsp_init(void);
int bt_ras_rrsp_alloc(struct bt_conn *conn);
void bt_ras_rrsp_free(struct bt_conn *conn);

int bt_ras_rreq_init(void);
int bt_ras_rreq_alloc_and_init(struct bt_gatt_dm *dm, struct bt_conn *conn);
int bt_ras_rreq_feature_read(struct bt_conn *conn);
int bt_ras_rreq_cp_get_ranging_data(struct bt_conn *conn, struct net_buf_simple *ranging_data_out,
				    uint16_t ranging_counter,
				    bt_ras_rreq_ranging_data_get_complete_t data_get_complete_cb);
int bt_ras_rreq_cp_subscribe(struct bt_conn *conn);
int bt_ras_rreq_alloc(struct bt_conn *conn);
void bt_ras_rreq_free(struct bt_conn *conn);
int bt_ras_rreq_on_demand_ranging_data_subscribe_all(
	struct bt_conn *conn, bt_ras_rreq_rd_ready_cb_t rd_ready_cb,
	bt_ras_rreq_rd_overwritten_cb_t rd_overwritten_cb);
