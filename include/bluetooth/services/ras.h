/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BT_RAS_H_
#define BT_RAS_H_

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/gatt_dm.h>
#include <zephyr/bluetooth/gatt.h>

#ifdef __cplusplus
extern "C" {
#endif

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

#define BT_RAS_MAX_SUBEVENTS_PER_PROCEDURE 32
#define BT_RAS_MAX_STEPS_PER_PROCEDURE     256

#define BT_RAS_RANGING_HEADER_LEN  (sizeof(struct ras_ranging_header))
#define BT_RAS_SUBEVENT_HEADER_LEN (sizeof(struct ras_subevent_header))
#define BT_RAS_STEP_MODE_LEN       1
#define BT_RAS_MAX_STEP_DATA_LEN   35

/* TODO: This is most likely overestimating the size */
#define BT_RAS_PROCEDURE_MEM (BT_RAS_RANGING_HEADER_LEN \
			     + (BT_RAS_MAX_SUBEVENTS_PER_PROCEDURE * BT_RAS_SUBEVENT_HEADER_LEN) \
			     + (BT_RAS_MAX_STEPS_PER_PROCEDURE * BT_RAS_STEP_MODE_LEN) \
			     + (BT_RAS_MAX_STEPS_PER_PROCEDURE * BT_RAS_MAX_STEP_DATA_LEN))

/** @brief RAS Ranging Data Buffer callback structure. */
struct bt_ras_rd_buffer_cb {
	/** @brief New ranging data has been received from the local controller.
	 *
	 *  This callback notifies the application that the ranging data buffer
	 *  has reassembled a complete ranging procedure from the local controller.
	 *
	 *  @param conn Connection object.
	 *  @param ranging_counter Ranging counter of the stored procedure.
	 */
	void (*new_ranging_data_received)(struct bt_conn *conn, uint16_t ranging_counter);

	/** @brief Ranging data has been overwritten.
	 *
	 *  This callback notifies the application that the ranging data buffer
	 *  has overwritten a stored procedure due to running out of buffers
	 *  to store a newer procedure from the local controller.
	 *
	 *  @param conn Connection object.
	 *  @param ranging_counter Ranging counter of the overwritten procedure.
	 */
	void (*ranging_data_overwritten)(struct bt_conn *conn, uint16_t ranging_counter);

	sys_snode_t node;
};

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

struct ras_rd_buffer {
	struct bt_conn *conn;
	uint16_t ranging_counter;
	uint16_t subevent_cursor;
	uint16_t read_cursor;
	uint8_t ready : 1; /* All ranging data has been written */
	uint8_t busy : 1; /* Buffer is receiving data from HCI */
	uint8_t acked : 1; /* Buffer has been ACKed, do not notify overwritten */
	uint8_t refcount; /* TODO atomic */
	union {
		uint8_t buf[BT_RAS_PROCEDURE_MEM];
		struct {
			struct ras_ranging_header ranging_header;
			uint8_t subevents[];
		} __packed;
	} procedure;
};

/** Subevent result step */
struct ras_rd_cs_subevent_step {
	/** CS step mode. */
	uint8_t mode;
	/** Pointer to role- and mode-specific information. */
	const uint8_t *data;
};

/* Ranging service API */

int bt_ras_rrsp_init(void);
int bt_ras_rrsp_alloc(struct bt_conn *conn);
void bt_ras_rrsp_free(struct bt_conn *conn);

/* Ranging data buffer API */

void bt_ras_rd_buffer_init(void);
void bt_ras_rd_buffer_cb_register(struct bt_ras_rd_buffer_cb *cb);
bool bt_ras_rd_buffer_ready_check(struct bt_conn *conn, uint16_t ranging_counter);
struct ras_rd_buffer *bt_ras_rd_buffer_claim(struct bt_conn *conn, uint16_t ranging_counter);
int bt_ras_rd_buffer_release(struct ras_rd_buffer *buf);
int bt_ras_rd_buffer_bytes_pull(struct ras_rd_buffer *buf, uint8_t *out_buf, uint16_t max_data_len);
void bt_ras_rd_buffer_rewind(struct ras_rd_buffer *buf, uint16_t data_len);

/** @brief Allocate a RREQ context and assign GATT handles. Takes a reference to the connection.
 *
 * @param[in] dm   Discovery Object.
 * @param[in] conn Connection Object.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a negative error code is returned.
 */
int bt_ras_rreq_alloc_and_assign_handles(struct bt_gatt_dm *dm, struct bt_conn *conn);

/** @brief Get ranging data for given ranging counter.
 *
 * @param[in] conn                 Connection Object.
 * @param[in] ranging_data_out     Simple buffer to store received ranging data.
 * @param[in] ranging_counter      Ranging counter to get.
 * @param[in] data_get_complete_cb Callback called when get ranging data completes.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a negative error code is returned.
 */
int bt_ras_rreq_cp_get_ranging_data(struct bt_conn *conn, struct net_buf_simple *ranging_data_out,
				    uint16_t ranging_counter,
				    bt_ras_rreq_ranging_data_get_complete_t data_get_complete_cb);

/** @brief Free RREQ context for connection. Should be called from disconnected callback.
 *
 * @param[in] conn Connection Object.
 */
void bt_ras_rreq_free(struct bt_conn *conn);

/** @brief Subscribe to all required on-demand ranging data subscriptions. Subscribes to ranging
 * data ready, ranging data overwritten, on-demand ranging data and RAS-CP.
 *
 * @param[in] conn              Connection Object.
 * @param[in] rd_ready_cb       Callback called when ranging data ready notification has been
 * received.
 * @param[in] rd_overwritten_cb Callback called when ranging data overwritten notification has been
 * received, unless get ranging data has already been called for this ranging counter where
 * data_get_complete_cb will be called instead.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a negative error code is returned.
 */
int bt_ras_rreq_on_demand_ranging_data_subscribe_all(
	struct bt_conn *conn, bt_ras_rreq_rd_ready_cb_t rd_ready_cb,
	bt_ras_rreq_rd_overwritten_cb_t rd_overwritten_cb);


void bt_ras_rreq_rd_subevent_data_parse(struct net_buf_simple *ranging_data_buf,
			      bool (*func1)(struct ras_subevent_header *subevent_header, void *user_data),
			      bool (*func)(struct ras_rd_cs_subevent_step *step, uint16_t *step_data_length, void *user_data),
			      void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* BT_RAS_H_ */
