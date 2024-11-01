/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/console/console.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/att.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/services/ras.h>
#include <bluetooth/gatt_dm.h>

static K_SEM_DEFINE(sem_remote_capabilities_obtained, 0, 1);
static K_SEM_DEFINE(sem_config_created, 0, 1);
static K_SEM_DEFINE(sem_cs_security_enabled, 0, 1);
static K_SEM_DEFINE(sem_procedure_done, 0, 1);
static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_discovered, 0, 1);
static K_SEM_DEFINE(sem_data_received, 0, 1);
static K_SEM_DEFINE(rd_ready_sem, 0, 1);
static K_SEM_DEFINE(pairing_complete_sem, 0, 1);

#define CS_CONFIG_ID      0
#define NUM_MODE_0_STEPS  1
#define NAME_LEN          30
#define STEP_DATA_BUF_LEN 512 /* Maximum GATT characteristic length */

void estimate_distance(struct net_buf_simple * local_steps, struct net_buf_simple * peer_steps,
		       uint8_t n_ap, enum bt_conn_le_cs_role role);
static struct bt_conn *connection;
static enum bt_conn_le_cs_role role_selection;
static uint8_t n_ap;
static uint8_t latest_num_steps_reported;

NET_BUF_SIMPLE_DEFINE_STATIC(latest_local_steps, 5500);
NET_BUF_SIMPLE_DEFINE_STATIC(latest_peer_steps, 5500);
static uint16_t most_recent_ranging_counter;

static const char sample_str[] = "CS Sample";
static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, "CS Sample", sizeof(sample_str) - 1),
};

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing completed: %s, bonded: %d\n", addr, bonded);
	k_sem_give(&pairing_complete_sem);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing failed conn: %s, reason %d %s\n", addr, reason,
	       bt_security_err_to_str(reason));
}

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {.pairing_complete = pairing_complete,
							       .pairing_failed = pairing_failed};

static void subevent_result_cb(struct bt_conn *conn, struct bt_conn_le_cs_subevent_result *result)
{
	latest_num_steps_reported = result->header.num_steps_reported;
	n_ap = result->header.num_antenna_paths;

	if (result->step_data_buf) {
		if (result->step_data_buf->len <= net_buf_simple_tailroom(&latest_local_steps)) {
			uint16_t len = result->step_data_buf->len;
			uint8_t *step_data = net_buf_simple_pull_mem(result->step_data_buf, len);
			net_buf_simple_add_mem(&latest_local_steps, step_data, len);
		} else {
			printk("Not enough memory to store step data. (%d > %d)\n",
			       result->step_data_buf->len, STEP_DATA_BUF_LEN);
			latest_num_steps_reported = 0;
		}
	}

	if (result->header.procedure_done_status == BT_CONN_LE_CS_PROCEDURE_COMPLETE) {
		k_sem_give(&sem_procedure_done);
	}
}

void ranging_data_get_complete_cb(int err, uint16_t ranging_counter)
{
	if (err) {
		printk("Error %d, when getting ranging data with ranging counter %d\n", err,
		       ranging_counter);
	} else {
		printk("Ranging data get completed for ranging counter %d\n", ranging_counter);
	}

	k_sem_give(&sem_data_received);
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	(void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Connected to %s (err 0x%02X)\n", addr, err);

	__ASSERT(connection == conn, "Unexpected connected callback");

	if (err) {
		bt_conn_unref(conn);
		connection = NULL;
	}

	if (role_selection == BT_CONN_LE_CS_ROLE_REFLECTOR) {
		connection = bt_conn_ref(conn);
	}

	k_sem_give(&sem_connected);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02X)\n", reason);

	bt_conn_unref(conn);
	connection = NULL;
}

static void remote_capabilities_cb(struct bt_conn *conn, struct bt_conn_le_cs_capabilities *params)
{
	ARG_UNUSED(params);
	printk("CS capability exchange completed.\n");
	k_sem_give(&sem_remote_capabilities_obtained);
}

static void config_created_cb(struct bt_conn *conn, struct bt_conn_le_cs_config *config)
{
	printk("CS config creation complete. ID: %d\n", config->id);
	k_sem_give(&sem_config_created);
}

static void security_enabled_cb(struct bt_conn *conn)
{
	printk("CS security enabled.\n");
	k_sem_give(&sem_cs_security_enabled);
}

static void procedure_enabled_cb(struct bt_conn *conn,
				 struct bt_conn_le_cs_procedure_enable_complete *params)
{
	if (params->state == 1) {
		printk("CS procedures enabled.\n");
	} else {
		printk("CS procedures disabled.\n");
	}
}

static bool data_cb(struct bt_data *data, void *user_data)
{
	char *name = user_data;
	uint8_t len;

	switch (data->type) {
	case BT_DATA_NAME_SHORTENED:
	case BT_DATA_NAME_COMPLETE:
		len = MIN(data->data_len, NAME_LEN - 1);
		memcpy(name, data->data, len);
		name[len] = '\0';
		return false;
	default:
		return true;
	}
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	char name[NAME_LEN] = {};
	int err;

	if (connection) {
		return;
	}

	/* We're only interested in connectable events */
	if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}

	bt_data_parse(ad, data_cb, name);

	if (strcmp(name, sample_str)) {
		return;
	}

	if (bt_le_scan_stop()) {
		return;
	}

	printk("Found device with name %s, connecting...\n", name);

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT,
				&connection);
	if (err) {
		printk("Create conn to %s failed (%u)\n", addr_str, err);
	}
}

static void discovery_completed_cb(struct bt_gatt_dm *dm, void *context)
{
	int err;

	printk("The discovery procedure succeeded\n");

	struct bt_conn *conn = bt_gatt_dm_conn_get(dm);

	bt_gatt_dm_data_print(dm);

	err = bt_ras_rreq_alloc_and_assign_handles(dm, conn);
	if (err) {
		printk("RAS RREQ alloc init failed, err %d\n", err);
	}

	err = bt_gatt_dm_data_release(dm);
	if (err) {
		printk("Could not release the discovery data, err %d\n", err);
	}

	k_sem_give(&sem_discovered);
}

static void discovery_service_not_found_cb(struct bt_conn *conn, void *context)
{
	printk("The service could not be found during the discovery, disconnecting\n");
	bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static void discovery_error_found_cb(struct bt_conn *conn, int err, void *context)
{
	printk("The discovery procedure failed, err %d\n", err);
	bt_conn_disconnect(connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static struct bt_gatt_dm_cb discovery_cb = {
	.completed = discovery_completed_cb,
	.service_not_found = discovery_service_not_found_cb,
	.error_found = discovery_error_found_cb,
};

static void ranging_data_ready_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
	printk("Ranging data ready %i\n", ranging_counter);
	most_recent_ranging_counter = ranging_counter;
	k_sem_give(&rd_ready_sem);
}

static void ranging_data_overwritten_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
	printk("Ranging data overwritten %i\n", ranging_counter);
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.le_cs_remote_capabilities_available = remote_capabilities_cb,
	.le_cs_config_created = config_created_cb,
	.le_cs_security_enabled = security_enabled_cb,
	.le_cs_procedure_enabled = procedure_enabled_cb,
	.le_cs_subevent_data_available = subevent_result_cb,
};

int main(void)
{
	int err;

	console_init();

	printk("Starting Channel Sounding Demo\n");

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) {
		printk("Failed to register authorization info callbacks.\n");
		return 0;
	}

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	while (true) {
		printk("Choose device role - type i (initiator) or r (reflector): ");

		char input_char = console_getchar();

		printk("\n");

		if (input_char == 'i') {
			printk("Initiator selected.\n");
			role_selection = BT_CONN_LE_CS_ROLE_INITIATOR;
			break;
		} else if (input_char == 'r') {
			printk("Reflector selected.\n");
			role_selection = BT_CONN_LE_CS_ROLE_REFLECTOR;
			break;
		}

		printk("Invalid role.\n");
	}

	if (role_selection == BT_CONN_LE_CS_ROLE_INITIATOR) {
		err = bt_le_scan_start(BT_LE_SCAN_ACTIVE_CONTINUOUS, device_found);
		if (err) {
			printk("Scanning failed to start (err %d)\n", err);
			return 0;
		}
	} else {
		err = bt_ras_rrsp_init();
		if (err) {
			printk("Error occurred when initializing RAS RRSP service (err %d)\n", err);
			return 0;
		}

		err = bt_le_adv_start(BT_LE_ADV_PARAM(BIT(0) | BIT(1), BT_GAP_ADV_FAST_INT_MIN_1,
						      BT_GAP_ADV_FAST_INT_MAX_1, NULL),
				      ad, ARRAY_SIZE(ad), NULL, 0);
		if (err) {
			printk("Advertising failed to start (err %d)\n", err);
			return 0;
		}
	}

	k_sem_take(&sem_connected, K_FOREVER);

	const struct bt_le_cs_set_default_settings_param default_settings = {
		.enable_initiator_role = true,
		.enable_reflector_role = true,
		.cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
		.max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
	};

	err = bt_le_cs_set_default_settings(connection, &default_settings);
	if (err) {
		printk("Failed to configure default CS settings (err %d)\n", err);
	}

	if (role_selection == BT_CONN_LE_CS_ROLE_INITIATOR) {
		err = bt_gatt_dm_start(connection, BT_UUID_RANGING_SERVICE, &discovery_cb, NULL);
		if (err) {
			printk("Discovery failed (err %d)\n", err);
			return 0;
		}

		err = k_sem_take(&sem_discovered, K_SECONDS(10));
		if (err) {
			printk("Timed out during GATT discovery\n");
			return 0;
		}

		err = bt_ras_rreq_on_demand_ranging_data_subscribe_all(connection, ranging_data_ready_cb,
							       ranging_data_overwritten_cb);
		if (err) {
			printk("RAS RREQ On-demand ranging data subscribe all failed, err %d\n", err);
			return 0;
		}

		printk("Subscribed\n");

		k_sem_take(&pairing_complete_sem, K_FOREVER);

		err = bt_le_cs_read_remote_supported_capabilities(connection);
		if (err) {
			printk("Failed to exchange CS capabilities (err %d)\n", err);
			return 0;
		}

		printk("Waiting for capabilities\n");

		k_sem_take(&sem_remote_capabilities_obtained, K_FOREVER);

		printk("Remote capabilities obtained\n");

		struct bt_le_cs_create_config_params config_params = {
			.id = CS_CONFIG_ID,
			.main_mode_type = BT_CONN_LE_CS_MAIN_MODE_2,
			.sub_mode_type = BT_CONN_LE_CS_SUB_MODE_1,
			.min_main_mode_steps = 2,
			.max_main_mode_steps = 10,
			.main_mode_repetition = 0,
			.mode_0_steps = NUM_MODE_0_STEPS,
			.role = role_selection,
			.rtt_type = BT_CONN_LE_CS_RTT_TYPE_AA_ONLY,
			.cs_sync_phy = BT_CONN_LE_CS_SYNC_1M_PHY,
			.channel_map_repetition = 1,
			.channel_selection_type = BT_CONN_LE_CS_CHSEL_TYPE_3B,
			.ch3c_shape = BT_CONN_LE_CS_CH3C_SHAPE_HAT,
			.ch3c_jump = 2,
		};

		bt_le_cs_set_valid_chmap_bits(config_params.channel_map);

		err = bt_le_cs_create_config(connection, &config_params,
					     BT_LE_CS_CREATE_CONFIG_CONTEXT_LOCAL_AND_REMOTE);
		if (err) {
			printk("Failed to create CS config (err %d)\n", err);
			return 0;
		}

		k_sem_take(&sem_config_created, K_FOREVER);

		err = bt_le_cs_security_enable(connection);
		if (err) {
			printk("Failed to start CS Security (err %d)\n", err);
			return 0;
		}

		k_sem_take(&sem_cs_security_enabled, K_FOREVER);

		const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
			.config_id = CS_CONFIG_ID,
			.max_procedure_len = 12,
			.min_procedure_interval = 100,
			.max_procedure_interval = 100,
			.max_procedure_count = 0,
			.min_subevent_len = 6750,
			.max_subevent_len = 6750,
			.tone_antenna_config_selection =
				BT_LE_CS_TONE_ANTENNA_CONFIGURATION_INDEX_ONE,
			.phy = BT_LE_CS_PROCEDURE_PHY_1M,
			.tx_power_delta = 0x80,
			.preferred_peer_antenna = BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
			.snr_control_initiator = BT_LE_CS_INITIATOR_SNR_CONTROL_NOT_USED,
			.snr_control_reflector = BT_LE_CS_REFLECTOR_SNR_CONTROL_NOT_USED,
		};

		printk("Setting CS procedure params\n");

		err = bt_le_cs_set_procedure_parameters(connection, &procedure_params);
		if (err) {
			printk("Failed to set procedure parameters (err %d)\n", err);
			return 0;
		}

		struct bt_le_cs_procedure_enable_param params = {
			.config_id = CS_CONFIG_ID,
			.enable = 1,
		};

		printk("Starting CS procedure\n");

		err = bt_le_cs_procedure_enable(connection, &params);
		if (err) {
			printk("Failed to enable CS procedures (err %d)\n", err);
			return 0;
		}
	}

	while (true) {
		printk("Waiting for procedure done\n");

		k_sem_take(&sem_procedure_done, K_FOREVER);

		printk("Procedure done\n");
		if (role_selection == BT_CONN_LE_CS_ROLE_INITIATOR) {
			printk("Waiting for RD ready\n");
			k_sem_take(&rd_ready_sem, K_FOREVER);

			printk("Requesting RD\n");
			err = bt_ras_rreq_cp_get_ranging_data(connection, &latest_peer_steps,
						      most_recent_ranging_counter,
						      ranging_data_get_complete_cb);
			if (err) {
				printk("Get ranging data, err %d\n", err);
			}

			printk("Waiting for RD\n");
			k_sem_take(&sem_data_received, K_FOREVER);
			printk("RD received\n");

			estimate_distance(
				&latest_local_steps, &latest_peer_steps,
				n_ap, role_selection);

			net_buf_simple_reset(&latest_local_steps);
			net_buf_simple_reset(&latest_peer_steps);
		}
	}

	return 0;
}
