#include "app_iic.h"
#include "tuner.h"

#include "cy_pdl.h"
#include "cy_scb_i2c.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_peripherals.h"
#include "force_vector_cal.h"
#include "sensor_algorithm.h"

#include <stdbool.h>
#include <string.h>

#if (TUNER_INTERFACE_TYPE == USER_I2C)

#define APP_IIC_PROTOCOL_VERSION (0x03u)

#define APP_IIC_COMMAND_HEADER0 (0xAAu)
#define APP_IIC_COMMAND_HEADER1 (0x55u)
#define APP_IIC_REPLY_HEADER0 (0x55u)
#define APP_IIC_REPLY_HEADER1 (0xAAu)

#define APP_IIC_COMMAND_ADJUST_CHANNELS (0x63u)
#define APP_IIC_COMMAND_SET_ADDRESS (0x70u)
#define APP_IIC_COMMAND_READ_ADDRESS (0x71u)
#define APP_IIC_COMMAND_RESET_ALL_CHANNELS (0x79u)
#define APP_IIC_COMMAND_RESET_SINGLE_CHANNEL (0x7Cu)
#define APP_IIC_COMMAND_SET_SEND_DATA_TYPE (0x7Fu)
#define APP_IIC_COMMAND_READ_FIRMWARE_VERSION (0xA0u)
#define APP_IIC_COMMAND_SOFTWARE_RESET (0xA1u)
#define APP_IIC_COMMAND_READ_DEVICE_TYPE (0xA2u)
#define APP_IIC_COMMAND_DISCOVERY (0xA6u)
#define APP_IIC_COMMAND_UP_DATA (0x85u)

#define APP_IIC_DEVICE_TYPE_REPLY_COMMAND (0xA4u)
#define APP_IIC_DEVICE_TYPE_REPLY_PREFIX (0x10u)

#define APP_IIC_CHANNEL_COUNT (SENSOR_ALGO_CHANNEL_COUNT)
#define APP_IIC_FORCE_SLOT_COUNT (2u)
#define APP_IIC_SPROX_COUNT (2u)
#define APP_IIC_MPROX_COUNT (1u)
#define APP_IIC_FORCE_BYTES_PER_SLOT (6u)
#define APP_IIC_CHANNEL_BYTES (3u)
#define APP_IIC_CHANNEL_VALUE_MAX (0x7FFFFFu)
#define APP_IIC_STATE_BYTES (1u)
#define APP_IIC_CHECKSUM_BYTES (2u)
#define APP_IIC_INTR_PRIORITY (2u)
#define APP_IIC_PHASE_DUTY_CYCLE (8u)
#define APP_IIC_HW (SCB1)
#define APP_IIC_IRQ (scb_1_interrupt_IRQn)

#define APP_IIC_IAP_ENTRY_SIZE (3u)
#define APP_IIC_IAP_ENTRY_SYNC (0xA5u)
#define APP_IIC_IAP_ENTRY_COMMAND (0x04u)
#define APP_IIC_IAP_ENTRY_CRC (0x45u)

#define APP_IIC_SINGLE_CHANNEL_UNSUPPORTED_RESULT (1u)

#define APP_IIC_FW_VERSION_COPY_LEN (20u)
#define APP_IIC_FW_VERSION_TEXT "May 22 2025 09:30:34"
#define APP_IIC_DEVICE_TYPE_TEXT "TS-F-L-000000185+"

typedef enum {
	APP_IIC_SEND_RAW = 0u,
	APP_IIC_SEND_SIGNAL = 1u,
	APP_IIC_SEND_BASELINE = 2u,
	APP_IIC_SEND_DELTA = 3u,
	APP_IIC_SEND_NOISE = 4u,
} app_iic_send_type_t;

static cy_stc_scb_i2c_context_t app_iic_context;

static const cy_stc_scb_i2c_config_t app_iic_config = {
	.i2cMode = CY_SCB_I2C_SLAVE,
	.useRxFifo = true,
	.useTxFifo = true,
	.slaveAddress = APP_IIC_DEFAULT_ADDRESS,
	.slaveAddressMask = 0u,
	.acceptAddrInFifo = false,
	.ackGeneralAddr = false,
	.hsEnable = false,
	.enableWakeFromSleep = false,
	.enableDigitalFilter = false,
	.lowPhaseDutyCycle = APP_IIC_PHASE_DUTY_CYCLE,
	.highPhaseDutyCycle = APP_IIC_PHASE_DUTY_CYCLE,
};

extern volatile bool is_fw_upgrade_event_detected;

static uint8_t app_iic_rx_command[APP_IIC_COMMAND_FRAME_SIZE];
static uint8_t app_iic_reply_frame[APP_IIC_SPECIAL_A2_FRAME_SIZE];
static uint8_t app_iic_zero_frame[APP_IIC_DATA_FRAME_SIZE];
static uint8_t app_iic_data_frame_a[APP_IIC_DATA_FRAME_SIZE];
static uint8_t app_iic_data_frame_b[APP_IIC_DATA_FRAME_SIZE];

static uint8_t *app_iic_live_frame = app_iic_data_frame_a;
static uint8_t *app_iic_pending_frame = NULL;
static uint8_t *app_iic_active_tx_buffer = app_iic_data_frame_a;
static uint32_t app_iic_active_tx_length = APP_IIC_DATA_FRAME_SIZE;
static uint32_t app_iic_reply_length = APP_IIC_ACK_FRAME_SIZE;

static uint8_t app_iic_frame_number = 0u;
static uint8_t app_iic_send_data_type = APP_IIC_SEND_SIGNAL;
static uint8_t app_iic_current_address = APP_IIC_DEFAULT_ADDRESS;
static uint8_t app_iic_pending_address = APP_IIC_DEFAULT_ADDRESS;

static bool app_iic_reply_pending = false;
static bool app_iic_zero_frame_armed = false;
static bool app_iic_recalibration_after_zero = false;
static bool app_iic_pending_recalibration = false;
static bool app_iic_pending_software_reset = false;
static bool app_iic_pending_address_change = false;

static void app_iic_isr(void);
static void app_iic_configure_hardware(void);
static void app_iic_event_callback(uint32_t event);
static uint8_t app_iic_next_frame_number(void);
static bool app_iic_is_reply_active(void);
static bool app_iic_is_zero_frame_active(void);
static uint16_t app_iic_checksum(const uint8_t *data, uint32_t length);
static bool app_iic_command_is_valid(const uint8_t *frame);
static void app_iic_write_checksum(uint8_t *frame, uint32_t length_without_checksum);
static void app_iic_write_u16(uint8_t *frame, uint32_t *offset, uint16_t value);
static void app_iic_write_u24(uint8_t *frame, uint32_t *offset, uint32_t value);
static void app_iic_set_read_buffer(uint8_t *buffer, uint32_t length);
static void app_iic_rearm_write_buffer(void);
static void app_iic_build_standard_ack(uint8_t version, uint8_t command, uint8_t result);
static void app_iic_build_discovery_reply(void);
static void app_iic_build_firmware_reply(void);
static void app_iic_build_device_type_reply(void);
static void app_iic_build_all_zero_data_frame(uint8_t *frame, uint8_t sequence);
static void app_iic_build_live_data_frame(uint8_t *frame);
static uint32_t app_iic_get_channel_value(const sensor_algorithm_snapshot_t *snapshot, uint32_t channel_index);
static uint16_t app_iic_scale_force_x100(float value);
static uint16_t app_iic_theta_rad_to_deg(float theta_rad);
static void app_iic_publish_latest_data_frame(void);
static void app_iic_activate_zero_frame(void);
static void app_iic_restore_live_frame(void);
static void app_iic_handle_read_request(void);
static void app_iic_handle_read_complete(void);
static void app_iic_handle_write_complete(void);
static void app_iic_process_command(uint32_t received_length);

void AppIic_Init(void) {
	cy_en_scb_i2c_status_t status;
	const cy_stc_sysint_t app_iic_interrupt_config = {
		.intrSrc = APP_IIC_IRQ,
		.intrPriority = APP_IIC_INTR_PRIORITY,
	};

	memset(&app_iic_context, 0, sizeof(app_iic_context));
	memset(app_iic_rx_command, 0, sizeof(app_iic_rx_command));
	memset(app_iic_reply_frame, 0, sizeof(app_iic_reply_frame));
	memset(app_iic_zero_frame, 0, sizeof(app_iic_zero_frame));
	memset(app_iic_data_frame_a, 0, sizeof(app_iic_data_frame_a));
	memset(app_iic_data_frame_b, 0, sizeof(app_iic_data_frame_b));

	app_iic_live_frame = app_iic_data_frame_a;
	app_iic_pending_frame = NULL;
	app_iic_active_tx_buffer = app_iic_data_frame_a;
	app_iic_active_tx_length = APP_IIC_DATA_FRAME_SIZE;
	app_iic_reply_length = APP_IIC_ACK_FRAME_SIZE;
	app_iic_frame_number = 0u;
	app_iic_send_data_type = APP_IIC_SEND_SIGNAL;
	app_iic_current_address = APP_IIC_DEFAULT_ADDRESS;
	app_iic_pending_address = APP_IIC_DEFAULT_ADDRESS;
	app_iic_reply_pending = false;
	app_iic_zero_frame_armed = false;
	app_iic_recalibration_after_zero = false;
	app_iic_pending_recalibration = false;
	app_iic_pending_software_reset = false;
	app_iic_pending_address_change = false;

	app_iic_build_all_zero_data_frame(app_iic_live_frame, 0u);
	app_iic_build_all_zero_data_frame(app_iic_zero_frame, 0u);
	app_iic_configure_hardware();
	status = Cy_SCB_I2C_Init(APP_IIC_HW, &app_iic_config, &app_iic_context);
	if (status != CY_SCB_I2C_SUCCESS) {
		CY_ASSERT(0u);
	}

	Cy_SCB_I2C_SlaveSetAddress(APP_IIC_HW, APP_IIC_DEFAULT_ADDRESS);
	Cy_SCB_I2C_SlaveSetAddressMask(APP_IIC_HW, 0xFEu);
	Cy_SCB_I2C_SlaveConfigReadBuf(APP_IIC_HW, app_iic_live_frame, APP_IIC_DATA_FRAME_SIZE, &app_iic_context);
	Cy_SCB_I2C_SlaveConfigWriteBuf(APP_IIC_HW, app_iic_rx_command, APP_IIC_COMMAND_FRAME_SIZE, &app_iic_context);
	Cy_SCB_I2C_RegisterEventCallback(APP_IIC_HW, app_iic_event_callback, &app_iic_context);

	Cy_SysInt_Init(&app_iic_interrupt_config, app_iic_isr);
	NVIC_ClearPendingIRQ(app_iic_interrupt_config.intrSrc);
	NVIC_EnableIRQ(app_iic_interrupt_config.intrSrc);
	Cy_SCB_I2C_Enable(APP_IIC_HW, &app_iic_context);
}

void AppIic_Poll(void) {}

void AppIic_OnScanComplete(void) { app_iic_publish_latest_data_frame(); }

static void app_iic_isr(void) { Cy_SCB_I2C_SlaveInterrupt(APP_IIC_HW, &app_iic_context); }

static void app_iic_configure_hardware(void) {
	Cy_GPIO_Write(GPIO_PRT5, 1u, 1u);
	Cy_GPIO_Write(GPIO_PRT5, 2u, 1u);
	Cy_GPIO_SetDrivemode(GPIO_PRT5, 1u, CY_GPIO_DM_OD_DRIVESLOW);
	Cy_GPIO_SetDrivemode(GPIO_PRT5, 2u, CY_GPIO_DM_OD_DRIVESLOW);
	Cy_GPIO_SetHSIOM(GPIO_PRT5, 1u, P5_1_SCB1_I2C_SCL);
	Cy_GPIO_SetHSIOM(GPIO_PRT5, 2u, P5_2_SCB1_I2C_SDA);

	Cy_SysClk_PeriphDisableDivider(CY_SYSCLK_DIV_16_BIT, 1u);
	Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 1u, 3u);
	Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, 1u);
	Cy_SysClk_PeriphAssignDivider(PCLK_SCB1_CLOCK, CY_SYSCLK_DIV_16_BIT, 1u);
}

static void app_iic_event_callback(uint32_t event) {
	if ((event & CY_SCB_I2C_SLAVE_READ_EVENT) != 0u) {
		app_iic_handle_read_request();
	}

	if ((event & CY_SCB_I2C_SLAVE_WR_CMPLT_EVENT) != 0u) {
		app_iic_handle_write_complete();
	}

	if ((event & CY_SCB_I2C_SLAVE_RD_CMPLT_EVENT) != 0u) {
		app_iic_handle_read_complete();
	}
}

static uint8_t app_iic_next_frame_number(void) {
	app_iic_frame_number++;
	return app_iic_frame_number;
}

static bool app_iic_is_reply_active(void) { return (app_iic_active_tx_buffer == app_iic_reply_frame); }

static bool app_iic_is_zero_frame_active(void) { return (app_iic_active_tx_buffer == app_iic_zero_frame); }

static uint16_t app_iic_checksum(const uint8_t *data, uint32_t length) {
	uint32_t index;
	uint16_t sum = 0u;

	for (index = 0u; index < length; index++) {
		sum = (uint16_t)(sum + data[index]);
	}

	return sum;
}

static bool app_iic_command_is_valid(const uint8_t *frame) {
	uint16_t expected_checksum;
	uint16_t actual_checksum;

	if ((frame[0] != APP_IIC_COMMAND_HEADER0) || (frame[1] != APP_IIC_COMMAND_HEADER1)) {
		return false;
	}

	expected_checksum = app_iic_checksum(frame, APP_IIC_COMMAND_FRAME_SIZE - APP_IIC_CHECKSUM_BYTES);
	actual_checksum = (uint16_t)frame[APP_IIC_COMMAND_FRAME_SIZE - 2u]
		| (uint16_t)((uint16_t)frame[APP_IIC_COMMAND_FRAME_SIZE - 1u] << 8u);
	return (expected_checksum == actual_checksum);
}

static void app_iic_write_checksum(uint8_t *frame, uint32_t length_without_checksum) {
	uint16_t sum = app_iic_checksum(frame, length_without_checksum);

	frame[length_without_checksum] = (uint8_t)(sum & 0xFFu);
	frame[length_without_checksum + 1u] = (uint8_t)(sum >> 8u);
}

static void app_iic_write_u16(uint8_t *frame, uint32_t *offset, uint16_t value) {
	frame[*offset] = (uint8_t)(value & 0xFFu);
	(*offset)++;
	frame[*offset] = (uint8_t)(value >> 8u);
	(*offset)++;
}

static void app_iic_write_u24(uint8_t *frame, uint32_t *offset, uint32_t value) {
	frame[*offset] = (uint8_t)(value & 0xFFu);
	(*offset)++;
	frame[*offset] = (uint8_t)((value >> 8u) & 0xFFu);
	(*offset)++;
	frame[*offset] = (uint8_t)((value >> 16u) & 0xFFu);
	(*offset)++;
}

static void app_iic_set_read_buffer(uint8_t *buffer, uint32_t length) {
	uint32_t interrupt_state = Cy_SysLib_EnterCriticalSection();

	app_iic_active_tx_buffer = buffer;
	app_iic_active_tx_length = length;
	Cy_SCB_I2C_SlaveConfigReadBuf(APP_IIC_HW, buffer, length, &app_iic_context);

	Cy_SysLib_ExitCriticalSection(interrupt_state);
}

static void app_iic_rearm_write_buffer(void) {
	uint32_t interrupt_state = Cy_SysLib_EnterCriticalSection();

	Cy_SCB_I2C_SlaveConfigWriteBuf(APP_IIC_HW, app_iic_rx_command, APP_IIC_COMMAND_FRAME_SIZE, &app_iic_context);

	Cy_SysLib_ExitCriticalSection(interrupt_state);
}

static void app_iic_build_standard_ack(uint8_t version, uint8_t command, uint8_t result) {
	memset(app_iic_reply_frame, 0, APP_IIC_ACK_FRAME_SIZE);
	app_iic_reply_frame[0] = APP_IIC_REPLY_HEADER0;
	app_iic_reply_frame[1] = APP_IIC_REPLY_HEADER1;
	app_iic_reply_frame[2] = version;
	app_iic_reply_frame[3] = (uint8_t)(command | 0x80u);
	app_iic_reply_frame[7] = result;
	app_iic_write_checksum(app_iic_reply_frame, APP_IIC_ACK_FRAME_SIZE - APP_IIC_CHECKSUM_BYTES);
}

static void app_iic_build_discovery_reply(void) {
	static const uint8_t discovery_reply[APP_IIC_ACK_FRAME_SIZE] = {
		0x55u, 0xAAu, 0x03u, 0xA6u,
		0x01u, 0x04u, 0x12u, 0x34u,
		0x00u, 0xF3u, 0x01u,
	};

	memcpy(app_iic_reply_frame, discovery_reply, sizeof(discovery_reply));
}

static void app_iic_build_firmware_reply(void) {
	static const char firmware_text[] = APP_IIC_FW_VERSION_TEXT;
	uint32_t index;

	memset(app_iic_reply_frame, 0, APP_IIC_SPECIAL_A0_FRAME_SIZE);
	app_iic_reply_frame[0] = APP_IIC_REPLY_HEADER0;
	app_iic_reply_frame[1] = APP_IIC_REPLY_HEADER1;
	app_iic_reply_frame[2] = APP_IIC_PROTOCOL_VERSION;
	app_iic_reply_frame[3] = APP_IIC_COMMAND_READ_FIRMWARE_VERSION;

	for (index = 0u; index < APP_IIC_FW_VERSION_COPY_LEN; index++) {
		if (firmware_text[index] == '\0') {
			break;
		}
		app_iic_reply_frame[4u + index] = (uint8_t)firmware_text[index];
	}

	app_iic_reply_frame[APP_IIC_SPECIAL_A0_FRAME_SIZE - 2u] = 0xDDu;
	app_iic_reply_frame[APP_IIC_SPECIAL_A0_FRAME_SIZE - 1u] = 0x05u;
}

static void app_iic_build_device_type_reply(void) {
	static const char device_type_text[] = APP_IIC_DEVICE_TYPE_TEXT;
	uint32_t index;

	memset(app_iic_reply_frame, 0, APP_IIC_SPECIAL_A2_FRAME_SIZE);
	app_iic_reply_frame[0] = APP_IIC_REPLY_HEADER0;
	app_iic_reply_frame[1] = APP_IIC_REPLY_HEADER1;
	app_iic_reply_frame[2] = APP_IIC_PROTOCOL_VERSION;
	app_iic_reply_frame[3] = APP_IIC_DEVICE_TYPE_REPLY_COMMAND;
	app_iic_reply_frame[4] = APP_IIC_DEVICE_TYPE_REPLY_PREFIX;

	for (index = 0u; index < (APP_IIC_SPECIAL_A2_FRAME_SIZE - 6u); index++) {
		if (device_type_text[index] == '\0') {
			break;
		}
		app_iic_reply_frame[5u + index] = (uint8_t)device_type_text[index];
	}

	app_iic_reply_frame[5u + index] = 0x05u;
}

static void app_iic_build_all_zero_data_frame(uint8_t *frame, uint8_t sequence) {
	memset(frame, 0, APP_IIC_DATA_FRAME_SIZE);
	frame[0] = APP_IIC_REPLY_HEADER0;
	frame[1] = APP_IIC_REPLY_HEADER1;
	frame[2] = APP_IIC_PROTOCOL_VERSION;
	frame[3] = APP_IIC_COMMAND_UP_DATA;
	frame[4] = sequence;
	frame[5] = APP_IIC_CHANNEL_COUNT;
	app_iic_write_checksum(frame, APP_IIC_DATA_FRAME_SIZE - APP_IIC_CHECKSUM_BYTES);
}

static void app_iic_build_live_data_frame(uint8_t *frame) {
	sensor_algorithm_snapshot_t snapshot;
	uint32_t channel_index;
	uint32_t offset = 0u;
	uint16_t direction[APP_IIC_FORCE_SLOT_COUNT] = {0u, 0u};
	float normal_force[APP_IIC_FORCE_SLOT_COUNT];
	float tangential_force[APP_IIC_FORCE_SLOT_COUNT];
	float calibrated_direction[APP_IIC_FORCE_SLOT_COUNT];

	SensorAlgorithm_GetLatestSnapshot(&snapshot);
	SensorAlgorithm_GetCalibratedForcesForRegion(
		0u, &normal_force[0u], &tangential_force[0u],
		&calibrated_direction[0u]);
	SensorAlgorithm_GetCalibratedForcesForRegion(
		1u, &normal_force[1u], &tangential_force[1u],
		&calibrated_direction[1u]);

	frame[offset++] = APP_IIC_REPLY_HEADER0;
	frame[offset++] = APP_IIC_REPLY_HEADER1;
	frame[offset++] = APP_IIC_PROTOCOL_VERSION;
	frame[offset++] = APP_IIC_COMMAND_UP_DATA;
	frame[offset++] = app_iic_next_frame_number();
	frame[offset++] = APP_IIC_CHANNEL_COUNT;

	for (channel_index = 0u; channel_index < APP_IIC_CHANNEL_COUNT; channel_index++) {
		app_iic_write_u24(frame, &offset, app_iic_get_channel_value(&snapshot, channel_index));
	}

	if (tangential_force[0u] > 0.0f) {
		direction[0u] = app_iic_theta_rad_to_deg(calibrated_direction[0u]);
	}
	if (tangential_force[1u] > 0.0f) {
		direction[1u] = app_iic_theta_rad_to_deg(calibrated_direction[1u]);
	}

	for (channel_index = 0u; channel_index < APP_IIC_FORCE_SLOT_COUNT; channel_index++) {
		app_iic_write_u16(frame, &offset, app_iic_scale_force_x100(normal_force[channel_index]));
		app_iic_write_u16(frame, &offset, app_iic_scale_force_x100(tangential_force[channel_index]));
		app_iic_write_u16(frame, &offset, direction[channel_index]);
	}

	for (channel_index = 0u; channel_index < APP_IIC_SPROX_COUNT; channel_index++) {
		app_iic_write_u24(frame, &offset, 0u);
	}

	for (channel_index = 0u; channel_index < APP_IIC_MPROX_COUNT; channel_index++) {
		app_iic_write_u24(frame, &offset, 0u);
	}

	frame[offset++] = 0u;
	app_iic_write_checksum(frame, APP_IIC_DATA_FRAME_SIZE - APP_IIC_CHECKSUM_BYTES);
}

static uint32_t app_iic_get_channel_value(const sensor_algorithm_snapshot_t *snapshot, uint32_t channel_index) {
	uint32_t value;
	uint16_t raw = snapshot->channels[channel_index].raw;
	uint16_t baseline = snapshot->channels[channel_index].baseline;
	uint16_t signal = snapshot->channels[channel_index].signal;
	uint16_t noise = snapshot->channels[channel_index].noise;
	/* 恢复状态虽然不是 IDLE，但不再向主机报告接触有效。 */
	uint32_t active_bit = SensorAlgorithm_IsContactActiveState(snapshot->states[channel_index]) ? 1u : 0u;

	switch ((app_iic_send_type_t)app_iic_send_data_type) {
	case APP_IIC_SEND_RAW:
		value = raw;
		break;
	case APP_IIC_SEND_SIGNAL:
		value = (uint32_t)signal * 200u;
		if (value > APP_IIC_CHANNEL_VALUE_MAX) {
			value = APP_IIC_CHANNEL_VALUE_MAX;
		}
		break;
	case APP_IIC_SEND_BASELINE:
		value = baseline;
		break;
	case APP_IIC_SEND_DELTA:
		value = (raw > baseline) ? (uint32_t)(raw - baseline) : 0u;
		break;
	case APP_IIC_SEND_NOISE:
	default:
		value = noise;
		break;
	}

	return (value << 1u) | active_bit;
}

static uint16_t app_iic_scale_force_x100(float value) {
	float scaled;

	if (!(value > 0.0f)) {
		return 0u;
	}

	scaled = (value * 100.0f) + 0.5f;
	if (scaled >= 65535.0f) {
		return 65535u;
	}

	return (uint16_t)scaled;
}

static uint16_t app_iic_theta_rad_to_deg(float theta_rad) {
	float degrees = theta_rad * (180.0f / 3.14159265f);

	while (degrees < 0.0f) {
		degrees += 360.0f;
	}
	while (degrees >= 360.0f) {
		degrees -= 360.0f;
	}

	return (uint16_t)(degrees + 0.5f);
}

static void app_iic_publish_latest_data_frame(void) {
	uint32_t interrupt_state;
	uint32_t status;
	uint8_t *frame = (app_iic_live_frame == app_iic_data_frame_a) ? app_iic_data_frame_b : app_iic_data_frame_a;

	app_iic_build_live_data_frame(frame);

	interrupt_state = Cy_SysLib_EnterCriticalSection();
	status = Cy_SCB_I2C_SlaveGetStatus(APP_IIC_HW, &app_iic_context);

	if (!app_iic_reply_pending && !app_iic_is_zero_frame_active() && !app_iic_zero_frame_armed &&
		((status & CY_SCB_I2C_SLAVE_RD_BUSY) == 0u)) {
		app_iic_live_frame = frame;
		app_iic_pending_frame = NULL;
		app_iic_active_tx_buffer = frame;
		app_iic_active_tx_length = APP_IIC_DATA_FRAME_SIZE;
		Cy_SCB_I2C_SlaveConfigReadBuf(APP_IIC_HW, frame, APP_IIC_DATA_FRAME_SIZE, &app_iic_context);
	} else {
		app_iic_pending_frame = frame;
	}

	Cy_SysLib_ExitCriticalSection(interrupt_state);
}

static void app_iic_activate_zero_frame(void) {
	app_iic_zero_frame_armed = true;
	app_iic_set_read_buffer(app_iic_zero_frame, APP_IIC_DATA_FRAME_SIZE);
}

static void app_iic_restore_live_frame(void) {
	if (app_iic_pending_frame != NULL) {
		app_iic_live_frame = app_iic_pending_frame;
		app_iic_pending_frame = NULL;
	}

	app_iic_set_read_buffer(app_iic_live_frame, APP_IIC_DATA_FRAME_SIZE);
}

static void app_iic_handle_read_request(void) {
	if (app_iic_reply_pending) {
		app_iic_set_read_buffer(app_iic_reply_frame, app_iic_reply_length);
		return;
	}

	if (app_iic_is_zero_frame_active() || app_iic_zero_frame_armed) {
		app_iic_set_read_buffer(app_iic_zero_frame, APP_IIC_DATA_FRAME_SIZE);
		return;
	}

	app_iic_restore_live_frame();
}

static void app_iic_handle_read_complete(void) {
	(void)Cy_SCB_I2C_SlaveClearReadStatus(APP_IIC_HW, &app_iic_context);

	if (app_iic_is_reply_active()) {
		if (app_iic_pending_address_change) {
			app_iic_current_address = app_iic_pending_address;
			Cy_SCB_I2C_SlaveSetAddress(APP_IIC_HW, app_iic_current_address);
			app_iic_pending_address_change = false;
		}

		if (app_iic_pending_software_reset) {
			NVIC_SystemReset();
		}

		if (app_iic_pending_recalibration) {
			SensorAlgorithm_RequestRecalibration();
			app_iic_pending_recalibration = false;
		}

		app_iic_reply_pending = false;

		if (app_iic_zero_frame_armed) {
			app_iic_activate_zero_frame();
			return;
		}

		app_iic_restore_live_frame();
		return;
	}

	if (app_iic_is_zero_frame_active()) {
		app_iic_zero_frame_armed = false;

		if (app_iic_recalibration_after_zero) {
			SensorAlgorithm_RequestRecalibration();
			app_iic_recalibration_after_zero = false;
		}

		app_iic_restore_live_frame();
		return;
	}

	app_iic_restore_live_frame();
}

static void app_iic_handle_write_complete(void) {
	uint32_t received_length = Cy_SCB_I2C_SlaveGetWriteTransferCount(APP_IIC_HW, &app_iic_context);

	(void)Cy_SCB_I2C_SlaveClearWriteStatus(APP_IIC_HW, &app_iic_context);
	app_iic_rearm_write_buffer();
	app_iic_process_command(received_length);
}

static void app_iic_process_command(uint32_t received_length) {
	uint8_t version;
	uint8_t command;
	uint8_t reply_length = APP_IIC_ACK_FRAME_SIZE;

	if ((received_length == APP_IIC_IAP_ENTRY_SIZE)
		&& (app_iic_rx_command[0] == APP_IIC_IAP_ENTRY_SYNC)
		&& (app_iic_rx_command[1] == APP_IIC_IAP_ENTRY_COMMAND)
		&& (app_iic_rx_command[2] == APP_IIC_IAP_ENTRY_CRC)) {
		is_fw_upgrade_event_detected = true;
		return;
	}

	if ((received_length != APP_IIC_COMMAND_FRAME_SIZE) || !app_iic_command_is_valid(app_iic_rx_command)) {
		return;
	}

	version = app_iic_rx_command[2];
	command = app_iic_rx_command[3];

	switch (command) {
	case APP_IIC_COMMAND_ADJUST_CHANNELS:
		app_iic_pending_recalibration = true;
		app_iic_build_standard_ack(version, command, 0u);
		break;

	case APP_IIC_COMMAND_SET_ADDRESS:
		if ((app_iic_rx_command[7] != 0u) && (app_iic_rx_command[7] <= 0x6Fu)) {
			app_iic_pending_address = app_iic_rx_command[7];
			app_iic_pending_address_change = true;
		}
		app_iic_build_standard_ack(version, command, app_iic_pending_address_change ? app_iic_pending_address : app_iic_current_address);
		break;

	case APP_IIC_COMMAND_READ_ADDRESS:
		app_iic_build_standard_ack(version, command, app_iic_current_address);
		break;

	case APP_IIC_COMMAND_RESET_ALL_CHANNELS:
		app_iic_build_all_zero_data_frame(app_iic_zero_frame, app_iic_next_frame_number());
		app_iic_zero_frame_armed = true;
		app_iic_recalibration_after_zero = true;
		app_iic_build_standard_ack(version, command, 0u);
		break;

	case APP_IIC_COMMAND_RESET_SINGLE_CHANNEL:
		app_iic_build_standard_ack(version, command, APP_IIC_SINGLE_CHANNEL_UNSUPPORTED_RESULT);
		break;

	case APP_IIC_COMMAND_SET_SEND_DATA_TYPE:
		if (app_iic_rx_command[7] <= APP_IIC_SEND_NOISE) {
			app_iic_send_data_type = app_iic_rx_command[7];
		}
		app_iic_build_standard_ack(version, command, 0u);
		app_iic_publish_latest_data_frame();
		break;

	case APP_IIC_COMMAND_READ_FIRMWARE_VERSION:
		app_iic_build_firmware_reply();
		reply_length = APP_IIC_SPECIAL_A0_FRAME_SIZE;
		break;

	case APP_IIC_COMMAND_SOFTWARE_RESET:
		app_iic_pending_software_reset = true;
		app_iic_build_standard_ack(version, command, 0u);
		break;

	case APP_IIC_COMMAND_READ_DEVICE_TYPE:
		app_iic_build_device_type_reply();
		reply_length = APP_IIC_SPECIAL_A2_FRAME_SIZE;
		break;

	case APP_IIC_COMMAND_DISCOVERY:
		app_iic_build_discovery_reply();
		reply_length = APP_IIC_ACK_FRAME_SIZE;
		break;

	default:
		app_iic_build_standard_ack(version, command, 0u);
		break;
	}

	app_iic_reply_length = reply_length;
	app_iic_reply_pending = true;
}

#else

void AppIic_Init(void) {}

void AppIic_Poll(void) {}

void AppIic_OnScanComplete(void) {}

#endif
