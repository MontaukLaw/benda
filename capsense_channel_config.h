#ifndef CAPSENSE_CHANNEL_CONFIG_H
#define CAPSENSE_CHANNEL_CONFIG_H

#include "device_variant.h"
#include <stdbool.h>
#include <stdint.h>

/* 保存单个逻辑通道的配置值、扫描帧硬件值和Raw满量程。 */
typedef struct {
	uint16_t configured_nsub;
	uint16_t hardware_nsub;
	uint16_t configured_tx_clk_div;
	uint16_t hardware_tx_clk_div;
	uint16_t max_raw_count;
	uint8_t configured_ref_cdac;
	uint8_t hardware_ref_cdac;
	uint8_t widget_id;
	uint8_t sensor_id;
} capsense_channel_config_t;

/* 按CH0到最后一个逻辑通道的顺序读取当前CapSense参数。 */
bool CapSenseChannelConfig_ReadAll(
	capsense_channel_config_t configs[DEVICE_SENSOR_CHANNEL_COUNT]);

/* 在首次扫描前把全部逻辑通道强制设置为同一固定档位。 */
bool CapSenseChannelConfig_ApplyFixedAll(
	uint16_t nsub,
	uint16_t tx_clk_div,
	uint8_t ref_cdac);

#endif
