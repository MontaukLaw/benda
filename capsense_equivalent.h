#ifndef CAPSENSE_EQUIVALENT_H
#define CAPSENSE_EQUIVALENT_H

#include <stdint.h>

/* 引入当前手指变体对应的传感器数量。 */
#include "device_variant.h"

/* 等效电容诊断覆盖当前固件的全部传感器通道。 */
#define CAPSENSE_EQUIVALENT_SENSOR_COUNT DEVICE_SENSOR_CHANNEL_COUNT

typedef enum {
	CAPSENSE_EQUIVALENT_SINGLE_CDAC = 0u,
	CAPSENSE_EQUIVALENT_DUAL_CDAC = 1u,
} capsense_equivalent_cdac_mode_t;

typedef struct {
	uint16_t raw_count;
	uint16_t max_raw_count;
	uint16_t nsub;
	uint16_t tx_clk_div;
	uint16_t comp_clk_div;
	uint8_t ref_cdac;
	uint8_t comp_cdac;
	capsense_equivalent_cdac_mode_t cdac_mode;
} capsense_equivalent_params_t;

float CapSense_CalculateEquivalentCsPf(const capsense_equivalent_params_t *params);
void CapSense_GetEquivalentCsParams(uint32_t sensor_id, capsense_equivalent_params_t *params);
float CapSense_GetEquivalentCs(uint32_t sensor_id);

#endif
