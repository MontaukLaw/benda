#include "capsense_equivalent.h"

#include "cy_device_headers.h"
#include "cycfg_capsense.h"

#define CAPSENSE_FF_PER_PF (1000.0f)
#define CAPSENSE_CDAC_LSB_SCALE (100.0f)

/* 二十点矩阵的全部通道共享同一个 Widget。 */
static const uint8_t sensor_widget_ids[CAPSENSE_EQUIVALENT_SENSOR_COUNT] = {
#if DEVICE_MATRIX20
	CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID, CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID,
	CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID, CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID,
	CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID, CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID,
	CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID, CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID,
	CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID, CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID,
	CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID, CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID,
	CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID, CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID,
	CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID, CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID,
	CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID, CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID,
	CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID, CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID,
#elif (DEVICE_SENSOR_CHANNEL_COUNT == 5u)
	CY_CAPSENSE_BUTTON0_WDGT_ID,
	CY_CAPSENSE_BUTTON1_WDGT_ID,
	CY_CAPSENSE_BUTTON2_WDGT_ID,
	CY_CAPSENSE_BUTTON3_WDGT_ID,
	CY_CAPSENSE_BUTTON4_WDGT_ID,
#else
	CY_CAPSENSE_BUTTON8_WDGT_ID,
	CY_CAPSENSE_BUTTON7_WDGT_ID,
	CY_CAPSENSE_BUTTON6_WDGT_ID,
	CY_CAPSENSE_BUTTON5_WDGT_ID,
	CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID,
	CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID,
	CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID,
	CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID,
#endif
};

static uint32_t sensor_index_in_widget(uint32_t sensor_id) {
	/* 五通道拓扑的每个Button只包含一个Sensor。 */
#if DEVICE_MATRIX20
	/* 矩阵通道直接对应 Widget 内的传感器编号。 */
	return sensor_id;
#elif (DEVICE_SENSOR_CHANNEL_COUNT == 5u)
	/* 当前五个 Widget 均只包含一个 Sensor。 */
	(void)sensor_id;
	return 0u;
	/* 八通道拓扑的后四路来自同一个2乘2矩阵Widget。 */
#else
	return (sensor_id < 4u) ? 0u : (sensor_id - 4u);
#endif
}

static const uint32_t *scan_slot_register(uint32_t slot_id, uint32_t register_index) {
	return &cy_capsense_context.ptrSensorFrameContext[
		(slot_id * CY_CAPSENSE_SENSOR_FRAME_SIZE) + register_index];
}

float CapSense_CalculateEquivalentCsPf(const capsense_equivalent_params_t *params) {
	float cdac_lsb_ff = (float)CY_CAPSENSE_REF_CDAC_LSB_X100 / CAPSENSE_CDAC_LSB_SCALE;
	float reference_cdac_ff = (float)params->ref_cdac * cdac_lsb_ff;
	float sensor_capacitance_ff =
		((float)params->raw_count * (float)params->tx_clk_div * reference_cdac_ff)
		/ (2.0f * (float)params->max_raw_count);

	if (params->cdac_mode == CAPSENSE_EQUIVALENT_DUAL_CDAC) {
		sensor_capacitance_ff +=
			((float)params->tx_clk_div * (float)params->comp_cdac * cdac_lsb_ff)
			/ (float)params->comp_clk_div;
	}

	return sensor_capacitance_ff / CAPSENSE_FF_PER_PF;
}

void CapSense_GetEquivalentCsParams(uint32_t sensor_id, capsense_equivalent_params_t *params) {
	uint32_t widget_id = sensor_widget_ids[sensor_id];
	uint32_t widget_sensor_id = sensor_index_in_widget(sensor_id);
	const cy_stc_capsense_widget_config_t *widget_config = &cy_capsense_context.ptrWdConfig[widget_id];
	uint32_t slot_id = (uint32_t)widget_config->firstSlotId + widget_sensor_id;
	uint32_t scan_ctl = *scan_slot_register(slot_id, CY_CAPSENSE_SNS_SCAN_CTL_INDEX);
	uint32_t cdac_ctl = *scan_slot_register(slot_id, CY_CAPSENSE_SNS_CDAC_CTL_INDEX);
	uint32_t sns_ctl = *scan_slot_register(slot_id, CY_CAPSENSE_SNS_CTL_INDEX);

	params->raw_count = widget_config->ptrSnsContext[widget_sensor_id].raw;
	params->max_raw_count = widget_config->ptrWdContext->maxRawCount;
	params->nsub = (uint16_t)(
		((scan_ctl & MSCLP_SNS_SNS_SCAN_CTL_NUM_SUB_CONVS_Msk)
			>> MSCLP_SNS_SNS_SCAN_CTL_NUM_SUB_CONVS_Pos) + 1u);
	params->tx_clk_div = (uint16_t)(
		((sns_ctl & MSCLP_SNS_SNS_CTL_SENSE_DIV_Msk)
			>> MSCLP_SNS_SNS_CTL_SENSE_DIV_Pos) + 1u);
	params->comp_clk_div = (uint16_t)(
		((scan_ctl & MSCLP_SNS_SNS_SCAN_CTL_COMP_DIV_Msk)
			>> MSCLP_SNS_SNS_SCAN_CTL_COMP_DIV_Pos) + 1u);
	params->ref_cdac = (uint8_t)(
		(cdac_ctl & MSCLP_SNS_SNS_CDAC_CTL_SEL_RE_Msk)
			>> MSCLP_SNS_SNS_CDAC_CTL_SEL_RE_Pos);
	params->comp_cdac = (uint8_t)(
		(cdac_ctl & MSCLP_SNS_SNS_CDAC_CTL_SEL_CO_Msk)
			>> MSCLP_SNS_SNS_CDAC_CTL_SEL_CO_Pos);
	params->cdac_mode = (params->comp_cdac == 0u)
		? CAPSENSE_EQUIVALENT_SINGLE_CDAC
		: CAPSENSE_EQUIVALENT_DUAL_CDAC;
}

float CapSense_GetEquivalentCs(uint32_t sensor_id) {
	capsense_equivalent_params_t params;

	CapSense_GetEquivalentCsParams(sensor_id, &params);
	return CapSense_CalculateEquivalentCsPf(&params);
}
