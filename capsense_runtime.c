#include "capsense_runtime.h"
/* 在线参数更新与算法共用Baseline开关。 */
#include "sensor_algorithm.h"

#include "cy_pdl.h"
#include "cycfg_capsense.h"
#include <string.h>

/* 仅在配置包含旧 MatrixButtons0 Widget 时启用矩阵参数写入和持久化。 */
#if defined(CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID)


static volatile uint16_t pending_nsub;
static volatile bool capsense_nsub_update_pending;
static volatile uint8_t pending_cref;
static volatile bool capsense_cref_update_pending;
static capsense_runtime_nsub_status_t nsub_status;
static capsense_runtime_cref_status_t cref_status;
static uint32_t scan_start_cycles;
static uint32_t scan_time_us;
static uint32_t scan_fps_milli;
static bool scan_timing_active;
static bool baseline_update_pending;
static cy_capsense_status_t baseline_status;
static uint32_t persistence_status;
static bool persistent_config_loaded;

static const uint32_t comp_cdac_param_ids[CAPSENSE_RUNTIME_MATRIX_SENSOR_COUNT] = {
	CY_CAPSENSE_MATRIXBUTTONS0_RX0_TX0_CDAC_COMP0_PARAM_ID,
	CY_CAPSENSE_MATRIXBUTTONS0_RX0_TX1_CDAC_COMP0_PARAM_ID,
	CY_CAPSENSE_MATRIXBUTTONS0_RX1_TX0_CDAC_COMP0_PARAM_ID,
	CY_CAPSENSE_MATRIXBUTTONS0_RX1_TX1_CDAC_COMP0_PARAM_ID,
};

static uint32_t *matrix_slot_register(uint32_t sensor_index, uint32_t register_index) {
	uint32_t slot = CY_CAPSENSE_MATRIXBUTTONS0_FIRST_SLOT_ID_VALUE + sensor_index;
	return &cy_capsense_context.ptrSensorFrameContext[
		(slot * CY_CAPSENSE_SENSOR_FRAME_SIZE) + register_index];
}

static void invalidate_repeat_scan(void) {
	cy_capsense_context.ptrInternalContext->repeatScanEn = CY_CAPSENSE_DISABLE;
	cy_capsense_context.ptrCommonContext->status &= (uint32_t)~CY_CAPSENSE_MW_STATE_WD_SCAN_MASK;
}


/* 彻底停用旧配置保存；原地址属于DFU备份，任何调用都不得擦写。 */
static uint32_t save_persistent_config(uint16_t nsub, uint8_t cref) {
    /* 参数只保留接口兼容，不再构造Flash记录。 */
    (void)nsub;
    (void)cref;
    /* 返回明确不支持状态，让调用者不能误报保存成功。 */
    return (uint32_t)CY_FLASH_DRV_DEV_NOT_SUPPORTED;
}

static void sync_matrix_buttons_nsub_frames(uint16_t nsub) {
	uint32_t slot;
	uint32_t encoded_nsub = ((uint32_t)nsub - 1u) << MSCLP_SNS_SNS_SCAN_CTL_NUM_SUB_CONVS_Pos;
	uint32_t *scan_ctl;

	for (slot = CY_CAPSENSE_MATRIXBUTTONS0_FIRST_SLOT_ID_VALUE;
	     slot < (CY_CAPSENSE_MATRIXBUTTONS0_FIRST_SLOT_ID_VALUE + CY_CAPSENSE_MATRIXBUTTONS0_NUM_SLOTS_VALUE);
	     slot++) {
		scan_ctl = &cy_capsense_context.ptrSensorFrameContext[
			(slot * CY_CAPSENSE_SENSOR_FRAME_SIZE) + CY_CAPSENSE_SNS_SCAN_CTL_INDEX];
		*scan_ctl &= ~MSCLP_SNS_SNS_SCAN_CTL_NUM_SUB_CONVS_Msk;
		*scan_ctl |= encoded_nsub & MSCLP_SNS_SNS_SCAN_CTL_NUM_SUB_CONVS_Msk;
	}
}

static void sync_matrix_buttons_cref_frames(uint8_t cref) {
	uint32_t sensor_index;
	uint32_t *cdac_ctl;

	for (sensor_index = 0u; sensor_index < CAPSENSE_RUNTIME_MATRIX_SENSOR_COUNT; sensor_index++) {
		cdac_ctl = matrix_slot_register(sensor_index, CY_CAPSENSE_SNS_CDAC_CTL_INDEX);
		*cdac_ctl &= ~MSCLP_SNS_SNS_CDAC_CTL_SEL_RE_Msk;
		*cdac_ctl |= (uint32_t)cref << MSCLP_SNS_SNS_CDAC_CTL_SEL_RE_Pos;
	}
}

void CapSenseRuntime_Init(void) {
	uint32_t actual_nsub = 0u;

	nsub_status.get_status = Cy_CapSense_GetParam(
		CY_CAPSENSE_MATRIXBUTTONS0_NUM_SUBCONVERSIONS_PARAM_ID,
		&actual_nsub,
		&cy_capsense_tuner,
		&cy_capsense_context);
	nsub_status.actual_nsub = (uint16_t)actual_nsub;
	nsub_status.requested_nsub = nsub_status.actual_nsub;
	nsub_status.set_status = CY_CAPSENSE_STATUS_SUCCESS;
	pending_nsub = nsub_status.actual_nsub;
	capsense_nsub_update_pending = false;

	actual_nsub = 0u;
	cref_status.get_status = Cy_CapSense_GetParam(
		CY_CAPSENSE_MATRIXBUTTONS0_CDAC_REF0_PARAM_ID,
		&actual_nsub,
		&cy_capsense_tuner,
		&cy_capsense_context);
	cref_status.actual_cref = (uint8_t)actual_nsub;
	cref_status.requested_cref = cref_status.actual_cref;
	cref_status.set_status = CY_CAPSENSE_STATUS_SUCCESS;
	pending_cref = cref_status.actual_cref;
	capsense_cref_update_pending = false;
	persistence_status = (uint32_t)CY_FLASH_DRV_SUCCESS;
	persistent_config_loaded = false;
#if 0 /* Use the generated CapSense configuration instead of persisted values. */
	persistent_config_loaded = load_persistent_config(&stored_nsub, &stored_cref);
#endif
	baseline_status = CY_CAPSENSE_STATUS_SUCCESS;
	baseline_update_pending = false;
#if 0 /* Temporarily disable applying persisted Nsub/Cref values. */
	if (persistent_config_loaded) {
		nsub_status.requested_nsub = stored_nsub;
		nsub_status.set_status = Cy_CapSense_SetParam(CY_CAPSENSE_MATRIXBUTTONS0_NUM_SUBCONVERSIONS_PARAM_ID, stored_nsub, &cy_capsense_tuner, &cy_capsense_context);
		nsub_status.get_status = Cy_CapSense_GetParam(CY_CAPSENSE_MATRIXBUTTONS0_NUM_SUBCONVERSIONS_PARAM_ID, &actual_nsub, &cy_capsense_tuner, &cy_capsense_context);
		nsub_status.actual_nsub = (uint16_t)actual_nsub;
		cref_status.requested_cref = stored_cref;
		cref_status.set_status = Cy_CapSense_SetParam(CY_CAPSENSE_MATRIXBUTTONS0_CDAC_REF0_PARAM_ID, stored_cref, &cy_capsense_tuner, &cy_capsense_context);
		cref_status.get_status = Cy_CapSense_GetParam(CY_CAPSENSE_MATRIXBUTTONS0_CDAC_REF0_PARAM_ID, &actual_nsub, &cy_capsense_tuner, &cy_capsense_context);
		cref_status.actual_cref = (uint8_t)actual_nsub;
		if ((nsub_status.set_status == CY_CAPSENSE_STATUS_SUCCESS) && (nsub_status.get_status == CY_CAPSENSE_STATUS_SUCCESS) &&
		    (cref_status.set_status == CY_CAPSENSE_STATUS_SUCCESS) && (cref_status.get_status == CY_CAPSENSE_STATUS_SUCCESS)) {
			sync_matrix_buttons_nsub_frames(nsub_status.actual_nsub);
			sync_matrix_buttons_cref_frames(cref_status.actual_cref);
			invalidate_repeat_scan();
			(void)Cy_CapSense_InitializeMaxRaw(CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID, &cy_capsense_context);
			baseline_update_pending = true;
		}
	}
#endif

	Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_CPU, 0x00FFFFFFu);
	Cy_SysTick_DisableInterrupt();
	scan_timing_active = false;
}

bool CapSenseRuntime_RequestNsub(uint16_t nsub) {
	nsub_status.requested_nsub = nsub;
	if ((nsub < CAPSENSE_RUNTIME_NSUB_MIN) || (nsub > CAPSENSE_RUNTIME_NSUB_MAX)) {
		nsub_status.set_status = CY_CAPSENSE_STATUS_BAD_PARAM;
		return false;
	}

	pending_nsub = nsub;
	capsense_nsub_update_pending = true;
	return true;
}

bool CapSenseRuntime_RequestCref(uint8_t cref) {
	cref_status.requested_cref = cref;
	if ((cref < CAPSENSE_RUNTIME_CREF_MIN) || (cref > CAPSENSE_RUNTIME_CREF_MAX)) {
		cref_status.set_status = CY_CAPSENSE_STATUS_BAD_PARAM;
		return false;
	}

	pending_cref = cref;
	capsense_cref_update_pending = true;
	return true;
}

void CapSenseRuntime_ApplyPending(void) {
	uint16_t requested_nsub;
	uint32_t actual_nsub = nsub_status.actual_nsub;
	uint8_t requested_cref;
	uint32_t actual_cref = cref_status.actual_cref;
	bool config_changed = false;
	bool measurement_timing_changed = false;

	if (CY_CAPSENSE_NOT_BUSY != Cy_CapSense_IsBusy(&cy_capsense_context)) {
		return;
	}

	if (capsense_nsub_update_pending) {
		requested_nsub = pending_nsub;
		capsense_nsub_update_pending = false;
		nsub_status.set_status = Cy_CapSense_SetParam(
			CY_CAPSENSE_MATRIXBUTTONS0_NUM_SUBCONVERSIONS_PARAM_ID,
			requested_nsub,
			&cy_capsense_tuner,
			&cy_capsense_context);
		nsub_status.get_status = Cy_CapSense_GetParam(
			CY_CAPSENSE_MATRIXBUTTONS0_NUM_SUBCONVERSIONS_PARAM_ID,
			&actual_nsub,
			&cy_capsense_tuner,
			&cy_capsense_context);
		nsub_status.actual_nsub = (uint16_t)actual_nsub;

		if ((CY_CAPSENSE_STATUS_SUCCESS == nsub_status.set_status) &&
		    (CY_CAPSENSE_STATUS_SUCCESS == nsub_status.get_status)) {
			sync_matrix_buttons_nsub_frames(nsub_status.actual_nsub);
			invalidate_repeat_scan();
			config_changed = true;
			measurement_timing_changed = true;
		}
	}

	if (capsense_cref_update_pending) {
		requested_cref = pending_cref;
		capsense_cref_update_pending = false;
		cref_status.set_status = Cy_CapSense_SetParam(
			CY_CAPSENSE_MATRIXBUTTONS0_CDAC_REF0_PARAM_ID,
			requested_cref,
			&cy_capsense_tuner,
			&cy_capsense_context);
		cref_status.get_status = Cy_CapSense_GetParam(
			CY_CAPSENSE_MATRIXBUTTONS0_CDAC_REF0_PARAM_ID,
			&actual_cref,
			&cy_capsense_tuner,
			&cy_capsense_context);
		cref_status.actual_cref = (uint8_t)actual_cref;

		if ((CY_CAPSENSE_STATUS_SUCCESS == cref_status.set_status) &&
		    (CY_CAPSENSE_STATUS_SUCCESS == cref_status.get_status)) {
			sync_matrix_buttons_cref_frames(cref_status.actual_cref);
			invalidate_repeat_scan();
			config_changed = true;
		}
	}

	if (config_changed) {
		if (measurement_timing_changed) {
			(void)Cy_CapSense_InitializeMaxRaw(CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID, &cy_capsense_context);
		}
		persistence_status = save_persistent_config(nsub_status.actual_nsub, cref_status.actual_cref);
		persistent_config_loaded = persistence_status == (uint32_t)CY_FLASH_DRV_SUCCESS;
		baseline_update_pending = true;
	}
}

void CapSenseRuntime_GetNsubStatus(capsense_runtime_nsub_status_t *status) {
	*status = nsub_status;
}

void CapSenseRuntime_GetCrefStatus(capsense_runtime_cref_status_t *status) {
	*status = cref_status;
}

void CapSenseRuntime_GetConfig(capsense_runtime_config_t *config) {
	uint32_t value;
	uint32_t sensor_index;
	uint32_t frame_value;
	cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;

	status |= Cy_CapSense_GetParam(CY_CAPSENSE_MATRIXBUTTONS0_NUM_SUBCONVERSIONS_PARAM_ID, &value, &cy_capsense_tuner, &cy_capsense_context);
	config->measurement_nsub = (uint16_t)value;
	frame_value = *matrix_slot_register(0u, CY_CAPSENSE_SNS_SCAN_CTL_INDEX);
	config->hardware_nsub = (uint16_t)(((frame_value & MSCLP_SNS_SNS_SCAN_CTL_NUM_SUB_CONVS_Msk) >> MSCLP_SNS_SNS_SCAN_CTL_NUM_SUB_CONVS_Pos) + 1u);
	config->init_nsub = cy_capsense_context.ptrInternalContext->numFineInitCycles;

	status |= Cy_CapSense_GetParam(CY_CAPSENSE_MATRIXBUTTONS0_CDAC_REF0_PARAM_ID, &value, &cy_capsense_tuner, &cy_capsense_context);
	config->ref_cdac = (uint8_t)value;
	frame_value = *matrix_slot_register(0u, CY_CAPSENSE_SNS_CDAC_CTL_INDEX);
	config->hardware_ref_cdac = (uint8_t)((frame_value & MSCLP_SNS_SNS_CDAC_CTL_SEL_RE_Msk) >> MSCLP_SNS_SNS_CDAC_CTL_SEL_RE_Pos);

	for (sensor_index = 0u; sensor_index < CAPSENSE_RUNTIME_MATRIX_SENSOR_COUNT; sensor_index++) {
		status |= Cy_CapSense_GetParam(comp_cdac_param_ids[sensor_index], &value, &cy_capsense_tuner, &cy_capsense_context);
		config->comp_cdac[sensor_index] = (uint8_t)value;
		frame_value = *matrix_slot_register(sensor_index, CY_CAPSENSE_SNS_CDAC_CTL_INDEX);
		config->hardware_comp_cdac[sensor_index] = (uint8_t)((frame_value & MSCLP_SNS_SNS_CDAC_CTL_SEL_CO_Msk) >> MSCLP_SNS_SNS_CDAC_CTL_SEL_CO_Pos);
	}

	status |= Cy_CapSense_GetParam(CY_CAPSENSE_MATRIXBUTTONS0_TX_CLK_PARAM_ID, &value, &cy_capsense_tuner, &cy_capsense_context);
	config->sense_clk_div = (uint16_t)value;
	frame_value = *matrix_slot_register(0u, CY_CAPSENSE_SNS_CTL_INDEX);
	config->hardware_sense_clk_div = (uint16_t)(((frame_value & MSCLP_SNS_SNS_CTL_SENSE_DIV_Msk) >> MSCLP_SNS_SNS_CTL_SENSE_DIV_Pos) + 1u);
	status |= Cy_CapSense_GetParam(CY_CAPSENSE_MATRIXBUTTONS0_MAX_RAW_COUNT_PARAM_ID, &value, &cy_capsense_tuner, &cy_capsense_context);
	config->max_count = (uint16_t)value;

	config->smartsense_enabled = (uint8_t)CY_CAPSENSE_SMARTSENSE_EN;
	config->auto_calibration_enabled = (uint8_t)CY_CAPSENSE_CSX_CALIBRATION_EN;
	config->capsense_busy = (uint8_t)Cy_CapSense_IsBusy(&cy_capsense_context);
	config->scan_time_us = scan_time_us;
	config->fps_milli = scan_fps_milli;
	config->get_status = status;
	config->persistence_status = persistence_status;
	config->persistent_config_loaded = persistent_config_loaded ? 1u : 0u;
	config->baseline_status = baseline_status;
	config->baseline_update_pending = baseline_update_pending ? 1u : 0u;
}

void CapSenseRuntime_MarkScanStarted(void) {
	scan_start_cycles = Cy_SysTick_GetValue();
	scan_timing_active = true;
}

void CapSenseRuntime_MarkScanComplete(void) {
	uint32_t end_cycles;
	uint32_t elapsed_cycles;

	if (!scan_timing_active) {
		return;
	}

	end_cycles = Cy_SysTick_GetValue();
	if (scan_start_cycles >= end_cycles) {
		elapsed_cycles = scan_start_cycles - end_cycles;
	} else {
		elapsed_cycles = scan_start_cycles + 0x01000000u - end_cycles;
	}
	scan_time_us = elapsed_cycles / (CY_CAPSENSE_CPU_CLK / 1000000u);
	scan_fps_milli = (scan_time_us == 0u) ? 0u : (1000000000u / scan_time_us);
	scan_timing_active = false;
}

bool CapSenseRuntime_UpdateBaselineIfPending(void) {
/* Raw模式仍消费待办标志，但不运行会改写Raw的预处理或基线初始化。 */
#if SENSOR_ALGO_BASELINE_ENABLED
	uint32_t widget_id;
#endif

	if (!baseline_update_pending) {
		return false;
	}
	baseline_update_pending = false;
#if SENSOR_ALGO_BASELINE_ENABLED
	for (widget_id = 0u; widget_id < CY_CAPSENSE_TOTAL_WIDGET_COUNT; widget_id++) {
		Cy_CapSense_PreProcessWidget(widget_id, &cy_capsense_context);
	}
#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_RAWCOUNT_FILTER_EN)
	Cy_CapSense_InitializeAllFilters(&cy_capsense_context);
#endif
	baseline_status = Cy_CapSense_InitializeAllBaselines(&cy_capsense_context);
#else
	baseline_status = CY_CAPSENSE_STATUS_SUCCESS;
#endif
	return baseline_status == CY_CAPSENSE_STATUS_SUCCESS;
}

/* 五个独立 Button 保留各自 Configurator 参数，只提供只读诊断和扫描计时。 */
#else

/* 保存五 Button 配置的 Nsub 只读状态。 */
static capsense_runtime_nsub_status_t nsub_status;
/* 保存五 Button 配置的 Ref CDAC 只读状态。 */
static capsense_runtime_cref_status_t cref_status;
/* 保存最近一次扫描开始时的 SysTick。 */
static uint32_t scan_start_cycles;
/* 保存最近一次完整扫描耗时。 */
static uint32_t scan_time_us;
/* 保存根据扫描耗时换算的毫帧率。 */
static uint32_t scan_fps_milli;
/* 标记扫描计时是否已经开始。 */
static bool scan_timing_active;

/* 返回 Button0 首个扫描槽的寄存器镜像。 */
static const uint32_t *button0_slot_register(uint32_t register_index) {
	uint32_t slot = cy_capsense_context.ptrWdConfig[CY_CAPSENSE_BUTTON0_WDGT_ID].firstSlotId;
	return &cy_capsense_context.ptrSensorFrameContext[
		(slot * CY_CAPSENSE_SENSOR_FRAME_SIZE) + register_index];
}

/* 初始化五 Button 配置的只读诊断值和扫描计时。 */
void CapSenseRuntime_Init(void) {
	uint32_t value = 0u;

	nsub_status.get_status = Cy_CapSense_GetParam(
		CY_CAPSENSE_BUTTON0_NUM_SUBCONVERSIONS_PARAM_ID,
		&value,
		&cy_capsense_tuner,
		&cy_capsense_context);
	nsub_status.actual_nsub = (uint16_t)value;
	nsub_status.requested_nsub = nsub_status.actual_nsub;
	nsub_status.set_status = CY_CAPSENSE_STATUS_SUCCESS;
	value = 0u;
	cref_status.get_status = Cy_CapSense_GetParam(
		CY_CAPSENSE_BUTTON0_CDAC_REF0_PARAM_ID,
		&value,
		&cy_capsense_tuner,
		&cy_capsense_context);
	cref_status.actual_cref = (uint8_t)value;
	cref_status.requested_cref = cref_status.actual_cref;
	cref_status.set_status = CY_CAPSENSE_STATUS_SUCCESS;
	Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_CPU, 0x00FFFFFFu);
	Cy_SysTick_DisableInterrupt();
	scan_timing_active = false;
}

/* 拒绝会覆盖五个 Button 独立 Nsub 的旧矩阵命令。 */
bool CapSenseRuntime_RequestNsub(uint16_t nsub) {
	nsub_status.requested_nsub = nsub;
	nsub_status.set_status = CY_CAPSENSE_STATUS_BAD_PARAM;
	return false;
}

/* 拒绝会覆盖五个 Button 独立 Ref CDAC 的旧矩阵命令。 */
bool CapSenseRuntime_RequestCref(uint8_t cref) {
	cref_status.requested_cref = cref;
	cref_status.set_status = CY_CAPSENSE_STATUS_BAD_PARAM;
	return false;
}

/* 五 Button 配置没有等待写入的矩阵参数。 */
void CapSenseRuntime_ApplyPending(void) {
}

/* 返回 Button0 的代表性 Nsub 诊断状态。 */
void CapSenseRuntime_GetNsubStatus(capsense_runtime_nsub_status_t *status) {
	*status = nsub_status;
}

/* 返回 Button0 的代表性 Ref CDAC 诊断状态。 */
void CapSenseRuntime_GetCrefStatus(capsense_runtime_cref_status_t *status) {
	*status = cref_status;
}

/* 读取 Button0 的代表性配置，同时保留原协议的四项补偿 CDAC 字段。 */
void CapSenseRuntime_GetConfig(capsense_runtime_config_t *config) {
	uint32_t value = 0u;
	uint32_t frame_value;
	uint32_t sensor_index;
	cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;

	status |= Cy_CapSense_GetParam(CY_CAPSENSE_BUTTON0_NUM_SUBCONVERSIONS_PARAM_ID, &value, &cy_capsense_tuner, &cy_capsense_context);
	config->measurement_nsub = (uint16_t)value;
	frame_value = *button0_slot_register(CY_CAPSENSE_SNS_SCAN_CTL_INDEX);
	config->hardware_nsub = (uint16_t)(((frame_value & MSCLP_SNS_SNS_SCAN_CTL_NUM_SUB_CONVS_Msk) >> MSCLP_SNS_SNS_SCAN_CTL_NUM_SUB_CONVS_Pos) + 1u);
	config->init_nsub = cy_capsense_context.ptrInternalContext->numFineInitCycles;
	status |= Cy_CapSense_GetParam(CY_CAPSENSE_BUTTON0_CDAC_REF0_PARAM_ID, &value, &cy_capsense_tuner, &cy_capsense_context);
	config->ref_cdac = (uint8_t)value;
	frame_value = *button0_slot_register(CY_CAPSENSE_SNS_CDAC_CTL_INDEX);
	config->hardware_ref_cdac = (uint8_t)((frame_value & MSCLP_SNS_SNS_CDAC_CTL_SEL_RE_Msk) >> MSCLP_SNS_SNS_CDAC_CTL_SEL_RE_Pos);
	for (sensor_index = 0u; sensor_index < CAPSENSE_RUNTIME_MATRIX_SENSOR_COUNT; sensor_index++) {
		config->comp_cdac[sensor_index] = 0u;
		config->hardware_comp_cdac[sensor_index] = 0u;
	}
	status |= Cy_CapSense_GetParam(CY_CAPSENSE_BUTTON0_RX0_CDAC_COMP0_PARAM_ID, &value, &cy_capsense_tuner, &cy_capsense_context);
	config->comp_cdac[0u] = (uint8_t)value;
	config->hardware_comp_cdac[0u] = (uint8_t)((frame_value & MSCLP_SNS_SNS_CDAC_CTL_SEL_CO_Msk) >> MSCLP_SNS_SNS_CDAC_CTL_SEL_CO_Pos);
	status |= Cy_CapSense_GetParam(CY_CAPSENSE_BUTTON0_TX_CLK_PARAM_ID, &value, &cy_capsense_tuner, &cy_capsense_context);
	config->sense_clk_div = (uint16_t)value;
	frame_value = *button0_slot_register(CY_CAPSENSE_SNS_CTL_INDEX);
	config->hardware_sense_clk_div = (uint16_t)(((frame_value & MSCLP_SNS_SNS_CTL_SENSE_DIV_Msk) >> MSCLP_SNS_SNS_CTL_SENSE_DIV_Pos) + 1u);
	status |= Cy_CapSense_GetParam(CY_CAPSENSE_BUTTON0_MAX_RAW_COUNT_PARAM_ID, &value, &cy_capsense_tuner, &cy_capsense_context);
	config->max_count = (uint16_t)value;
	config->smartsense_enabled = (uint8_t)CY_CAPSENSE_SMARTSENSE_EN;
	config->auto_calibration_enabled = (uint8_t)CY_CAPSENSE_CSX_CALIBRATION_EN;
	config->capsense_busy = (uint8_t)Cy_CapSense_IsBusy(&cy_capsense_context);
	config->scan_time_us = scan_time_us;
	config->fps_milli = scan_fps_milli;
	config->get_status = status;
	config->persistence_status = (uint32_t)CY_FLASH_DRV_SUCCESS;
	config->persistent_config_loaded = 0u;
	config->baseline_status = CY_CAPSENSE_STATUS_SUCCESS;
	config->baseline_update_pending = 0u;
}

/* 记录下一轮扫描的起始时间。 */
void CapSenseRuntime_MarkScanStarted(void) {
	scan_start_cycles = Cy_SysTick_GetValue();
	scan_timing_active = true;
}

/* 计算本轮扫描耗时和对应帧率。 */
void CapSenseRuntime_MarkScanComplete(void) {
	uint32_t end_cycles;
	uint32_t elapsed_cycles;

	if (!scan_timing_active) {
		return;
	}
	end_cycles = Cy_SysTick_GetValue();
	elapsed_cycles = (scan_start_cycles >= end_cycles)
		? (scan_start_cycles - end_cycles)
		: (scan_start_cycles + 0x01000000u - end_cycles);
	scan_time_us = elapsed_cycles / (CY_CAPSENSE_CPU_CLK / 1000000u);
	scan_fps_milli = (scan_time_us == 0u) ? 0u : (1000000000u / scan_time_us);
	scan_timing_active = false;
}

/* 五 Button 配置不会产生矩阵参数更新后的基线任务。 */
bool CapSenseRuntime_UpdateBaselineIfPending(void) {
	return false;
}

#endif

/* 保留实际扫描耗时，另外用完整帧周期更新对外FPS诊断。 */
void CapSenseRuntime_SetFramePeriodUs(uint32_t frame_period_us) {
	scan_fps_milli = frame_period_us == 0u ? 0u : 1000000000u / frame_period_us;
}
