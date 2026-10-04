/* 引入本模块公开接口。 */
#include "capsense_nsub_control.h"
/* 三参数首帧初始化与算法共用Baseline开关，Raw模式不改写扫描值。 */
#include "sensor_algorithm.h"

/* 引入全部逻辑通道三参数同步与实际值回读接口。 */
#include "capsense_channel_config.h"

/* 引入 CapSense 自动生成的参数标识和运行上下文。 */
#include "cycfg_capsense.h"

/* 仅在配置包含旧 MatrixButtons0 Widget 时编译矩阵专用在线调参实现。 */
/* 20点矩阵走统一全通道配置路径，避免引用已经不存在的Button5至Button8。 */
#if defined(CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID) && !DEVICE_MATRIX20

/* 定义本模块实际需要配置的 Widget 数量。 */
#define CAPSENSE_NSUB_CONTROL_WIDGET_COUNT (5u)
/* Only MatrixButtons0 (CH4-CH7) is modified by the V8 Nsub command. */
#define CAPSENSE_NSUB_CONTROL_FIRST_MODIFIED_WIDGET_INDEX (4u)

/* 保存等待应用的 Nsub 档位。 */
static volatile uint16_t pending_nsub;
static volatile uint16_t pending_tx_clk_div;
static volatile uint8_t pending_ref_cdac;
/* 标记是否存在尚未写入 CapSense 的请求。 */
static volatile bool request_pending;
/* 标记是否需要在新配置的第一帧结束后重置滤波器和基线。 */
static bool first_scan_finalize_pending;
/* 保存供通信层读取的当前状态。 */
static capsense_nsub_control_status_t control_status;
static capsense_matrix_config_status_t matrix_config_status;

/* 自动量程兼容路径按CH0～CH7逻辑顺序定义五个物理Widget。 */
#if DEVICE_HAS_MATRIX_AUTORANGE
static const uint8_t widget_ids[CAPSENSE_NSUB_CONTROL_WIDGET_COUNT] = {
	/* CH0 对应 Button8。 */
	CY_CAPSENSE_BUTTON8_WDGT_ID,
	/* CH1 对应 Button7。 */
	CY_CAPSENSE_BUTTON7_WDGT_ID,
	/* CH2 对应 Button6。 */
	CY_CAPSENSE_BUTTON6_WDGT_ID,
	/* CH3 对应 Button5。 */
	CY_CAPSENSE_BUTTON5_WDGT_ID,
	/* CH4～CH7 共用 MatrixButtons0。 */
	CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID,
/* 结束 Widget 标识数组。 */
};
#endif

/* 按相同顺序定义五个 Widget 的 Nsub 参数标识。 */
static const uint32_t nsub_param_ids[CAPSENSE_NSUB_CONTROL_WIDGET_COUNT] = {
	/* Button8 的 Nsub 参数标识。 */
	CY_CAPSENSE_BUTTON8_NUM_SUBCONVERSIONS_PARAM_ID,
	/* Button7 的 Nsub 参数标识。 */
	CY_CAPSENSE_BUTTON7_NUM_SUBCONVERSIONS_PARAM_ID,
	/* Button6 的 Nsub 参数标识。 */
	CY_CAPSENSE_BUTTON6_NUM_SUBCONVERSIONS_PARAM_ID,
	/* Button5 的 Nsub 参数标识。 */
	CY_CAPSENSE_BUTTON5_NUM_SUBCONVERSIONS_PARAM_ID,
	/* MatrixButtons0 的 Nsub 参数标识。 */
	CY_CAPSENSE_MATRIXBUTTONS0_NUM_SUBCONVERSIONS_PARAM_ID,
/* 结束 Nsub 参数标识数组。 */
};

/* 判断请求值是否属于上位机允许选择的四个档位。 */
static bool is_supported_tier(uint16_t nsub) {
	/* 只接受 100、200、400 和 800 四个明确档位。 */
	return (nsub == 100u) || (nsub == 200u) || (nsub == 400u) || (nsub == 800u);
/* 结束档位检查函数。 */
}

static bool is_valid_matrix_config(uint16_t nsub, uint16_t tx_clk_div, uint8_t ref_cdac) {
	/* 按中间件与 AN85951 的约束检查 Nsub、偶数分频值和手动 Reference CDAC 下限。 */
	return (nsub >= 1u) && (nsub <= 16383u) &&
		(tx_clk_div >= 4u) && (tx_clk_div <= 4096u) && ((tx_clk_div & 1u) == 0u) &&
		(ref_cdac >= CY_CAPSENSE_CAL_REF_CDAC_MIN_CODE);
}

/* 自动量程兼容路径同步矩阵TxDivider扫描帧。 */
#if DEVICE_HAS_MATRIX_AUTORANGE
static void sync_matrix_tx_clk_frames(uint16_t tx_clk_div) {
	const cy_stc_capsense_widget_config_t *widget_config =
		&cy_capsense_context.ptrWdConfig[CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID];
	uint32_t encoded = ((uint32_t)tx_clk_div - 1u) << MSCLP_SNS_SNS_CTL_SENSE_DIV_Pos;
	uint32_t slot_id;
	uint32_t *sns_control;

	for (slot_id = widget_config->firstSlotId;
		 slot_id < ((uint32_t)widget_config->firstSlotId + widget_config->numSlots);
		 slot_id++) {
		sns_control = &cy_capsense_context.ptrSensorFrameContext[
			(slot_id * CY_CAPSENSE_SENSOR_FRAME_SIZE) + CY_CAPSENSE_SNS_CTL_INDEX];
		*sns_control &= ~MSCLP_SNS_SNS_CTL_SENSE_DIV_Msk;
		*sns_control |= encoded & MSCLP_SNS_SNS_CTL_SENSE_DIV_Msk;
	}
}
#endif

static void sync_matrix_ref_cdac_frames(uint8_t ref_cdac) {
	const cy_stc_capsense_widget_config_t *widget_config =
		&cy_capsense_context.ptrWdConfig[CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID];
	uint32_t slot_id;
	uint32_t *cdac_control;

	for (slot_id = widget_config->firstSlotId;
		 slot_id < ((uint32_t)widget_config->firstSlotId + widget_config->numSlots);
		 slot_id++) {
		cdac_control = &cy_capsense_context.ptrSensorFrameContext[
			(slot_id * CY_CAPSENSE_SENSOR_FRAME_SIZE) + CY_CAPSENSE_SNS_CDAC_CTL_INDEX];
		*cdac_control &= ~MSCLP_SNS_SNS_CDAC_CTL_SEL_RE_Msk;
		*cdac_control |= ((uint32_t)ref_cdac << MSCLP_SNS_SNS_CDAC_CTL_SEL_RE_Pos) &
			MSCLP_SNS_SNS_CDAC_CTL_SEL_RE_Msk;
	}
}

static bool refresh_matrix_config(void) {
	uint32_t value = 0u;
	cy_capsense_status_t status = Cy_CapSense_GetParam(
		CY_CAPSENSE_MATRIXBUTTONS0_NUM_SUBCONVERSIONS_PARAM_ID, &value,
		&cy_capsense_tuner, &cy_capsense_context);
	matrix_config_status.actual_nsub = (uint16_t)value;
	status |= Cy_CapSense_GetParam(CY_CAPSENSE_MATRIXBUTTONS0_TX_CLK_PARAM_ID, &value,
		&cy_capsense_tuner, &cy_capsense_context);
	matrix_config_status.actual_tx_clk_div = (uint16_t)value;
	status |= Cy_CapSense_GetParam(CY_CAPSENSE_MATRIXBUTTONS0_CDAC_REF0_PARAM_ID, &value,
		&cy_capsense_tuner, &cy_capsense_context);
	matrix_config_status.actual_ref_cdac = (uint8_t)value;
	return status == CY_CAPSENSE_STATUS_SUCCESS;
}

/* 自动量程兼容路径同步单个Widget的Nsub扫描帧。 */
#if DEVICE_HAS_MATRIX_AUTORANGE
static void sync_widget_sensor_frames(uint8_t widget_id, uint16_t nsub) {
	/* 保存当前 Widget 的配置指针。 */
	const cy_stc_capsense_widget_config_t *widget_config;
	/* 保存当前遍历的扫描槽编号。 */
	uint32_t slot_id;
	/* 保存 Nsub 减一后的硬件字段编码。 */
	uint32_t encoded_nsub;
	/* 保存当前扫描槽控制寄存器镜像的地址。 */
	uint32_t *scan_control;

	/* 取得当前 Widget 的运行时配置。 */
	widget_config = &cy_capsense_context.ptrWdConfig[widget_id];
	/* 按 MSCLP 的 Nsub-1 编码规则生成寄存器字段。 */
	encoded_nsub = ((uint32_t)nsub - 1u) << MSCLP_SNS_SNS_SCAN_CTL_NUM_SUB_CONVS_Pos;

	/* 遍历该 Widget 占用的全部扫描槽。 */
	for (slot_id = (uint32_t)widget_config->firstSlotId;
		 slot_id < ((uint32_t)widget_config->firstSlotId + (uint32_t)widget_config->numSlots);
		 slot_id++) {
		/* 定位当前扫描槽的扫描控制寄存器镜像。 */
		scan_control = &cy_capsense_context.ptrSensorFrameContext[(slot_id * CY_CAPSENSE_SENSOR_FRAME_SIZE) + CY_CAPSENSE_SNS_SCAN_CTL_INDEX];
		/* 清除旧的 Nsub 字段。 */
		*scan_control &= ~MSCLP_SNS_SNS_SCAN_CTL_NUM_SUB_CONVS_Msk;
		/* 写入新的 Nsub 字段。 */
		*scan_control |= encoded_nsub & MSCLP_SNS_SNS_SCAN_CTL_NUM_SUB_CONVS_Msk;
	/* 结束当前扫描槽处理。 */
	}
/* 结束扫描槽同步函数。 */
}
#endif

/* 让下一轮扫描重新装载修改后的扫描帧。 */
static void invalidate_repeat_scan(void) {
	/* 禁止沿用上一轮的重复扫描设置。 */
	cy_capsense_context.ptrInternalContext->repeatScanEn = CY_CAPSENSE_DISABLE;
	/* 清除中间件记录的 Widget 扫描状态。 */
	cy_capsense_context.ptrCommonContext->status &= (uint32_t)~CY_CAPSENSE_MW_STATE_WD_SCAN_MASK;
/* 结束重复扫描失效函数。 */
}

/* 在扫描空闲点更新矩阵Ref，并通过回读确认硬件参数已经生效。 */
bool CapSenseMatrixRefCdac_Apply(uint8_t ref_cdac) {
	uint32_t actual_value = 0u;

	/* 拒绝非法Ref以及扫描进行中的写入请求。 */
	if ((ref_cdac < CY_CAPSENSE_CAL_REF_CDAC_MIN_CODE) ||
		(CY_CAPSENSE_NOT_BUSY != Cy_CapSense_IsBusy(&cy_capsense_context))) {
		return false;
	}

	/* 只修改MatrixButtons0的Ref参数，不重算固定的CIC2配置。 */
	if (CY_CAPSENSE_STATUS_SUCCESS != Cy_CapSense_SetParam(
			CY_CAPSENSE_MATRIXBUTTONS0_CDAC_REF0_PARAM_ID, ref_cdac,
			&cy_capsense_tuner, &cy_capsense_context)) {
		return false;
	}

	/* 同步四个矩阵Sensor Frame并禁止沿用旧Ref的重复扫描。 */
	sync_matrix_ref_cdac_frames(ref_cdac);
	invalidate_repeat_scan();

	/* 从中间件回读实际Ref，防止软件状态与硬件参数不一致。 */
	if (CY_CAPSENSE_STATUS_SUCCESS != Cy_CapSense_GetParam(
			CY_CAPSENSE_MATRIXBUTTONS0_CDAC_REF0_PARAM_ID, &actual_value,
			&cy_capsense_tuner, &cy_capsense_context)) {
		return false;
	}

	/* 保存请求值和回读值，供自动量程状态及诊断协议使用。 */
	matrix_config_status.actual_ref_cdac = (uint8_t)actual_value;
	matrix_config_status.requested_ref_cdac = ref_cdac;
	return matrix_config_status.actual_ref_cdac == ref_cdac;
}

/* 把五个 Widget 的实际 Nsub 展开为 CH0～CH7 八通道值。 */
static bool refresh_actual_nsub(void) {
	/* 保存当前 Widget 回读值。 */
	uint32_t actual_value;
	/* 保存当前 Widget 序号。 */
	uint32_t widget_index;
	/* 保存 MatrixButtons0 的共同 Nsub。 */
	uint16_t matrix_nsub;
	/* 标记所有回读操作是否成功。 */
	bool success;

	/* 初始化矩阵 Widget 的回读值。 */
	matrix_nsub = 0u;
	/* 默认所有回读操作均成功。 */
	success = true;

	/* 依次回读五个 Widget 的实际 Nsub。 */
	for (widget_index = 0u; widget_index < CAPSENSE_NSUB_CONTROL_WIDGET_COUNT; widget_index++) {
		/* 清零本次回读缓存。 */
		actual_value = 0u;
		/* 从 CapSense 中间件读取当前 Widget 的 Nsub。 */
		if (CY_CAPSENSE_STATUS_SUCCESS != Cy_CapSense_GetParam(nsub_param_ids[widget_index], &actual_value, &cy_capsense_tuner, &cy_capsense_context)) {
			/* 记录至少有一个 Widget 回读失败。 */
			success = false;
			/* 保留对应通道上一次成功回读的值。 */
			continue;
		/* 结束回读失败分支。 */
		}

		/* 前四个 Widget 分别直接对应 CH0～CH3。 */
		if (widget_index < 4u) {
			/* 保存单通道 Widget 的实际 Nsub。 */
			control_status.actual_nsub[widget_index] = (uint16_t)actual_value;
		/* MatrixButtons0 的一个参数同时作用于 CH4～CH7。 */
		} else {
			/* 保存矩阵 Widget 的共同 Nsub。 */
			matrix_nsub = (uint16_t)actual_value;
		/* 结束 Widget 类型判断。 */
		}
	/* 结束五个 Widget 回读循环。 */
	}

	/* 只有矩阵 Widget 回读成功时才更新 CH4～CH7。 */
	if (matrix_nsub != 0u) {
		/* 保存 CH4 的实际 Nsub。 */
		control_status.actual_nsub[4u] = matrix_nsub;
		/* 保存 CH5 的实际 Nsub。 */
		control_status.actual_nsub[5u] = matrix_nsub;
		/* 保存 CH6 的实际 Nsub。 */
		control_status.actual_nsub[6u] = matrix_nsub;
		/* 保存 CH7 的实际 Nsub。 */
		control_status.actual_nsub[7u] = matrix_nsub;
	/* 结束矩阵通道更新分支。 */
	}

	/* 返回五个 Widget 是否全部回读成功。 */
	return success;
/* 结束实际值刷新函数。 */
}

/* 初始化全通道易失性 Nsub 控制模块。 */
void CapSenseNsubControl_Init(void) {
	/* 初始化请求值为零，表示本次启动尚未手动选择档位。 */
	control_status.requested_nsub = 0u;
	/* 清除等待应用标志。 */
	request_pending = false;
	/* 清除首帧初始化标志。 */
	first_scan_finalize_pending = false;
	/* 默认最近一次操作成功。 */
	control_status.result = (uint8_t)CAPSENSE_NSUB_CONTROL_RESULT_SUCCESS;
	/* 清除对外的等待状态。 */
	control_status.pending = 0u;
	(void)refresh_matrix_config();
	matrix_config_status.requested_nsub = matrix_config_status.actual_nsub;
	matrix_config_status.requested_tx_clk_div = matrix_config_status.actual_tx_clk_div;
	matrix_config_status.requested_ref_cdac = matrix_config_status.actual_ref_cdac;
	matrix_config_status.pending = 0u;
	matrix_config_status.result = CAPSENSE_NSUB_CONTROL_RESULT_SUCCESS;
	/* 读取启动时五个 Widget 的真实 Nsub。 */
	if (!refresh_actual_nsub()) {
		/* 回读失败时记录错误，但不主动修改任何配置。 */
		control_status.result = (uint8_t)CAPSENSE_NSUB_CONTROL_RESULT_GET_FAILED;
	/* 结束启动回读结果判断。 */
	}
/* 结束模块初始化函数。 */
}

/* 请求在扫描空闲点把所有通道切换到指定档位。 */
bool CapSenseNsubControl_Request(uint16_t nsub) {
	/* 保存最近收到的请求值，便于上位机诊断非法请求。 */
	control_status.requested_nsub = nsub;

	/* 拒绝四个允许档位之外的数值。 */
	if (!is_supported_tier(nsub)) {
		/* 记录非法档位错误。 */
		control_status.result = (uint8_t)CAPSENSE_NSUB_CONTROL_RESULT_INVALID_TIER;
		/* 报告请求未被接受。 */
		return false;
	/* 结束非法档位分支。 */
	}

	/* 保存等待应用的有效档位。 */
	pending_nsub = nsub;
	(void)refresh_matrix_config();
	pending_tx_clk_div = matrix_config_status.actual_tx_clk_div;
	pending_ref_cdac = matrix_config_status.actual_ref_cdac;
	/* 标记主循环需要应用本次请求。 */
	request_pending = true;
	/* 对外显示操作正在进行。 */
	control_status.pending = 1u;
	/* 清除上一次操作错误。 */
	control_status.result = (uint8_t)CAPSENSE_NSUB_CONTROL_RESULT_SUCCESS;
	/* 报告请求已被接受。 */
	return true;
/* 结束 Nsub 请求函数。 */
}

bool CapSenseMatrixConfig_Request(uint16_t nsub, uint16_t tx_clk_div, uint8_t ref_cdac) {
	matrix_config_status.requested_nsub = nsub;
	matrix_config_status.requested_tx_clk_div = tx_clk_div;
	matrix_config_status.requested_ref_cdac = ref_cdac;
	if (!is_valid_matrix_config(nsub, tx_clk_div, ref_cdac)) {
		matrix_config_status.result = CAPSENSE_NSUB_CONTROL_RESULT_INVALID_TIER;
		return false;
	}
	pending_nsub = nsub;
	pending_tx_clk_div = tx_clk_div;
	pending_ref_cdac = ref_cdac;
	request_pending = true;
	control_status.requested_nsub = nsub;
	control_status.pending = 1u;
	matrix_config_status.pending = 1u;
	matrix_config_status.result = CAPSENSE_NSUB_CONTROL_RESULT_SUCCESS;
	return true;
}

/* 在扫描空闲时应用等待中的 Nsub 请求，且绝不写入 Flash。 */
bool CapSenseNsubControl_ApplyPending(void) {
	/* 保存本轮需要应用的档位。 */
	uint16_t requested_nsub;
	#if DEVICE_HAS_MATRIX_AUTORANGE
	/* 保存当前 Widget 序号。 */
	uint32_t widget_index;
	#endif
	/* 标记所有 Widget 参数是否写入成功。 */
	bool set_success;

	/* 没有等待请求时不执行任何操作。 */
	if (!request_pending) {
		/* 报告本轮没有应用新配置。 */
		return false;
	/* 结束无请求分支。 */
	}

	/* CapSense 忙时保留请求，等待下一个空闲点。 */
	if (CY_CAPSENSE_NOT_BUSY != Cy_CapSense_IsBusy(&cy_capsense_context)) {
		/* 报告本轮尚未应用新配置。 */
		return false;
	/* 结束忙状态分支。 */
	}

	/* 读取易失性等待值。 */
	requested_nsub = pending_nsub;
	/* 消费本次请求，后续新请求仍可覆盖。 */
	request_pending = false;
	/* 默认所有 Widget 均可写入成功。 */
	set_success = true;

	#if DEVICE_HAS_MATRIX_AUTORANGE
	if (CY_CAPSENSE_STATUS_SUCCESS != Cy_CapSense_SetParam(
			CY_CAPSENSE_MATRIXBUTTONS0_TX_CLK_PARAM_ID, pending_tx_clk_div,
			&cy_capsense_tuner, &cy_capsense_context)) {
		set_success = false;
	} else {
		sync_matrix_tx_clk_frames(pending_tx_clk_div);
	}
	if (CY_CAPSENSE_STATUS_SUCCESS != Cy_CapSense_SetParam(
			CY_CAPSENSE_MATRIXBUTTONS0_CDAC_REF0_PARAM_ID, pending_ref_cdac,
			&cy_capsense_tuner, &cy_capsense_context)) {
		set_success = false;
	} else {
		sync_matrix_ref_cdac_frames(pending_ref_cdac);
	}

	/* 仅设置 MatrixButtons0 的 Nsub，保持 CH0～CH3 不变。 */
	for (widget_index = CAPSENSE_NSUB_CONTROL_FIRST_MODIFIED_WIDGET_INDEX;
		 widget_index < CAPSENSE_NSUB_CONTROL_WIDGET_COUNT;
		 widget_index++) {
		/* 通过 CapSense 中间件修改运行时参数。 */
		if (CY_CAPSENSE_STATUS_SUCCESS != Cy_CapSense_SetParam(nsub_param_ids[widget_index], requested_nsub, &cy_capsense_tuner, &cy_capsense_context)) {
			/* 记录至少有一个 Widget 写入失败。 */
			set_success = false;
			/* 跳过失败 Widget 的扫描帧同步和最大值初始化。 */
			continue;
		/* 结束写入失败分支。 */
		}

		/* 把新值同步到该 Widget 的全部扫描槽。 */
		sync_widget_sensor_frames(widget_ids[widget_index], requested_nsub);
	/* 结束五个 Widget 设置循环。 */
	}

	/* 只有运行参数全部写入成功后，才按启动时的先后顺序重新计算 CIC2 抽取率。 */
	if (set_success &&
		(CY_CAPSENSE_STATUS_SUCCESS != Cy_CapSense_InitializeCic2Rate(&cy_capsense_context))) {
		/* 记录 CIC2 抽取率自动初始化失败。 */
		set_success = false;
	/* 结束 CIC2 抽取率初始化结果判断。 */
	}

	/* 抽取率成功后重新计算 CIC2 Shift，避免沿用旧 Nsub 和 TxDiv 对应的缩放。 */
	if (set_success &&
		(CY_CAPSENSE_STATUS_SUCCESS != Cy_CapSense_InitializeCic2Shift(&cy_capsense_context))) {
		/* 记录 CIC2 Shift 自动初始化失败。 */
		set_success = false;
	/* 结束 CIC2 Shift 初始化结果判断。 */
	}

	/* Rate 和 Shift 都成功后，按启动流程重新测量所有启用 Widget 的最大 Raw。 */
	if (set_success) {
		/* 遍历本工程使用的全部五个 Widget。 */
		for (widget_index = 0u;
			 widget_index < CAPSENSE_NSUB_CONTROL_WIDGET_COUNT;
			 widget_index++) {
			/* 跳过运行时被禁用的 Widget。 */
			if (0u == Cy_CapSense_IsWidgetEnabled(widget_ids[widget_index], &cy_capsense_context)) {
				/* 继续检查下一个 Widget。 */
				continue;
			/* 结束禁用 Widget 判断。 */
			}

			/* 使用新的 CIC2 Rate 和 Shift 重新测量当前 Widget 的最大 Raw。 */
			if (CY_CAPSENSE_STATUS_SUCCESS !=
				Cy_CapSense_InitializeMaxRaw(widget_ids[widget_index], &cy_capsense_context)) {
				/* 记录最大 Raw 重新初始化失败。 */
				set_success = false;
				/* 后续 Widget 不再继续执行依赖同一硬件状态的测量。 */
				break;
			/* 结束最大 Raw 初始化结果判断。 */
			}
		/* 结束全部 Widget 最大 Raw 初始化循环。 */
		}
	/* 结束初始化链路执行条件。 */
	}
	#else
	/* 固定档位固件把同一组三参数同步到全部逻辑通道。 */
	set_success = CapSenseChannelConfig_ApplyFixedAll(
		requested_nsub,
		pending_tx_clk_div,
		pending_ref_cdac);
	#endif

	/* 强制下一轮扫描使用新的扫描帧。 */
	invalidate_repeat_scan();
	/* 标记新配置的第一帧完成后需要重置滤波器和基线。 */
	first_scan_finalize_pending = true;

	/* 回读五个 Widget 并更新八通道状态。 */
	if (!refresh_actual_nsub()) {
		/* 优先报告回读失败，便于发现实际值未知。 */
		control_status.result = (uint8_t)CAPSENSE_NSUB_CONTROL_RESULT_GET_FAILED;
	/* 回读成功但写入不完整时报告写入失败。 */
	} else if (!set_success) {
		/* 记录至少一个 Widget 写入失败。 */
		control_status.result = (uint8_t)CAPSENSE_NSUB_CONTROL_RESULT_SET_FAILED;
	/* 所有写入和回读都成功。 */
	} else {
		/* 记录本次设置成功。 */
		control_status.result = (uint8_t)CAPSENSE_NSUB_CONTROL_RESULT_SUCCESS;
	/* 结束应用结果判断。 */
	}
	(void)refresh_matrix_config();
	matrix_config_status.result = control_status.result;

	/* 报告本轮已经尝试应用新配置。 */
	return true;
/* 结束等待请求应用函数。 */
}

/* 在新 Nsub 的第一帧扫描结束后重置 CapSense 滤波器和基线。 */
bool CapSenseNsubControl_FinalizeAppliedScan(void) {
/* 关闭Baseline时仅完成参数事务，不执行预处理、滤波或基线初始化。 */
#if SENSOR_ALGO_BASELINE_ENABLED
	/* 保存当前 Widget 序号。 */
	uint32_t widget_id;
	/* 保存 CapSense 基线初始化结果。 */
	cy_capsense_status_t baseline_status;
#endif

	/* 尚未完成新配置的第一帧时无需处理。 */
	if (!first_scan_finalize_pending) {
		/* 报告算法层无需重新校准。 */
		return false;
	/* 结束无首帧初始化任务分支。 */
	}

	/* 消费首帧初始化任务。 */
	first_scan_finalize_pending = false;

/* 仅完整算法模式需要按新Raw量级重建CapSense基线。 */
#if SENSOR_ALGO_BASELINE_ENABLED
	/* 先为每个 Widget 预处理刚完成的新 Nsub 原始数据。 */
	for (widget_id = 0u; widget_id < CY_CAPSENSE_TOTAL_WIDGET_COUNT; widget_id++) {
		/* 执行当前 Widget 的预处理。 */
		(void)Cy_CapSense_PreProcessWidget(widget_id, &cy_capsense_context);
	/* 结束全部 Widget 预处理循环。 */
	}

	/* 仅在工程启用 Raw 滤波器时重新初始化全部滤波状态。 */
#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_RAWCOUNT_FILTER_EN)
	/* 清除旧 Nsub 尺度下遗留的滤波历史。 */
	Cy_CapSense_InitializeAllFilters(&cy_capsense_context);
/* 结束 Raw 滤波器条件编译区域。 */
#endif

	/* 用新 Nsub 的首帧数据重新建立 CapSense 基线。 */
	baseline_status = Cy_CapSense_InitializeAllBaselines(&cy_capsense_context);
#endif
	/* 清除对外的等待状态。 */
	control_status.pending = 0u;
	matrix_config_status.pending = 0u;

	/* 基线初始化失败时记录设置失败。 */
#if SENSOR_ALGO_BASELINE_ENABLED
	if (CY_CAPSENSE_STATUS_SUCCESS != baseline_status) {
		/* 保存失败结果供上位机显示。 */
		control_status.result = (uint8_t)CAPSENSE_NSUB_CONTROL_RESULT_SET_FAILED;
		/* 报告算法层不应基于失败配置重新校准。 */
		return false;
	/* 结束基线初始化失败分支。 */
	}
#endif

	/* 报告算法层需要针对新量程执行在线重校准。 */
	return true;
/* 结束首帧初始化函数。 */
}

/* 获取当前请求值、八通道实际值、等待标志和执行结果。 */
void CapSenseNsubControl_GetStatus(capsense_nsub_control_status_t *status) {
	/* 合并尚未应用和等待首帧初始化两种等待状态。 */
	control_status.pending = (request_pending || first_scan_finalize_pending) ? 1u : 0u;
	/* 复制完整状态快照给通信层。 */
	*status = control_status;
/* 结束状态读取函数。 */
}

void CapSenseMatrixConfig_GetStatus(capsense_matrix_config_status_t *status) {
	(void)refresh_matrix_config();
	matrix_config_status.pending = (request_pending || first_scan_finalize_pending) ? 1u : 0u;
	*status = matrix_config_status;
}

/* 五Button和二十点矩阵均通过全通道接口统一更新三参数。 */
#else

/* 保存全部逻辑通道等待应用的三参数。 */
static volatile uint16_t pending_nsub;
static volatile uint16_t pending_tx_clk_div;
static volatile uint8_t pending_ref_cdac;
/* 标记是否存在尚未应用的三参数请求。 */
static volatile bool request_pending;
/* 标记是否需要在新参数首帧后重建滤波和基线。 */
static bool first_scan_finalize_pending;
/* 保存全通道Nsub状态。 */
static capsense_nsub_control_status_t control_status;
/* 保存全通道三参数状态。 */
static capsense_matrix_config_status_t matrix_config_status;

/* 检查运行时三参数是否落在CapSense硬件允许范围内。 */
static bool is_valid_runtime_config(
	uint16_t nsub,
	uint16_t tx_clk_div,
	uint8_t ref_cdac) {
	return (nsub >= 1u) && (nsub <= 16383u) &&
		(tx_clk_div >= 4u) && (tx_clk_div <= 4096u) &&
		((tx_clk_div & 1u) == 0u) &&
		(ref_cdac >= CY_CAPSENSE_CAL_REF_CDAC_MIN_CODE);
}

/* 回读全部通道，并用CH0代表统一三参数的当前值。 */
static bool refresh_runtime_config(void) {
	capsense_channel_config_t configs[DEVICE_SENSOR_CHANNEL_COUNT];
	uint32_t channel_index;
	bool success;

	success = CapSenseChannelConfig_ReadAll(configs);
	for (channel_index = 0u;
		 channel_index < CAPSENSE_NSUB_CONTROL_CHANNEL_COUNT;
		 channel_index++) {
		control_status.actual_nsub[channel_index] =
			configs[channel_index].hardware_nsub;
	}
	matrix_config_status.actual_nsub = configs[0u].hardware_nsub;
	matrix_config_status.actual_tx_clk_div =
		configs[0u].hardware_tx_clk_div;
	matrix_config_status.actual_ref_cdac =
		configs[0u].hardware_ref_cdac;
	return success;
}

/* 根据Configurator当前值初始化全通道易失性三参数状态。 */
void CapSenseNsubControl_Init(void) {
	request_pending = false;
	first_scan_finalize_pending = false;
	control_status.pending = 0u;
	control_status.result = (uint8_t)CAPSENSE_NSUB_CONTROL_RESULT_SUCCESS;
	matrix_config_status.pending = 0u;
	matrix_config_status.result =
		(uint8_t)CAPSENSE_NSUB_CONTROL_RESULT_SUCCESS;
	if (!refresh_runtime_config()) {
		control_status.result =
			(uint8_t)CAPSENSE_NSUB_CONTROL_RESULT_GET_FAILED;
		matrix_config_status.result = control_status.result;
	}
	control_status.requested_nsub = matrix_config_status.actual_nsub;
	matrix_config_status.requested_nsub =
		matrix_config_status.actual_nsub;
	matrix_config_status.requested_tx_clk_div =
		matrix_config_status.actual_tx_clk_div;
	matrix_config_status.requested_ref_cdac =
		matrix_config_status.actual_ref_cdac;
}

/* 旧Nsub单参数命令继续保持禁用，避免与三参数原子更新混用。 */
bool CapSenseNsubControl_Request(uint16_t nsub) {
	control_status.requested_nsub = nsub;
	control_status.result = (uint8_t)CAPSENSE_NSUB_CONTROL_RESULT_UNSUPPORTED_CONFIG;
	return false;
}

/* 保存全通道三参数请求，等待主循环扫描空闲点统一应用。 */
bool CapSenseMatrixConfig_Request(uint16_t nsub, uint16_t tx_clk_div, uint8_t ref_cdac) {
	matrix_config_status.requested_nsub = nsub;
	matrix_config_status.requested_tx_clk_div = tx_clk_div;
	matrix_config_status.requested_ref_cdac = ref_cdac;
	if (!is_valid_runtime_config(nsub, tx_clk_div, ref_cdac)) {
		matrix_config_status.result =
			(uint8_t)CAPSENSE_NSUB_CONTROL_RESULT_INVALID_TIER;
		return false;
	}
	pending_nsub = nsub;
	pending_tx_clk_div = tx_clk_div;
	pending_ref_cdac = ref_cdac;
	request_pending = true;
	control_status.requested_nsub = nsub;
	control_status.pending = 1u;
	matrix_config_status.pending = 1u;
	matrix_config_status.result =
		(uint8_t)CAPSENSE_NSUB_CONTROL_RESULT_SUCCESS;
	return true;
}

/* 拒绝需要统一修改矩阵 Ref CDAC 的请求。 */
bool CapSenseMatrixRefCdac_Apply(uint8_t ref_cdac) {
	matrix_config_status.requested_ref_cdac = ref_cdac;
	matrix_config_status.result = (uint8_t)CAPSENSE_NSUB_CONTROL_RESULT_UNSUPPORTED_CONFIG;
	return false;
}

/* 在扫描空闲点把等待中的三参数应用到全部逻辑通道。 */
bool CapSenseNsubControl_ApplyPending(void) {
	bool success;

	if (!request_pending) {
		return false;
	}
	if (CY_CAPSENSE_NOT_BUSY != Cy_CapSense_IsBusy(&cy_capsense_context)) {
		return false;
	}
	request_pending = false;
	success = CapSenseChannelConfig_ApplyFixedAll(
		pending_nsub,
		pending_tx_clk_div,
		pending_ref_cdac);
	first_scan_finalize_pending = true;
	if (!refresh_runtime_config()) {
		control_status.result =
			(uint8_t)CAPSENSE_NSUB_CONTROL_RESULT_GET_FAILED;
	} else if (!success) {
		control_status.result =
			(uint8_t)CAPSENSE_NSUB_CONTROL_RESULT_SET_FAILED;
	} else {
		control_status.result =
			(uint8_t)CAPSENSE_NSUB_CONTROL_RESULT_SUCCESS;
	}
	matrix_config_status.result = control_status.result;
	return true;
}

/* 新三参数首帧完成后重建滤波历史和全部CapSense基线。 */
bool CapSenseNsubControl_FinalizeAppliedScan(void) {
/* Raw模式保留三参数事务结束逻辑，跳过全部采样后处理。 */
#if SENSOR_ALGO_BASELINE_ENABLED
	uint32_t widget_id;
	cy_capsense_status_t baseline_status;
#endif

	if (!first_scan_finalize_pending) {
		return false;
	}
	first_scan_finalize_pending = false;
/* 完整算法模式才重建滤波历史和基线。 */
#if SENSOR_ALGO_BASELINE_ENABLED
	for (widget_id = 0u;
		 widget_id < CY_CAPSENSE_TOTAL_WIDGET_COUNT;
		 widget_id++) {
		(void)Cy_CapSense_PreProcessWidget(widget_id, &cy_capsense_context);
	}
	#if (CY_CAPSENSE_DISABLE != CY_CAPSENSE_RAWCOUNT_FILTER_EN)
	Cy_CapSense_InitializeAllFilters(&cy_capsense_context);
	#endif
	baseline_status = Cy_CapSense_InitializeAllBaselines(&cy_capsense_context);
#endif
	control_status.pending = 0u;
	matrix_config_status.pending = 0u;
/* Raw模式不产生基线初始化错误。 */
#if SENSOR_ALGO_BASELINE_ENABLED
	if (CY_CAPSENSE_STATUS_SUCCESS != baseline_status) {
		control_status.result =
			(uint8_t)CAPSENSE_NSUB_CONTROL_RESULT_SET_FAILED;
		matrix_config_status.result = control_status.result;
		return false;
	}
#endif
	return true;
}

/* 返回当前全部通道的Nsub实际值。 */
void CapSenseNsubControl_GetStatus(capsense_nsub_control_status_t *status) {
	(void)refresh_runtime_config();
	control_status.pending =
		(request_pending || first_scan_finalize_pending) ? 1u : 0u;
	*status = control_status;
}

/* 返回全部通道共用的三参数及应用状态。 */
void CapSenseMatrixConfig_GetStatus(capsense_matrix_config_status_t *status) {
	(void)refresh_runtime_config();
	matrix_config_status.pending =
		(request_pending || first_scan_finalize_pending) ? 1u : 0u;
	*status = matrix_config_status;
}

#endif
