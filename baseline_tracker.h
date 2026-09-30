/* 基线模块只执行当前通道状态对应的滤波，不再维护第二套状态机。 */
#ifndef BASELINE_TRACKER_H
#define BASELINE_TRACKER_H

/* 引入统一通道状态和整数类型。 */
#include "sensor_algorithm.h"

/* 保存本帧基线执行器输入，不保留历史状态或恢复计数。 */
typedef struct {
	/* 本帧Raw及相邻帧变化，仅供零点重建的重新按压保护。 */
	uint16_t raw;
	int32_t raw_delta;
	/* 唯一通道状态机本帧已经提交的真实状态。 */
	sensor_algorithm_state_t channel_state;
	/* 状态机给出的本帧冻结请求，仅作执行许可，不维护候选或历史阶段。 */
	bool hold_requested;
	/* 校准由专用流程处理，活动抑制期间不做普通基线跟随。 */
	bool calibrated;
	bool suppress_activity;
} baseline_tracker_input_t;

/* 根据IDLE、RECOVER、ZERO_REBASE或冻结状态执行一次基线更新。 */
void BaselineTracker_Update(const baseline_tracker_input_t *input, uint32_t *baseline_q16);
#endif
