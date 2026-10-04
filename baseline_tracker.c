/* 基线滤波执行器：状态转换全部由channel_state_machine负责。 */
#include "baseline_tracker.h"

/* 二十点矩阵的空闲跟随降至原来的约十分之一，减少短暂负向形变污染零点。 */
#define BASELINE_Q16_ONE (65536u)
#if DEVICE_MATRIX20
#define BASELINE_IDLE_ALPHA_Q16 (33u)
#else
/* 其他手指保持原有空闲跟随速度。 */
#define BASELINE_IDLE_ALPHA_Q16 (327u)
#endif
/* 矩阵稳定释放后将恢复系数提高至0.04，缩短零点重建尾段。 */
#if DEVICE_MATRIX20
#define BASELINE_RECOVERY_ALPHA_Q16 (2621u)
#else
/* 其他手指保留原有0.02恢复系数。 */
#define BASELINE_RECOVERY_ALPHA_Q16 (1311u)
#endif

/* 显式按状态选择跟随速度，未列出的状态一律冻结。 */
void BaselineTracker_Update(const baseline_tracker_input_t *input, uint32_t *baseline_q16) {
	uint32_t alpha_q16;
	/* 尊重状态机当帧冻结请求，并跳过初始化、重校准和切档抑制。 */
	if (!input->calibrated || input->suppress_activity || input->hold_requested) {
		return;
	}
	switch (input->channel_state) {
	case SENSOR_ALGO_STATE_IDLE:
		/* 普通空闲以小系数跟随长期漂移。 */
		alpha_q16 = BASELINE_IDLE_ALPHA_Q16;
		break;
	case SENSOR_ALGO_STATE_RECOVER:
		/* RECOVER已由状态机确认稳定，使用快速恢复系数。 */
		alpha_q16 = BASELINE_RECOVERY_ALPHA_Q16;
		break;
	case SENSOR_ALGO_STATE_ZERO_REBASE:
		/* 保留零点重建的正向单帧保护，避免吸收新的按压。 */
		if (input->raw_delta > 2) {
			return;
		}
		alpha_q16 = BASELINE_RECOVERY_ALPHA_Q16;
		break;
	default:
		/* WAIT_STABLE、按压和SLOW_BACK均保持基线不变。 */
		return;
	}
/* 矩阵按实测帧率缩放跟随系数，保持原160帧食指的近似时间常数。 */
#if DEVICE_MATRIX20
	alpha_q16 = alpha_q16 * 160u / SENSOR_ALGO_FRAME_RATE_HZ;
	/* 极慢采样时最多直接跟随当前Raw，避免系数超过一而产生过冲。 */
	if (alpha_q16 > BASELINE_Q16_ONE) {
		alpha_q16 = BASELINE_Q16_ONE;
	}
#endif
	/* 有符号差值支持上下两个方向，结果始终位于旧基线与Raw之间。 */
	int64_t delta = ((int64_t)input->raw << 16u) - *baseline_q16;
	*baseline_q16 = (uint32_t)((int64_t)*baseline_q16 + delta * alpha_q16 / BASELINE_Q16_ONE);
}
