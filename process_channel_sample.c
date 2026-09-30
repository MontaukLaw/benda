/* 引入通道采样处理模块的接口定义。 */
#include "process_channel_sample.h"

/* 处理指定通道的一次原始采样，并生成对应的算法输出。 */
void process_channel_sample(const process_channel_sample_context_t *context, uint32_t index, uint16_t raw, sensor_algorithm_output_t *output) {
	/* 获取本次处理所需的算法操作接口。 */
	const process_channel_sample_ops_t *ops = context->ops;

	/* 将原始采样值转换为 Q16 定点格式。 */
	uint32_t raw_q16 = (uint32_t)raw << 16u;

	/* 保存经过输入噪声门限处理后的正向差值。 */
	uint32_t positive_diff;

	/* 保存原始采样值相对基线的正向差值。 */
	uint32_t raw_positive_diff;

	/* 计算当前采样相对上一采样的变化量，首次采样时变化量取零。 */
	int32_t raw_delta = context->initialized[index] ? ((int32_t)raw - (int32_t)context->previous_raw_values[index]) : 0;

	/* 保存当前通道滤波后的信号强度。 */
	uint16_t signal;

	/* 标记当前采样是否需要抑制活动检测。 */
	bool suppress_activity = false;

	/* 标记当前通道是否触发快速释放。 */
	bool fast_release_active = false;

	/* 保存负向回弹释放检测结果。 */
	bool negative_rebound_release;

	/* 检查当前通道是否尚未完成首次初始化。 */
	if (!context->initialized[index]) {
		/* 使用首次采样值初始化通道基线。 */
		context->baseline_q16[index] = raw_q16;

		/* 将当前通道标记为已初始化。 */
		context->initialized[index] = true;
	}

	/* 在全局校准完成前持续更新校准数据。 */
	if (!*context->calibrated) {
		/* 判断当前采样是否仍处于启动静默阶段。 */
		if (*context->startup_quiet_frame_count < context->startup_quiet_frames) {
			/* 静默阶段直接使用当前采样刷新基线。 */
			context->baseline_q16[index] = raw_q16;
		} else {
			/* 静默阶段结束后通过 IIR 滤波平滑更新基线。 */
			context->baseline_q16[index] =
				/* 对旧基线和当前采样执行 Q16 定点 IIR 滤波。 */
				ops->apply_iir_q16(context->baseline_q16[index], raw_q16, context->calibration_alpha_q16);
		}
	/* 全局校准完成后，优先处理正在执行的单通道重新校准。 */
	} else if (context->channel_recalibrating[index]) {
		/* 重新校准期间抑制该通道的活动输出。 */
		suppress_activity = true;

		/* 处理本次通道重新校准采样，并显式忽略其返回值。 */
		(void)ops->process_channel_recalibration_sample(index, raw, raw_q16);
	}

	/* 检测当前采样是否满足负向回弹释放条件。 */
	negative_rebound_release = ops->detect_negative_rebound_release(index, raw, suppress_activity);

	/* 负向回弹时将正向差值清零，否则计算采样值相对基线的正向偏移。 */
	raw_positive_diff = negative_rebound_release ? 0u : ops->positive_diff_from_baseline(index, raw);

	/* 根据释放后的采样变化更新动态门限。 */
	ops->update_post_release_dynamic_gate(index, raw_positive_diff, raw_delta, suppress_activity);

	/* 应用输入噪声门限，得到参与信号计算的有效正向差值。 */
	positive_diff = ops->apply_input_noise_gate(index, raw_positive_diff, context->signal_input_gate_multiplier);

	/* 更新通道信号，并同步获取快速释放状态。 */
	signal = ops->update_channel_signal(index, positive_diff, raw_q16, raw_delta, suppress_activity, &fast_release_active);

	/* 使用当前 Raw 和有效正向差值更新状态，并允许恢复或锁定状态覆盖本帧 Signal。 */
	ops->update_output_state(index, raw, positive_diff, suppress_activity, &signal);

	/* 本帧状态先提交，随后按该真实状态执行对应速度的基线更新。 */
	ops->update_baseline(index, raw, raw_delta, suppress_activity);

	/* 将原始采样值写入输出结构。 */
	output->raw = raw;

	/* 将 Q16 格式的通道基线转换为无符号 16 位整数后写入输出。 */
	output->baseline = ops->q16_to_u16(context->baseline_q16[index]);

	/* 将计算得到的信号强度写入输出结构。 */
	output->signal = signal;

	/* Debug和I2C上报实际用于算法判断的固定噪声值。 */
	output->noise = SENSOR_ALGO_FIXED_NOISE;

	/* 保存当前原始采样值，供下一次计算采样变化量。 */
	context->previous_raw_values[index] = raw;
}
