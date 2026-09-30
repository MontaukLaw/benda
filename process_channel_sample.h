/* 防止头文件被重复包含。 */
#ifndef PROCESS_CHANNEL_SAMPLE_H

/* 定义头文件包含保护宏。 */
#define PROCESS_CHANNEL_SAMPLE_H

/* 引入布尔类型定义。 */
#include <stdbool.h>

/* 引入固定宽度整数类型定义。 */
#include <stdint.h>

/* 引入传感器算法输出类型定义。 */
#include "sensor_algorithm.h"

/* 定义通道采样处理所依赖的算法操作集合。 */
typedef struct {
	/* 对 Q16 定点数执行一次 IIR 滤波。 */
	uint32_t (*apply_iir_q16)(uint32_t current_q16, uint32_t target_q16, uint32_t alpha_q16);

	/* 处理指定通道的一次重新校准采样。 */
	bool (*process_channel_recalibration_sample)(uint32_t index, uint32_t raw, uint32_t raw_q16);

	/* 检测指定通道是否发生负向回弹释放。 */
	bool (*detect_negative_rebound_release)(uint32_t index, uint32_t raw, bool suppress_activity);

	/* 计算指定通道原始采样值相对基线的正向差值。 */
	uint32_t (*positive_diff_from_baseline)(uint32_t index, uint32_t raw);

	/* 更新指定通道在释放后的动态门限。 */
	void (*update_post_release_dynamic_gate)(uint32_t index, uint32_t positive_diff, int32_t raw_delta, bool suppress_activity);

	/* 对指定通道的正向差值应用输入噪声门限。 */
	uint32_t (*apply_input_noise_gate)(uint32_t index, uint32_t positive_diff, uint32_t multiplier);

	/* 根据当前采样状态更新指定通道的信号强度，并返回快速释放状态。 */
	uint16_t (*update_channel_signal)(uint32_t index, uint32_t positive_diff, uint32_t raw_q16, int32_t raw_delta, bool suppress_activity,
								  bool *fast_release_active);

	/* 根据当前 Raw 和有效正向差值更新通道状态，并允许专用状态覆盖本帧 Signal。 */
	void (*update_output_state)(uint32_t index, uint16_t raw, uint32_t positive_diff, bool suppress_activity, uint16_t *signal);

	/* 统一状态机提交状态后，仅执行一次对应的基线跟随。 */
	void (*update_baseline)(uint32_t index, uint16_t raw, int32_t raw_delta, bool suppress_activity);

	/* 将 Q16 定点值转换为无符号 16 位整数。 */
	uint16_t (*q16_to_u16)(uint32_t value_q16);
/* 结束算法操作集合定义。 */
} process_channel_sample_ops_t;

/* 定义处理单次通道采样所需的上下文数据。 */
typedef struct {
	/* 指向各通道 Q16 格式基线值数组。 */
	uint32_t *baseline_q16;

	/* 指向各通道上一原始采样值数组。 */
	uint16_t *previous_raw_values;

	/* 指向各通道初始化状态数组。 */
	bool *initialized;

	/* 指向各通道重新校准状态数组。 */
	bool *channel_recalibrating;

	/* 指向全局校准完成状态。 */
	const bool *calibrated;

	/* 指向启动静默阶段的累计帧数。 */
	const uint16_t *startup_quiet_frame_count;

	/* 保存启动静默阶段所需的总帧数。 */
	uint16_t startup_quiet_frames;

	/* 保存全局校准阶段使用的 Q16 IIR 滤波系数。 */
	uint32_t calibration_alpha_q16;

	/* 保存输入噪声门限的倍率。 */
	uint32_t signal_input_gate_multiplier;

	/* 指向通道采样处理所使用的算法操作集合。 */
	const process_channel_sample_ops_t *ops;
/* 结束通道采样处理上下文定义。 */
} process_channel_sample_context_t;

/* 声明指定通道的单次采样处理函数。 */
void process_channel_sample(const process_channel_sample_context_t *context, uint32_t index, uint16_t raw, sensor_algorithm_output_t *output);

/* 结束头文件包含保护。 */
#endif
