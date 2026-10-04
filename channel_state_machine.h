/* 防止通道状态机头文件被重复包含。 */
#ifndef CHANNEL_STATE_MACHINE_H
/* 定义通道状态机头文件包含保护宏。 */
#define CHANNEL_STATE_MACHINE_H

/* 引入标准布尔类型定义。 */
#include <stdbool.h>
/* 引入固定宽度整数类型定义。 */
#include <stdint.h>

/* 引入传感器算法状态枚举定义。 */
#include "sensor_algorithm.h"

/* 定义约 100 ms 快速 Raw 回归窗口的采样点数。 */
/* 历史数组按最大帧率预留容量，实际窗口由运行时采样率确定。 */
#define CHANNEL_STATE_FAST_WINDOW_SAMPLES ((DEVICE_ALGORITHM_FRAME_RATE_HZ + 9u) / 10u)
/* 定义约 1 s 慢速 Raw 回归窗口的采样点数。 */
#define CHANNEL_STATE_SLOW_WINDOW_SAMPLES DEVICE_ALGORITHM_FRAME_RATE_HZ

/* 保存单个通道状态机的全部运行数据。 */
typedef struct {
	/* 保存上一帧Raw，供统一状态机判断恢复前的稳定性。 */
	uint16_t previous_raw;
	/* WAIT_STABLE连续稳定计数由通道状态机唯一维护。 */
	uint16_t baseline_stable_count;
	/* 保存快速窗口中的 Raw 历史采样值。 */
	uint16_t fast_raw_history[CHANNEL_STATE_FAST_WINDOW_SAMPLES];
	/* 保存慢速窗口中的 Raw 历史采样值。 */
	uint16_t slow_raw_history[CHANNEL_STATE_SLOW_WINDOW_SAMPLES];
	/* 保存快速窗口当前有效采样点数量。 */
	uint16_t fast_sample_count;
	/* 保存慢速窗口当前有效采样点数量。 */
	uint16_t slow_sample_count;
	/* 保存快速循环缓冲区的下一写入位置。 */
	uint16_t fast_write_index;
	/* 保存慢速循环缓冲区的下一写入位置。 */
	uint16_t slow_write_index;
	/* 保存本轮受力零点；恢复中再次按压时采用不低于永久基线的局部低点。 */
	uint16_t activation_baseline;
	/* 保存本次活动期间观察到的最大压力差值。 */
	uint16_t peak_pressure;
	/* 保存 SLOW_BACK 期间用于检测重新按压的局部 Raw 低点。 */
	uint16_t slow_back_reference_raw;
	/* 保存按压后恢复阶段用于识别真正再次按压的局部 Raw 低点。 */
	uint16_t recovery_reference_raw;
	/* 保存 SLOW_BACK 接近 Baseline 且 Raw 稳定的连续帧数。 */
	uint16_t slow_back_stable_count;
	/* 保存当前状态已经持续的采样帧数，供状态超时判断使用。 */
	uint32_t state_elapsed_frames;
	/* 保存本轮有效按压已经持续的总帧数，不随普通活动状态切换清零。 */
	uint32_t active_elapsed_frames;
	/* 保存普通状态候选值连续出现的帧数。 */
	uint8_t pending_count;
	/* 保存普通活动状态返回 IDLE 所使用的确认帧数。 */
	uint8_t idle_confirm_count;
	/* 保存分段松手检测已连续满足低残余压力条件的帧数。 */
	uint8_t release_confirm_count;
	/* 保存 Fast K 和 Slow K 遥测降频计数。 */
	uint8_t telemetry_update_count;
	/* 标记 Fast K 是否已经得到有效计算结果。 */
	bool fast_normalized_k_valid;
	/* 标记 Slow K 是否已经得到有效计算结果。 */
	bool slow_normalized_k_valid;
	/* 本帧检测到未确认按压时请求冻结基线，每帧重新计算，不保存独立阶段。 */
	bool baseline_hold_requested;
	/* 保存当前已经提交的通道状态。 */
	sensor_algorithm_state_t current_state;
	/* 保存等待消抖确认的候选状态。 */
	sensor_algorithm_state_t pending_state;
	/* 保存快速窗口内 Raw 值的累加和。 */
	uint64_t fast_sum;
	/* 保存慢速窗口内 Raw 值的累加和。 */
	uint64_t slow_sum;
	/* 保存快速窗口内横坐标乘 Raw 值的加权和。 */
	uint64_t fast_weighted_sum;
	/* 保存慢速窗口内横坐标乘 Raw 值的加权和。 */
	uint64_t slow_weighted_sum;
	/* 保存快速窗口线性回归斜率的分子。 */
	int64_t fast_slope_numerator;
	/* 保存慢速窗口线性回归斜率的分子。 */
	int64_t slow_slope_numerator;
	/* 保存单位为 0.01 %/s 的快速归一化 K。 */
	int32_t fast_normalized_k_x100;
	/* 保存单位为 0.01 %/s 的慢速归一化 K。 */
	int32_t slow_normalized_k_x100;
#if DEVICE_MATRIX20
	/* 当前真实状态的累计微秒数，由统一提交入口复位。 */
	uint32_t state_elapsed_us;
	/* 保存最近快速下降证据的年龄，允许回落比例稍后达到松手门槛。 */
	uint16_t release_edge_age_frames;
	/* 下降证据只在当前活动周期短暂有效，不构成独立通道状态。 */
	bool release_edge_valid;
	/* 恢复周期保留受力前参考，低谷回弹不能单独作为再次按压依据。 */
	uint16_t recovery_anchor_raw;
	/* 当前恢复周期的再按压幅度门槛，进入恢复时按释放前峰值确定。 */
	uint16_t recovery_retrigger_delta;
	/* 恢复再按压候选的连续微秒数，只由统一提交入口确认或清除。 */
	uint32_t retrigger_confirm_us;
	/* 保存本轮明确加载前的可信平台，松手下冲不得降低此恢复参考。 */
	uint16_t loading_reference_raw;
	/* 保存活动保持期间最近一次窗口稳定的平台Raw，供下一次快加载读取。 */
	uint16_t active_plateau_raw;
	/* 平台仅在本轮稳定保持后有效，下降或恢复入口清除旧平台。 */
	bool active_plateau_valid;
	/* 连续慢下降确认后保留本轮证据，明确再加载或进入恢复时清除。 */
	uint16_t slow_release_confirm_count;
#endif
/* 结束通道状态机运行数据定义。 */
} channel_state_machine_t;

/* 定义状态机处理单帧采样所需的输入数据。 */
typedef struct {
	/* 保存当前通道的 Raw 采样值。 */
	uint16_t raw;
	/* 保存当前通道的 Baseline。 */
	uint16_t baseline;
	/* 保留接口兼容字段；当前判断统一使用SENSOR_ALGO_FIXED_NOISE。 */
	uint16_t noise;
	/* 标记全局校准是否已经完成。 */
	bool calibrated;
	/* 标记本帧是否需要抑制活动判断。 */
	bool suppress_activity;
	/* 标记自动量程切档过渡期间需要保持当前状态。 */
	bool autorange_transition_active;
	/* 标记自动量程过渡结束后需要重建斜率窗口。 */
	bool autorange_transition_finished;
/* 结束状态机单帧输入数据定义。 */
} channel_state_input_t;

/* 初始化指定通道的状态机上下文。 */
void ChannelState_Init(channel_state_machine_t *machine, uint16_t raw, uint16_t baseline);
/* 复位指定通道的状态机上下文。 */
void ChannelState_Reset(channel_state_machine_t *machine, uint16_t raw, uint16_t baseline);
/* 输入一帧通道数据并返回已经提交的通道状态。 */
sensor_algorithm_state_t ChannelState_Update(channel_state_machine_t *machine, const channel_state_input_t *input);
/* 获取指定通道状态机的当前状态。 */
sensor_algorithm_state_t ChannelState_Get(const channel_state_machine_t *machine);
/* 供Signal读取状态机已确认的本轮零点，禁止在输出层另建受力参考。 */
uint16_t ChannelState_GetActivationBaseline(const channel_state_machine_t *machine);
/* 读取状态机本帧给出的按压候选冻结请求，不改变真实通道状态。 */
bool ChannelState_ShouldHoldBaseline(const channel_state_machine_t *machine);
/* 统一判断无有效接触的空闲、等待和基线恢复状态。 */
bool ChannelState_IsIdleLike(sensor_algorithm_state_t state);
/* 获取单位为 0.01 %/s 的 Fast K。 */
int32_t ChannelState_GetFastNormalizedKX100(const channel_state_machine_t *machine);
/* 获取单位为 0.01 %/s 的 Slow K。 */
int32_t ChannelState_GetSlowNormalizedKX100(const channel_state_machine_t *machine);

/* 结束通道状态机头文件包含保护。 */
#endif
