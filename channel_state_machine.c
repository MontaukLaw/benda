/* 引入通道状态机的数据类型、窗口长度和公共接口。 */
#include "channel_state_machine.h"

/* 引入 memset() 所需的标准字符串处理声明。 */
#include <string.h>

/* 二十点矩阵只在初始化或重新校准时改变窗口长度，随后统一复位各通道。 */
#if DEVICE_MATRIX20
static uint32_t matrix_sample_rate_hz = DEVICE_ALGORITHM_FRAME_RATE_HZ;
/* 保存真实完整帧周期，仅用于持续时间，窗口长度仍在校准时锁定。 */
static uint32_t matrix_frame_period_us = 1000000u / DEVICE_ALGORITHM_FRAME_RATE_HZ;
/* 主循环每帧提供实际周期，运行负载变化不得延长恢复超时。 */
void ChannelState_SetFramePeriodUs(uint32_t frame_period_us) {
	if (frame_period_us > 0u) {
		matrix_frame_period_us = frame_period_us;
	}
}
/* 返回最近一次测得并锁定的完整采样周期对应帧率。 */
uint32_t ChannelState_GetSampleRateHz(void) {
	return matrix_sample_rate_hz;
}
/* 限制窗口不超过静态数组容量，并保证快速回归至少包含两个采样。 */
void ChannelState_SetSampleRateHz(uint32_t sample_rate_hz) {
	matrix_sample_rate_hz = sample_rate_hz < 2u ? 2u : sample_rate_hz;
	if (matrix_sample_rate_hz > DEVICE_ALGORITHM_FRAME_RATE_HZ) {
		matrix_sample_rate_hz = DEVICE_ALGORITHM_FRAME_RATE_HZ;
	}
	/* 独立算法回放与首次测量前使用锁定帧率对应周期。 */
	matrix_frame_period_us = (1000000u + matrix_sample_rate_hz / 2u) / matrix_sample_rate_hz;
}
/* 数组容量不变，实际回归窗口按实测帧率对应约100毫秒和1秒。 */
#undef CHANNEL_STATE_FAST_WINDOW_SAMPLES
#undef CHANNEL_STATE_SLOW_WINDOW_SAMPLES
#define CHANNEL_STATE_FAST_WINDOW_SAMPLES ((SENSOR_ALGO_FRAME_RATE_HZ < 20u) ? 2u : SENSOR_ALGO_MILLISECONDS_TO_FRAMES(100u))
#define CHANNEL_STATE_SLOW_WINDOW_SAMPLES SENSOR_ALGO_SECONDS_TO_FRAMES(1u)
#endif

/* 使用当前手指的真实采样率，保证回归斜率和时间门槛一致。 */
#define CHANNEL_STATE_SAMPLE_RATE_HZ SENSOR_ALGO_FRAME_RATE_HZ
/* 定义普通状态候选值需要连续出现的确认帧数。 */
#define CHANNEL_STATE_DEBOUNCE_FRAMES (3u)
/* 定义返回 IDLE 前需要连续满足空闲条件的帧数。 */
/* 矩阵维持约100毫秒确认时长，其他变体保留原16帧确认。 */
#if DEVICE_MATRIX20
#define CHANNEL_STATE_IDLE_CONFIRM_FRAMES SENSOR_ALGO_MILLISECONDS_TO_FRAMES(100u)
#else
#define CHANNEL_STATE_IDLE_CONFIRM_FRAMES (16u)
#endif
/* 保留原基线恢复的80帧等待和2 counts相邻变化门槛。 */
/* 矩阵窗口已确认稳定后再等待250毫秒，避免重复长时间冻结。 */
#if DEVICE_MATRIX20
#define CHANNEL_STATE_BASELINE_STABLE_FRAMES SENSOR_ALGO_MILLISECONDS_TO_FRAMES(250u)
#else
#define CHANNEL_STATE_BASELINE_STABLE_FRAMES (80u)
#endif
#define CHANNEL_STATE_BASELINE_STABLE_DELTA (2)
/* 限制按压后稳定等待时长，持续噪声不能让Baseline永久冻结。 */
#define CHANNEL_STATE_WAIT_STABLE_MAX_FRAMES (CHANNEL_STATE_BASELINE_STABLE_FRAMES * 4u)
/* 快速恢复完成后返回普通空闲跟随，矩阵单独容忍量化误差。 */
#if DEVICE_MATRIX20
/* 矩阵允许3 counts量化误差结束快速恢复，避免基线追逐单点噪声。 */
#define CHANNEL_STATE_BASELINE_RECOVERED_ERROR (3u)
#else
#define CHANNEL_STATE_BASELINE_RECOVERED_ERROR (1u)
#endif
/* 矩阵采用40/32 counts迟滞，过滤日志中短暂负向回弹后的微小正残差。 */
#if DEVICE_MATRIX20
#define CHANNEL_STATE_PRESSURE_ON_MIN (40u)
#define CHANNEL_STATE_PRESSURE_OFF_MIN (32u)
#else
/* 其他手指保留原有激活和释放门槛。 */
#define CHANNEL_STATE_PRESSURE_ON_MIN (12u)
#define CHANNEL_STATE_PRESSURE_OFF_MIN (10u)
#endif
/* 定义归一化压力分母允许使用的最小值。 */
#define CHANNEL_STATE_NORMALIZATION_MIN (20u)
/* 定义激活压力阈值相对于噪声估计值的倍数。 */
#define CHANNEL_STATE_PRESSURE_ON_NOISE_MULTIPLIER (3u)
/* 定义空闲压力阈值相对于噪声估计值的倍数。 */
#define CHANNEL_STATE_PRESSURE_OFF_NOISE_MULTIPLIER (2u)
/* 定义归一化压力下限相对于噪声估计值的倍数。 */
#define CHANNEL_STATE_NORMALIZATION_NOISE_MULTIPLIER (10u)
/* 定义 IDLE 进入 RISING 所需的未归一化 Fast Raw 斜率经验初值，单位为 count/s。 */
#define CHANNEL_STATE_RISING_RAW_COUNTS_PER_SECOND_MIN (20)
/* 定义 SLOW_BACK 重新触发按压所需的未归一化 Fast Raw 斜率经验初值，单位为 count/s。 */
#define CHANNEL_STATE_SLOW_BACK_RETRIGGER_RAW_COUNTS_PER_SECOND_MIN (80)
/* 定义 SLOW_BACK 退出前允许的最大 Fast Raw 斜率绝对值，单位为 count/s。 */
#define CHANNEL_STATE_SLOW_BACK_STABLE_RAW_COUNTS_PER_SECOND_MAX (20)
/* 矩阵稳定确认缩短至250毫秒，其他变体保留400毫秒。 */
#if DEVICE_MATRIX20
#define CHANNEL_STATE_SLOW_BACK_STABLE_FRAMES SENSOR_ALGO_MILLISECONDS_TO_FRAMES(250u)
/* 已确认松手最多等待8秒自然回弹，超时后仍须稳定才能重建零点。 */
#define CHANNEL_STATE_SLOW_BACK_TIMEOUT_US (8000000u)
#else
#define CHANNEL_STATE_SLOW_BACK_STABLE_FRAMES SENSOR_ALGO_MILLISECONDS_TO_FRAMES(400u)
#endif
/* 定义 SLOW_BACK 自然回弹等待上限为 30 秒。 */
#define CHANNEL_STATE_SLOW_BACK_TIMEOUT_FRAMES SENSOR_ALGO_SECONDS_TO_FRAMES(30u)
/* 弱信号进入 ACTIVE_FLOATING 前至少持续激活约一秒，保留轻压抗量化抖动能力。 */
#if DEVICE_HAS_FLOATING_LOCK
#define CHANNEL_STATE_FLOATING_MIN_ACTIVE_FRAMES SENSOR_ALGO_SECONDS_TO_FRAMES(1u)
/* 中高信号至少持续受力四秒再开始锁定确认，为分段加载和末段小步补压预留时间。 */
#define CHANNEL_STATE_FLOATING_LOADED_MIN_ACTIVE_FRAMES SENSOR_ALGO_SECONDS_TO_FRAMES(4u)
/* 中高幅度加载的快慢斜率须连续稳定约300毫秒，避免分段加载短暂停顿提前锁定。 */
#define CHANNEL_STATE_FLOATING_ENTRY_CONFIRM_FRAMES SENSOR_ALGO_MILLISECONDS_TO_FRAMES(300u)
/* 定义 FLOATING 退出到 RISING 的专用 Fast K 阈值为 +80.00 %/s。 */
#define CHANNEL_STATE_FLOATING_EXIT_RISING_FAST_PERCENT_PER_SECOND_X100 (8000)
/* 定义 FLOATING 退出到 FALLING 的专用 Fast K 阈值为 -100.00 %/s。 */
#define CHANNEL_STATE_FLOATING_EXIT_FALLING_FAST_PERCENT_PER_SECOND_X100 (-10000)
/* 定义 FLOATING 退出到普通活动状态前需要连续确认约 50 ms。 */
#define CHANNEL_STATE_FLOATING_EXIT_CONFIRM_FRAMES SENSOR_ALGO_MILLISECONDS_TO_FRAMES(50u)
#endif
/* 定义进入上升状态的 Fast K 阈值为 +20.00 %/s。 */
#define CHANNEL_STATE_RISING_FAST_PERCENT_PER_SECOND_X100 (2000)
/* 定义快速窗口进入下降状态的阈值为 -30.00 %/s。 */
#define CHANNEL_STATE_FALLING_FAST_PERCENT_PER_SECOND_X100 (-3000)
/* 定义慢速窗口进入下降状态的阈值为 -8.00 %/s。 */
#define CHANNEL_STATE_FALLING_SLOW_PERCENT_PER_SECOND_X100 (-800)
/* 大压力松手保留原快速下降门槛，矩阵小压力在释放判定中独立放宽。 */
#define CHANNEL_STATE_RELEASE_TRIGGER_PERCENT_PER_SECOND_X100 (-25000)
/* 定义快速窗口被认为稳定时允许的最大绝对斜率。 */
#define CHANNEL_STATE_FAST_STABLE_PERCENT_PER_SECOND_X100 (1500)
/* 定义慢速窗口被认为稳定时允许的最大绝对斜率。 */
#define CHANNEL_STATE_SLOW_STABLE_PERCENT_PER_SECOND_X100 (500)
/* 定义进入 ACTIVE_FLOATING 时允许的专用 Slow K 最大绝对斜率为 10.00 %/s。 */
#if DEVICE_HAS_FLOATING_LOCK
#define CHANNEL_STATE_FLOATING_SLOW_STABLE_PERCENT_PER_SECOND_X100 (1000)
#endif
/* 定义松手事件相对本次压力峰值至少需要下降的百分比。 */
#define CHANNEL_STATE_RELEASE_TRIGGER_MIN_DROP_PERCENT (15u)
/* 松手小压力边界固定为300 counts，避免降低噪声后破坏轻压保持。 */
#define CHANNEL_STATE_RELEASE_SMALL_LIMIT_COUNTS (300u)
/* 松手大压力边界固定为1000 counts，分段不再依赖噪声或归一化分母。 */
#define CHANNEL_STATE_RELEASE_LARGE_LIMIT_COUNTS (1000u)
/* 矩阵小压力回落35%后确认释放，配合100毫秒去抖过滤瞬态；其他手指保持原门槛。 */
#if DEVICE_MATRIX20
#define CHANNEL_STATE_RELEASE_SMALL_DROP_PERCENT (35u)
#define CHANNEL_STATE_MATRIX_RELEASE_SMALL_FAST_X100 (-5000)
#define CHANNEL_STATE_RELEASE_SMALL_CONFIRM_FRAMES SENSOR_ALGO_MILLISECONDS_TO_FRAMES(100u)
#else
#define CHANNEL_STATE_RELEASE_SMALL_DROP_PERCENT (65u)
#define CHANNEL_STATE_RELEASE_SMALL_CONFIRM_FRAMES SENSOR_ALGO_MILLISECONDS_TO_FRAMES(50u)
#endif
/* 矩阵允许充分慢释放后保留最多四分之一峰值残余，再经稳定确认恢复零点。 */
#if DEVICE_MATRIX20
#define CHANNEL_STATE_SETTLED_PEAK_PERCENT (25u)
/* 慢释放必须持续下降且至少卸载一半，不能只因长时间保持而清零。 */
#define CHANNEL_STATE_SLOW_RELEASE_REMAIN_PERCENT (50u)
/* 残余上的重新按压已扣除旧负载，慢释放采用小压力相同的35%回落门槛。 */
#define CHANNEL_STATE_RETRIGGER_SLOW_REMAIN_PERCENT (65u)
/* 慢窗口达到矩阵下降门槛后连续确认200毫秒，兼顾缓慢卸载和量化噪声。 */
#define CHANNEL_STATE_SLOW_RELEASE_CONFIRM_FRAMES SENSOR_ALGO_MILLISECONDS_TO_FRAMES(200u)
/* 已明显下降四分之一的FALLING保持真实下降状态，直到再加载或释放完成。 */
#define CHANNEL_STATE_FALLING_REMAIN_PERCENT (75u)
/* 慢回落采用-2%/s并保留幅度与卸载比例保护，兼容材料缓慢释放。 */
#define CHANNEL_STATE_MATRIX_SLOW_FALLING_X100 (-200)
/* 已释放后的稳定允许20 counts/s慢漂及短窗24 counts噪声范围。 */
#define CHANNEL_STATE_MATRIX_SETTLED_RAW_COUNTS_PER_SECOND_MAX (20)
#define CHANNEL_STATE_MATRIX_SETTLED_RAW_RANGE_MAX (24u)
/* 恢复可跟随缓慢尾漂，但回到IDLE仍要求5 counts/s，防止尾漂重新触发按压。 */
#define CHANNEL_STATE_MATRIX_IDLE_RAW_COUNTS_PER_SECOND_MAX (5)
/* 矩阵首次按压优先要求200 counts/s快沿，避免慢漂叠加短窗噪声重新锁定。 */
#define CHANNEL_STATE_MATRIX_PRESS_FAST_RAW_MIN (200)
/* 缓慢真实加载可由完整慢窗40 counts/s确认，保留慢按压入口。 */
#define CHANNEL_STATE_MATRIX_PRESS_SLOW_RAW_MIN (40)
/* 恢复再按压采用独立幅度迟滞，过滤松手后约150 counts的正向回弹。 */
#define CHANNEL_STATE_MATRIX_RETRIGGER_DELTA_MIN (200u)
/* 大幅受力后的回弹按峰值5%加严，但限制为400 counts以保留再次按压。 */
#define CHANNEL_STATE_MATRIX_RETRIGGER_PEAK_PERCENT (5u)
#define CHANNEL_STATE_MATRIX_RETRIGGER_DELTA_MAX (400u)
/* 恢复再按压要求明确快沿，普通首次按压仍使用原来的快慢入口。 */
#define CHANNEL_STATE_MATRIX_RETRIGGER_FAST_RAW_MIN (500)
/* 恢复候选必须连续成立80毫秒，使用真实帧周期而非锁定FPS换算。 */
#define CHANNEL_STATE_MATRIX_RETRIGGER_CONFIRM_US (80000u)
#else
/* 其他手指保留原有近零比例。 */
#define CHANNEL_STATE_SETTLED_PEAK_PERCENT (12u)
#endif
/* 根据窗口点数计算横坐标和，兼容160 Hz和250 Hz手指变体。 */
#define CHANNEL_STATE_SUM_X(n) ((1LL * (n) * ((n) - 1u)) / 2LL)
/* 根据窗口点数计算回归分母N平方乘以N平方减一再除以12。 */
#define CHANNEL_STATE_REGRESSION_DENOMINATOR(n) ((1LL * (n) * (n) * (1LL * (n) * (n) - 1LL)) / 12LL)
/* 保存快速窗口的横坐标和与回归分母。 */
#define CHANNEL_STATE_FAST_SUM_X CHANNEL_STATE_SUM_X(CHANNEL_STATE_FAST_WINDOW_SAMPLES)
#define CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR CHANNEL_STATE_REGRESSION_DENOMINATOR(CHANNEL_STATE_FAST_WINDOW_SAMPLES)
/* 保存慢速窗口的横坐标和与回归分母。 */
#define CHANNEL_STATE_SLOW_SUM_X CHANNEL_STATE_SUM_X(CHANNEL_STATE_SLOW_WINDOW_SAMPLES)
#define CHANNEL_STATE_SLOW_REGRESSION_DENOMINATOR CHANNEL_STATE_REGRESSION_DENOMINATOR(CHANNEL_STATE_SLOW_WINDOW_SAMPLES)
/* 将归一化结果放大 10000 倍，以整数表示 0.01 %/s。 */
#define CHANNEL_STATE_NORMALIZED_SCALE (10000LL)
/* 定义 Fast K 和 Slow K 遥测数据每 8 帧更新一次。 */
#define CHANNEL_STATE_TELEMETRY_UPDATE_DIVIDER (8u)

/* 前置声明清除快慢归一化K遥测的内部函数。 */
static void clear_normalized_k_telemetry(channel_state_machine_t *machine);

/* 根据 Raw 和 Baseline 计算非负的压力差值。 */
static uint16_t pressure_from_baseline(uint16_t raw, uint16_t baseline) {
	/* Raw 高于 Baseline 时返回差值，否则将压力钳制为零。 */
	return (raw > baseline) ? (uint16_t)(raw - baseline) : 0u;
/* 结束压力差值计算函数。 */
}

/* 返回两个无符号 16 位整数中的较大值。 */
static uint16_t maximum_u16(uint16_t left, uint16_t right) {
	/* 使用条件表达式选择较大的输入值。 */
	return (left > right) ? left : right;
/* 结束无符号整数最大值函数。 */
}

#if DEVICE_MATRIX20
/* 再激活判定与提交零点共用同一可信参考，下冲不得在提交时降低零点。 */
static uint16_t matrix_retrigger_reference(const channel_state_machine_t *machine,
	uint16_t baseline, uint16_t local_reference) {
	return maximum_u16(maximum_u16(baseline, machine->recovery_anchor_raw), local_reference);
}
#endif

/* 向指定长度的滑动窗口追加一个 Raw 采样点。 */
static void append_window_sample(
	/* 指向保存历史采样值的循环缓冲区。 */
	uint16_t *history,
	/* 指定滑动窗口能够容纳的采样点数。 */
	uint16_t capacity,
	/* 传入本帧需要加入窗口的 Raw 值。 */
	uint16_t raw,
	/* 指向窗口当前有效采样点数量。 */
	uint16_t *sample_count,
	/* 指向循环缓冲区下一次写入的位置。 */
	uint16_t *write_index,
	/* 指向窗口内所有 Raw 值的累加和。 */
	uint64_t *sum,
	/* 指向窗口内横坐标乘 Raw 值的加权和。 */
	uint64_t *weighted_sum) {
	/* 窗口尚未填满时按照初始顺序加入新采样点。 */
	if (*sample_count < capacity) {
		/* 当前有效点数就是新采样点的横坐标位置。 */
		uint16_t position = *sample_count;
		/* 将新 Raw 值写入当前循环缓冲区位置。 */
		history[*write_index] = raw;
		/* 将新 Raw 值加入窗口普通和。 */
		*sum += raw;
		/* 将横坐标乘 Raw 的结果加入窗口加权和。 */
		*weighted_sum += (uint64_t)position * raw;
		/* 将窗口有效采样点数量增加一。 */
		(*sample_count)++;
	/* 窗口已经填满时移除最旧点并加入最新点。 */
	} else {
		/* 读取本帧即将被覆盖的最旧 Raw 值。 */
		uint16_t oldest = history[*write_index];
		/* 平移原有横坐标权重，并把新 Raw 加入窗口末端。 */
		*weighted_sum = *weighted_sum - (*sum - oldest) + ((uint64_t)(capacity - 1u) * raw);
		/* 从普通和中移除最旧值并加入新 Raw 值。 */
		*sum = *sum - oldest + raw;
		/* 使用新 Raw 值覆盖循环缓冲区中的最旧值。 */
		history[*write_index] = raw;
	/* 结束窗口填充状态分支。 */
	}

	/* 将循环缓冲区写入位置移动到下一项。 */
	(*write_index)++;
	/* 写入位置到达容量上限时执行循环回绕。 */
	if (*write_index >= capacity) {
		/* 将下一次写入位置重置为缓冲区起点。 */
		*write_index = 0u;
	/* 结束循环缓冲区回绕判断。 */
	}
/* 结束滑动窗口采样追加函数。 */
}

/* 将本帧 Raw 同时加入快速和慢速窗口，并更新回归分子。 */
static void append_raw(channel_state_machine_t *machine, uint16_t raw) {
	/* 更新 16 点快速 Raw 滑动窗口。 */
	append_window_sample(
		/* 传入快速窗口历史缓冲区。 */
		machine->fast_raw_history,
		/* 传入快速窗口容量。 */
		CHANNEL_STATE_FAST_WINDOW_SAMPLES,
		/* 传入本帧 Raw 值。 */
		raw,
		/* 传入快速窗口有效点数。 */
		&machine->fast_sample_count,
		/* 传入快速窗口循环写入位置。 */
		&machine->fast_write_index,
		/* 传入快速窗口普通和。 */
		&machine->fast_sum,
		/* 传入快速窗口加权和。 */
		&machine->fast_weighted_sum);

	/* 更新 160 点慢速 Raw 滑动窗口。 */
	append_window_sample(
		/* 传入慢速窗口历史缓冲区。 */
		machine->slow_raw_history,
		/* 传入慢速窗口容量。 */
		CHANNEL_STATE_SLOW_WINDOW_SAMPLES,
		/* 传入本帧 Raw 值。 */
		raw,
		/* 传入慢速窗口有效点数。 */
		&machine->slow_sample_count,
		/* 传入慢速窗口循环写入位置。 */
		&machine->slow_write_index,
		/* 传入慢速窗口普通和。 */
		&machine->slow_sum,
		/* 传入慢速窗口加权和。 */
		&machine->slow_weighted_sum);

	/* 快速窗口填满后计算线性回归斜率分子，否则使用零。 */
	machine->fast_slope_numerator = (machine->fast_sample_count >= CHANNEL_STATE_FAST_WINDOW_SAMPLES)
		/* 使用 N×Σxy 作为回归分子的第一项。 */
		? ((int64_t)CHANNEL_STATE_FAST_WINDOW_SAMPLES * (int64_t)machine->fast_weighted_sum) -
			/* 减去 Σx×Σy，得到完整的快速窗口回归分子。 */
			(CHANNEL_STATE_FAST_SUM_X * (int64_t)machine->fast_sum)
		/* 快速窗口尚未填满时斜率分子保持为零。 */
		: 0;

	/* 慢速窗口填满后计算线性回归斜率分子，否则使用零。 */
	machine->slow_slope_numerator = (machine->slow_sample_count >= CHANNEL_STATE_SLOW_WINDOW_SAMPLES)
		/* 使用 N×Σxy 作为回归分子的第一项。 */
		? ((int64_t)CHANNEL_STATE_SLOW_WINDOW_SAMPLES * (int64_t)machine->slow_weighted_sum) -
			/* 减去 Σx×Σy，得到完整的慢速窗口回归分子。 */
			(CHANNEL_STATE_SLOW_SUM_X * (int64_t)machine->slow_sum)
		/* 慢速窗口尚未填满时斜率分子保持为零。 */
		: 0;
/* 结束 Raw 窗口与回归分子更新函数。 */
}

/* 使用当前Raw重新建立快慢斜率窗口，清除切档合成轨迹的历史影响。 */
static void reset_slope_windows(channel_state_machine_t *machine, uint16_t raw) {
	/* 清空快速和慢速Raw历史缓冲区。 */
	memset(machine->fast_raw_history, 0, sizeof(machine->fast_raw_history));
	memset(machine->slow_raw_history, 0, sizeof(machine->slow_raw_history));

	/* 清空两个窗口的有效样本数和循环写入位置。 */
	machine->fast_sample_count = 0u;
	machine->slow_sample_count = 0u;
	machine->fast_write_index = 0u;
	machine->slow_write_index = 0u;

	/* 清空两个窗口的普通和、加权和以及回归斜率。 */
	machine->fast_sum = 0u;
	machine->slow_sum = 0u;
	machine->fast_weighted_sum = 0u;
	machine->slow_weighted_sum = 0u;
	machine->fast_slope_numerator = 0;
	machine->slow_slope_numerator = 0;

	/* 清除旧斜率遥测值，等待新窗口重新形成有效结果。 */
	clear_normalized_k_telemetry(machine);

	/* 以过渡结束后的当前Raw作为新窗口首个真实样本。 */
	append_raw(machine, raw);
/* 结束斜率窗口重建函数。 */
}

/* 将回归分子换算为按秒计并放大 10000 倍的中间量。 */
static int64_t normalized_slope_scaled(int64_t slope_numerator) {
	/* 乘采样率完成每采样点到每秒的换算，再乘整数缩放倍数。 */
	return slope_numerator * CHANNEL_STATE_SAMPLE_RATE_HZ * CHANNEL_STATE_NORMALIZED_SCALE;
/* 结束归一化斜率中间量计算函数。 */
}

/* 判断未归一化 Raw 斜率是否达到指定的 count/s 下限。 */
static bool raw_slope_at_least(
	/* 传入线性回归斜率分子。 */
	int64_t slope_numerator,
	/* 传入对应窗口的固定回归分母。 */
	int64_t regression_denominator,
	/* 传入单位为 count/s 的斜率下限。 */
	int32_t threshold_counts_per_second) {
	/* 将回归斜率分子乘以采样率，换算为按秒计的比较量。 */
	int64_t slope_per_second_scaled = slope_numerator * CHANNEL_STATE_SAMPLE_RATE_HZ;
	/* 将 count/s 阈值乘以回归分母，构造无除法的等价比较量。 */
	int64_t threshold_scaled = (int64_t)threshold_counts_per_second * regression_denominator;
	/* 返回未归一化 Raw 斜率是否达到指定下限。 */
	return slope_per_second_scaled >= threshold_scaled;
/* 结束未归一化 Raw 斜率下限判断函数。 */
}

/* 判断未归一化 Raw 斜率的绝对值是否位于稳定范围。 */
static bool raw_slope_is_stable(
	/* 传入线性回归斜率分子。 */
	int64_t slope_numerator,
	/* 传入对应窗口的固定回归分母。 */
	int64_t regression_denominator,
	/* 传入单位为 count/s 的斜率绝对值上限。 */
	int32_t limit_counts_per_second) {
	/* 将回归斜率分子乘以采样率，换算为按秒计的比较量。 */
	int64_t slope_per_second_scaled = slope_numerator * CHANNEL_STATE_SAMPLE_RATE_HZ;
	/* 将稳定阈值乘以回归分母，构造无除法的等价比较量。 */
	int64_t limit_scaled = (int64_t)limit_counts_per_second * regression_denominator;
	/* 负斜率需要先转换成绝对值。 */
	if (slope_per_second_scaled < 0) {
		/* 对负斜率取反得到绝对值。 */
		slope_per_second_scaled = -slope_per_second_scaled;
	/* 结束负斜率绝对值处理分支。 */
	}
	/* Raw 斜率绝对值不超过限制时认为快速窗口稳定。 */
	return slope_per_second_scaled <= limit_scaled;
/* 结束未归一化 Raw 斜率稳定性判断函数。 */
}

/* 判断归一化斜率是否大于或等于指定阈值。 */
static bool normalized_slope_at_least(
	/* 传入线性回归斜率分子。 */
	int64_t slope_numerator,
	/* 传入对应窗口的固定回归分母。 */
	int64_t regression_denominator,
	/* 传入压力归一化参考值。 */
	uint16_t reference_pressure,
	/* 传入单位为 0.01 %/s 的判断阈值。 */
	int32_t threshold_x100) {
	/* 将阈值乘回两个分母，避免整数除法造成精度损失。 */
	int64_t threshold_scaled = (int64_t)threshold_x100 * regression_denominator * reference_pressure;
	/* 返回归一化斜率是否达到下限阈值。 */
	return normalized_slope_scaled(slope_numerator) >= threshold_scaled;
/* 结束归一化斜率下限判断函数。 */
}

/* 判断归一化斜率是否小于或等于指定阈值。 */
static bool normalized_slope_at_most(
	/* 传入线性回归斜率分子。 */
	int64_t slope_numerator,
	/* 传入对应窗口的固定回归分母。 */
	int64_t regression_denominator,
	/* 传入压力归一化参考值。 */
	uint16_t reference_pressure,
	/* 传入单位为 0.01 %/s 的判断阈值。 */
	int32_t threshold_x100) {
	/* 将阈值乘回两个分母，得到等价整数比较值。 */
	int64_t threshold_scaled = (int64_t)threshold_x100 * regression_denominator * reference_pressure;
	/* 返回归一化斜率是否达到上限阈值。 */
	return normalized_slope_scaled(slope_numerator) <= threshold_scaled;
/* 结束归一化斜率上限判断函数。 */
}

/* 判断归一化斜率的绝对值是否位于稳定范围。 */
static bool normalized_slope_is_stable(
	/* 传入线性回归斜率分子。 */
	int64_t slope_numerator,
	/* 传入对应窗口的固定回归分母。 */
	int64_t regression_denominator,
	/* 传入压力归一化参考值。 */
	uint16_t reference_pressure,
	/* 传入单位为 0.01 %/s 的稳定范围上限。 */
	int32_t limit_x100) {
	/* 计算按秒换算并完成整数放大的斜率中间量。 */
	int64_t slope_scaled = normalized_slope_scaled(slope_numerator);
	/* 将稳定阈值乘回两个分母，得到等价整数比较值。 */
	int64_t limit_scaled = (int64_t)limit_x100 * regression_denominator * reference_pressure;
	/* 负斜率需要先转换成绝对值。 */
	if (slope_scaled < 0) {
		/* 对负斜率取反得到绝对值。 */
		slope_scaled = -slope_scaled;
	/* 结束负斜率处理分支。 */
	}
	/* 斜率绝对值不超过限制时认为窗口稳定。 */
	return slope_scaled <= limit_scaled;
/* 结束归一化斜率稳定性判断函数。 */
}

/* 计算单位为 0.01 %/s 的归一化斜率。 */
static int32_t normalized_slope_x100(
	/* 传入线性回归斜率分子。 */
	int64_t slope_numerator,
	/* 传入对应窗口的固定回归分母。 */
	int64_t regression_denominator,
	/* 传入压力归一化参考值。 */
	uint16_t reference_pressure) {
	/* 合并线性回归分母和压力归一化分母。 */
	int64_t denominator = regression_denominator * reference_pressure;
	/* 分母有效时完成计算，否则返回零以避免除零。 */
	return (denominator > 0) ? (int32_t)(normalized_slope_scaled(slope_numerator) / denominator) : 0;
/* 结束归一化斜率数值计算函数。 */
}

/* 按设定周期更新用于通信和显示的 Fast K 与 Slow K。 */
static void update_normalized_k_telemetry(channel_state_machine_t *machine, uint16_t reference_pressure) {
	/* 计数器为零表示本帧需要进行周期更新。 */
	bool periodic_update = machine->telemetry_update_count == 0u;

	/* 快速窗口有效且满足周期更新或首次更新条件时计算 Fast K。 */
	if ((machine->fast_sample_count >= CHANNEL_STATE_FAST_WINDOW_SAMPLES) &&
		/* 首次有效时不等待下一次周期。 */
		(periodic_update || !machine->fast_normalized_k_valid)) {
		/* 使用快速窗口回归数据计算归一化 Fast K。 */
		machine->fast_normalized_k_x100 = normalized_slope_x100(
			/* 传入快速窗口斜率分子。 */
			machine->fast_slope_numerator,
			/* 传入快速窗口固定回归分母。 */
			CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR,
			/* 传入本帧压力归一化参考值。 */
			reference_pressure);
		/* 标记 Fast K 已经具有有效结果。 */
		machine->fast_normalized_k_valid = true;
	/* 结束 Fast K 遥测更新分支。 */
	}

	/* 慢速窗口有效且满足周期更新或首次更新条件时计算 Slow K。 */
	if ((machine->slow_sample_count >= CHANNEL_STATE_SLOW_WINDOW_SAMPLES) &&
		/* 首次有效时不等待下一次周期。 */
		(periodic_update || !machine->slow_normalized_k_valid)) {
		/* 使用慢速窗口回归数据计算归一化 Slow K。 */
		machine->slow_normalized_k_x100 = normalized_slope_x100(
			/* 传入慢速窗口斜率分子。 */
			machine->slow_slope_numerator,
			/* 传入慢速窗口固定回归分母。 */
			CHANNEL_STATE_SLOW_REGRESSION_DENOMINATOR,
			/* 传入本帧压力归一化参考值。 */
			reference_pressure);
		/* 标记 Slow K 已经具有有效结果。 */
		machine->slow_normalized_k_valid = true;
	/* 结束 Slow K 遥测更新分支。 */
	}

	/* 每处理一帧就推进一次遥测更新计数器。 */
	machine->telemetry_update_count++;
	/* 计数器达到更新周期时准备开始下一轮计数。 */
	if (machine->telemetry_update_count >= CHANNEL_STATE_TELEMETRY_UPDATE_DIVIDER) {
		/* 清零计数器，使下一帧重新计算遥测 K。 */
		machine->telemetry_update_count = 0u;
	/* 结束遥测更新计数器回绕判断。 */
	}
/* 结束归一化 K 遥测更新函数。 */
}

/* 清除对外输出的归一化 Fast K 和 Slow K。 */
static void clear_normalized_k_telemetry(channel_state_machine_t *machine) {
	/* 将 Fast K 标记为无效，使下次激活后立即重新计算。 */
	machine->fast_normalized_k_valid = false;
	/* 将 Slow K 标记为无效，使下次激活后立即重新计算。 */
	machine->slow_normalized_k_valid = false;
	/* 空闲、等待和恢复阶段对外输出的 Fast K 固定为零。 */
	machine->fast_normalized_k_x100 = 0;
	/* 空闲、等待和恢复阶段对外输出的 Slow K 固定为零。 */
	machine->slow_normalized_k_x100 = 0;
	/* 清零降频计数器，使进入激活状态的第一帧立即更新 K。 */
	machine->telemetry_update_count = 0u;
/* 结束归一化 K 清零函数。 */
}

/* 这是本模块中唯一允许写入 current_state 的函数。 */
static void commit_state(channel_state_machine_t *machine, sensor_algorithm_state_t state, uint16_t raw, uint16_t baseline) {
	/* 保存当前状态切换需要满足的连续确认帧数。 */
	uint16_t required_frames = CHANNEL_STATE_DEBOUNCE_FRAMES;
#if DEVICE_MATRIX20
	/* 专用确认仅覆盖四个恢复状态到真实上升状态的切换。 */
	bool recovery_retrigger = (state == SENSOR_ALGO_STATE_ACTIVE_RISING) &&
		((machine->current_state == SENSOR_ALGO_STATE_SLOW_BACK) ||
		 (machine->current_state == SENSOR_ALGO_STATE_ZERO_REBASE) ||
		 (machine->current_state == SENSOR_ALGO_STATE_WAIT_STABLE) ||
		 (machine->current_state == SENSOR_ALGO_STATE_RECOVER));
#endif

	/* 候选状态与当前状态相同时不需要切换。 */
	if (state == machine->current_state) {
		/* 将待确认状态同步为当前状态。 */
		machine->pending_state = state;
		/* 清除不再需要的候选状态累计帧数。 */
		machine->pending_count = 0u;
#if DEVICE_MATRIX20
		/* 候选撤销后不得把不连续的回弹片段累计成再次按压。 */
		machine->retrigger_confirm_us = 0u;
#endif
		/* 提前结束本次状态提交。 */
		return;
	/* 结束候选状态未改变分支。 */
	}

	/* 释放、零点重建及三个无接触阶段均在当帧立即提交。 */
	if ((state == SENSOR_ALGO_STATE_SLOW_BACK) ||
		/* 零点重建也需要在超时当帧立即生效。 */
		(state == SENSOR_ALGO_STATE_ZERO_REBASE) ||
		/* 返回空闲前已经由各状态完成专用确认。 */
		ChannelState_IsIdleLike(state)) {
		/* 立即将候选状态写入当前状态。 */
#if DEVICE_MATRIX20
		/* 从空闲扰动或真实活动进入恢复时保存参考，恢复阶段内切换不得改成低谷。 */
		if (((state == SENSOR_ALGO_STATE_WAIT_STABLE) || (state == SENSOR_ALGO_STATE_SLOW_BACK)) &&
			((machine->current_state == SENSOR_ALGO_STATE_IDLE) ||
			 (!ChannelState_IsIdleLike(machine->current_state) &&
			  machine->current_state != SENSOR_ALGO_STATE_SLOW_BACK &&
			  machine->current_state != SENSOR_ALGO_STATE_ZERO_REBASE))) {
			/* 空闲扰动采用当前零点；已受力松手同时保护最近一次快加载前的稳定平台。 */
			machine->recovery_anchor_raw = (machine->current_state == SENSOR_ALGO_STATE_IDLE)
				? machine->activation_baseline
				: maximum_u16(machine->activation_baseline, machine->loading_reference_raw);
			/* 进入新恢复周期才重算幅度门槛，恢复状态互转不得丢失释放前峰值。 */
			uint32_t delta = ((uint32_t)machine->peak_pressure * CHANNEL_STATE_MATRIX_RETRIGGER_PEAK_PERCENT) / 100u;
			if (delta < CHANNEL_STATE_MATRIX_RETRIGGER_DELTA_MIN) delta = CHANNEL_STATE_MATRIX_RETRIGGER_DELTA_MIN;
			if (delta > CHANNEL_STATE_MATRIX_RETRIGGER_DELTA_MAX) delta = CHANNEL_STATE_MATRIX_RETRIGGER_DELTA_MAX;
			machine->recovery_retrigger_delta = (uint16_t)delta;
		}
#endif
		/* 所有恢复状态仍只在本入口提交。 */
		machine->current_state = state;
		/* 进入按压后稳定等待时，从当前Raw建立再次按压的局部参考。 */
		if (state == SENSOR_ALGO_STATE_WAIT_STABLE) {
			machine->recovery_reference_raw = raw;
		}
		/* 恢复或空闲状态不继承尚未完成的松手确认。 */
		machine->release_confirm_count = 0u;
#if DEVICE_MATRIX20
		/* 进入释放或空闲恢复阶段时清除上一轮下降证据，禁止跨按压复用。 */
		machine->release_edge_valid = false;
		machine->release_edge_age_frames = 0u;
		/* 恢复与空闲入口统一清除本轮慢释放及锁定参考。 */
		machine->slow_release_confirm_count = 0u;
		/* 恢复入口不继承上一轮活动平台，恢复锚点由上方统一保存。 */
		machine->active_plateau_valid = false;
#endif
		/* 进入慢回弹时统一建立局部低点和稳定确认起点。 */
		if (state == SENSOR_ALGO_STATE_SLOW_BACK) {
			machine->slow_back_reference_raw = raw;
			machine->slow_back_stable_count = 0u;
			machine->idle_confirm_count = 0u;
		}
		/* 每次进入新的空闲/恢复阶段，都重新开始稳定确认。 */
		machine->baseline_stable_count = 0u;
		/* 新状态从零开始累计持续时间。 */
		machine->state_elapsed_frames = 0u;
#if DEVICE_MATRIX20
		/* 每次真实切换重新开始按实际帧周期计时。 */
		machine->state_elapsed_us = 0u;
		/* 进入任何新的恢复或空闲状态均撤销旧再按压候选。 */
		machine->retrigger_confirm_us = 0u;
		/* 超时进入零点重建的首帧先冻结，下一帧仍须由稳定判定允许跟随。 */
		if (state == SENSOR_ALGO_STATE_ZERO_REBASE) {
			machine->baseline_hold_requested = true;
		}
#endif
		/* 将待确认状态同步为已经提交的新状态。 */
		machine->pending_state = state;
		/* 清除普通状态切换使用的消抖计数器。 */
		machine->pending_count = 0u;
		/* 提前结束本次状态提交。 */
		return;
	/* 结束立即切换状态分支。 */
	}

	/* 本轮峰值超过小压力分界时延长锁定确认，弱信号保留三帧确认以兼容量化抖动。 */
#if DEVICE_HAS_FLOATING_LOCK
	if ((state == SENSOR_ALGO_STATE_ACTIVE_FLOATING) &&
		/* 沿用300 counts的小压力边界，用本轮峰值避免临界Raw噪声反复切换确认时长。 */
		(machine->peak_pressure > CHANNEL_STATE_RELEASE_SMALL_LIMIT_COUNTS)) {
		/* 将连续稳定确认与本轮激活满一秒分开，避免首次短平台过早锁定。 */
		required_frames = CHANNEL_STATE_FLOATING_ENTRY_CONFIRM_FRAMES;
	/* FLOATING退出仍使用原有约50毫秒确认，保持轻力抗量化抖动能力。 */
	} else if ((machine->current_state == SENSOR_ALGO_STATE_ACTIVE_FLOATING) &&
		/* 只延长 FLOATING 到两个普通变化状态的确认时间。 */
		((state == SENSOR_ALGO_STATE_ACTIVE_RISING) ||
			/* 将下降候选状态也纳入 FLOATING 专用退出消抖。 */
			(state == SENSOR_ALGO_STATE_ACTIVE_FALLING))) {
		/* 使用约 50 ms 的连续确认抑制低压力通道的量化抖动。 */
		required_frames = CHANNEL_STATE_FLOATING_EXIT_CONFIRM_FRAMES;
	/* 结束 FLOATING 专用退出确认帧数选择。 */
	}
#endif

	/* 新候选状态与上一帧候选状态不同时重新开始计数。 */
	if (state != machine->pending_state) {
		/* 记录本帧检测到的新候选状态。 */
		machine->pending_state = state;
		/* 将本帧记为候选状态连续出现的第一帧。 */
		machine->pending_count = 1u;
#if DEVICE_MATRIX20
		/* 新候选从本帧真实周期开始计时，不沿用旧候选的时间。 */
		machine->retrigger_confirm_us = recovery_retrigger ? matrix_frame_period_us : 0u;
#endif
		/* 等待后续帧继续确认，本帧不切换状态。 */
		return;
	/* 结束新候选状态初始化分支。 */
	}

	/* 候选状态持续存在且尚未达到确认帧数时继续累加。 */
	if (machine->pending_count < required_frames) {
		/* 将候选状态连续出现帧数增加一。 */
		machine->pending_count++;
	/* 结束候选状态计数递增分支。 */
	}
#if DEVICE_MATRIX20
	/* 连续恢复再按压候选采用饱和微秒累计，确认失败不改变真实恢复状态。 */
	if (recovery_retrigger) {
		if (UINT32_MAX - machine->retrigger_confirm_us < matrix_frame_period_us) {
			machine->retrigger_confirm_us = UINT32_MAX;
		} else {
			machine->retrigger_confirm_us += matrix_frame_period_us;
		}
		if (machine->retrigger_confirm_us < CHANNEL_STATE_MATRIX_RETRIGGER_CONFIRM_US) return;
	}
#endif
	/* 候选状态连续达到设定帧数及专用时间后正式切换。 */
	if (machine->pending_count >= required_frames) {
#if DEVICE_MATRIX20
		/* 初次按压以真实空闲基线建立加载参考，不继承旧平台。 */
		if ((state == SENSOR_ALGO_STATE_ACTIVE_RISING) &&
			(machine->current_state == SENSOR_ALGO_STATE_IDLE)) {
			machine->loading_reference_raw = baseline;
		}
		/* 稳定保持后的明确快加载保存原平台，随后松手回到此平台不构成新按压。 */
		if ((state == SENSOR_ALGO_STATE_ACTIVE_RISING) &&
			(machine->current_state == SENSOR_ALGO_STATE_ACTIVE_HOLD) &&
			machine->active_plateau_valid &&
			raw_slope_at_least(machine->fast_slope_numerator, CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR,
				CHANNEL_STATE_MATRIX_RETRIGGER_FAST_RAW_MIN)) {
			machine->loading_reference_raw = maximum_u16(machine->activation_baseline, machine->active_plateau_raw);
		}
		/* 恢复后的新按压由提交入口一次性建立零点、峰值和斜率历史。 */
		if ((state == SENSOR_ALGO_STATE_ACTIVE_RISING) &&
			((machine->current_state == SENSOR_ALGO_STATE_SLOW_BACK) ||
			 (machine->current_state == SENSOR_ALGO_STATE_ZERO_REBASE) ||
			 (machine->current_state == SENSOR_ALGO_STATE_WAIT_STABLE) ||
			 (machine->current_state == SENSOR_ALGO_STATE_RECOVER))) {
			/* 残余形变只作零点，不再进入新一轮压力峰值和K分母。 */
			uint16_t local_reference = ((machine->current_state == SENSOR_ALGO_STATE_SLOW_BACK) ||
				(machine->current_state == SENSOR_ALGO_STATE_ZERO_REBASE))
				? machine->slow_back_reference_raw : machine->recovery_reference_raw;
			/* 判定与进入处理采用同一参考，不能丢掉已保留的恢复锚点。 */
			machine->activation_baseline = matrix_retrigger_reference(machine, baseline, local_reference);
			/* 新的真实按压重新建立本轮加载参考。 */
			machine->loading_reference_raw = machine->activation_baseline;
			machine->peak_pressure = pressure_from_baseline(raw, machine->activation_baseline);
			machine->active_elapsed_frames = 0u;
			machine->idle_confirm_count = 0u;
			machine->release_confirm_count = 0u;
			machine->release_edge_valid = false;
			machine->release_edge_age_frames = 0u;
			machine->slow_release_confirm_count = 0u;
			reset_slope_windows(machine, raw);
		}
		/* 上升或下降开始后须重新形成稳定保持平台，禁止复用旧负载平台。 */
		if ((state == SENSOR_ALGO_STATE_ACTIVE_RISING) || (state == SENSOR_ALGO_STATE_ACTIVE_FALLING)) {
			machine->active_plateau_valid = false;
		}
#else
		/* 非矩阵变体继续使用原有受力参考处理。 */
		(void)baseline;
#endif
		/* 将已经确认的候选状态写入当前状态。 */
		machine->current_state = state;
		/* 普通状态正式切换后重新开始累计状态持续时间。 */
		machine->state_elapsed_frames = 0u;
#if DEVICE_MATRIX20
		/* 普通活动状态切换也统一复位实际持续时间。 */
		machine->state_elapsed_us = 0u;
		/* 已提交的新状态不保留本次再按压确认时间。 */
		machine->retrigger_confirm_us = 0u;
#endif
		/* 状态切换完成后清零候选状态计数器。 */
		machine->pending_count = 0u;
	/* 结束普通状态切换确认分支。 */
	}
/* 结束统一状态提交函数。 */
}

/* 判断当前通道是否发生足以进入 SLOW_BACK 的松手事件。 */
static bool release_trigger_detected(
	/* 传入允许累计松手确认帧数的状态机上下文。 */
	channel_state_machine_t *machine,
	/* 传入当前压力差值。 */
	uint16_t pressure,
	/* 传入当前压力归一化参考值。 */
	uint16_t normalization_reference) {
	/* 直接采用固定压力边界，不再从噪声倍数推导松手分段。 */
	uint32_t small_limit = CHANNEL_STATE_RELEASE_SMALL_LIMIT_COUNTS;
	/* 保持已验证的大压力松手边界。 */
	uint32_t large_limit = CHANNEL_STATE_RELEASE_LARGE_LIMIT_COUNTS;
	/* 默认使用大压力原有的15%回落与一帧快速响应。 */
	uint32_t drop_percent = CHANNEL_STATE_RELEASE_TRIGGER_MIN_DROP_PERCENT;
	uint32_t confirm_frames = 1u;
	/* 仅矩阵小压力放宽至-50%/s，中压力连续过渡到原有大压力门槛。 */
	int32_t trigger_threshold = CHANNEL_STATE_RELEASE_TRIGGER_PERCENT_PER_SECOND_X100;
#if DEVICE_MATRIX20
	if (machine->peak_pressure < large_limit) {
		uint32_t remaining = large_limit - maximum_u16(machine->peak_pressure, small_limit);
		uint32_t threshold_span = (uint32_t)(CHANNEL_STATE_MATRIX_RELEASE_SMALL_FAST_X100 - trigger_threshold);
		trigger_threshold += (int32_t)(remaining * threshold_span / (large_limit - small_limit));
	}
#endif
	/* 小压力保留足够回落空间，使瞬态峰值之后仍能保持有效接触。 */
	if (machine->peak_pressure <= small_limit) {
		drop_percent = CHANNEL_STATE_RELEASE_SMALL_DROP_PERCENT;
		confirm_frames = CHANNEL_STATE_RELEASE_SMALL_CONFIRM_FRAMES;
	/* 中压力连续插值门槛，避免跨分段边界时突然改变松手灵敏度。 */
	} else if (machine->peak_pressure < large_limit) {
		/* 越靠近小压力段，回落比例和确认时间越接近小压力配置。 */
		uint32_t remaining = large_limit - machine->peak_pressure;
		uint32_t span = large_limit - small_limit;
		drop_percent += (remaining * (CHANNEL_STATE_RELEASE_SMALL_DROP_PERCENT - CHANNEL_STATE_RELEASE_TRIGGER_MIN_DROP_PERCENT) + span - 1u) / span;
		confirm_frames += (remaining * (CHANNEL_STATE_RELEASE_SMALL_CONFIRM_FRAMES - 1u) + span - 1u) / span;
	}
	/* 矩阵保留500毫秒内的下降证据，避免斜率先变平、回落比例后达标时漏掉释放。 */
#if DEVICE_MATRIX20
	if (normalized_slope_at_most(machine->fast_slope_numerator, CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR,
		normalization_reference, trigger_threshold)) {
		machine->release_edge_valid = true;
		machine->release_edge_age_frames = 0u;
	} else if (machine->release_edge_valid) {
		machine->release_edge_age_frames++;
		if (machine->release_edge_age_frames > SENSOR_ALGO_MILLISECONDS_TO_FRAMES(500u)) {
			machine->release_edge_valid = false;
		}
	}
#endif
	/* 压力未充分回落或再次明显上升时撤销候选，不能累积不连续的松手片段。 */
	if ((machine->peak_pressure == 0u) || (pressure >= machine->peak_pressure) ||
		(((uint32_t)(machine->peak_pressure - pressure) * 100u) < ((uint32_t)machine->peak_pressure * drop_percent)) ||
		raw_slope_at_least(machine->fast_slope_numerator, CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR, CHANNEL_STATE_RISING_RAW_COUNTS_PER_SECOND_MIN)) {
		machine->release_confirm_count = 0u;
		return false;
	}
	/* 首帧必须检测到真实快速下降，后续只确认残余持续较低，允许松手后斜率变平。 */
	if ((machine->release_confirm_count == 0u) &&
#if DEVICE_MATRIX20
		/* 已保存的近期下降与当前回落比例共同构成候选，仍须连续确认。 */
		!machine->release_edge_valid) {
#else
		!normalized_slope_at_most(machine->fast_slope_numerator, CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR,
			normalization_reference, trigger_threshold)) {
#endif
		return false;
	}
	/* 仅在分段回落条件持续成立时累计，达到当前压力段的确认时间才清零输出。 */
	if (machine->release_confirm_count < confirm_frames) {
		machine->release_confirm_count++;
	}
	return machine->release_confirm_count >= confirm_frames;
/* 结束松手事件检测函数。 */
}

#if DEVICE_MATRIX20
/* 使用慢窗口斜率和短窗口幅度判断稳定，既识别持续回弹，也容忍少量量化噪声。 */
static bool matrix_raw_is_settled(const channel_state_machine_t *machine) {
	if ((machine->slow_sample_count < CHANNEL_STATE_SLOW_WINDOW_SAMPLES) ||
		!raw_slope_is_stable(machine->slow_slope_numerator, CHANNEL_STATE_SLOW_REGRESSION_DENOMINATOR,
			CHANNEL_STATE_MATRIX_SETTLED_RAW_COUNTS_PER_SECOND_MAX)) {
		return false;
	}
	uint16_t minimum = UINT16_MAX;
	uint16_t maximum = 0u;
	for (uint16_t i = 0u; i < CHANNEL_STATE_FAST_WINDOW_SAMPLES; ++i) {
		uint16_t value = machine->fast_raw_history[i];
		if (value < minimum) minimum = value;
		if (value > maximum) maximum = value;
	}
	return (maximum - minimum) <= CHANNEL_STATE_MATRIX_SETTLED_RAW_RANGE_MAX;
}

/* 恢复中再次按压须同时越过可信基线与局部参考，并具有明确的正向变化。 */
static bool matrix_retrigger_detected(const channel_state_machine_t *machine,
	uint16_t raw, uint16_t baseline, uint16_t local_reference, uint16_t pressure_on) {
	uint16_t reference = matrix_retrigger_reference(machine, baseline, local_reference);
	/* 恢复门槛独立于首次按压，并始终不低于正常激活幅度门槛。 */
	uint16_t delta = maximum_u16(pressure_on, machine->recovery_retrigger_delta);
	return ((uint32_t)raw > (uint32_t)reference + delta) &&
		raw_slope_at_least(machine->fast_slope_numerator, CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR,
			CHANNEL_STATE_MATRIX_RETRIGGER_FAST_RAW_MIN);
}
#endif

/* 根据当前压力、快慢斜率和当前状态计算候选状态。 */
static sensor_algorithm_state_t classify_state(
	/* 传入允许更新空闲确认计数器的状态机上下文。 */
	channel_state_machine_t *machine,
	/* 传入当前 Raw 采样值。 */
	uint16_t raw,
	/* 传入本帧状态判断之前的当前 Baseline。 */
	uint16_t baseline,
	/* 传入当前压力差值。 */
	uint16_t pressure,
	/* 传入激活压力阈值。 */
	uint16_t pressure_on,
	/* 传入空闲压力阈值。 */
	uint16_t pressure_off,
	/* 传入斜率归一化压力参考值。 */
	uint16_t normalization_reference) {

	/* 判断快速窗口斜率绝对值是否位于稳定范围。 */
	bool fast_stable = normalized_slope_is_stable(
		/* 传入快速窗口斜率分子。 */
		machine->fast_slope_numerator,
		/* 传入快速窗口固定回归分母。 */
		CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR,
		/* 传入压力归一化参考值。 */
		normalization_reference,
		/* 传入快速窗口稳定阈值。 */
		CHANNEL_STATE_FAST_STABLE_PERCENT_PER_SECOND_X100);

	/* 慢速窗口未填满时暂按稳定处理，否则检查慢速斜率。 */
	bool slow_stable = (machine->slow_sample_count < CHANNEL_STATE_SLOW_WINDOW_SAMPLES) ||
		/* 使用慢速窗口斜率判断长期变化是否稳定。 */
		normalized_slope_is_stable(
			/* 传入慢速窗口斜率分子。 */
			machine->slow_slope_numerator,
			/* 传入慢速窗口固定回归分母。 */
			CHANNEL_STATE_SLOW_REGRESSION_DENOMINATOR,
			/* 传入压力归一化参考值。 */
			normalization_reference,
			/* 传入慢速窗口稳定阈值。 */
			CHANNEL_STATE_SLOW_STABLE_PERCENT_PER_SECOND_X100);

	/* ACTIVE_FLOATING 使用较宽松的专用 Slow K 稳定范围，以缩短蠕变锁定等待时间。 */
#if DEVICE_HAS_FLOATING_LOCK
	bool floating_slow_stable = (machine->slow_sample_count < CHANNEL_STATE_SLOW_WINDOW_SAMPLES) ||
		/* 使用慢速窗口斜率检查是否进入 FLOATING 专用稳定范围。 */
		normalized_slope_is_stable(
			/* 传入慢速窗口斜率分子。 */
			machine->slow_slope_numerator,
			/* 传入慢速窗口固定回归分母。 */
			CHANNEL_STATE_SLOW_REGRESSION_DENOMINATOR,
			/* 传入压力归一化参考值。 */
			normalization_reference,
			/* 传入 FLOATING 专用 Slow K 稳定阈值。 */
			CHANNEL_STATE_FLOATING_SLOW_STABLE_PERCENT_PER_SECOND_X100);
#endif

	/* 判断当前压力是否已经回落到本次峰值的 12% 以下。 */
	bool settled_after_release = (machine->peak_pressure > 0u) &&
		/* 使用整数交叉相乘完成百分比比较。 */
		(((uint32_t)pressure * 100u) <= ((uint32_t)machine->peak_pressure * CHANNEL_STATE_SETTLED_PEAK_PERCENT));

	/* IDLE内确认首次按压，候选期间只请求冻结基线。 */
	if (machine->current_state == SENSOR_ALGO_STATE_IDLE) {
#if DEVICE_MATRIX20
		/* 明显负向形变进入公开的等待恢复状态，冻结零点直至回弹稳定。 */
		if (((uint32_t)raw + pressure_on < baseline) &&
			/* 空闲负向形变沿用严格尾漂门槛，不能因放宽恢复跟随而污染原零点。 */
			(!matrix_raw_is_settled(machine) || !raw_slope_is_stable(machine->slow_slope_numerator,
				CHANNEL_STATE_SLOW_REGRESSION_DENOMINATOR, CHANNEL_STATE_MATRIX_IDLE_RAW_COUNTS_PER_SECOND_MAX))) {
			return SENSOR_ALGO_STATE_WAIT_STABLE;
		}
#endif
		if (pressure > pressure_on) {
			/* 恢复后回到IDLE也使用明确加载证据，单靠40 counts残差不构成按压。 */
#if DEVICE_MATRIX20
			bool press_detected = raw_slope_at_least(machine->fast_slope_numerator,
				CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR, CHANNEL_STATE_MATRIX_PRESS_FAST_RAW_MIN) ||
				((machine->slow_sample_count >= CHANNEL_STATE_SLOW_WINDOW_SAMPLES) &&
				 raw_slope_at_least(machine->fast_slope_numerator, CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR,
					CHANNEL_STATE_RISING_RAW_COUNTS_PER_SECOND_MIN) &&
				 raw_slope_at_least(machine->slow_slope_numerator, CHANNEL_STATE_SLOW_REGRESSION_DENOMINATOR,
					CHANNEL_STATE_MATRIX_PRESS_SLOW_RAW_MIN));
			if (press_detected) {
#else
			if (raw_slope_at_least(machine->fast_slope_numerator,
				CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR,
				CHANNEL_STATE_RISING_RAW_COUNTS_PER_SECOND_MIN)) {
#endif
				return SENSOR_ALGO_STATE_ACTIVE_RISING;
			}
			/* 尚未确认有效按压时保持IDLE，不启动按压后恢复。 */
			return SENSOR_ALGO_STATE_IDLE;
		}
		return SENSOR_ALGO_STATE_IDLE;
	}

	/* WAIT_STABLE与RECOVER只处理已确认按压后的恢复及真正的再次按压。 */
	if ((machine->current_state == SENSOR_ALGO_STATE_WAIT_STABLE) ||
		(machine->current_state == SENSOR_ALGO_STATE_RECOVER)) {
		/* RECOVER跟随Raw上漂时同步推进局部参考，避免历史低点长期失效。 */
		if ((machine->current_state == SENSOR_ALGO_STATE_RECOVER) &&
			(baseline > machine->recovery_reference_raw) && (baseline <= raw)) {
			machine->recovery_reference_raw = baseline;
		}
		/* Raw继续下降时刷新局部低点，使残余偏移本身不构成再次按压。 */
		if (raw < machine->recovery_reference_raw) {
			machine->recovery_reference_raw = raw;
		}
		/* 再次按压必须相对恢复局部低点产生足够增量，不能沿用冻结基线残差。 */
		uint16_t retrigger_delta = (raw > machine->recovery_reference_raw)
			? (uint16_t)(raw - machine->recovery_reference_raw) : 0u;
		/* 局部增量与快速上升必须同时成立，慢漂不能提前冻结恢复基线。 */
		bool retrigger_candidate = (retrigger_delta > pressure_on) &&
			raw_slope_at_least(machine->fast_slope_numerator,
				CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR,
				CHANNEL_STATE_SLOW_BACK_RETRIGGER_RAW_COUNTS_PER_SECOND_MIN);
#if DEVICE_MATRIX20
		/* 矩阵恢复阶段统一核对受力前参考，不能只比较不断降低的局部低点。 */
		retrigger_candidate = matrix_retrigger_detected(machine, raw, baseline, machine->recovery_reference_raw, pressure_on);
#endif
		/* RECOVER检测到完整再次按压候选后立即暂停跟随，等待三帧确认。 */
		if ((machine->current_state == SENSOR_ALGO_STATE_RECOVER) && retrigger_candidate) {
			machine->baseline_hold_requested = true;
		}
		if (retrigger_candidate) {
			machine->baseline_stable_count = 0u;
			return SENSOR_ALGO_STATE_ACTIVE_RISING;
		}
		/* 恢复完成在本帧切回IDLE，后续基线执行器使用0.005。 */
		uint16_t error = (raw > baseline) ? (raw - baseline) : (baseline - raw);
#if DEVICE_MATRIX20
		/* 回弹未稳定时暂停恢复；只有窗口稳定且基线贴近Raw才能重新进入IDLE。 */
		bool raw_settled = matrix_raw_is_settled(machine);
		/* 跟随恢复与重新开放空闲触发分开，漂移中的近零误差不能提前结束恢复。 */
		bool idle_settled = raw_settled && raw_slope_is_stable(machine->slow_slope_numerator,
			CHANNEL_STATE_SLOW_REGRESSION_DENOMINATOR, CHANNEL_STATE_MATRIX_IDLE_RAW_COUNTS_PER_SECOND_MAX);
		if (machine->current_state == SENSOR_ALGO_STATE_RECOVER) {
			if (!raw_settled) return SENSOR_ALGO_STATE_WAIT_STABLE;
			return (idle_settled && (error <= CHANNEL_STATE_BASELINE_RECOVERED_ERROR))
				? SENSOR_ALGO_STATE_IDLE : SENSOR_ALGO_STATE_RECOVER;
		}
		if (raw_settled) {
			if (machine->baseline_stable_count < CHANNEL_STATE_BASELINE_STABLE_FRAMES) ++machine->baseline_stable_count;
		} else {
			machine->baseline_stable_count = 0u;
		}
		if (machine->baseline_stable_count >= CHANNEL_STATE_BASELINE_STABLE_FRAMES) {
			return (idle_settled && (error <= CHANNEL_STATE_BASELINE_RECOVERED_ERROR))
				? SENSOR_ALGO_STATE_IDLE : SENSOR_ALGO_STATE_RECOVER;
		}
		return SENSOR_ALGO_STATE_WAIT_STABLE;
#else
		if (machine->current_state == SENSOR_ALGO_STATE_RECOVER) {
			return (error <= CHANNEL_STATE_BASELINE_RECOVERED_ERROR)
				? SENSOR_ALGO_STATE_IDLE : SENSOR_ALGO_STATE_RECOVER;
		}
		/* 等待阶段基线冻结，Raw稳定计数不得跨波动累积。 */
		int32_t raw_delta = (int32_t)raw - machine->previous_raw;
		if ((raw_delta >= -CHANNEL_STATE_BASELINE_STABLE_DELTA) &&
			(raw_delta <= CHANNEL_STATE_BASELINE_STABLE_DELTA)) {
			if (machine->baseline_stable_count < CHANNEL_STATE_BASELINE_STABLE_FRAMES) {
				machine->baseline_stable_count++;
			}
		} else {
			machine->baseline_stable_count = 0u;
		}
		/* 满80个稳定帧或达到等待上限后恢复跟随；已经贴近Raw则直接结束等待。 */
		if ((machine->baseline_stable_count >= CHANNEL_STATE_BASELINE_STABLE_FRAMES) ||
			(machine->state_elapsed_frames >= CHANNEL_STATE_WAIT_STABLE_MAX_FRAMES)) {
			return (error <= CHANNEL_STATE_BASELINE_RECOVERED_ERROR)
				? SENSOR_ALGO_STATE_IDLE : SENSOR_ALGO_STATE_RECOVER;
		}
		return SENSOR_ALGO_STATE_WAIT_STABLE;
#endif
	}

	/* 当前处于 SLOW_BACK 时执行慢回弹屏蔽、重新按压和退出判断。 */
	if (machine->current_state == SENSOR_ALGO_STATE_SLOW_BACK) {
#if DEVICE_MATRIX20
		/* 回弹平台已稳定后推进局部参考，禁止长期从历史谷底累计正向残差。 */
		if (matrix_raw_is_settled(machine) && (raw > machine->slow_back_reference_raw)) {
			machine->slow_back_reference_raw = raw;
		}
#endif
		/* Raw 继续下降时同步刷新慢回弹阶段的局部低点。 */
		if (raw < machine->slow_back_reference_raw) {
			/* 将更低的 Raw 保存为重新按压判断参考值。 */
			machine->slow_back_reference_raw = raw;
		/* 结束 SLOW_BACK 局部 Raw 低点更新分支。 */
		}
		/* 计算当前 Raw 相对慢回弹局部低点的正向增量。 */
#if !DEVICE_MATRIX20
		uint16_t retrigger_delta = (raw > machine->slow_back_reference_raw) ? (uint16_t)(raw - machine->slow_back_reference_raw) : 0u;
#endif
		/* 局部增量越过激活阈值且 Fast Raw 正向斜率足够大时判定重新按压。 */
		if (
#if DEVICE_MATRIX20
			/* 矩阵低于原基线的谷底回升不会退出SLOW_BACK。 */
			matrix_retrigger_detected(machine, raw, baseline, machine->slow_back_reference_raw, pressure_on)
#else
			(retrigger_delta > pressure_on) &&
			/* 使用更高的 SLOW_BACK 专用斜率阈值过滤材料回弹和少量噪声。 */
			raw_slope_at_least(
				/* 传入快速窗口斜率分子。 */
				machine->fast_slope_numerator,
				/* 传入快速窗口固定回归分母。 */
				CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR,
				/* 传入 SLOW_BACK 重新按压斜率阈值。 */
				CHANNEL_STATE_SLOW_BACK_RETRIGGER_RAW_COUNTS_PER_SECOND_MIN)
#endif
		) {
			/* 重新按压后取消慢回弹稳定累计。 */
			machine->slow_back_stable_count = 0u;
			/* 返回上升候选状态，仍由统一入口执行三帧消抖。 */
			return SENSOR_ALGO_STATE_ACTIVE_RISING;
		/* 结束 SLOW_BACK 重新按压判断。 */
		}
		/* Raw 已接近冻结 Baseline 且快速窗口稳定时累计退出确认帧数。 */
		if (
#if DEVICE_MATRIX20
			/* 已确认释放后允许稳定残余进入恢复，不再等残余自行降到冻结基线。 */
			matrix_raw_is_settled(machine)
#else
			(pressure <= pressure_off) &&
			/* 使用未归一化 Fast Raw 斜率检查短期稳定性。 */
			raw_slope_is_stable(
				/* 传入快速窗口斜率分子。 */
				machine->fast_slope_numerator,
				/* 传入快速窗口固定回归分母。 */
				CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR,
				/* 传入 SLOW_BACK 退出稳定斜率上限。 */
				CHANNEL_STATE_SLOW_BACK_STABLE_RAW_COUNTS_PER_SECOND_MAX)
#endif
		) {
			/* 稳定累计尚未达到目标帧数时继续累加。 */
			if (machine->slow_back_stable_count < CHANNEL_STATE_SLOW_BACK_STABLE_FRAMES) {
				/* 将连续稳定帧数增加一帧。 */
				machine->slow_back_stable_count++;
			/* 结束 SLOW_BACK 稳定帧数递增分支。 */
			}
		/* Raw 尚未接近 Baseline 或仍在变化时重新开始稳定计数。 */
		} else {
			/* 清零 SLOW_BACK 稳定累计帧数。 */
			machine->slow_back_stable_count = 0u;
		/* 结束 SLOW_BACK 退出稳定条件判断。 */
		}
		/* 连续稳定达到对应变体时长后结束SLOW_BACK，统一入口转为WAIT_STABLE。 */
		if (machine->slow_back_stable_count >= CHANNEL_STATE_SLOW_BACK_STABLE_FRAMES) {
			/* 返回空闲候选状态。 */
			return SENSOR_ALGO_STATE_IDLE;
		/* 结束 SLOW_BACK 返回 IDLE 判断。 */
		}
		/* 自然回弹等待达到对应变体上限时转入受控零点重建。 */
		if (
#if DEVICE_MATRIX20
			/* 矩阵超时按真实周期累计，不能再由开机测得的较高FPS拉长。 */
			machine->state_elapsed_us >= CHANNEL_STATE_SLOW_BACK_TIMEOUT_US
#else
			machine->state_elapsed_frames >= CHANNEL_STATE_SLOW_BACK_TIMEOUT_FRAMES
#endif
		) {
			/* 清零恢复稳定计数，供 ZERO_REBASE 使用自己的退出确认。 */
			machine->slow_back_stable_count = 0u;
			/* 返回零点重建候选状态，统一入口将在本帧立即提交。 */
			return SENSOR_ALGO_STATE_ZERO_REBASE;
		/* 结束 SLOW_BACK 超时判断。 */
		}
		/* 未发生重新按压且尚未完全恢复时持续保持 SLOW_BACK。 */
		return SENSOR_ALGO_STATE_SLOW_BACK;
	/* 结束 SLOW_BACK 专用状态判断。 */
	}

	/* 当前处于 ZERO_REBASE 时优先检测重新按压和零点重建完成条件。 */
	if (machine->current_state == SENSOR_ALGO_STATE_ZERO_REBASE) {
#if DEVICE_MATRIX20
		/* 零点重建与慢回弹使用同一稳定平台参考，旧谷底不能永久成为再按压起点。 */
		if (matrix_raw_is_settled(machine) && (raw > machine->slow_back_reference_raw)) {
			machine->slow_back_reference_raw = raw;
		}
#endif
		/* Raw 继续下降时同步刷新重新按压使用的局部低点。 */
		if (raw < machine->slow_back_reference_raw) {
			/* 保存更低的 Raw 作为重新按压参考值。 */
			machine->slow_back_reference_raw = raw;
		/* 结束 ZERO_REBASE 局部低点更新分支。 */
		}
		/* 计算当前 Raw 相对零点重建阶段局部低点的正向增量。 */
#if !DEVICE_MATRIX20
		uint16_t retrigger_delta = (raw > machine->slow_back_reference_raw) ? (uint16_t)(raw - machine->slow_back_reference_raw) : 0u;
#endif
		/* 局部增量和 Fast Raw 斜率同时满足时判定重新按压。 */
		if (
#if DEVICE_MATRIX20
			/* 零点重建与其他恢复阶段共用同一再次按压门槛。 */
			matrix_retrigger_detected(machine, raw, baseline, machine->slow_back_reference_raw, pressure_on)
#else
			(retrigger_delta > pressure_on) &&
			/* 使用 SLOW_BACK 相同的高斜率门限过滤零点重建过程中的小波动。 */
			raw_slope_at_least(
				/* 传入快速窗口斜率分子。 */
				machine->fast_slope_numerator,
				/* 传入快速窗口固定回归分母。 */
				CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR,
				/* 传入恢复阶段重新按压斜率阈值。 */
				CHANNEL_STATE_SLOW_BACK_RETRIGGER_RAW_COUNTS_PER_SECOND_MIN)
#endif
		) {
			/* 重新按压后取消零点重建退出确认。 */
			machine->slow_back_stable_count = 0u;
			/* 返回 RISING 候选状态并执行普通三帧消抖。 */
			return SENSOR_ALGO_STATE_ACTIVE_RISING;
		/* 结束 ZERO_REBASE 重新按压判断。 */
		}
		/* 计算当前 Raw 与已经开始追随的 Baseline 之间的绝对误差。 */
		uint16_t baseline_error = (raw > baseline) ? (uint16_t)(raw - baseline) : (uint16_t)(baseline - raw);
#if DEVICE_MATRIX20
		/* 超时恢复也不能在回弹中追随，稳定前冻结基线。 */
		if (!matrix_raw_is_settled(machine)) {
			machine->baseline_hold_requested = true;
			machine->slow_back_stable_count = 0u;
			return SENSOR_ALGO_STATE_ZERO_REBASE;
		}
#endif
		/* Baseline 已追到 Raw 附近且快速窗口稳定时累计退出确认。 */
		if ((baseline_error <= pressure_off) &&
#if DEVICE_MATRIX20
			/* 零点重建也必须等尾漂结束后开放IDLE触发，保持与RECOVER一致。 */
			raw_slope_is_stable(machine->slow_slope_numerator, CHANNEL_STATE_SLOW_REGRESSION_DENOMINATOR,
				CHANNEL_STATE_MATRIX_IDLE_RAW_COUNTS_PER_SECOND_MAX) &&
#endif
			/* 使用 Fast Raw 斜率限制避免在重新受压时错误退出。 */
			raw_slope_is_stable(
				/* 传入快速窗口斜率分子。 */
				machine->fast_slope_numerator,
				/* 传入快速窗口固定回归分母。 */
				CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR,
				/* 传入恢复阶段退出稳定斜率上限。 */
				CHANNEL_STATE_SLOW_BACK_STABLE_RAW_COUNTS_PER_SECOND_MAX)) {
			/* 稳定累计尚未达到目标帧数时继续累加。 */
			if (machine->slow_back_stable_count < CHANNEL_STATE_SLOW_BACK_STABLE_FRAMES) {
				/* 将连续稳定帧数增加一帧。 */
				machine->slow_back_stable_count++;
			/* 结束 ZERO_REBASE 稳定帧数递增分支。 */
			}
		/* Baseline 尚未追到 Raw 或 Raw 仍在变化时重新开始确认。 */
		} else {
			/* 清零 ZERO_REBASE 退出稳定计数。 */
			machine->slow_back_stable_count = 0u;
		/* 结束 ZERO_REBASE 退出稳定条件判断。 */
		}
		/* Baseline 与 Raw 连续稳定约 400 ms 后返回 IDLE。 */
		if (machine->slow_back_stable_count >= CHANNEL_STATE_SLOW_BACK_STABLE_FRAMES) {
			/* 返回 IDLE 候选状态，统一入口将在本帧立即提交。 */
			return SENSOR_ALGO_STATE_IDLE;
		/* 结束 ZERO_REBASE 返回 IDLE 判断。 */
		}
		/* 零点尚未重建完成时持续保持 ZERO_REBASE。 */
		return SENSOR_ALGO_STATE_ZERO_REBASE;
	/* 结束 ZERO_REBASE 专用状态判断。 */
	}

	/* 激活状态下优先使用 Fast K 与峰值下降比例检测松手事件。 */
	if (release_trigger_detected(machine, pressure, normalization_reference)) {
		/* 检测成功后立即进入慢回弹屏蔽状态。 */
		return SENSOR_ALGO_STATE_SLOW_BACK;
	/* 结束松手事件优先判断。 */
	}

#if DEVICE_MATRIX20
	/* 慢下降证据与累计卸载幅度分开确认，尾段变平不能抹去本轮真实下降。 */
	bool slow_falling = (machine->slow_sample_count >= CHANNEL_STATE_SLOW_WINDOW_SAMPLES) &&
		normalized_slope_at_most(machine->slow_slope_numerator, CHANNEL_STATE_SLOW_REGRESSION_DENOMINATOR,
			/* 连续输出模式直接使用本轮真实压力峰值，不再维护锁定参考。 */
			normalization_reference,
			CHANNEL_STATE_MATRIX_SLOW_FALLING_X100) &&
		!raw_slope_at_least(machine->fast_slope_numerator, CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR,
			CHANNEL_STATE_RISING_RAW_COUNTS_PER_SECOND_MIN);
	/* 明确再加载撤销旧下降证据，静态长按和量化噪声不能单独累计松手确认。 */
	bool reloading = raw_slope_at_least(machine->fast_slope_numerator, CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR,
		CHANNEL_STATE_RISING_RAW_COUNTS_PER_SECOND_MIN) &&
		normalized_slope_at_least(machine->fast_slope_numerator, CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR,
			normalization_reference, CHANNEL_STATE_RISING_FAST_PERCENT_PER_SECOND_X100);
	if (reloading) {
		machine->slow_release_confirm_count = 0u;
	} else if (slow_falling) {
		if (machine->slow_release_confirm_count < CHANNEL_STATE_SLOW_RELEASE_CONFIRM_FRAMES) {
			machine->slow_release_confirm_count++;
		}
	} else if (machine->slow_release_confirm_count < CHANNEL_STATE_SLOW_RELEASE_CONFIRM_FRAMES) {
		/* 未确认的下降仍需连续，只有完整证据允许等到慢回落幅度达标。 */
		machine->slow_release_confirm_count = 0u;
	}
	/* 本轮零点高于永久基线表明在正残余上重新按压，回落比较只使用新增受力。 */
	uint32_t slow_remain_percent = (machine->activation_baseline > baseline)
		? CHANNEL_STATE_RETRIGGER_SLOW_REMAIN_PERCENT : CHANNEL_STATE_SLOW_RELEASE_REMAIN_PERCENT;
	if (!reloading && (machine->slow_release_confirm_count >= CHANNEL_STATE_SLOW_RELEASE_CONFIRM_FRAMES) &&
		(machine->peak_pressure > 0u) &&
		((uint32_t)pressure * 100u <= (uint32_t)machine->peak_pressure * slow_remain_percent)) {
		return SENSOR_ALGO_STATE_SLOW_BACK;
	}
#endif

	/* 压力足够低，或释放后接近零点且快慢窗口稳定时累计空闲确认。 */
	if ((pressure <= pressure_off) || (settled_after_release && fast_stable && slow_stable)) {
		/* 空闲确认计数尚未达到上限时继续累加。 */
		if (machine->idle_confirm_count < CHANNEL_STATE_IDLE_CONFIRM_FRAMES) {
			/* 将连续满足空闲条件的帧数增加一。 */
			machine->idle_confirm_count++;
		/* 结束空闲确认计数递增分支。 */
		}
		/* 连续空闲确认帧数达到设定值时允许返回 IDLE。 */
		if (machine->idle_confirm_count >= CHANNEL_STATE_IDLE_CONFIRM_FRAMES) {
			/* 返回空闲候选状态。 */
			return SENSOR_ALGO_STATE_IDLE;
		/* 结束空闲确认完成分支。 */
		}
	/* 当前不满足空闲条件时取消此前累计的连续确认。 */
	} else {
		/* 将空闲确认帧数清零。 */
		machine->idle_confirm_count = 0u;
	/* 结束空闲条件判断。 */
	}

	/* FLOATING 使用独立且带迟滞的退出门槛，避免弱压力下的归一化 K 放大量化噪声。 */
#if DEVICE_HAS_FLOATING_LOCK
	if (machine->current_state == SENSOR_ALGO_STATE_ACTIVE_FLOATING) {
		/* Fast K 连续达到专用正向阈值时准备退出到 RISING。 */
		if (normalized_slope_at_least(
			/* 传入快速窗口斜率分子。 */
			machine->fast_slope_numerator,
			/* 传入快速窗口固定回归分母。 */
			CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR,
			/* 传入压力归一化参考值。 */
			normalization_reference,
			/* 传入 FLOATING 专用正向退出阈值。 */
			CHANNEL_STATE_FLOATING_EXIT_RISING_FAST_PERCENT_PER_SECOND_X100)) {
			/* 返回 RISING 候选状态并由统一入口连续确认八帧。 */
			return SENSOR_ALGO_STATE_ACTIVE_RISING;
		/* 结束 FLOATING 正向退出判断。 */
		}

		/* Fast K 连续达到专用负向阈值时准备退出到 FALLING。 */
		if (normalized_slope_at_most(
			/* 传入快速窗口斜率分子。 */
			machine->fast_slope_numerator,
			/* 传入快速窗口固定回归分母。 */
			CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR,
			/* 传入压力归一化参考值。 */
			normalization_reference,
			/* 传入 FLOATING 专用负向退出阈值。 */
			CHANNEL_STATE_FLOATING_EXIT_FALLING_FAST_PERCENT_PER_SECOND_X100)) {
			/* 返回 FALLING 候选状态并由统一入口连续确认八帧。 */
			return SENSOR_ALGO_STATE_ACTIVE_FALLING;
		/* 结束 FLOATING 负向退出判断。 */
		}

		/* 未达到专用退出门槛时继续保持 FLOATING。 */
		return SENSOR_ALGO_STATE_ACTIVE_FLOATING;
	/* 结束 FLOATING 专用迟滞分类分支。 */
	}
#endif

	/* Fast K 达到上升阈值时判定为 ACTIVE_RISING。 */
	if (normalized_slope_at_least(
		/* 传入快速窗口斜率分子。 */
		machine->fast_slope_numerator,
		/* 传入快速窗口固定回归分母。 */
		CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR,
		/* 传入压力归一化参考值。 */
		normalization_reference,
		/* 传入上升状态斜率阈值。 */
		CHANNEL_STATE_RISING_FAST_PERCENT_PER_SECOND_X100)) {
		/* 返回上升候选状态。 */
		return SENSOR_ALGO_STATE_ACTIVE_RISING;
	/* 结束上升状态判断。 */
	}

#if DEVICE_MATRIX20
	/* 持续慢下降保持FALLING，直到再加载或释放完成。 */
	if (slow_falling || ((machine->current_state == SENSOR_ALGO_STATE_ACTIVE_FALLING) &&
		(machine->peak_pressure > 0u) && ((uint32_t)pressure * 100u <=
		 (uint32_t)machine->peak_pressure * CHANNEL_STATE_FALLING_REMAIN_PERCENT))) {
		return SENSOR_ALGO_STATE_ACTIVE_FALLING;
	}
#endif

	/* 沿用本轮峰值的300 counts边界，普通活动状态切换不会重启本轮受力等待。 */
#if DEVICE_HAS_FLOATING_LOCK
	uint32_t floating_min_active_frames = (machine->peak_pressure > CHANNEL_STATE_RELEASE_SMALL_LIMIT_COUNTS)
		? CHANNEL_STATE_FLOATING_LOADED_MIN_ACTIVE_FRAMES : CHANNEL_STATE_FLOATING_MIN_ACTIVE_FRAMES;
	/* 达到对应受力时长后才允许稳定锁定候选，继续优先于普通慢速下降分类。 */
	if ((machine->active_elapsed_frames >= floating_min_active_frames) && fast_stable && floating_slow_stable) {
		/* 仅形成候选，由统一状态入口按本轮压力幅度执行连续稳定确认。 */
		return SENSOR_ALGO_STATE_ACTIVE_FLOATING;
	}
#endif

	/* Fast K 达到快速下降阈值，或 Slow K 达到慢速下降阈值时判定下降。 */
	if (normalized_slope_at_most(
		/* 传入快速窗口斜率分子。 */
		machine->fast_slope_numerator,
		/* 传入快速窗口固定回归分母。 */
		CHANNEL_STATE_FAST_REGRESSION_DENOMINATOR,
		/* 传入压力归一化参考值。 */
		normalization_reference,
		/* 传入快速下降阈值。 */
		CHANNEL_STATE_FALLING_FAST_PERCENT_PER_SECOND_X100) ||
		/* 只有慢速窗口填满后才允许 Slow K 参与下降判断。 */
		((machine->slow_sample_count >= CHANNEL_STATE_SLOW_WINDOW_SAMPLES) &&
			/* 检查慢速窗口斜率是否低于慢速下降阈值。 */
			normalized_slope_at_most(
				/* 传入慢速窗口斜率分子。 */
				machine->slow_slope_numerator,
				/* 传入慢速窗口固定回归分母。 */
				CHANNEL_STATE_SLOW_REGRESSION_DENOMINATOR,
				/* 传入压力归一化参考值。 */
				normalization_reference,
				/* 传入慢速下降阈值。 */
				CHANNEL_STATE_FALLING_SLOW_PERCENT_PER_SECOND_X100))) {
		/* 返回下降候选状态。 */
		return SENSOR_ALGO_STATE_ACTIVE_FALLING;
	/* 结束下降状态判断。 */
	}

	/* 快慢窗口同时稳定时进入HOLD，矩阵Signal仍连续跟随实际Raw差值。 */
	if (fast_stable && slow_stable) {
		/* 返回保持候选状态。 */
		return SENSOR_ALGO_STATE_ACTIVE_HOLD;
	/* 结束保持状态判断。 */
	}

	/* 没有任何新状态条件成立时延续当前状态。 */
#if DEVICE_HAS_FLOATING_LOCK
	return machine->current_state;
#else
	/* 矩阵只返回上升、下降或稳定候选，不保留无法分类的历史活动状态。 */
	return SENSOR_ALGO_STATE_ACTIVE_HOLD;
#endif
/* 结束候选状态分类函数。 */
}

/* 初始化一个通道状态机实例。 */
void ChannelState_Init(channel_state_machine_t *machine, uint16_t raw, uint16_t baseline) {
	/* 复用完整复位函数完成初始化。 */
	ChannelState_Reset(machine, raw, baseline);
/* 结束通道状态机初始化函数。 */
}

/* 清空状态机全部历史数据并恢复为 IDLE。 */
void ChannelState_Reset(channel_state_machine_t *machine, uint16_t raw, uint16_t baseline) {
	/* 清零整个上下文，包括窗口、计数器和 K 值。 */
	memset(machine, 0, sizeof(*machine));
	/* 首帧差值相对实际初始Raw计算，不继承上次恢复历史。 */
	machine->previous_raw = raw;
	/* 复位时同步恢复局部参考，避免首次进入恢复前保留无效零值。 */
	machine->recovery_reference_raw = raw;
	/* 将传入 Baseline 保存为初始激活参考基线。 */
	machine->activation_baseline = baseline;
#if DEVICE_MATRIX20
	/* 复位同时建立可信恢复参考，不继承上次负向低谷。 */
	machine->recovery_anchor_raw = baseline;
	/* 校准复位不继承上一轮大压力的恢复迟滞或确认时间。 */
	machine->recovery_retrigger_delta = CHANNEL_STATE_MATRIX_RETRIGGER_DELTA_MIN;
	/* 初始加载参考与平台统一由校准零点建立，不继承旧受力历史。 */
	machine->loading_reference_raw = baseline;
	machine->active_plateau_raw = raw;
#endif
	/* 将当前通道状态初始化为 IDLE。 */
	machine->current_state = SENSOR_ALGO_STATE_IDLE;
	/* 将待确认状态同步初始化为 IDLE。 */
	machine->pending_state = SENSOR_ALGO_STATE_IDLE;
/* 结束通道状态机复位函数。 */
}

/* 输入一帧通道数据，更新斜率、压力峰值和状态。 */
sensor_algorithm_state_t ChannelState_Update(channel_state_machine_t *machine, const channel_state_input_t *input) {

	/* 保存本帧 Raw 相对激活 Baseline 的非负压力。 */
	uint16_t pressure;
	/* 固定噪声与当前硬件变体的激活门槛共同决定按压下限。 */
	uint16_t pressure_on = maximum_u16((uint16_t)(SENSOR_ALGO_FIXED_NOISE * CHANNEL_STATE_PRESSURE_ON_NOISE_MULTIPLIER), CHANNEL_STATE_PRESSURE_ON_MIN);
	/* 返回空闲使用当前变体的较低门槛，保留激活与释放迟滞。 */
	uint16_t pressure_off = maximum_u16((uint16_t)(SENSOR_ALGO_FIXED_NOISE * CHANNEL_STATE_PRESSURE_OFF_NOISE_MULTIPLIER), CHANNEL_STATE_PRESSURE_OFF_MIN);

	/* 固定噪声后的归一化下限为20，松手分段使用独立的固定counts边界。 */
	uint16_t normalization_floor = maximum_u16((uint16_t)(SENSOR_ALGO_FIXED_NOISE * CHANNEL_STATE_NORMALIZATION_NOISE_MULTIPLIER), CHANNEL_STATE_NORMALIZATION_MIN);
	/* 保存最终选定的压力归一化参考值。 */
	uint16_t normalization_reference;
	/* 保存本帧状态分类给出的候选状态。 */
	sensor_algorithm_state_t candidate;
	/* 保存提交候选状态之前的通道状态，用于识别 SLOW_BACK 重新触发。 */
	sensor_algorithm_state_t previous_state;

	/* 冻结请求只描述当前帧，校准、切档或候选撤销后不能沿用历史请求。 */
	machine->baseline_hold_requested = false;

	/* 已校准通道在切档过渡期间保持原状态，不接收合成Norm轨迹的斜率。 */
	if (input->calibrated && !input->suppress_activity && input->autorange_transition_active) {
		/* 取消尚未完成的普通状态消抖，避免过渡结束后提交旧候选状态。 */
		machine->pending_state = machine->current_state;
		machine->pending_count = 0u;
		/* 切档过渡不得继承上一个量程尚未完成的松手确认。 */
		machine->release_confirm_count = 0u;
#if DEVICE_MATRIX20
		/* 切档抑制清除快慢释放证据，禁止跨量程累计。 */
		machine->release_edge_valid = false;
		machine->release_edge_age_frames = 0u;
		machine->slow_release_confirm_count = 0u;
		/* 切档抑制撤销未完成的恢复再按压时间。 */
		machine->retrigger_confirm_us = 0u;
		/* 切档抑制撤销活动平台，合成轨迹不能成为真实加载参考。 */
		machine->active_plateau_valid = false;
#endif

		/* 切档期间不累计空闲或慢回弹稳定帧数。 */
		machine->idle_confirm_count = 0u;
		machine->slow_back_stable_count = 0u;
		/* 合成轨迹不参与基线稳定确认。 */
		machine->baseline_stable_count = 0u;
		machine->previous_raw = input->raw;

		/* 切档合成轨迹不作为有效快慢斜率对外显示。 */
		clear_normalized_k_telemetry(machine);

		/* 保持进入切档前已经确认的通道状态。 */
		return machine->current_state;
	/* 结束自动量程过渡状态保持分支。 */
	}

	/* 过渡结束首帧重建窗口，避免残留合成斜率在随后数帧触发状态变化。 */
	if (input->calibrated && !input->suppress_activity && input->autorange_transition_finished) {
		/* 使用当前真实候选终点重新建立快慢斜率历史。 */
		reset_slope_windows(machine, input->raw);
		/* 斜率窗口重建后重新确认松手，避免旧候选跨窗口生效。 */
		machine->release_confirm_count = 0u;
#if DEVICE_MATRIX20
		/* 窗口重建后所有释放证据均需由新采样重新形成。 */
		machine->release_edge_valid = false;
		machine->release_edge_age_frames = 0u;
		machine->slow_release_confirm_count = 0u;
		/* 切档结束重建窗口时，恢复候选必须重新计时。 */
		machine->retrigger_confirm_us = 0u;
		/* 新量程必须重新积累稳定平台，不能跨量程继承旧Raw。 */
		machine->active_plateau_valid = false;
#endif

		/* 清除切档前尚未完成的状态和空闲确认计数。 */
		machine->pending_state = machine->current_state;
		machine->pending_count = 0u;
		machine->idle_confirm_count = 0u;
		machine->slow_back_stable_count = 0u;
		/* 新量程从真实Raw终点重新开始稳定确认。 */
		machine->baseline_stable_count = 0u;
		machine->previous_raw = input->raw;

		/* SLOW_BACK以当前Raw重新建立局部低点，防止跨档差值被误认为重新按压。 */
		if (machine->current_state == SENSOR_ALGO_STATE_SLOW_BACK) {
			machine->slow_back_reference_raw = input->raw;
		}

		/* 本帧只完成斜率同步，从下一帧开始恢复正常状态分类。 */
		return machine->current_state;
	/* 结束自动量程过渡后窗口重建分支。 */
	}

	/* 正常工作时把本帧Raw加入快慢窗口。 */
	append_raw(machine, input->raw);
#if DEVICE_MATRIX20
	/* 稳定HOLD只记录观测平台，不改写状态、基线或输出，也不触发超时清零。 */
	if ((machine->current_state == SENSOR_ALGO_STATE_ACTIVE_HOLD) && matrix_raw_is_settled(machine)) {
		machine->active_plateau_raw = input->raw;
		machine->active_plateau_valid = true;
	}
#endif
	/* 空闲、等待和恢复阶段不累计活动持续时间。 */
	if (ChannelState_IsIdleLike(machine->current_state)) {
		/* 清零活动累计帧数，等待下一轮有效按压重新开始。 */
		machine->active_elapsed_frames = 0u;
	/* 按压及释放过程属于同一轮活动，继续累计时长。 */
	} else if (machine->active_elapsed_frames < UINT32_MAX) {
		/* 使用饱和计数累计本轮活动总时长。 */
		machine->active_elapsed_frames++;
	/* 结束活动持续时间累计分支。 */
	}
	/* 当前状态持续帧数尚未饱和时增加一帧。 */
	if (machine->state_elapsed_frames < UINT32_MAX) {
		/* 使用饱和计数避免设备长期运行后发生回绕。 */
		machine->state_elapsed_frames++;
	/* 结束状态持续帧数递增分支。 */
	}
#if DEVICE_MATRIX20
	/* 累计实际帧周期并饱和，避免长时间运行后超时计数回绕。 */
	if (UINT32_MAX - machine->state_elapsed_us < matrix_frame_period_us) {
		machine->state_elapsed_us = UINT32_MAX;
	} else {
		machine->state_elapsed_us += matrix_frame_period_us;
	}
#endif

	/* 尚未校准或上层要求抑制活动时强制保持 IDLE。 */
	if (!input->calibrated || input->suppress_activity) {
		/* 使用当前 Baseline 更新激活参考基线。 */
		machine->activation_baseline = input->baseline;
		/* 清除本次活动记录的压力峰值。 */
		machine->peak_pressure = 0u;
		/* 清除返回空闲的连续确认帧数。 */
		machine->idle_confirm_count = 0u;
		/* 清除被抑制阶段不应保留的活动持续时间。 */
		machine->active_elapsed_frames = 0u;
		/* 清除未校准阶段不应对外输出的归一化 K。 */
		clear_normalized_k_telemetry(machine);
		/* 校准或活动抑制清除恢复历史，即使当前已是IDLE也执行。 */
		machine->baseline_stable_count = 0u;
		machine->previous_raw = input->raw;
		/* 通过统一入口提交 IDLE 状态。 */
		/* 即使已经空闲，活动抑制也不得留下旧释放证据。 */
#if DEVICE_MATRIX20
		machine->release_edge_valid = false;
		machine->release_edge_age_frames = 0u;
		machine->slow_release_confirm_count = 0u;
#endif
		commit_state(machine, SENSOR_ALGO_STATE_IDLE, input->raw, input->baseline);
		/* 返回强制空闲后的当前状态。 */
		return machine->current_state;
	/* 结束未校准或活动抑制处理分支。 */
	}

	/* 空闲、等待和恢复阶段持续保存当前Baseline作为下次激活参考。 */
	if (ChannelState_IsIdleLike(machine->current_state)) {
		/* 将本帧 Baseline 保存为活动开始前的参考基线。 */
		machine->activation_baseline = input->baseline;
	/* 结束无接触阶段的激活基线更新分支。 */
	}

	/* 使用 Raw 和锁定的激活基线计算当前非负压力。 */
	pressure = pressure_from_baseline(input->raw, machine->activation_baseline);
	/* 当前压力超过历史峰值时更新本次活动峰值。 */
	if (pressure > machine->peak_pressure) {
		/* 将本帧压力记录为新的峰值。 */
		machine->peak_pressure = pressure;
	/* 结束本次活动压力峰值更新分支。 */
	}

	/* 在本次峰值和噪声归一化下限之间选择较大值。 */
	normalization_reference = maximum_u16(machine->peak_pressure, normalization_floor);

	/* 保存状态提交前的当前状态。 */
	previous_state = machine->current_state;
	/* 首次按压确认期间只冻结基线，真实状态保持IDLE直至三帧确认完成。 */
	machine->baseline_hold_requested = (previous_state == SENSOR_ALGO_STATE_IDLE) && (pressure > pressure_on);
	/* 根据 Raw、压力和快慢斜率计算本帧候选状态。 */
	candidate = classify_state(machine, input->raw, input->baseline, pressure, pressure_on, pressure_off, normalization_reference);
#if DEVICE_MATRIX20
	/* IDLE只有完整按压候选才冻结，未确认的慢漂继续跟随，避免基线永久停在旧零点。 */
	if (previous_state == SENSOR_ALGO_STATE_IDLE) {
		machine->baseline_hold_requested = candidate == SENSOR_ALGO_STATE_ACTIVE_RISING;
	}
#endif
	/* 普通活动结束先明确等待；ZERO_REBASE已完成零点重建，直接回IDLE。 */
	if ((candidate == SENSOR_ALGO_STATE_IDLE) && !ChannelState_IsIdleLike(previous_state) &&
		(previous_state != SENSOR_ALGO_STATE_ZERO_REBASE)) {
		candidate = SENSOR_ALGO_STATE_WAIT_STABLE;
	}
	/* 通过唯一状态写入口执行立即切换或三帧消抖切换。 */
	commit_state(machine, candidate, input->raw, input->baseline);
#if !DEVICE_MATRIX20
	/* 恢复状态重新触发并完成消抖后，从当前 Baseline 重新建立活动参考。 */
	if (((previous_state == SENSOR_ALGO_STATE_SLOW_BACK) ||
		(previous_state == SENSOR_ALGO_STATE_ZERO_REBASE) ||
		(previous_state == SENSOR_ALGO_STATE_WAIT_STABLE) ||
		(previous_state == SENSOR_ALGO_STATE_RECOVER)) &&
		/* 只有正式切换到 RISING 时才执行活动参考重建。 */
		(machine->current_state == SENSOR_ALGO_STATE_ACTIVE_RISING)) {
		/* 使用当前 Baseline 覆盖上一轮按压冻结的激活基线。 */
		machine->activation_baseline = input->baseline;
		/* 恢复状态重新触发后从零开始计算新一轮活动持续时间。 */
		machine->active_elapsed_frames = 0u;
		/* 相对新激活基线重新计算当前压力。 */
		pressure = pressure_from_baseline(input->raw, machine->activation_baseline);
		/* 使用重新计算的当前压力覆盖上一轮按压峰值。 */
		machine->peak_pressure = pressure;
		/* 使用新峰值重新计算归一化压力参考值。 */
		normalization_reference = maximum_u16(machine->peak_pressure, normalization_floor);
	/* 结束恢复状态重新触发活动参考重建分支。 */
	}
#else
	/* 矩阵提交入口可能重建峰值，遥测必须立即使用已提交的新分母。 */
	normalization_reference = maximum_u16(machine->peak_pressure, normalization_floor);
#endif

	/* 状态提交后处于空闲、等待或恢复时清除上一轮活动记录。 */
	if (ChannelState_IsIdleLike(machine->current_state)) {
		/* 清零峰值，使下一帧重新建立活动峰值。 */
		machine->peak_pressure = 0u;
		/* 无接触阶段结束本轮活动持续时间累计。 */
		machine->active_elapsed_frames = 0u;
		/* 无接触阶段只维护Raw窗口，对外Fast K和Slow K固定输出零。 */
		clear_normalized_k_telemetry(machine);
	/* 通道正式进入激活状态后才允许计算和输出归一化 K。 */
	} else {
		/* 使用当前归一化参考值更新显示用 Fast K 和 Slow K。 */
		update_normalized_k_telemetry(machine, normalization_reference);
	/* 结束无接触阶段清零与激活态K更新分支。 */
	}

	/* 保存本帧真实Raw，供下一帧等待稳定判断使用。 */
	machine->previous_raw = input->raw;
	/* 返回已经提交并完成消抖的当前状态。 */
	return machine->current_state;
/* 结束单帧状态机更新函数。 */
}

/* 无有效接触的三个阶段共用活动计时、零Signal及慢基线兼容规则。 */
bool ChannelState_IsIdleLike(sensor_algorithm_state_t state) {
	return (state == SENSOR_ALGO_STATE_IDLE) || (state == SENSOR_ALGO_STATE_WAIT_STABLE) ||
		(state == SENSOR_ALGO_STATE_RECOVER);
}

/* 只提供本帧按压候选冻结决策；活动状态的冻结仍由真实状态决定。 */
bool ChannelState_ShouldHoldBaseline(const channel_state_machine_t *machine) {
	return machine->baseline_hold_requested;
}

/* 获取状态机当前已经提交的通道状态。 */
sensor_algorithm_state_t ChannelState_Get(const channel_state_machine_t *machine) {
	/* 返回 current_state，不修改状态机数据。 */
	return machine->current_state;
/* 结束当前状态读取函数。 */
}

/* 输出层只读取已提交的受力零点，不自行创建或修改状态参考。 */
uint16_t ChannelState_GetActivationBaseline(const channel_state_machine_t *machine) {
	return machine->activation_baseline;
}

/* 获取快速窗口归一化 K，单位为 0.01 %/s。 */
int32_t ChannelState_GetFastNormalizedKX100(const channel_state_machine_t *machine) {
	/* Fast K 有效时返回结果，否则返回零。 */
	return machine->fast_normalized_k_valid ? machine->fast_normalized_k_x100 : 0;
/* 结束 Fast K 读取函数。 */
}

/* 获取慢速窗口归一化 K，单位为 0.01 %/s。 */
int32_t ChannelState_GetSlowNormalizedKX100(const channel_state_machine_t *machine) {
	/* Slow K 有效时返回结果，否则返回零。 */
	return machine->slow_normalized_k_valid ? machine->slow_normalized_k_x100 : 0;
/* 结束 Slow K 读取函数。 */
}
