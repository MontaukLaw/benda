/* 引入自动量程公开常量、状态结构和接口声明。 */
#include "capsense_autorange.h"

/* 引入已通过Flash校验的逐通道逐档仿射参数。 */
#include "capsense_affine_calibration.h"

/* 引入矩阵Ref CDAC运行时写入和配置回读接口。 */
#include "capsense_nsub_control.h"

/* 定义Ref CDAC自动量程的总档数。 */
#define CAPSENSE_AUTORANGE_TIER_COUNT (7u)

/* 定义上电时使用的第1档索引。 */
#define CAPSENSE_AUTORANGE_START_TIER_INDEX (0u)

/* 根据总档数计算最高档索引。 */
#define CAPSENSE_AUTORANGE_MAX_TIER_INDEX (CAPSENSE_AUTORANGE_TIER_COUNT - 1u)

/* 定义普通扩档阈值，对应Max Count的82%。 */
#define CAPSENSE_AUTORANGE_UP_THRESHOLD (48321u)

/* 定义紧急扩档阈值，对应Max Count的95%。 */
#define CAPSENSE_AUTORANGE_EMERGENCY_THRESHOLD (55982u)

/* 定义允许累计降档的低水位阈值，对应Max Count的35%。 */
#define CAPSENSE_AUTORANGE_DOWN_THRESHOLD (20625u)

/* 定义降一档后的预计峰值上限，对应Max Count的65%。 */
#define CAPSENSE_AUTORANGE_DOWN_PREDICT_LIMIT (38303u)

/* 定义紧急扩档希望达到的目标Raw，约为Max Count的70%。 */
#define CAPSENSE_AUTORANGE_TARGET_RAW (41250u)

/* 定义普通扩档需要连续满足高水位的帧数。 */
#define CAPSENSE_AUTORANGE_HIGH_CONFIRM_FRAMES (3u)

/* 定义普通降档需要连续满足低水位的帧数。 */
#define CAPSENSE_AUTORANGE_LOW_CONFIRM_FRAMES (32u)

/* 定义切档后禁止普通升降档的冷却帧数。 */
#define CAPSENSE_AUTORANGE_COOLDOWN_FRAMES (16u)

/* 每次写入新Ref后先等待8帧候选数据，屏蔽CapSense硬件切档瞬态。 */
#define CAPSENSE_AUTORANGE_SWITCH_SETTLE_FRAMES (8u)

/* 新Ref候选值可用后用8帧三次曲线衔接预测轨迹和真实轨迹。 */
#define CAPSENSE_AUTORANGE_SWITCH_HERMITE_FRAMES (8u)

/* 一条连续自动切档链在第24帧强制结束候选等待，紧急扩档不重新计时。 */
#define CAPSENSE_AUTORANGE_SWITCH_CHAIN_MAX_SETTLE_FRAMES (24u)

/* 使用最近4个输出差分估计切档前速度，抑制单帧噪声影响。 */
#define CAPSENSE_AUTORANGE_VELOCITY_HISTORY_LENGTH (4u)

/* 使用最近3个Candidate差分判断新Ref的速度是否已稳定。 */
#define CAPSENSE_AUTORANGE_CANDIDATE_VELOCITY_WINDOW (3u)

/* 最近3个Candidate速度的极差不超过128 counts/帧时认为本帧稳定。 */
#define CAPSENSE_AUTORANGE_CANDIDATE_VELOCITY_RANGE_LIMIT (128)

/* 四通道Candidate速度需要连续3帧稳定才允许进入Hermite衔接。 */
#define CAPSENSE_AUTORANGE_CANDIDATE_VELOCITY_CONFIRM_FRAMES (3u)

/* 将预测和候选速度限制在每帧8192 counts内，避免异常样本导致轨迹失控。 */
#define CAPSENSE_AUTORANGE_VELOCITY_LIMIT (8192)

/* 三次Hermite基函数使用Q16系数，65536表示实数1。 */
#define CAPSENSE_AUTORANGE_HERMITE_ONE_Q16 (65536LL)

/* 定义峰值一阶滤波步长为当前误差的四分之一。 */
#define CAPSENSE_AUTORANGE_FILTER_SHIFT (2u)

/* 标记存在尚未写入硬件的档位请求。 */
#define CAPSENSE_AUTORANGE_FLAG_PENDING (1u << 0u)

/* 标记当前已经位于最低档。 */
#define CAPSENSE_AUTORANGE_FLAG_LOW_LIMIT (1u << 1u)

/* 标记当前已经位于最高档。 */
#define CAPSENSE_AUTORANGE_FLAG_HIGH_LIMIT (1u << 2u)

/* 标记固定参数校验或Ref写入失败。 */
#define CAPSENSE_AUTORANGE_FLAG_ERROR (1u << 3u)

/* 标记矩阵四路中至少一路仍有真实接触。 */
#define CAPSENSE_AUTORANGE_FLAG_CONTACT (1u << 4u)

/* 标记32位归一化Raw超过算法16位输入范围。 */
#define CAPSENSE_AUTORANGE_FLAG_NORMALIZED_CLIPPED (1u << 5u)

/* 标记当前由上位机手动选择Ref，自动升降档已经暂停。 */
#define CAPSENSE_AUTORANGE_FLAG_MANUAL_MODE (1u << 6u)

/* 标记本帧CH4至CH7已使用Flash仿射参数生成Norm Raw。 */
#define CAPSENSE_AUTORANGE_FLAG_AFFINE_ACTIVE (1u << 7u)

/* 档位按约1.5倍递增，在灵敏度和量程覆盖之间取得平衡。 */
static const uint8_t ref_cdac_tiers[CAPSENSE_AUTORANGE_TIER_COUNT] = {
	CAPSENSE_AUTORANGE_START_REF, 18u, 27u, 40u, 60u, 90u, 135u,
};

/* 保存当前扫描实际使用的档位索引。 */
static uint8_t active_tier_index;

/* 保存扫描空闲点即将应用的档位索引。 */
static uint8_t pending_tier_index;

/* 标记是否存在尚未写入硬件的Ref切换请求。 */
static bool ref_change_pending;

/* 标记Hardware Raw峰值滤波器是否已经取得首个样本。 */
static bool filter_initialized;

/* 保存上一帧矩阵通道是否仍有真实接触。 */
static bool last_contact_active;

/* 标记是否处于上位机控制Ref的手动校准模式。 */
static bool manual_mode_enabled;

/* 以Q8格式保存跨帧滤波峰值，避免整数滤波丢失小数精度。 */
static uint32_t filtered_peak_q8;

/* 保存各通道最近一次最终Norm Raw，供新切档建立连续起点。 */
static uint32_t last_normalized_raw[CAPSENSE_AUTORANGE_TOTAL_CHANNEL_COUNT];

/* 区分无过渡、速度预测和三次Hermite衔接三个输出阶段。 */
typedef enum {
	/* 当前直接输出仿射换算后的候选Norm Raw。 */
	CAPSENSE_AUTORANGE_TRANSITION_IDLE = 0u,
	/* 新Ref硬件稳定期间按切档前速度继续预测输出。 */
	CAPSENSE_AUTORANGE_TRANSITION_PREDICT = 1u,
	/* 候选值可用后用三次曲线消除位置和速度误差。 */
	CAPSENSE_AUTORANGE_TRANSITION_HERMITE = 2u,
} capsense_autorange_transition_phase_t;

/* 保存当前Norm Raw切档过渡阶段。 */
static capsense_autorange_transition_phase_t switch_transition_phase;

/* 标记当前是否处于同一次连续自动切档链。 */
static bool switch_chain_active;

/* 记录当前Ref写入后已经使用速度预测的帧数。 */
static uint8_t switch_settle_elapsed_frames;

/* 记录连续自动切档链已经消耗的总帧数。 */
static uint8_t switch_chain_elapsed_frames;

/* 记录当前三次Hermite衔接已经输出的帧数。 */
static uint8_t switch_hermite_step;

/* 记录四通道Candidate速度已经连续稳定的帧数。 */
static uint8_t switch_candidate_velocity_stable_count;

/* 保存CH4至CH7当前连续预测轨迹的位置。 */
static int64_t switch_trajectory_position[CAPSENSE_AUTORANGE_CHANNEL_COUNT];

/* 保存CH4至CH7当前连续预测轨迹的每帧速度。 */
static int32_t switch_trajectory_velocity[CAPSENSE_AUTORANGE_CHANNEL_COUNT];

/* 保存Hermite起点相对候选轨迹的位置误差。 */
static int64_t switch_hermite_position_error[CAPSENSE_AUTORANGE_CHANNEL_COUNT];

/* 保存Hermite起点相对候选轨迹的速度误差。 */
static int32_t switch_hermite_velocity_error[CAPSENSE_AUTORANGE_CHANNEL_COUNT];

/* 保存每个矩阵通道最近4个最终输出差分，用于估计切档前速度。 */
static int32_t switch_output_delta_history[CAPSENSE_AUTORANGE_CHANNEL_COUNT]
	[CAPSENSE_AUTORANGE_VELOCITY_HISTORY_LENGTH];

/* 保存每个矩阵通道最近4个Candidate差分，供最近3帧稳定检查和速度估计。 */
static int32_t switch_candidate_delta_history[CAPSENSE_AUTORANGE_CHANNEL_COUNT]
	[CAPSENSE_AUTORANGE_VELOCITY_HISTORY_LENGTH];

/* 保存上一帧最终输出和新Ref候选值，供跨帧差分使用。 */
static uint32_t switch_previous_output[CAPSENSE_AUTORANGE_CHANNEL_COUNT];
static uint32_t switch_previous_candidate[CAPSENSE_AUTORANGE_CHANNEL_COUNT];

/* 保存输出差分环形缓存的位置和当前有效样本数。 */
static uint8_t switch_output_history_index;
static uint8_t switch_output_history_count;

/* 保存候选差分环形缓存的位置和当前有效样本数。 */
static uint8_t switch_candidate_history_index;
static uint8_t switch_candidate_history_count;

/* 标记最终输出和候选值是否已经取得可用于差分的上一帧。 */
static bool switch_output_history_initialized;
static bool switch_candidate_history_initialized;

/* 保存自动量程运行状态，供控制逻辑和上位机协议共同读取。 */
static capsense_autorange_status_t autorange_status;

/* 将实际Ref映射到不小于该值的最近档位。 */
static uint8_t find_tier_index(uint8_t ref_cdac) {
	/* 保存当前正在比较的档位索引。 */
	uint8_t tier_index;

	/* 从最低档开始查找第一个能够覆盖实际Ref的档位。 */
	for (tier_index = 0u; tier_index < CAPSENSE_AUTORANGE_TIER_COUNT; tier_index++) {
		/* 当前档位Ref不小于实际Ref时完成映射。 */
		if (ref_cdac <= ref_cdac_tiers[tier_index]) {
			/* 返回匹配到的档位索引。 */
			return tier_index;
		}
	}

	/* 实际Ref超过档位表时限制到最高档。 */
	return CAPSENSE_AUTORANGE_MAX_TIER_INDEX;
}

/* 根据当前峰值直接选择能够把Raw压回目标区间的紧急档位。 */
static uint8_t find_emergency_tier(uint16_t peak_raw) {
	/* 保存按当前峰值反推得到的最低所需Ref。 */
	uint32_t required_ref;

	/* 保存当前正在比较的候选档位索引。 */
	uint8_t tier_index;

	/* 按Raw与Ref近似反比例关系计算目标Raw对应的最低Ref，并向上取整。 */
	required_ref = (((uint32_t)ref_cdac_tiers[active_tier_index] * peak_raw) +
		(CAPSENSE_AUTORANGE_TARGET_RAW - 1u)) / CAPSENSE_AUTORANGE_TARGET_RAW;

	/* 只从当前档位以上查找，紧急处理不允许反向降档。 */
	for (tier_index = (uint8_t)(active_tier_index + 1u);
		 tier_index < CAPSENSE_AUTORANGE_TIER_COUNT;
		 tier_index++) {
		/* 选择第一个不小于最低所需Ref的档位。 */
		if ((uint32_t)ref_cdac_tiers[tier_index] >= required_ref) {
			/* 返回能够满足目标Raw的最低档位。 */
			return tier_index;
		}
	}

	/* 所有候选档都不足时限制到最高档。 */
	return CAPSENSE_AUTORANGE_MAX_TIER_INDEX;
}

/* 记录待切换档位，实际写硬件操作延后到扫描空闲点执行。 */
static void request_tier(uint8_t tier_index, capsense_autorange_state_t state) {
	/* 目标档与当前档相同或已有请求时不重复排队。 */
	if ((tier_index == active_tier_index) || ref_change_pending) {
		/* 保留现有状态并结束本次请求。 */
		return;
	}

	/* 保存下一次扫描要使用的档位。 */
	pending_tier_index = tier_index;

	/* 保存自动量程请求的目标Ref，供上位机区分请求值和实际值。 */
	autorange_status.requested_ref_cdac = ref_cdac_tiers[tier_index];

	/* 标记扫描空闲点需要执行Ref写入。 */
	ref_change_pending = true;

	/* 记录本次请求是扩档还是降档。 */
	autorange_status.state = (uint8_t)state;
}

/* 算法接口暂时使用16位Raw，溢出时限幅并上报诊断标志。 */
static uint16_t clamp_algorithm_raw(uint32_t normalized) {
	/* 检查32位归一化结果是否超过算法16位输入范围。 */
	if (normalized > UINT16_MAX) {
		/* 设置限幅标志，通知上位机当前算法输入已经失真。 */
		autorange_status.flags |= CAPSENSE_AUTORANGE_FLAG_NORMALIZED_CLIPPED;

		/* 返回16位最大值，避免截断后回绕。 */
		return UINT16_MAX;
	}

	/* 数值位于16位范围内时直接转换。 */
	return (uint16_t)normalized;
}

/* 将有符号轨迹值限制到32位Norm Raw可表示范围。 */
static uint32_t clamp_transition_value(int64_t value) {
	/* 负的预测或平滑结果限制为零。 */
	if (value <= 0LL) {
		return 0u;
	}

	/* 超过32位上限时执行饱和，避免无符号转换回绕。 */
	if ((uint64_t)value > UINT32_MAX) {
		return UINT32_MAX;
	}

	/* 返回位于合法范围内的32位Norm Raw。 */
	return (uint32_t)value;
}

/* 计算相邻32位样本差并限制速度范围，避免异常点驱动预测轨迹。 */
static int32_t calculate_limited_velocity(uint32_t current, uint32_t previous) {
	/* 使用64位有符号差值同时覆盖上升和下降过程。 */
	int64_t delta = (int64_t)current - (int64_t)previous;

	/* 正向变化超过上限时限制到最大允许速度。 */
	if (delta > CAPSENSE_AUTORANGE_VELOCITY_LIMIT) {
		return CAPSENSE_AUTORANGE_VELOCITY_LIMIT;
	}

	/* 反向变化超过上限时限制到最小允许速度。 */
	if (delta < -CAPSENSE_AUTORANGE_VELOCITY_LIMIT) {
		return -CAPSENSE_AUTORANGE_VELOCITY_LIMIT;
	}

	/* 返回正常范围内的每帧变化量。 */
	return (int32_t)delta;
}

/* 对最多4个有符号差分取中位数，得到抗单点噪声的轨迹速度。 */
static int32_t calculate_median_velocity(
	const int32_t history[CAPSENSE_AUTORANGE_VELOCITY_HISTORY_LENGTH],
	uint8_t count) {
	/* 保存排序使用的局部副本，避免修改环形历史。 */
	int32_t sorted[CAPSENSE_AUTORANGE_VELOCITY_HISTORY_LENGTH];

	/* 保存插入排序使用的索引和值。 */
	uint8_t index;
	uint8_t insert_index;
	int32_t value;

	/* 没有历史差分时按静止处理。 */
	if (count == 0u) {
		return 0;
	}

	/* 只复制当前已经有效的差分样本。 */
	for (index = 0u; index < count; index++) {
		sorted[index] = history[index];
	}

	/* 使用小规模插入排序把差分从小到大排列。 */
	for (index = 1u; index < count; index++) {
		/* 暂存当前待插入的差分值。 */
		value = sorted[index];
		insert_index = index;

		/* 将所有较大元素右移一位。 */
		while ((insert_index > 0u) && (sorted[insert_index - 1u] > value)) {
			sorted[insert_index] = sorted[insert_index - 1u];
			insert_index--;
		}

		/* 把当前差分放入排序后位置。 */
		sorted[insert_index] = value;
	}

	/* 奇数个样本直接返回居中的差分。 */
	if ((count & 1u) != 0u) {
		return sorted[count / 2u];
	}

	/* 偶数个样本返回中间两个差分的均值。 */
	return (int32_t)(((int64_t)sorted[(count / 2u) - 1u] +
		(int64_t)sorted[count / 2u]) / 2LL);
}

/* 读取指定通道候选速度环形缓存中倒数第N个样本。 */
static int32_t get_recent_candidate_velocity(uint8_t matrix_index, uint8_t reverse_index) {
	/* 保存从当前写入位置反向计算的实际环形索引。 */
	uint8_t history_index;

	/* 下一写入位置的前一项是最新速度样本。 */
	history_index =
		(uint8_t)((switch_candidate_history_index +
			CAPSENSE_AUTORANGE_VELOCITY_HISTORY_LENGTH - 1u - reverse_index) %
			CAPSENSE_AUTORANGE_VELOCITY_HISTORY_LENGTH);

	/* 返回指定新旧顺序的Candidate速度。 */
	return switch_candidate_delta_history[matrix_index][history_index];
}

/* 使用最近3个Candidate差分的中位数估计Hermite目标速度。 */
static int32_t calculate_recent_candidate_velocity(uint8_t matrix_index) {
	/* 保存按新到旧顺序取出的候选速度。 */
	int32_t recent_velocity[CAPSENSE_AUTORANGE_VELOCITY_HISTORY_LENGTH] = {0};

	/* 保存拷贝最近速度时使用的索引和数量。 */
	uint8_t velocity_index;
	uint8_t velocity_count = switch_candidate_history_count;

	/* 已有3个以上样本时只使用最新的3个，不混入硬件暂态旧速度。 */
	if (velocity_count > CAPSENSE_AUTORANGE_CANDIDATE_VELOCITY_WINDOW) {
		velocity_count = CAPSENSE_AUTORANGE_CANDIDATE_VELOCITY_WINDOW;
	}

	/* 从环形缓存中依次取出最新的有效速度样本。 */
	for (velocity_index = 0u; velocity_index < velocity_count; velocity_index++) {
		recent_velocity[velocity_index] =
			get_recent_candidate_velocity(matrix_index, velocity_index);
	}

	/* 连续切档在总等待上限提前释放时，对已有样本仍取中位数。 */
	return calculate_median_velocity(recent_velocity, velocity_count);
}

/* 检查CH4至CH7最近的Candidate速度是否已经同时趋于稳定。 */
static bool candidate_velocity_is_stable(void) {
	/* 保存当前检查的矩阵通道和速度样本索引。 */
	uint8_t matrix_index;
	uint8_t velocity_index;

	/* 保存当前通道最近3个速度的最小值、最大值和本次样本。 */
	int32_t minimum_velocity;
	int32_t maximum_velocity;
	int32_t velocity;

	/* 少于3个差分时无法判断速度稳定性。 */
	if (switch_candidate_history_count < CAPSENSE_AUTORANGE_CANDIDATE_VELOCITY_WINDOW) {
		return false;
	}

	/* 四个矩阵通道必须全部通过速度极差检查。 */
	for (matrix_index = 0u; matrix_index < CAPSENSE_AUTORANGE_CHANNEL_COUNT; matrix_index++) {
		/* 使用最新速度同时初始化当前通道的极值。 */
		minimum_velocity = get_recent_candidate_velocity(matrix_index, 0u);
		maximum_velocity = minimum_velocity;

		/* 检查其余两个最近速度并更新极值。 */
		for (velocity_index = 1u;
			 velocity_index < CAPSENSE_AUTORANGE_CANDIDATE_VELOCITY_WINDOW;
			 velocity_index++) {
			/* 取出当前待比较的Candidate速度。 */
			velocity = get_recent_candidate_velocity(matrix_index, velocity_index);

			/* 更新当前通道最小速度。 */
			if (velocity < minimum_velocity) {
				minimum_velocity = velocity;
			}

			/* 更新当前通道最大速度。 */
			if (velocity > maximum_velocity) {
				maximum_velocity = velocity;
			}
		}

		/* 任一通道速度极差过大都说明硬件暂态尚未结束。 */
		if ((maximum_velocity - minimum_velocity) >
			CAPSENSE_AUTORANGE_CANDIDATE_VELOCITY_RANGE_LIMIT) {
			return false;
		}
	}

	/* 所有矩阵通道均通过本帧Candidate速度稳定性检查。 */
	return true;
}

/* 清空当前Ref下的候选Norm Raw差分历史。 */
static void reset_candidate_velocity_history(void) {
	/* 新Ref第一帧尚无可比较的上一帧候选值。 */
	switch_candidate_history_initialized = false;

	/* 环形缓存从首项重新写入并清除有效样本数。 */
	switch_candidate_history_index = 0u;
	switch_candidate_history_count = 0u;

	/* 新Ref必须重新累计连续速度稳定帧数。 */
	switch_candidate_velocity_stable_count = 0u;
}

/* 记录本帧候选Norm Raw差分，供Hermite终点速度估计。 */
static void record_candidate_velocity(
	const uint32_t candidate_normalized[CAPSENSE_AUTORANGE_TOTAL_CHANNEL_COUNT]) {
	/* 保存矩阵四通道内部索引和完整通道号。 */
	uint8_t matrix_index;
	uint8_t channel_index;

	/* 第一帧只建立上一候选值，不生成无意义差分。 */
	if (!switch_candidate_history_initialized) {
		for (matrix_index = 0u; matrix_index < CAPSENSE_AUTORANGE_CHANNEL_COUNT; matrix_index++) {
			/* 将矩阵内部索引映射到CH4至CH7。 */
			channel_index = (uint8_t)(CAPSENSE_AUTORANGE_CHANNEL_FIRST + matrix_index);
			switch_previous_candidate[matrix_index] = candidate_normalized[channel_index];
		}

		/* 标记下一帧可以开始计算候选速度。 */
		switch_candidate_history_initialized = true;
		return;
	}

	/* 四通道在同一个环形位置保存本帧候选差分。 */
	for (matrix_index = 0u; matrix_index < CAPSENSE_AUTORANGE_CHANNEL_COUNT; matrix_index++) {
		/* 将矩阵内部索引映射到CH4至CH7。 */
		channel_index = (uint8_t)(CAPSENSE_AUTORANGE_CHANNEL_FIRST + matrix_index);

		/* 保存限幅后的候选值每帧速度。 */
		switch_candidate_delta_history[matrix_index][switch_candidate_history_index] =
			calculate_limited_velocity(
				candidate_normalized[channel_index],
				switch_previous_candidate[matrix_index]);

		/* 更新下一帧差分使用的候选值。 */
		switch_previous_candidate[matrix_index] = candidate_normalized[channel_index];
	}

	/* 推进候选差分环形缓存写入位置。 */
	switch_candidate_history_index =
		(uint8_t)((switch_candidate_history_index + 1u) %
			CAPSENSE_AUTORANGE_VELOCITY_HISTORY_LENGTH);

	/* 有效候选差分数最多保持为4。 */
	if (switch_candidate_history_count < CAPSENSE_AUTORANGE_VELOCITY_HISTORY_LENGTH) {
		switch_candidate_history_count++;
	}
}

/* 记录最终Norm Raw差分，供下一条独立切档轨迹估计起始速度。 */
static void record_output_velocity(
	const uint32_t normalized_raw[CAPSENSE_AUTORANGE_TOTAL_CHANNEL_COUNT]) {
	/* 保存矩阵四通道内部索引和完整通道号。 */
	uint8_t matrix_index;
	uint8_t channel_index;

	/* 第一帧只建立上一输出，不生成无意义差分。 */
	if (!switch_output_history_initialized) {
		for (matrix_index = 0u; matrix_index < CAPSENSE_AUTORANGE_CHANNEL_COUNT; matrix_index++) {
			/* 将矩阵内部索引映射到CH4至CH7。 */
			channel_index = (uint8_t)(CAPSENSE_AUTORANGE_CHANNEL_FIRST + matrix_index);
			switch_previous_output[matrix_index] = normalized_raw[channel_index];
		}

		/* 标记下一帧可以开始计算最终输出速度。 */
		switch_output_history_initialized = true;
		return;
	}

	/* 四通道在同一个环形位置保存本帧最终输出差分。 */
	for (matrix_index = 0u; matrix_index < CAPSENSE_AUTORANGE_CHANNEL_COUNT; matrix_index++) {
		/* 将矩阵内部索引映射到CH4至CH7。 */
		channel_index = (uint8_t)(CAPSENSE_AUTORANGE_CHANNEL_FIRST + matrix_index);

		/* 保存限幅后的最终输出每帧速度。 */
		switch_output_delta_history[matrix_index][switch_output_history_index] =
			calculate_limited_velocity(
				normalized_raw[channel_index],
				switch_previous_output[matrix_index]);

		/* 更新下一帧差分使用的最终输出。 */
		switch_previous_output[matrix_index] = normalized_raw[channel_index];
	}

	/* 推进最终输出差分环形缓存写入位置。 */
	switch_output_history_index =
		(uint8_t)((switch_output_history_index + 1u) %
			CAPSENSE_AUTORANGE_VELOCITY_HISTORY_LENGTH);

	/* 有效最终输出差分数最多保持为4。 */
	if (switch_output_history_count < CAPSENSE_AUTORANGE_VELOCITY_HISTORY_LENGTH) {
		switch_output_history_count++;
	}
}

/* 清除当前切档预测、Hermite衔接和连续切档链状态。 */
static void reset_switch_transition(void) {
	/* 恢复为直接输出候选Norm Raw的空闲阶段。 */
	switch_transition_phase = CAPSENSE_AUTORANGE_TRANSITION_IDLE;

	/* 结束当前连续自动切档链。 */
	switch_chain_active = false;

	/* 清空单次切档和整条切档链累计帧数。 */
	switch_settle_elapsed_frames = 0u;
	switch_chain_elapsed_frames = 0u;

	/* 清空三次Hermite衔接进度。 */
	switch_hermite_step = 0u;

	/* 清除当前Ref的候选速度历史。 */
	reset_candidate_velocity_history();

	/* 协议不再报告候选等待或平滑过渡帧。 */
	autorange_status.blank_frames = 0u;
}

/* 更新连续切档链计时，并在控制器完全恢复保持后结束本次切档链。 */
static void update_switch_chain_age(void) {
	/* 非切档链状态不需要累计或清理。 */
	if (!switch_chain_active) {
		return;
	}

	/* 总计时只累加到候选等待上限，避免8位计数回绕。 */
	if (switch_chain_elapsed_frames < CAPSENSE_AUTORANGE_SWITCH_CHAIN_MAX_SETTLE_FRAMES) {
		switch_chain_elapsed_frames++;
	}

	/* 过渡、待写和冷却全部结束后才允许下一次切档重新计时。 */
	if ((switch_transition_phase == CAPSENSE_AUTORANGE_TRANSITION_IDLE) &&
		!ref_change_pending &&
		(autorange_status.cooldown_frames == 0u)) {
		/* 当前链已经真正结束，下一次独立切档可以重新取得完整预算。 */
		switch_chain_active = false;
		switch_chain_elapsed_frames = 0u;
	}
}

/* 候选稳定期结束时建立预测轨迹到候选轨迹的三次Hermite约束。 */
static void start_switch_hermite(
	const uint32_t candidate_normalized[CAPSENSE_AUTORANGE_TOTAL_CHANNEL_COUNT]) {
	/* 保存矩阵四通道内部索引和完整通道号。 */
	uint8_t matrix_index;
	uint8_t channel_index;

	/* 保存当前通道在新Ref下估算出的候选速度。 */
	int32_t candidate_velocity;

	/* 四通道分别保存起点位置误差和速度误差。 */
	for (matrix_index = 0u; matrix_index < CAPSENSE_AUTORANGE_CHANNEL_COUNT; matrix_index++) {
		/* 将矩阵内部索引映射到CH4至CH7。 */
		channel_index = (uint8_t)(CAPSENSE_AUTORANGE_CHANNEL_FIRST + matrix_index);

		/* 只使用最近3个已趋稳定的差分，避免硬件暂态污染Hermite目标速度。 */
		candidate_velocity = calculate_recent_candidate_velocity(matrix_index);

		/* 位置误差使Hermite曲线从当前预测输出连续起步。 */
		switch_hermite_position_error[matrix_index] =
			switch_trajectory_position[matrix_index] -
			(int64_t)candidate_normalized[channel_index];

		/* 速度误差使Hermite曲线起点斜率与当前预测斜率连续。 */
		switch_hermite_velocity_error[matrix_index] =
			switch_trajectory_velocity[matrix_index] - candidate_velocity;
	}

	/* 下一帧从Hermite第1步开始逐渐并入实时候选轨迹。 */
	switch_transition_phase = CAPSENSE_AUTORANGE_TRANSITION_HERMITE;
	switch_hermite_step = 0u;
	autorange_status.blank_frames = CAPSENSE_AUTORANGE_SWITCH_HERMITE_FRAMES;
}

/* 按当前过渡阶段生成单个矩阵通道的连续Norm Raw输出。 */
static uint32_t calculate_transition_output(
	uint8_t channel_index,
	uint32_t candidate_normalized) {
	/* 保存完整通道号对应的矩阵内部索引。 */
	uint8_t matrix_index;

	/* 保存Hermite计算使用的Q16时间及二次、三次项。 */
	int64_t time_q16;
	int64_t time_squared_q16;
	int64_t time_cubed_q16;

	/* 保存位置误差和速度误差对应的Hermite基函数。 */
	int64_t position_basis_q16;
	int64_t velocity_basis_q16;

	/* 保存Q16修正量、整数修正量和最终输出。 */
	int64_t correction_q16;
	int64_t correction;
	int64_t output;

	/* 保存沿上一帧速度前进一步的预测边界及最终包络边界。 */
	uint32_t predicted_output;
	uint32_t lower_envelope;
	uint32_t upper_envelope;

	/* 非矩阵通道和空闲阶段始终直接返回候选值。 */
	if ((channel_index < CAPSENSE_AUTORANGE_CHANNEL_FIRST) ||
		(switch_transition_phase == CAPSENSE_AUTORANGE_TRANSITION_IDLE)) {
		return candidate_normalized;
	}

	/* 将完整通道号转换为CH4至CH7内部索引。 */
	matrix_index = (uint8_t)(channel_index - CAPSENSE_AUTORANGE_CHANNEL_FIRST);

	/* 新Ref候选数据等待期间沿用切档前轨迹速度，不再输出水平平台。 */
	if (switch_transition_phase == CAPSENSE_AUTORANGE_TRANSITION_PREDICT) {
		/* 用上一位置加每帧速度推进当前预测轨迹。 */
		switch_trajectory_position[matrix_index] +=
			(int64_t)switch_trajectory_velocity[matrix_index];

		/* 预测轨迹异常越界时执行32位饱和。 */
		output = (int64_t)clamp_transition_value(switch_trajectory_position[matrix_index]);
		switch_trajectory_position[matrix_index] = output;
		return (uint32_t)output;
	}

	/* 将当前Hermite步数换算为0至1范围内的Q16时间。 */
	time_q16 =
		((int64_t)switch_hermite_step * CAPSENSE_AUTORANGE_HERMITE_ONE_Q16) /
		(int64_t)CAPSENSE_AUTORANGE_SWITCH_HERMITE_FRAMES;

	/* 计算Q16格式的时间二次项和三次项。 */
	time_squared_q16 =
		(time_q16 * time_q16) / CAPSENSE_AUTORANGE_HERMITE_ONE_Q16;
	time_cubed_q16 =
		(time_squared_q16 * time_q16) / CAPSENSE_AUTORANGE_HERMITE_ONE_Q16;

	/* h00使位置误差在终点同时衰减到零且导数为零。 */
	position_basis_q16 =
		(2LL * time_cubed_q16) -
		(3LL * time_squared_q16) +
		CAPSENSE_AUTORANGE_HERMITE_ONE_Q16;

	/* h10使起点速度误差连续，并在终点与候选速度完全汇合。 */
	velocity_basis_q16 =
		time_cubed_q16 -
		(2LL * time_squared_q16) +
		time_q16;

	/* 合并位置误差和按8帧跨度换算的速度误差。 */
	correction_q16 =
		(position_basis_q16 * switch_hermite_position_error[matrix_index]) +
		(velocity_basis_q16 *
			(int64_t)CAPSENSE_AUTORANGE_SWITCH_HERMITE_FRAMES *
			(int64_t)switch_hermite_velocity_error[matrix_index]);

	/* 将Q16有符号修正量按符号对称地四舍五入到整数counts。 */
	if (correction_q16 >= 0LL) {
		correction =
			(correction_q16 + (CAPSENSE_AUTORANGE_HERMITE_ONE_Q16 / 2LL)) /
			CAPSENSE_AUTORANGE_HERMITE_ONE_Q16;
	} else {
		correction =
			-((-correction_q16 + (CAPSENSE_AUTORANGE_HERMITE_ONE_Q16 / 2LL)) /
				CAPSENSE_AUTORANGE_HERMITE_ONE_Q16);
	}

	/* 在实时候选值上叠加三次衰减误差得到连续输出。 */
	output = (int64_t)candidate_normalized + correction;
	output = (int64_t)clamp_transition_value(output);

	/* 沿当前轨迹速度计算本帧本应到达的预测位置。 */
	predicted_output = clamp_transition_value(
		switch_trajectory_position[matrix_index] +
		(int64_t)switch_trajectory_velocity[matrix_index]);

	/* 预测位置不大于Candidate时使用正向包络边界。 */
	if (predicted_output <= candidate_normalized) {
		lower_envelope = predicted_output;
		upper_envelope = candidate_normalized;
	} else {
		/* 预测位置大于Candidate时交换边界顺序。 */
		lower_envelope = candidate_normalized;
		upper_envelope = predicted_output;
	}

	/* 禁止Hermite输出低于Candidate和预测轨迹共同形成的下边界。 */
	if ((uint32_t)output < lower_envelope) {
		output = (int64_t)lower_envelope;
	}

	/* 禁止Hermite速度项把输出推过上边界并形成软件鼓包。 */
	if ((uint32_t)output > upper_envelope) {
		output = (int64_t)upper_envelope;
	}

	/* 保存本帧实际轨迹速度，连续紧急升档时从当前斜率继续预测。 */
	switch_trajectory_velocity[matrix_index] = calculate_limited_velocity(
		(uint32_t)output,
		clamp_transition_value(switch_trajectory_position[matrix_index]));

	/* 保存本帧轨迹位置，作为下一帧或下一次紧急切档的连续起点。 */
	switch_trajectory_position[matrix_index] = output;
	return (uint32_t)output;
}

/* 完成本帧过渡输出后推进预测等待或Hermite衔接状态。 */
static void finish_switch_transition_frame(
	const uint32_t candidate_normalized[CAPSENSE_AUTORANGE_TOTAL_CHANNEL_COUNT]) {
	/* 预测阶段按当前Ref累计候选等待帧数。 */
	if (switch_transition_phase == CAPSENSE_AUTORANGE_TRANSITION_PREDICT) {
		/* 当前帧已经沿预测轨迹输出，可以计入当前Ref的最小等待。 */
		if (switch_settle_elapsed_frames < CAPSENSE_AUTORANGE_SWITCH_SETTLE_FRAMES) {
			switch_settle_elapsed_frames++;
		}

		/* 整条连续切档链达到24帧时强制释放，避免异常候选值永久阻塞。 */
		if (switch_chain_elapsed_frames >= CAPSENSE_AUTORANGE_SWITCH_CHAIN_MAX_SETTLE_FRAMES) {
			start_switch_hermite(candidate_normalized);
			return;
		}

		/* 未满8帧时继续报告最小候选等待的剩余帧数。 */
		if (switch_settle_elapsed_frames < CAPSENSE_AUTORANGE_SWITCH_SETTLE_FRAMES) {
			autorange_status.blank_frames =
				(uint8_t)(CAPSENSE_AUTORANGE_SWITCH_SETTLE_FRAMES - switch_settle_elapsed_frames);
			return;
		}

		/* 最小等待后检查四通道最近3个Candidate速度的极差。 */
		if (candidate_velocity_is_stable()) {
			/* 本帧四通道全部稳定时累加连续通过帧数。 */
			if (switch_candidate_velocity_stable_count <
				CAPSENSE_AUTORANGE_CANDIDATE_VELOCITY_CONFIRM_FRAMES) {
				switch_candidate_velocity_stable_count++;
			}
		} else {
			/* 任一通道未稳定时重新计算连续通过帧数。 */
			switch_candidate_velocity_stable_count = 0u;
		}

		/* 四通道连续3帧速度稳定后才启动Hermite衔接。 */
		if (switch_candidate_velocity_stable_count >=
			CAPSENSE_AUTORANGE_CANDIDATE_VELOCITY_CONFIRM_FRAMES) {
			start_switch_hermite(candidate_normalized);
			return;
		}

		/* 超过最小等待但尚未稳定时保持非零，上位机不会提前判定切档完成。 */
		autorange_status.blank_frames = 1u;
		return;
	}

	/* Hermite阶段在第8步精确回到实时候选轨迹。 */
	if (switch_transition_phase == CAPSENSE_AUTORANGE_TRANSITION_HERMITE) {
		/* 已完成全部8步时结束切档过渡。 */
		if (switch_hermite_step >= CAPSENSE_AUTORANGE_SWITCH_HERMITE_FRAMES) {
			switch_transition_phase = CAPSENSE_AUTORANGE_TRANSITION_IDLE;
			switch_hermite_step = 0u;
			autorange_status.blank_frames = 0u;
			return;
		}

		/* 未完成时报告仍需执行的Hermite衔接帧数。 */
		autorange_status.blank_frames =
			(uint8_t)(CAPSENSE_AUTORANGE_SWITCH_HERMITE_FRAMES - switch_hermite_step);
		return;
	}

	/* 空闲阶段不应保留任何切档过渡帧。 */
	autorange_status.blank_frames = 0u;
}

/* 初始化自动量程状态，并确认固定采样参数和起始Ref已经生效。 */
bool CapSenseAutoRange_Init(void) {
	/* 保存从CapSense中间件回读的矩阵配置。 */
	capsense_matrix_config_status_t matrix_status;

	/* 保存初始化归一化缓存时使用的通道索引。 */
	uint8_t channel_index;

	/* 保存清空速度环形缓存时使用的历史索引。 */
	uint8_t history_index;

	/* 从CapSense运行时回读实际配置，避免软件档位与硬件Ref不一致。 */
	CapSenseMatrixConfig_GetStatus(&matrix_status);

	/* 将硬件实际Ref映射成当前软件档位。 */
	active_tier_index = find_tier_index(matrix_status.actual_ref_cdac);

	/* 初始化待切换档位为当前档，避免未请求时引用无效索引。 */
	pending_tier_index = active_tier_index;

	/* 清除上电前不存在的Ref切换请求。 */
	ref_change_pending = false;

	/* 标记峰值滤波器尚未取得首个有效样本。 */
	filter_initialized = false;

	/* 清除上电前不存在的接触状态。 */
	last_contact_active = false;

	/* 上电默认使用自动量程，只有收到协议命令后才进入手动校准模式。 */
	manual_mode_enabled = false;

	/* 清空Q8格式的Hardware Raw滤波峰值。 */
	filtered_peak_q8 = 0u;

	/* 清空切档轨迹建立使用的上一帧最终数据。 */
	for (channel_index = 0u; channel_index < CAPSENSE_AUTORANGE_TOTAL_CHANNEL_COUNT; channel_index++) {
		/* 每个通道的最终输出缓存均从零开始。 */
		last_normalized_raw[channel_index] = 0u;
	}

	/* 上电时不存在尚未完成的切档预测、平滑或连续切档链。 */
	reset_switch_transition();

	/* 清空最终输出速度历史的公共状态。 */
	switch_output_history_initialized = false;
	switch_output_history_index = 0u;
	switch_output_history_count = 0u;

	/* 清空四个矩阵通道的预测、Hermite约束和速度历史。 */
	for (channel_index = 0u; channel_index < CAPSENSE_AUTORANGE_CHANNEL_COUNT; channel_index++) {
		/* 每个通道的轨迹位置和速度均从零开始。 */
		switch_trajectory_position[channel_index] = 0LL;
		switch_trajectory_velocity[channel_index] = 0;

		/* 每个通道的Hermite位置和速度误差均从零开始。 */
		switch_hermite_position_error[channel_index] = 0LL;
		switch_hermite_velocity_error[channel_index] = 0;

		/* 上电时尚未保存上一帧最终输出或候选值。 */
		switch_previous_output[channel_index] = 0u;
		switch_previous_candidate[channel_index] = 0u;

		/* 清空当前通道的最终输出和候选速度环形缓存。 */
		for (history_index = 0u;
			 history_index < CAPSENSE_AUTORANGE_VELOCITY_HISTORY_LENGTH;
			 history_index++) {
			switch_output_delta_history[channel_index][history_index] = 0;
			switch_candidate_delta_history[channel_index][history_index] = 0;
		}
	}

	/* 初始化协议状态，便于上位机在首个状态帧中看到完整配置。 */
	/* 保存固定Nsub。 */
	autorange_status.nsub = CAPSENSE_AUTORANGE_FIXED_NSUB;

	/* 保存固定TxDiv。 */
	autorange_status.tx_div = CAPSENSE_AUTORANGE_FIXED_TX_DIV;

	/* 保存固定配置对应的理论Max Count。 */
	autorange_status.max_count = CAPSENSE_AUTORANGE_MAX_COUNT;

	/* 保存当前硬件实际Ref。 */
	autorange_status.actual_ref_cdac = matrix_status.actual_ref_cdac;

	/* 上电时请求Ref与当前硬件Ref保持一致。 */
	autorange_status.requested_ref_cdac = matrix_status.actual_ref_cdac;

	/* 保存当前软件档位索引。 */
	autorange_status.tier_index = active_tier_index;

	/* 保存上位机显示使用的总档数。 */
	autorange_status.tier_count = CAPSENSE_AUTORANGE_TIER_COUNT;

	/* 将控制状态初始化为启动阶段。 */
	autorange_status.state = CAPSENSE_AUTORANGE_STATE_STARTUP;

	/* 清空所有诊断标志。 */
	autorange_status.flags = 0u;

	/* 清空本帧Hardware Raw峰值。 */
	autorange_status.peak_raw = 0u;

	/* 清空滤波后的Hardware Raw峰值。 */
	autorange_status.filtered_peak_raw = 0u;

	/* 清空连续高水位计数。 */
	autorange_status.high_count = 0u;

	/* 清空连续低水位计数。 */
	autorange_status.low_count = 0u;

	/* 清空档位冷却计数。 */
	autorange_status.cooldown_frames = 0u;

	/* 清空候选等待或Hermite衔接剩余帧数。 */
	autorange_status.blank_frames = 0u;

	/* 任一固定参数未正确应用时停止自动切档，避免使用错误比例归一化。 */
	if ((matrix_status.actual_nsub != CAPSENSE_AUTORANGE_FIXED_NSUB) ||
		(matrix_status.actual_tx_clk_div != CAPSENSE_AUTORANGE_FIXED_TX_DIV) ||
		(matrix_status.actual_ref_cdac != ref_cdac_tiers[CAPSENSE_AUTORANGE_START_TIER_INDEX])) {
		/* 将控制状态切换为错误。 */
		autorange_status.state = CAPSENSE_AUTORANGE_STATE_ERROR;

		/* 设置协议错误标志。 */
		autorange_status.flags |= CAPSENSE_AUTORANGE_FLAG_ERROR;

		/* 返回初始化失败，禁止正常自动切档。 */
		return false;
	}

	/* 固定参数校验成功后强制从第1档开始。 */
	active_tier_index = CAPSENSE_AUTORANGE_START_TIER_INDEX;

	/* 将待切换索引同步为第1档。 */
	pending_tier_index = active_tier_index;

	/* 将协议显示档位同步为第1档。 */
	autorange_status.tier_index = active_tier_index;

	/* 返回初始化成功。 */
	return true;
}

/* 归一化一帧Raw，并根据CH4至CH7的Hardware Raw峰值决定升降档。 */
void CapSenseAutoRange_ProcessFrame(
	const uint16_t hardware_raw[CAPSENSE_AUTORANGE_TOTAL_CHANNEL_COUNT],
	bool matrix_contact_active,
	bool matrix_release_confirmed,
	uint16_t algorithm_raw[CAPSENSE_AUTORANGE_TOTAL_CHANNEL_COUNT],
	uint32_t normalized_raw[CAPSENSE_AUTORANGE_TOTAL_CHANNEL_COUNT],
	uint32_t candidate_normalized_raw[CAPSENSE_AUTORANGE_TOTAL_CHANNEL_COUNT]) {
	/* 保存当前处理的通道索引。 */
	uint8_t channel_index;

	/* 保存CH4至CH7本帧最大的Hardware Raw。 */
	uint16_t peak_raw = 0u;

	/* 保存由Q8滤波值还原出的16位峰值。 */
	uint16_t filtered_peak;

	/* 保存当前通道按Ref=60折算后的32位Raw。 */
	uint32_t normalized;

	/* 保存本帧各通道尚未决定是否释放的候选归一化Raw。 */
	uint32_t candidate_normalized[CAPSENSE_AUTORANGE_TOTAL_CHANNEL_COUNT];

	/* 保存降一档后预计出现的Hardware Raw峰值。 */
	uint32_t predicted_peak;

	/* 标记本帧是否仍处于速度预测或Hermite衔接阶段。 */
	bool transition_busy;

	/* 每帧重新生成瞬时标志，错误和待切换状态由各自逻辑单独维护。 */
	autorange_status.flags &= (uint8_t)~(CAPSENSE_AUTORANGE_FLAG_CONTACT |
		CAPSENSE_AUTORANGE_FLAG_LOW_LIMIT |
		CAPSENSE_AUTORANGE_FLAG_HIGH_LIMIT |
		CAPSENSE_AUTORANGE_FLAG_NORMALIZED_CLIPPED |
		CAPSENSE_AUTORANGE_FLAG_AFFINE_ACTIVE);

	/* 任一矩阵通道仍有真实接触时设置接触标志。 */
	if (matrix_contact_active) {
		/* 将接触状态写入协议标志。 */
		autorange_status.flags |= CAPSENSE_AUTORANGE_FLAG_CONTACT;
	}

	/* 保存本帧接触结果，供状态查询时重新生成接触标志。 */
	last_contact_active = matrix_contact_active;

	/* 先计算完整候选值，CH0至CH3保持原始值，CH4至CH7归一化到Ref=60。 */
	for (channel_index = 0u; channel_index < CAPSENSE_AUTORANGE_TOTAL_CHANNEL_COUNT; channel_index++) {
		/* 非矩阵通道不参与Ref自动量程，保持Hardware Raw不变。 */
		if (channel_index < CAPSENSE_AUTORANGE_CHANNEL_FIRST) {
			/* 直接把Hardware Raw作为该通道归一化结果。 */
			normalized = hardware_raw[channel_index];
		} else {
			/* 仅在标定时的固定Nsub和TxDiv下使用Flash仿射参数。 */
			if ((autorange_status.nsub == CAPSENSE_AUTORANGE_FIXED_NSUB) &&
				(autorange_status.tx_div == CAPSENSE_AUTORANGE_FIXED_TX_DIV) &&
				CapSenseAffineCalibration_Apply(
					channel_index,
					autorange_status.actual_ref_cdac,
					hardware_raw[channel_index],
					&normalized)) {
				/* 标记当前Norm Raw已进入实际仿射应用路径。 */
				autorange_status.flags |= CAPSENSE_AUTORANGE_FLAG_AFFINE_ACTIVE;
			} else {
				/* 无有效Flash参数时保留旧理论比例，确保设备仍可正常运行。 */
				normalized = (((uint32_t)hardware_raw[channel_index] * autorange_status.actual_ref_cdac) +
					(CAPSENSE_AUTORANGE_NORMALIZATION_REF / 2u)) / CAPSENSE_AUTORANGE_NORMALIZATION_REF;
			}

			/* 使用Hardware Raw而不是归一化Raw寻找矩阵四路峰值。 */
			if (hardware_raw[channel_index] > peak_raw) {
				/* 更新本帧矩阵Hardware Raw峰值。 */
				peak_raw = hardware_raw[channel_index];
			}
		}

		/* 暂存本帧候选结果，等待预测和Hermite逻辑生成实际输出。 */
		candidate_normalized[channel_index] = normalized;

		/* 同步保存未经过渡处理的候选值，供调试协议和CSV独立记录。 */
		candidate_normalized_raw[channel_index] = normalized;
	}

	/* 每帧只在一处累计连续切档链的共享候选等待时间。 */
	update_switch_chain_age();

	/* 预测阶段持续记录新Ref候选速度，供随后Hermite终点约束使用。 */
	if (switch_transition_phase == CAPSENSE_AUTORANGE_TRANSITION_PREDICT) {
		record_candidate_velocity(candidate_normalized);
	}

	/* Hermite阶段在四通道计算前统一推进步数，确保使用相同时间比例。 */
	if ((switch_transition_phase == CAPSENSE_AUTORANGE_TRANSITION_HERMITE) &&
		(switch_hermite_step < CAPSENSE_AUTORANGE_SWITCH_HERMITE_FRAMES)) {
		switch_hermite_step++;
	}

	/* 保存本帧开始时的过渡状态，完成最后一步后下一帧再恢复普通控制。 */
	transition_busy = switch_transition_phase != CAPSENSE_AUTORANGE_TRANSITION_IDLE;

	/* 根据速度预测或三次Hermite曲线生成算法和协议最终使用的Norm Raw。 */
	for (channel_index = 0u; channel_index < CAPSENSE_AUTORANGE_TOTAL_CHANNEL_COUNT; channel_index++) {
		/* 从本帧预先计算的候选数组取出当前通道结果。 */
		normalized = candidate_normalized[channel_index];

		/* 矩阵通道在切档期间使用连续预测或Hermite衔接后的结果。 */
		normalized = calculate_transition_output(channel_index, normalized);

		/* 保存最终输出，供下一次独立或连续紧急切档建立连续起点。 */
		last_normalized_raw[channel_index] = normalized;

		/* 保存完整32位归一化Raw，供协议和CSV记录使用。 */
		normalized_raw[channel_index] = normalized;

		/* 将归一化结果限幅成算法当前使用的16位输入。 */
		algorithm_raw[channel_index] = clamp_algorithm_raw(normalized);
	}

	/* 保存最终输出差分，为下一次独立切档估计当前运动速度。 */
	record_output_velocity(normalized_raw);

	/* 四通道均完成本帧输出后统一推进预测或Hermite状态。 */
	finish_switch_transition_frame(candidate_normalized);

	/* 对四路Hardware Raw最大值做一阶低通，普通升降档使用滤波值判断。 */
	autorange_status.peak_raw = peak_raw;

	/* 首帧直接使用当前峰值初始化滤波器，避免从零缓慢爬升。 */
	if (!filter_initialized) {
		/* 将16位峰值左移8位转换为Q8格式。 */
		filtered_peak_q8 = (uint32_t)peak_raw << 8u;

		/* 标记后续帧可以执行正常一阶滤波。 */
		filter_initialized = true;
	} else {
		/* 将本帧Hardware Raw峰值转换为Q8滤波目标。 */
		uint32_t target_peak_q8 = (uint32_t)peak_raw << 8u;

		/* 目标不小于当前滤波值时从下方向目标靠近。 */
		if (target_peak_q8 >= filtered_peak_q8) {
			/* 将两者差值的四分之一累加到滤波值。 */
			filtered_peak_q8 += (target_peak_q8 - filtered_peak_q8) >> CAPSENSE_AUTORANGE_FILTER_SHIFT;
		} else {
			/* 目标较小时将差值的四分之一从滤波值中扣除。 */
			filtered_peak_q8 -= (filtered_peak_q8 - target_peak_q8) >> CAPSENSE_AUTORANGE_FILTER_SHIFT;
		}
	}

	/* 右移8位，把Q8滤波值还原成16位Hardware Raw计数。 */
	filtered_peak = (uint16_t)(filtered_peak_q8 >> 8u);

	/* 保存滤波峰值，供状态协议显示和后续升降档判断。 */
	autorange_status.filtered_peak_raw = filtered_peak;

	/* 已有待写请求或硬件错误时不再产生新的普通档位请求。 */
	if (ref_change_pending || (autorange_status.state == CAPSENSE_AUTORANGE_STATE_ERROR)) {
		/* 等待现有请求处理或保留错误状态。 */
		return;
	}

	/* 手动校准模式只完成归一化和状态更新，不执行任何自动升降档判断。 */
	if (manual_mode_enabled) {
		/* 速度预测和Hermite衔接均结束时才报告当前手动档位稳定可采样。 */
		if (!transition_busy) {
			/* 使用独立状态区分自动保持和手动校准保持。 */
			autorange_status.state = CAPSENSE_AUTORANGE_STATE_MANUAL_HOLD;
		}

		/* 手动模式处理到此结束，保持上位机指定的Ref。 */
		return;
	}

	/* 切档预测或Hermite期间只允许紧急扩档，防止接近满量程时继续饱和。 */
	if (transition_busy) {
		/* 峰值达到紧急阈值且尚未到最高档时继续扩档。 */
		if ((peak_raw >= CAPSENSE_AUTORANGE_EMERGENCY_THRESHOLD) &&
			(active_tier_index < CAPSENSE_AUTORANGE_MAX_TIER_INDEX)) {
			/* 计算并请求能够快速解除饱和的目标档位。 */
			request_tier(find_emergency_tier(peak_raw), CAPSENSE_AUTORANGE_STATE_UP_PENDING);
		}

		/* 切档过渡期不执行普通升档或降档判断。 */
		return;
	}

	/* 冷却期间禁止普通升降档，但紧急扩档仍具有最高优先级。 */
	if (autorange_status.cooldown_frames != 0u) {
		/* 消耗一帧冷却时间。 */
		autorange_status.cooldown_frames--;

		/* 将协议控制状态标记为冷却。 */
		autorange_status.state = CAPSENSE_AUTORANGE_STATE_COOLDOWN;

		/* 冷却期峰值达到紧急阈值时允许继续扩档。 */
		if ((peak_raw >= CAPSENSE_AUTORANGE_EMERGENCY_THRESHOLD) &&
			(active_tier_index < CAPSENSE_AUTORANGE_MAX_TIER_INDEX)) {
			/* 计算并请求紧急目标档位。 */
			request_tier(find_emergency_tier(peak_raw), CAPSENSE_AUTORANGE_STATE_UP_PENDING);
		}

		/* 冷却期不执行其他普通升降档判断。 */
		return;
	}

	/* 单帧达到95%满量程时立即选择更高档，避免Raw饱和丢失信息。 */
	if ((peak_raw >= CAPSENSE_AUTORANGE_EMERGENCY_THRESHOLD) &&
		(active_tier_index < CAPSENSE_AUTORANGE_MAX_TIER_INDEX)) {
		/* 直接请求能够把峰值压回目标区间的更高档位。 */
		request_tier(find_emergency_tier(peak_raw), CAPSENSE_AUTORANGE_STATE_UP_PENDING);

		/* 紧急切档后清除普通高水位计数。 */
		autorange_status.high_count = 0u;

		/* 紧急切档后同时清除低水位计数。 */
		autorange_status.low_count = 0u;

		/* 已产生紧急请求，本帧不再执行后续判断。 */
		return;
	}

	/* 滤波峰值连续3帧达到82%时逐级扩档。 */
	if (filtered_peak >= CAPSENSE_AUTORANGE_UP_THRESHOLD) {
		/* 高水位计数未达到确认上限时继续累加。 */
		if (autorange_status.high_count < CAPSENSE_AUTORANGE_HIGH_CONFIRM_FRAMES) {
			/* 记录本帧连续高水位。 */
			autorange_status.high_count++;
		}

		/* 高水位出现时取消正在积累的降档计数。 */
		autorange_status.low_count = 0u;

		/* 连续高水位达到确认帧数后执行扩档或报告上限。 */
		if (autorange_status.high_count >= CAPSENSE_AUTORANGE_HIGH_CONFIRM_FRAMES) {
			/* 当前档位不是最高档时只上调一级。 */
			if (active_tier_index < CAPSENSE_AUTORANGE_MAX_TIER_INDEX) {
				/* 请求相邻的下一高档。 */
				request_tier((uint8_t)(active_tier_index + 1u), CAPSENSE_AUTORANGE_STATE_UP_PENDING);
			} else {
				/* 已在最高档时将控制状态标记为边界。 */
				autorange_status.state = CAPSENSE_AUTORANGE_STATE_LIMIT;

				/* 设置最高档标志，提示上位机已没有扩档余量。 */
				autorange_status.flags |= CAPSENSE_AUTORANGE_FLAG_HIGH_LIMIT;
			}
		}

		/* 高水位分支处理完成后结束本帧判断。 */
		return;
	}

	/* 未达到扩档阈值时清除高水位连续计数。 */
	autorange_status.high_count = 0u;
	/* 四路分别处于IDLE或SLOW_BACK的任意组合时，允许累计低水位并请求降档。 */
	if (matrix_release_confirmed && (filtered_peak <= CAPSENSE_AUTORANGE_DOWN_THRESHOLD)) {
		/* 低水位计数未达到确认上限时继续累加。 */
		if (autorange_status.low_count < CAPSENSE_AUTORANGE_LOW_CONFIRM_FRAMES) {
			/* 记录本帧连续低水位。 */
			autorange_status.low_count++;
		}

		/* 低水位持续32帧后，再验证下一低档不会超过65%满量程。 */
		if (autorange_status.low_count >= CAPSENSE_AUTORANGE_LOW_CONFIRM_FRAMES) {
			/* 第1档无法继续降档。 */
			if (active_tier_index == 0u) {
				/* 将控制状态标记为边界。 */
				autorange_status.state = CAPSENSE_AUTORANGE_STATE_LIMIT;

				/* 设置最低档标志，提示上位机已到最高灵敏度。 */
				autorange_status.flags |= CAPSENSE_AUTORANGE_FLAG_LOW_LIMIT;

				/* 最低档处理完成后结束本帧判断。 */
				return;
			}

			/* 按Ref反比例估算降一档后的Hardware Raw峰值。 */
			predicted_peak = ((uint32_t)filtered_peak * ref_cdac_tiers[active_tier_index] +
				(ref_cdac_tiers[active_tier_index - 1u] - 1u)) /
				ref_cdac_tiers[active_tier_index - 1u];

			/* 预计峰值不超过65%满量程时才允许降一级。 */
			if (predicted_peak <= CAPSENSE_AUTORANGE_DOWN_PREDICT_LIMIT) {
				/* 请求相邻的下一低档，提高无压力或小压力时的灵敏度。 */
				request_tier((uint8_t)(active_tier_index - 1u), CAPSENSE_AUTORANGE_STATE_DOWN_PENDING);
			}
		}

		/* 低水位分支处理完成后结束本帧判断。 */
		return;
	}

	/* 松手条件或低水位条件不满足时保持当前档位并清除降档计数。 */
	autorange_status.low_count = 0u;

	/* 将控制状态恢复为保持。 */
	autorange_status.state = CAPSENSE_AUTORANGE_STATE_HOLD;
}

/* 在扫描空闲点应用待切换Ref，并启动速度预测和Hermite衔接。 */
bool CapSenseAutoRange_ApplyPending(void) {
	/* 保存当前连续切档链仍可使用的候选等待帧数。 */
	uint8_t remaining_settle_budget;

	/* 没有待切换请求时不访问CapSense硬件。 */
	if (!ref_change_pending) {
		/* 返回未执行切档。 */
		return false;
	}

	/* Ref写入或回读失败时进入错误状态，停止继续自动切档。 */
	if (!CapSenseMatrixRefCdac_Apply(ref_cdac_tiers[pending_tier_index])) {
		/* 将控制状态切换为错误。 */
		autorange_status.state = CAPSENSE_AUTORANGE_STATE_ERROR;

		/* 设置协议错误标志。 */
		autorange_status.flags |= CAPSENSE_AUTORANGE_FLAG_ERROR;

		/* 清除失败请求，避免每帧重复写入硬件。 */
		ref_change_pending = false;

		/* 返回Ref应用失败。 */
		return false;
	}

	/* 写入成功后同步软件档位，并从下一帧开始执行稳定等待。 */
	/* 将当前档位切换为刚刚写入硬件的档位。 */
	active_tier_index = pending_tier_index;

	/* 清除已经完成的Ref切换请求。 */
	ref_change_pending = false;

	/* 将协议中的实际Ref更新为当前档位值。 */
	autorange_status.actual_ref_cdac = ref_cdac_tiers[active_tier_index];

	/* 将协议中的档位索引更新为当前档位。 */
	autorange_status.tier_index = active_tier_index;

	/* 将控制状态标记为切换后稳定阶段。 */
	autorange_status.state = CAPSENSE_AUTORANGE_STATE_SWITCHING;

	/* 清除切档前积累的高水位计数。 */
	autorange_status.high_count = 0u;

	/* 清除切档前积累的低水位计数。 */
	autorange_status.low_count = 0u;

	/* 手动校准不需要自动量程冷却，自动模式切档后仍使用原冷却帧数。 */
	if (manual_mode_enabled) {
		/* 手动切档稳定后即可继续采样，不额外等待自动量程冷却。 */
		autorange_status.cooldown_frames = 0u;
	} else {
		/* 自动切档继续使用原有冷却帧数，避免档位往返摆动。 */
		autorange_status.cooldown_frames = CAPSENSE_AUTORANGE_COOLDOWN_FRAMES;
	}

	/* 手动切档是独立校准操作，每次都重新取得完整硬件稳定预算。 */
	if (manual_mode_enabled) {
		/* 新建独立手动切档链并从零开始累计总帧数。 */
		switch_chain_active = true;
		switch_chain_elapsed_frames = 0u;
	} else if (!switch_chain_active) {
		/* 自动模式只有真正开始新切档链时才清零共享计时。 */
		switch_chain_active = true;
		switch_chain_elapsed_frames = 0u;
	}


	/* 空闲输出首次进入切档时从最近最终输出和中位速度建立预测轨迹。 */
	if (switch_transition_phase == CAPSENSE_AUTORANGE_TRANSITION_IDLE) {
		/* 保存初始化四通道轨迹使用的矩阵索引和完整通道号。 */
		uint8_t matrix_index;
		uint8_t channel_index;

		/* 四通道分别从自身最后输出和最近4帧速度启动预测。 */
		for (matrix_index = 0u; matrix_index < CAPSENSE_AUTORANGE_CHANNEL_COUNT; matrix_index++) {
			/* 将矩阵内部索引映射到CH4至CH7。 */
			channel_index = (uint8_t)(CAPSENSE_AUTORANGE_CHANNEL_FIRST + matrix_index);

			/* 预测位置从切档前最后一个真实输出开始。 */
			switch_trajectory_position[matrix_index] =
				(int64_t)last_normalized_raw[channel_index];

			/* 最近4个输出差分的中位数作为切档前速度。 */
			switch_trajectory_velocity[matrix_index] = calculate_median_velocity(
				switch_output_delta_history[matrix_index],
				switch_output_history_count);
		}
	}

	/* 连续紧急扩档期间保留当前轨迹位置和速度，只重新等待新Ref候选值。 */
	switch_transition_phase = CAPSENSE_AUTORANGE_TRANSITION_PREDICT;
	switch_settle_elapsed_frames = 0u;
	switch_hermite_step = 0u;

	/* 新Ref的候选速度必须重新采集，不能沿用上一档硬件差分。 */
	reset_candidate_velocity_history();

	/* 计算整条自动切档链达到24帧边界前还剩多少候选等待时间。 */
	if (switch_chain_elapsed_frames < CAPSENSE_AUTORANGE_SWITCH_CHAIN_MAX_SETTLE_FRAMES) {
		remaining_settle_budget =
			(uint8_t)(CAPSENSE_AUTORANGE_SWITCH_CHAIN_MAX_SETTLE_FRAMES -
				switch_chain_elapsed_frames);
	} else {
		/* 已到总等待边界时下一帧预测后立即开始Hermite衔接。 */
		remaining_settle_budget = 1u;
	}

	/* 协议先显示最小8帧等待，随后未通过速度稳定检查时保持为1。 */
	if (remaining_settle_budget < CAPSENSE_AUTORANGE_SWITCH_SETTLE_FRAMES) {
		autorange_status.blank_frames = remaining_settle_budget;
	} else {
		autorange_status.blank_frames = CAPSENSE_AUTORANGE_SWITCH_SETTLE_FRAMES;
	}

	/* 清空旧档位下的Q8滤波峰值。 */
	filtered_peak_q8 = 0u;

	/* 使下一帧直接初始化新档位下的峰值滤波器。 */
	filter_initialized = false;

	/* 返回Ref切换成功。 */
	return true;
}

/* 切换手动校准模式，并在需要时排队写入指定的固定Ref档位。 */
bool CapSenseAutoRange_SetManualMode(bool enabled, uint8_t ref_cdac) {
	/* 保存由固定Ref值查找到的档位索引。 */
	uint8_t tier_index;

	/* 退出手动模式时恢复自动量程，并保留当前实际档位作为自动控制起点。 */
	if (!enabled) {
		/* 清除手动模式标志，使下一帧恢复自动升降档判断。 */
		manual_mode_enabled = false;

		/* 自动模式的请求Ref与当前实际Ref保持一致。 */
		autorange_status.requested_ref_cdac = autorange_status.actual_ref_cdac;

		/* 清除手动采样期间不再适用的升档连续计数。 */
		autorange_status.high_count = 0u;

		/* 清除手动采样期间不再适用的降档连续计数。 */
		autorange_status.low_count = 0u;

		/* 恢复自动保持状态，下一帧会根据真实峰值继续判断。 */
		autorange_status.state = CAPSENSE_AUTORANGE_STATE_HOLD;

		/* 返回模式切换成功。 */
		return true;
	}

	/* 将请求Ref映射到七个固定档位中的候选索引。 */
	tier_index = find_tier_index(ref_cdac);

	/* 映射后的值必须与请求值完全相同，拒绝任意中间Ref。 */
	if (ref_cdac_tiers[tier_index] != ref_cdac) {
		/* 返回参数无效，不改变当前模式和硬件档位。 */
		return false;
	}

	/* 先进入手动模式，确保同一轮扫描不会再产生自动切档请求。 */
	manual_mode_enabled = true;

	/* 保存上位机请求值，供0x28状态帧确认命令执行进度。 */
	autorange_status.requested_ref_cdac = ref_cdac;

	/* 清除自动模式留下的升档连续计数。 */
	autorange_status.high_count = 0u;

	/* 清除自动模式留下的降档连续计数。 */
	autorange_status.low_count = 0u;

	/* 目标档位已经生效且没有待写请求时直接进入手动稳定状态。 */
	if ((tier_index == active_tier_index) && !ref_change_pending) {
		/* 当前Ref无需重新写入硬件。 */
		autorange_status.state = CAPSENSE_AUTORANGE_STATE_MANUAL_HOLD;

		/* 清除不再需要的切档预测、平滑和连续切档链状态。 */
		reset_switch_transition();

		/* 清除自动模式遗留的冷却计数。 */
		autorange_status.cooldown_frames = 0u;

		/* 返回档位已经立即生效。 */
		return true;
	}

	/* 手动命令覆盖尚未应用的自动档位请求。 */
	pending_tier_index = tier_index;

	/* 标记扫描空闲点需要写入新的Ref。 */
	ref_change_pending = true;

	/* 在实际写入并完成稳定等待前报告切换状态。 */
	autorange_status.state = CAPSENSE_AUTORANGE_STATE_SWITCHING;

	/* 返回手动切档请求已经成功排队。 */
	return true;
}

/* 返回自动量程当前是否已被上位机暂停。 */
bool CapSenseAutoRange_IsManualMode(void) {
	/* 直接返回模块内部唯一的手动模式状态。 */
	return manual_mode_enabled;
}

/* 手动矩阵参数在首帧稳定后，同步自动量程模块保存的硬件实际值。 */
void CapSenseAutoRange_OnMatrixConfigApplied(void) {
	/* 保存Nsub控制模块回读的矩阵实际配置。 */
	capsense_matrix_config_status_t matrix_status;

	/* 从CapSense中间件回读最终生效值，避免使用请求值冒充实际值。 */
	CapSenseMatrixConfig_GetStatus(&matrix_status);

	/* 同步当前手动采样参数，供状态帧和归一化路径使用。 */
	autorange_status.nsub = matrix_status.actual_nsub;
	autorange_status.tx_div = matrix_status.actual_tx_clk_div;
	autorange_status.actual_ref_cdac = matrix_status.actual_ref_cdac;
	autorange_status.requested_ref_cdac = matrix_status.actual_ref_cdac;

	/* 为以后恢复自动量程准备不低于当前Ref的最近固定档位。 */
	active_tier_index = find_tier_index(matrix_status.actual_ref_cdac);
	pending_tier_index = active_tier_index;
	autorange_status.tier_index = active_tier_index;
	ref_change_pending = false;

	/* 新采样参数已经重建硬件基线，不再沿用旧切档连续轨迹。 */
	reset_switch_transition();
	filter_initialized = false;
	autorange_status.high_count = 0u;
	autorange_status.low_count = 0u;
	autorange_status.cooldown_frames = 0u;
	autorange_status.state = manual_mode_enabled ?
		CAPSENSE_AUTORANGE_STATE_MANUAL_HOLD : CAPSENSE_AUTORANGE_STATE_HOLD;
}

/* 复制当前状态，并在返回前补充待应用和接触标志。 */
void CapSenseAutoRange_GetStatus(capsense_autorange_status_t *status) {
	/* 先清除上一次查询生成的待应用标志。 */
	autorange_status.flags &= (uint8_t)~CAPSENSE_AUTORANGE_FLAG_PENDING;

	/* 当前仍有Ref请求等待写入时重新设置待应用标志。 */
	if (ref_change_pending) {
		/* 将待应用状态写入协议标志。 */
		autorange_status.flags |= CAPSENSE_AUTORANGE_FLAG_PENDING;
	}

	/* 上一帧仍有真实接触时保留接触标志。 */
	if (last_contact_active) {
		/* 将接触状态写入协议标志。 */
		autorange_status.flags |= CAPSENSE_AUTORANGE_FLAG_CONTACT;
	} else {
		/* 没有真实接触时清除协议接触标志。 */
		autorange_status.flags &= (uint8_t)~CAPSENSE_AUTORANGE_FLAG_CONTACT;
	}

	/* 根据当前控制模式实时生成手动校准标志。 */
	if (manual_mode_enabled) {
		/* 设置手动模式标志，通知上位机自动升降档已经暂停。 */
		autorange_status.flags |= CAPSENSE_AUTORANGE_FLAG_MANUAL_MODE;
	} else {
		/* 自动模式下清除手动标志。 */
		autorange_status.flags &= (uint8_t)~CAPSENSE_AUTORANGE_FLAG_MANUAL_MODE;
	}

	/* 将完整状态复制到调用者提供的输出结构。 */
	*status = autorange_status;
}
