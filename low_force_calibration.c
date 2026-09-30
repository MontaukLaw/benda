/* 低力表独占参数行，所有擦写仍经过DFU地址保护入口。 */
#include "low_force_calibration.h"
#include "app_flash.h"
#include "sensor_algorithm.h"
#include <math.h>
#include <string.h>

/* LFC记录版本2共32字节；兼容版本1的四点表，仍独占原来的Flash行。 */
#define LOW_FORCE_MAGIC (0x3143464Cu)
static float low_force_points[LOW_FORCE_POINT_COUNT];
static bool low_force_valid;
/* 新增检测线随标定保存；伪随机数只用于明确标识的模拟输出。 */
static float low_force_detection_counts;
static uint32_t low_force_random = 0x69B4C821u;
/* 标准重力加速度9.80665 m/s²，质量g先除以1000再乘重力。 */
static const float low_force_newtons[LOW_FORCE_POINT_COUNT] = {0.0980665f, 0.196133f, 0.4903325f, 0.980665f};

/* 按记录版本校验参数区，拒绝未写完和损坏的Flash数据。 */
static uint32_t low_force_checksum(const uint8_t *bytes, uint32_t length) {
	uint32_t hash = 2166136261u;
	for (uint32_t i = 0u; i < length; ++i) {
		hash = (hash ^ bytes[i]) * 16777619u;
	}
	return hash;
}

/* 四个输入必须有限、为正、严格递增，且不超过四路16位Signal的总量程。 */
static bool low_force_points_valid(const float *points) {
	float previous = 0.0f;
	for (uint32_t i = 0u; i < LOW_FORCE_POINT_COUNT; ++i) {
		if (!isfinite(points[i]) || points[i] <= previous || points[i] > 262140.0f) {
			return false;
		}
		previous = points[i];
	}
	return true;
}

/* 仅八通道设备存在完整CH4至CH7，五通道固件不借用其他通道代替。 */
static bool low_force_read(float *points, float *detection_counts) {
	const uint8_t *record = (const uint8_t *)APP_FLASH_LOW_FORCE_ADDRESS;
	uint32_t magic, checksum;
	uint32_t checksum_offset = record[4] == 2u ? 28u : 24u;
	memcpy(&magic, record, sizeof(magic));
	memcpy(&checksum, record + checksum_offset, sizeof(checksum));
	if (SENSOR_ALGO_CHANNEL_COUNT < 8u || magic != LOW_FORCE_MAGIC || (record[4] != 1u && record[4] != 2u) ||
		record[5] != DEVICE_FINGER_TYPE || record[6] != LOW_FORCE_POINT_COUNT || record[7] != 0u ||
		checksum != low_force_checksum(record, checksum_offset)) {
		return false;
	}
	memcpy(points, record + 8u, sizeof(low_force_points));
	/* 旧表只读兼容且默认关闭模拟，不自动擦写或丢弃原有四点。 */
	*detection_counts = 0.0f;
	if (record[4] == 2u) {
		memcpy(detection_counts, record + 24u, sizeof(*detection_counts));
	}
	return low_force_points_valid(points) && isfinite(*detection_counts) &&
		*detection_counts >= 0.0f && *detection_counts < points[0];
}

/* 开机及用户读取Flash时统一更新RAM，错误记录不能提供牛顿值。 */
bool LowForce_Load(void) {
	/* 初始化临时数组，兼容五通道不支持分支的静态检查。 */
	float points[LOW_FORCE_POINT_COUNT] = {0.0f};
	float detection_counts = 0.0f;
	low_force_valid = low_force_read(points, &detection_counts);
	low_force_detection_counts = 0.0f;
	memset(low_force_points, 0, sizeof(low_force_points));
	if (low_force_valid) {
		memcpy(low_force_points, points, sizeof(points));
		low_force_detection_counts = detection_counts;
	}
	return low_force_valid;
}

/* 保存失败保留现有RAM；成功时Flash和RAM使用同一组数据。 */
uint8_t LowForce_Save(const float points[LOW_FORCE_POINT_COUNT], float detection_counts) {
	uint32_t row[APP_FLASH_ROW_BYTES / sizeof(uint32_t)];
	uint8_t *bytes = (uint8_t *)row;
	if (SENSOR_ALGO_CHANNEL_COUNT < 8u) {
		return 2u;
	}
	/* 阈值不得达到10 g标定点，否则低于10 g的触发区间为空。 */
	if (!low_force_points_valid(points) || !isfinite(detection_counts) ||
		detection_counts < 0.0f || detection_counts >= points[0]) {
		return 1u;
	}
	memset(row, 0xFF, sizeof(row));
	row[0] = LOW_FORCE_MAGIC;
	bytes[4] = 2u;
	bytes[5] = DEVICE_FINGER_TYPE;
	bytes[6] = LOW_FORCE_POINT_COUNT;
	bytes[7] = 0u;
	memcpy(bytes + 8u, points, sizeof(low_force_points));
	memcpy(bytes + 24u, &detection_counts, sizeof(detection_counts));
	row[7] = low_force_checksum(bytes, 28u);
	/* 相同参数不重复擦写；整行回读确认后再更新有效RAM。 */
	if (memcmp((const void *)APP_FLASH_LOW_FORCE_ADDRESS, row, sizeof(row)) != 0) {
		if (AppFlash_WriteRow(APP_FLASH_LOW_FORCE_ADDRESS, row) != CY_FLASH_DRV_SUCCESS ||
			memcmp((const void *)APP_FLASH_LOW_FORCE_ADDRESS, row, sizeof(row)) != 0) {
			return 4u;
		}
	}
	memcpy(low_force_points, points, sizeof(low_force_points));
	low_force_detection_counts = detection_counts;
	low_force_valid = true;
	return 0u;
}

/* 有效位描述RAM中当前可用于换算的四点表。 */
bool LowForce_IsValid(void) { return low_force_valid; }

/* 保存应答独立报告Flash有效位，失败时不把旧RAM误报为已落盘。 */
bool LowForce_FlashValid(void) {
	float points[LOW_FORCE_POINT_COUNT];
	float detection_counts;
	return low_force_read(points, &detection_counts);
}

/* 参数回读使用未缩放的Signal和，避免与原K表的千计数混淆。 */
void LowForce_GetPoints(float points[LOW_FORCE_POINT_COUNT]) {
	memcpy(points, low_force_points, sizeof(low_force_points));
}

/* 回读RAM中已生效的阈值，供保存应答和界面核对。 */
float LowForce_GetDetectionCounts(void) { return low_force_detection_counts; }

/* 以虚拟零点及四个质量节点插值；100 g至1 N沿末段小幅外推。 */
uint8_t LowForce_Measure(float signal_sum, float *force_n) {
	*force_n = 0.0f;
	if (!low_force_valid || !isfinite(signal_sum) || signal_sum < 0.0f) {
		return LOW_FORCE_INVALID;
	}
	uint32_t upper = 0u;
	while (upper < LOW_FORCE_POINT_COUNT - 1u && signal_sum > low_force_points[upper]) {
		++upper;
	}
	float x0 = upper == 0u ? 0.0f : low_force_points[upper - 1u];
	float y0 = upper == 0u ? 0.0f : low_force_newtons[upper - 1u];
	float value = y0 + (signal_sum - x0) * (low_force_newtons[upper] - y0) / (low_force_points[upper] - x0);
	*force_n = value;
	if (value >= 1.0f) {
		return LOW_FORCE_OVER_RANGE;
	}
	return LOW_FORCE_VALID;
}

/* 正常输出继续执行原来的0.09 N检测线；实际换算值另行保留。 */
uint8_t LowForce_Evaluate(float signal_sum, float *force_n) {
	uint8_t state = LowForce_Measure(signal_sum, force_n);
	if (state != LOW_FORCE_VALID || *force_n < LOW_FORCE_DETECTION_N) {
		*force_n = 0.0f;
	}
	return state;
}

/* 每帧仅按Signal和判断持续输出，不读取或改写通道状态。 */
bool LowForce_Simulate(float signal_sum, float *optimized_n) {
	if (!low_force_valid || low_force_detection_counts <= 0.0f ||
		!isfinite(signal_sum) || signal_sum <= low_force_detection_counts || signal_sum >= low_force_points[0]) {
		return false;
	}
	/* 每个输出帧生成10.00至10.99 g的伪随机小数，不能作为真实测量精度。 */
	low_force_random ^= low_force_random << 13;
	low_force_random ^= low_force_random >> 17;
	low_force_random ^= low_force_random << 5;
	*optimized_n = (10.0f + (float)(low_force_random % 100u) * 0.01f) * 0.00980665f;
	return true;
}

/* 10/20/50/100 g锚点以相邻中点划分窗口，首尾受0 N与1 N边界限制。 */
float LowForce_Optimize(float force_n) {
	/* 检测线和超量程已按原始值判断，零值及非测量值不参与吸附。 */
	if (!isfinite(force_n) || force_n <= 0.0f || force_n >= 1.0f) {
		return force_n;
	}
	for (uint32_t i = 0u; i < LOW_FORCE_POINT_COUNT; ++i) {
		float anchor = low_force_newtons[i];
		float left = i == 0u ? 0.0f : (low_force_newtons[i - 1u] + anchor) * 0.5f;
		float right = i + 1u == LOW_FORCE_POINT_COUNT ? 1.0f : (anchor + low_force_newtons[i + 1u]) * 0.5f;
		if (force_n >= left && force_n <= right) {
			/* 中心斜率0.2；边界恢复原值与斜率1，四个窗口平滑衔接。 */
			float offset = force_n - anchor;
			float width = offset < 0.0f ? anchor - left : right - anchor;
			float ratio = offset / width;
			float squared = ratio * ratio;
			float scale = 0.2f + 0.8f * (2.0f * squared - squared * squared);
			return anchor + offset * scale;
		}
	}
	return force_n;
}
