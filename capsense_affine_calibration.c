#include "capsense_affine_calibration.h"

/* 仿射预留地址及写入必须遵守统一保护边界。 */
#include "app_flash.h"
/* 由设备功能开关同时控制仿射计算与持久化入口。 */
#include "device_variant.h"

/* Flash行大小和写入接口来自PSoC PDL。 */
#include "cy_pdl.h"

/* 清零运行时参数使用标准内存函数。 */
#include <string.h>

/* 地址前移到0x1FC00，禁止读取或写入旧DFU尾部区域。 */
#define CAPSENSE_AFFINE_STORAGE_ADDRESS APP_FLASH_AFFINE_ADDRESS

/* Flash记录格式独立版本，后续改变布局时必须递增。 */
#define CAPSENSE_AFFINE_STORAGE_VERSION (1u)

/* 当前中三指设备类型固定为1。 */
#define CAPSENSE_AFFINE_DEVICE_TYPE (1u)

/* 定义Flash记录中各字段的固定偏移。 */
#define CAPSENSE_AFFINE_STORAGE_CALIBRATION_ID_OFFSET (8u)
#define CAPSENSE_AFFINE_STORAGE_GENERATION_OFFSET (12u)
#define CAPSENSE_AFFINE_STORAGE_REF_TABLE_OFFSET (16u)
#define CAPSENSE_AFFINE_STORAGE_COEFFICIENT_OFFSET (24u)
#define CAPSENSE_AFFINE_STORAGE_CHECKSUM_OFFSET (248u)
#define CAPSENSE_AFFINE_STORAGE_DATA_SIZE CAPSENSE_AFFINE_STORAGE_CHECKSUM_OFFSET

/* 定义参数取值保护范围，避免后续应用时发生明显溢出。 */
#define CAPSENSE_AFFINE_GAIN_Q16_MAX (16L * 65536L)
#define CAPSENSE_AFFINE_OFFSET_ABS_MAX (1000000L)

/* 定义Flash读写结果之外的内部校验错误码。 */
#define CAPSENSE_AFFINE_NVM_STATUS_NOT_LOADED (0xFFFFFFF0u)
#define CAPSENSE_AFFINE_NVM_STATUS_RECORD_INVALID (0xFFFFFFF1u)
#define CAPSENSE_AFFINE_NVM_STATUS_VERIFY_FAILED (0xFFFFFFF2u)

/* 编译期确认完整记录不会超过一个Flash行。 */
#if ((CAPSENSE_AFFINE_STORAGE_CHECKSUM_OFFSET + 4u) > CY_FLASH_SIZEOF_ROW)
#error "CapSense affine calibration record exceeds one flash row"
#endif

/* 固定档位表同时写入Flash，用于阻止错误固件解释旧记录。 */
const uint8_t capsense_affine_ref_cdac_tiers[CAPSENSE_AFFINE_TIER_COUNT] = {
	12u, 18u, 27u, 40u, 60u, 90u, 135u,
};

/* 保存一整组四通道参数的RAM表示。 */
typedef struct {
	uint32_t calibration_id;
	uint32_t generation;
	int32_t gain_q16[CAPSENSE_AFFINE_CHANNEL_COUNT][CAPSENSE_AFFINE_TIER_COUNT];
	int32_t offset[CAPSENSE_AFFINE_CHANNEL_COUNT][CAPSENSE_AFFINE_TIER_COUNT];
} capsense_affine_record_t;

/* 保存已从Flash确认的当前记录。 */
static capsense_affine_record_t active_record;

/* 保存正在接收且尚未写Flash的新记录。 */
static capsense_affine_record_t staging_record;

/* 每一位表示对应通道的新参数已经完整收到。 */
static uint8_t staging_channel_mask;

/* 标记当前记录是否通过全部格式和校验检查。 */
static bool active_record_valid;

/* 标记当前记录是否来自Flash加载或写后回读。 */
static bool active_record_loaded_from_flash;

/* 保存最近一次Flash操作或记录校验结果。 */
static uint32_t last_nvm_status;

/* 从字节缓冲区读取小端32位无符号整数。 */
static uint32_t read_u32_le(const volatile uint8_t *data) {
	return (uint32_t)data[0u] |
		((uint32_t)data[1u] << 8u) |
		((uint32_t)data[2u] << 16u) |
		((uint32_t)data[3u] << 24u);
}

/* 从普通字节缓冲区读取小端32位有符号整数。 */
static int32_t read_i32_le(const volatile uint8_t *data) {
	return (int32_t)read_u32_le(data);
}

/* 向字节缓冲区写入小端32位无符号整数。 */
static void write_u32_le(uint8_t *data, uint32_t value) {
	data[0u] = (uint8_t)value;
	data[1u] = (uint8_t)(value >> 8u);
	data[2u] = (uint8_t)(value >> 16u);
	data[3u] = (uint8_t)(value >> 24u);
}

/* 向字节缓冲区写入小端32位有符号整数。 */
static void write_i32_le(uint8_t *data, int32_t value) {
	write_u32_le(data, (uint32_t)value);
}

/* 对Flash记录数据计算FNV-1a校验值。 */
static uint32_t calculate_storage_checksum(const uint8_t *data, uint32_t size) {
	uint32_t checksum = 2166136261u;
	uint32_t index;

	/* 逐字节更新校验值，覆盖头部、档位表和全部参数。 */
	for (index = 0u; index < size; index++) {
		checksum ^= data[index];
		checksum *= 16777619u;
	}

	/* 返回最终记录校验值。 */
	return checksum;
}

/* 把一个32位值按协议小端顺序加入参数ID。 */
static void update_parameter_id_i32(uint32_t *checksum, int32_t value) {
	uint32_t unsigned_value = (uint32_t)value;
	uint32_t byte_index;

	/* 参数ID必须与Python端逐字节计算顺序完全一致。 */
	for (byte_index = 0u; byte_index < 4u; byte_index++) {
		*checksum ^= (uint8_t)(unsigned_value >> (byte_index * 8u));
		*checksum *= 16777619u;
	}
}

/* 根据固定通道、档位和参数顺序计算整组参数ID。 */
static uint32_t calculate_parameter_id(const capsense_affine_record_t *record) {
	uint32_t checksum = 2166136261u;
	uint32_t channel_index;
	uint32_t tier_index;

	/* 通道优先排列，保证固件和Python生成相同ID。 */
	for (channel_index = 0u; channel_index < CAPSENSE_AFFINE_CHANNEL_COUNT; channel_index++) {
		/* 将物理通道号纳入参数ID。 */
		checksum ^= (uint8_t)(CAPSENSE_AFFINE_CHANNEL_FIRST + channel_index);
		checksum *= 16777619u;

		/* 依次加入档位号、Q16增益和整数偏移。 */
		for (tier_index = 0u; tier_index < CAPSENSE_AFFINE_TIER_COUNT; tier_index++) {
			checksum ^= capsense_affine_ref_cdac_tiers[tier_index];
			checksum *= 16777619u;
			update_parameter_id_i32(&checksum, record->gain_q16[channel_index][tier_index]);
			update_parameter_id_i32(&checksum, record->offset[channel_index][tier_index]);
		}
	}

	/* 返回用于识别整组参数内容的ID。 */
	return checksum;
}

/* 检查单个通道的七组定点参数是否处于保护范围内。 */
static bool channel_values_valid(
	const int32_t gain_q16[CAPSENSE_AFFINE_TIER_COUNT],
	const int32_t offset[CAPSENSE_AFFINE_TIER_COUNT]) {
	uint32_t tier_index;

	/* 拒绝负增益、过大增益和明显异常偏移。 */
	for (tier_index = 0u; tier_index < CAPSENSE_AFFINE_TIER_COUNT; tier_index++) {
		if ((gain_q16[tier_index] <= 0) ||
			(gain_q16[tier_index] > CAPSENSE_AFFINE_GAIN_Q16_MAX) ||
			(offset[tier_index] < -CAPSENSE_AFFINE_OFFSET_ABS_MAX) ||
			(offset[tier_index] > CAPSENSE_AFFINE_OFFSET_ABS_MAX)) {
			return false;
		}
	}

	/* Ref60必须保持单位映射，防止通道或档位排列错误。 */
	if ((gain_q16[4u] != 65536L) || (offset[4u] != 0L)) {
		return false;
	}

	/* 所有参数均通过检查。 */
	return true;
}

/* 检查完整四通道记录及其参数ID。 */
static bool record_values_valid(const capsense_affine_record_t *record) {
	uint32_t channel_index;

	/* 参数ID不得为保留的空值。 */
	if (record->calibration_id == 0u) {
		return false;
	}

	/* 每个通道都必须通过相同的范围和Ref60检查。 */
	for (channel_index = 0u; channel_index < CAPSENSE_AFFINE_CHANNEL_COUNT; channel_index++) {
		if (!channel_values_valid(record->gain_q16[channel_index], record->offset[channel_index])) {
			return false;
		}
	}

	/* 参数ID必须与实际224字节参数完全一致。 */
	return calculate_parameter_id(record) == record->calibration_id;
}

/* 从Flash行解析并验证一整组参数。 */
static bool load_record_from_flash(capsense_affine_record_t *record) {
	const volatile uint8_t *storage =
		(const volatile uint8_t *)CAPSENSE_AFFINE_STORAGE_ADDRESS;
	uint8_t checksum_data[CAPSENSE_AFFINE_STORAGE_DATA_SIZE];
	uint32_t stored_checksum;
	uint32_t calculated_checksum;
	uint32_t channel_index;
	uint32_t tier_index;
	uint32_t offset;

	/* 先复制受校验保护的记录部分，避免直接对volatile区域反复计算。 */
	for (offset = 0u; offset < CAPSENSE_AFFINE_STORAGE_DATA_SIZE; offset++) {
		checksum_data[offset] = storage[offset];
	}

	/* 校验魔数、版本、设备类型和固定维度。 */
	if ((checksum_data[0u] != 'A') ||
		(checksum_data[1u] != 'R') ||
		(checksum_data[2u] != 'A') ||
		(checksum_data[3u] != 'F') ||
		(checksum_data[4u] != CAPSENSE_AFFINE_STORAGE_VERSION) ||
		(checksum_data[5u] != CAPSENSE_AFFINE_DEVICE_TYPE) ||
		(checksum_data[6u] != CAPSENSE_AFFINE_CHANNEL_COUNT) ||
		(checksum_data[7u] != CAPSENSE_AFFINE_TIER_COUNT)) {
		return false;
	}

	/* 校验记录内保存的七档Ref顺序。 */
	for (tier_index = 0u; tier_index < CAPSENSE_AFFINE_TIER_COUNT; tier_index++) {
		if (checksum_data[CAPSENSE_AFFINE_STORAGE_REF_TABLE_OFFSET + tier_index] !=
			capsense_affine_ref_cdac_tiers[tier_index]) {
			return false;
		}
	}

	/* 校验整个记录的数据完整性。 */
	stored_checksum = read_u32_le(&storage[CAPSENSE_AFFINE_STORAGE_CHECKSUM_OFFSET]);
	calculated_checksum = calculate_storage_checksum(
		checksum_data,
		CAPSENSE_AFFINE_STORAGE_DATA_SIZE);
	if (stored_checksum != calculated_checksum) {
		return false;
	}

	/* 解析记录标识和保存代次。 */
	record->calibration_id = read_u32_le(
		&storage[CAPSENSE_AFFINE_STORAGE_CALIBRATION_ID_OFFSET]);
	record->generation = read_u32_le(
		&storage[CAPSENSE_AFFINE_STORAGE_GENERATION_OFFSET]);

	/* 按通道和档位顺序解析全部28组参数。 */
	offset = CAPSENSE_AFFINE_STORAGE_COEFFICIENT_OFFSET;
	for (channel_index = 0u; channel_index < CAPSENSE_AFFINE_CHANNEL_COUNT; channel_index++) {
		for (tier_index = 0u; tier_index < CAPSENSE_AFFINE_TIER_COUNT; tier_index++) {
			record->gain_q16[channel_index][tier_index] = read_i32_le(&storage[offset]);
			offset += 4u;
			record->offset[channel_index][tier_index] = read_i32_le(&storage[offset]);
			offset += 4u;
		}
	}

	/* 最后校验全部参数和内容ID。 */
	return record_values_valid(record);
}

/* 把完整记录编码为一个Flash行。 */
static void build_flash_row(
	const capsense_affine_record_t *record,
	uint32_t row_words[CY_FLASH_SIZEOF_ROW / sizeof(uint32_t)]) {
	uint8_t *row = (uint8_t *)row_words;
	uint32_t checksum;
	uint32_t channel_index;
	uint32_t tier_index;
	uint32_t offset;

	/* 未使用字节保持擦除态，便于后续扩展记录格式。 */
	memset(row, 0xFF, CY_FLASH_SIZEOF_ROW);

	/* 写入固定记录头。 */
	row[0u] = 'A';
	row[1u] = 'R';
	row[2u] = 'A';
	row[3u] = 'F';
	row[4u] = CAPSENSE_AFFINE_STORAGE_VERSION;
	row[5u] = CAPSENSE_AFFINE_DEVICE_TYPE;
	row[6u] = CAPSENSE_AFFINE_CHANNEL_COUNT;
	row[7u] = CAPSENSE_AFFINE_TIER_COUNT;
	write_u32_le(&row[CAPSENSE_AFFINE_STORAGE_CALIBRATION_ID_OFFSET], record->calibration_id);
	write_u32_le(&row[CAPSENSE_AFFINE_STORAGE_GENERATION_OFFSET], record->generation);

	/* 写入固定档位表。 */
	for (tier_index = 0u; tier_index < CAPSENSE_AFFINE_TIER_COUNT; tier_index++) {
		row[CAPSENSE_AFFINE_STORAGE_REF_TABLE_OFFSET + tier_index] =
			capsense_affine_ref_cdac_tiers[tier_index];
	}

	/* 写入通道优先排列的28组参数。 */
	offset = CAPSENSE_AFFINE_STORAGE_COEFFICIENT_OFFSET;
	for (channel_index = 0u; channel_index < CAPSENSE_AFFINE_CHANNEL_COUNT; channel_index++) {
		for (tier_index = 0u; tier_index < CAPSENSE_AFFINE_TIER_COUNT; tier_index++) {
			write_i32_le(&row[offset], record->gain_q16[channel_index][tier_index]);
			offset += 4u;
			write_i32_le(&row[offset], record->offset[channel_index][tier_index]);
			offset += 4u;
		}
	}

	/* 记录末尾保存覆盖前248字节的FNV校验值。 */
	checksum = calculate_storage_checksum(row, CAPSENSE_AFFINE_STORAGE_DATA_SIZE);
	write_u32_le(&row[CAPSENSE_AFFINE_STORAGE_CHECKSUM_OFFSET], checksum);
}

/* 启动时读取Flash，失败时保持参数无效且不影响现有算法。 */
void CapSenseAffineCalibration_Init(void) {
	/* 清除RAM中的当前记录和未完成暂存。 */
	memset(&active_record, 0, sizeof(active_record));
	memset(&staging_record, 0, sizeof(staging_record));
	staging_channel_mask = 0u;
	active_record_valid = false;
	active_record_loaded_from_flash = false;
	last_nvm_status = CAPSENSE_AFFINE_NVM_STATUS_NOT_LOADED;
	/* 停用时不读取任何Flash记录，也不使旧参数参与Raw计算。 */
	if (!DEVICE_HAS_AFFINE_CALIBRATION) {
		return;
	}

	/* 有效Flash记录只加载到RAM，当前阶段不参与Raw计算。 */
	if (load_record_from_flash(&active_record)) {
		active_record_valid = true;
		active_record_loaded_from_flash = true;
		last_nvm_status = (uint32_t)CY_FLASH_DRV_SUCCESS;
	} else {
		last_nvm_status = CAPSENSE_AFFINE_NVM_STATUS_RECORD_INVALID;
	}
}

/* 暂存一个通道，参数ID变化时自动开始全新的四通道事务。 */
bool CapSenseAffineCalibration_StageChannel(
	uint32_t calibration_id,
	uint8_t channel,
	uint8_t format_version,
	uint8_t tier_count,
	const int32_t gain_q16[CAPSENSE_AFFINE_TIER_COUNT],
	const int32_t offset[CAPSENSE_AFFINE_TIER_COUNT]) {
	uint32_t channel_index;

	/* 校验协议格式、物理通道和本通道参数。 */
	if (!DEVICE_HAS_AFFINE_CALIBRATION || (calibration_id == 0u) ||
		(format_version != CAPSENSE_AFFINE_FORMAT_VERSION) ||
		(tier_count != CAPSENSE_AFFINE_TIER_COUNT) ||
		(channel < CAPSENSE_AFFINE_CHANNEL_FIRST) ||
		(channel >= (CAPSENSE_AFFINE_CHANNEL_FIRST + CAPSENSE_AFFINE_CHANNEL_COUNT)) ||
		!channel_values_valid(gain_q16, offset)) {
		return false;
	}

	/* 新参数ID表示开始新事务，清除上一组未完成通道。 */
	if ((staging_channel_mask == 0u) ||
		(staging_record.calibration_id != calibration_id)) {
		memset(&staging_record, 0, sizeof(staging_record));
		staging_record.calibration_id = calibration_id;
		staging_channel_mask = 0u;
	}

	/* 保存本通道七组参数到暂存区。 */
	channel_index = (uint32_t)(channel - CAPSENSE_AFFINE_CHANNEL_FIRST);
	memcpy(
		staging_record.gain_q16[channel_index],
		gain_q16,
		sizeof(staging_record.gain_q16[channel_index]));
	memcpy(
		staging_record.offset[channel_index],
		offset,
		sizeof(staging_record.offset[channel_index]));

	/* 标记本通道到齐，重复下发同一通道可安全覆盖。 */
	staging_channel_mask |= (uint8_t)(1u << channel_index);
	return true;
}

/* 将完整暂存事务写入Flash，并以Flash实际内容作为当前记录。 */
bool CapSenseAffineCalibration_Save(uint32_t calibration_id) {
	uint32_t row_words[CY_FLASH_SIZEOF_ROW / sizeof(uint32_t)];
	capsense_affine_record_t verified_record;
	cy_en_flashdrv_status_t flash_status;
	uint32_t next_generation;

	/* 固件内部直接调用也不能绕过停用开关。 */
	if (!DEVICE_HAS_AFFINE_CALIBRATION) {
		return false;
	}

	/* 必须收到同一参数ID的全部四个通道。 */
	if ((calibration_id == 0u) ||
		(staging_record.calibration_id != calibration_id) ||
		(staging_channel_mask != ((1u << CAPSENSE_AFFINE_CHANNEL_COUNT) - 1u))) {
		return false;
	}

	/* 重新计算内容ID，检测串口漏包、错序和内容破坏。 */
	if (!record_values_valid(&staging_record)) {
		return false;
	}

	/* 每次成功保存递增代次，溢出后从1重新开始。 */
	next_generation = active_record_valid ? (active_record.generation + 1u) : 1u;
	if (next_generation == 0u) {
		next_generation = 1u;
	}
	staging_record.generation = next_generation;

	/* 构造完整Flash行并执行单行写入。 */
	build_flash_row(&staging_record, row_words);
	/* 统一写入入口再次检查整行不越过DFU保护边界。 */
	flash_status = AppFlash_WriteRow(CAPSENSE_AFFINE_STORAGE_ADDRESS, row_words);
	last_nvm_status = (uint32_t)flash_status;
	if (flash_status != CY_FLASH_DRV_SUCCESS) {
		return false;
	}

	/* 写后立即从Flash重新解析，防止只依据驱动返回值误判成功。 */
	memset(&verified_record, 0, sizeof(verified_record));
	if (!load_record_from_flash(&verified_record) ||
		(verified_record.calibration_id != calibration_id) ||
		(memcmp(&verified_record, &staging_record, sizeof(verified_record)) != 0)) {
		last_nvm_status = CAPSENSE_AFFINE_NVM_STATUS_VERIFY_FAILED;
		return false;
	}

	/* 只有回读完全一致才替换当前RAM记录。 */
	active_record = verified_record;
	active_record_valid = true;
	active_record_loaded_from_flash = true;
	last_nvm_status = (uint32_t)CY_FLASH_DRV_SUCCESS;

	/* 保存成功后结束暂存事务。 */
	memset(&staging_record, 0, sizeof(staging_record));
	staging_channel_mask = 0u;
	return true;
}

/* 回传指定通道的参数和全局存储状态。 */
bool CapSenseAffineCalibration_GetChannelStatus(
	uint8_t channel,
	capsense_affine_channel_status_t *status) {
	uint32_t channel_index;

	/* 拒绝空指针和CH4至CH7之外的通道。 */
	if ((status == NULL) ||
		(channel < CAPSENSE_AFFINE_CHANNEL_FIRST) ||
		(channel >= (CAPSENSE_AFFINE_CHANNEL_FIRST + CAPSENSE_AFFINE_CHANNEL_COUNT))) {
		return false;
	}

	/* 先清空输出，保证无有效记录时不会泄漏旧栈数据。 */
	memset(status, 0, sizeof(*status));
	status->nvm_status = last_nvm_status;

	/* 汇总当前记录和暂存事务状态。 */
	if (active_record_valid) {
		status->flags |= CAPSENSE_AFFINE_FLAG_VALID;
	}
	if (active_record_loaded_from_flash) {
		status->flags |= CAPSENSE_AFFINE_FLAG_LOADED_FROM_FLASH;
	}
	if (staging_channel_mask != 0u) {
		status->flags |= CAPSENSE_AFFINE_FLAG_STAGING_ACTIVE;
	}
	if (staging_channel_mask == ((1u << CAPSENSE_AFFINE_CHANNEL_COUNT) - 1u)) {
		status->flags |= CAPSENSE_AFFINE_FLAG_STAGING_COMPLETE;
	}

	/* 无有效Flash记录时仍返回状态帧，但参数保持为零。 */
	if (!active_record_valid) {
		return true;
	}

	/* 复制指定通道的当前有效记录。 */
	channel_index = (uint32_t)(channel - CAPSENSE_AFFINE_CHANNEL_FIRST);
	status->calibration_id = active_record.calibration_id;
	status->generation = active_record.generation;
	memcpy(
		status->gain_q16,
		active_record.gain_q16[channel_index],
		sizeof(status->gain_q16));
	memcpy(
		status->offset,
		active_record.offset[channel_index],
		sizeof(status->offset));
	return true;
}

/* 使用指定通道和Ref档位的Q16仿射参数计算Norm Raw。 */
bool CapSenseAffineCalibration_Apply(
	uint8_t channel,
	uint8_t ref_cdac,
	uint16_t hardware_raw,
	uint32_t *normalized_raw) {
	uint32_t channel_index;
	uint32_t tier_index;
	int64_t normalized_q16;
	uint64_t rounded_normalized;

	/* 功能停用时不应用仿射；启用后仍要求有效记录和合法输入。 */
	if (!DEVICE_HAS_AFFINE_CALIBRATION || !active_record_valid ||
		(normalized_raw == NULL) ||
		(channel < CAPSENSE_AFFINE_CHANNEL_FIRST) ||
		(channel >= (CAPSENSE_AFFINE_CHANNEL_FIRST + CAPSENSE_AFFINE_CHANNEL_COUNT))) {
		return false;
	}

	/* 在固定七档Ref表中查找当前硬件档位。 */
	for (tier_index = 0u; tier_index < CAPSENSE_AFFINE_TIER_COUNT; tier_index++) {
		if (capsense_affine_ref_cdac_tiers[tier_index] == ref_cdac) {
			break;
		}
	}

	/* 拒绝未收录的Ref，由调用方执行理论比例回退。 */
	if (tier_index >= CAPSENSE_AFFINE_TIER_COUNT) {
		return false;
	}

	/* 按物理通道取出对应的七档参数。 */
	channel_index = (uint32_t)(channel - CAPSENSE_AFFINE_CHANNEL_FIRST);

	/* 使用64位中间值完成Raw乘Q16增益再加整数偏移，避免溢出。 */
	normalized_q16 =
		((int64_t)hardware_raw * active_record.gain_q16[channel_index][tier_index]) +
		((int64_t)active_record.offset[channel_index][tier_index] * 65536LL);

	/* 负的仿射结果限制为零，防止转换成无符号数后回绕。 */
	if (normalized_q16 <= 0LL) {
		*normalized_raw = 0u;
		return true;
	}

	/* 对正的Q16结果执行四舍五入并还原成整数counts。 */
	rounded_normalized = ((uint64_t)normalized_q16 + 32768u) >> 16u;

	/* 异常大的结果限制到32位上限，保持后续协议语义稳定。 */
	if (rounded_normalized > UINT32_MAX) {
		*normalized_raw = UINT32_MAX;
	} else {
		*normalized_raw = (uint32_t)rounded_normalized;
	}

	/* 返回真表示本帧已使用持久化仿射参数。 */
	return true;
}
