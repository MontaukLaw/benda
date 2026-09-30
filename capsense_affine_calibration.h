#ifndef CAPSENSE_AFFINE_CALIBRATION_H
#define CAPSENSE_AFFINE_CALIBRATION_H

/*
 * 提供Ref CDAC仿射参数的暂存、Flash持久化和启动回读接口。
 * 当前阶段只保存和回传参数，不把参数接入Raw归一化计算。
 */

#include <stdbool.h>
#include <stdint.h>

/* 固定保存CH4至CH7四个通道。 */
#define CAPSENSE_AFFINE_CHANNEL_FIRST (4u)
#define CAPSENSE_AFFINE_CHANNEL_COUNT (4u)

/* 固定保存七个Ref CDAC档位。 */
#define CAPSENSE_AFFINE_TIER_COUNT (7u)

/* 定义Q16增益加整数偏移的参数格式版本。 */
#define CAPSENSE_AFFINE_FORMAT_VERSION (1u)

/* 标记当前RAM记录已经通过Flash校验加载。 */
#define CAPSENSE_AFFINE_FLAG_VALID (1u << 0u)

/* 标记当前RAM记录来自Flash启动加载或刚完成的Flash回读。 */
#define CAPSENSE_AFFINE_FLAG_LOADED_FROM_FLASH (1u << 1u)

/* 标记上位机正在暂存一组尚未保存的新参数。 */
#define CAPSENSE_AFFINE_FLAG_STAGING_ACTIVE (1u << 2u)

/* 标记四个通道的新参数已经全部到齐。 */
#define CAPSENSE_AFFINE_FLAG_STAGING_COMPLETE (1u << 3u)

/* 使用固定档位顺序解释每个通道的七组参数。 */
extern const uint8_t capsense_affine_ref_cdac_tiers[CAPSENSE_AFFINE_TIER_COUNT];

/* 返回单个通道的当前持久化参数和存储状态。 */
typedef struct {
	uint32_t calibration_id;
	uint32_t generation;
	uint32_t nvm_status;
	uint8_t flags;
	int32_t gain_q16[CAPSENSE_AFFINE_TIER_COUNT];
	int32_t offset[CAPSENSE_AFFINE_TIER_COUNT];
} capsense_affine_channel_status_t;

/* 启动时校验Flash记录并加载到RAM。 */
void CapSenseAffineCalibration_Init(void);

/* 暂存一个通道的七组参数，不写Flash也不应用。 */
bool CapSenseAffineCalibration_StageChannel(
	uint32_t calibration_id,
	uint8_t channel,
	uint8_t format_version,
	uint8_t tier_count,
	const int32_t gain_q16[CAPSENSE_AFFINE_TIER_COUNT],
	const int32_t offset[CAPSENSE_AFFINE_TIER_COUNT]);

/* 四个通道到齐后写入一个带完整性校验的Flash行并立即回读。 */
bool CapSenseAffineCalibration_Save(uint32_t calibration_id);

/* 读取指定物理通道的当前参数，供协议回传。 */
bool CapSenseAffineCalibration_GetChannelStatus(
	uint8_t channel,
	capsense_affine_channel_status_t *status);

/* 使用Flash确认的当前档位仿射参数将Hardware Raw换算为Norm Raw。 */
bool CapSenseAffineCalibration_Apply(
	uint8_t channel,
	uint8_t ref_cdac,
	uint16_t hardware_raw,
	uint32_t *normalized_raw);

#endif
