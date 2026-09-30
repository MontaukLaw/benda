#ifndef CAPSENSE_AUTORANGE_H
#define CAPSENSE_AUTORANGE_H

#include <stdbool.h>
#include <stdint.h>

/* 自动量程只控制矩阵传感器CH4至CH7，协议仍传输全部8路数据。 */
#define CAPSENSE_AUTORANGE_CHANNEL_FIRST (4u)
#define CAPSENSE_AUTORANGE_CHANNEL_COUNT (4u)
#define CAPSENSE_AUTORANGE_TOTAL_CHANNEL_COUNT (8u)

/* Nsub和TxDiv固定不变，Ref从12起步，归一化计数以Ref=60为基准。 */
#define CAPSENSE_AUTORANGE_FIXED_NSUB (300u)
#define CAPSENSE_AUTORANGE_FIXED_TX_DIV (100u)
#define CAPSENSE_AUTORANGE_MAX_COUNT (58928u)
#define CAPSENSE_AUTORANGE_START_REF (12u)
#define CAPSENSE_AUTORANGE_NORMALIZATION_REF (60u)

/* 自动量程状态同时用于内部控制和上位机诊断显示。 */
typedef enum {
	/* 上电初始化完成，尚未进入正常保持状态。 */
	CAPSENSE_AUTORANGE_STATE_STARTUP = 0u,
	/* 当前Raw位于迟滞区间，保持现有档位。 */
	CAPSENSE_AUTORANGE_STATE_HOLD = 1u,
	/* 已产生扩档请求，等待扫描空闲点应用。 */
	CAPSENSE_AUTORANGE_STATE_UP_PENDING = 2u,
	/* 已产生降档请求，等待扫描空闲点应用。 */
	CAPSENSE_AUTORANGE_STATE_DOWN_PENDING = 3u,
	/* 新Ref已经写入，正在执行速度预测或Hermite平滑衔接。 */
	CAPSENSE_AUTORANGE_STATE_SWITCHING = 4u,
	/* 稳定等待结束后的档位冷却阶段。 */
	CAPSENSE_AUTORANGE_STATE_COOLDOWN = 5u,
	/* 已到最低档或最高档，无法继续切换。 */
	CAPSENSE_AUTORANGE_STATE_LIMIT = 6u,
	/* 固定参数校验或Ref写入失败。 */
	CAPSENSE_AUTORANGE_STATE_ERROR = 7u,
	/* 手动校准模式已经稳定在上位机指定的Ref档位。 */
	CAPSENSE_AUTORANGE_STATE_MANUAL_HOLD = 8u,
} capsense_autorange_state_t;

/* 上位机0x28状态帧直接使用这些字段进行序列化。 */
typedef struct {
	/* 固定采样参数和理论最大计数。 */
	uint16_t nsub;
	uint16_t tx_div;
	uint16_t max_count;
	/* 当前Ref、请求Ref、档位编号、总档数、控制状态和诊断标志。 */
	uint8_t actual_ref_cdac;
	uint8_t requested_ref_cdac;
	uint8_t tier_index;
	uint8_t tier_count;
	uint8_t state;
	uint8_t flags;
	/* 本帧峰值和用于普通升降档的滤波峰值。 */
	uint16_t peak_raw;
	uint16_t filtered_peak_raw;
	/* 升降档迟滞计数、剩余冷却帧数以及切档过渡状态。 */
	uint8_t high_count;
	uint8_t low_count;
	uint8_t cooldown_frames;
	/* 候选等待或Hermite衔接期间表示剩余帧数，切档过渡完成后为0。 */
	uint8_t blank_frames;
} capsense_autorange_status_t;

/* 初始化自动量程并校验Nsub、TxDiv和起始Ref。 */
bool CapSenseAutoRange_Init(void);

/* 处理一帧自动量程，并分别输出最终Norm Raw和未经过渡处理的候选值。 */
void CapSenseAutoRange_ProcessFrame(
	const uint16_t hardware_raw[CAPSENSE_AUTORANGE_TOTAL_CHANNEL_COUNT],
	bool matrix_contact_active,
	bool matrix_release_confirmed,
	uint16_t algorithm_raw[CAPSENSE_AUTORANGE_TOTAL_CHANNEL_COUNT],
	uint32_t normalized_raw[CAPSENSE_AUTORANGE_TOTAL_CHANNEL_COUNT],
	uint32_t candidate_normalized_raw[CAPSENSE_AUTORANGE_TOTAL_CHANNEL_COUNT]);

/* 在扫描空闲点应用待切换Ref，无请求时返回false。 */
bool CapSenseAutoRange_ApplyPending(void);

/* 进入或退出手动校准模式；手动模式只接受七个固定Ref档位。 */
bool CapSenseAutoRange_SetManualMode(bool enabled, uint8_t ref_cdac);

/* 返回当前是否已暂停自动量程并进入手动控制。 */
bool CapSenseAutoRange_IsManualMode(void);

/* 手动矩阵配置完成后同步实际Nsub、TxDiv和Ref，避免自动量程状态使用旧值。 */
void CapSenseAutoRange_OnMatrixConfigApplied(void);

/* 获取用于协议显示的自动量程状态快照。 */
void CapSenseAutoRange_GetStatus(capsense_autorange_status_t *status);

#endif
