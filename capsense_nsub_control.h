/* 防止头文件被重复包含。 */
#ifndef CAPSENSE_NSUB_CONTROL_H
/* 定义头文件保护宏。 */
#define CAPSENSE_NSUB_CONTROL_H

/* 引入布尔类型定义。 */
#include <stdbool.h>
/* 引入固定宽度整数类型定义。 */
#include <stdint.h>

/* 引入当前手指变体对应的逻辑通道数。 */
#include "device_variant.h"

/* 定义当前配置包含的逻辑通道数量。 */
#define CAPSENSE_NSUB_CONTROL_CHANNEL_COUNT DEVICE_SENSOR_CHANNEL_COUNT

/* 定义手动 Nsub 设置操作的结果。 */
typedef enum {
	/* 表示最近一次操作成功。 */
	CAPSENSE_NSUB_CONTROL_RESULT_SUCCESS = 0u,
	/* 表示请求的 Nsub 不属于允许的四个档位。 */
	CAPSENSE_NSUB_CONTROL_RESULT_INVALID_TIER = 1u,
	/* 表示 CapSense 参数写入失败。 */
	CAPSENSE_NSUB_CONTROL_RESULT_SET_FAILED = 2u,
	/* 表示 CapSense 参数回读失败。 */
	CAPSENSE_NSUB_CONTROL_RESULT_GET_FAILED = 3u,
	/* 表示当前传感器拓扑不支持矩阵专用运行时调参。 */
	CAPSENSE_NSUB_CONTROL_RESULT_UNSUPPORTED_CONFIG = 4u,
/* 结束操作结果枚举。 */
} capsense_nsub_control_result_t;

/* 定义发送给上位机的易失性 Nsub 状态快照。 */
typedef struct {
	/* 保存上位机最近请求的 Nsub 档位。 */
	uint16_t requested_nsub;
	/* 保存 CH0～CH7 当前回读到的实际 Nsub。 */
	uint16_t actual_nsub[CAPSENSE_NSUB_CONTROL_CHANNEL_COUNT];
	/* 标记请求是否仍在等待应用或等待首帧初始化。 */
	uint8_t pending;
	/* 保存最近一次设置操作的结果。 */
	uint8_t result;
/* 结束状态快照结构体。 */
} capsense_nsub_control_status_t;

typedef struct {
	uint16_t requested_nsub;
	uint16_t requested_tx_clk_div;
	uint8_t requested_ref_cdac;
	uint16_t actual_nsub;
	uint16_t actual_tx_clk_div;
	uint8_t actual_ref_cdac;
	uint8_t pending;
	uint8_t result;
} capsense_matrix_config_status_t;

/* 初始化全通道易失性 Nsub 控制模块。 */
void CapSenseNsubControl_Init(void);
/* 请求在扫描空闲点把所有通道切换到指定Nsub档位。 */
bool CapSenseNsubControl_Request(uint16_t nsub);
/* 请求易失性修改三参数；固定档位固件作用于全部逻辑通道。 */
bool CapSenseMatrixConfig_Request(uint16_t nsub, uint16_t tx_clk_div, uint8_t ref_cdac);
/* 读取最近请求及当前实际生效的三参数。 */
void CapSenseMatrixConfig_GetStatus(capsense_matrix_config_status_t *status);
/* 在扫描空闲点只更新 MatrixButtons0 Ref CDAC，不重算固定的 CIC2 参数。 */
bool CapSenseMatrixRefCdac_Apply(uint8_t ref_cdac);
/* 在扫描空闲时应用等待中的 Nsub 请求，且绝不写入 Flash。 */
bool CapSenseNsubControl_ApplyPending(void);
/* 在新 Nsub 的第一帧扫描结束后重置 CapSense 滤波器和基线。 */
bool CapSenseNsubControl_FinalizeAppliedScan(void);
/* 获取当前请求值、八通道实际值、等待标志和执行结果。 */
void CapSenseNsubControl_GetStatus(capsense_nsub_control_status_t *status);

/* 结束头文件保护区域。 */
#endif
