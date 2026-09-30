#include "sensor_algorithm.h"
/* 全部力标定写入必须通过应用参数区的地址保护。 */
#include "app_flash.h"
/* 独立四点低力标定不依赖48点K表或通道状态切换。 */
#include "low_force_calibration.h"
#include "baseline_tracker.h"
/* 引入Ref CDAC仿射参数的Flash持久化接口。 */
#include "capsense_affine_calibration.h"
#include "capsense_autorange.h"
/* 引入逐通道CapSense当前参数只读诊断接口。 */
#include "capsense_channel_config.h"
/* 引入 V8 全通道易失性 Nsub 控制接口。 */
#include "capsense_nsub_control.h"
#include "capsense_runtime.h"
#include "channel_state_machine.h"
#include "process_channel_sample.h"
#include "tuner.h"

#if SENSOR_ALGO_FORCE_VECTOR_OUTPUT
#include "force_vector_cal.h"
#endif

#include "cy_pdl.h"
#include "cybsp.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include <stdint.h>
#include <stdio.h>

/*
 * All timing in this module is frame based. SensorAlgorithm_Process() is
 * expected to be called once per sensor frame.
 */
#define SENSOR_ALGO_CALIBRATION_FRAMES (SENSOR_ALGO_CALIBRATION_SECONDS * SENSOR_ALGO_FRAME_RATE_HZ)
#define SENSOR_ALGO_ACTIVE_RECAL_FRAMES (SENSOR_ALGO_ACTIVE_RECAL_SECONDS * SENSOR_ALGO_FRAME_RATE_HZ)
/* Group-level release supervisor. Recovery keeps outputs at zero while both
 * baseline layers absorb rebound; only a spatially confirmed re-press may
 * interrupt it after the hard blanking window.  Release uses a robust group
 * statistic: two channels may move in the wrong direction as silicone
 * rebounds, while the returning channels and the drop in total deformation
 * still identify removal of the shared contact. */
#define SENSOR_ALGO_GROUP_RELEASE_SCORE_THRESHOLD (0.45f)
#define SENSOR_ALGO_GROUP_RELEASE_PEAK_RATIO (0.90f)
#define SENSOR_ALGO_GROUP_RELEASE_CONFIRM_FRAMES (4u)
#define SENSOR_ALGO_GROUP_RECOVERY_HARD_FRAMES (12u)
#define SENSOR_ALGO_GROUP_RECOVERY_MIN_FRAMES (30u)
#define SENSOR_ALGO_GROUP_RECOVERY_MAX_FRAMES (300u)
#define SENSOR_ALGO_GROUP_RECOVERY_STABLE_FRAMES (12u)
#define SENSOR_ALGO_GROUP_RECOVERY_STABLE_CHANNELS (3u)
#define SENSOR_ALGO_GROUP_RECOVERY_RESIDUAL_NOISE_SCALE (3.0f)
#define SENSOR_ALGO_GROUP_RECOVERY_RESIDUAL_MIN_COUNTS (6.0f)
#define SENSOR_ALGO_GROUP_RECOVERY_SLOPE_NOISE_SCALE (1.5f)
#define SENSOR_ALGO_GROUP_RECOVERY_SLOPE_MIN_COUNTS (2.0f)
#define SENSOR_ALGO_GROUP_RECONTACT_DELTA_NOISE_SCALE (3.0f)
#define SENSOR_ALGO_GROUP_RECONTACT_DELTA_MIN_COUNTS (6.0f)
#define SENSOR_ALGO_GROUP_RECONTACT_SLOPE_NOISE_SCALE (1.5f)
#define SENSOR_ALGO_GROUP_RECONTACT_SLOPE_MIN_COUNTS (2.0f)
#define SENSOR_ALGO_GROUP_RECONTACT_CONFIRM_FRAMES (3u)
#define SENSOR_ALGO_BYTES_PER_VALUE (2u)
#define SENSOR_ALGO_BYTES_PER_K_VALUE (4u)
#define SENSOR_ALGO_DRIFT_SAMPLE_RATE_HZ (160u)
#define SENSOR_ALGO_DRIFT_WINDOW_FRAMES (SENSOR_ALGO_DRIFT_SAMPLE_RATE_HZ)
#define SENSOR_ALGO_DRIFT_K_UPDATE_DIVIDER (8u)
#define SENSOR_ALGO_PROTOCOL_MAGIC_SIZE (4u)
#define SENSOR_ALGO_PROTOCOL_HEADER_SIZE (10u)
#define SENSOR_ALGO_DEVICE_TYPE DEVICE_FINGER_TYPE
#define SENSOR_ALGO_FRAME_TYPE_SIGNAL (0x01u)
#define SENSOR_ALGO_FRAME_TYPE_RUN_FORCE (0x02u)
/* Type 0x03按0.1摄氏度单位上传芯片内部温度。 */
#define SENSOR_ALGO_FRAME_TYPE_TEMPERATURE (0x03u)
#define SENSOR_ALGO_FRAME_TYPE_DEBUG (0x10u)
/* 固定四通道统计使用独立帧，不改变已有信号帧和调试帧布局。 */
#define SENSOR_ALGO_FRAME_TYPE_CHANNEL_4567_STATISTICS (0x14u)
#define SENSOR_ALGO_FRAME_TYPE_FORCE_VECTOR (0x11u)
#define SENSOR_ALGO_FRAME_TYPE_FINGER_PAD_FORCE_VECTOR (0x12u)
#if SENSOR_ALGO_CHANNEL_4567_SIGNAL_SUM_OUTPUT
#define SENSOR_ALGO_FRAME_TYPE_CHANNEL_4567_SIGNAL_SUM (0x13u)
#endif
#define SENSOR_ALGO_FRAME_TYPE_FORCE_CALIBRATION (0x20u)
#define SENSOR_ALGO_FRAME_TYPE_FORCE_CALIBRATION_PARAMETERS (0x21u)
#define SENSOR_ALGO_FRAME_TYPE_CAPSENSE_NSUB_STATUS (0x22u)
#define SENSOR_ALGO_FRAME_TYPE_CAPSENSE_CONFIG (0x23u)
#define SENSOR_ALGO_FRAME_TYPE_CAPSENSE_CREF_STATUS (0x24u)
/* V8 新增全通道易失性 Nsub 状态帧。 */
#define SENSOR_ALGO_FRAME_TYPE_CAPSENSE_NSUB_TIER_STATUS (0x26u)
#define SENSOR_ALGO_FRAME_TYPE_CAPSENSE_MATRIX_CONFIG_STATUS (0x27u)
/* V9新增Ref CDAC自动量程状态帧。 */
#define SENSOR_ALGO_FRAME_TYPE_CAPSENSE_AUTORANGE_STATUS (0x28u)
/* V11新增Ref CDAC仿射参数回读帧。 */
#define SENSOR_ALGO_FRAME_TYPE_CAPSENSE_AFFINE_CALIBRATION_STATUS (0x29u)
/* V11新增可关联命令类型的仿射参数操作结果帧。 */
#define SENSOR_ALGO_FRAME_TYPE_CAPSENSE_AFFINE_OPERATION_STATUS (0x2Au)
/* V14新增设备身份信息响应帧。 */
#define SENSOR_ALGO_FRAME_TYPE_DEVICE_INFO (0x2Bu)
/* V15新增逐通道CapSense当前参数响应帧。 */
#define SENSOR_ALGO_FRAME_TYPE_CAPSENSE_CHANNEL_CONFIG_STATUS (0x2Cu)
#define SENSOR_ALGO_FRAME_TYPE_COMMAND_ACK (0x81u)
#define SENSOR_ALGO_COMMAND_RECALIBRATE (0x80u)
#define SENSOR_ALGO_COMMAND_SET_OUTPUT_MODE (0x82u)
#define SENSOR_ALGO_COMMAND_READ_FORCE_CALIBRATION (0x83u)
#define SENSOR_ALGO_COMMAND_SET_FORCE_CALIBRATION (0x84u)
#define SENSOR_ALGO_COMMAND_SAVE_FORCE_CALIBRATION (0x85u)
#define SENSOR_ALGO_COMMAND_DISCARD_FORCE_CALIBRATION (0x86u)
#define SENSOR_ALGO_COMMAND_SET_CAPSENSE_NSUB (0x87u)
#define SENSOR_ALGO_COMMAND_GET_CAPSENSE_NSUB (0x88u)
#define SENSOR_ALGO_COMMAND_GET_CAPSENSE_CONFIG (0x89u)
#define SENSOR_ALGO_COMMAND_SET_CAPSENSE_CREF (0x8Au)
#define SENSOR_ALGO_COMMAND_GET_CAPSENSE_CREF (0x8Bu)
/* V8 新增全通道易失性 Nsub 设置命令。 */
#define SENSOR_ALGO_COMMAND_SET_CAPSENSE_NSUB_TIER (0x8Cu)
/* V8 新增全通道易失性 Nsub 查询命令。 */
#define SENSOR_ALGO_COMMAND_GET_CAPSENSE_NSUB_TIER (0x8Du)
#define SENSOR_ALGO_COMMAND_SET_CAPSENSE_MATRIX_CONFIG (0x8Eu)
#define SENSOR_ALGO_COMMAND_GET_CAPSENSE_MATRIX_CONFIG (0x8Fu)
/* 主机使用空载荷0x90命令立即查询自动量程状态。 */
#define SENSOR_ALGO_COMMAND_GET_CAPSENSE_AUTORANGE_STATUS (0x90u)
/* 主机使用0x91命令进入、切换或退出Ref手动校准模式。 */
#define SENSOR_ALGO_COMMAND_SET_CAPSENSE_AUTORANGE_CALIBRATION (0x91u)
/* 主机使用0x92命令暂存单通道七档仿射参数。 */
#define SENSOR_ALGO_COMMAND_STAGE_CAPSENSE_AFFINE_CALIBRATION (0x92u)
/* 主机使用0x93命令把完整28组参数写入Flash。 */
#define SENSOR_ALGO_COMMAND_SAVE_CAPSENSE_AFFINE_CALIBRATION (0x93u)
/* 主机使用0x94命令回读单通道七档仿射参数。 */
#define SENSOR_ALGO_COMMAND_GET_CAPSENSE_AFFINE_CALIBRATION (0x94u)
/* 主机使用0x95命令执行软件复位，以验证启动加载。 */
#define SENSOR_ALGO_COMMAND_SOFTWARE_RESET (0x95u)
/* V14新增设备身份信息查询命令。 */
#define SENSOR_ALGO_COMMAND_GET_DEVICE_INFO (0x96u)
/* V15新增逐通道CapSense当前参数查询命令。 */
#define SENSOR_ALGO_COMMAND_GET_CAPSENSE_CHANNEL_CONFIG (0x97u)
/* 低力读写使用独立命令及带请求编号的应答，避免通用ACK串扰。 */
#define SENSOR_ALGO_COMMAND_READ_LOW_FORCE (0x98u)
#define SENSOR_ALGO_COMMAND_SAVE_LOW_FORCE (0x99u)
#define SENSOR_ALGO_FRAME_LOW_FORCE (0x2Du)
#define SENSOR_ALGO_RESULT_SUCCESS (0u)
#define SENSOR_ALGO_RESULT_INVALID_PAYLOAD (1u)
#define SENSOR_ALGO_RESULT_UNSUPPORTED_COMMAND (2u)
#define SENSOR_ALGO_RESULT_UNSUPPORTED_REGION (3u)
#define SENSOR_ALGO_RESULT_NVM_FAILURE (4u)
#define SENSOR_ALGO_FORCE_CALIBRATION_REGION_PRIMARY (0u)
#define SENSOR_ALGO_FORCE_CALIBRATION_REGION_SECONDARY (1u)
#define SENSOR_ALGO_FORCE_CALIBRATION_REGION_COUNT (2u)
#define SENSOR_ALGO_FORCE_CALIBRATION_FORCE_TYPE_NORMAL (0u)
#define SENSOR_ALGO_FORCE_CALIBRATION_FORCE_TYPE_TANGENTIAL (1u)
#define SENSOR_ALGO_FORCE_CALIBRATION_FORCE_TYPE_DIRECTION (2u)
#define SENSOR_ALGO_FORCE_CALIBRATION_MODEL_PIECEWISE_LINEAR (2u)
/* 法向协议、RAM及Flash统一保留两段共48个实测节点。 */
#define SENSOR_ALGO_FORCE_CALIBRATION_MAX_NORMAL_POINTS SENSOR_ALGO_FORCE_CALIBRATION_POINT_COUNT
/* 本次只扩展法向两段标定，切向表继续保留20节点容量。 */
#define SENSOR_ALGO_FORCE_CALIBRATION_MAX_TANGENTIAL_POINTS (20u)
#define SENSOR_ALGO_FORCE_CALIBRATION_DIRECTION_POINTS (2u)
/* FCV9从0x1F600开始，每区三行；末尾不超过0x1FC00。 */
#define SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_ADDRESS APP_FLASH_FORCE_ADDRESS
#define SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_ROWS_PER_REGION (3u)
#define SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_VERSION (9u)
/* FCV7兼容读取保留升级前的13节点单行记录地址与布局。 */
#define SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_STORAGE_ADDRESS (0x0001FB00u)
#define SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_STORAGE_VERSION (7u)
#define SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_MAX_NORMAL_POINTS (13u)
#define SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_MAX_TANGENTIAL_POINTS (13u)
#define SENSOR_ALGO_FORCE_REPORT_DIVISOR (1000.0f)
/* 停用整数软吸附，使0.1 N低力标定结果直接按K表输出。 */
#define SENSOR_ALGO_FORCE_OUTPUT_SOFT_ANCHOR_ENABLED (0u)
/* 锚点中心附近保留20%的输入斜率，兼顾读数聚拢与严格单调。 */
#define SENSOR_ALGO_FORCE_OUTPUT_SOFT_ANCHOR_CENTER_SCALE (0.20f)
#define SENSOR_ALGO_FORCE_OUTPUT_SOFT_ANCHOR_MIN_N (1.0f)
#define SENSOR_ALGO_FORCE_OUTPUT_SOFT_ANCHOR_MAX_N (20.0f)
#define SENSOR_ALGO_FORCE_OUTPUT_SOFT_ANCHOR_WINDOW_N (0.5f)
#define SENSOR_ALGO_RAD_TO_DEG (57.29577951308232f)
#define SENSOR_ALGO_DEG_TO_RAD (0.017453292519943295f)
#define SENSOR_ALGO_SIGNAL_VALUES_PER_SENSOR (1u)
#define SENSOR_ALGO_DEBUG_U16_VALUES_PER_SENSOR (4u)
/* V8 仅上传 Fast K 与 Slow K 两组有符号 32 位数据。 */
#define SENSOR_ALGO_DEBUG_K_VALUES_PER_SENSOR (2u)
#define SENSOR_ALGO_FORCE_VECTOR_VALUE_COUNT (14u)
#define SENSOR_ALGO_BYTES_PER_FLOAT (4u)
#define SENSOR_ALGO_FORCE_VECTOR_CHANNEL_FIRST (4u)
#define SENSOR_ALGO_FORCE_VECTOR_CHANNEL_LAST (8u)
#define SENSOR_ALGO_SIGNAL_PAYLOAD_SIZE (SENSOR_ALGO_CHANNEL_COUNT * SENSOR_ALGO_SIGNAL_VALUES_PER_SENSOR * SENSOR_ALGO_BYTES_PER_VALUE)
#define SENSOR_ALGO_SIGNAL_FRAME_SIZE (SENSOR_ALGO_PROTOCOL_HEADER_SIZE + SENSOR_ALGO_SIGNAL_PAYLOAD_SIZE)
#define SENSOR_ALGO_DEBUG_PAYLOAD_SIZE                                                                                                                                             \
	(SENSOR_ALGO_CHANNEL_COUNT *                                                                                                                                                   \
	 ((SENSOR_ALGO_DEBUG_U16_VALUES_PER_SENSOR * SENSOR_ALGO_BYTES_PER_VALUE) +                                                               \
	  (SENSOR_ALGO_DEBUG_K_VALUES_PER_SENSOR * SENSOR_ALGO_BYTES_PER_K_VALUE)))
#define SENSOR_ALGO_DEBUG_FRAME_SIZE (SENSOR_ALGO_PROTOCOL_HEADER_SIZE + SENSOR_ALGO_DEBUG_PAYLOAD_SIZE)
/* V18在原28字节后追加4字节主法向优化值，同帧保留原始值。 */
/* V19在双输出之后保留实际换算值和未模拟的优化值。 */
#define SENSOR_ALGO_RUN_FORCE_PAYLOAD_SIZE (40u)
#define SENSOR_ALGO_RUN_FORCE_FRAME_SIZE (SENSOR_ALGO_PROTOCOL_HEADER_SIZE + SENSOR_ALGO_RUN_FORCE_PAYLOAD_SIZE)
/* 温度帧载荷为一个小端int16，通道数固定为零。 */
#define SENSOR_ALGO_TEMPERATURE_PAYLOAD_SIZE (2u)
#define SENSOR_ALGO_TEMPERATURE_FRAME_SIZE (SENSOR_ALGO_PROTOCOL_HEADER_SIZE + SENSOR_ALGO_TEMPERATURE_PAYLOAD_SIZE)
#define SENSOR_ALGO_FORCE_VECTOR_PAYLOAD_SIZE (SENSOR_ALGO_FORCE_VECTOR_VALUE_COUNT * SENSOR_ALGO_BYTES_PER_FLOAT)
#define SENSOR_ALGO_FORCE_VECTOR_FRAME_SIZE (SENSOR_ALGO_PROTOCOL_HEADER_SIZE + SENSOR_ALGO_FORCE_VECTOR_PAYLOAD_SIZE)
#if SENSOR_ALGO_CHANNEL_4567_SIGNAL_SUM_OUTPUT
#define SENSOR_ALGO_CHANNEL_4567_SIGNAL_SUM_PAYLOAD_SIZE (4u)
#define SENSOR_ALGO_CHANNEL_4567_SIGNAL_SUM_FRAME_SIZE (SENSOR_ALGO_PROTOCOL_HEADER_SIZE + SENSOR_ALGO_CHANNEL_4567_SIGNAL_SUM_PAYLOAD_SIZE)
#endif
#define SENSOR_ALGO_ACK_PAYLOAD_SIZE (1u)
#define SENSOR_ALGO_ACK_FRAME_SIZE (SENSOR_ALGO_PROTOCOL_HEADER_SIZE + SENSOR_ALGO_ACK_PAYLOAD_SIZE)
#define SENSOR_ALGO_CAPSENSE_NSUB_STATUS_PAYLOAD_SIZE (12u)
#define SENSOR_ALGO_CAPSENSE_NSUB_STATUS_FRAME_SIZE (SENSOR_ALGO_PROTOCOL_HEADER_SIZE + SENSOR_ALGO_CAPSENSE_NSUB_STATUS_PAYLOAD_SIZE)
#define SENSOR_ALGO_CAPSENSE_CONFIG_PAYLOAD_SIZE (54u)
#define SENSOR_ALGO_CAPSENSE_CONFIG_FRAME_SIZE (SENSOR_ALGO_PROTOCOL_HEADER_SIZE + SENSOR_ALGO_CAPSENSE_CONFIG_PAYLOAD_SIZE)
#define SENSOR_ALGO_CAPSENSE_CREF_STATUS_PAYLOAD_SIZE (10u)
#define SENSOR_ALGO_CAPSENSE_CREF_STATUS_FRAME_SIZE (SENSOR_ALGO_PROTOCOL_HEADER_SIZE + SENSOR_ALGO_CAPSENSE_CREF_STATUS_PAYLOAD_SIZE)
#define SENSOR_ALGO_CAPSENSE_CONFIG_SENSOR_INDEX (4u)
/* 请求值、八个实际值、等待标志和结果码共占 20 字节。 */
#define SENSOR_ALGO_CAPSENSE_NSUB_TIER_STATUS_PAYLOAD_SIZE \
	(4u + (SENSOR_ALGO_CHANNEL_COUNT * SENSOR_ALGO_BYTES_PER_VALUE))
/* 定义 V8 全通道易失性 Nsub 状态帧总长度。 */
#define SENSOR_ALGO_CAPSENSE_NSUB_TIER_STATUS_FRAME_SIZE (SENSOR_ALGO_PROTOCOL_HEADER_SIZE + SENSOR_ALGO_CAPSENSE_NSUB_TIER_STATUS_PAYLOAD_SIZE)
#define SENSOR_ALGO_CAPSENSE_MATRIX_CONFIG_STATUS_PAYLOAD_SIZE (12u)
#define SENSOR_ALGO_CAPSENSE_MATRIX_CONFIG_STATUS_FRAME_SIZE (SENSOR_ALGO_PROTOCOL_HEADER_SIZE + SENSOR_ALGO_CAPSENSE_MATRIX_CONFIG_STATUS_PAYLOAD_SIZE)
#define SENSOR_ALGO_CAPSENSE_AUTORANGE_STATUS_PAYLOAD_SIZE (20u)
#define SENSOR_ALGO_CAPSENSE_AUTORANGE_STATUS_FRAME_SIZE (SENSOR_ALGO_PROTOCOL_HEADER_SIZE + SENSOR_ALGO_CAPSENSE_AUTORANGE_STATUS_PAYLOAD_SIZE)
#define SENSOR_ALGO_CAPSENSE_AUTORANGE_STATUS_PERIOD_FRAMES (50u)
/* 仿射参数状态包含16字节元数据和七组8字节定点参数。 */
#define SENSOR_ALGO_CAPSENSE_AFFINE_STATUS_PAYLOAD_SIZE (16u + (CAPSENSE_AFFINE_TIER_COUNT * 8u))
/* 定义V11仿射参数状态帧总长度。 */
#define SENSOR_ALGO_CAPSENSE_AFFINE_STATUS_FRAME_SIZE (SENSOR_ALGO_PROTOCOL_HEADER_SIZE + SENSOR_ALGO_CAPSENSE_AFFINE_STATUS_PAYLOAD_SIZE)
/* 操作结果包含命令、结果、通道、保留字节和参数ID。 */
#define SENSOR_ALGO_CAPSENSE_AFFINE_OPERATION_PAYLOAD_SIZE (8u)
/* 定义仿射参数操作结果帧总长度。 */
#define SENSOR_ALGO_CAPSENSE_AFFINE_OPERATION_FRAME_SIZE (SENSOR_ALGO_PROTOCOL_HEADER_SIZE + SENSOR_ALGO_CAPSENSE_AFFINE_OPERATION_PAYLOAD_SIZE)
/* 设备信息载荷由四字节固定字段和不带结尾零字符的ASCII设备名组成。 */
#define SENSOR_ALGO_DEVICE_NAME_LENGTH ((uint16_t)(sizeof(DEVICE_FINGER_NAME) - 1u))
/* 定义设备信息帧载荷和总长度。 */
#define SENSOR_ALGO_DEVICE_INFO_PAYLOAD_SIZE (4u + SENSOR_ALGO_DEVICE_NAME_LENGTH)
#define SENSOR_ALGO_DEVICE_INFO_FRAME_SIZE (SENSOR_ALGO_PROTOCOL_HEADER_SIZE + SENSOR_ALGO_DEVICE_INFO_PAYLOAD_SIZE)
/* 参数帧头占4字节，每路参数记录占14字节。 */
#define SENSOR_ALGO_CAPSENSE_CHANNEL_CONFIG_ENTRY_SIZE (14u)
#define SENSOR_ALGO_CAPSENSE_CHANNEL_CONFIG_PAYLOAD_SIZE \
	(4u + (SENSOR_ALGO_CHANNEL_COUNT * SENSOR_ALGO_CAPSENSE_CHANNEL_CONFIG_ENTRY_SIZE))
#define SENSOR_ALGO_CAPSENSE_CHANNEL_CONFIG_FRAME_SIZE \
	(SENSOR_ALGO_PROTOCOL_HEADER_SIZE + SENSOR_ALGO_CAPSENSE_CHANNEL_CONFIG_PAYLOAD_SIZE)
/* 单通道下发载荷包含8字节事务头和七组8字节定点参数。 */
#define SENSOR_ALGO_CAPSENSE_AFFINE_STAGE_PAYLOAD_SIZE (8u + (CAPSENSE_AFFINE_TIER_COUNT * 8u))
#define SENSOR_ALGO_FORCE_CALIBRATION_PARAMETERS_BASE_PAYLOAD_SIZE (8u)
#define SENSOR_ALGO_FORCE_CALIBRATION_PARAMETERS_MAX_PAYLOAD_SIZE SENSOR_ALGO_FORCE_CALIBRATION_COMMAND_MAX_PAYLOAD_SIZE
#define SENSOR_ALGO_FORCE_CALIBRATION_PARAMETERS_MAX_FRAME_SIZE (SENSOR_ALGO_PROTOCOL_HEADER_SIZE + SENSOR_ALGO_FORCE_CALIBRATION_PARAMETERS_MAX_PAYLOAD_SIZE)
#define SENSOR_ALGO_UART_TX_BUFFER_COUNT (4u)
#define SENSOR_ALGO_UART_TX_BUFFER_SIZE                                                                                                                                            \
	((SENSOR_ALGO_DEBUG_FRAME_SIZE > SENSOR_ALGO_FORCE_CALIBRATION_PARAMETERS_MAX_FRAME_SIZE) ? SENSOR_ALGO_DEBUG_FRAME_SIZE                                                       \
																							  : SENSOR_ALGO_FORCE_CALIBRATION_PARAMETERS_MAX_FRAME_SIZE)
#define SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_NORMAL_RAW_OFFSET (16u)
#define SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_NORMAL_FORCE_OFFSET (SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_NORMAL_RAW_OFFSET + (SENSOR_ALGO_FORCE_CALIBRATION_MAX_NORMAL_POINTS * 4u))
#define SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_TANGENTIAL_RAW_OFFSET                                                                                                                \
	(SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_NORMAL_FORCE_OFFSET + (SENSOR_ALGO_FORCE_CALIBRATION_MAX_NORMAL_POINTS * 4u))
#define SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_TANGENTIAL_FORCE_OFFSET                                                                                                              \
	(SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_TANGENTIAL_RAW_OFFSET + (SENSOR_ALGO_FORCE_CALIBRATION_MAX_TANGENTIAL_POINTS * 4u))
#define SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_DIRECTION_RAW_OFFSET                                                                                                                 \
	(SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_TANGENTIAL_FORCE_OFFSET + (SENSOR_ALGO_FORCE_CALIBRATION_MAX_TANGENTIAL_POINTS * 4u))
#define SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_DIRECTION_OUTPUT_OFFSET                                                                                                              \
	(SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_DIRECTION_RAW_OFFSET + (SENSOR_ALGO_FORCE_CALIBRATION_DIRECTION_POINTS * 4u))
#define SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_DATA_SIZE (SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_DIRECTION_OUTPUT_OFFSET + (SENSOR_ALGO_FORCE_CALIBRATION_DIRECTION_POINTS * 4u))
#define SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_CHECKSUM_OFFSET SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_DATA_SIZE
/* FCV9三行记录覆盖580字节有效数据及填充，大小必须与集中布局一致。 */
#define SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_RECORD_SIZE (SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_ROWS_PER_REGION * CY_FLASH_SIZEOF_ROW)
/* FCV7旧记录偏移仅用于固件升级后的兼容加载。 */
#define SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_NORMAL_RAW_OFFSET (16u)
#define SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_NORMAL_FORCE_OFFSET \
	(SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_NORMAL_RAW_OFFSET + (SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_MAX_NORMAL_POINTS * 4u))
#define SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_TANGENTIAL_RAW_OFFSET \
	(SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_NORMAL_FORCE_OFFSET + (SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_MAX_NORMAL_POINTS * 4u))
#define SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_TANGENTIAL_FORCE_OFFSET \
	(SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_TANGENTIAL_RAW_OFFSET + (SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_MAX_TANGENTIAL_POINTS * 4u))
#define SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_DIRECTION_RAW_OFFSET \
	(SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_TANGENTIAL_FORCE_OFFSET + (SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_MAX_TANGENTIAL_POINTS * 4u))
#define SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_DIRECTION_OUTPUT_OFFSET \
	(SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_DIRECTION_RAW_OFFSET + (SENSOR_ALGO_FORCE_CALIBRATION_DIRECTION_POINTS * 4u))
#define SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_DATA_SIZE \
	(SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_DIRECTION_OUTPUT_OFFSET + (SENSOR_ALGO_FORCE_CALIBRATION_DIRECTION_POINTS * 4u))
#define SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_CHECKSUM_OFFSET SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_DATA_SIZE

#if ((SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_CHECKSUM_OFFSET + 4u) > SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_RECORD_SIZE) || \
    (SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_RECORD_SIZE != APP_FLASH_FORCE_REGION_BYTES)
#error "FCV9 force calibration record exceeds reserved Flash rows"
#endif

typedef struct {
	float normal_raw_points[SENSOR_ALGO_FORCE_CALIBRATION_MAX_NORMAL_POINTS];
	float normal_force_points[SENSOR_ALGO_FORCE_CALIBRATION_MAX_NORMAL_POINTS];
	float tangential_raw_points[SENSOR_ALGO_FORCE_CALIBRATION_MAX_TANGENTIAL_POINTS];
	float tangential_force_points[SENSOR_ALGO_FORCE_CALIBRATION_MAX_TANGENTIAL_POINTS];
	float direction_raw_points[SENSOR_ALGO_FORCE_CALIBRATION_DIRECTION_POINTS];
	float direction_output_points[SENSOR_ALGO_FORCE_CALIBRATION_DIRECTION_POINTS];
	uint8_t normal_point_count;
	uint8_t tangential_point_count;
	uint8_t direction_point_count;
	bool normal_valid;
	bool tangential_valid;
	bool direction_valid;
	bool normal_dirty;
	bool tangential_dirty;
	bool direction_dirty;
	bool valid;
	bool dirty;
} sensor_force_calibration_t;

typedef struct {
	uint16_t raw_history[SENSOR_ALGO_DRIFT_WINDOW_FRAMES];
	uint16_t signal_history[SENSOR_ALGO_DRIFT_WINDOW_FRAMES];
	uint16_t sample_count;
	uint16_t write_index;
	uint8_t k_update_count;
	uint64_t raw_sum;
	uint64_t signal_sum;
	uint64_t raw_weighted_sum;
	uint64_t signal_weighted_sum;
	int32_t raw_k_q16;
	int32_t signal_k_q16;
} channel_drift_tracker_t;

static uint32_t baseline_q16[SENSOR_ALGO_CHANNEL_COUNT];
static uint32_t signal_q16[SENSOR_ALGO_CHANNEL_COUNT];
/* 保存每个通道进入 ACTIVE_FLOATING 时锁定的 Q16 Signal。 */
static uint32_t floating_locked_signal_q16[SENSOR_ALGO_CHANNEL_COUNT];
/* 保存每个通道已经吸收的活动期蠕变补偿量，单位为 Signal count。 */
static int32_t floating_compensation_values[SENSOR_ALGO_CHANNEL_COUNT];
/* 标记每个通道当前是否保留有效的蠕变补偿。 */
static bool floating_compensation_valid[SENSOR_ALGO_CHANNEL_COUNT];
/* 保存每个通道快速重复按压前尚未恢复的临时零点偏移，单位为 count。 */
static uint16_t cycle_zero_offset_values[SENSOR_ALGO_CHANNEL_COUNT];
/* 矩阵仅保留状态机的快慢K历史，停用重复的旧版漂移缓冲以节省RAM。 */
#if !DEVICE_MATRIX20
static channel_drift_tracker_t drift_trackers[SENSOR_ALGO_CHANNEL_COUNT];
#endif

static uint32_t idle_counts[SENSOR_ALGO_CHANNEL_COUNT];
static uint32_t active_recal_counts[SENSOR_ALGO_CHANNEL_COUNT];
static uint32_t post_active_baseline_delay_counts[SENSOR_ALGO_CHANNEL_COUNT];
static uint32_t post_active_fast_baseline_counts[SENSOR_ALGO_CHANNEL_COUNT];
static uint32_t release_negative_slope_counts[SENSOR_ALGO_CHANNEL_COUNT];
static uint8_t negative_excursion_counts[SENSOR_ALGO_CHANNEL_COUNT];
static bool channel_was_active[SENSOR_ALGO_CHANNEL_COUNT];

static uint32_t calibration_count;
static bool calibrated;
static uint16_t startup_quiet_frame_count;

static bool channel_recalibrating[SENSOR_ALGO_CHANNEL_COUNT];
static uint32_t channel_calibration_counts[SENSOR_ALGO_CHANNEL_COUNT];

static bool initialized[SENSOR_ALGO_CHANNEL_COUNT];

/* 保存本帧自动量程是否仍处于预测或Hermite过渡阶段。 */
static bool autorange_transition_active;
/* 保存上一帧自动量程过渡状态，用于识别过渡结束边沿。 */
static bool autorange_transition_was_active;
/* 标记本帧需要为矩阵通道重建状态机斜率窗口。 */
static bool autorange_transition_finished;
static channel_state_machine_t channel_state_machines[SENSOR_ALGO_CHANNEL_COUNT];

static uint16_t previous_positive_diff_values[SENSOR_ALGO_CHANNEL_COUNT];
static uint16_t previous_raw_values[SENSOR_ALGO_CHANNEL_COUNT];
static uint16_t active_peak_diff_values[SENSOR_ALGO_CHANNEL_COUNT];
static uint16_t post_release_gate_values[SENSOR_ALGO_CHANNEL_COUNT];
static uint16_t post_release_blank_counts[SENSOR_ALGO_CHANNEL_COUNT];
static uint8_t release_counts[SENSOR_ALGO_CHANNEL_COUNT];
static uint8_t fast_release_counts[SENSOR_ALGO_CHANNEL_COUNT];
static bool fast_release_latched[SENSOR_ALGO_CHANNEL_COUNT];
/* 将快速释放状态保持约 75 ms，避免上位机刷新时漏掉仅持续一帧的触发。 */
static uint8_t fast_release_state_counts[SENSOR_ALGO_CHANNEL_COUNT];
static uint32_t release_recovery_counts[SENSOR_ALGO_CHANNEL_COUNT];
static uint8_t release_recovery_stable_counts[SENSOR_ALGO_CHANNEL_COUNT];
static uint32_t force_vector_group_signal_sum;
static uint32_t force_vector_group_noise_sum;
static bool force_vector_group_contact_hint;
static float force_vector_group_touch_peak;
static sensor_algorithm_snapshot_t latest_snapshot;
/* 保存最近一帧硬件Raw，供调试协议和统计使用。 */
static uint16_t latest_hardware_raw[SENSOR_ALGO_CHANNEL_COUNT];
static uint8_t autorange_status_period_count;
static uint8_t output_mode;
static sensor_force_calibration_t force_calibration[SENSOR_ALGO_FORCE_CALIBRATION_REGION_COUNT];
static sensor_force_calibration_t saved_force_calibration[SENSOR_ALGO_FORCE_CALIBRATION_REGION_COUNT];
/* 标记只读载入的旧布局，防止保存region 1时覆盖尚未迁移的region 0。 */
static bool force_calibration_legacy_loaded[SENSOR_ALGO_FORCE_CALIBRATION_REGION_COUNT];
static uint8_t uart_tx_buffers[SENSOR_ALGO_UART_TX_BUFFER_COUNT][SENSOR_ALGO_UART_TX_BUFFER_SIZE];
static uint32_t uart_tx_sizes[SENSOR_ALGO_UART_TX_BUFFER_COUNT];
static uint32_t uart_tx_head;
static uint32_t uart_tx_tail;
static uint32_t uart_tx_offset;
static uint32_t uart_tx_count;
#if SENSOR_ALGO_FORCE_VECTOR_OUTPUT
typedef enum {
	FORCE_GROUP_IDLE = 0u,
	FORCE_GROUP_TOUCH_ACTIVE = 1u,
	FORCE_GROUP_REBOUND_RECOVERY = 2u,
} force_group_state_t;

typedef struct {
	force_group_state_t state;
	float contact_origin[4];
	float max_delta[4];
	float min_delta[4];
	float previous_raw[4];
	float release_valley_raw[4];
	float peak_deformation;
	float previous_energy;
	uint16_t recovery_frames;
	uint8_t release_confirm_frames;
	uint8_t stable_frames;
	uint8_t recontact_confirm_frames;
} force_group_recovery_t;

static const uint32_t fingertip_ch_idx[4] = {6u, 7u, 4u, 5u};
static const uint32_t finger_pad_ch_idx[4] = {1u, 0u, 3u, 2u};
static force_vector_context_t fingertip_force_vector;
static force_vector_context_t finger_pad_force_vector;
static force_group_recovery_t fingertip_group_recovery;
static force_group_recovery_t finger_pad_group_recovery;
#endif

static void reset_channel_runtime_state(uint32_t index, bool reset_signal);
static void initialize_baseline_runtime(void);
static void update_force_vector_group_snapshot(void);
static void update_force_vector_group_contact_hint(const uint16_t raw_values[SENSOR_ALGO_CHANNEL_COUNT]);
static bool is_force_vector_channel(uint32_t index);
static bool is_force_vector_contact_confirmed(void);
static bool is_force_vector_active(void);

/* 基线执行入口仅接收本帧数据，恢复历史统一保存在通道状态机。 */
static void update_baseline(uint32_t index, uint16_t raw, int32_t raw_delta, bool suppress_activity);
static void begin_channel_release_recovery(uint32_t index);
static bool process_channel_recalibration_sample(uint32_t index, uint32_t raw, uint32_t raw_q16);
static void finish_global_calibration_frame(void);

static void begin_calibration(void);
static void finish_calibration(void);
static void begin_channel_recalibration(uint32_t index);
static void finish_channel_recalibration(uint32_t index);

static uint32_t positive_diff_from_baseline(uint32_t index, uint32_t raw);
static bool detect_negative_rebound_release(uint32_t index, uint32_t raw, bool suppress_activity);
static void update_post_release_dynamic_gate(uint32_t index, uint32_t positive_diff, int32_t raw_delta, bool suppress_activity);
static uint32_t apply_input_noise_gate(uint32_t index, uint32_t positive_diff, uint32_t multiplier);
static uint16_t update_channel_signal(uint32_t index, uint32_t positive_diff, uint32_t raw_q16, int32_t raw_delta, bool suppress_activity, bool *fast_release_active);
static void update_fast_release(uint32_t index, uint32_t positive_diff, uint32_t raw_q16, uint32_t *signal_target_q16, bool *fast_release_active);
static void apply_release_snap(uint32_t index, uint16_t previous_signal, uint16_t *signal);
/* 声明通道状态更新与 SLOW_BACK Signal 屏蔽函数。 */
/* 根据状态机结果执行慢回弹清零或稳定按压输出锁定。 */
static void update_output_state(uint32_t index, uint16_t raw, uint32_t positive_diff, bool suppress_activity, uint16_t *signal);
static bool detect_release_retouch(uint32_t index, uint32_t positive_diff, int32_t raw_delta);

static uint32_t isqrt_u64(uint64_t value);
static uint32_t apply_iir_q16(uint32_t current_q16, uint32_t target_q16, uint32_t alpha_q16);
static uint32_t follow_baseline_upward_q16(uint32_t current_q16, uint32_t raw_q16, uint32_t alpha_q16);
static uint16_t q16_to_u16(uint32_t value_q16);
static void reset_channel_drift(uint32_t index);
static void update_channel_drift(uint32_t index, uint16_t raw, uint16_t signal, sensor_algorithm_output_t *output);
static int32_t calculate_drift_k_q16(uint64_t sum, uint64_t weighted_sum);
static void send_signal_frame(const sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT]);
static void send_debug_frame(const sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT]);
/* 上报同一扫描的四通道硬件原始值统计及信号和。 */
#if SENSOR_ALGO_CHANNEL_COUNT >= 8u
static void send_channel_4567_statistics_frame(const sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT]);
#endif
#if SENSOR_ALGO_CHANNEL_4567_SIGNAL_SUM_OUTPUT
static void send_channel_4567_signal_sum_frame(const sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT]);
#endif
static void send_run_force_frame(void);
static void send_force_calibration_frame(void);
static void send_force_calibration_parameters_frame(uint8_t region, uint8_t force_type);
static void send_output_frame(const uint8_t *frame, uint32_t size);
static void send_command_response_frame(uint8_t *frame, uint32_t size);
static void send_capsense_nsub_status_frame(void);
static void send_capsense_config_frame(void);
static void send_capsense_cref_status_frame(void);
/* 发送 V8 全通道易失性 Nsub 状态。 */
static void send_capsense_nsub_tier_status_frame(void);
static void send_capsense_matrix_config_status_frame(void);
/* 仅自动量程变体需要序列化周期状态。 */
#if DEVICE_HAS_MATRIX_AUTORANGE
static void send_capsense_autorange_status_frame(bool command_response);
#endif
/* 主法向兼容独立高力10/20节点及原两段48节点，Flash布局与容量保持不变。 */
static bool primary_normal_point_count_valid(uint8_t count) {
	return (count == 10u) || (count == 20u) || (count == SENSOR_ALGO_FORCE_CALIBRATION_MAX_NORMAL_POINTS);
}

/* 新10/20点表必须实际覆盖1至30 N附近，拒绝把旧1至20 N表误当作高力新表。 */
static bool primary_normal_table_valid(const sensor_force_calibration_t *table) {
	if (!primary_normal_point_count_valid(table->normal_point_count)) {
		return false;
	}
	if (table->normal_point_count == SENSOR_ALGO_FORCE_CALIBRATION_MAX_NORMAL_POINTS) {
		return true;
	}
	return (table->normal_force_points[0u] >= 0.9f) && (table->normal_force_points[0u] <= 1.1f) &&
		(table->normal_force_points[table->normal_point_count - 1u] >= 27.0f) &&
		(table->normal_force_points[table->normal_point_count - 1u] <= 33.0f);
}

/* 声明V14设备身份信息响应函数。 */
static void send_device_info_frame(void);
/* 声明V15逐通道CapSense当前参数响应函数。 */
static void send_capsense_channel_config_status_frame(void);
static void process_sensor_frame(const uint16_t raw_values[SENSOR_ALGO_CHANNEL_COUNT], sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT]);
static void send_sensor_frame(const sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT]);
static void initialize_force_calibration(void);
static bool load_force_calibration_from_nvm(sensor_force_calibration_t *calibration, uint8_t region);
/* 回报本次低力读写的操作、结果、编号及RAM/Flash有效位。 */
static void send_low_force_response(uint8_t operation, uint8_t result, uint32_t token);
/* 升级时只读旧FCV8/FCV7，旧主表不能冒充新的48节点标定。 */
static bool load_previous_force_calibration_from_nvm(sensor_force_calibration_t *calibration, uint8_t region);
/* 升级后若FCV8尚未写入，则从原FCV7单行记录恢复参数。 */
static bool load_legacy_force_calibration_from_nvm(sensor_force_calibration_t *calibration, uint8_t region);
static bool save_force_calibration_record(const sensor_force_calibration_t *calibration, uint8_t region);
static bool save_force_calibration_to_nvm(void);
static bool force_calibration_values_valid(const sensor_force_calibration_t *calibration);
static float apply_piecewise_force_calibration(float raw, const float *raw_points, const float *force_points, uint8_t point_count);
/* 旧法向K表仅供五通道输出或显式启用的三维力模块调用。 */
#if (SENSOR_ALGO_CHANNEL_COUNT < 8u) || SENSOR_ALGO_FORCE_VECTOR_OUTPUT
static float apply_normal_force_calibration(float raw, const sensor_force_calibration_t *calibration);
/* 声明仅用于Type 0x02运行法向力的临时关键点软吸附函数。 */
static float apply_force_output_soft_anchor(float calibrated_force_n);
#endif
static float apply_tangential_force_calibration(float raw, const sensor_force_calibration_t *calibration);
static float apply_direction_calibration(float raw_radians, const sensor_force_calibration_t *calibration);
static uint32_t calculate_storage_checksum(const uint8_t *data, uint32_t size);
static float read_float(const uint8_t *data);
static void write_float_to_bytes(uint8_t *data, float value);
#if SENSOR_ALGO_FORCE_VECTOR_OUTPUT
static void reset_force_group_recovery(force_group_recovery_t *recovery);
static void clear_channel_signal_output(sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT], uint32_t channel_index);
static bool is_force_group_recovering_channel(uint32_t channel_index);
static bool force_group_has_adjacent_pair(uint8_t mask);
static void enter_force_group_recovery(force_group_recovery_t *recovery, force_vector_context_t *context, sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT],
									   const uint32_t ch_idx[4], bool fingertip_group);
static void suppress_force_group_outputs(force_vector_context_t *context, sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT], const uint32_t ch_idx[4],
										 bool fingertip_group);
static void finish_force_group_recovery(force_group_recovery_t *recovery, force_vector_context_t *context, sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT],
										const uint32_t ch_idx[4], bool fingertip_group);
static void update_force_group_recovery(force_group_recovery_t *recovery, force_vector_context_t *context, sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT],
										const uint32_t ch_idx[4], bool fingertip_group);
static void update_force_group_recoveries(sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT]);
static void update_force_vector(sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT]);
static void update_force_vector_context(force_vector_context_t *context, sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT], const uint32_t ch_idx[4]);
static void send_force_vector_frame(uint8_t frame_type, uint8_t region, const force_vector_debug_t *debug, bool contact_active);
#endif
static void write_frame_header(uint8_t *frame, uint32_t *offset, uint8_t frame_type, uint8_t channel_count, uint16_t payload_len);
static void write_u16(uint8_t *frame, uint32_t *offset, uint16_t value);
static void write_u32(uint8_t *frame, uint32_t *offset, uint32_t value);
/* 按小端格式读取协议中的32位有符号定点参数。 */
static int32_t read_i32(const uint8_t *data);
/* 返回指定CH4至CH7通道的Flash仿射参数状态。 */
static void send_capsense_affine_calibration_status_frame(uint8_t channel);
/* 返回带原命令类型的仿射参数操作结果，避免通用ACK串扰。 */
static void send_capsense_affine_operation_status_frame(
	uint8_t command_type,
	uint8_t result,
	uint8_t channel,
	uint32_t calibration_id);
static void write_float(uint8_t *frame, uint32_t *offset, float value);

static const process_channel_sample_ops_t channel_sample_ops = {
	.apply_iir_q16 = apply_iir_q16,
	.process_channel_recalibration_sample = process_channel_recalibration_sample,
	.detect_negative_rebound_release = detect_negative_rebound_release,
	.positive_diff_from_baseline = positive_diff_from_baseline,
	.update_post_release_dynamic_gate = update_post_release_dynamic_gate,
	.apply_input_noise_gate = apply_input_noise_gate,
	.update_channel_signal = update_channel_signal,
	.update_output_state = update_output_state,
	/* 状态提交后执行一次对应速度的基线更新。 */
	.update_baseline = update_baseline,
	.q16_to_u16 = q16_to_u16,
};

static const process_channel_sample_context_t channel_sample_context = {
	.baseline_q16 = baseline_q16,
	.previous_raw_values = previous_raw_values,
	.initialized = initialized,
	.channel_recalibrating = channel_recalibrating,
	.calibrated = &calibrated,
	.startup_quiet_frame_count = &startup_quiet_frame_count,
	.startup_quiet_frames = SENSOR_ALGO_STARTUP_QUIET_FRAMES,
	.calibration_alpha_q16 = SENSOR_ALGO_CALIBRATION_ALPHA_Q16,
	.signal_input_gate_multiplier = SENSOR_ALGO_SIGNAL_INPUT_GATE_MULTIPLIER,
	.ops = &channel_sample_ops,
};

void SensorAlgorithm_Init(void) {
	initialize_baseline_runtime();
	/* 矩阵默认发送基线、真实状态和快慢K，便于直接观察全部通道。 */
	output_mode = DEVICE_MATRIX20 ? SENSOR_ALGO_OUTPUT_MODE_DEBUG : SENSOR_ALGO_OUTPUT_MODE_RUN;
	initialize_force_calibration();
	/* 每次上电从独立Flash行恢复低力表，无有效表时保持未标定。 */
	(void)LowForce_Load();
	/* 启动时加载Ref CDAC仿射参数，但暂不把参数应用到Raw。 */
	CapSenseAffineCalibration_Init();
}

/* UART和其他查询直接返回唯一状态机的真实状态，不再做上报替换。 */
void SensorAlgorithm_GetStates(sensor_algorithm_state_t states[SENSOR_ALGO_CHANNEL_COUNT]) {
	uint32_t index;

	for (index = 0u; index < SENSOR_ALGO_CHANNEL_COUNT; index++) {
		states[index] = ChannelState_Get(&channel_state_machines[index]);
	}
}

/* 判断指定通道状态是否仍然代表真实接触。 */
bool SensorAlgorithm_IsContactActiveState(sensor_algorithm_state_t state) {
	/* 上升、保持、下降和蠕变锁定状态都对外报告接触有效。 */
	return (state == SENSOR_ALGO_STATE_ACTIVE_RISING) ||
		/* 稳定保持压力仍然属于有效接触。 */
		(state == SENSOR_ALGO_STATE_ACTIVE_HOLD) ||
		/* 尚未确认松手的下降过程仍然属于有效接触。 */
		(state == SENSOR_ALGO_STATE_ACTIVE_FALLING) ||
		/* 蠕变锁定期间仍然保持真实有效接触。 */
		(state == SENSOR_ALGO_STATE_ACTIVE_FLOATING);
/* 结束通道接触有效性判断函数。 */
}

void printf_q16_as_float(int32_t k_q16) {

	int64_t abs_k = (k_q16 >= 0) ? k_q16 : -(int64_t)k_q16;

	int32_t integer_part = (int32_t)(abs_k / 65536);
	int32_t frac_part = (int32_t)(((abs_k % 65536) * 10000) / 65536);

	printf("%s%ld.%04ld/s\r\n", k_q16 < 0 ? "-" : "", (long)integer_part, (long)frac_part);
}

void SensorAlgorithm_GetLatestSnapshot(sensor_algorithm_snapshot_t *snapshot) {
	uint32_t index;

	for (index = 0u; index < SENSOR_ALGO_CHANNEL_COUNT; index++) {
		snapshot->channels[index] = latest_snapshot.channels[index];
		snapshot->states[index] = latest_snapshot.states[index];
	}
}

/* 手动重新校准只重建基线，统一固定噪声策略持续生效。 */
void SensorAlgorithm_RequestRecalibration(void) { begin_calibration(); }

void SensorAlgorithm_RestartBaselineInitialization(void) { initialize_baseline_runtime(); }

void SensorAlgorithm_HandleProtocolCommand(uint8_t command_type, const uint8_t *payload, uint16_t payload_len) {
	sensor_force_calibration_t parameters;
	int32_t affine_gain_q16[CAPSENSE_AFFINE_TIER_COUNT];
	int32_t affine_offset[CAPSENSE_AFFINE_TIER_COUNT];
	uint32_t affine_calibration_id;
	uint16_t expected_payload_len;
	uint32_t index;

	/* 仿射停用覆盖旧上位机直发命令，不能仅依靠新界面禁用按钮。 */
	if (!DEVICE_HAS_AFFINE_CALIBRATION &&
		(command_type >= SENSOR_ALGO_COMMAND_STAGE_CAPSENSE_AFFINE_CALIBRATION) &&
		(command_type <= SENSOR_ALGO_COMMAND_GET_CAPSENSE_AFFINE_CALIBRATION)) {
		SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_UNSUPPORTED_COMMAND);
		return;
	}

	switch (command_type) {
	case SENSOR_ALGO_COMMAND_READ_LOW_FORCE:
	case SENSOR_ALGO_COMMAND_SAVE_LOW_FORCE: {
		/* 长度及载荷均验证，错误请求也通过专用响应返回。 */
		uint32_t token = 0u;
		uint8_t result = SENSOR_ALGO_RESULT_INVALID_PAYLOAD;
		bool save = command_type == SENSOR_ALGO_COMMAND_SAVE_LOW_FORCE;
		if (payload != NULL && payload_len >= 4u) {
			memcpy(&token, payload, sizeof(token));
		}
		if (SENSOR_ALGO_CHANNEL_COUNT < 8u) {
			result = SENSOR_ALGO_RESULT_UNSUPPORTED_REGION;
		} else if (payload != NULL && payload_len == (save ? 24u : 4u)) {
			if (save) {
				/* 四点和最低检测线同次校验、保存并更新RAM。 */
				float points[LOW_FORCE_POINT_COUNT];
				float detection_counts;
				memcpy(points, payload + 4u, sizeof(points));
				memcpy(&detection_counts, payload + 20u, sizeof(detection_counts));
				result = LowForce_Save(points, detection_counts);
			} else {
				/* 读取Flash同时刷新RAM，未保存通过有效位表达。 */
				(void)LowForce_Load();
				result = SENSOR_ALGO_RESULT_SUCCESS;
			}
		}
		send_low_force_response(command_type, result, token);
		return;
	}
	case SENSOR_ALGO_COMMAND_RECALIBRATE:
		if (payload_len != 0u) {
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_INVALID_PAYLOAD);
			return;
		}
		SensorAlgorithm_RequestRecalibration();
		SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_SUCCESS);
		return;
	case SENSOR_ALGO_COMMAND_SET_OUTPUT_MODE:
		if ((payload_len != 1u) || (payload[0u] > SENSOR_ALGO_OUTPUT_MODE_FORCE_CALIBRATION)) {
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_INVALID_PAYLOAD);
			return;
		}
		output_mode = payload[0u];
		SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_SUCCESS);
		return;
	case SENSOR_ALGO_COMMAND_READ_FORCE_CALIBRATION:
		if ((payload_len != 2u) || (payload[0u] >= SENSOR_ALGO_FORCE_CALIBRATION_REGION_COUNT) || (payload[1u] > SENSOR_ALGO_FORCE_CALIBRATION_FORCE_TYPE_DIRECTION)) {
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_UNSUPPORTED_REGION);
			return;
		}
		send_force_calibration_parameters_frame(payload[0u], payload[1u]);
		return;
	case SENSOR_ALGO_COMMAND_SET_FORCE_CALIBRATION:
		if ((payload_len < SENSOR_ALGO_FORCE_CALIBRATION_PARAMETERS_BASE_PAYLOAD_SIZE) || (payload[0u] >= SENSOR_ALGO_FORCE_CALIBRATION_REGION_COUNT) ||
			(payload[1u] > SENSOR_ALGO_FORCE_CALIBRATION_FORCE_TYPE_DIRECTION) || (payload[2u] != SENSOR_ALGO_FORCE_CALIBRATION_MODEL_PIECEWISE_LINEAR) || (payload[4u] != 0u) ||
			(payload[5u] != 0u) || (payload[6u] != 0u) || (payload[7u] != 0u)) {
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_INVALID_PAYLOAD);
			return;
		}
		expected_payload_len = (uint16_t)(SENSOR_ALGO_FORCE_CALIBRATION_PARAMETERS_BASE_PAYLOAD_SIZE + ((uint16_t)payload[3u] * 8u));
		/* 主力K表整张传输10/20点高力表或原48点表，低力四点参数独立保存。 */
		if ((payload[3u] < 2u) || (payload[3u] > SENSOR_ALGO_FORCE_CALIBRATION_MAX_NORMAL_POINTS) ||
			/* 切向数组仍为20节点，必须独立检查以防载荷越界。 */
			((payload[1u] == SENSOR_ALGO_FORCE_CALIBRATION_FORCE_TYPE_TANGENTIAL) && (payload[3u] > SENSOR_ALGO_FORCE_CALIBRATION_MAX_TANGENTIAL_POINTS)) ||
			((payload[0u] == SENSOR_ALGO_FORCE_CALIBRATION_REGION_PRIMARY) && (payload[1u] == SENSOR_ALGO_FORCE_CALIBRATION_FORCE_TYPE_NORMAL) &&
			 !primary_normal_point_count_valid(payload[3u])) ||
			((payload[1u] == SENSOR_ALGO_FORCE_CALIBRATION_FORCE_TYPE_DIRECTION) && (payload[3u] != SENSOR_ALGO_FORCE_CALIBRATION_DIRECTION_POINTS)) ||
			(payload_len != expected_payload_len)) {
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_INVALID_PAYLOAD);
			return;
		}
		parameters = force_calibration[payload[0u]];
		if (payload[1u] == SENSOR_ALGO_FORCE_CALIBRATION_FORCE_TYPE_NORMAL) {
			parameters.normal_point_count = payload[3u];
			for (index = 0u; index < parameters.normal_point_count; index++) {
				parameters.normal_raw_points[index] = read_float(&payload[8u + index * 8u]);
				parameters.normal_force_points[index] = read_float(&payload[12u + index * 8u]);
			}
			parameters.normal_valid = true;
			parameters.normal_dirty = true;
		} else if (payload[1u] == SENSOR_ALGO_FORCE_CALIBRATION_FORCE_TYPE_TANGENTIAL) {
			parameters.tangential_point_count = payload[3u];
			for (index = 0u; index < parameters.tangential_point_count; index++) {
				parameters.tangential_raw_points[index] = read_float(&payload[8u + index * 8u]);
				parameters.tangential_force_points[index] = read_float(&payload[12u + index * 8u]);
			}
			parameters.tangential_valid = true;
			parameters.tangential_dirty = true;
		} else {
			parameters.direction_point_count = payload[3u];
			for (index = 0u; index < parameters.direction_point_count; index++) {
				parameters.direction_raw_points[index] = read_float(&payload[8u + index * 8u]);
				parameters.direction_output_points[index] = read_float(&payload[12u + index * 8u]);
			}
			parameters.direction_valid = true;
			parameters.direction_dirty = true;
		}
		parameters.valid = parameters.normal_valid && parameters.tangential_valid && parameters.direction_valid;
		parameters.dirty = parameters.normal_dirty || parameters.tangential_dirty || parameters.direction_dirty;
		/* 原有数值校验之外，主法向新表还需覆盖完整高力范围。 */
		if (!force_calibration_values_valid(&parameters) ||
			((payload[0u] == SENSOR_ALGO_FORCE_CALIBRATION_REGION_PRIMARY) && parameters.normal_valid && !primary_normal_table_valid(&parameters))) {
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_INVALID_PAYLOAD);
			return;
		}
		force_calibration[payload[0u]] = parameters;
		SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_SUCCESS);
		return;
	case SENSOR_ALGO_COMMAND_SAVE_FORCE_CALIBRATION:
		if ((payload_len != 0u) || !save_force_calibration_to_nvm()) {
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_NVM_FAILURE);
			return;
		}
		SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_SUCCESS);
		return;
	case SENSOR_ALGO_COMMAND_DISCARD_FORCE_CALIBRATION:
		if (payload_len != 0u) {
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_INVALID_PAYLOAD);
			return;
		}
		memcpy(force_calibration, saved_force_calibration, sizeof(force_calibration));
		SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_SUCCESS);
		return;
	case SENSOR_ALGO_COMMAND_SET_CAPSENSE_NSUB:
		SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_UNSUPPORTED_COMMAND);
		return;
	case SENSOR_ALGO_COMMAND_GET_CAPSENSE_NSUB:
		if (payload_len != 0u) {
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_INVALID_PAYLOAD);
			return;
		}
		send_capsense_nsub_status_frame();
		return;
	case SENSOR_ALGO_COMMAND_GET_CAPSENSE_CONFIG:
		if (payload_len != 0u) {
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_INVALID_PAYLOAD);
			return;
		}
		send_capsense_config_frame();
		return;
	case SENSOR_ALGO_COMMAND_SET_CAPSENSE_CREF:
		SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_UNSUPPORTED_COMMAND);
		return;
	case SENSOR_ALGO_COMMAND_GET_CAPSENSE_CREF:
		if (payload_len != 0u) {
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_INVALID_PAYLOAD);
			return;
		}
		send_capsense_cref_status_frame();
		return;
	/* 处理 V8 全通道易失性 Nsub 设置命令。 */
	case SENSOR_ALGO_COMMAND_SET_CAPSENSE_NSUB_TIER:
		SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_UNSUPPORTED_COMMAND);
		return;
	/* 处理 V8 全通道易失性 Nsub 查询命令。 */
	case SENSOR_ALGO_COMMAND_GET_CAPSENSE_NSUB_TIER:
		/* 查询命令不允许携带载荷。 */
		if (payload_len != 0u) {
			/* 返回载荷长度错误。 */
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_INVALID_PAYLOAD);
			/* 结束本次命令处理。 */
			return;
		}
		/* 返回请求值、八通道实际值和执行状态。 */
		send_capsense_nsub_tier_status_frame();
		/* 结束本次命令处理。 */
		return;
	case SENSOR_ALGO_COMMAND_SET_CAPSENSE_MATRIX_CONFIG:
	#if !DEVICE_HAS_RUNTIME_CAPSENSE_CONFIG
		/* 当前固件不支持运行时修改三参数。 */
		SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_UNSUPPORTED_COMMAND);
		return;
	#else
		/* 三参数命令固定携带Nsub、TxDivider和RefCDAC共5字节。 */
		if (payload_len != 5u) {
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_INVALID_PAYLOAD);
			return;
		}
	#if DEVICE_HAS_MATRIX_AUTORANGE
		/* 自动量程固件必须先切换到手动模式。 */
		if (!CapSenseAutoRange_IsManualMode()) {
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_INVALID_PAYLOAD);
			return;
		}
	#endif
		/* 固定档位固件直接排队，自动量程固件在手动模式下排队。 */
		if (!CapSenseMatrixConfig_Request(
				(uint16_t)((uint16_t)payload[0u] | ((uint16_t)payload[1u] << 8u)),
				(uint16_t)((uint16_t)payload[2u] | ((uint16_t)payload[3u] << 8u)),
				payload[4u])) {
			/* 拒绝超出中间件合法范围的参数。 */
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_INVALID_PAYLOAD);
			return;
		}
		/* 请求已排队，将在CapSense扫描空闲点应用。 */
		SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_SUCCESS);
		return;
	#endif
	case SENSOR_ALGO_COMMAND_GET_CAPSENSE_MATRIX_CONFIG:
		if (payload_len != 0u) {
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_INVALID_PAYLOAD);
			return;
		}
		send_capsense_matrix_config_status_frame();
		return;
	case SENSOR_ALGO_COMMAND_GET_CAPSENSE_AUTORANGE_STATUS:
	#if !DEVICE_HAS_MATRIX_AUTORANGE
		/* 固定档位固件没有自动量程状态。 */
		SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_UNSUPPORTED_COMMAND);
		return;
	#else
		/* 查询命令不接受载荷，合法请求立即返回0x28状态帧。 */
		if (payload_len != 0u) {
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_INVALID_PAYLOAD);
			return;
		}
		send_capsense_autorange_status_frame(true);
		return;
	#endif
	/* 处理上位机发出的Ref手动校准模式控制命令。 */
	case SENSOR_ALGO_COMMAND_SET_CAPSENSE_AUTORANGE_CALIBRATION:
	#if !DEVICE_HAS_MATRIX_AUTORANGE
		/* 固定档位固件禁止进入手动切档模式。 */
		SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_UNSUPPORTED_COMMAND);
		return;
	#else
		/* 载荷固定为模式和Ref两个字节，模式只允许自动0或手动1。 */
		if ((payload_len != 2u) || (payload[0u] > 1u)) {
			/* 返回载荷格式错误。 */
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_INVALID_PAYLOAD);
			/* 结束本次非法命令处理。 */
			return;
		}

		/* 自动模式要求Ref字段为0，防止主机误把退出命令当成切档。 */
		if ((payload[0u] == 0u) && (payload[1u] != 0u)) {
			/* 返回载荷内容错误。 */
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_INVALID_PAYLOAD);
			/* 结束本次非法命令处理。 */
			return;
		}

		/* 请求自动量程模块切换控制模式并校验手动Ref档位。 */
		if (!CapSenseAutoRange_SetManualMode(payload[0u] != 0u, payload[1u])) {
			/* 非七档固定Ref时返回参数错误。 */
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_INVALID_PAYLOAD);
			/* 结束本次失败命令处理。 */
			return;
		}

		/* 模式或档位请求已经接受，向上位机返回成功确认。 */
		SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_SUCCESS);
		/* 立即返回状态帧，让界面不必等待周期状态。 */
		send_capsense_autorange_status_frame(true);
		/* 结束本次命令处理。 */
		return;
	#endif
	/* 暂存单个通道的七档仿射参数，本命令不写Flash。 */
	case SENSOR_ALGO_COMMAND_STAGE_CAPSENSE_AFFINE_CALIBRATION:
		/* 校验固定载荷长度和保留字节。 */
		if ((payload_len != SENSOR_ALGO_CAPSENSE_AFFINE_STAGE_PAYLOAD_SIZE) ||
			(payload[7u] != 0u)) {
			send_capsense_affine_operation_status_frame(
				command_type,
				SENSOR_ALGO_RESULT_INVALID_PAYLOAD,
				0xFFu,
				0u);
			return;
		}

		/* 读取整组内容ID。 */
		affine_calibration_id = (uint32_t)read_i32(&payload[0u]);

		/* 按固定档位顺序解析本通道七组定点参数。 */
		for (index = 0u; index < CAPSENSE_AFFINE_TIER_COUNT; index++) {
			affine_gain_q16[index] = read_i32(&payload[8u + index * 8u]);
			affine_offset[index] = read_i32(&payload[12u + index * 8u]);
		}

		/* 参数范围、通道、格式和事务ID由持久化模块统一校验。 */
		if (!CapSenseAffineCalibration_StageChannel(
				affine_calibration_id,
				payload[4u],
				payload[5u],
				payload[6u],
				affine_gain_q16,
				affine_offset)) {
			send_capsense_affine_operation_status_frame(
				command_type,
				SENSOR_ALGO_RESULT_INVALID_PAYLOAD,
				payload[4u],
				affine_calibration_id);
			return;
		}

		/* 暂存成功后确认，当前运行Raw仍完全不变。 */
		send_capsense_affine_operation_status_frame(
			command_type,
			SENSOR_ALGO_RESULT_SUCCESS,
			payload[4u],
			affine_calibration_id);
		return;
	/* 将同一事务的完整28组参数一次性写入Flash。 */
	case SENSOR_ALGO_COMMAND_SAVE_CAPSENSE_AFFINE_CALIBRATION:
		/* 保存命令只携带四字节参数ID。 */
		if (payload_len != 4u) {
			send_capsense_affine_operation_status_frame(
				command_type,
				SENSOR_ALGO_RESULT_INVALID_PAYLOAD,
				0xFFu,
				0u);
			return;
		}

		/* 只有四通道到齐、内容ID一致且写后回读成功才确认保存。 */
		affine_calibration_id = (uint32_t)read_i32(&payload[0u]);
		if (!CapSenseAffineCalibration_Save(affine_calibration_id)) {
			send_capsense_affine_operation_status_frame(
				command_type,
				SENSOR_ALGO_RESULT_NVM_FAILURE,
				0xFFu,
				affine_calibration_id);
			return;
		}

		/* Flash保存和立即回读均成功。 */
		send_capsense_affine_operation_status_frame(
			command_type,
			SENSOR_ALGO_RESULT_SUCCESS,
			0xFFu,
			affine_calibration_id);
		return;
	/* 回读当前RAM中由Flash确认的单通道仿射参数。 */
	case SENSOR_ALGO_COMMAND_GET_CAPSENSE_AFFINE_CALIBRATION:
		/* 查询命令只携带一个物理通道号。 */
		if ((payload_len != 1u) ||
			(payload[0u] < CAPSENSE_AFFINE_CHANNEL_FIRST) ||
			(payload[0u] >= (CAPSENSE_AFFINE_CHANNEL_FIRST + CAPSENSE_AFFINE_CHANNEL_COUNT))) {
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_INVALID_PAYLOAD);
			return;
		}

		/* 返回参数内容、参数ID、代次和Flash加载标志。 */
		send_capsense_affine_calibration_status_frame(payload[0u]);
		return;
	/* 完成串口确认发送后执行软件复位。 */
	case SENSOR_ALGO_COMMAND_SOFTWARE_RESET:
		/* 软件复位命令不允许携带载荷。 */
		if (payload_len != 0u) {
			send_capsense_affine_operation_status_frame(
				command_type,
				SENSOR_ALGO_RESULT_INVALID_PAYLOAD,
				0xFFu,
				0u);
			return;
		}

		/* 阻塞发送成功确认，确保上位机在复位前收到应答。 */
		send_capsense_affine_operation_status_frame(
			command_type,
			SENSOR_ALGO_RESULT_SUCCESS,
			0xFFu,
			0u);
		/* 立即复位，启动路径将重新从Flash加载仿射参数。 */
		NVIC_SystemReset();
		return;
	/* 处理上位机连接后发送的设备身份查询。 */
	case SENSOR_ALGO_COMMAND_GET_DEVICE_INFO:
		/* 身份查询不允许携带载荷。 */
		if (payload_len != 0u) {
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_INVALID_PAYLOAD);
			return;
		}
		/* 立即返回当前编译变体的手指类型、能力和名称。 */
		send_device_info_frame();
		return;
	/* 处理上位机发出的逐通道CapSense参数查询。 */
	case SENSOR_ALGO_COMMAND_GET_CAPSENSE_CHANNEL_CONFIG:
		if (payload_len != 0u) {
			SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_INVALID_PAYLOAD);
			return;
		}
		/* 返回配置值、扫描帧硬件值和每路Raw满量程。 */
		send_capsense_channel_config_status_frame();
		return;
	default:
		SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_UNSUPPORTED_COMMAND);
		return;
	}
}

static void send_capsense_nsub_status_frame(void) {
	uint32_t offset = 0u;
	uint8_t frame[SENSOR_ALGO_CAPSENSE_NSUB_STATUS_FRAME_SIZE];
	capsense_runtime_nsub_status_t status;

	CapSenseRuntime_GetNsubStatus(&status);
	write_frame_header(frame, &offset, SENSOR_ALGO_FRAME_TYPE_CAPSENSE_NSUB_STATUS, 0u, SENSOR_ALGO_CAPSENSE_NSUB_STATUS_PAYLOAD_SIZE);
	write_u16(frame, &offset, status.requested_nsub);
	write_u16(frame, &offset, status.actual_nsub);
	write_u32(frame, &offset, status.set_status);
	write_u32(frame, &offset, status.get_status);
	send_command_response_frame(frame, SENSOR_ALGO_CAPSENSE_NSUB_STATUS_FRAME_SIZE);
}

static void send_capsense_config_frame(void) {
	uint32_t offset = 0u;
	uint32_t sensor_index;
	uint8_t frame[SENSOR_ALGO_CAPSENSE_CONFIG_FRAME_SIZE];
	capsense_runtime_config_t config;
	sensor_algorithm_snapshot_t snapshot;

	CapSenseRuntime_GetConfig(&config);
	SensorAlgorithm_GetLatestSnapshot(&snapshot);
	write_frame_header(frame, &offset, SENSOR_ALGO_FRAME_TYPE_CAPSENSE_CONFIG, 0u, SENSOR_ALGO_CAPSENSE_CONFIG_PAYLOAD_SIZE);
	write_u16(frame, &offset, config.measurement_nsub);
	write_u16(frame, &offset, config.hardware_nsub);
	write_u16(frame, &offset, config.init_nsub);
	frame[offset++] = config.ref_cdac;
	frame[offset++] = config.hardware_ref_cdac;
	for (sensor_index = 0u; sensor_index < CAPSENSE_RUNTIME_MATRIX_SENSOR_COUNT; sensor_index++) {
		frame[offset++] = config.comp_cdac[sensor_index];
	}
	for (sensor_index = 0u; sensor_index < CAPSENSE_RUNTIME_MATRIX_SENSOR_COUNT; sensor_index++) {
		frame[offset++] = config.hardware_comp_cdac[sensor_index];
	}
	write_u16(frame, &offset, config.sense_clk_div);
	write_u16(frame, &offset, config.hardware_sense_clk_div);
	write_u16(frame, &offset, config.max_count);
	write_u16(frame, &offset, snapshot.channels[SENSOR_ALGO_CAPSENSE_CONFIG_SENSOR_INDEX].raw);
	write_u16(frame, &offset, snapshot.channels[SENSOR_ALGO_CAPSENSE_CONFIG_SENSOR_INDEX].baseline);
	write_u16(frame, &offset, snapshot.channels[SENSOR_ALGO_CAPSENSE_CONFIG_SENSOR_INDEX].signal);
	frame[offset++] = config.smartsense_enabled;
	frame[offset++] = config.auto_calibration_enabled;
	frame[offset++] = config.capsense_busy;
	frame[offset++] = 0u;
	write_u32(frame, &offset, config.scan_time_us);
	write_u32(frame, &offset, config.fps_milli);
	write_u32(frame, &offset, config.get_status);
	write_u32(frame, &offset, config.persistence_status);
	frame[offset++] = config.persistent_config_loaded;
	write_u32(frame, &offset, config.baseline_status);
	frame[offset++] = config.baseline_update_pending;
	send_command_response_frame(frame, SENSOR_ALGO_CAPSENSE_CONFIG_FRAME_SIZE);
}

static void send_capsense_cref_status_frame(void) {
	uint32_t offset = 0u;
	uint8_t frame[SENSOR_ALGO_CAPSENSE_CREF_STATUS_FRAME_SIZE];
	capsense_runtime_cref_status_t status;

	CapSenseRuntime_GetCrefStatus(&status);
	write_frame_header(frame, &offset, SENSOR_ALGO_FRAME_TYPE_CAPSENSE_CREF_STATUS, 0u, SENSOR_ALGO_CAPSENSE_CREF_STATUS_PAYLOAD_SIZE);
	frame[offset++] = status.requested_cref;
	frame[offset++] = status.actual_cref;
	write_u32(frame, &offset, status.set_status);
	write_u32(frame, &offset, status.get_status);
	send_command_response_frame(frame, SENSOR_ALGO_CAPSENSE_CREF_STATUS_FRAME_SIZE);
}

/* 发送 V8 全通道易失性 Nsub 状态。 */
static void send_capsense_nsub_tier_status_frame(void) {
	/* 保存帧写入偏移。 */
	uint32_t offset = 0u;
	/* 保存通道遍历序号。 */
	uint32_t channel_index;
	/* 分配固定长度的状态帧缓存。 */
	uint8_t frame[SENSOR_ALGO_CAPSENSE_NSUB_TIER_STATUS_FRAME_SIZE];
	/* 保存 Nsub 控制模块状态快照。 */
	capsense_nsub_control_status_t status;

	/* 读取当前易失性 Nsub 状态。 */
	CapSenseNsubControl_GetStatus(&status);
	/* 写入 V8 状态帧头，并明确声明八个通道。 */
	write_frame_header(frame, &offset, SENSOR_ALGO_FRAME_TYPE_CAPSENSE_NSUB_TIER_STATUS, SENSOR_ALGO_CHANNEL_COUNT,
					   SENSOR_ALGO_CAPSENSE_NSUB_TIER_STATUS_PAYLOAD_SIZE);
	/* 写入最近请求的档位。 */
	write_u16(frame, &offset, status.requested_nsub);
	/* 依次写入 CH0～CH7 的实际档位。 */
	for (channel_index = 0u; channel_index < SENSOR_ALGO_CHANNEL_COUNT; channel_index++) {
		/* 写入当前通道的实际 Nsub。 */
		write_u16(frame, &offset, status.actual_nsub[channel_index]);
	/* 结束八通道实际值写入循环。 */
	}
	/* 写入操作是否仍在等待完成。 */
	frame[offset++] = status.pending;
	/* 写入最近一次操作结果。 */
	frame[offset++] = status.result;
	/* 通过命令响应队列发送完整状态帧。 */
	send_command_response_frame(frame, SENSOR_ALGO_CAPSENSE_NSUB_TIER_STATUS_FRAME_SIZE);
/* 结束 V8 全通道易失性 Nsub 状态发送函数。 */
}

static void send_capsense_matrix_config_status_frame(void) {
	uint32_t offset = 0u;
	uint8_t frame[SENSOR_ALGO_CAPSENSE_MATRIX_CONFIG_STATUS_FRAME_SIZE];
	capsense_matrix_config_status_t status;
	CapSenseMatrixConfig_GetStatus(&status);
	write_frame_header(frame, &offset, SENSOR_ALGO_FRAME_TYPE_CAPSENSE_MATRIX_CONFIG_STATUS, DEVICE_MATRIX20 ? SENSOR_ALGO_CHANNEL_COUNT : 4u,
		SENSOR_ALGO_CAPSENSE_MATRIX_CONFIG_STATUS_PAYLOAD_SIZE);
	write_u16(frame, &offset, status.requested_nsub);
	write_u16(frame, &offset, status.requested_tx_clk_div);
	frame[offset++] = status.requested_ref_cdac;
	write_u16(frame, &offset, status.actual_nsub);
	write_u16(frame, &offset, status.actual_tx_clk_div);
	frame[offset++] = status.actual_ref_cdac;
	frame[offset++] = status.pending;
	frame[offset++] = status.result;
	send_command_response_frame(frame, SENSOR_ALGO_CAPSENSE_MATRIX_CONFIG_STATUS_FRAME_SIZE);
}

/* 发送当前编译变体的设备身份信息。 */
static void send_device_info_frame(void) {
	uint32_t offset = 0u;
	uint32_t name_index;
	uint8_t frame[SENSOR_ALGO_DEVICE_INFO_FRAME_SIZE];
	static const char device_name[] = DEVICE_FINGER_NAME;

	/* 通道数字段直接给出当前设备可用的传感器数量。 */
	write_frame_header(
		frame,
		&offset,
		SENSOR_ALGO_FRAME_TYPE_DEVICE_INFO,
		SENSOR_ALGO_CHANNEL_COUNT,
		SENSOR_ALGO_DEVICE_INFO_PAYLOAD_SIZE);
	/* 固定字段依次为格式版本、手指类型、能力位和设备名长度。 */
	frame[offset++] = 1u;
	frame[offset++] = (uint8_t)DEVICE_FINGER_TYPE;
	frame[offset++] = (uint8_t)DEVICE_CAPABILITY_FLAGS;
	frame[offset++] = (uint8_t)SENSOR_ALGO_DEVICE_NAME_LENGTH;
	/* 设备名使用ASCII且不发送字符串结尾零字符。 */
	for (name_index = 0u; name_index < SENSOR_ALGO_DEVICE_NAME_LENGTH; name_index++) {
		frame[offset++] = (uint8_t)device_name[name_index];
	}
	/* 身份信息使用高优先级命令响应队列发送。 */
	send_command_response_frame(frame, SENSOR_ALGO_DEVICE_INFO_FRAME_SIZE);
}

/* 返回所有逻辑通道当前真正参与扫描的Nsub、TxDivider和Ref CDAC。 */
static void send_capsense_channel_config_status_frame(void) {
	uint32_t offset = 0u;
	uint32_t channel;
	uint8_t frame[SENSOR_ALGO_CAPSENSE_CHANNEL_CONFIG_FRAME_SIZE];
	capsense_channel_config_t configs[SENSOR_ALGO_CHANNEL_COUNT];
	bool read_succeeded = CapSenseChannelConfig_ReadAll(configs);

	write_frame_header(
		frame,
		&offset,
		SENSOR_ALGO_FRAME_TYPE_CAPSENSE_CHANNEL_CONFIG_STATUS,
		SENSOR_ALGO_CHANNEL_COUNT,
		SENSOR_ALGO_CAPSENSE_CHANNEL_CONFIG_PAYLOAD_SIZE);
	frame[offset++] = 1u;
	frame[offset++] = SENSOR_ALGO_CAPSENSE_CHANNEL_CONFIG_ENTRY_SIZE;
	frame[offset++] = read_succeeded ? 1u : 0u;
	frame[offset++] = 0u;

	for (channel = 0u; channel < SENSOR_ALGO_CHANNEL_COUNT; channel++) {
		write_u16(frame, &offset, configs[channel].configured_nsub);
		write_u16(frame, &offset, configs[channel].hardware_nsub);
		write_u16(frame, &offset, configs[channel].configured_tx_clk_div);
		write_u16(frame, &offset, configs[channel].hardware_tx_clk_div);
		write_u16(frame, &offset, configs[channel].max_raw_count);
		frame[offset++] = configs[channel].configured_ref_cdac;
		frame[offset++] = configs[channel].hardware_ref_cdac;
		frame[offset++] = configs[channel].widget_id;
		frame[offset++] = configs[channel].sensor_id;
	}

	send_command_response_frame(frame, SENSOR_ALGO_CAPSENSE_CHANNEL_CONFIG_FRAME_SIZE);
}

/* 序列化自动量程状态，可作为命令响应或周期输出发送。 */
#if DEVICE_HAS_MATRIX_AUTORANGE
static void send_capsense_autorange_status_frame(bool command_response) {
	uint32_t offset = 0u;
	uint8_t frame[SENSOR_ALGO_CAPSENSE_AUTORANGE_STATUS_FRAME_SIZE];
	capsense_autorange_status_t status;

	/* 获取同一时刻的状态快照，避免组帧期间字段发生变化。 */
	CapSenseAutoRange_GetStatus(&status);
	write_frame_header(frame, &offset, SENSOR_ALGO_FRAME_TYPE_CAPSENSE_AUTORANGE_STATUS,
		CAPSENSE_AUTORANGE_CHANNEL_COUNT, SENSOR_ALGO_CAPSENSE_AUTORANGE_STATUS_PAYLOAD_SIZE);
	write_u16(frame, &offset, status.nsub);
	write_u16(frame, &offset, status.tx_div);
	write_u16(frame, &offset, status.max_count);
	frame[offset++] = status.actual_ref_cdac;
	frame[offset++] = status.tier_index;
	frame[offset++] = status.tier_count;
	frame[offset++] = status.state;
	frame[offset++] = status.flags;
	/* 原保留字节用于回传上位机最近请求的Ref，不改变状态帧长度。 */
	frame[offset++] = status.requested_ref_cdac;
	write_u16(frame, &offset, status.peak_raw);
	write_u16(frame, &offset, status.filtered_peak_raw);
	frame[offset++] = status.high_count;
	frame[offset++] = status.low_count;
	frame[offset++] = status.cooldown_frames;
	frame[offset++] = status.blank_frames;

	/* 查询响应进入高优先级命令队列，周期状态进入普通输出队列。 */
	if (command_response) {
		send_command_response_frame(frame, SENSOR_ALGO_CAPSENSE_AUTORANGE_STATUS_FRAME_SIZE);
	} else {
		send_output_frame(frame, SENSOR_ALGO_CAPSENSE_AUTORANGE_STATUS_FRAME_SIZE);
	}
}
#endif

/* 序列化指定通道的当前Flash仿射参数和加载状态。 */
static void send_capsense_affine_calibration_status_frame(uint8_t channel) {
	uint32_t offset = 0u;
	uint32_t tier_index;
	uint8_t frame[SENSOR_ALGO_CAPSENSE_AFFINE_STATUS_FRAME_SIZE];
	capsense_affine_channel_status_t status;

	/* 通道已经由命令入口校验，此处仍防止内部误调用。 */
	if (!CapSenseAffineCalibration_GetChannelStatus(channel, &status)) {
		SensorAlgorithm_SendCommandAck(SENSOR_ALGO_RESULT_INVALID_PAYLOAD);
		return;
	}

	/* 写入固定协议头和16字节状态元数据。 */
	write_frame_header(
		frame,
		&offset,
		SENSOR_ALGO_FRAME_TYPE_CAPSENSE_AFFINE_CALIBRATION_STATUS,
		CAPSENSE_AFFINE_CHANNEL_COUNT,
		SENSOR_ALGO_CAPSENSE_AFFINE_STATUS_PAYLOAD_SIZE);
	frame[offset++] = CAPSENSE_AFFINE_FORMAT_VERSION;
	frame[offset++] = channel;
	frame[offset++] = CAPSENSE_AFFINE_TIER_COUNT;
	frame[offset++] = status.flags;
	write_u32(frame, &offset, status.calibration_id);
	write_u32(frame, &offset, status.generation);
	write_u32(frame, &offset, status.nvm_status);

	/* 按固定Ref顺序写入七组Q16增益和整数偏移。 */
	for (tier_index = 0u; tier_index < CAPSENSE_AFFINE_TIER_COUNT; tier_index++) {
		write_u32(frame, &offset, (uint32_t)status.gain_q16[tier_index]);
		write_u32(frame, &offset, (uint32_t)status.offset[tier_index]);
	}

	/* 回读帧使用阻塞命令响应路径，避免被周期数据队列丢弃。 */
	send_command_response_frame(frame, SENSOR_ALGO_CAPSENSE_AFFINE_STATUS_FRAME_SIZE);
}

/* 返回可精确关联到新命令的操作结果。 */
static void send_capsense_affine_operation_status_frame(
	uint8_t command_type,
	uint8_t result,
	uint8_t channel,
	uint32_t calibration_id) {
	uint32_t offset = 0u;
	uint8_t frame[SENSOR_ALGO_CAPSENSE_AFFINE_OPERATION_FRAME_SIZE];

	/* 写入固定协议头。 */
	write_frame_header(
		frame,
		&offset,
		SENSOR_ALGO_FRAME_TYPE_CAPSENSE_AFFINE_OPERATION_STATUS,
		0u,
		SENSOR_ALGO_CAPSENSE_AFFINE_OPERATION_PAYLOAD_SIZE);

	/* 回传原命令、结果、物理通道和参数ID。 */
	frame[offset++] = command_type;
	frame[offset++] = result;
	frame[offset++] = channel;
	frame[offset++] = 0u;
	write_u32(frame, &offset, calibration_id);

	/* 操作结果使用命令响应路径保证送达。 */
	send_command_response_frame(frame, SENSOR_ALGO_CAPSENSE_AFFINE_OPERATION_FRAME_SIZE);
}

void SensorAlgorithm_SendCommandAck(uint8_t result) {
	uint32_t offset = 0u;
	uint8_t frame[SENSOR_ALGO_ACK_FRAME_SIZE];

	write_frame_header(frame, &offset, SENSOR_ALGO_FRAME_TYPE_COMMAND_ACK, 0u, SENSOR_ALGO_ACK_PAYLOAD_SIZE);
	frame[offset] = result;
	send_command_response_frame(frame, SENSOR_ALGO_ACK_FRAME_SIZE);
}

float count_normal_k_f(uint16_t signal, int32_t signal_k_q16) { return (signal != 0) ? ((float)signal_k_q16 * 100.0f) / (65536.0f * signal) : 0.0f; }

int32_t count_normal_k_x10000(uint16_t signal, int32_t signal_k_q16) {
	int32_t normalized_k_x10000;

	if (signal != 0) {
		normalized_k_x10000 = (int32_t)(((int64_t)signal_k_q16 * 100 * 10000) / ((int64_t)65536 * signal));
	} else {
		normalized_k_x10000 = 0;
	}
	return normalized_k_x10000;
}

void SensorAlgorithm_Process(const uint16_t raw_values[SENSOR_ALGO_CHANNEL_COUNT], sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT]) {
	uint32_t index;

/* 仅旧手指产品使用力组接触提示，矩阵各通道独立处理。 */
#if DEVICE_HAS_FORCE_OUTPUT
	update_force_vector_group_contact_hint(raw_values);
	update_force_vector_group_snapshot();
#endif

	for (index = 0u; index < SENSOR_ALGO_CHANNEL_COUNT; index++) {
		process_channel_sample(&channel_sample_context, index, raw_values[index], &outputs[index]);
		outputs[index].fast_normalized_k_x100 = ChannelState_GetFastNormalizedKX100(&channel_state_machines[index]);
		outputs[index].slow_normalized_k_x100 = ChannelState_GetSlowNormalizedKX100(&channel_state_machines[index]);
		update_channel_drift(index, outputs[index].raw, outputs[index].signal, &outputs[index]);
	}

#if 0
	if (channel_states[7] != SENSOR_ALGO_STATE_IDLE) {
		int32_t ch7_normalized_k_x10000 = count_normal_k_x10000(outputs[7].signal, outputs[7].signal_k_q16);
		printf("CH7 normalized k (x10000): %ld\r\n", (long)ch7_normalized_k_x10000);
	}
#endif

	finish_global_calibration_frame();

	for (index = 0u; index < SENSOR_ALGO_CHANNEL_COUNT; index++) {
		outputs[index].baseline = q16_to_u16(baseline_q16[index]);
		latest_snapshot.channels[index] = outputs[index];
		/* 快照直接保存真实状态，和本帧基线执行策略一致。 */
		latest_snapshot.states[index] = ChannelState_Get(&channel_state_machines[index]);
	}
}

static void process_sensor_frame(const uint16_t raw_values[SENSOR_ALGO_CHANNEL_COUNT], sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT]) {

	/* 处理本轮原始采样值，并把各通道计算结果写入 outputs。 */
	SensorAlgorithm_Process(raw_values, outputs);

	/* 仅在启用力矢量输出功能时编译下面的力矢量处理代码。 */
#if SENSOR_ALGO_FORCE_VECTOR_OUTPUT

	/* 根据本轮通道输出更新指尖和指腹的力矢量计算结果。 */
	update_force_vector(outputs);

	/* 更新两个力传感器组的释放恢复状态，并在恢复期间抑制无效输出。 */
	update_force_group_recoveries(outputs);

	/* 结束力矢量输出功能的条件编译区域。 */
#endif
}

static void send_sensor_frame(const sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT]) {
/* 矩阵使用V21长度帧发送各通道数据，不发送四通道统计或合力。 */
#if DEVICE_MATRIX20
	if (startup_quiet_frame_count < SENSOR_ALGO_STARTUP_QUIET_FRAMES) {
		startup_quiet_frame_count++;
		return;
	}
	if (output_mode == SENSOR_ALGO_OUTPUT_MODE_RUN) {
		send_signal_frame(outputs);
	} else {
		send_debug_frame(outputs);
	}
	return;
#endif
	/* 启动静默帧数尚未达到设定值时，暂不向外发送算法数据。 */
	if (startup_quiet_frame_count < SENSOR_ALGO_STARTUP_QUIET_FRAMES) {

		/* 记录已经跳过的启动静默帧数。 */
		startup_quiet_frame_count++;

		/* 提前结束本轮处理，等待启动阶段稳定后再发送数据。 */
		return;
	}

	/* 调试输出模式下发送完整的算法内部状态和力矢量调试数据。 */
	/* 先发送统计帧，接收端将其与紧随其后的通道采样关联。 */
#if SENSOR_ALGO_CHANNEL_COUNT >= 8u
	send_channel_4567_statistics_frame(outputs);
#endif
	if (output_mode == SENSOR_ALGO_OUTPUT_MODE_DEBUG) {

		/* V20仅上传Raw、Baseline、Signal、快慢K和真实状态。 */
		send_debug_frame(outputs);

		/* 调试模式同步上传带标定有效位的Type 0x02法向力，供上位机直接显示MCU换算后的牛顿值。 */
		send_run_force_frame();

		/* 仅在启用力矢量输出功能时编译下面两种力矢量调试帧。 */
#if SENSOR_ALGO_FORCE_VECTOR_OUTPUT

		/* 发送主区域（指尖）的力矢量调试帧及接触状态。 */
		send_force_vector_frame(SENSOR_ALGO_FRAME_TYPE_FORCE_VECTOR, SENSOR_ALGO_FORCE_CALIBRATION_REGION_PRIMARY, &fingertip_force_vector.debug,
								fingertip_force_vector.contact_active);

		/* 发送次区域（指腹）的力矢量调试帧及接触状态。 */
		send_force_vector_frame(SENSOR_ALGO_FRAME_TYPE_FINGER_PAD_FORCE_VECTOR, SENSOR_ALGO_FORCE_CALIBRATION_REGION_SECONDARY, &finger_pad_force_vector.debug,
								finger_pad_force_vector.contact_active);

		/* 结束力矢量调试帧的条件编译区域。 */
#endif

		/* 力标定模式下发送用于建立标定关系的实时采样数据。 */
	} else if (output_mode == SENSOR_ALGO_OUTPUT_MODE_FORCE_CALIBRATION) {
		/* 发送当前两个力区域的标定数据帧。 */
		send_force_calibration_frame();
		/* 其他情况为正常运行模式，发送通道信号和标定后的力数据。 */
	} else {
		/* 发送所有通道的 signal 值。 */
		send_signal_frame(outputs);
		/* 发送指尖和指腹两个区域的法向力、切向力及方向。 */
		send_run_force_frame();
	}

	/* Send the CH4..CH7 signal total as a separate frame when enabled. */
#if SENSOR_ALGO_CHANNEL_4567_SIGNAL_SUM_OUTPUT
	send_channel_4567_signal_sum_frame(outputs);
#endif

	/* 仅八通道矩阵固件周期发送自动量程状态。 */
#if DEVICE_HAS_MATRIX_AUTORANGE
	autorange_status_period_count++;
	if (autorange_status_period_count >= SENSOR_ALGO_CAPSENSE_AUTORANGE_STATUS_PERIOD_FRAMES) {
		autorange_status_period_count = 0u;
		send_capsense_autorange_status_frame(false);
	}
#endif
}

int _write(int file, char *ptr, int len) {
	(void)file;

	if ((ptr == NULL) || (len <= 0)) {
		return 0;
	}

	const uint8_t *frame = (const uint8_t *)ptr;
	uint32_t size = (uint32_t)len;

	send_output_frame(frame, size);

	return len;
}

/* 使用唯一硬件Raw输入驱动算法并上传V20诊断帧。 */
void SensorAlgorithm_ProcessAndSend(
	const uint16_t raw_values[SENSOR_ALGO_CHANNEL_COUNT]) {
	/* 保存本轮所有传感器通道的算法输出。 */
	sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT];
	/* 保存当前自动量程状态，用于隔离切档合成轨迹和通道状态机。 */
	capsense_autorange_status_t autorange_status;
	uint32_t channel_index;

	/* 读取过渡剩余帧数，识别本帧是否仍在切档以及是否刚结束切档。 */
	CapSenseAutoRange_GetStatus(&autorange_status);
	autorange_transition_active = autorange_status.blank_frames != 0u;
	autorange_transition_finished =
		autorange_transition_was_active && !autorange_transition_active;

	/* 保存本帧过渡状态，供下一帧检测结束边沿。 */
	autorange_transition_was_active = autorange_transition_active;

	/* 缓存同一扫描的硬件Raw，用于诊断帧和原始值统计。 */
	for (channel_index = 0u; channel_index < SENSOR_ALGO_CHANNEL_COUNT; channel_index++) {
		latest_hardware_raw[channel_index] = raw_values[channel_index];
	}

	/* 处理原始采样、力矢量和释放恢复状态。 */
	process_sensor_frame(raw_values, outputs);

	/* 根据当前输出模式发送处理结果。 */
	send_sensor_frame(outputs);
}

static void initialize_baseline_runtime(void) {
	uint32_t index;

	/* 初始化时不存在尚未结束的自动量程切档过渡。 */
	autorange_transition_active = false;
	autorange_transition_was_active = false;
	autorange_transition_finished = false;

	for (index = 0u; index < SENSOR_ALGO_CHANNEL_COUNT; index++) {
		baseline_q16[index] = 0u;
		initialized[index] = false;
		latest_snapshot.channels[index].raw = 0u;
		latest_snapshot.channels[index].baseline = 0u;
		latest_snapshot.channels[index].signal = 0u;
		/* 噪声统一按固定值计算，不受校准结果影响。 */
		latest_snapshot.channels[index].noise = SENSOR_ALGO_FIXED_NOISE;
		latest_snapshot.states[index] = SENSOR_ALGO_STATE_IDLE;

		reset_channel_runtime_state(index, true);
	}

	calibrated = false;
	calibration_count = 0u;
	startup_quiet_frame_count = 0u;
	autorange_status_period_count = 0u;
	force_vector_group_signal_sum = 0u;
	force_vector_group_noise_sum = 0u;
	force_vector_group_contact_hint = false;
	force_vector_group_touch_peak = 0.0f;
#if SENSOR_ALGO_FORCE_VECTOR_OUTPUT
	force_vector_context_reset(&fingertip_force_vector);
	force_vector_context_reset(&finger_pad_force_vector);
	reset_force_group_recovery(&fingertip_group_recovery);
	reset_force_group_recovery(&finger_pad_group_recovery);
#endif
}

void SensorAlgorithm_UartTxPoll(void) {
#if (TUNER_INTERFACE_TYPE == UART)
	uint32_t sent_count;

	while (uart_tx_count != 0u) {
		sent_count = Cy_SCB_UART_PutArray(CYBSP_UART_HW, &uart_tx_buffers[uart_tx_head][uart_tx_offset], uart_tx_sizes[uart_tx_head] - uart_tx_offset);
		if (sent_count == 0u) {
			break;
		}

		uart_tx_offset += sent_count;
		if (uart_tx_offset == uart_tx_sizes[uart_tx_head]) {
			uart_tx_head = (uart_tx_head + 1u) % SENSOR_ALGO_UART_TX_BUFFER_COUNT;
			uart_tx_count--;
			uart_tx_offset = 0u;
		}
	}
#endif
}

/* 只执行统一状态机本帧提交的基线策略，不维护隐藏恢复状态。 */
static void update_baseline(uint32_t index, uint16_t raw, int32_t raw_delta, bool suppress_activity) {
	/* 自动量程合成轨迹不应驱动真实基线。 */
	bool transition_blocked = (index >= CAPSENSE_AUTORANGE_CHANNEL_FIRST) &&
		(index < CAPSENSE_AUTORANGE_CHANNEL_FIRST + CAPSENSE_AUTORANGE_CHANNEL_COUNT) &&
		(autorange_transition_active || autorange_transition_finished);
	baseline_tracker_input_t input = {
		.raw = raw,
		.raw_delta = raw_delta,
		.channel_state = ChannelState_Get(&channel_state_machines[index]),
		/* 直接消费状态机的按压候选冻结请求，基线执行器不重复判断阈值。 */
		.hold_requested = ChannelState_ShouldHoldBaseline(&channel_state_machines[index]),
		.calibrated = calibrated,
		.suppress_activity = suppress_activity || transition_blocked,
	};
	BaselineTracker_Update(&input, &baseline_q16[index]);
}

static void begin_channel_release_recovery(uint32_t index) {
	post_active_baseline_delay_counts[index] = SENSOR_ALGO_POST_ACTIVE_BASELINE_DELAY_FRAMES;
	post_active_fast_baseline_counts[index] = 0u;
	release_negative_slope_counts[index] = 0u;
	negative_excursion_counts[index] = 0u;
	release_recovery_counts[index] = SENSOR_ALGO_RELEASE_RECOVERY_FRAMES;
	release_recovery_stable_counts[index] = 0u;
	post_release_gate_values[index] = (uint16_t)(active_peak_diff_values[index] / SENSOR_ALGO_RELEASE_DYNAMIC_GATE_DIVISOR);
	post_release_blank_counts[index] = SENSOR_ALGO_RELEASE_RECOVERY_MIN_FRAMES + (uint16_t)(active_peak_diff_values[index] / SENSOR_ALGO_RELEASE_DYNAMIC_GATE_DIVISOR);
	if (post_release_blank_counts[index] > SENSOR_ALGO_RELEASE_RECOVERY_FRAMES) {
		post_release_blank_counts[index] = SENSOR_ALGO_RELEASE_RECOVERY_FRAMES;
	}
	active_peak_diff_values[index] = 0u;
	channel_was_active[index] = false;
}

/* 单通道重校准只更新基线和采样计时，不再累加噪声统计。 */
static bool process_channel_recalibration_sample(uint32_t index, uint32_t raw, uint32_t raw_q16) {
	/* 保留采样接口，基线更新直接使用Q16输入。 */
	(void)raw;
	baseline_q16[index] = apply_iir_q16(baseline_q16[index], raw_q16, SENSOR_ALGO_CALIBRATION_ALPHA_Q16);
	channel_calibration_counts[index]++;

	if (channel_calibration_counts[index] < SENSOR_ALGO_CALIBRATION_FRAMES) {
		return false;
	}

	finish_channel_recalibration(index);
	return true;
}

static void update_force_vector_group_snapshot(void) {
	uint32_t index;

	force_vector_group_signal_sum = 0u;
	force_vector_group_noise_sum = 0u;

	/* Only the 4 tangential-force channels participate in this group output. */
	for (index = 0u; index < SENSOR_ALGO_CHANNEL_COUNT; index++) {
		if (is_force_vector_channel(index)) {
			force_vector_group_signal_sum += q16_to_u16(signal_q16[index]);
			/* 噪声统一按固定值计算，不受校准结果影响。 */
			force_vector_group_noise_sum += SENSOR_ALGO_FIXED_NOISE;
		}
	}
}

static void update_force_vector_group_contact_hint(const uint16_t raw_values[SENSOR_ALGO_CHANNEL_COUNT]) {
#if SENSOR_ALGO_FORCE_VECTOR_OUTPUT
#if SENSOR_ALGO_CHANNEL_COUNT == 5u
	const uint32_t ch_idx[4] = {0u, 3u, 1u, 4u};
#elif SENSOR_ALGO_CHANNEL_COUNT == 8u
	const uint32_t ch_idx[4] = {6u, 7u, 4u, 5u};
#else
	force_vector_group_contact_hint = false;
	return;
#endif
	const float K[4] = {FV_CHANNEL_K_00, FV_CHANNEL_K_01, FV_CHANNEL_K_10, FV_CHANNEL_K_11};
	float S_raw = 0.0f;
	float avg_noise = 0.0f;
	float touch_on;
	float touch_off;
	bool release_by_threshold;
	bool release_by_drop;
	uint32_t index;

	if (!calibrated) {
		force_vector_group_contact_hint = false;
		force_vector_group_touch_peak = 0.0f;
		return;
	}

	for (index = 0u; index < 4u; index++) {
		uint32_t channel_index = ch_idx[index];
		float positive_diff =
			(float)apply_input_noise_gate(channel_index, positive_diff_from_baseline(channel_index, raw_values[channel_index]), SENSOR_ALGO_FORCE_VECTOR_CONTACT_GATE_MULTIPLIER);

		S_raw += positive_diff * K[index];
		/* 噪声统一按固定值计算，不受校准结果影响。 */
		avg_noise += (float)SENSOR_ALGO_FIXED_NOISE * K[index];
	}

	avg_noise *= 0.25f;
	touch_on = avg_noise * 4.0f * DIR_TOUCH_ON_SCALE;
	touch_off = avg_noise * 4.0f * DIR_TOUCH_OFF_SCALE;

	if (!force_vector_group_contact_hint) {
		if (S_raw >= touch_on) {
			force_vector_group_contact_hint = true;
			force_vector_group_touch_peak = S_raw;
		}
		return;
	}

	if (S_raw > force_vector_group_touch_peak) {
		force_vector_group_touch_peak = S_raw;
	}

	release_by_threshold = (S_raw < touch_off);
	release_by_drop = (force_vector_group_touch_peak > touch_on) && (S_raw < (force_vector_group_touch_peak * BASE_RELEASE_RATIO));
	if (release_by_threshold || release_by_drop) {
		force_vector_group_contact_hint = false;
		force_vector_group_touch_peak = 0.0f;
	}
#else
	(void)raw_values;
	force_vector_group_contact_hint = false;
	force_vector_group_touch_peak = 0.0f;
#endif
}

/* 矩阵没有合力组，不对任何通道套用力组处理。 */
static bool is_force_vector_channel(uint32_t index) { return DEVICE_HAS_FORCE_OUTPUT && (index >= SENSOR_ALGO_FORCE_VECTOR_CHANNEL_FIRST) && (index < SENSOR_ALGO_FORCE_VECTOR_CHANNEL_LAST); }

static bool is_force_vector_contact_confirmed(void) {
#if SENSOR_ALGO_FORCE_VECTOR_OUTPUT
	return fingertip_force_vector.contact_active;
#else
	return false;
#endif
}

static bool is_force_vector_active(void) {
#if SENSOR_ALGO_FORCE_VECTOR_OUTPUT
	return force_vector_group_contact_hint || fingertip_force_vector.contact_active;
#else
	return false;
#endif
}

static void finish_global_calibration_frame(void) {
	uint32_t required_frames;

	if (calibrated) {
		return;
	}

	required_frames = (startup_quiet_frame_count < SENSOR_ALGO_STARTUP_QUIET_FRAMES) ? SENSOR_ALGO_STARTUP_QUIET_FRAMES : SENSOR_ALGO_CALIBRATION_FRAMES;
	calibration_count++;
	if (calibration_count >= required_frames) {
		finish_calibration();
	}
}

static uint16_t update_channel_signal(uint32_t index, uint32_t positive_diff, uint32_t raw_q16, int32_t raw_delta, bool suppress_activity, bool *fast_release_active) {
	/* 保存依次应用本轮零点补偿和活动期蠕变补偿后的有符号 Signal 目标。 */
	int64_t compensated_target;
	/* 保存经过范围钳制后用于滤波的 Q16 Signal 目标。 */
	uint32_t signal_target_q16;
	uint32_t signal_alpha_q16;
	uint16_t previous_signal = q16_to_u16(signal_q16[index]);
	uint16_t signal;

	*fast_release_active = false;
	(void)raw_q16;
	(void)raw_delta;

	/* 先扣除快速重复按压前记录的材料残余偏移，永久 Baseline 保持不变。 */
	compensated_target = (int64_t)positive_diff - (int64_t)cycle_zero_offset_values[index];
	/* 已经建立蠕变补偿时，再扣除稳定按压期间累计吸收的蠕变量。 */
	if (floating_compensation_valid[index]) {
		/* 使用有符号运算兼容退出 FLOATING 时可能保留的负向补偿。 */
		compensated_target -= floating_compensation_values[index];
	/* 结束活动期蠕变补偿处理。 */
	}
	/* 补偿后的目标低于零时钳制为零。 */
	if (compensated_target < 0) {
		/* 防止负值转换为无符号数后产生异常大 Signal。 */
		compensated_target = 0;
	/* 补偿后的目标超过 16 位 Signal 范围时执行上限钳制。 */
	} else if (compensated_target > UINT16_MAX) {
		/* 将目标限制在算法输出允许的最大值。 */
		compensated_target = UINT16_MAX;
	/* 结束补偿目标范围钳制。 */
	}
	/* 将补偿后的整数目标转换为 Q16 格式。 */
	signal_target_q16 = (uint32_t)compensated_target << 16u;

	if (!calibrated) {
		signal_q16[index] = 0u;
		release_counts[index] = 0u;
		fast_release_counts[index] = 0u;
		fast_release_latched[index] = false;
		return 0u;
	}

	if (suppress_activity) {
		signal_target_q16 = 0u;
		fast_release_counts[index] = 0u;
		fast_release_latched[index] = false;
	}

	signal_alpha_q16 = (signal_target_q16 < signal_q16[index]) ? SENSOR_ALGO_SIGNAL_FALL_ALPHA_Q16 : SENSOR_ALGO_SIGNAL_RISE_ALPHA_Q16;

	signal_q16[index] = apply_iir_q16(signal_q16[index], signal_target_q16, signal_alpha_q16);
	signal = q16_to_u16(signal_q16[index]);

	apply_release_snap(index, previous_signal, &signal);

	return signal;
}

static void update_fast_release(uint32_t index, uint32_t positive_diff, uint32_t raw_q16, uint32_t *signal_target_q16, bool *fast_release_active) {
	uint16_t positive_diff_u16 = (positive_diff > 0xFFFFu) ? 0xFFFFu : (uint16_t)positive_diff;
	int32_t positive_diff_delta = (int32_t)positive_diff_u16 - (int32_t)previous_positive_diff_values[index];
	/* 噪声统一按固定值计算，不受校准结果影响。 */
	uint32_t active_threshold = (uint32_t)SENSOR_ALGO_FIXED_NOISE * SENSOR_ALGO_NOISE_MULTIPLIER;
	/* 噪声统一按固定值计算，不受校准结果影响。 */
	uint32_t release_threshold = (uint32_t)SENSOR_ALGO_FIXED_NOISE * SENSOR_ALGO_RELEASE_MULTIPLIER;
	/* 噪声统一按固定值计算，不受校准结果影响。 */
	uint32_t fast_release_start = (uint32_t)SENSOR_ALGO_FIXED_NOISE * SENSOR_ALGO_FAST_RELEASE_START_MULTIPLIER;
	bool matrix_group_held = is_force_vector_channel(index) && is_force_vector_contact_confirmed();
	/* 噪声统一按固定值计算，不受校准结果影响。 */
	uint32_t slope_threshold = SENSOR_ALGO_FIXED_NOISE;

#if !SENSOR_ALGO_FAST_RELEASE_SNAP_BASELINE
	(void)raw_q16;
#endif

	if (slope_threshold < SENSOR_ALGO_MIN_SLOPE_THRESHOLD) {
		slope_threshold = SENSOR_ALGO_MIN_SLOPE_THRESHOLD;
	}

	if (matrix_group_held) {
		fast_release_counts[index] = 0u;
		fast_release_latched[index] = false;
	} else if (positive_diff <= release_threshold) {
		fast_release_counts[index] = 0u;
		fast_release_latched[index] = false;
	} else if (positive_diff_delta >= (int32_t)active_threshold) {
		fast_release_counts[index] = 0u;
		fast_release_latched[index] = false;
	} else if ((positive_diff > active_threshold) && (positive_diff <= fast_release_start) && (positive_diff_delta <= -(int32_t)slope_threshold)) {
		if (fast_release_counts[index] < SENSOR_ALGO_FAST_RELEASE_DEBOUNCE_FRAMES) {
			fast_release_counts[index]++;
		}
		if (fast_release_counts[index] >= SENSOR_ALGO_FAST_RELEASE_DEBOUNCE_FRAMES) {
#if SENSOR_ALGO_FAST_RELEASE_SNAP_BASELINE
			if (!fast_release_latched[index]) {
				baseline_q16[index] = raw_q16;
			}
#endif
			if (!fast_release_latched[index]) {
				/* 首次锁存快速释放时立即打开动态门限和硬消隐窗口。 */
				begin_channel_release_recovery(index);
			}
			fast_release_latched[index] = true;
		}
	} else if (!fast_release_latched[index] && (positive_diff_delta >= 0)) {
		fast_release_counts[index] = 0u;
	}

	previous_positive_diff_values[index] = positive_diff_u16;
	*fast_release_active = fast_release_latched[index];

	if (*fast_release_active) {
		/* Baseline 保持不动；Signal 由调用者在当前帧直接清零。 */
		*signal_target_q16 = 0u;
	}
}

static bool detect_release_retouch(uint32_t index, uint32_t positive_diff, int32_t raw_delta) {
	/* 噪声统一按固定值计算，不受校准结果影响。 */
	uint32_t active_threshold = (uint32_t)SENSOR_ALGO_FIXED_NOISE * SENSOR_ALGO_NOISE_MULTIPLIER;
	/* 噪声统一按固定值计算，不受校准结果影响。 */
	uint32_t slope_threshold = SENSOR_ALGO_FIXED_NOISE;

#if SENSOR_ALGO_FORCE_VECTOR_OUTPUT
	if (is_force_group_recovering_channel(index)) {
		return false;
	}
#endif

	if (slope_threshold < SENSOR_ALGO_MIN_SLOPE_THRESHOLD) {
		slope_threshold = SENSOR_ALGO_MIN_SLOPE_THRESHOLD;
	}

	return (positive_diff > active_threshold) && (raw_delta > (int32_t)slope_threshold);
}

static void apply_release_snap(uint32_t index, uint16_t previous_signal, uint16_t *signal) {
	/* 噪声统一按固定值计算，不受校准结果影响。 */
	uint32_t release_threshold = (uint32_t)SENSOR_ALGO_FIXED_NOISE * SENSOR_ALGO_RELEASE_MULTIPLIER;
	bool group_channel = is_force_vector_channel(index);
	bool group_contact_active = is_force_vector_active();
	uint32_t group_signal_base = 0u;

	if (group_channel) {
		group_signal_base = (force_vector_group_signal_sum > previous_signal) ? (force_vector_group_signal_sum - previous_signal) : 0u;
	}

	if (!calibrated || (*signal > release_threshold)) {
		release_counts[index] = 0u;
		if (group_channel) {
			force_vector_group_signal_sum = group_signal_base + *signal;
		}
		return;
	}

	if (group_channel && group_contact_active && ((group_signal_base + *signal) > (force_vector_group_noise_sum * SENSOR_ALGO_RELEASE_MULTIPLIER))) {
		/* Keep weak channels alive while the matrix group still clearly has contact. */
		release_counts[index] = 0u;
		force_vector_group_signal_sum = group_signal_base + *signal;
		return;
	}

	if (release_counts[index] < SENSOR_ALGO_RELEASE_DEBOUNCE_FRAMES) {
		release_counts[index]++;
	}
	if (release_counts[index] >= SENSOR_ALGO_RELEASE_DEBOUNCE_FRAMES) {
		signal_q16[index] = 0u;
		*signal = 0u;
	}

	if (group_channel) {
		force_vector_group_signal_sum = group_signal_base + *signal;
	}
}

/* 根据当前 Raw 更新指定通道状态，并执行恢复清零或稳定按压锁定。 */
static void update_output_state(uint32_t index, uint16_t raw, uint32_t positive_diff, bool suppress_activity, uint16_t *signal) {
	/* 保存独立通道状态机提交后的当前状态。 */
	sensor_algorithm_state_t state;
	/* 保存状态机更新前的状态，用于识别 ACTIVE_FLOATING 的进入和退出。 */
	sensor_algorithm_state_t previous_state = ChannelState_Get(&channel_state_machines[index]);
	/* 保存扣除本轮临时零点后的有效正向差值。 */
	uint32_t cycle_compensated_positive_diff;
	/* 标记由真实恢复状态正式进入新一轮按压的当前帧。 */
	bool resumed_from_recovery;

	/* 组织独立通道状态机处理本帧所需的输入数据。 */
	channel_state_input_t input = {
		/* 传入当前 Raw 采样值。 */
		.raw = raw,
		/* 传入当前 Baseline 的整数值。 */
		.baseline = q16_to_u16(baseline_q16[index]),
		/* 噪声统一按固定值计算，不受校准结果影响。 */
		.noise = SENSOR_ALGO_FIXED_NOISE,
		/* 传入全局校准完成状态。 */
		.calibrated = calibrated,
		/* 传入本帧活动抑制状态。 */
		.suppress_activity = suppress_activity,
		/* 只有CH4至CH7在Ref切档期间保持原有状态。 */
		.autorange_transition_active =
			(index >= CAPSENSE_AUTORANGE_CHANNEL_FIRST) &&
			(index < (CAPSENSE_AUTORANGE_CHANNEL_FIRST + CAPSENSE_AUTORANGE_CHANNEL_COUNT)) &&
			autorange_transition_active,
		/* 只有CH4至CH7在切档结束首帧重建斜率窗口。 */
		.autorange_transition_finished =
			(index >= CAPSENSE_AUTORANGE_CHANNEL_FIRST) &&
			(index < (CAPSENSE_AUTORANGE_CHANNEL_FIRST + CAPSENSE_AUTORANGE_CHANNEL_COUNT)) &&
			autorange_transition_finished,
	/* 结束通道状态机输入数据初始化。 */
	};

	/* 通道状态只允许由独立状态机模块提交。 */
	state = ChannelState_Update(&channel_state_machines[index], &input);
	/* 非矩阵变体保留原来的两种恢复入口。 */
	resumed_from_recovery = ((previous_state == SENSOR_ALGO_STATE_SLOW_BACK) ||
		(previous_state == SENSOR_ALGO_STATE_ZERO_REBASE)) && (state == SENSOR_ALGO_STATE_ACTIVE_RISING);
#if DEVICE_MATRIX20
	/* 矩阵的等待及快速恢复再次触发时，也必须使用同一个已提交零点。 */
	resumed_from_recovery = resumed_from_recovery ||
		(((previous_state == SENSOR_ALGO_STATE_WAIT_STABLE) || (previous_state == SENSOR_ALGO_STATE_RECOVER)) &&
		 (state == SENSOR_ALGO_STATE_ACTIVE_RISING));
#endif

	/* 恢复状态重新触发时，使用再次上升前的局部 Raw 低点建立本轮临时零点。 */
	if (resumed_from_recovery) {
		/* 保存当前永久 Baseline 的整数值。 */
		uint16_t baseline = q16_to_u16(baseline_q16[index]);
#if DEVICE_MATRIX20
		/* Signal只消费状态机的本轮零点，残余偏移不能只从输出扣除而留在释放判据中。 */
		uint16_t reference = ChannelState_GetActivationBaseline(&channel_state_machines[index]);
		cycle_zero_offset_values[index] = (reference > baseline) ? reference - baseline : 0u;
#else
		/* 局部低点高于永久 Baseline 的部分就是本轮开始前尚未恢复的材料残余。 */
		cycle_zero_offset_values[index] =
			(channel_state_machines[index].slow_back_reference_raw > baseline)
				/* 保存正向残余偏移。 */
				? (uint16_t)(channel_state_machines[index].slow_back_reference_raw - baseline)
				/* 局部低点不高于 Baseline 时不需要零点补偿。 */
				: 0u;
#endif
	/* 结束恢复状态重新触发的本轮零点建立。 */
	}

	/* 从原始正向差值中扣除本轮临时零点，并对负结果执行零钳制。 */
	cycle_compensated_positive_diff =
		(positive_diff > cycle_zero_offset_values[index])
			/* 保留本轮按压相对局部零点新增的压力。 */
			? (positive_diff - cycle_zero_offset_values[index])
			/* 尚未超过临时零点时保持为零。 */
			: 0u;

	/* 恢复状态正式切换到 RISING 的首帧，按新零点重建滤波输出。 */
	if (resumed_from_recovery) {
		/* 前一恢复状态已将内部 Signal 清零，因此从零按正常上升系数重新滤波。 */
		signal_q16[index] = apply_iir_q16(0u, cycle_compensated_positive_diff << 16u, SENSOR_ALGO_SIGNAL_RISE_ALPHA_Q16);
		/* 将本帧对外输出同步为按临时零点修正后的滤波结果。 */
		*signal = q16_to_u16(signal_q16[index]);
	/* 结束重新触发首帧 Signal 重建。 */
	}

	/* SLOW_BACK 和 ZERO_REBASE 期间都不允许残余 Signal 对外输出。 */
	if ((state == SENSOR_ALGO_STATE_SLOW_BACK) || (state == SENSOR_ALGO_STATE_ZERO_REBASE)) {
		/* 恢复阶段不再保留上一轮稳定按压建立的蠕变补偿。 */
		floating_compensation_valid[index] = false;
		/* 清零补偿值，避免调试时看到已经失效的历史数据。 */
		floating_compensation_values[index] = 0;
		/* 清零锁定值，等待下一轮稳定按压重新建立。 */
		floating_locked_signal_q16[index] = 0u;
		/* 清零内部 Q16 Signal，避免离开 SLOW_BACK 时带出历史残值。 */
		signal_q16[index] = 0u;
		/* 将本帧返回给采样处理流程的 Signal 清零。 */
		*signal = 0u;
	/* 当前状态进入或保持 ACTIVE_FLOATING 时锁定输出并更新蠕变补偿。 */
	} else if (state == SENSOR_ALGO_STATE_ACTIVE_FLOATING) {
		/* 刚进入锁定状态时保存经过滤波的当前 Signal。 */
		if (previous_state != SENSOR_ALGO_STATE_ACTIVE_FLOATING) {
			/* 使用本帧内部 Q16 Signal 作为连续的输出锁定值。 */
			floating_locked_signal_q16[index] = signal_q16[index];
			/* 标记本通道已经建立有效的活动期蠕变补偿。 */
			floating_compensation_valid[index] = true;
		/* 结束 ACTIVE_FLOATING 首帧初始化。 */
		}
		/* 用当前有效正向差值减去锁定输出，得到本帧累计蠕变补偿。 */
		floating_compensation_values[index] = (int32_t)cycle_compensated_positive_diff - (int32_t)q16_to_u16(floating_locked_signal_q16[index]);
		/* 强制内部 Signal 保持为进入状态时保存的锁定值。 */
		signal_q16[index] = floating_locked_signal_q16[index];
		/* 将本帧对外 Signal 同步覆盖为锁定值。 */
		*signal = q16_to_u16(floating_locked_signal_q16[index]);
	/* 返回空闲、等待或恢复阶段时结束本轮活动期蠕变补偿。 */
	} else if (ChannelState_IsIdleLike(state)) {
		/* 清除有效标记，使下一轮按压使用未经补偿的正常 Signal。 */
		floating_compensation_valid[index] = false;
		/* 清零活动期补偿值。 */
		floating_compensation_values[index] = 0;
		/* 清零上一轮保存的锁定 Signal。 */
		floating_locked_signal_q16[index] = 0u;
		/* 完全回到空闲后清除快速重复按压使用的本轮临时零点。 */
		cycle_zero_offset_values[index] = 0u;
		/* 清零内部滤波 Signal，防止 SLOW_BACK 退出时残余 Raw-Baseline 差值重新泄漏。 */
		signal_q16[index] = 0u;
		/* 空闲状态始终对外输出零，只有重新进入活动状态后才恢复 Signal。 */
		*signal = 0u;
	/* 结束恢复清零、稳定锁定和空闲复位状态处理。 */
	}
#if 0
	/* 默认将本帧的候选状态设为空闲状态。 */
	sensor_algorithm_state_t candidate_state = SENSOR_ALGO_STATE_IDLE;
	(void)positive_diff;

	/* 计算当前信号相对上一帧信号的变化量，用于判断信号的变化方向。 */
	int32_t signal_delta = (int32_t)signal - (int32_t)previous_signal_values[index];

	/* 保存当前信号值，供下一帧计算信号变化量。 */
	previous_signal_values[index] = signal;

	/* 快速释放触发后短暂保持独立状态；这里只延长状态显示，不改变释放算法。 */
	if (!calibrated || suppress_activity) {
		fast_release_state_counts[index] = 0u;
	} else if (fast_release_active) {
		fast_release_state_counts[index] = 12u;
	}

	/* SLOW_BACK 不参与普通状态防抖，确保松手触发当帧即可通过通信上传。 */
	if (calibrated && !suppress_activity && (fast_release_state_counts[index] > 0u)) {
		fast_release_state_counts[index]--;
		channel_states[index] = SENSOR_ALGO_STATE_SLOW_BACK;
		pending_channel_states[index] = SENSOR_ALGO_STATE_SLOW_BACK;
		pending_state_counts[index] = 0u;
		return;
	}

	/* 仅在全局校准完成且当前通道未被抑制时进行活动状态判断。 */
	if (calibrated && !suppress_activity) {

		/* 根据当前通道的噪声值计算进入活动状态的门限。 */
		/* 根据当前通道的噪声值计算弱活动状态的上限。 */
		// uint32_t weak_active_upper = (uint32_t)SENSOR_ALGO_FIXED_NOISE * SENSOR_ALGO_AUTO_CAL_MULTIPLIER;

		/* 根据当前信号、信号变化方向及门限计算本帧的候选状态。 */
		// 简单粗暴的去掉了weak_active状态的判断
		candidate_state = classify_channel_state(signal, signal_delta, SENSOR_ACTIVE_THREADHOLD, 0, 0);

	}

	/* 对候选状态执行防抖处理，并在满足条件时更新通道的最终状态。 */
	update_channel_state(index, candidate_state);
#endif
}

static uint32_t positive_diff_from_baseline(uint32_t index, uint32_t raw) {
	uint16_t baseline = q16_to_u16(baseline_q16[index]);

	return (raw > baseline) ? (raw - baseline) : 0u;
}

static bool detect_negative_rebound_release(uint32_t index, uint32_t raw, bool suppress_activity) {
#if 0
	/* 噪声统一按固定值计算，不受校准结果影响。 */
	uint32_t active_threshold = (uint32_t)SENSOR_ALGO_FIXED_NOISE * SENSOR_ALGO_NOISE_MULTIPLIER;
	/* 噪声统一按固定值计算，不受校准结果影响。 */
	uint32_t rebound_negative_threshold = (uint32_t)SENSOR_ALGO_FIXED_NOISE * SENSOR_ALGO_REBOUND_NEGATIVE_MULTIPLIER;
	uint16_t baseline = q16_to_u16(baseline_q16[index]);
	uint32_t raw_negative_diff = (baseline > raw) ? ((uint32_t)baseline - raw) : 0u;
	uint16_t signal = q16_to_u16(signal_q16[index]);

	if (!calibrated || suppress_activity || (signal > active_threshold)) {
		negative_excursion_counts[index] = 0u;
		return false;
	}

	if (raw_negative_diff >= rebound_negative_threshold) {
		if (negative_excursion_counts[index] < SENSOR_ALGO_REBOUND_NEGATIVE_FRAMES) {
			negative_excursion_counts[index]++;
		}
		return false;
	}

	if ((negative_excursion_counts[index] >= SENSOR_ALGO_REBOUND_NEGATIVE_FRAMES) && (raw >= baseline)) {
		begin_channel_release_recovery(index);
		return true;
	}

	return false;
#else
	(void)index;
	(void)raw;
	(void)suppress_activity;
	return false;
#endif
}

static void update_post_release_dynamic_gate(uint32_t index, uint32_t positive_diff, int32_t raw_delta, bool suppress_activity) {
#if 0
	/* 噪声统一按固定值计算，不受校准结果影响。 */
	uint32_t active_threshold = (uint32_t)SENSOR_ALGO_FIXED_NOISE * SENSOR_ALGO_NOISE_MULTIPLIER;
	/* 噪声统一按固定值计算，不受校准结果影响。 */
	uint32_t slope_threshold = SENSOR_ALGO_FIXED_NOISE;
	uint32_t decay;

	if (slope_threshold < SENSOR_ALGO_MIN_SLOPE_THRESHOLD) {
		slope_threshold = SENSOR_ALGO_MIN_SLOPE_THRESHOLD;
	}

	if (!calibrated || suppress_activity) {
		active_peak_diff_values[index] = 0u;
		post_release_gate_values[index] = 0u;
		post_release_blank_counts[index] = 0u;
		return;
	}

	/* 硬消隐期间任何回弹斜率都不能提前撤销回弹抑制。 */
	if (post_release_blank_counts[index] > 0u) {
		post_release_blank_counts[index]--;
		return;
	}

	/* 释放恢复期间保留动态门限，防止回弹被误判为新的按压。 */
	if (release_recovery_counts[index] > 0u) {
		return;
	}

	if ((positive_diff > active_threshold) && (positive_diff > active_peak_diff_values[index])) {
		active_peak_diff_values[index] = (positive_diff > 0xFFFFu) ? 0xFFFFu : (uint16_t)positive_diff;
	}

	if ((positive_diff > active_threshold) && (raw_delta > (int32_t)slope_threshold)) {
		post_release_gate_values[index] = 0u;
		post_release_blank_counts[index] = 0u;
		return;
	}

	if (post_release_gate_values[index] == 0u) {
		return;
	}

	decay = ((uint32_t)post_release_gate_values[index] >> SENSOR_ALGO_RELEASE_DYNAMIC_GATE_DECAY_SHIFT) + 1u;
	post_release_gate_values[index] = (post_release_gate_values[index] > decay) ? (uint16_t)(post_release_gate_values[index] - decay) : 0u;
#else
	(void)index;
	(void)positive_diff;
	(void)raw_delta;
	(void)suppress_activity;
#endif
}

static uint32_t apply_input_noise_gate(uint32_t index, uint32_t positive_diff, uint32_t multiplier) {
	/* 所有通道共用固定噪声门限，保留通道参数以兼容采样接口。 */
	(void)index;
	/* 噪声统一按固定值计算，不受校准结果影响。 */
	uint32_t threshold = (uint32_t)SENSOR_ALGO_FIXED_NOISE * multiplier;

	return (positive_diff > threshold) ? (positive_diff - threshold) : 0u;
}

static void reset_channel_runtime_state(uint32_t index, bool reset_signal) {
	if (reset_signal) {
		signal_q16[index] = 0u;
	}

	/* 复位通道时清除稳定按压阶段保存的 Signal 锁定值。 */
	floating_locked_signal_q16[index] = 0u;
	/* 复位通道时清除活动期蠕变补偿。 */
	floating_compensation_values[index] = 0;
	/* 复位通道时将蠕变补偿标记为无效。 */
	floating_compensation_valid[index] = false;

	idle_counts[index] = 0u;
	active_recal_counts[index] = 0u;
	post_active_baseline_delay_counts[index] = 0u;
	post_active_fast_baseline_counts[index] = 0u;
	release_negative_slope_counts[index] = 0u;
	negative_excursion_counts[index] = 0u;
	release_recovery_counts[index] = 0u;
	release_recovery_stable_counts[index] = 0u;
	channel_was_active[index] = false;
	channel_recalibrating[index] = false;
	channel_calibration_counts[index] = 0u;
	ChannelState_Reset(&channel_state_machines[index], previous_raw_values[index], q16_to_u16(baseline_q16[index]));
	previous_positive_diff_values[index] = 0u;
	previous_raw_values[index] = 0u;
	active_peak_diff_values[index] = 0u;
	post_release_gate_values[index] = 0u;
	post_release_blank_counts[index] = 0u;
	release_counts[index] = 0u;
	fast_release_counts[index] = 0u;
	fast_release_latched[index] = false;
	fast_release_state_counts[index] = 0u;
	reset_channel_drift(index);
}

/* 重新进入全局校准，仅重建baseline；噪声始终采用统一固定值。 */
static void begin_calibration(void) {
	uint32_t index;

	for (index = 0u; index < SENSOR_ALGO_CHANNEL_COUNT; index++) {

		reset_channel_runtime_state(index, true);
	}

	calibrated = false;
	calibration_count = 0u;
	force_vector_group_contact_hint = false;
	force_vector_group_touch_peak = 0.0f;
#if SENSOR_ALGO_FORCE_VECTOR_OUTPUT
	force_vector_context_reset(&fingertip_force_vector);
	force_vector_context_reset(&finger_pad_force_vector);
	reset_force_group_recovery(&fingertip_group_recovery);
	reset_force_group_recovery(&finger_pad_group_recovery);
#endif

}

/* 校准完成时对齐基线，取消实测噪声估计和由噪声触发的校准重试。 */
static void finish_calibration(void) {
	uint32_t index;

	for (index = 0u; index < SENSOR_ALGO_CHANNEL_COUNT; index++) {
		/* 消除校准期 IIR 对仍在缓慢变化的 raw 所产生的残差。 */
		baseline_q16[index] = (uint32_t)previous_raw_values[index] << 16u;
	}

	calibrated = true;
	calibration_count = 0u;
}

/* 弱 active 超时只重校准当前通道，不影响其它通道的输出。 */
static void begin_channel_recalibration(uint32_t index) {
	channel_calibration_counts[index] = 0u;
	channel_recalibrating[index] = true;

	idle_counts[index] = 0u;
	active_recal_counts[index] = 0u;
	post_active_baseline_delay_counts[index] = 0u;
	post_active_fast_baseline_counts[index] = 0u;
	release_negative_slope_counts[index] = 0u;
	negative_excursion_counts[index] = 0u;
	release_recovery_counts[index] = 0u;
	release_recovery_stable_counts[index] = 0u;
	channel_was_active[index] = false;
	signal_q16[index] = 0u;
	/* 开始通道重校准时清除活动期Signal锁定值。 */
	floating_locked_signal_q16[index] = 0u;
	/* 开始通道重校准时清除蠕变补偿值。 */
	floating_compensation_values[index] = 0;
	/* 开始通道重校准时使蠕变补偿失效。 */
	floating_compensation_valid[index] = false;
	previous_positive_diff_values[index] = 0u;
	previous_raw_values[index] = 0u;
	active_peak_diff_values[index] = 0u;
	post_release_gate_values[index] = 0u;
	post_release_blank_counts[index] = 0u;
	release_counts[index] = 0u;
	fast_release_counts[index] = 0u;
	fast_release_latched[index] = false;
}

/* 单通道重校准结束后恢复跟踪，固定噪声不会因重新校准而改变。 */
static void finish_channel_recalibration(uint32_t index) {

	channel_calibration_counts[index] = 0u;
	channel_recalibrating[index] = false;
	idle_counts[index] = 0u;
	active_recal_counts[index] = 0u;
	post_active_baseline_delay_counts[index] = 0u;
	post_active_fast_baseline_counts[index] = 0u;
	release_negative_slope_counts[index] = 0u;
	negative_excursion_counts[index] = 0u;
	release_recovery_counts[index] = 0u;
	release_recovery_stable_counts[index] = 0u;
	channel_was_active[index] = false;
	active_peak_diff_values[index] = 0u;
	post_release_gate_values[index] = 0u;
	post_release_blank_counts[index] = 0u;
	ChannelState_Reset(&channel_state_machines[index], previous_raw_values[index], q16_to_u16(baseline_q16[index]));
}

/* uint64 整数平方根，避免引入浮点和 sqrt。 */
static uint32_t isqrt_u64(uint64_t value) {
	uint64_t bit = 1ull << 62u;
	uint64_t result = 0u;

	while (bit > value) {
		bit >>= 2u;
	}

	while (bit != 0u) {
		if (value >= result + bit) {
			value -= result + bit;
			result = (result >> 1u) + bit;
		} else {
			result >>= 1u;
		}
		bit >>= 2u;
	}

	return (uint32_t)result;
}

/* Q16 一阶 IIR：current = current + alpha * (target - current)。 */
static uint32_t apply_iir_q16(uint32_t current_q16, uint32_t target_q16, uint32_t alpha_q16) {
	int64_t delta = (int64_t)target_q16 - (int64_t)current_q16;

	return (uint32_t)((int64_t)current_q16 + ((delta * (int64_t)alpha_q16) / (int64_t)SENSOR_ALGO_Q16_ONE));
}

// 实际滤波
static uint32_t follow_baseline_upward_q16(uint32_t current_q16, uint32_t raw_q16, uint32_t alpha_q16) {
	if (raw_q16 <= current_q16) {
		return current_q16;
	}

	return apply_iir_q16(current_q16, raw_q16, alpha_q16);
}

/* Q16 转 uint16_t，带四舍五入和饱和保护。 */
static uint16_t q16_to_u16(uint32_t value_q16) {
	uint32_t value = (value_q16 + (SENSOR_ALGO_Q16_ONE / 2u)) >> 16u;

	return (value > 0xFFFFu) ? 0xFFFFu : (uint16_t)value;
}

#if 0
/* 把 signal 和变化方向映射成单通道状态。 */
#if 0
static sensor_algorithm_state_t classify_channel_state(uint16_t signal, int32_t signal_delta, uint32_t active_threshold, uint32_t weak_active_upper, uint16_t noise) {
	/* 噪声统一按固定值计算，不受校准结果影响。 */
	int32_t slope_threshold = (int32_t)SENSOR_ALGO_FIXED_NOISE;

	if (slope_threshold < (int32_t)SENSOR_ALGO_MIN_SLOPE_THRESHOLD) {
		slope_threshold = (int32_t)SENSOR_ALGO_MIN_SLOPE_THRESHOLD;
	}

	if (signal <= active_threshold) {
		return SENSOR_ALGO_STATE_IDLE;
	}

	if ((signal < weak_active_upper) && (weak_active_upper > active_threshold)) {
		return SENSOR_ALGO_STATE_WEAK_ACTIVE;
	}

	if (signal_delta >= slope_threshold) {
		return SENSOR_ALGO_STATE_RISING_ACTIVE;
	}

	if (signal_delta <= -slope_threshold) {
		return SENSOR_ALGO_STATE_FALLING_ACTIVE;
	}

	return SENSOR_ALGO_STATE_ACTIVE;
}
#endif

/* 把 signal 和变化方向映射成单通道状态。 */
static sensor_algorithm_state_t classify_channel_state(uint16_t signal, int32_t signal_delta, uint32_t active_threshold, uint32_t weak_active_upper, uint16_t noise) {
	// int32_t slope_threshold = (int32_t)SENSOR_ALGO_FIXED_NOISE;

	// if (slope_threshold < (int32_t)SENSOR_ALGO_MIN_SLOPE_THRESHOLD) {
	// 	slope_threshold = (int32_t)SENSOR_ALGO_MIN_SLOPE_THRESHOLD;
	// }

	if (signal <= active_threshold) {
		return SENSOR_ALGO_STATE_IDLE;
	}

	if (signal_delta >= SENSOR_ALGO_MIN_SLOPE_THRESHOLD) {
		return SENSOR_ALGO_STATE_RISING_ACTIVE;
	}

	if (signal_delta <= -SENSOR_ALGO_MIN_SLOPE_THRESHOLD) {
		return SENSOR_ALGO_STATE_FALLING_ACTIVE;
	}

	return SENSOR_ALGO_STATE_ACTIVE;
}

/* 对状态做短防抖，避免边界附近一两帧抖动直接上传。 */
static void update_channel_state(uint32_t index, sensor_algorithm_state_t candidate_state) {
	if (candidate_state == channel_states[index]) {
		pending_channel_states[index] = candidate_state;
		pending_state_counts[index] = 0u;
		return;
	}

	if (candidate_state != pending_channel_states[index]) {
		pending_channel_states[index] = candidate_state;
		pending_state_counts[index] = 1u;
		return;
	}

	if (pending_state_counts[index] < SENSOR_ALGO_STATE_DEBOUNCE_FRAMES) {
		pending_state_counts[index]++;
	}

	if (pending_state_counts[index] >= SENSOR_ALGO_STATE_DEBOUNCE_FRAMES) {
		channel_states[index] = candidate_state;
		pending_state_counts[index] = 0u;
	}
}
#endif

/* 清除指定通道的激活态漂移斜率跟踪状态。 */
static void reset_channel_drift(uint32_t index) {
/* 矩阵的快慢K由状态机计算，无需重置旧V19漂移历史。 */
#if DEVICE_MATRIX20
	(void)index;
#else
	channel_drift_tracker_t *tracker = &drift_trackers[index];

	tracker->sample_count = 0u;
	tracker->write_index = 0u;
	tracker->k_update_count = 0u;
	tracker->raw_sum = 0u;
	tracker->signal_sum = 0u;
	tracker->raw_weighted_sum = 0u;
	tracker->signal_weighted_sum = 0u;
	tracker->raw_k_q16 = 0;
	tracker->signal_k_q16 = 0;

#endif
}

/* 根据等间隔的 160 个样本计算线性回归斜率，返回 Q16 counts/s。 */
static int32_t calculate_drift_k_q16(uint64_t sum, uint64_t weighted_sum) {

	// 计算线性回归斜率 k = (N * Σ(xy) - Σx * Σy) / (N * Σ(x^2) - (Σx)^2)
	// x = 0,1,2,...,N-1
	// 所以sum_x = 0+1+2+...+(N-1) = (N-1)*N/2
	// 高斯发明的
	const int64_t count = (int64_t)SENSOR_ALGO_DRIFT_WINDOW_FRAMES;
	const int64_t sum_x = ((count - 1) * count) / 2;
	// Σ(x^2) = 0^2 + 1^2 + 2^2 + ... + (N-1)^2 = (N-1)*N*(2N-1)/6
	const int64_t sum_xx = ((count - 1) * count * ((2 * count) - 1)) / 6;

	// 计算分母和分子，避免浮点运算
	const int64_t denominator = (count * sum_xx) - (sum_x * sum_x);

	// 计算分子，使用 int64_t 避免溢出
	int64_t numerator = (count * (int64_t)weighted_sum) - (sum_x * (int64_t)sum);

	// 最近这个窗口内，signal 每秒平均增加/减少多少。
	return (int32_t)((numerator * (int64_t)SENSOR_ALGO_DRIFT_SAMPLE_RATE_HZ * (int64_t)SENSOR_ALGO_Q16_ONE) / denominator);
}

/* 激活连续满一秒后，更新指定通道 raw 与 signal 的滑动线性回归斜率。 */
static void update_channel_drift(uint32_t index, uint16_t raw, uint16_t signal, sensor_algorithm_output_t *output) {
/* 矩阵快慢K由状态机提供，旧V19未上报字段保持零。 */
#if DEVICE_MATRIX20
	(void)index; (void)raw; (void)signal;
	output->raw_k_q16 = 0;
	output->signal_k_q16 = 0;
#else
	channel_drift_tracker_t *tracker = &drift_trackers[index];
	uint32_t sample_position;

	if (ChannelState_IsIdleLike(ChannelState_Get(&channel_state_machines[index]))) {
		reset_channel_drift(index);
		output->raw_k_q16 = 0;
		output->signal_k_q16 = 0;
		return;
	}

	// 160次采集之内
	if (tracker->sample_count < SENSOR_ALGO_DRIFT_WINDOW_FRAMES) {

		// 记录当前采样位置，作为加权计算的 x 值
		sample_position = tracker->sample_count;

		// 记录当前采样的 raw 与 signal 值
		tracker->raw_history[tracker->write_index] = raw;
		tracker->signal_history[tracker->write_index] = signal;

		// 累加当前采样的 raw 与 signal 值
		tracker->raw_sum += raw;

		// 累加当前采样的 signal 值
		tracker->signal_sum += signal;

		// 累加当前采样位置与 raw 的乘积，用于计算加权和
		tracker->raw_weighted_sum += (uint64_t)sample_position * raw;
		tracker->signal_weighted_sum += (uint64_t)sample_position * signal;

		// 更新采样计数和写入索引，确保在 160 次采样后循环覆盖历史数据
		tracker->sample_count++;
		tracker->write_index++;
		if (tracker->write_index >= SENSOR_ALGO_DRIFT_WINDOW_FRAMES) {
			tracker->write_index = 0u;
		}
		// 超过160次采集之后
	} else {

		uint16_t oldest_raw = tracker->raw_history[tracker->write_index];
		uint16_t oldest_signal = tracker->signal_history[tracker->write_index];

		// 计算新的加权和，减去最旧的样本贡献，并加上新的样本贡献
		tracker->raw_weighted_sum = tracker->raw_weighted_sum - (tracker->raw_sum - oldest_raw) + ((uint64_t)(SENSOR_ALGO_DRIFT_WINDOW_FRAMES - 1u) * raw);
		tracker->signal_weighted_sum = tracker->signal_weighted_sum - (tracker->signal_sum - oldest_signal) + ((uint64_t)(SENSOR_ALGO_DRIFT_WINDOW_FRAMES - 1u) * signal);

		// 更新 raw_sum 和 signal_sum，减去最旧的样本值，并加上新的样本值
		tracker->raw_sum = tracker->raw_sum - oldest_raw + raw;
		tracker->signal_sum = tracker->signal_sum - oldest_signal + signal;

		// 将当前的 raw 和 signal 样本存储在历史数组中，以便在下一次更新时使用
		tracker->raw_history[tracker->write_index] = raw;
		tracker->signal_history[tracker->write_index] = signal;

		// 更新写入索引，确保在 160 次采样后循环覆盖历史数据
		tracker->write_index++;
		if (tracker->write_index >= SENSOR_ALGO_DRIFT_WINDOW_FRAMES) {
			tracker->write_index = 0u;
		}
	}

	if (tracker->sample_count >= SENSOR_ALGO_DRIFT_WINDOW_FRAMES) {
		if (tracker->k_update_count == 0u) {
			tracker->raw_k_q16 = calculate_drift_k_q16(tracker->raw_sum, tracker->raw_weighted_sum);
			tracker->signal_k_q16 = calculate_drift_k_q16(tracker->signal_sum, tracker->signal_weighted_sum);
		}
		tracker->k_update_count++;
		if (tracker->k_update_count >= SENSOR_ALGO_DRIFT_K_UPDATE_DIVIDER) {
			tracker->k_update_count = 0u;
		}
	}

	output->raw_k_q16 = tracker->raw_k_q16;
	output->signal_k_q16 = tracker->signal_k_q16;

#if 0
	if (7 == index) {
		int32_t k_q16 = output->signal_k_q16;

		int64_t abs_k = (k_q16 >= 0) ? k_q16 : -(int64_t)k_q16;

		int32_t integer_part = (int32_t)(abs_k / 65536);
		int32_t frac_part = (int32_t)(((abs_k % 65536) * 10000) / 65536);

		printf("signal_k: %s%ld.%04ld/s\r\n", k_q16 < 0 ? "-" : "", (long)integer_part, (long)frac_part);
		// printf("signal_k: %.4f/s\r\n", (double)output->signal_k_q16 / 65536.0);
		// printf("signal_k: %d\r\n", output->signal_k_q16);
	}
#endif

#endif
}

static void send_signal_frame(const sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT]) {
	uint32_t sensor_index;
	uint32_t offset = 0u;
	uint8_t frame[SENSOR_ALGO_SIGNAL_FRAME_SIZE];

	write_frame_header(frame, &offset, SENSOR_ALGO_FRAME_TYPE_SIGNAL, SENSOR_ALGO_CHANNEL_COUNT, SENSOR_ALGO_SIGNAL_PAYLOAD_SIZE);

	for (sensor_index = 0u; sensor_index < SENSOR_ALGO_CHANNEL_COUNT; sensor_index++) {
		write_u16(frame, &offset, outputs[sensor_index].signal);
	}

	send_output_frame(frame, SENSOR_ALGO_SIGNAL_FRAME_SIZE);
}

#if SENSOR_ALGO_CHANNEL_4567_SIGNAL_SUM_OUTPUT
static void send_channel_4567_signal_sum_frame(const sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT]) {
	uint32_t sensor_index;
	uint32_t signal_sum = 0u;
	uint32_t offset = 0u;
	uint8_t frame[SENSOR_ALGO_CHANNEL_4567_SIGNAL_SUM_FRAME_SIZE];

	for (sensor_index = 4u; sensor_index <= 7u; sensor_index++) {
		signal_sum += (uint32_t)outputs[sensor_index].signal;
	}

	write_frame_header(frame, &offset, SENSOR_ALGO_FRAME_TYPE_CHANNEL_4567_SIGNAL_SUM, 4u, SENSOR_ALGO_CHANNEL_4567_SIGNAL_SUM_PAYLOAD_SIZE);
	write_u32(frame, &offset, signal_sum);
	send_output_frame(frame, SENSOR_ALGO_CHANNEL_4567_SIGNAL_SUM_FRAME_SIZE);
}
#endif

/* 使用硬件Raw计算总体标准差，整数运算避免大基数下的浮点相消。 */
#if SENSOR_ALGO_CHANNEL_COUNT >= 8u
static uint32_t calculate_hardware_raw_stddev_x100(const uint16_t raw[4], uint32_t *raw_sum) {
	uint64_t square_sum = 0u;
	uint32_t sum = 0u;
	uint32_t index;
	uint64_t scaled_variance;
	uint32_t root;
	for (index = 0u; index < 4u; index++) {
		sum += raw[index];
		square_sum += (uint64_t)raw[index] * raw[index];
	}
	/* 标准差平方乘10000等于(4*平方和-和值平方)*625；全量程不溢出64位。 */
	scaled_variance = (4u * square_sum - (uint64_t)sum * sum) * 625u;
	root = isqrt_u64(scaled_variance);
	/* 四舍五入到0.01 count；平均值由原始和值除以4得到，保留小数。 */
	if (scaled_variance - (uint64_t)root * root > root) {
		root++;
	}
	*raw_sum = sum;
	return root;
}

/* Type 0x14载荷为小端u32信号和、硬件Raw和、硬件Raw总体标准差乘100。 */
static void send_channel_4567_statistics_frame(const sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT]) {
	uint32_t index;
	uint32_t signal_sum = 0u;
	uint32_t raw_sum;
	uint32_t offset = 0u;
	uint32_t stddev_x100;
	uint8_t frame[SENSOR_ALGO_PROTOCOL_HEADER_SIZE + 12u];
	for (index = 4u; index <= 7u; index++) {
		signal_sum += outputs[index].signal;
	}
	/* 始终读取未归一化、未减基线的同帧硬件值，状态切换不屏蔽此统计量。 */
	stddev_x100 = calculate_hardware_raw_stddev_x100(&latest_hardware_raw[4], &raw_sum);
	write_frame_header(frame, &offset, SENSOR_ALGO_FRAME_TYPE_CHANNEL_4567_STATISTICS, 4u, 12u);
	write_u32(frame, &offset, signal_sum);
	write_u32(frame, &offset, raw_sum);
	write_u32(frame, &offset, stddev_x100);
	send_output_frame(frame, sizeof(frame));
}
#endif

/* V20调试帧依次上传Raw、Baseline、Signal、Fast/Slow K和真实状态。 */
static void send_debug_frame(const sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT]) {
	uint32_t sensor_index;
	uint32_t offset = 0u;
	uint8_t frame[SENSOR_ALGO_DEBUG_FRAME_SIZE];
	sensor_algorithm_state_t states[SENSOR_ALGO_CHANNEL_COUNT];

	write_frame_header(frame, &offset, SENSOR_ALGO_FRAME_TYPE_DEBUG, SENSOR_ALGO_CHANNEL_COUNT, SENSOR_ALGO_DEBUG_PAYLOAD_SIZE);

	SensorAlgorithm_GetStates(states);

	/* 第一组上传未经量程折算的16位Hardware Raw。 */
	for (sensor_index = 0u; sensor_index < SENSOR_ALGO_CHANNEL_COUNT; sensor_index++) {
		write_u16(frame, &offset, latest_hardware_raw[sensor_index]);
	}

	for (sensor_index = 0u; sensor_index < SENSOR_ALGO_CHANNEL_COUNT; sensor_index++) {
		write_u16(frame, &offset, outputs[sensor_index].baseline);
	}

	for (sensor_index = 0u; sensor_index < SENSOR_ALGO_CHANNEL_COUNT; sensor_index++) {
		write_u16(frame, &offset, outputs[sensor_index].signal);
	}

	for (sensor_index = 0u; sensor_index < SENSOR_ALGO_CHANNEL_COUNT; sensor_index++) {
		write_u32(frame, &offset, (uint32_t)outputs[sensor_index].fast_normalized_k_x100);
	}

	for (sensor_index = 0u; sensor_index < SENSOR_ALGO_CHANNEL_COUNT; sensor_index++) {
		write_u32(frame, &offset, (uint32_t)outputs[sensor_index].slow_normalized_k_x100);
	}


	for (sensor_index = 0u; sensor_index < SENSOR_ALGO_CHANNEL_COUNT; sensor_index++) {
		write_u16(frame, &offset, (uint16_t)states[sensor_index]);
	}

	send_output_frame(frame, SENSOR_ALGO_DEBUG_FRAME_SIZE);
}

/* 返回主受力区四个物理通道在当前手指版本中的索引。 */
static uint32_t primary_force_channel_index(uint32_t patch_index) {
#if SENSOR_ALGO_CHANNEL_COUNT == 8u
	static const uint32_t channel_indices[4] = {6u, 7u, 4u, 5u};
#else
	static const uint32_t channel_indices[4] = {0u, 3u, 1u, 4u};
#endif

	return channel_indices[patch_index];
}

/* 将最近一次芯片温度转换结果排入现有非阻塞UART发送队列。 */
void SensorAlgorithm_SendTemperatureFrame(int16_t temperature_tenth_celsius) {
	uint32_t offset = 0u;
	uint8_t frame[SENSOR_ALGO_TEMPERATURE_FRAME_SIZE];

	write_frame_header(
		frame,
		&offset,
		SENSOR_ALGO_FRAME_TYPE_TEMPERATURE,
		0u,
		SENSOR_ALGO_TEMPERATURE_PAYLOAD_SIZE);
	write_u16(frame, &offset, (uint16_t)temperature_tenth_celsius);
	send_output_frame(frame, SENSOR_ALGO_TEMPERATURE_FRAME_SIZE);
}

/* 汇总四通道实际上传的整数Signal，低力表直接使用counts避免二次缩放误差。 */
static uint32_t primary_force_group_signal_sum(void) {
	uint32_t index;
	uint32_t signal_sum = 0u;

	for (index = 0u; index < 4u; index++) {
		signal_sum += q16_to_u16(signal_q16[primary_force_channel_index(index)]);
	}

	return signal_sum;
}

/* 原K表继续使用千计数输入域，保持既有表的数据单位。 */
#if SENSOR_ALGO_CHANNEL_COUNT < 8u
static float primary_force_group_raw_value(void) {
	return (float)primary_force_group_signal_sum() / SENSOR_ALGO_FORCE_REPORT_DIVISOR;
}
#endif

/* 生成每个受力区、每个分量各自的标定有效位。 */
static uint16_t force_component_calibration_mask(void) {
	uint16_t mask = 0u;
	uint8_t region;

	for (region = 0u; region < SENSOR_ALGO_FORCE_CALIBRATION_REGION_COUNT; region++) {
		uint16_t shift = (uint16_t)region * 3u;
		const sensor_force_calibration_t *calibration = &force_calibration[region];

		mask |= calibration->normal_valid ? (uint16_t)(1u << shift) : 0u;
		/* 只有切向力算法实际编译启用时，才允许声明切向力和方向标定有效。 */
#if SENSOR_ALGO_FORCE_VECTOR_OUTPUT
		mask |= calibration->tangential_valid ? (uint16_t)(1u << (shift + 1u)) : 0u;
		mask |= calibration->direction_valid ? (uint16_t)(1u << (shift + 2u)) : 0u;
#endif
	}

	return mask;
}

/* 请求编号逐字回传；参数是未缩放的四通道Signal和。 */
static void send_low_force_response(uint8_t operation, uint8_t result, uint32_t token) {
	/* 参数响应尾部追加最低检测线，单位为Signal和counts。 */
	uint8_t frame[38u];
	uint32_t offset = 0u;
	float points[LOW_FORCE_POINT_COUNT];
	LowForce_GetPoints(points);
	write_frame_header(frame, &offset, SENSOR_ALGO_FRAME_LOW_FORCE, 0u, 28u);
	frame[offset++] = operation;
	frame[offset++] = result;
	write_u16(frame, &offset, (LowForce_FlashValid() ? 1u : 0u) | (LowForce_IsValid() ? 2u : 0u));
	write_u32(frame, &offset, token);
	for (uint32_t i = 0u; i < LOW_FORCE_POINT_COUNT; ++i) {
		write_float(frame, &offset, points[i]);
	}
	write_float(frame, &offset, LowForce_GetDetectionCounts());
	send_command_response_frame(frame, sizeof(frame));
}

/* 持续上传MCU完成四通道当前合力计算和Flash K表换算后的运行力值。 */
static void send_run_force_frame(void) {
	uint32_t offset = 0u;
	uint8_t frame[SENSOR_ALGO_RUN_FORCE_FRAME_SIZE];
	float fingertip_normal = 0.0f;
	/* 对外双输出与实际数据分开，模拟时也保留未替换的测量结果。 */
	float fingertip_optimized = 0.0f;
	float fingertip_measured = 0.0f;
	float fingertip_measured_optimized = 0.0f;
	bool low_force_simulated = false;
	/* 高力表只在有效10/20节点换算达到1 N时接管输出，低力四点参数独立保留。 */
	bool high_force_active = false;
	float fingertip_tangential = 0.0f;
	float fingertip_direction = 0.0f;
	float finger_pad_normal = 0.0f;
	float finger_pad_tangential = 0.0f;
	float finger_pad_direction = 0.0f;
	/* 低力路径维持四点换算，V21新高力表可独立接管1 N及以上输出。 */
	uint8_t low_force_state = LOW_FORCE_INVALID;
	uint16_t component_mask = force_component_calibration_mask();

	/* 五通道设备缺少完整CH4至CH7，继续沿用其原有力值路径。 */
#if SENSOR_ALGO_CHANNEL_COUNT >= 8u
	if (calibrated) {
		float signal_sum = (float)primary_force_group_signal_sum();
		/* 仅显式的新高力表参与接管；旧48点表不会意外改变现有低力显示行为。 */
		const sensor_force_calibration_t *normal = &force_calibration[SENSOR_ALGO_FORCE_CALIBRATION_REGION_PRIMARY];
		if (normal->normal_valid && ((normal->normal_point_count == 10u) || (normal->normal_point_count == 20u)) &&
			(signal_sum / SENSOR_ALGO_FORCE_REPORT_DIVISOR >= normal->normal_raw_points[0u])) {
			/* 使用四路Signal和及原分段插值函数，不叠加旧K值或整数软吸附。 */
			float force = apply_piecewise_force_calibration(signal_sum / SENSOR_ALGO_FORCE_REPORT_DIVISOR,
				normal->normal_raw_points, normal->normal_force_points, normal->normal_point_count);
			if (force >= 1.0f) {
				fingertip_normal = force;
				fingertip_measured = force;
				fingertip_optimized = force;
				fingertip_measured_optimized = force;
				high_force_active = true;
			}
		}
		/* 未进入高力范围时完整保留原微力换算、检测线及模拟输出。 */
		if (!high_force_active) {
			/* 合力输出只依据标定与检测线，不受FLOATING进入或退出影响。 */
			low_force_state = LowForce_Evaluate(signal_sum, &fingertip_normal);
			(void)LowForce_Measure(signal_sum, &fingertip_measured);
			/* 仅原始测量有效时吸附；检测线及超量程不使用优化值判定。 */
			if (low_force_state == LOW_FORCE_VALID) {
				fingertip_optimized = LowForce_Optimize(fingertip_normal);
				fingertip_measured_optimized = fingertip_optimized;
				/* 模拟仅在实测低于10 g且超过counts阈值时生效，退出条件立即恢复正常。 */
				low_force_simulated = LowForce_Simulate(signal_sum, &fingertip_optimized);
				if (low_force_simulated) {
					fingertip_normal = 8.88f * 0.00980665f;
				}
			}
		}
	}
	/* bit9明确标记模拟双输出，bit0/6/7/8继续描述有效性及低力量程。 */
	component_mask = (component_mask & (uint16_t)~1u) | 0x40u |
		(low_force_simulated ? 0x200u : 0u) |
		(LowForce_IsValid() ? 0x100u : 0u) |
		(low_force_state == LOW_FORCE_VALID ? 1u : 0u) |
		(low_force_state == LOW_FORCE_OVER_RANGE ? 0x80u : 0u);
	/* 高力输出使用真实法向有效位并关闭低力量程标志，避免上位机继续显示超量程。 */
	if (high_force_active) {
		component_mask = (component_mask & (uint16_t)~(0x40u | 0x80u | 0x200u)) | 1u;
	}
#else
	(void)high_force_active;
	(void)low_force_state;
	(void)low_force_simulated;
	/* CapSense已校准且主受力区法向K表有效时，不再以FLOATING作为力值换算门槛。 */
	if (calibrated && force_calibration[SENSOR_ALGO_FORCE_CALIBRATION_REGION_PRIMARY].normal_valid) {
		/* 先保留原有Flash K表换算，再只对对外运行力施加连续单调的临时软吸附。 */
		fingertip_normal = apply_force_output_soft_anchor(apply_normal_force_calibration(
			primary_force_group_raw_value(),
			&force_calibration[SENSOR_ALGO_FORCE_CALIBRATION_REGION_PRIMARY]));
	}
#endif
#if SENSOR_ALGO_FORCE_VECTOR_OUTPUT
	if (calibrated && finger_pad_force_vector.contact_active) {
		finger_pad_normal = apply_normal_force_calibration(finger_pad_force_vector.debug.normal_force / SENSOR_ALGO_FORCE_REPORT_DIVISOR,
														   &force_calibration[SENSOR_ALGO_FORCE_CALIBRATION_REGION_SECONDARY]);
		finger_pad_tangential = apply_tangential_force_calibration(finger_pad_force_vector.debug.tangential_force / SENSOR_ALGO_FORCE_REPORT_DIVISOR,
																   &force_calibration[SENSOR_ALGO_FORCE_CALIBRATION_REGION_SECONDARY]);
		finger_pad_direction = (finger_pad_tangential > 0.0f)
								   ? apply_direction_calibration(finger_pad_force_vector.debug.theta, &force_calibration[SENSOR_ALGO_FORCE_CALIBRATION_REGION_SECONDARY])
								   : 0.0f;
	}
#endif

	write_frame_header(frame, &offset, SENSOR_ALGO_FRAME_TYPE_RUN_FORCE, SENSOR_ALGO_CHANNEL_COUNT, SENSOR_ALGO_RUN_FORCE_PAYLOAD_SIZE);
	frame[offset++] = 2u;
	/* 完整受力区标志只在法向、切向和方向三种输出均已启用且标定有效时置位。 */
#if SENSOR_ALGO_FORCE_VECTOR_OUTPUT
	frame[offset++] =
		(force_calibration[SENSOR_ALGO_FORCE_CALIBRATION_REGION_PRIMARY].valid ? 1u : 0u) | (force_calibration[SENSOR_ALGO_FORCE_CALIBRATION_REGION_SECONDARY].valid ? 2u : 0u);
#else
	frame[offset++] = 0u;
#endif
	write_u16(frame, &offset, component_mask);
	write_float(frame, &offset, fingertip_normal);
	write_float(frame, &offset, fingertip_tangential);
	write_float(frame, &offset, fingertip_direction);
	write_float(frame, &offset, finger_pad_normal);
	write_float(frame, &offset, finger_pad_tangential);
	write_float(frame, &offset, finger_pad_direction);
	/* 两个值同帧上传，上位机不跨帧拼接，也不自行重算优化值。 */
	write_float(frame, &offset, fingertip_optimized);
	/* 同帧保留截断前实测力及正常优化值，CSV分析不受模拟替换影响。 */
	write_float(frame, &offset, fingertip_measured);
	write_float(frame, &offset, fingertip_measured_optimized);

	send_output_frame(frame, SENSOR_ALGO_RUN_FORCE_FRAME_SIZE);
}

#if SENSOR_ALGO_FORCE_VECTOR_OUTPUT
static void reset_force_group_recovery(force_group_recovery_t *recovery) {
	uint32_t index;

	recovery->state = FORCE_GROUP_IDLE;
	recovery->peak_deformation = 0.0f;
	recovery->previous_energy = 0.0f;
	recovery->recovery_frames = 0u;
	recovery->release_confirm_frames = 0u;
	recovery->stable_frames = 0u;
	recovery->recontact_confirm_frames = 0u;
	for (index = 0u; index < 4u; index++) {
		recovery->contact_origin[index] = 0.0f;
		recovery->max_delta[index] = 0.0f;
		recovery->min_delta[index] = 0.0f;
		recovery->previous_raw[index] = 0.0f;
		recovery->release_valley_raw[index] = 0.0f;
	}
}

static void clear_channel_signal_output(sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT], uint32_t channel_index) {
	signal_q16[channel_index] = 0u;
	/* 强制清除通道输出时同步清除FLOATING锁定值。 */
	floating_locked_signal_q16[channel_index] = 0u;
	/* 强制清除通道输出时同步清除蠕变补偿值。 */
	floating_compensation_values[channel_index] = 0;
	/* 强制清除通道输出时同步使蠕变补偿失效。 */
	floating_compensation_valid[channel_index] = false;
	idle_counts[channel_index] = 0u;
	active_recal_counts[channel_index] = 0u;
	post_active_baseline_delay_counts[channel_index] = 0u;
	post_active_fast_baseline_counts[channel_index] = 0u;
	release_negative_slope_counts[channel_index] = 0u;
	negative_excursion_counts[channel_index] = 0u;
	channel_was_active[channel_index] = false;
	active_peak_diff_values[channel_index] = 0u;
	post_release_gate_values[channel_index] = 0u;
	post_release_blank_counts[channel_index] = 0u;
	release_counts[channel_index] = 0u;
	fast_release_counts[channel_index] = 0u;
	fast_release_latched[channel_index] = false;
	fast_release_state_counts[channel_index] = 0u;
	release_recovery_counts[channel_index] = 0u;
	release_recovery_stable_counts[channel_index] = 0u;
	previous_signal_values[channel_index] = 0u;
	previous_positive_diff_values[channel_index] = 0u;
	channel_states[channel_index] = SENSOR_ALGO_STATE_IDLE;
	pending_channel_states[channel_index] = SENSOR_ALGO_STATE_IDLE;
	pending_state_counts[channel_index] = 0u;
	outputs[channel_index].signal = 0u;
	latest_snapshot.channels[channel_index].signal = 0u;
	latest_snapshot.states[channel_index] = SENSOR_ALGO_STATE_IDLE;
}

static bool is_force_group_recovering_channel(uint32_t channel_index) {
	uint32_t index;

	for (index = 0u; index < 4u; index++) {
		if ((fingertip_ch_idx[index] == channel_index) && (fingertip_group_recovery.state == FORCE_GROUP_REBOUND_RECOVERY)) {
			return true;
		}
		if ((finger_pad_ch_idx[index] == channel_index) && (finger_pad_group_recovery.state == FORCE_GROUP_REBOUND_RECOVERY)) {
			return true;
		}
	}

	return false;
}

static bool force_group_has_adjacent_pair(uint8_t mask) {
	static const uint8_t adjacent_pairs[4][2] = {
		{0u, 1u},
		{0u, 2u},
		{1u, 3u},
		{2u, 3u},
	};
	uint32_t index;

	for (index = 0u; index < 4u; index++) {
		uint8_t first = (uint8_t)(1u << adjacent_pairs[index][0]);
		uint8_t second = (uint8_t)(1u << adjacent_pairs[index][1]);
		if (((mask & first) != 0u) && ((mask & second) != 0u)) {
			return true;
		}
	}

	return false;
}

static void suppress_force_group_outputs(force_vector_context_t *context, sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT], const uint32_t ch_idx[4],
										 bool fingertip_group) {
	uint32_t index;

	for (index = 0u; index < 4u; index++) {
		uint32_t channel_index = ch_idx[index];

		signal_q16[channel_index] = 0u;
		/* 力组抑制通道输出时同步清除FLOATING锁定值。 */
		floating_locked_signal_q16[channel_index] = 0u;
		/* 力组抑制通道输出时同步清除蠕变补偿值。 */
		floating_compensation_values[channel_index] = 0;
		/* 力组抑制通道输出时同步使蠕变补偿失效。 */
		floating_compensation_valid[channel_index] = false;
		release_counts[channel_index] = 0u;
		fast_release_counts[channel_index] = 0u;
		fast_release_latched[channel_index] = false;
		fast_release_state_counts[channel_index] = 0u;
		previous_signal_values[channel_index] = 0u;
		channel_states[channel_index] = SENSOR_ALGO_STATE_IDLE;
		pending_channel_states[channel_index] = SENSOR_ALGO_STATE_IDLE;
		pending_state_counts[channel_index] = 0u;
		outputs[channel_index].signal = 0u;
		latest_snapshot.channels[channel_index].signal = 0u;
		latest_snapshot.states[channel_index] = SENSOR_ALGO_STATE_IDLE;
	}

	context->contact_active = false;
	context->theta = 0.0f;
	context->tangential_force = 0.0f;
	context->confidence = 0.0f;
	memset(&context->debug, 0, sizeof(context->debug));
	if (fingertip_group) {
		force_vector_group_signal_sum = 0u;
		force_vector_group_contact_hint = false;
		force_vector_group_touch_peak = 0.0f;
	}
}

static void enter_force_group_recovery(force_group_recovery_t *recovery, force_vector_context_t *context, sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT],
									   const uint32_t ch_idx[4], bool fingertip_group) {
	uint32_t index;

	recovery->state = FORCE_GROUP_REBOUND_RECOVERY;
	recovery->recovery_frames = 0u;
	recovery->stable_frames = 0u;
	recovery->recontact_confirm_frames = 0u;
	recovery->release_confirm_frames = 0u;
	recovery->previous_energy = 0.0f;
	for (index = 0u; index < 4u; index++) {
		uint32_t channel_index = ch_idx[index];
		float raw = (float)outputs[channel_index].raw;

		recovery->release_valley_raw[index] = raw;
		recovery->previous_raw[index] = raw;
		begin_channel_release_recovery(channel_index);
	}

	force_vector_context_set_touch_detection_enabled(context, false);
	force_vector_context_force_recovery(context);
	suppress_force_group_outputs(context, outputs, ch_idx, fingertip_group);
}

static void finish_force_group_recovery(force_group_recovery_t *recovery, force_vector_context_t *context, sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT],
										const uint32_t ch_idx[4], bool fingertip_group) {
	float recovered_baseline[4];
	uint32_t index;

	force_vector_context_get_idle_baseline(context, recovered_baseline);
	for (index = 0u; index < 4u; index++) {
		uint32_t channel_index = ch_idx[index];
		uint32_t baseline = (uint32_t)(recovered_baseline[index] + 0.5f);

		baseline_q16[channel_index] = baseline << 16u;
		clear_channel_signal_output(outputs, channel_index);
	}

	force_vector_context_finish_recovery(context);
	force_vector_context_set_touch_detection_enabled(context, true);
	reset_force_group_recovery(recovery);
	if (fingertip_group) {
		force_vector_group_signal_sum = 0u;
		force_vector_group_contact_hint = false;
		force_vector_group_touch_peak = 0.0f;
	}
}

static void update_force_group_recovery(force_group_recovery_t *recovery, force_vector_context_t *context, sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT],
										const uint32_t ch_idx[4], bool fingertip_group) {
	uint32_t index;

	if (recovery->state == FORCE_GROUP_IDLE) {
		if (context->contact_active) {
			recovery->state = FORCE_GROUP_TOUCH_ACTIVE;
			recovery->peak_deformation = 0.0f;
			recovery->release_confirm_frames = 0u;
			for (index = 0u; index < 4u; index++) {
				uint32_t channel_index = ch_idx[index];
				float raw = (float)outputs[channel_index].raw;
				float origin = (float)q16_to_u16(baseline_q16[channel_index]);

				recovery->contact_origin[index] = origin;
				recovery->max_delta[index] = fmaxf(raw - origin, 0.0f);
				recovery->min_delta[index] = fminf(raw - origin, 0.0f);
				recovery->previous_raw[index] = raw;
			}
		}
		return;
	}

	if (recovery->state == FORCE_GROUP_TOUCH_ACTIVE) {
		float score_sum = 0.0f;
		float weight_sum = 0.0f;
		float square_sum = 0.0f;
		float release_score;
		float deformation;

		if (!context->contact_active) {
			enter_force_group_recovery(recovery, context, outputs, ch_idx, fingertip_group);
			return;
		}

		for (index = 0u; index < 4u; index++) {
			uint32_t channel_index = ch_idx[index];
			float raw = (float)outputs[channel_index].raw;
			/* 噪声统一按固定值计算，不受校准结果影响。 */
			float noise = (float)SENSOR_ALGO_FIXED_NOISE;
			float delta = raw - recovery->contact_origin[index];
			float extreme;
			float progress;

			recovery->max_delta[index] = fmaxf(recovery->max_delta[index], delta);
			recovery->min_delta[index] = fminf(recovery->min_delta[index], delta);
			extreme = fmaxf(recovery->max_delta[index], -recovery->min_delta[index]);
			square_sum += delta * delta;
			if (extreme >= noise * 3.0f) {
				if (recovery->max_delta[index] >= -recovery->min_delta[index]) {
					progress = (recovery->max_delta[index] - delta) / fmaxf(recovery->max_delta[index], 1.0f);
				} else {
					progress = (delta - recovery->min_delta[index]) / fmaxf(-recovery->min_delta[index], 1.0f);
				}
				progress = fminf(fmaxf(progress, 0.0f), 1.0f);
				score_sum += fminf(extreme, 300.0f) * progress;
				weight_sum += fminf(extreme, 300.0f);
			}
			recovery->previous_raw[index] = raw;
		}

		deformation = sqrtf(square_sum * 0.25f);
		recovery->peak_deformation = fmaxf(recovery->peak_deformation, deformation);
		release_score = (weight_sum > 0.0f) ? (score_sum / weight_sum) : 0.0f;
		if ((release_score >= SENSOR_ALGO_GROUP_RELEASE_SCORE_THRESHOLD) && (recovery->peak_deformation > 0.0f) &&
			(deformation <= recovery->peak_deformation * SENSOR_ALGO_GROUP_RELEASE_PEAK_RATIO)) {
			if (recovery->release_confirm_frames < UINT8_MAX) {
				recovery->release_confirm_frames++;
			}
		} else {
			recovery->release_confirm_frames = 0u;
		}

		if (recovery->release_confirm_frames >= SENSOR_ALGO_GROUP_RELEASE_CONFIRM_FRAMES) {
			enter_force_group_recovery(recovery, context, outputs, ch_idx, fingertip_group);
		}
		return;
	}

	if (recovery->state == FORCE_GROUP_REBOUND_RECOVERY) {
		uint8_t stable_channels = 0u;
		uint8_t recontact_mask = 0u;
		float energy = 0.0f;
		bool energy_growing;

		if (recovery->recovery_frames < UINT16_MAX) {
			recovery->recovery_frames++;
		}

		for (index = 0u; index < 4u; index++) {
			uint32_t channel_index = ch_idx[index];
			float raw = (float)outputs[channel_index].raw;
			float baseline = (float)q16_to_u16(baseline_q16[channel_index]);
			/* 噪声统一按固定值计算，不受校准结果影响。 */
			float noise = (float)SENSOR_ALGO_FIXED_NOISE;
			float slope = raw - recovery->previous_raw[index];
			float residual = raw - baseline;
			float delta_from_valley;
			float residual_limit = fmaxf(noise * SENSOR_ALGO_GROUP_RECOVERY_RESIDUAL_NOISE_SCALE, SENSOR_ALGO_GROUP_RECOVERY_RESIDUAL_MIN_COUNTS);
			float slope_limit = fmaxf(noise * SENSOR_ALGO_GROUP_RECOVERY_SLOPE_NOISE_SCALE, SENSOR_ALGO_GROUP_RECOVERY_SLOPE_MIN_COUNTS);

			recovery->release_valley_raw[index] = fminf(recovery->release_valley_raw[index], raw);
			delta_from_valley = raw - recovery->release_valley_raw[index];
			if (residual > 0.0f) {
				energy += residual;
			}

			if ((recovery->recovery_frames >= SENSOR_ALGO_GROUP_RECOVERY_HARD_FRAMES) &&
				(delta_from_valley >= fmaxf(noise * SENSOR_ALGO_GROUP_RECONTACT_DELTA_NOISE_SCALE, SENSOR_ALGO_GROUP_RECONTACT_DELTA_MIN_COUNTS)) &&
				(residual >= fmaxf(noise * SENSOR_ALGO_GROUP_RECONTACT_DELTA_NOISE_SCALE, SENSOR_ALGO_GROUP_RECONTACT_DELTA_MIN_COUNTS)) &&
				(slope >= fmaxf(noise * SENSOR_ALGO_GROUP_RECONTACT_SLOPE_NOISE_SCALE, SENSOR_ALGO_GROUP_RECONTACT_SLOPE_MIN_COUNTS))) {
				recontact_mask |= (uint8_t)(1u << index);
			}

			if ((fabsf(residual) <= residual_limit) && (fabsf(slope) <= slope_limit)) {
				stable_channels++;
			}
			recovery->previous_raw[index] = raw;
		}

		energy_growing = energy > recovery->previous_energy + 1.0f;
		if (force_group_has_adjacent_pair(recontact_mask) && energy_growing) {
			if (recovery->recontact_confirm_frames < UINT8_MAX) {
				recovery->recontact_confirm_frames++;
			}
		} else {
			recovery->recontact_confirm_frames = 0u;
		}
		recovery->previous_energy = energy;

		if (recovery->recontact_confirm_frames >= SENSOR_ALGO_GROUP_RECONTACT_CONFIRM_FRAMES) {
			recovery->state = FORCE_GROUP_TOUCH_ACTIVE;
			recovery->peak_deformation = 0.0f;
			recovery->release_confirm_frames = 0u;
			recovery->recontact_confirm_frames = 0u;
			force_vector_context_set_touch_detection_enabled(context, true);
			force_vector_context_confirm_recontact(context);
			for (index = 0u; index < 4u; index++) {
				uint32_t channel_index = ch_idx[index];
				float raw = (float)outputs[channel_index].raw;
				float origin = (float)q16_to_u16(baseline_q16[channel_index]);

				recovery->contact_origin[index] = origin;
				recovery->max_delta[index] = fmaxf(raw - origin, 0.0f);
				recovery->min_delta[index] = fminf(raw - origin, 0.0f);
				recovery->previous_raw[index] = raw;
				signal_q16[channel_index] = 0u;
				/* 力组重新接触时清除上一轮FLOATING锁定值。 */
				floating_locked_signal_q16[channel_index] = 0u;
				/* 力组重新接触时清除上一轮蠕变补偿值。 */
				floating_compensation_values[channel_index] = 0;
				/* 力组重新接触时使上一轮蠕变补偿失效。 */
				floating_compensation_valid[channel_index] = false;
				release_recovery_counts[channel_index] = 0u;
				release_recovery_stable_counts[channel_index] = 0u;
				post_release_gate_values[channel_index] = 0u;
				post_release_blank_counts[channel_index] = 0u;
				release_counts[channel_index] = 0u;
				fast_release_counts[channel_index] = 0u;
				fast_release_latched[channel_index] = false;
				previous_signal_values[channel_index] = 0u;
				previous_positive_diff_values[channel_index] = 0u;
				outputs[channel_index].signal = 0u;
			}
			return;
		}

		if (stable_channels >= SENSOR_ALGO_GROUP_RECOVERY_STABLE_CHANNELS) {
			if (recovery->stable_frames < UINT8_MAX) {
				recovery->stable_frames++;
			}
		} else {
			recovery->stable_frames = 0u;
		}

		if (((recovery->recovery_frames >= SENSOR_ALGO_GROUP_RECOVERY_MIN_FRAMES) && (recovery->stable_frames >= SENSOR_ALGO_GROUP_RECOVERY_STABLE_FRAMES)) ||
			(recovery->recovery_frames >= SENSOR_ALGO_GROUP_RECOVERY_MAX_FRAMES)) {
			finish_force_group_recovery(recovery, context, outputs, ch_idx, fingertip_group);
			return;
		}

		suppress_force_group_outputs(context, outputs, ch_idx, fingertip_group);
	}
}

static void update_force_group_recoveries(sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT]) {
	if (!calibrated) {
		reset_force_group_recovery(&fingertip_group_recovery);
		reset_force_group_recovery(&finger_pad_group_recovery);
		force_vector_context_set_touch_detection_enabled(&fingertip_force_vector, true);
		force_vector_context_set_touch_detection_enabled(&finger_pad_force_vector, true);
		return;
	}

	update_force_group_recovery(&fingertip_group_recovery, &fingertip_force_vector, outputs, fingertip_ch_idx, true);
	update_force_group_recovery(&finger_pad_group_recovery, &finger_pad_force_vector, outputs, finger_pad_ch_idx, false);
}

static void update_force_vector(sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT]) {
#if SENSOR_ALGO_CHANNEL_COUNT == 8u
	/* 指尖: p00=ch6, p01=ch7, p10=ch4, p11=ch5。 */
	/* 指腹: 左上ch1、左下ch0、右上ch3、右下ch2。 */
#else
	static const uint32_t fingertip_ch_idx[4] = {0u, 3u, 1u, 4u};
#endif

	if (!calibrated) {
		force_vector_context_reset(&fingertip_force_vector);
		force_vector_context_reset(&finger_pad_force_vector);
		return;
	}

	force_vector_context_set_touch_detection_enabled(&fingertip_force_vector, fingertip_group_recovery.state != FORCE_GROUP_REBOUND_RECOVERY);
	force_vector_context_set_touch_detection_enabled(&finger_pad_force_vector, finger_pad_group_recovery.state != FORCE_GROUP_REBOUND_RECOVERY);

	update_force_vector_context(&fingertip_force_vector, outputs, fingertip_ch_idx);
#if SENSOR_ALGO_CHANNEL_COUNT == 8u
	update_force_vector_context(&finger_pad_force_vector, outputs, finger_pad_ch_idx);
#endif
}

static void update_force_vector_context(force_vector_context_t *context, sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT], const uint32_t ch_idx[4]) {
	float raw[4];
	float noise[4];
	float patch_values[4];
	uint32_t index;

	for (index = 0u; index < 4u; index++) {
		uint32_t channel_index = ch_idx[index];

		raw[index] = (float)outputs[channel_index].raw;
		/* 噪声统一按固定值计算，不受校准结果影响。 */
		noise[index] = (float)SENSOR_ALGO_FIXED_NOISE;
	}

	force_vector_context_cal(context, raw, noise);
	if (context->contact_active) {
		patch_values[0u] = context->debug.p00;
		patch_values[1u] = context->debug.p01;
		patch_values[2u] = context->debug.p10;
		patch_values[3u] = context->debug.p11;
		for (index = 0u; index < 4u; index++) {
			uint32_t channel_index = ch_idx[index];
			float patch = patch_values[index];
			uint16_t signal = (patch >= 65535.0f) ? UINT16_MAX : (uint16_t)(patch + 0.5f);
			/* 噪声统一按固定值计算，不受校准结果影响。 */
			sensor_algorithm_state_t state = (signal > SENSOR_ALGO_FIXED_NOISE) ? SENSOR_ALGO_STATE_ACTIVE : SENSOR_ALGO_STATE_IDLE;

			signal_q16[channel_index] = (uint32_t)signal << 16u;
			previous_signal_values[channel_index] = signal;
			channel_states[channel_index] = state;
			pending_channel_states[channel_index] = state;
			pending_state_counts[channel_index] = 0u;
			outputs[channel_index].signal = signal;
			latest_snapshot.channels[channel_index].signal = signal;
			latest_snapshot.states[channel_index] = state;
		}
	}
}

static void send_force_vector_frame(uint8_t frame_type, uint8_t region, const force_vector_debug_t *debug, bool contact_active) {
	uint32_t offset = 0u;
	uint8_t frame[SENSOR_ALGO_FORCE_VECTOR_FRAME_SIZE];
	float direction = 0.0f;
	float tangential_force = 0.0f;
	float normal_force = 0.0f;
	float force_x = 0.0f;
	float force_y = 0.0f;
	const sensor_force_calibration_t *calibration = &force_calibration[region];

	if (calibrated && contact_active) {
		tangential_force = apply_tangential_force_calibration(debug->tangential_force / SENSOR_ALGO_FORCE_REPORT_DIVISOR, calibration);
		direction = (tangential_force > 0.0f) ? apply_direction_calibration(debug->theta, calibration) : 0.0f;
		normal_force = apply_normal_force_calibration(debug->normal_force / SENSOR_ALGO_FORCE_REPORT_DIVISOR, calibration);
		force_x = tangential_force * cosf(direction);
		force_y = tangential_force * sinf(direction);
	}

	write_frame_header(frame, &offset, frame_type, SENSOR_ALGO_CHANNEL_COUNT, SENSOR_ALGO_FORCE_VECTOR_PAYLOAD_SIZE);
	write_float(frame, &offset, direction);
	write_float(frame, &offset, tangential_force);
	write_float(frame, &offset, debug->confidence);
	write_float(frame, &offset, normal_force);
	write_float(frame, &offset, debug->cop_x);
	write_float(frame, &offset, debug->cop_y);
	write_float(frame, &offset, debug->diff_x);
	write_float(frame, &offset, debug->diff_y);
	write_float(frame, &offset, force_x);
	write_float(frame, &offset, force_y);
	write_float(frame, &offset, debug->p00);
	write_float(frame, &offset, debug->p01);
	write_float(frame, &offset, debug->p10);
	write_float(frame, &offset, debug->p11);

	send_output_frame(frame, SENSOR_ALGO_FORCE_VECTOR_FRAME_SIZE);
}
#endif

void SensorAlgorithm_GetCalibratedForces(float *normal_force, float *tangential_force, float *direction) {
	SensorAlgorithm_GetCalibratedForcesForRegion(SENSOR_ALGO_FORCE_CALIBRATION_REGION_PRIMARY, normal_force, tangential_force, direction);
}

void SensorAlgorithm_GetCalibratedForcesForRegion(uint8_t region, float *normal_force, float *tangential_force, float *direction) {
#if SENSOR_ALGO_FORCE_VECTOR_OUTPUT
	const force_vector_context_t *context;
	const sensor_force_calibration_t *calibration;

	if (region >= SENSOR_ALGO_FORCE_CALIBRATION_REGION_COUNT) {
		*normal_force = 0.0f;
		*tangential_force = 0.0f;
		*direction = 0.0f;
		return;
	}
	context = (region == SENSOR_ALGO_FORCE_CALIBRATION_REGION_PRIMARY) ? &fingertip_force_vector : &finger_pad_force_vector;
	calibration = &force_calibration[region];
	if (!calibrated || !context->contact_active) {
		*normal_force = 0.0f;
		*tangential_force = 0.0f;
		*direction = 0.0f;
		return;
	}

	*normal_force = apply_normal_force_calibration(context->debug.normal_force / SENSOR_ALGO_FORCE_REPORT_DIVISOR, calibration);
	*tangential_force = apply_tangential_force_calibration(context->debug.tangential_force / SENSOR_ALGO_FORCE_REPORT_DIVISOR, calibration);
	*direction = (*tangential_force > 0.0f) ? apply_direction_calibration(context->debug.theta, calibration) : 0.0f;
#else
	*normal_force = 0.0f;
	*tangential_force = 0.0f;
	*direction = 0.0f;
#endif
}

void SensorAlgorithm_GetCalibratedForceVector(float *force_x, float *force_y, float *force_z) {
#if SENSOR_ALGO_FORCE_VECTOR_OUTPUT
	const sensor_force_calibration_t *calibration = &force_calibration[SENSOR_ALGO_FORCE_CALIBRATION_REGION_PRIMARY];
	float tangential_force;
	float direction;

	if (!calibrated || !fingertip_force_vector.contact_active) {
		*force_x = 0.0f;
		*force_y = 0.0f;
		*force_z = 0.0f;
		return;
	}

	tangential_force = apply_tangential_force_calibration(fingertip_force_vector.debug.tangential_force / SENSOR_ALGO_FORCE_REPORT_DIVISOR, calibration);
	direction = apply_direction_calibration(fingertip_force_vector.debug.theta, calibration);
	*force_x = tangential_force * cosf(direction);
	*force_y = tangential_force * sinf(direction);
	*force_z = apply_normal_force_calibration(fingertip_force_vector.debug.normal_force / SENSOR_ALGO_FORCE_REPORT_DIVISOR, calibration);
#else
	*force_x = 0.0f;
	*force_y = 0.0f;
	*force_z = 0.0f;
#endif
}

static void send_force_calibration_frame(void) {
	uint8_t region;

	for (region = 0u; region < SENSOR_ALGO_FORCE_CALIBRATION_REGION_COUNT; region++) {
		uint32_t offset = 0u;
		uint8_t frame[SENSOR_ALGO_PROTOCOL_HEADER_SIZE + 24u];
		const sensor_force_calibration_t *calibration = &force_calibration[region];
#if SENSOR_ALGO_FORCE_VECTOR_OUTPUT
		const force_vector_context_t *context = (region == SENSOR_ALGO_FORCE_CALIBRATION_REGION_PRIMARY) ? &fingertip_force_vector : &finger_pad_force_vector;
#endif
		float normal_raw = 0.0f;
		float tangential_raw = 0.0f;
		float normal_force = 0.0f;
		float tangential_force = 0.0f;
		float direction = 0.0f;

#if SENSOR_ALGO_FORCE_VECTOR_OUTPUT
		if (calibrated && context->contact_active) {
			normal_raw = context->debug.normal_force / SENSOR_ALGO_FORCE_REPORT_DIVISOR;
			tangential_raw = context->debug.tangential_force / SENSOR_ALGO_FORCE_REPORT_DIVISOR;
			normal_force = apply_normal_force_calibration(normal_raw, calibration);
			tangential_force = apply_tangential_force_calibration(tangential_raw, calibration);
			direction = (tangential_force > 0.0f) ? context->debug.theta : 0.0f;
		}
#endif
		write_frame_header(frame, &offset, SENSOR_ALGO_FRAME_TYPE_FORCE_CALIBRATION, SENSOR_ALGO_CHANNEL_COUNT, 24u);
		frame[offset++] = region;
		frame[offset++] = (calibration->valid ? 1u : 0u) | (calibration->dirty ? 2u : 0u);
		write_u16(frame, &offset, 0u);
		write_float(frame, &offset, normal_raw);
		write_float(frame, &offset, tangential_raw);
		write_float(frame, &offset, direction);
		write_float(frame, &offset, normal_force);
		write_float(frame, &offset, tangential_force);
		send_output_frame(frame, sizeof(frame));
	}
}

static void send_force_calibration_parameters_frame(uint8_t region, uint8_t force_type) {
	uint32_t offset = 0u;
	uint32_t index;
	const sensor_force_calibration_t *calibration = &force_calibration[region];
	uint8_t point_count;
	const float *raw_points;
	const float *force_points;
	bool region_valid;
	bool region_dirty;
	uint16_t payload_size;
	uint8_t frame[SENSOR_ALGO_FORCE_CALIBRATION_PARAMETERS_MAX_FRAME_SIZE];

	if (force_type == SENSOR_ALGO_FORCE_CALIBRATION_FORCE_TYPE_NORMAL) {
		point_count = calibration->normal_point_count;
		raw_points = calibration->normal_raw_points;
		force_points = calibration->normal_force_points;
		region_valid = calibration->normal_valid;
		region_dirty = calibration->normal_dirty;
	} else if (force_type == SENSOR_ALGO_FORCE_CALIBRATION_FORCE_TYPE_TANGENTIAL) {
		point_count = calibration->tangential_point_count;
		raw_points = calibration->tangential_raw_points;
		force_points = calibration->tangential_force_points;
		region_valid = calibration->tangential_valid;
		region_dirty = calibration->tangential_dirty;
	} else {
		point_count = calibration->direction_point_count;
		raw_points = calibration->direction_raw_points;
		force_points = calibration->direction_output_points;
		region_valid = calibration->direction_valid;
		region_dirty = calibration->direction_dirty;
	}
	payload_size = (uint16_t)(SENSOR_ALGO_FORCE_CALIBRATION_PARAMETERS_BASE_PAYLOAD_SIZE + ((uint16_t)point_count * 2u * SENSOR_ALGO_BYTES_PER_FLOAT));

	write_frame_header(frame, &offset, SENSOR_ALGO_FRAME_TYPE_FORCE_CALIBRATION_PARAMETERS, 0u, payload_size);
	frame[offset++] = region;
	frame[offset++] = force_type;
	frame[offset++] = SENSOR_ALGO_FORCE_CALIBRATION_MODEL_PIECEWISE_LINEAR;
	frame[offset++] = point_count;
	frame[offset++] = (region_valid ? 1u : 0u) | (region_dirty ? 2u : 0u);
	frame[offset++] = 0u;
	frame[offset++] = 0u;
	frame[offset++] = 0u;
	for (index = 0u; index < point_count; index++) {
		write_float(frame, &offset, raw_points[index]);
		write_float(frame, &offset, force_points[index]);
	}
	send_command_response_frame(frame, SENSOR_ALGO_PROTOCOL_HEADER_SIZE + payload_size);
}

static void send_output_frame(const uint8_t *frame, uint32_t size) {
#if (TUNER_INTERFACE_TYPE == UART)
	if (uart_tx_count == SENSOR_ALGO_UART_TX_BUFFER_COUNT) {
		return;
	}

	memcpy(uart_tx_buffers[uart_tx_tail], frame, size);
	uart_tx_sizes[uart_tx_tail] = size;
	uart_tx_tail = (uart_tx_tail + 1u) % SENSOR_ALGO_UART_TX_BUFFER_COUNT;
	uart_tx_count++;
	SensorAlgorithm_UartTxPoll();
#else
	(void)frame;
	(void)size;
#endif
}

static void send_command_response_frame(uint8_t *frame, uint32_t size) {
#if (TUNER_INTERFACE_TYPE == UART)
	while (uart_tx_count != 0u) {
		SensorAlgorithm_UartTxPoll();
	}

	while (!Cy_SCB_UART_IsTxComplete(CYBSP_UART_HW)) {
	}
	Cy_SCB_UART_PutArrayBlocking(CYBSP_UART_HW, frame, size);
	while (!Cy_SCB_UART_IsTxComplete(CYBSP_UART_HW)) {
	}
#else
	(void)frame;
	(void)size;
#endif
}

static void initialize_force_calibration(void) {
	sensor_force_calibration_t loaded;
	uint8_t region;

	memset(force_calibration, 0, sizeof(force_calibration));
	/* 每次重新加载都重新判断记录所在布局，不自动写Flash。 */
	memset(force_calibration_legacy_loaded, 0, sizeof(force_calibration_legacy_loaded));
	for (region = 0u; region < SENSOR_ALGO_FORCE_CALIBRATION_REGION_COUNT; region++) {
		force_calibration[region].normal_raw_points[1u] = 1.0f;
		force_calibration[region].normal_force_points[1u] = 1.0f;
		force_calibration[region].tangential_raw_points[1u] = 1.0f;
		force_calibration[region].tangential_force_points[1u] = 1.0f;
		force_calibration[region].direction_raw_points[1u] = 360.0f;
		force_calibration[region].direction_output_points[1u] = 360.0f;
		force_calibration[region].normal_point_count = 2u;
		force_calibration[region].tangential_point_count = 2u;
		force_calibration[region].direction_point_count = 2u;
		if (load_force_calibration_from_nvm(&loaded, region)) {
			force_calibration[region] = loaded;
		}
	}
	memcpy(saved_force_calibration, force_calibration, sizeof(force_calibration));
}

static bool load_force_calibration_from_nvm(sensor_force_calibration_t *calibration, uint8_t region) {
	const volatile uint8_t *storage =
		(const volatile uint8_t *)(SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_ADDRESS +
								 ((uint32_t)region * SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_RECORD_SIZE));
	uint8_t data[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_DATA_SIZE];
	uint32_t index;
	uint32_t checksum;
	for (index = 0u; index < sizeof(data); index++) {
		data[index] = storage[index];
	}
	/* FCV9兼容10/20点高力表与48点旧表；无效旧表可保留用于迁移其他分量。 */
	if ((data[0u] != 'F') || (data[1u] != 'C') || (data[2u] != 'V') || (data[3u] != '9') || (data[4u] != SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_VERSION) ||
		(data[5u] != SENSOR_ALGO_DEVICE_TYPE) || (data[9u] != region) || (data[10u] != SENSOR_ALGO_FORCE_CALIBRATION_DIRECTION_POINTS) || (data[7u] < 2u) ||
		/* 有效主法向表允许10/20/48节点，其余分量和旧无效默认表维持原检查。 */
		((region == SENSOR_ALGO_FORCE_CALIBRATION_REGION_PRIMARY) && ((data[6u] & 1u) != 0u) && !primary_normal_point_count_valid(data[7u])) ||
		(data[7u] > SENSOR_ALGO_FORCE_CALIBRATION_MAX_NORMAL_POINTS) || (data[8u] < 2u) || (data[8u] > SENSOR_ALGO_FORCE_CALIBRATION_MAX_TANGENTIAL_POINTS)) {
		return load_previous_force_calibration_from_nvm(calibration, region);
	}
	checksum = (uint32_t)storage[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_CHECKSUM_OFFSET] | ((uint32_t)storage[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_CHECKSUM_OFFSET + 1u] << 8u) |
			   ((uint32_t)storage[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_CHECKSUM_OFFSET + 2u] << 16u) |
			   ((uint32_t)storage[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_CHECKSUM_OFFSET + 3u] << 24u);
	if (checksum != calculate_storage_checksum(data, sizeof(data))) {
		return load_previous_force_calibration_from_nvm(calibration, region);
	}
	memset(calibration, 0, sizeof(*calibration));
	calibration->normal_point_count = data[7u];
	calibration->tangential_point_count = data[8u];
	calibration->direction_point_count = data[10u];
	calibration->normal_valid = (data[6u] & 1u) != 0u;
	calibration->tangential_valid = (data[6u] & 2u) != 0u;
	calibration->direction_valid = (data[6u] & 4u) != 0u;
	calibration->valid = calibration->normal_valid && calibration->tangential_valid && calibration->direction_valid;
	calibration->dirty = false;
	for (index = 0u; index < SENSOR_ALGO_FORCE_CALIBRATION_MAX_NORMAL_POINTS; index++) {
		calibration->normal_raw_points[index] = read_float(&data[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_NORMAL_RAW_OFFSET + index * 4u]);
		calibration->normal_force_points[index] = read_float(&data[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_NORMAL_FORCE_OFFSET + index * 4u]);
	}
	for (index = 0u; index < SENSOR_ALGO_FORCE_CALIBRATION_MAX_TANGENTIAL_POINTS; index++) {
		calibration->tangential_raw_points[index] = read_float(&data[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_TANGENTIAL_RAW_OFFSET + index * 4u]);
		calibration->tangential_force_points[index] = read_float(&data[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_TANGENTIAL_FORCE_OFFSET + index * 4u]);
	}
	for (index = 0u; index < SENSOR_ALGO_FORCE_CALIBRATION_DIRECTION_POINTS; index++) {
		calibration->direction_raw_points[index] = read_float(&data[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_DIRECTION_RAW_OFFSET + index * 4u]);
		calibration->direction_output_points[index] = read_float(&data[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_DIRECTION_OUTPUT_OFFSET + index * 4u]);
	}
	/* 回读时重复验证高力范围，损坏或不完整的新表不能成为有效运行表。 */
	return force_calibration_values_valid(calibration) &&
		((region != SENSOR_ALGO_FORCE_CALIBRATION_REGION_PRIMARY) || !calibration->normal_valid || primary_normal_table_valid(calibration));
}

/* 只读迁移旧20节点布局，保留其他分量而不自动擦写任何Flash。 */
static bool load_previous_force_calibration_from_nvm(sensor_force_calibration_t *calibration, uint8_t region) {
	/* FCV8旧记录的两个区域也完全位于DFU保护边界之前。 */
	const volatile uint8_t *storage = (const volatile uint8_t *)(0x0001F900u + (uint32_t)region * 512u);
	uint8_t data[352u];
	uint32_t index;
	uint32_t checksum;
	/* 按旧布局读取固定头、节点和校验，绝不使用新数组大小解释旧记录。 */
	for (index = 0u; index < sizeof(data); index++) {
		data[index] = storage[index];
	}
	checksum = (uint32_t)storage[352u] | ((uint32_t)storage[353u] << 8u) |
		((uint32_t)storage[354u] << 16u) | ((uint32_t)storage[355u] << 24u);
	if ((data[0u] != 'F') || (data[1u] != 'C') || (data[2u] != 'V') || (data[3u] != '8') ||
		(data[4u] != 8u) || (data[5u] != SENSOR_ALGO_DEVICE_TYPE) || (data[9u] != region) ||
		(data[7u] < 2u) || (data[7u] > 20u) || (data[8u] < 2u) || (data[8u] > 20u) ||
		(data[10u] != 2u) || (checksum != calculate_storage_checksum(data, sizeof(data)))) {
		return load_legacy_force_calibration_from_nvm(calibration, region);
	}
	memset(calibration, 0, sizeof(*calibration));
	calibration->normal_point_count = data[7u];
	calibration->tangential_point_count = data[8u];
	calibration->direction_point_count = 2u;
	/* 旧主表缺少低力段及20至30N范围，需重新标定后才能按N使用。 */
	calibration->normal_valid = (region != SENSOR_ALGO_FORCE_CALIBRATION_REGION_PRIMARY) && ((data[6u] & 1u) != 0u);
	calibration->tangential_valid = (data[6u] & 2u) != 0u;
	calibration->direction_valid = (data[6u] & 4u) != 0u;
	calibration->valid = calibration->normal_valid && calibration->tangential_valid && calibration->direction_valid;
	/* 固定旧表偏移，避免扩容后错位读取。 */
	for (index = 0u; index < 20u; index++) {
		calibration->normal_raw_points[index] = read_float(&data[16u + 4u * index]);
		calibration->normal_force_points[index] = read_float(&data[96u + 4u * index]);
		calibration->tangential_raw_points[index] = read_float(&data[176u + 4u * index]);
		calibration->tangential_force_points[index] = read_float(&data[256u + 4u * index]);
	}
	for (index = 0u; index < 2u; index++) {
		calibration->direction_raw_points[index] = read_float(&data[336u + 4u * index]);
		calibration->direction_output_points[index] = read_float(&data[344u + 4u * index]);
	}
	/* 仅有效旧记录需要在将来保存重叠区域前迁移。 */
	force_calibration_legacy_loaded[region] = force_calibration_values_valid(calibration);
	return force_calibration_legacy_loaded[region];
}

/* 按原FCV7的13节点布局只读加载旧参数。 */
static bool load_legacy_force_calibration_from_nvm(sensor_force_calibration_t *calibration, uint8_t region) {
	const volatile uint8_t *storage =
		(const volatile uint8_t *)(SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_STORAGE_ADDRESS + ((uint32_t)region * CY_FLASH_SIZEOF_ROW));
	uint8_t data[SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_DATA_SIZE];
	uint32_t index;
	uint32_t checksum;

	for (index = 0u; index < sizeof(data); index++) {
		data[index] = storage[index];
	}
	if ((data[0u] != 'F') || (data[1u] != 'C') || (data[2u] != 'V') || (data[3u] != '6') ||
		(data[4u] != SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_STORAGE_VERSION) || (data[5u] != SENSOR_ALGO_DEVICE_TYPE) || (data[9u] != region) ||
		(data[10u] != SENSOR_ALGO_FORCE_CALIBRATION_DIRECTION_POINTS) || (data[7u] < 2u) ||
		(data[7u] > SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_MAX_NORMAL_POINTS) || (data[8u] < 2u) ||
		(data[8u] > SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_MAX_TANGENTIAL_POINTS)) {
		return false;
	}
	checksum = (uint32_t)storage[SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_CHECKSUM_OFFSET] |
			   ((uint32_t)storage[SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_CHECKSUM_OFFSET + 1u] << 8u) |
			   ((uint32_t)storage[SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_CHECKSUM_OFFSET + 2u] << 16u) |
			   ((uint32_t)storage[SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_CHECKSUM_OFFSET + 3u] << 24u);
	if (checksum != calculate_storage_checksum(data, sizeof(data))) {
		return false;
	}

	memset(calibration, 0, sizeof(*calibration));
	calibration->normal_point_count = data[7u];
	calibration->tangential_point_count = data[8u];
	calibration->direction_point_count = data[10u];
	/* 旧主表只能回读，不能作为新的两段力值表使用。 */
	calibration->normal_valid = (region != SENSOR_ALGO_FORCE_CALIBRATION_REGION_PRIMARY) && ((data[6u] & 1u) != 0u);
	calibration->tangential_valid = (data[6u] & 2u) != 0u;
	calibration->direction_valid = (data[6u] & 4u) != 0u;
	calibration->valid = calibration->normal_valid && calibration->tangential_valid && calibration->direction_valid;
	calibration->dirty = false;
	for (index = 0u; index < SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_MAX_NORMAL_POINTS; index++) {
		calibration->normal_raw_points[index] = read_float(&data[SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_NORMAL_RAW_OFFSET + index * 4u]);
		calibration->normal_force_points[index] = read_float(&data[SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_NORMAL_FORCE_OFFSET + index * 4u]);
	}
	for (index = 0u; index < SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_MAX_TANGENTIAL_POINTS; index++) {
		calibration->tangential_raw_points[index] = read_float(&data[SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_TANGENTIAL_RAW_OFFSET + index * 4u]);
		calibration->tangential_force_points[index] = read_float(&data[SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_TANGENTIAL_FORCE_OFFSET + index * 4u]);
	}
	for (index = 0u; index < SENSOR_ALGO_FORCE_CALIBRATION_DIRECTION_POINTS; index++) {
		calibration->direction_raw_points[index] = read_float(&data[SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_DIRECTION_RAW_OFFSET + index * 4u]);
		calibration->direction_output_points[index] = read_float(&data[SENSOR_ALGO_FORCE_CALIBRATION_LEGACY_DIRECTION_OUTPUT_OFFSET + index * 4u]);
	}
	/* 旧单行布局同样会被新region 1覆盖，需要保留迁移标志。 */
	force_calibration_legacy_loaded[region] = force_calibration_values_valid(calibration);
	return force_calibration_legacy_loaded[region];
}

static bool save_force_calibration_to_nvm(void) {
	uint8_t region;

	for (region = 0u; region < SENSOR_ALGO_FORCE_CALIBRATION_REGION_COUNT; region++) {
		/* 通常只写修改区；写region 1之前先保护仍在旧地址的region 0。 */
		bool migration_required = (region == SENSOR_ALGO_FORCE_CALIBRATION_REGION_PRIMARY) &&
			force_calibration_legacy_loaded[region] && force_calibration[SENSOR_ALGO_FORCE_CALIBRATION_REGION_SECONDARY].dirty;
		if (!force_calibration[region].dirty && !migration_required) {
			continue;
		}
		if (!save_force_calibration_record(&force_calibration[region], region)) {
			return false;
		}
		/* 完整三行回读成功后，此区域已具有新布局记录。 */
		force_calibration_legacy_loaded[region] = false;
	}
	for (region = 0u; region < SENSOR_ALGO_FORCE_CALIBRATION_REGION_COUNT; region++) {
		force_calibration[region].dirty = false;
		force_calibration[region].normal_dirty = false;
		force_calibration[region].tangential_dirty = false;
		force_calibration[region].direction_dirty = false;
	}
	memcpy(saved_force_calibration, force_calibration, sizeof(force_calibration));
	return true;
}

static bool save_force_calibration_record(const sensor_force_calibration_t *calibration, uint8_t region) {
	uint32_t record_data[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_RECORD_SIZE / sizeof(uint32_t)];
	uint8_t *data = (uint8_t *)record_data;
	uint32_t index;
	uint32_t row;
	uint32_t checksum;
	uint32_t storage_address = SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_ADDRESS +
						   ((uint32_t)region * SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_RECORD_SIZE);
	/* 无效主表允许随其他分量迁移，有效主表必须为完整10/20/48节点。 */
	if ((region >= SENSOR_ALGO_FORCE_CALIBRATION_REGION_COUNT) || !force_calibration_values_valid(calibration)) {
		return false;
	}
	if ((region == SENSOR_ALGO_FORCE_CALIBRATION_REGION_PRIMARY) && calibration->normal_valid &&
		!primary_normal_table_valid(calibration)) {
		return false;
	}
	for (index = 0u; index < SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_RECORD_SIZE; index++) {
		data[index] = 0xFFu;
	}
	data[0u] = 'F';
	data[1u] = 'C';
	data[2u] = 'V';
	data[3u] = '9';
	data[4u] = SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_VERSION;
	data[5u] = SENSOR_ALGO_DEVICE_TYPE;
	data[6u] = (calibration->normal_valid ? 1u : 0u) | (calibration->tangential_valid ? 2u : 0u) | (calibration->direction_valid ? 4u : 0u);
	data[7u] = calibration->normal_point_count;
	data[8u] = calibration->tangential_point_count;
	data[9u] = region;
	data[10u] = calibration->direction_point_count;
	for (index = 0u; index < SENSOR_ALGO_FORCE_CALIBRATION_MAX_NORMAL_POINTS; index++) {
		write_float_to_bytes(&data[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_NORMAL_RAW_OFFSET + index * 4u], calibration->normal_raw_points[index]);
		write_float_to_bytes(&data[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_NORMAL_FORCE_OFFSET + index * 4u], calibration->normal_force_points[index]);
	}
	for (index = 0u; index < SENSOR_ALGO_FORCE_CALIBRATION_MAX_TANGENTIAL_POINTS; index++) {
		write_float_to_bytes(&data[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_TANGENTIAL_RAW_OFFSET + index * 4u], calibration->tangential_raw_points[index]);
		write_float_to_bytes(&data[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_TANGENTIAL_FORCE_OFFSET + index * 4u], calibration->tangential_force_points[index]);
	}
	for (index = 0u; index < SENSOR_ALGO_FORCE_CALIBRATION_DIRECTION_POINTS; index++) {
		write_float_to_bytes(&data[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_DIRECTION_RAW_OFFSET + index * 4u], calibration->direction_raw_points[index]);
		write_float_to_bytes(&data[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_DIRECTION_OUTPUT_OFFSET + index * 4u], calibration->direction_output_points[index]);
	}
	checksum = calculate_storage_checksum(data, SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_DATA_SIZE);
	write_float_to_bytes(&data[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_CHECKSUM_OFFSET], 0.0f);
	data[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_CHECKSUM_OFFSET] = (uint8_t)checksum;
	data[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_CHECKSUM_OFFSET + 1u] = (uint8_t)(checksum >> 8u);
	data[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_CHECKSUM_OFFSET + 2u] = (uint8_t)(checksum >> 16u);
	data[SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_CHECKSUM_OFFSET + 3u] = (uint8_t)(checksum >> 24u);
	/* 每个修改区域按地址顺序写入三行，全部低于DFU保护边界。 */
	for (row = 0u; row < SENSOR_ALGO_FORCE_CALIBRATION_STORAGE_ROWS_PER_REGION; row++) {
		/* 无论协议载荷如何，底层统一拒绝DFU区域及越界地址。 */
		if (AppFlash_WriteRow(storage_address + (row * CY_FLASH_SIZEOF_ROW),
						  &record_data[row * (CY_FLASH_SIZEOF_ROW / sizeof(uint32_t))]) != CY_FLASH_DRV_SUCCESS) {
			return false;
		}
	}
	/* 写后逐字节回读，包括校验与填充；失败时保留RAM脏标志。 */
	for (index = 0u; index < sizeof(record_data); index++) {
		if (((const volatile uint8_t *)storage_address)[index] != data[index]) {
			return false;
		}
	}
	return true;
}

static bool force_calibration_values_valid(const sensor_force_calibration_t *calibration) {
	uint32_t index;

	if ((calibration->normal_point_count < 2u) || (calibration->normal_point_count > SENSOR_ALGO_FORCE_CALIBRATION_MAX_NORMAL_POINTS) ||
		(calibration->tangential_point_count < 2u) || (calibration->tangential_point_count > SENSOR_ALGO_FORCE_CALIBRATION_MAX_TANGENTIAL_POINTS) ||
		(calibration->direction_point_count != SENSOR_ALGO_FORCE_CALIBRATION_DIRECTION_POINTS)) {
		return false;
	}
	for (index = 0u; index < calibration->normal_point_count; index++) {
		if (!isfinite(calibration->normal_raw_points[index]) || !isfinite(calibration->normal_force_points[index]) || (calibration->normal_raw_points[index] < 0.0f) ||
			(calibration->normal_force_points[index] < 0.0f) ||
			((index > 0u) && ((calibration->normal_raw_points[index] <= calibration->normal_raw_points[index - 1u]) ||
							  (calibration->normal_force_points[index] < calibration->normal_force_points[index - 1u])))) {
			return false;
		}
	}
	for (index = 0u; index < calibration->tangential_point_count; index++) {
		if (!isfinite(calibration->tangential_raw_points[index]) || !isfinite(calibration->tangential_force_points[index]) || (calibration->tangential_raw_points[index] < 0.0f) ||
			(calibration->tangential_force_points[index] < 0.0f) ||
			((index > 0u) && ((calibration->tangential_raw_points[index] <= calibration->tangential_raw_points[index - 1u]) ||
							  (calibration->tangential_force_points[index] < calibration->tangential_force_points[index - 1u])))) {
			return false;
		}
	}
	if (!isfinite(calibration->direction_raw_points[0u]) || !isfinite(calibration->direction_raw_points[1u]) || !isfinite(calibration->direction_output_points[0u]) ||
		!isfinite(calibration->direction_output_points[1u]) || (fabsf(calibration->direction_raw_points[0u]) > 0.01f) ||
		(fabsf(calibration->direction_raw_points[1u] - 360.0f) > 0.01f) ||
		(fabsf(fabsf(calibration->direction_output_points[1u] - calibration->direction_output_points[0u]) - 360.0f) > 0.01f)) {
		return false;
	}
	return true;
}

static float apply_piecewise_force_calibration(float raw, const float *raw_points, const float *force_points, uint8_t point_count) {
	uint32_t upper = 1u;
	uint32_t lower;
	float value;
	while ((upper < point_count - 1u) && (raw > raw_points[upper])) {
		upper++;
	}
	lower = upper - 1u;
	value = force_points[lower] + ((raw - raw_points[lower]) * (force_points[upper] - force_points[lower]) / (raw_points[upper] - raw_points[lower]));
	return value > 0.0f ? value : 0.0f;
}

/* 食指主法向已采用四点低力表，旧换算只在仍有调用方的变体编译。 */
#if (SENSOR_ALGO_CHANNEL_COUNT < 8u) || SENSOR_ALGO_FORCE_VECTOR_OUTPUT
static float apply_normal_force_calibration(float raw, const sensor_force_calibration_t *calibration) {
	/* 首个实测节点以下使用虚拟(0,0)原点插值，避免首段外推截距造成空载非零力。 */
	if ((raw < calibration->normal_raw_points[0u]) && (calibration->normal_raw_points[0u] > 0.0f)) {
		if (raw <= 0.0f) {
			return 0.0f;
		}
		return raw * calibration->normal_force_points[0u] / calibration->normal_raw_points[0u];
	}

	return apply_piecewise_force_calibration(raw, calibration->normal_raw_points, calibration->normal_force_points, calibration->normal_point_count);
}

/* 把运行法向力连续、单调地轻拉向1～20 N的每个整数点，窗口外保持原值。 */
static float apply_force_output_soft_anchor(float calibrated_force_n) {
#if SENSOR_ALGO_FORCE_OUTPUT_SOFT_ANCHOR_ENABLED
	/* 每个整数锚点各使用左右0.5 N窗口，共用边界恢复原值并保持连续单调。 */
	if ((calibrated_force_n >= SENSOR_ALGO_FORCE_OUTPUT_SOFT_ANCHOR_MIN_N - SENSOR_ALGO_FORCE_OUTPUT_SOFT_ANCHOR_WINDOW_N) &&
		(calibrated_force_n <= SENSOR_ALGO_FORCE_OUTPUT_SOFT_ANCHOR_MAX_N + SENSOR_ALGO_FORCE_OUTPUT_SOFT_ANCHOR_WINDOW_N)) {
		float anchor_n = floorf(calibrated_force_n + SENSOR_ALGO_FORCE_OUTPUT_SOFT_ANCHOR_WINDOW_N);
		float offset_n;
		float normalized_distance;
		float normalized_distance_squared;
		float scale;

		if (anchor_n < SENSOR_ALGO_FORCE_OUTPUT_SOFT_ANCHOR_MIN_N) {
			anchor_n = SENSOR_ALGO_FORCE_OUTPUT_SOFT_ANCHOR_MIN_N;
		} else if (anchor_n > SENSOR_ALGO_FORCE_OUTPUT_SOFT_ANCHOR_MAX_N) {
			anchor_n = SENSOR_ALGO_FORCE_OUTPUT_SOFT_ANCHOR_MAX_N;
		}
		offset_n = calibrated_force_n - anchor_n;
		normalized_distance = fabsf(offset_n) / SENSOR_ALGO_FORCE_OUTPUT_SOFT_ANCHOR_WINDOW_N;
		normalized_distance_squared = normalized_distance * normalized_distance;
		/* 四次恢复项增强整数点附近聚拢，窗口边界仍精确回到原值。 */
		scale = SENSOR_ALGO_FORCE_OUTPUT_SOFT_ANCHOR_CENTER_SCALE +
				((1.0f - SENSOR_ALGO_FORCE_OUTPUT_SOFT_ANCHOR_CENTER_SCALE) * normalized_distance_squared * normalized_distance_squared);
		return anchor_n + (offset_n * scale);
	}

	/* 0.5 N以下和20.5 N以上保持Flash K表的原始标定结果。 */
	return calibrated_force_n;
#else
	return calibrated_force_n;
#endif
}

#endif

static float apply_tangential_force_calibration(float raw, const sensor_force_calibration_t *calibration) {
	return apply_piecewise_force_calibration(raw, calibration->tangential_raw_points, calibration->tangential_force_points, calibration->tangential_point_count);
}

static float apply_direction_calibration(float raw_radians, const sensor_force_calibration_t *calibration) {
	float raw_degrees = raw_radians * SENSOR_ALGO_RAD_TO_DEG;
	float calibrated_degrees;

	while (raw_degrees < 0.0f) {
		raw_degrees += 360.0f;
	}
	while (raw_degrees >= 360.0f) {
		raw_degrees -= 360.0f;
	}
	calibrated_degrees = calibration->direction_output_points[0u] +
						 ((raw_degrees - calibration->direction_raw_points[0u]) * (calibration->direction_output_points[1u] - calibration->direction_output_points[0u]) /
						  (calibration->direction_raw_points[1u] - calibration->direction_raw_points[0u]));
	calibrated_degrees = fmodf(calibrated_degrees, 360.0f);
	if (calibrated_degrees < 0.0f) {
		calibrated_degrees += 360.0f;
	}
	return calibrated_degrees * SENSOR_ALGO_DEG_TO_RAD;
}

static uint32_t calculate_storage_checksum(const uint8_t *data, uint32_t size) {
	uint32_t checksum = 2166136261u;
	uint32_t index;
	for (index = 0u; index < size; index++) {
		checksum ^= data[index];
		checksum *= 16777619u;
	}
	return checksum;
}

/* 从协议载荷读取小端32位有符号整数。 */
static int32_t read_i32(const uint8_t *data) {
	uint32_t value;

	/* 显式组合字节，避免未对齐地址访问。 */
	value = (uint32_t)data[0u] |
		((uint32_t)data[1u] << 8u) |
		((uint32_t)data[2u] << 16u) |
		((uint32_t)data[3u] << 24u);

	/* 保留二进制补码位模式返回。 */
	return (int32_t)value;
}

static float read_float(const uint8_t *data) {
	union {
		float value;
		uint8_t bytes[4];
	} value;
	value.bytes[0u] = data[0u];
	value.bytes[1u] = data[1u];
	value.bytes[2u] = data[2u];
	value.bytes[3u] = data[3u];
	return value.value;
}
static void write_float_to_bytes(uint8_t *data, float value) {
	union {
		float value;
		uint8_t bytes[4];
	} converter;
	converter.value = value;
	data[0u] = converter.bytes[0u];
	data[1u] = converter.bytes[1u];
	data[2u] = converter.bytes[2u];
	data[3u] = converter.bytes[3u];
}

static void write_frame_header(uint8_t *frame, uint32_t *offset, uint8_t frame_type, uint8_t channel_count, uint16_t payload_len) {
	static const uint8_t magic[SENSOR_ALGO_PROTOCOL_MAGIC_SIZE] = {'D', 'A', 'T', 'A'};

	frame[(*offset)++] = magic[0u];
	frame[(*offset)++] = magic[1u];
	frame[(*offset)++] = magic[2u];
	frame[(*offset)++] = magic[3u];
	frame[(*offset)++] = (uint8_t)SENSOR_ALGO_PROTOCOL_VERSION;
	frame[(*offset)++] = frame_type;
	frame[(*offset)++] = (uint8_t)SENSOR_ALGO_DEVICE_TYPE;
	frame[(*offset)++] = channel_count;
	write_u16(frame, offset, payload_len);
}

static void write_u16(uint8_t *frame, uint32_t *offset, uint16_t value) {
	frame[*offset] = (uint8_t)(value & 0xFFu);
	(*offset)++;
	frame[*offset] = (uint8_t)(value >> 8u);
	(*offset)++;
}

static void write_u32(uint8_t *frame, uint32_t *offset, uint32_t value) {
	frame[*offset] = (uint8_t)(value & 0xFFu);
	(*offset)++;
	frame[*offset] = (uint8_t)((value >> 8u) & 0xFFu);
	(*offset)++;
	frame[*offset] = (uint8_t)((value >> 16u) & 0xFFu);
	(*offset)++;
	frame[*offset] = (uint8_t)((value >> 24u) & 0xFFu);
	(*offset)++;
}

static void write_float(uint8_t *frame, uint32_t *offset, float value) {
	union {
		float f;
		uint8_t bytes[SENSOR_ALGO_BYTES_PER_FLOAT];
	} converter;

	converter.f = value;
	frame[(*offset)++] = converter.bytes[0u];
	frame[(*offset)++] = converter.bytes[1u];
	frame[(*offset)++] = converter.bytes[2u];
	frame[(*offset)++] = converter.bytes[3u];
}
