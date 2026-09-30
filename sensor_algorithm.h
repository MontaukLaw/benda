#ifndef SENSOR_ALGORITHM_H
#define SENSOR_ALGORITHM_H

/*
 * 固定宽度整数类型。
 * 这里所有串口输出和内部状态位宽都希望明确，不依赖编译器默认 int 宽度。
 */
#include <stdbool.h>
#include <stdint.h>

/* 引入编译时选择的手指类型、通道数和算法时基。 */
#include "device_variant.h"

/* 当前固件通道数由手指变体统一配置，并与CapSense生成配置交叉检查。 */
#define SENSOR_ALGO_CHANNEL_COUNT DEVICE_SENSOR_CHANNEL_COUNT

/* 所有应用算法暂按每通道噪声1 count计算；校准不再改变阈值。 */
#define SENSOR_ALGO_FIXED_NOISE (1u)

/*
 * 定义 UART 算法协议版本。
 * V14增加设备信息查询和设备信息响应，用于上位机自动识别手指类型。
 * V15增加逐通道CapSense配置值、硬件值和Max Raw回读。
 * 数据发送端与命令接收端必须共同使用这个版本号。
 */
/* V16启用48节点两段法向K表以及DFU尾部保护布局。 */
/* V19增加最低检测线、模拟标志及独立的实际测量值。 */
/* V20删除重复Raw、固定噪声遥测与独立慢基线帧。 */
/* V21兼容1至30 N自动标定的10/20节点法向表，保留V20数据帧布局。 */
#define SENSOR_ALGO_PROTOCOL_VERSION (21u)

/* 0.1至2 N每0.1 N一点，3至30 N每1 N一点，共48个实体节点。 */
#define SENSOR_ALGO_FORCE_CALIBRATION_POINT_COUNT (48u)
/* 0x84参数头8字节，每个标定节点包含两个float32。 */
#define SENSOR_ALGO_FORCE_CALIBRATION_COMMAND_MAX_PAYLOAD_SIZE \
	(8u + (SENSOR_ALGO_FORCE_CALIBRATION_POINT_COUNT * 8u))

/*
 * 控制可选的切向力与方向算法；四通道法向Signal合力及Flash K表换算不受此开关影响。
 * 当前切向力旧模块尚未完成与新通道状态机的接口迁移，因此保持关闭。
 */
#ifndef SENSOR_ALGO_FORCE_VECTOR_OUTPUT
#define SENSOR_ALGO_FORCE_VECTOR_OUTPUT (0u)
#endif

/*
 * 合力由上位机按照勾选通道实时计算，固件不再发送固定通道和值帧。
 */
#ifndef SENSOR_ALGO_CHANNEL_4567_SIGNAL_SUM_OUTPUT
#define SENSOR_ALGO_CHANNEL_4567_SIGNAL_SUM_OUTPUT (0u)
#endif

/*
 * activate 判断阈值。
 * signal > noise 时认为该通道处于 activate。
 */
#define SENSOR_ALGO_NOISE_MULTIPLIER (1u)

/*
 * activate 判断阈值。
 * signal > 该值 activate。
 */
#define SENSOR_ACTIVE_THREADHOLD (10u)

/*
 * 输入侧噪声门。
 * 预先扣除 2 倍 noise，抑制静置时的通道误输出。
 */
#define SENSOR_ALGO_SIGNAL_INPUT_GATE_MULTIPLIER (2u)

/*
 * 三维力组接触提示预先扣除 2 倍 noise，抑制静置误触发。
 */
#define SENSOR_ALGO_FORCE_VECTOR_CONTACT_GATE_MULTIPLIER (2u)

/*
 * 重压松手后的动态噪声底。
 * 本次触摸峰值越大，释放后的暂态抑制也越大，用来压掉与峰值相关的回弹尾巴。
 */
#define SENSOR_ALGO_RELEASE_DYNAMIC_GATE_DIVISOR (3u)
#define SENSOR_ALGO_RELEASE_DYNAMIC_GATE_DECAY_SHIFT (7u)

/*
 * 弱 activate 自动重校准上限。
 * signal 在 noise * 4 到 noise * 20 之间，并持续足够久，触发重新 calibration。
 */
#define SENSOR_ALGO_AUTO_CAL_MULTIPLIER (200u)

/*
 * 算法按帧计时。
 * 如果实际帧率长期变化较大，需要先改这里，否则 2 秒/10 秒的帧数会不准。
 */
/* 矩阵时基由开机或三参数变更后的实测周期确定，其他变体沿用固定时基。 */
#if DEVICE_MATRIX20
uint32_t ChannelState_GetSampleRateHz(void);
void ChannelState_SetSampleRateHz(uint32_t sample_rate_hz);
#define SENSOR_ALGO_FRAME_RATE_HZ ChannelState_GetSampleRateHz()
#else
#define SENSOR_ALGO_FRAME_RATE_HZ DEVICE_ALGORITHM_FRAME_RATE_HZ
#endif

/* 将毫秒时长向上换算为至少一帧的算法帧数。 */
#define SENSOR_ALGO_MILLISECONDS_TO_FRAMES(milliseconds) \
	(((SENSOR_ALGO_FRAME_RATE_HZ * (milliseconds)) + 999u) / 1000u)

/* 将整数秒时长换算为算法帧数。 */
#define SENSOR_ALGO_SECONDS_TO_FRAMES(seconds) \
	(SENSOR_ALGO_FRAME_RATE_HZ * (seconds))

/*
 * 上电初期不发送周期数据，并让 baseline 逐帧贴合当前 raw。
 * 同一窗口用于完成首次 baseline 校准；noise固定为1，窗口结束后开始有效输出。
 * 该窗口只在 SensorAlgorithm_Init() 后执行一次，主动重校准不会重复静默。
 */
#define SENSOR_ALGO_STARTUP_QUIET_FRAMES (300u)

#define SENSOR_ALGO_OUTPUT_MODE_RUN (0u)
#define SENSOR_ALGO_OUTPUT_MODE_DEBUG (1u)
#define SENSOR_ALGO_OUTPUT_MODE_FORCE_CALIBRATION (2u)

/*
 * 每次 calibration 的采样时长。
 * 这些样本用于更新各通道baseline，不再估算noise/stddev。
 */
#define SENSOR_ALGO_CALIBRATION_SECONDS (2u)

/*
 * 弱 activate 连续达到这个时间后，认为 baseline 可能卡住，自动重新 calibration。
 */
#define SENSOR_ALGO_ACTIVE_RECAL_SECONDS (10u)

/*
 * calibration 阶段 baseline 的较快跟踪系数。
 * Q16 下 3277 / 65536 约等于 0.05。
 */
#define SENSOR_ALGO_CALIBRATION_ALPHA_Q16 (3277u)

/*
 * 最终输出 signal 的低通系数。
 * 低 raw 硬件下提高上升跟随速度，让轻抚 signal 更快脱离 0。
 */
#define SENSOR_ALGO_SIGNAL_RISE_ALPHA_Q16 (32768u)
#define SENSOR_ALGO_SIGNAL_FALL_ALPHA_Q16 (24576u)
#define SENSOR_ALGO_SIGNAL_FAST_RELEASE_ALPHA_Q16 (32768u)
#define SENSOR_ALGO_FAST_RELEASE_BASELINE_ALPHA_Q16 (6554u)
#define SENSOR_ALGO_RELEASE_MULTIPLIER (1u)
// 轻抚稳定性：提高 fast_release 触发门槛，防止轻触时的信号波动被误判为释放
#define SENSOR_ALGO_FAST_RELEASE_START_MULTIPLIER (52u)
#define SENSOR_ALGO_RELEASE_DEBOUNCE_FRAMES (6u)
#define SENSOR_ALGO_FAST_RELEASE_DEBOUNCE_FRAMES (10u)
#define SENSOR_ALGO_RELEASE_RECOVERY_FRAMES (300u)
#define SENSOR_ALGO_RELEASE_RECOVERY_MIN_FRAMES (90u)
#define SENSOR_ALGO_RELEASE_RECOVERY_STABLE_FRAMES (12u)
#define SENSOR_ALGO_RELEASE_RECOVERY_ALPHA_Q16 (1311u)

/*
 * 结构形变可能先使 raw 低于 baseline，松手回弹时才越过 baseline。
 * 该回弹不是有效按压：只有连续观察到足够大的负向行程后，才进入
 * 既有的松手恢复窗口，避免把单帧噪声或正常轻触误判为形变回弹。
 */
#define SENSOR_ALGO_REBOUND_NEGATIVE_MULTIPLIER (4u)
#define SENSOR_ALGO_REBOUND_NEGATIVE_FRAMES (3u)

/*
 * 4 通道组确认进入下降阶段后，最多允许残留的时间。
 * 指尖和指腹分别计时，超时只清除对应组。
 */
#define SENSOR_ALGO_RELEASE_TIMEOUT_SECONDS (1u)

/*
 * 1: When fast-release is triggered, snap baseline to the current raw position.
 * 0: Keep baseline unchanged during fast-release.
 */
#ifndef SENSOR_ALGO_FAST_RELEASE_SNAP_BASELINE
#define SENSOR_ALGO_FAST_RELEASE_SNAP_BASELINE (0u)
#endif

/*
 * 松手后 raw 会回到本次峰值行程的约 3%~13%，因此只在 raw 已回到
 * baseline + peak_diff / 5 以内，且仍连续下降时，才认为正在松手。
 * 这个峰值比例条件防止大力降到小力时 baseline 追上 raw 并吃掉小力。
 */
#define SENSOR_ALGO_RELEASE_SLOPE_FRAMES (10u)
#define SENSOR_ALGO_RELEASE_RAW_PEAK_DIVISOR (5u)
#define SENSOR_ALGO_POST_ACTIVE_FAST_BASELINE_FRAMES (80u)
#define SENSOR_ALGO_POST_ACTIVE_BASELINE_DELAY_FRAMES (80u)
#define SENSOR_ALGO_POST_ACTIVE_BASELINE_ALPHA_Q16 (1311u)

/*
 * Q16 固定点中的 1.0。
 * alpha 都以这个数作为分母。
 */
#define SENSOR_ALGO_Q16_ONE (65536u)

#define SENSOR_ALGO_STATE_DEBOUNCE_FRAMES (3u)
#define SENSOR_ALGO_MIN_SLOPE_THRESHOLD (2u)

typedef enum {
	/* 通道未检测到有效按压。 */
	SENSOR_ALGO_STATE_IDLE = 0u,
	/* 通道压力正在上升。 */
	SENSOR_ALGO_STATE_ACTIVE_RISING = 1u,
	/* 通道保持有效按压且变化趋于稳定。 */
	SENSOR_ALGO_STATE_ACTIVE_HOLD = 2u,
	/* 通道压力正在下降但尚未确认松手。 */
	SENSOR_ALGO_STATE_ACTIVE_FALLING = 3u,
	/* 已确认松手，正在屏蔽传感器材料的缓慢回弹。 */
	SENSOR_ALGO_STATE_SLOW_BACK = 4u,
	/* 慢回弹超时且残余偏移无法自然消除，正在重建零点。 */
	SENSOR_ALGO_STATE_ZERO_REBASE = 5u,
	/* 稳定按压持续足够久后锁定输出，并吸收材料缓慢蠕变。 */
	SENSOR_ALGO_STATE_ACTIVE_FLOATING = 6u,
	/* 稳定等待完成后，基线以0.02恢复，完成后返回IDLE。 */
	SENSOR_ALGO_STATE_RECOVER = 7u,
	/* 已确认按压结束后冻结基线并等待Raw稳定，同时允许再次按压。 */
	SENSOR_ALGO_STATE_WAIT_STABLE = 8u,
} sensor_algorithm_state_t;

/*
 * 单个通道的一帧算法输出。
 * main.c 会把 8 个通道的 raw/baseline/signal 打包到同一个串口包。
 */
typedef struct {
	/*
	 * 原始 raw count。
	 * 直接来自 Capsense sensor_context->raw。
	 */
	uint16_t raw;

	/*
	 * 当前 baseline。
	 * 内部以 Q16 保存，输出时转换回 uint16_t。
	 */
	uint16_t baseline;

	/*
	 * 最终信号。
	 * 计算方式是 max(0, raw - baseline)，再经过低通滤波。
	 */
	uint16_t signal;

	/*
	 * 激活持续一秒后计算的 raw 线性回归斜率。
	 * 使用 Q16 格式，单位为 counts/s；未满足计算条件时为零。
	 */
	int32_t raw_k_q16;

	/*
	 * 激活持续一秒后计算的 signal 线性回归斜率。
	 * 使用 Q16 格式，单位为 counts/s；未满足计算条件时为零。
	 */
	int32_t signal_k_q16;

	/* 100 ms raw 窗口的归一化斜率，单位为 0.01 %/s。 */
	int32_t fast_normalized_k_x100;

	/* 1 s raw 窗口的归一化斜率，单位为 0.01 %/s。 */
	int32_t slow_normalized_k_x100;

	/*
	 * 算法使用的固定噪声值，初始化及重新校准后均为1，并非实测标准差。
	 */
	uint16_t noise;
} sensor_algorithm_output_t;

typedef struct {
	sensor_algorithm_output_t channels[SENSOR_ALGO_CHANNEL_COUNT];
	sensor_algorithm_state_t states[SENSOR_ALGO_CHANNEL_COUNT];
} sensor_algorithm_snapshot_t;

/*
 * 初始化算法内部状态。
 * 开机后、开始第一次扫描前调用一次。
 */
void SensorAlgorithm_Init(void);

/*
 * 处理一帧 8 通道 raw 数据。
 * 输出同一帧的 raw、baseline 和 signal。
 */
void SensorAlgorithm_Process(
	const uint16_t raw_values[SENSOR_ALGO_CHANNEL_COUNT],
	sensor_algorithm_output_t outputs[SENSOR_ALGO_CHANNEL_COUNT]);

void SensorAlgorithm_GetStates(
	sensor_algorithm_state_t states[SENSOR_ALGO_CHANNEL_COUNT]);

/* 判断指定通道状态是否代表真实接触仍然有效。 */
bool SensorAlgorithm_IsContactActiveState(sensor_algorithm_state_t state);

void SensorAlgorithm_GetLatestSnapshot(sensor_algorithm_snapshot_t *snapshot);

void SensorAlgorithm_RequestRecalibration(void);
void SensorAlgorithm_RestartBaselineInitialization(void);
void SensorAlgorithm_UartTxPoll(void);
/* 通过Type 0x03上传0.1摄氏度单位的芯片内部温度。 */
void SensorAlgorithm_SendTemperatureFrame(int16_t temperature_tenth_celsius);

void SensorAlgorithm_HandleProtocolCommand(uint8_t command_type, const uint8_t *payload, uint16_t payload_len);
void SensorAlgorithm_SendCommandAck(uint8_t result);
void SensorAlgorithm_GetCalibratedForces(float *normal_force, float *tangential_force, float *direction);
void SensorAlgorithm_GetCalibratedForcesForRegion(uint8_t region, float *normal_force, float *tangential_force, float *direction);
void SensorAlgorithm_GetCalibratedForceVector(float *force_x, float *force_y, float *force_z);

/* 直接处理硬件Raw，V20不再维护归一化Raw副本。 */
void SensorAlgorithm_ProcessAndSend(
	const uint16_t raw_values[SENSOR_ALGO_CHANNEL_COUNT]);

#endif
