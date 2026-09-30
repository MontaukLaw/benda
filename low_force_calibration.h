/* 四点手动低力标定：独立存储、启动加载及分段换算。 */
#ifndef LOW_FORCE_CALIBRATION_H
#define LOW_FORCE_CALIBRATION_H
#include <stdbool.h>
#include <stdint.h>

/* 依次对应10、20、50、100 g，输入单位为四通道Signal和counts。 */
#define LOW_FORCE_POINT_COUNT (4u)
/* 用户确认检测线降至0.09 N，使10 g节点可正常输出。 */
#define LOW_FORCE_DETECTION_N (0.09f)
/* 结果状态区分未标定、有效测量和超出低力量程。 */
#define LOW_FORCE_INVALID (0u)
#define LOW_FORCE_VALID (1u)
#define LOW_FORCE_OVER_RANGE (2u)

/* 从Flash加载到RAM，损坏或未保存时清除有效标志。 */
bool LowForce_Load(void);
/* 校验并保存，整行回读成功后才替换RAM参数；返回协议结果码。 */
uint8_t LowForce_Save(const float points[LOW_FORCE_POINT_COUNT], float detection_counts);
/* 提供RAM参数、有效状态和Flash记录状态供上位机核对。 */
bool LowForce_IsValid(void);
bool LowForce_FlashValid(void);
void LowForce_GetPoints(float points[LOW_FORCE_POINT_COUNT]);
/* 最低检测线使用Signal和counts；0关闭模拟，非零值必须低于10 g标定点。 */
float LowForce_GetDetectionCounts(void);
/* 保留检测线截断前的实际标定换算值，包括超量程结果，供诊断记录。 */
uint8_t LowForce_Measure(float signal_sum, float *force_n);
/* 超过可配置检测线且低于10 g时持续生成模拟值，不依赖通道状态。 */
bool LowForce_Simulate(float signal_sum, float *optimized_n);
/* 固件统一执行质量换算、分段插值、0.09 N检测线和1 N量程限制。 */
uint8_t LowForce_Evaluate(float signal_sum, float *force_n);
/* 对已通过原始值检测线的力值做连续单调软吸附，不修改原始值。 */
float LowForce_Optimize(float force_n);
#endif
