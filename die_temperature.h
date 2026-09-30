#ifndef DIE_TEMPERATURE_H
#define DIE_TEMPERATURE_H

#include <stdbool.h>
#include <stdint.h>

/* 初始化芯片内部温度传感器和独立SAR注入通道。 */
bool DieTemperature_Init(void);
/* 非阻塞启动一次芯片温度转换。 */
bool DieTemperature_StartMeasurement(void);
/* 非阻塞轮询转换结果，成功时返回0.1摄氏度单位的温度。 */
bool DieTemperature_TryReadTenthDegreeC(int16_t *temperature_tenth_celsius);
/* 返回最近一次成功转换的0.1摄氏度温度。 */
int16_t DieTemperature_GetLatestTenthDegreeC(void);

#endif
