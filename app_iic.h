#ifndef APP_IIC_H
#define APP_IIC_H

#include <stdint.h>
/* 使用当前变体通道数计算数据帧大小，防止20通道写入五通道缓冲区。 */
#include "device_variant.h"

#define APP_IIC_DEFAULT_ADDRESS (0x6Fu)
#define APP_IIC_COMMAND_FRAME_SIZE (11u)
#define APP_IIC_ACK_FRAME_SIZE (11u)
/* 固定头尾及力/邻近槽共30字节，每通道占3字节；五通道45字节，二十通道90字节。 */
#define APP_IIC_DATA_FRAME_SIZE (30u + (DEVICE_SENSOR_CHANNEL_COUNT * 3u))
#define APP_IIC_SPECIAL_A0_FRAME_SIZE (26u)
#define APP_IIC_SPECIAL_A2_FRAME_SIZE (50u)

void AppIic_Init(void);
void AppIic_Poll(void);
void AppIic_OnScanComplete(void);

#endif
