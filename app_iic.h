#ifndef APP_IIC_H
#define APP_IIC_H

#include <stdint.h>

#define APP_IIC_DEFAULT_ADDRESS (0x6Fu)
#define APP_IIC_COMMAND_FRAME_SIZE (11u)
#define APP_IIC_ACK_FRAME_SIZE (11u)
#define APP_IIC_DATA_FRAME_SIZE (45u)
#define APP_IIC_SPECIAL_A0_FRAME_SIZE (26u)
#define APP_IIC_SPECIAL_A2_FRAME_SIZE (50u)

void AppIic_Init(void);
void AppIic_Poll(void);
void AppIic_OnScanComplete(void);

#endif
