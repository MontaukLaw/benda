/* 应用所有参数写入必须经过本接口，禁止直接调用底层Flash擦写API。 */
#ifndef APP_FLASH_H
#define APP_FLASH_H
/* 引入PDL写入状态与统一地址规划。 */
#include "cy_pdl.h"
#include "app_flash_layout.h"
/* 仅允许在应用参数区内写入对齐的一整行，地址非法时不触碰硬件。 */
cy_en_flashdrv_status_t AppFlash_WriteRow(uint32_t address, const uint32_t *data);
#endif
