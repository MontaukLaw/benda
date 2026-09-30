/* 应用参数统一擦写入口。 */
#include "app_flash.h"

/* 确认芯片真实行大小与布局一致，避免换芯片后扩大擦除范围。 */
#if CY_FLASH_SIZEOF_ROW != APP_FLASH_ROW_BYTES
#error "Flash row size differs from application storage layout"
#endif

/* 先验证整行边界，再调用会先擦后写的PDL接口。 */
cy_en_flashdrv_status_t AppFlash_WriteRow(uint32_t address, const uint32_t *data) {
	/* 使用减法比较上界，避免传入大地址时加法溢出绕过保护。 */
	if ((data == NULL) || (address < APP_FLASH_PARAMETER_START) ||
		(address > (APP_FLASH_PROTECTED_START - APP_FLASH_ROW_BYTES)) ||
		((address % APP_FLASH_ROW_BYTES) != 0u)) {
		return CY_FLASH_DRV_INVALID_FLASH_ADDR;
	}
	/* 只有0x1F500至0x1FC00的完整参数行可到达底层写入。 */
	return Cy_Flash_WriteRow(address, data);
}
