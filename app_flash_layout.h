/* 应用参数区统一规划；Bootloader元数据及其后所有行永久排除。 */
#ifndef APP_FLASH_LAYOUT_H
#define APP_FLASH_LAYOUT_H

/* 应用镜像和参数区的分界，同时用于链接与升级镜像检查。 */
#define APP_FLASH_PARAMETER_START (0x0001F500u)
/* 四点手动低力标定独占一行，不擦写已有两区K表。 */
#define APP_FLASH_LOW_FORCE_ADDRESS (0x0001F500u)
/* 此地址及之后的全部Flash只归Bootloader/DFU管理。 */
#define APP_FLASH_PROTECTED_START (0x0001FD00u)
/* 当前芯片每次擦写一整行256字节。 */
#define APP_FLASH_ROW_BYTES (0x100u)
/* 两个受力区域各占三行，容纳48节点法向表及原20节点切向表。 */
#define APP_FLASH_FORCE_ADDRESS (0x0001F600u)
#define APP_FLASH_FORCE_REGION_BYTES (0x300u)
/* 仿射功能停用，仍在保护区之前预留独立一行，禁止沿用旧尾部地址。 */
#define APP_FLASH_AFFINE_ADDRESS (0x0001FC00u)

/* 地址必须按整行对齐，两个力区域不得与仿射区或DFU保护区相交。 */
#if ((APP_FLASH_PARAMETER_START % APP_FLASH_ROW_BYTES) != 0u) || \
    ((APP_FLASH_AFFINE_ADDRESS % APP_FLASH_ROW_BYTES) != 0u) || \
    (APP_FLASH_LOW_FORCE_ADDRESS != APP_FLASH_PARAMETER_START) || \
    ((APP_FLASH_LOW_FORCE_ADDRESS + APP_FLASH_ROW_BYTES) > APP_FLASH_FORCE_ADDRESS) || \
    (APP_FLASH_FORCE_ADDRESS < APP_FLASH_PARAMETER_START) || \
    ((APP_FLASH_FORCE_ADDRESS + 2u * APP_FLASH_FORCE_REGION_BYTES) > APP_FLASH_AFFINE_ADDRESS) || \
    ((APP_FLASH_AFFINE_ADDRESS + APP_FLASH_ROW_BYTES) > APP_FLASH_PROTECTED_START)
#error "Application Flash layout overlaps a protected region"
#endif
#endif
