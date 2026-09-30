/*******************************************************************************
 * File Name: cycfg_pins.c
 *
 * Description:
 * Pin configuration
 * This file was automatically generated and should not be modified.
 * Configurator Backend 3.80.0
 * device-db 4.35.0.9884
 * mtb-pdl-cat2 2.20.0.17814
 *
 *******************************************************************************
 * Copyright 2026, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

#include "cycfg_pins.h"

const cy_stc_gpio_pin_config_t CYBSP_CS_SHIELD1_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = CYBSP_CS_SHIELD1_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_CS_SHIELD1_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_CS_SHIELD1_PORT_NUM,
    .channel_num = CYBSP_CS_SHIELD1_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_CSD_PROX_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = CYBSP_CSD_PROX_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_CSD_PROX_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_CSD_PROX_PORT_NUM,
    .channel_num = CYBSP_CSD_PROX_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t ioss_0_port_0_pin_2_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = ioss_0_port_0_pin_2_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t ioss_0_port_0_pin_2_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = ioss_0_port_0_pin_2_PORT_NUM,
    .channel_num = ioss_0_port_0_pin_2_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_CSD_TP_COL3_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = CYBSP_CSD_TP_COL3_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_CSD_TP_COL3_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_CSD_TP_COL3_PORT_NUM,
    .channel_num = CYBSP_CSD_TP_COL3_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_CSD_TP_COL2_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = CYBSP_CSD_TP_COL2_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_CSD_TP_COL2_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_CSD_TP_COL2_PORT_NUM,
    .channel_num = CYBSP_CSD_TP_COL2_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_CSD_TP_COL1_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = CYBSP_CSD_TP_COL1_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_CSD_TP_COL1_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_CSD_TP_COL1_PORT_NUM,
    .channel_num = CYBSP_CSD_TP_COL1_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_CMOD1_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = CYBSP_CMOD1_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_CMOD1_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_CMOD1_PORT_NUM,
    .channel_num = CYBSP_CMOD1_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_CMOD2_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = CYBSP_CMOD2_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_CMOD2_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_CMOD2_PORT_NUM,
    .channel_num = CYBSP_CMOD2_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_CSD_TP_ROW3_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = CYBSP_CSD_TP_ROW3_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_CSD_TP_ROW3_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_CSD_TP_ROW3_PORT_NUM,
    .channel_num = CYBSP_CSD_TP_ROW3_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_CSD_TP_ROW2_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = CYBSP_CSD_TP_ROW2_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_CSD_TP_ROW2_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_CSD_TP_ROW2_PORT_NUM,
    .channel_num = CYBSP_CSD_TP_ROW2_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_CSD_TP_ROW1_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = CYBSP_CSD_TP_ROW1_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_CSD_TP_ROW1_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_CSD_TP_ROW1_PORT_NUM,
    .channel_num = CYBSP_CSD_TP_ROW1_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_CSD_TP_ROW0_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = CYBSP_CSD_TP_ROW0_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_CSD_TP_ROW0_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_CSD_TP_ROW0_PORT_NUM,
    .channel_num = CYBSP_CSD_TP_ROW0_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t ioss_0_port_1_pin_7_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = ioss_0_port_1_pin_7_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t ioss_0_port_1_pin_7_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = ioss_0_port_1_pin_7_PORT_NUM,
    .channel_num = ioss_0_port_1_pin_7_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_CS_HT_BTN3_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = CYBSP_CS_HT_BTN3_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_CS_HT_BTN3_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_CS_HT_BTN3_PORT_NUM,
    .channel_num = CYBSP_CS_HT_BTN3_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_CS_HT_BTN4_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = CYBSP_CS_HT_BTN4_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_CS_HT_BTN4_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_CS_HT_BTN4_PORT_NUM,
    .channel_num = CYBSP_CS_HT_BTN4_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_CS_HT_BTN2_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = CYBSP_CS_HT_BTN2_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_CS_HT_BTN2_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_CS_HT_BTN2_PORT_NUM,
    .channel_num = CYBSP_CS_HT_BTN2_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_CS_HT_BTN1_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = CYBSP_CS_HT_BTN1_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_CS_HT_BTN1_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_CS_HT_BTN1_PORT_NUM,
    .channel_num = CYBSP_CS_HT_BTN1_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t ioss_0_port_2_pin_4_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = ioss_0_port_2_pin_4_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t ioss_0_port_2_pin_4_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = ioss_0_port_2_pin_4_PORT_NUM,
    .channel_num = ioss_0_port_2_pin_4_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_ALS_ADC_IN_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = CYBSP_ALS_ADC_IN_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_ALS_ADC_IN_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_ALS_ADC_IN_PORT_NUM,
    .channel_num = CYBSP_ALS_ADC_IN_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_CS_HT_LED3_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom = CYBSP_CS_HT_LED3_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_CS_HT_LED3_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_CS_HT_LED3_PORT_NUM,
    .channel_num = CYBSP_CS_HT_LED3_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_SWDIO_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_STRONG,
    .hsiom = CYBSP_SWDIO_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_SWDIO_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_SWDIO_PORT_NUM,
    .channel_num = CYBSP_SWDIO_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_SWDCK_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_STRONG,
    .hsiom = CYBSP_SWDCK_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_SWDCK_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_SWDCK_PORT_NUM,
    .channel_num = CYBSP_SWDCK_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_DEBUG_UART_TX_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_HIGHZ,
    .hsiom = CYBSP_DEBUG_UART_TX_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_DEBUG_UART_TX_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_DEBUG_UART_TX_PORT_NUM,
    .channel_num = CYBSP_DEBUG_UART_TX_PIN,
};
#endif /* defined (CY_USING_HAL) */

const cy_stc_gpio_pin_config_t CYBSP_DEBUG_UART_RX_config =
{
    .outVal = 1,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom = CYBSP_DEBUG_UART_RX_HSIOM,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
};

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t CYBSP_DEBUG_UART_RX_obj =
{
    .type = CYHAL_RSC_GPIO,
    .block_num = CYBSP_DEBUG_UART_RX_PORT_NUM,
    .channel_num = CYBSP_DEBUG_UART_RX_PIN,
};
#endif /* defined (CY_USING_HAL) */

void init_cycfg_pins(void)
{
    Cy_GPIO_Pin_Init(CYBSP_CSD_PROX_PORT, CYBSP_CSD_PROX_PIN, &CYBSP_CSD_PROX_config);
    Cy_GPIO_Pin_Init(CYBSP_CSD_TP_COL2_PORT, CYBSP_CSD_TP_COL2_PIN, &CYBSP_CSD_TP_COL2_config);
    Cy_GPIO_Pin_Init(CYBSP_CSD_TP_ROW3_PORT, CYBSP_CSD_TP_ROW3_PIN, &CYBSP_CSD_TP_ROW3_config);
    Cy_GPIO_Pin_Init(CYBSP_CSD_TP_ROW1_PORT, CYBSP_CSD_TP_ROW1_PIN, &CYBSP_CSD_TP_ROW1_config);
    Cy_GPIO_Pin_Init(CYBSP_CSD_TP_ROW0_PORT, CYBSP_CSD_TP_ROW0_PIN, &CYBSP_CSD_TP_ROW0_config);
    Cy_GPIO_Pin_Init(CYBSP_CS_HT_BTN3_PORT, CYBSP_CS_HT_BTN3_PIN, &CYBSP_CS_HT_BTN3_config);
    Cy_GPIO_Pin_Init(CYBSP_CS_HT_BTN2_PORT, CYBSP_CS_HT_BTN2_PIN, &CYBSP_CS_HT_BTN2_config);
    Cy_GPIO_Pin_Init(CYBSP_CS_HT_BTN1_PORT, CYBSP_CS_HT_BTN1_PIN, &CYBSP_CS_HT_BTN1_config);
    Cy_GPIO_Pin_Init(CYBSP_SWDIO_PORT, CYBSP_SWDIO_PIN, &CYBSP_SWDIO_config);
    Cy_GPIO_Pin_Init(CYBSP_SWDCK_PORT, CYBSP_SWDCK_PIN, &CYBSP_SWDCK_config);
    Cy_GPIO_Pin_Init(CYBSP_DEBUG_UART_TX_PORT, CYBSP_DEBUG_UART_TX_PIN, &CYBSP_DEBUG_UART_TX_config);
    Cy_GPIO_Pin_Init(CYBSP_DEBUG_UART_RX_PORT, CYBSP_DEBUG_UART_RX_PIN, &CYBSP_DEBUG_UART_RX_config);
}
void reserve_cycfg_pins(void)
{
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&CYBSP_CS_SHIELD1_obj);
    cyhal_hwmgr_reserve(&CYBSP_CSD_PROX_obj);
    cyhal_hwmgr_reserve(&ioss_0_port_0_pin_2_obj);
    cyhal_hwmgr_reserve(&CYBSP_CSD_TP_COL3_obj);
    cyhal_hwmgr_reserve(&CYBSP_CSD_TP_COL2_obj);
    cyhal_hwmgr_reserve(&CYBSP_CSD_TP_COL1_obj);
    cyhal_hwmgr_reserve(&CYBSP_CMOD1_obj);
    cyhal_hwmgr_reserve(&CYBSP_CMOD2_obj);
    cyhal_hwmgr_reserve(&CYBSP_CSD_TP_ROW3_obj);
    cyhal_hwmgr_reserve(&CYBSP_CSD_TP_ROW2_obj);
    cyhal_hwmgr_reserve(&CYBSP_CSD_TP_ROW1_obj);
    cyhal_hwmgr_reserve(&CYBSP_CSD_TP_ROW0_obj);
    cyhal_hwmgr_reserve(&ioss_0_port_1_pin_7_obj);
    cyhal_hwmgr_reserve(&CYBSP_CS_HT_BTN3_obj);
    cyhal_hwmgr_reserve(&CYBSP_CS_HT_BTN4_obj);
    cyhal_hwmgr_reserve(&CYBSP_CS_HT_BTN2_obj);
    cyhal_hwmgr_reserve(&CYBSP_CS_HT_BTN1_obj);
    cyhal_hwmgr_reserve(&ioss_0_port_2_pin_4_obj);
    cyhal_hwmgr_reserve(&CYBSP_ALS_ADC_IN_obj);
    cyhal_hwmgr_reserve(&CYBSP_CS_HT_LED3_obj);
    cyhal_hwmgr_reserve(&CYBSP_SWDIO_obj);
    cyhal_hwmgr_reserve(&CYBSP_SWDCK_obj);
    cyhal_hwmgr_reserve(&CYBSP_DEBUG_UART_TX_obj);
    cyhal_hwmgr_reserve(&CYBSP_DEBUG_UART_RX_obj);
#endif /* defined (CY_USING_HAL) */
}
