/*******************************************************************************
 * File Name: cycfg_clocks.c
 *
 * Description:
 * Peripheral clock divider configuration
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

#include "cycfg_clocks.h"

#if defined (CY_USING_HAL)
const cyhal_resource_inst_t peri_0_div_16_1_obj =
{
    .type = CYHAL_RSC_CLOCK,
    .block_num = peri_0_div_16_1_HW,
    .channel_num = peri_0_div_16_1_NUM,
};
const cyhal_resource_inst_t peri_0_div_16_2_obj =
{
    .type = CYHAL_RSC_CLOCK,
    .block_num = peri_0_div_16_2_HW,
    .channel_num = peri_0_div_16_2_NUM,
};
const cyhal_resource_inst_t peri_0_div_16_3_obj =
{
    .type = CYHAL_RSC_CLOCK,
    .block_num = peri_0_div_16_3_HW,
    .channel_num = peri_0_div_16_3_NUM,
};
#endif /* defined (CY_USING_HAL) */

void init_cycfg_clocks(void)
{
    Cy_SysClk_PeriphDisableDivider(CY_SYSCLK_DIV_16_BIT, 1U);
    Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 1U, 5U);
    Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, 1U);
    Cy_SysClk_PeriphDisableDivider(CY_SYSCLK_DIV_16_BIT, 2U);
    Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 2U, 51U);
    Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, 2U);
    Cy_SysClk_PeriphDisableDivider(CY_SYSCLK_DIV_16_BIT, 3U);
    Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 3U, 51U);
    Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, 3U);
}
void reserve_cycfg_clocks(void)
{
#if defined (CY_USING_HAL)
    cyhal_hwmgr_reserve(&peri_0_div_16_1_obj);
    cyhal_hwmgr_reserve(&peri_0_div_16_2_obj);
    cyhal_hwmgr_reserve(&peri_0_div_16_3_obj);
#endif /* defined (CY_USING_HAL) */
}
