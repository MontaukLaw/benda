/*******************************************************************************
 * File Name: cycfg_peripherals.h
 *
 * Description:
 * Clock configuration
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

#if !defined(CYCFG_PERIPHERALS_H)
#define CYCFG_PERIPHERALS_H

#include "cycfg_notices.h"
#include "cy_scb_uart.h"
#include "cy_sysclk.h"
#include "cy_msclp.h"

#if defined (CY_USING_HAL)
#include "cyhal_hwmgr.h"
#endif /* defined (CY_USING_HAL) */

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#define CYBSP_UART_ENABLED 1U
#define CYBSP_UART_HW SCB0
#define CYBSP_UART_IRQ scb_0_interrupt_IRQn
#define CYBSP_MSC_ENABLED 1U
#define CY_MSCLP0_Cmod1_PORT GPIO_PRT1
#define CY_MSCLP0_Cmod2_PORT GPIO_PRT1
#define CY_MSCLP0_Cmod1_PIN 0u
#define CY_MSCLP0_Cmod2_PIN 1u
#define CY_MSCLP0_Cmod1_PORT_NUM 1u
#define CY_MSCLP0_Cmod2_PORT_NUM 1u
#define Button0_Rx0_PORT GPIO_PRT2
#define Button0_Tx_PORT GPIO_PRT3
#define Button1_Rx0_PORT GPIO_PRT1
#define Button1_Tx_PORT GPIO_PRT1
#define Button2_Rx0_PORT GPIO_PRT2
#define Button2_Tx_PORT GPIO_PRT2
#define Button3_Rx0_PORT GPIO_PRT0
#define Button3_Tx_PORT GPIO_PRT0
#define Button4_Rx0_PORT GPIO_PRT0
#define Button4_Tx_PORT GPIO_PRT0
#define Button0_Rx0_PIN 7u
#define Button0_Tx_PIN 1u
#define Button1_Rx0_PIN 4u
#define Button1_Tx_PIN 7u
#define Button2_Rx0_PIN 1u
#define Button2_Tx_PIN 4u
#define Button3_Rx0_PIN 7u
#define Button3_Tx_PIN 5u
#define Button4_Rx0_PIN 0u
#define Button4_Tx_PIN 2u
#define Button0_Rx0_PAD P2_7_MSCLP_MSC_GPIO_CTRL_SNS
#define Button0_Tx_PAD P3_1_MSCLP_MSC_GPIO_CTRL_SNS
#define Button1_Rx0_PAD P1_4_MSCLP_MSC_GPIO_CTRL_SNS
#define Button1_Tx_PAD P1_7_MSCLP_MSC_GPIO_CTRL_SNS
#define Button2_Rx0_PAD P2_1_MSCLP_MSC_GPIO_CTRL_SNS
#define Button2_Tx_PAD P2_4_MSCLP_MSC_GPIO_CTRL_SNS
#define Button3_Rx0_PAD P0_7_MSCLP_MSC_GPIO_CTRL_SNS
#define Button3_Tx_PAD P0_5_MSCLP_MSC_GPIO_CTRL_SNS
#define Button4_Rx0_PAD P0_0_MSCLP_MSC_GPIO_CTRL_SNS
#define Button4_Tx_PAD P0_2_MSCLP_MSC_GPIO_CTRL_SNS
#define Button0_Rx0_ELTD0_PORT GPIO_PRT2
#define Button0_Tx_ELTD0_PORT GPIO_PRT3
#define Button1_Rx0_ELTD0_PORT GPIO_PRT1
#define Button1_Tx_ELTD0_PORT GPIO_PRT1
#define Button2_Rx0_ELTD0_PORT GPIO_PRT2
#define Button2_Tx_ELTD0_PORT GPIO_PRT2
#define Button3_Rx0_ELTD0_PORT GPIO_PRT0
#define Button3_Tx_ELTD0_PORT GPIO_PRT0
#define Button4_Rx0_ELTD0_PORT GPIO_PRT0
#define Button4_Tx_ELTD0_PORT GPIO_PRT0
#define Button0_Rx0_ELTD0_PIN 7u
#define Button0_Tx_ELTD0_PIN 1u
#define Button1_Rx0_ELTD0_PIN 4u
#define Button1_Tx_ELTD0_PIN 7u
#define Button2_Rx0_ELTD0_PIN 1u
#define Button2_Tx_ELTD0_PIN 4u
#define Button3_Rx0_ELTD0_PIN 7u
#define Button3_Tx_ELTD0_PIN 5u
#define Button4_Rx0_ELTD0_PIN 0u
#define Button4_Tx_ELTD0_PIN 2u
#define Button0_Rx0_ELTD0_PAD P2_7_MSCLP_MSC_GPIO_CTRL_SNS
#define Button0_Tx_ELTD0_PAD P3_1_MSCLP_MSC_GPIO_CTRL_SNS
#define Button1_Rx0_ELTD0_PAD P1_4_MSCLP_MSC_GPIO_CTRL_SNS
#define Button1_Tx_ELTD0_PAD P1_7_MSCLP_MSC_GPIO_CTRL_SNS
#define Button2_Rx0_ELTD0_PAD P2_1_MSCLP_MSC_GPIO_CTRL_SNS
#define Button2_Tx_ELTD0_PAD P2_4_MSCLP_MSC_GPIO_CTRL_SNS
#define Button3_Rx0_ELTD0_PAD P0_7_MSCLP_MSC_GPIO_CTRL_SNS
#define Button3_Tx_ELTD0_PAD P0_5_MSCLP_MSC_GPIO_CTRL_SNS
#define Button4_Rx0_ELTD0_PAD P0_0_MSCLP_MSC_GPIO_CTRL_SNS
#define Button4_Tx_ELTD0_PAD P0_2_MSCLP_MSC_GPIO_CTRL_SNS
#define CY_MSCLP0_HW MSCLP0
#define CYBSP_MSC_HW MSCLP0
#define CY_MSCLP0_IRQ msclp_interrupt_IRQn
#define CYBSP_MSC_IRQ msclp_interrupt_IRQn
#define CY_MSCLP0_LP_IRQ msclp_interrupt_lp_IRQn
#define CYBSP_MSC_LP_IRQ msclp_interrupt_lp_IRQn
#define CYBSP_CAPSENSE_ENABLED 1U
#define CY_CAPSENSE_CPU_CLK 48000000u
#define CY_CAPSENSE_VDDA_MV 5000u
#define CY_CAPSENSE_CORE 0u
#define CY_MSCLP_CHIP_ID 0u
#define CY_MSCLP_ENABLED_CH_NUMBER 1u
#define CY_MSCLP_MASTER_CHIP_EN 0u
#define CY_MSCLP_COMM_CHIP_EN 0u
#define CY_MSCLP_LITE_CONFIG_EN 0u
#define CY_MSCLP_CHANNEL_OFFSET 0u
#define CY_MSCLP_SW_CONFIG_VER 2u
#define CY_MSCLP_SRAM_SIZE 1024u
#define CY_MSCLP_SENSE_PAD_NUMBER 32u
#define CY_MSCLP_SENSE_MODE_NUMBER 3u
#define CY_MSCLP_PROG_LFSR_PRESENT 1u
#define CY_MSCLP_PROG_LFSR_FL_PRESENT 1u
#define CY_MSCLP_SAMPLE_NUMBER 0u
#define CY_MSCLP_LP_AOC_PRESENT 1u
#define CY_MSCLP_LEGACY_CONN_PRESENT 0u
#define CY_MSCLP_EXT_SYNC_PRESENT 1u
#define CY_MSCLP_DEBUG_PRESENT 1u
#define CY_MSCLP_CSW_GLOBAL_FUNC_NUMBER 8u
#define CY_MSCLP_DMA_PRESENT 0u
#define CY_MSCLP_CMOD34_PRESENT 0u
#define CY_MSCLP_CIC2_PRESENT 1u
#define CY_MSCLP_CH_NUMBER 1u
#define CY_MSCLP_C_FINE_PRESENT 1u
#define CY_MSCLP_C_DITHER_PRESENT 1u
#define CY_MSCLP_PH2_NON_OVERLAP_PRESENT 1u

extern const cy_stc_scb_uart_config_t CYBSP_UART_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_UART_obj;
#endif /* defined (CY_USING_HAL) */

extern cy_stc_msclp_context_t cy_msclp_0_context;

void init_cycfg_peripherals(void);
void reserve_cycfg_peripherals(void);

#if defined(__cplusplus)
}
#endif /* defined(__cplusplus) */

#endif /* CYCFG_PERIPHERALS_H */
