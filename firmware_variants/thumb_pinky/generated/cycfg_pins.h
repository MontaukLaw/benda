/*******************************************************************************
 * File Name: cycfg_pins.h
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

#if !defined(CYCFG_PINS_H)
#define CYCFG_PINS_H

#include "cycfg_notices.h"
#include "cy_gpio.h"
#include "cycfg_routing.h"

#if defined (CY_USING_HAL)
#include "cyhal_hwmgr.h"
#endif /* defined (CY_USING_HAL) */

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#define CYBSP_CS_SHIELD1_ENABLED 1U
#define CYBSP_CS_SHIELD1_PORT GPIO_PRT0
#define CYBSP_CS_SHIELD1_PORT_NUM 0U
#define CYBSP_CS_SHIELD1_PIN 0U
#define CYBSP_CS_SHIELD1_NUM 0U
#define CYBSP_CS_SHIELD1_DRIVEMODE CY_GPIO_DM_ANALOG
#define CYBSP_CS_SHIELD1_INIT_DRIVESTATE 1
#ifndef ioss_0_port_0_pin_0_HSIOM
    #define ioss_0_port_0_pin_0_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_CS_SHIELD1_HSIOM ioss_0_port_0_pin_0_HSIOM
#define CYBSP_CS_SHIELD1_IRQ ioss_interrupts_gpio_0_IRQn

#if defined (CY_USING_HAL)
#define CYBSP_CS_SHIELD1_HAL_PORT_PIN P0_0
#define CYBSP_CS_SHIELD1 P0_0
#define CYBSP_CS_SHIELD1_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_CS_SHIELD1_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define CYBSP_CS_SHIELD1_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#endif /* defined (CY_USING_HAL) */

#define CYBSP_CSD_PROX_ENABLED 1U
#define CYBSP_CSD_PROX_PORT GPIO_PRT0
#define CYBSP_CSD_PROX_PORT_NUM 0U
#define CYBSP_CSD_PROX_PIN 1U
#define CYBSP_CSD_PROX_NUM 1U
#define CYBSP_CSD_PROX_DRIVEMODE CY_GPIO_DM_ANALOG
#define CYBSP_CSD_PROX_INIT_DRIVESTATE 1
#ifndef ioss_0_port_0_pin_1_HSIOM
    #define ioss_0_port_0_pin_1_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_CSD_PROX_HSIOM ioss_0_port_0_pin_1_HSIOM
#define CYBSP_CSD_PROX_IRQ ioss_interrupts_gpio_0_IRQn

#if defined (CY_USING_HAL)
#define CYBSP_CSD_PROX_HAL_PORT_PIN P0_1
#define CYBSP_CSD_PROX P0_1
#define CYBSP_CSD_PROX_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_CSD_PROX_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define CYBSP_CSD_PROX_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#endif /* defined (CY_USING_HAL) */

#define ioss_0_port_0_pin_2_ENABLED 1U
#define ioss_0_port_0_pin_2_PORT GPIO_PRT0
#define ioss_0_port_0_pin_2_PORT_NUM 0U
#define ioss_0_port_0_pin_2_PIN 2U
#define ioss_0_port_0_pin_2_NUM 2U
#define ioss_0_port_0_pin_2_DRIVEMODE CY_GPIO_DM_ANALOG
#define ioss_0_port_0_pin_2_INIT_DRIVESTATE 1
#ifndef ioss_0_port_0_pin_2_HSIOM
    #define ioss_0_port_0_pin_2_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_0_pin_2_IRQ ioss_interrupts_gpio_0_IRQn

#if defined (CY_USING_HAL)
#define ioss_0_port_0_pin_2_HAL_PORT_PIN P0_2
#define ioss_0_port_0_pin_2 P0_2
#define ioss_0_port_0_pin_2_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define ioss_0_port_0_pin_2_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define ioss_0_port_0_pin_2_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#define CYBSP_CSD_TP_COL4 (P0_4)
#define CYBSP_CSX_TP_RX4 CYBSP_CSD_TP_COL4
#endif /* defined (CY_USING_HAL) */

#define CYBSP_CSD_TP_COL3_ENABLED 1U
#define CYBSP_CSX_TP_RX3_ENABLED CYBSP_CSD_TP_COL3_ENABLED
#define CYBSP_CSD_TP_COL3_PORT GPIO_PRT0
#define CYBSP_CSX_TP_RX3_PORT CYBSP_CSD_TP_COL3_PORT
#define CYBSP_CSD_TP_COL3_PORT_NUM 0U
#define CYBSP_CSX_TP_RX3_PORT_NUM CYBSP_CSD_TP_COL3_PORT_NUM
#define CYBSP_CSD_TP_COL3_PIN 5U
#define CYBSP_CSX_TP_RX3_PIN CYBSP_CSD_TP_COL3_PIN
#define CYBSP_CSD_TP_COL3_NUM 5U
#define CYBSP_CSX_TP_RX3_NUM CYBSP_CSD_TP_COL3_NUM
#define CYBSP_CSD_TP_COL3_DRIVEMODE CY_GPIO_DM_ANALOG
#define CYBSP_CSX_TP_RX3_DRIVEMODE CYBSP_CSD_TP_COL3_DRIVEMODE
#define CYBSP_CSD_TP_COL3_INIT_DRIVESTATE 1
#define CYBSP_CSX_TP_RX3_INIT_DRIVESTATE CYBSP_CSD_TP_COL3_INIT_DRIVESTATE
#ifndef ioss_0_port_0_pin_5_HSIOM
    #define ioss_0_port_0_pin_5_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_CSD_TP_COL3_HSIOM ioss_0_port_0_pin_5_HSIOM
#define CYBSP_CSX_TP_RX3_HSIOM CYBSP_CSD_TP_COL3_HSIOM
#define CYBSP_CSD_TP_COL3_IRQ ioss_interrupts_gpio_0_IRQn
#define CYBSP_CSX_TP_RX3_IRQ CYBSP_CSD_TP_COL3_IRQ

#if defined (CY_USING_HAL)
#define CYBSP_CSD_TP_COL3_HAL_PORT_PIN P0_5
#define CYBSP_CSX_TP_RX3_HAL_PORT_PIN CYBSP_CSD_TP_COL3_HAL_PORT_PIN
#define CYBSP_CSD_TP_COL3 P0_5
#define CYBSP_CSX_TP_RX3 CYBSP_CSD_TP_COL3
#define CYBSP_CSD_TP_COL3_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_CSX_TP_RX3_HAL_IRQ CYBSP_CSD_TP_COL3_HAL_IRQ
#define CYBSP_CSD_TP_COL3_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define CYBSP_CSX_TP_RX3_HAL_DIR CYBSP_CSD_TP_COL3_HAL_DIR
#define CYBSP_CSD_TP_COL3_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#define CYBSP_CSX_TP_RX3_HAL_DRIVEMODE CYBSP_CSD_TP_COL3_HAL_DRIVEMODE
#endif /* defined (CY_USING_HAL) */

#define CYBSP_CSD_TP_COL2_ENABLED 1U
#define CYBSP_CSX_TP_RX2_ENABLED CYBSP_CSD_TP_COL2_ENABLED
#define CYBSP_CSD_TP_COL2_PORT GPIO_PRT0
#define CYBSP_CSX_TP_RX2_PORT CYBSP_CSD_TP_COL2_PORT
#define CYBSP_CSD_TP_COL2_PORT_NUM 0U
#define CYBSP_CSX_TP_RX2_PORT_NUM CYBSP_CSD_TP_COL2_PORT_NUM
#define CYBSP_CSD_TP_COL2_PIN 6U
#define CYBSP_CSX_TP_RX2_PIN CYBSP_CSD_TP_COL2_PIN
#define CYBSP_CSD_TP_COL2_NUM 6U
#define CYBSP_CSX_TP_RX2_NUM CYBSP_CSD_TP_COL2_NUM
#define CYBSP_CSD_TP_COL2_DRIVEMODE CY_GPIO_DM_ANALOG
#define CYBSP_CSX_TP_RX2_DRIVEMODE CYBSP_CSD_TP_COL2_DRIVEMODE
#define CYBSP_CSD_TP_COL2_INIT_DRIVESTATE 1
#define CYBSP_CSX_TP_RX2_INIT_DRIVESTATE CYBSP_CSD_TP_COL2_INIT_DRIVESTATE
#ifndef ioss_0_port_0_pin_6_HSIOM
    #define ioss_0_port_0_pin_6_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_CSD_TP_COL2_HSIOM ioss_0_port_0_pin_6_HSIOM
#define CYBSP_CSX_TP_RX2_HSIOM CYBSP_CSD_TP_COL2_HSIOM
#define CYBSP_CSD_TP_COL2_IRQ ioss_interrupts_gpio_0_IRQn
#define CYBSP_CSX_TP_RX2_IRQ CYBSP_CSD_TP_COL2_IRQ

#if defined (CY_USING_HAL)
#define CYBSP_CSD_TP_COL2_HAL_PORT_PIN P0_6
#define CYBSP_CSX_TP_RX2_HAL_PORT_PIN CYBSP_CSD_TP_COL2_HAL_PORT_PIN
#define CYBSP_CSD_TP_COL2 P0_6
#define CYBSP_CSX_TP_RX2 CYBSP_CSD_TP_COL2
#define CYBSP_CSD_TP_COL2_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_CSX_TP_RX2_HAL_IRQ CYBSP_CSD_TP_COL2_HAL_IRQ
#define CYBSP_CSD_TP_COL2_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define CYBSP_CSX_TP_RX2_HAL_DIR CYBSP_CSD_TP_COL2_HAL_DIR
#define CYBSP_CSD_TP_COL2_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#define CYBSP_CSX_TP_RX2_HAL_DRIVEMODE CYBSP_CSD_TP_COL2_HAL_DRIVEMODE
#endif /* defined (CY_USING_HAL) */

#define CYBSP_CSD_TP_COL1_ENABLED 1U
#define CYBSP_CSX_TP_RX1_ENABLED CYBSP_CSD_TP_COL1_ENABLED
#define CYBSP_CSD_TP_COL1_PORT GPIO_PRT0
#define CYBSP_CSX_TP_RX1_PORT CYBSP_CSD_TP_COL1_PORT
#define CYBSP_CSD_TP_COL1_PORT_NUM 0U
#define CYBSP_CSX_TP_RX1_PORT_NUM CYBSP_CSD_TP_COL1_PORT_NUM
#define CYBSP_CSD_TP_COL1_PIN 7U
#define CYBSP_CSX_TP_RX1_PIN CYBSP_CSD_TP_COL1_PIN
#define CYBSP_CSD_TP_COL1_NUM 7U
#define CYBSP_CSX_TP_RX1_NUM CYBSP_CSD_TP_COL1_NUM
#define CYBSP_CSD_TP_COL1_DRIVEMODE CY_GPIO_DM_ANALOG
#define CYBSP_CSX_TP_RX1_DRIVEMODE CYBSP_CSD_TP_COL1_DRIVEMODE
#define CYBSP_CSD_TP_COL1_INIT_DRIVESTATE 1
#define CYBSP_CSX_TP_RX1_INIT_DRIVESTATE CYBSP_CSD_TP_COL1_INIT_DRIVESTATE
#ifndef ioss_0_port_0_pin_7_HSIOM
    #define ioss_0_port_0_pin_7_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_CSD_TP_COL1_HSIOM ioss_0_port_0_pin_7_HSIOM
#define CYBSP_CSX_TP_RX1_HSIOM CYBSP_CSD_TP_COL1_HSIOM
#define CYBSP_CSD_TP_COL1_IRQ ioss_interrupts_gpio_0_IRQn
#define CYBSP_CSX_TP_RX1_IRQ CYBSP_CSD_TP_COL1_IRQ

#if defined (CY_USING_HAL)
#define CYBSP_CSD_TP_COL1_HAL_PORT_PIN P0_7
#define CYBSP_CSX_TP_RX1_HAL_PORT_PIN CYBSP_CSD_TP_COL1_HAL_PORT_PIN
#define CYBSP_CSD_TP_COL1 P0_7
#define CYBSP_CSX_TP_RX1 CYBSP_CSD_TP_COL1
#define CYBSP_CSD_TP_COL1_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_CSX_TP_RX1_HAL_IRQ CYBSP_CSD_TP_COL1_HAL_IRQ
#define CYBSP_CSD_TP_COL1_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define CYBSP_CSX_TP_RX1_HAL_DIR CYBSP_CSD_TP_COL1_HAL_DIR
#define CYBSP_CSD_TP_COL1_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#define CYBSP_CSX_TP_RX1_HAL_DRIVEMODE CYBSP_CSD_TP_COL1_HAL_DRIVEMODE
#endif /* defined (CY_USING_HAL) */

#define CYBSP_CMOD1_ENABLED 1U
#define CYBSP_CMOD1_PORT GPIO_PRT1
#define CYBSP_CMOD1_PORT_NUM 1U
#define CYBSP_CMOD1_PIN 0U
#define CYBSP_CMOD1_NUM 0U
#define CYBSP_CMOD1_DRIVEMODE CY_GPIO_DM_ANALOG
#define CYBSP_CMOD1_INIT_DRIVESTATE 1
#ifndef ioss_0_port_1_pin_0_HSIOM
    #define ioss_0_port_1_pin_0_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_CMOD1_HSIOM ioss_0_port_1_pin_0_HSIOM
#define CYBSP_CMOD1_IRQ ioss_interrupts_gpio_1_IRQn

#if defined (CY_USING_HAL)
#define CYBSP_CMOD1_HAL_PORT_PIN P1_0
#define CYBSP_CMOD1 P1_0
#define CYBSP_CMOD1_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_CMOD1_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define CYBSP_CMOD1_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#endif /* defined (CY_USING_HAL) */

#define CYBSP_CMOD2_ENABLED 1U
#define CYBSP_CMOD2_PORT GPIO_PRT1
#define CYBSP_CMOD2_PORT_NUM 1U
#define CYBSP_CMOD2_PIN 1U
#define CYBSP_CMOD2_NUM 1U
#define CYBSP_CMOD2_DRIVEMODE CY_GPIO_DM_ANALOG
#define CYBSP_CMOD2_INIT_DRIVESTATE 1
#ifndef ioss_0_port_1_pin_1_HSIOM
    #define ioss_0_port_1_pin_1_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_CMOD2_HSIOM ioss_0_port_1_pin_1_HSIOM
#define CYBSP_CMOD2_IRQ ioss_interrupts_gpio_1_IRQn

#if defined (CY_USING_HAL)
#define CYBSP_CMOD2_HAL_PORT_PIN P1_1
#define CYBSP_CMOD2 P1_1
#define CYBSP_CMOD2_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_CMOD2_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define CYBSP_CMOD2_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#define CYBSP_CSD_TP_COL0 (P1_2)
#define CYBSP_CSX_TP_RX0 CYBSP_CSD_TP_COL0
#endif /* defined (CY_USING_HAL) */

#define CYBSP_CSD_TP_ROW3_ENABLED 1U
#define CYBSP_CSX_TP_TX3_ENABLED CYBSP_CSD_TP_ROW3_ENABLED
#define CYBSP_CSD_TP_ROW3_PORT GPIO_PRT1
#define CYBSP_CSX_TP_TX3_PORT CYBSP_CSD_TP_ROW3_PORT
#define CYBSP_CSD_TP_ROW3_PORT_NUM 1U
#define CYBSP_CSX_TP_TX3_PORT_NUM CYBSP_CSD_TP_ROW3_PORT_NUM
#define CYBSP_CSD_TP_ROW3_PIN 3U
#define CYBSP_CSX_TP_TX3_PIN CYBSP_CSD_TP_ROW3_PIN
#define CYBSP_CSD_TP_ROW3_NUM 3U
#define CYBSP_CSX_TP_TX3_NUM CYBSP_CSD_TP_ROW3_NUM
#define CYBSP_CSD_TP_ROW3_DRIVEMODE CY_GPIO_DM_ANALOG
#define CYBSP_CSX_TP_TX3_DRIVEMODE CYBSP_CSD_TP_ROW3_DRIVEMODE
#define CYBSP_CSD_TP_ROW3_INIT_DRIVESTATE 1
#define CYBSP_CSX_TP_TX3_INIT_DRIVESTATE CYBSP_CSD_TP_ROW3_INIT_DRIVESTATE
#ifndef ioss_0_port_1_pin_3_HSIOM
    #define ioss_0_port_1_pin_3_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_CSD_TP_ROW3_HSIOM ioss_0_port_1_pin_3_HSIOM
#define CYBSP_CSX_TP_TX3_HSIOM CYBSP_CSD_TP_ROW3_HSIOM
#define CYBSP_CSD_TP_ROW3_IRQ ioss_interrupts_gpio_1_IRQn
#define CYBSP_CSX_TP_TX3_IRQ CYBSP_CSD_TP_ROW3_IRQ

#if defined (CY_USING_HAL)
#define CYBSP_CSD_TP_ROW3_HAL_PORT_PIN P1_3
#define CYBSP_CSX_TP_TX3_HAL_PORT_PIN CYBSP_CSD_TP_ROW3_HAL_PORT_PIN
#define CYBSP_CSD_TP_ROW3 P1_3
#define CYBSP_CSX_TP_TX3 CYBSP_CSD_TP_ROW3
#define CYBSP_CSD_TP_ROW3_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_CSX_TP_TX3_HAL_IRQ CYBSP_CSD_TP_ROW3_HAL_IRQ
#define CYBSP_CSD_TP_ROW3_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define CYBSP_CSX_TP_TX3_HAL_DIR CYBSP_CSD_TP_ROW3_HAL_DIR
#define CYBSP_CSD_TP_ROW3_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#define CYBSP_CSX_TP_TX3_HAL_DRIVEMODE CYBSP_CSD_TP_ROW3_HAL_DRIVEMODE
#endif /* defined (CY_USING_HAL) */

#define CYBSP_CSD_TP_ROW2_ENABLED 1U
#define CYBSP_CSX_TP_TX2_ENABLED CYBSP_CSD_TP_ROW2_ENABLED
#define CYBSP_CSD_TP_ROW2_PORT GPIO_PRT1
#define CYBSP_CSX_TP_TX2_PORT CYBSP_CSD_TP_ROW2_PORT
#define CYBSP_CSD_TP_ROW2_PORT_NUM 1U
#define CYBSP_CSX_TP_TX2_PORT_NUM CYBSP_CSD_TP_ROW2_PORT_NUM
#define CYBSP_CSD_TP_ROW2_PIN 4U
#define CYBSP_CSX_TP_TX2_PIN CYBSP_CSD_TP_ROW2_PIN
#define CYBSP_CSD_TP_ROW2_NUM 4U
#define CYBSP_CSX_TP_TX2_NUM CYBSP_CSD_TP_ROW2_NUM
#define CYBSP_CSD_TP_ROW2_DRIVEMODE CY_GPIO_DM_ANALOG
#define CYBSP_CSX_TP_TX2_DRIVEMODE CYBSP_CSD_TP_ROW2_DRIVEMODE
#define CYBSP_CSD_TP_ROW2_INIT_DRIVESTATE 1
#define CYBSP_CSX_TP_TX2_INIT_DRIVESTATE CYBSP_CSD_TP_ROW2_INIT_DRIVESTATE
#ifndef ioss_0_port_1_pin_4_HSIOM
    #define ioss_0_port_1_pin_4_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_CSD_TP_ROW2_HSIOM ioss_0_port_1_pin_4_HSIOM
#define CYBSP_CSX_TP_TX2_HSIOM CYBSP_CSD_TP_ROW2_HSIOM
#define CYBSP_CSD_TP_ROW2_IRQ ioss_interrupts_gpio_1_IRQn
#define CYBSP_CSX_TP_TX2_IRQ CYBSP_CSD_TP_ROW2_IRQ

#if defined (CY_USING_HAL)
#define CYBSP_CSD_TP_ROW2_HAL_PORT_PIN P1_4
#define CYBSP_CSX_TP_TX2_HAL_PORT_PIN CYBSP_CSD_TP_ROW2_HAL_PORT_PIN
#define CYBSP_CSD_TP_ROW2 P1_4
#define CYBSP_CSX_TP_TX2 CYBSP_CSD_TP_ROW2
#define CYBSP_CSD_TP_ROW2_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_CSX_TP_TX2_HAL_IRQ CYBSP_CSD_TP_ROW2_HAL_IRQ
#define CYBSP_CSD_TP_ROW2_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define CYBSP_CSX_TP_TX2_HAL_DIR CYBSP_CSD_TP_ROW2_HAL_DIR
#define CYBSP_CSD_TP_ROW2_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#define CYBSP_CSX_TP_TX2_HAL_DRIVEMODE CYBSP_CSD_TP_ROW2_HAL_DRIVEMODE
#endif /* defined (CY_USING_HAL) */

#define CYBSP_CSD_TP_ROW1_ENABLED 1U
#define CYBSP_CSX_TP_TX1_ENABLED CYBSP_CSD_TP_ROW1_ENABLED
#define CYBSP_CSD_TP_ROW1_PORT GPIO_PRT1
#define CYBSP_CSX_TP_TX1_PORT CYBSP_CSD_TP_ROW1_PORT
#define CYBSP_CSD_TP_ROW1_PORT_NUM 1U
#define CYBSP_CSX_TP_TX1_PORT_NUM CYBSP_CSD_TP_ROW1_PORT_NUM
#define CYBSP_CSD_TP_ROW1_PIN 5U
#define CYBSP_CSX_TP_TX1_PIN CYBSP_CSD_TP_ROW1_PIN
#define CYBSP_CSD_TP_ROW1_NUM 5U
#define CYBSP_CSX_TP_TX1_NUM CYBSP_CSD_TP_ROW1_NUM
#define CYBSP_CSD_TP_ROW1_DRIVEMODE CY_GPIO_DM_ANALOG
#define CYBSP_CSX_TP_TX1_DRIVEMODE CYBSP_CSD_TP_ROW1_DRIVEMODE
#define CYBSP_CSD_TP_ROW1_INIT_DRIVESTATE 1
#define CYBSP_CSX_TP_TX1_INIT_DRIVESTATE CYBSP_CSD_TP_ROW1_INIT_DRIVESTATE
#ifndef ioss_0_port_1_pin_5_HSIOM
    #define ioss_0_port_1_pin_5_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_CSD_TP_ROW1_HSIOM ioss_0_port_1_pin_5_HSIOM
#define CYBSP_CSX_TP_TX1_HSIOM CYBSP_CSD_TP_ROW1_HSIOM
#define CYBSP_CSD_TP_ROW1_IRQ ioss_interrupts_gpio_1_IRQn
#define CYBSP_CSX_TP_TX1_IRQ CYBSP_CSD_TP_ROW1_IRQ

#if defined (CY_USING_HAL)
#define CYBSP_CSD_TP_ROW1_HAL_PORT_PIN P1_5
#define CYBSP_CSX_TP_TX1_HAL_PORT_PIN CYBSP_CSD_TP_ROW1_HAL_PORT_PIN
#define CYBSP_CSD_TP_ROW1 P1_5
#define CYBSP_CSX_TP_TX1 CYBSP_CSD_TP_ROW1
#define CYBSP_CSD_TP_ROW1_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_CSX_TP_TX1_HAL_IRQ CYBSP_CSD_TP_ROW1_HAL_IRQ
#define CYBSP_CSD_TP_ROW1_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define CYBSP_CSX_TP_TX1_HAL_DIR CYBSP_CSD_TP_ROW1_HAL_DIR
#define CYBSP_CSD_TP_ROW1_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#define CYBSP_CSX_TP_TX1_HAL_DRIVEMODE CYBSP_CSD_TP_ROW1_HAL_DRIVEMODE
#endif /* defined (CY_USING_HAL) */

#define CYBSP_CSD_TP_ROW0_ENABLED 1U
#define CYBSP_CSX_TP_TX0_ENABLED CYBSP_CSD_TP_ROW0_ENABLED
#define CYBSP_CSD_TP_ROW0_PORT GPIO_PRT1
#define CYBSP_CSX_TP_TX0_PORT CYBSP_CSD_TP_ROW0_PORT
#define CYBSP_CSD_TP_ROW0_PORT_NUM 1U
#define CYBSP_CSX_TP_TX0_PORT_NUM CYBSP_CSD_TP_ROW0_PORT_NUM
#define CYBSP_CSD_TP_ROW0_PIN 6U
#define CYBSP_CSX_TP_TX0_PIN CYBSP_CSD_TP_ROW0_PIN
#define CYBSP_CSD_TP_ROW0_NUM 6U
#define CYBSP_CSX_TP_TX0_NUM CYBSP_CSD_TP_ROW0_NUM
#define CYBSP_CSD_TP_ROW0_DRIVEMODE CY_GPIO_DM_ANALOG
#define CYBSP_CSX_TP_TX0_DRIVEMODE CYBSP_CSD_TP_ROW0_DRIVEMODE
#define CYBSP_CSD_TP_ROW0_INIT_DRIVESTATE 1
#define CYBSP_CSX_TP_TX0_INIT_DRIVESTATE CYBSP_CSD_TP_ROW0_INIT_DRIVESTATE
#ifndef ioss_0_port_1_pin_6_HSIOM
    #define ioss_0_port_1_pin_6_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_CSD_TP_ROW0_HSIOM ioss_0_port_1_pin_6_HSIOM
#define CYBSP_CSX_TP_TX0_HSIOM CYBSP_CSD_TP_ROW0_HSIOM
#define CYBSP_CSD_TP_ROW0_IRQ ioss_interrupts_gpio_1_IRQn
#define CYBSP_CSX_TP_TX0_IRQ CYBSP_CSD_TP_ROW0_IRQ

#if defined (CY_USING_HAL)
#define CYBSP_CSD_TP_ROW0_HAL_PORT_PIN P1_6
#define CYBSP_CSX_TP_TX0_HAL_PORT_PIN CYBSP_CSD_TP_ROW0_HAL_PORT_PIN
#define CYBSP_CSD_TP_ROW0 P1_6
#define CYBSP_CSX_TP_TX0 CYBSP_CSD_TP_ROW0
#define CYBSP_CSD_TP_ROW0_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_CSX_TP_TX0_HAL_IRQ CYBSP_CSD_TP_ROW0_HAL_IRQ
#define CYBSP_CSD_TP_ROW0_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define CYBSP_CSX_TP_TX0_HAL_DIR CYBSP_CSD_TP_ROW0_HAL_DIR
#define CYBSP_CSD_TP_ROW0_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#define CYBSP_CSX_TP_TX0_HAL_DRIVEMODE CYBSP_CSD_TP_ROW0_HAL_DRIVEMODE
#endif /* defined (CY_USING_HAL) */

#define ioss_0_port_1_pin_7_ENABLED 1U
#define ioss_0_port_1_pin_7_PORT GPIO_PRT1
#define ioss_0_port_1_pin_7_PORT_NUM 1U
#define ioss_0_port_1_pin_7_PIN 7U
#define ioss_0_port_1_pin_7_NUM 7U
#define ioss_0_port_1_pin_7_DRIVEMODE CY_GPIO_DM_ANALOG
#define ioss_0_port_1_pin_7_INIT_DRIVESTATE 1
#ifndef ioss_0_port_1_pin_7_HSIOM
    #define ioss_0_port_1_pin_7_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_1_pin_7_IRQ ioss_interrupts_gpio_1_IRQn

#if defined (CY_USING_HAL)
#define ioss_0_port_1_pin_7_HAL_PORT_PIN P1_7
#define ioss_0_port_1_pin_7 P1_7
#define ioss_0_port_1_pin_7_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define ioss_0_port_1_pin_7_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define ioss_0_port_1_pin_7_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#endif /* defined (CY_USING_HAL) */

#define CYBSP_CS_HT_BTN3_ENABLED 1U
#define CYBSP_CS_HT_BTN3_PORT GPIO_PRT2
#define CYBSP_CS_HT_BTN3_PORT_NUM 2U
#define CYBSP_CS_HT_BTN3_PIN 0U
#define CYBSP_CS_HT_BTN3_NUM 0U
#define CYBSP_CS_HT_BTN3_DRIVEMODE CY_GPIO_DM_ANALOG
#define CYBSP_CS_HT_BTN3_INIT_DRIVESTATE 1
#ifndef ioss_0_port_2_pin_0_HSIOM
    #define ioss_0_port_2_pin_0_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_CS_HT_BTN3_HSIOM ioss_0_port_2_pin_0_HSIOM
#define CYBSP_CS_HT_BTN3_IRQ ioss_interrupts_gpio_2_IRQn

#if defined (CY_USING_HAL)
#define CYBSP_CS_HT_BTN3_HAL_PORT_PIN P2_0
#define CYBSP_CS_HT_BTN3 P2_0
#define CYBSP_CS_HT_BTN3_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_CS_HT_BTN3_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define CYBSP_CS_HT_BTN3_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#endif /* defined (CY_USING_HAL) */

#define CYBSP_CS_HT_BTN4_ENABLED 1U
#define CYBSP_CS_HT_BTN4_PORT GPIO_PRT2
#define CYBSP_CS_HT_BTN4_PORT_NUM 2U
#define CYBSP_CS_HT_BTN4_PIN 1U
#define CYBSP_CS_HT_BTN4_NUM 1U
#define CYBSP_CS_HT_BTN4_DRIVEMODE CY_GPIO_DM_ANALOG
#define CYBSP_CS_HT_BTN4_INIT_DRIVESTATE 1
#ifndef ioss_0_port_2_pin_1_HSIOM
    #define ioss_0_port_2_pin_1_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_CS_HT_BTN4_HSIOM ioss_0_port_2_pin_1_HSIOM
#define CYBSP_CS_HT_BTN4_IRQ ioss_interrupts_gpio_2_IRQn

#if defined (CY_USING_HAL)
#define CYBSP_CS_HT_BTN4_HAL_PORT_PIN P2_1
#define CYBSP_CS_HT_BTN4 P2_1
#define CYBSP_CS_HT_BTN4_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_CS_HT_BTN4_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define CYBSP_CS_HT_BTN4_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#endif /* defined (CY_USING_HAL) */

#define CYBSP_CS_HT_BTN2_ENABLED 1U
#define CYBSP_CS_HT_BTN2_PORT GPIO_PRT2
#define CYBSP_CS_HT_BTN2_PORT_NUM 2U
#define CYBSP_CS_HT_BTN2_PIN 2U
#define CYBSP_CS_HT_BTN2_NUM 2U
#define CYBSP_CS_HT_BTN2_DRIVEMODE CY_GPIO_DM_ANALOG
#define CYBSP_CS_HT_BTN2_INIT_DRIVESTATE 1
#ifndef ioss_0_port_2_pin_2_HSIOM
    #define ioss_0_port_2_pin_2_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_CS_HT_BTN2_HSIOM ioss_0_port_2_pin_2_HSIOM
#define CYBSP_CS_HT_BTN2_IRQ ioss_interrupts_gpio_2_IRQn

#if defined (CY_USING_HAL)
#define CYBSP_CS_HT_BTN2_HAL_PORT_PIN P2_2
#define CYBSP_CS_HT_BTN2 P2_2
#define CYBSP_CS_HT_BTN2_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_CS_HT_BTN2_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define CYBSP_CS_HT_BTN2_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#endif /* defined (CY_USING_HAL) */

#define CYBSP_CS_HT_BTN1_ENABLED 1U
#define CYBSP_CS_HT_BTN1_PORT GPIO_PRT2
#define CYBSP_CS_HT_BTN1_PORT_NUM 2U
#define CYBSP_CS_HT_BTN1_PIN 3U
#define CYBSP_CS_HT_BTN1_NUM 3U
#define CYBSP_CS_HT_BTN1_DRIVEMODE CY_GPIO_DM_ANALOG
#define CYBSP_CS_HT_BTN1_INIT_DRIVESTATE 1
#ifndef ioss_0_port_2_pin_3_HSIOM
    #define ioss_0_port_2_pin_3_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_CS_HT_BTN1_HSIOM ioss_0_port_2_pin_3_HSIOM
#define CYBSP_CS_HT_BTN1_IRQ ioss_interrupts_gpio_2_IRQn

#if defined (CY_USING_HAL)
#define CYBSP_CS_HT_BTN1_HAL_PORT_PIN P2_3
#define CYBSP_CS_HT_BTN1 P2_3
#define CYBSP_CS_HT_BTN1_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_CS_HT_BTN1_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define CYBSP_CS_HT_BTN1_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#endif /* defined (CY_USING_HAL) */

#define ioss_0_port_2_pin_4_ENABLED 1U
#define ioss_0_port_2_pin_4_PORT GPIO_PRT2
#define ioss_0_port_2_pin_4_PORT_NUM 2U
#define ioss_0_port_2_pin_4_PIN 4U
#define ioss_0_port_2_pin_4_NUM 4U
#define ioss_0_port_2_pin_4_DRIVEMODE CY_GPIO_DM_ANALOG
#define ioss_0_port_2_pin_4_INIT_DRIVESTATE 1
#ifndef ioss_0_port_2_pin_4_HSIOM
    #define ioss_0_port_2_pin_4_HSIOM HSIOM_SEL_GPIO
#endif
#define ioss_0_port_2_pin_4_IRQ ioss_interrupts_gpio_2_IRQn

#if defined (CY_USING_HAL)
#define ioss_0_port_2_pin_4_HAL_PORT_PIN P2_4
#define ioss_0_port_2_pin_4 P2_4
#define ioss_0_port_2_pin_4_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define ioss_0_port_2_pin_4_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define ioss_0_port_2_pin_4_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#define CYBSP_CS_HT_LED4 (P2_5)
#define CYBSP_CS_HT_LED2 (P2_6)
#endif /* defined (CY_USING_HAL) */

#define CYBSP_ALS_ADC_IN_ENABLED 1U
#define CYBSP_ALS_ADC_IN_PORT GPIO_PRT2
#define CYBSP_ALS_ADC_IN_PORT_NUM 2U
#define CYBSP_ALS_ADC_IN_PIN 7U
#define CYBSP_ALS_ADC_IN_NUM 7U
#define CYBSP_ALS_ADC_IN_DRIVEMODE CY_GPIO_DM_ANALOG
#define CYBSP_ALS_ADC_IN_INIT_DRIVESTATE 1
#ifndef ioss_0_port_2_pin_7_HSIOM
    #define ioss_0_port_2_pin_7_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_ALS_ADC_IN_HSIOM ioss_0_port_2_pin_7_HSIOM
#define CYBSP_ALS_ADC_IN_IRQ ioss_interrupts_gpio_2_IRQn

#if defined (CY_USING_HAL)
#define CYBSP_ALS_ADC_IN_HAL_PORT_PIN P2_7
#define CYBSP_ALS_ADC_IN P2_7
#define CYBSP_ALS_ADC_IN_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_ALS_ADC_IN_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define CYBSP_ALS_ADC_IN_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#endif /* defined (CY_USING_HAL) */

#define CYBSP_CS_HT_LED3_ENABLED 1U
#define CYBSP_CS_HT_LED3_PORT GPIO_PRT3
#define CYBSP_CS_HT_LED3_PORT_NUM 3U
#define CYBSP_CS_HT_LED3_PIN 1U
#define CYBSP_CS_HT_LED3_NUM 1U
#define CYBSP_CS_HT_LED3_DRIVEMODE CY_GPIO_DM_ANALOG
#define CYBSP_CS_HT_LED3_INIT_DRIVESTATE 1
#ifndef ioss_0_port_3_pin_1_HSIOM
    #define ioss_0_port_3_pin_1_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_CS_HT_LED3_HSIOM ioss_0_port_3_pin_1_HSIOM
#define CYBSP_CS_HT_LED3_IRQ ioss_interrupts_gpio_3_IRQn

#if defined (CY_USING_HAL)
#define CYBSP_CS_HT_LED3_HAL_PORT_PIN P3_1
#define CYBSP_CS_HT_LED3 P3_1
#define CYBSP_CS_HT_LED3_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_CS_HT_LED3_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define CYBSP_CS_HT_LED3_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_ANALOG
#endif /* defined (CY_USING_HAL) */

#define CYBSP_SWDIO_ENABLED 1U
#define CYBSP_SWDIO_PORT GPIO_PRT3
#define CYBSP_SWDIO_PORT_NUM 3U
#define CYBSP_SWDIO_PIN 2U
#define CYBSP_SWDIO_NUM 2U
#define CYBSP_SWDIO_DRIVEMODE CY_GPIO_DM_STRONG
#define CYBSP_SWDIO_INIT_DRIVESTATE 1
#ifndef ioss_0_port_3_pin_2_HSIOM
    #define ioss_0_port_3_pin_2_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_SWDIO_HSIOM ioss_0_port_3_pin_2_HSIOM
#define CYBSP_SWDIO_IRQ ioss_interrupts_gpio_3_IRQn

#if defined (CY_USING_HAL)
#define CYBSP_SWDIO_HAL_PORT_PIN P3_2
#define CYBSP_SWDIO P3_2
#define CYBSP_SWDIO_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_SWDIO_HAL_DIR CYHAL_GPIO_DIR_BIDIRECTIONAL 
#define CYBSP_SWDIO_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#endif /* defined (CY_USING_HAL) */

#define CYBSP_SWDCK_ENABLED 1U
#define CYBSP_SWDCK_PORT GPIO_PRT3
#define CYBSP_SWDCK_PORT_NUM 3U
#define CYBSP_SWDCK_PIN 3U
#define CYBSP_SWDCK_NUM 3U
#define CYBSP_SWDCK_DRIVEMODE CY_GPIO_DM_STRONG
#define CYBSP_SWDCK_INIT_DRIVESTATE 1
#ifndef ioss_0_port_3_pin_3_HSIOM
    #define ioss_0_port_3_pin_3_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_SWDCK_HSIOM ioss_0_port_3_pin_3_HSIOM
#define CYBSP_SWDCK_IRQ ioss_interrupts_gpio_3_IRQn

#if defined (CY_USING_HAL)
#define CYBSP_SWDCK_HAL_PORT_PIN P3_3
#define CYBSP_SWDCK P3_3
#define CYBSP_SWDCK_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_SWDCK_HAL_DIR CYHAL_GPIO_DIR_BIDIRECTIONAL 
#define CYBSP_SWDCK_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#define CYBSP_CS_HT_LED1 (P4_0)
#define CYBSP_CS_SHIELD2 (P4_3)
#define CYBSP_CSD_BTN1 (P4_4)
#define CYBSP_CSX_BTN0_RX (P4_5)
#define CYBSP_CSX_BTN0_TX (P4_6)
#endif /* defined (CY_USING_HAL) */

#define CYBSP_DEBUG_UART_TX_ENABLED 1U
#define CYBSP_DEBUG_UART_TX_PORT GPIO_PRT5
#define CYBSP_DEBUG_UART_TX_PORT_NUM 5U
#define CYBSP_DEBUG_UART_TX_PIN 1U
#define CYBSP_DEBUG_UART_TX_NUM 1U
#define CYBSP_DEBUG_UART_TX_DRIVEMODE CY_GPIO_DM_HIGHZ
#define CYBSP_DEBUG_UART_TX_INIT_DRIVESTATE 1
#ifndef ioss_0_port_5_pin_1_HSIOM
    #define ioss_0_port_5_pin_1_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_DEBUG_UART_TX_HSIOM ioss_0_port_5_pin_1_HSIOM
#define CYBSP_DEBUG_UART_TX_IRQ ioss_interrupt_gpio_IRQn

#if defined (CY_USING_HAL)
#define CYBSP_DEBUG_UART_TX_HAL_PORT_PIN P5_1
#define CYBSP_DEBUG_UART_TX P5_1
#define CYBSP_DEBUG_UART_TX_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_DEBUG_UART_TX_HAL_DIR CYHAL_GPIO_DIR_INPUT 
#define CYBSP_DEBUG_UART_TX_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_NONE
#endif /* defined (CY_USING_HAL) */

#define CYBSP_DEBUG_UART_RX_ENABLED 1U
#define CYBSP_DEBUG_UART_RX_PORT GPIO_PRT5
#define CYBSP_DEBUG_UART_RX_PORT_NUM 5U
#define CYBSP_DEBUG_UART_RX_PIN 2U
#define CYBSP_DEBUG_UART_RX_NUM 2U
#define CYBSP_DEBUG_UART_RX_DRIVEMODE CY_GPIO_DM_STRONG_IN_OFF
#define CYBSP_DEBUG_UART_RX_INIT_DRIVESTATE 1
#ifndef ioss_0_port_5_pin_2_HSIOM
    #define ioss_0_port_5_pin_2_HSIOM HSIOM_SEL_GPIO
#endif
#define CYBSP_DEBUG_UART_RX_HSIOM ioss_0_port_5_pin_2_HSIOM
#define CYBSP_DEBUG_UART_RX_IRQ ioss_interrupt_gpio_IRQn

#if defined (CY_USING_HAL)
#define CYBSP_DEBUG_UART_RX_HAL_PORT_PIN P5_2
#define CYBSP_DEBUG_UART_RX P5_2
#define CYBSP_DEBUG_UART_RX_HAL_IRQ CYHAL_GPIO_IRQ_NONE
#define CYBSP_DEBUG_UART_RX_HAL_DIR CYHAL_GPIO_DIR_OUTPUT 
#define CYBSP_DEBUG_UART_RX_HAL_DRIVEMODE CYHAL_GPIO_DRIVE_STRONG
#define CYBSP_USER_LED2 (P5_4)
#define CYBSP_LED2 CYBSP_USER_LED2
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_CS_SHIELD1_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_CS_SHIELD1_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_CSD_PROX_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_CSD_PROX_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t ioss_0_port_0_pin_2_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t ioss_0_port_0_pin_2_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_CSD_TP_COL3_config;

#define CYBSP_CSX_TP_RX3_config CYBSP_CSD_TP_COL3_config

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_CSD_TP_COL3_obj;
#define CYBSP_CSX_TP_RX3_obj CYBSP_CSD_TP_COL3_obj
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_CSD_TP_COL2_config;

#define CYBSP_CSX_TP_RX2_config CYBSP_CSD_TP_COL2_config

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_CSD_TP_COL2_obj;
#define CYBSP_CSX_TP_RX2_obj CYBSP_CSD_TP_COL2_obj
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_CSD_TP_COL1_config;

#define CYBSP_CSX_TP_RX1_config CYBSP_CSD_TP_COL1_config

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_CSD_TP_COL1_obj;
#define CYBSP_CSX_TP_RX1_obj CYBSP_CSD_TP_COL1_obj
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_CMOD1_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_CMOD1_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_CMOD2_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_CMOD2_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_CSD_TP_ROW3_config;

#define CYBSP_CSX_TP_TX3_config CYBSP_CSD_TP_ROW3_config

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_CSD_TP_ROW3_obj;
#define CYBSP_CSX_TP_TX3_obj CYBSP_CSD_TP_ROW3_obj
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_CSD_TP_ROW2_config;

#define CYBSP_CSX_TP_TX2_config CYBSP_CSD_TP_ROW2_config

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_CSD_TP_ROW2_obj;
#define CYBSP_CSX_TP_TX2_obj CYBSP_CSD_TP_ROW2_obj
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_CSD_TP_ROW1_config;

#define CYBSP_CSX_TP_TX1_config CYBSP_CSD_TP_ROW1_config

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_CSD_TP_ROW1_obj;
#define CYBSP_CSX_TP_TX1_obj CYBSP_CSD_TP_ROW1_obj
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_CSD_TP_ROW0_config;

#define CYBSP_CSX_TP_TX0_config CYBSP_CSD_TP_ROW0_config

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_CSD_TP_ROW0_obj;
#define CYBSP_CSX_TP_TX0_obj CYBSP_CSD_TP_ROW0_obj
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t ioss_0_port_1_pin_7_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t ioss_0_port_1_pin_7_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_CS_HT_BTN3_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_CS_HT_BTN3_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_CS_HT_BTN4_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_CS_HT_BTN4_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_CS_HT_BTN2_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_CS_HT_BTN2_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_CS_HT_BTN1_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_CS_HT_BTN1_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t ioss_0_port_2_pin_4_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t ioss_0_port_2_pin_4_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_ALS_ADC_IN_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_ALS_ADC_IN_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_CS_HT_LED3_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_CS_HT_LED3_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_SWDIO_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_SWDIO_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_SWDCK_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_SWDCK_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_DEBUG_UART_TX_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_DEBUG_UART_TX_obj;
#endif /* defined (CY_USING_HAL) */

extern const cy_stc_gpio_pin_config_t CYBSP_DEBUG_UART_RX_config;

#if defined (CY_USING_HAL)
extern const cyhal_resource_inst_t CYBSP_DEBUG_UART_RX_obj;
#endif /* defined (CY_USING_HAL) */

void init_cycfg_pins(void);
void reserve_cycfg_pins(void);

#if defined(__cplusplus)
}
#endif /* defined(__cplusplus) */

#endif /* CYCFG_PINS_H */
