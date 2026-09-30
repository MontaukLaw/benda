/******************************************************************************
 * File Name: tuner.h
 *
 * Description: Header file for the tuner interface. It contains macros,
 *              declarations, and function prototypes for the tuner callbacks
 *              used in EZI2C, UART, and RTT modes.
 *
 *******************************************************************************/

#ifndef TUNER_H
#define TUNER_H

#include "cy_pdl.h"
#include "cycfg_capsense.h"

/*******************************************************************************
 * User-configurable Macros
 ********************************************************************************/

/* Supported tuner interface types (do not change below 3 macros*/
#define EZI2C   (0u)  /* EZI2C Tuner Interface */
#define UART    (1u)  /* UART Tuner Interface */
#define RTT     (2u)  /* RTT Tuner Interface */
#define USER_I2C (3u) /* User I2C protocol interface */

/* User-configurable macro for selecting tuner interface type */
#ifndef TUNER_INTERFACE_TYPE
#define TUNER_INTERFACE_TYPE UART /* Set to desired interface: EZI2C, UART, RTT, USER_I2C */
#endif


/*******************************************************************************
 * Fixed Macros
 *******************************************************************************/
#if (TUNER_INTERFACE_TYPE == EZI2C) || (TUNER_INTERFACE_TYPE == USER_I2C)
/* EZI2C interrupt priority must be higher than CAPSENSE&trade; interrupt. */
#define EZI2C_INTR_PRIORITY             (2u)
#endif

#if (TUNER_INTERFACE_TYPE == RTT)
/* Macros required for RTT communication */
#define RTT_TUNER_CHANNEL               (1u)
#endif

#if (TUNER_INTERFACE_TYPE == UART)
/* UART buffer size */
/* 容纳V21最大命令，避免20通道扫描期间长命令被接收缓存截断。 */
#define UART_RINGBUFFER_SIZE            (1024u)
#endif


#if (TUNER_INTERFACE_TYPE == RTT | TUNER_INTERFACE_TYPE == UART)
/* Tuner Command package headers */
#define TUNER_CMD_TX_HEADER0            (0x0Du)
#define TUNER_CMD_TX_HEADER1            (0x0Au)
#define TUNER_CMD_TX_TAIL0              (0x00u)
#define TUNER_CMD_TX_TAIL1              (0xFFu)
#define TUNER_CMD_TX_TAIL2              (0xFFu)
#endif

#define CY_ASSERT_FAILED                (0u)
/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/

/* EZI2C ISR function */
#if (TUNER_INTERFACE_TYPE == EZI2C)
void ezi2c_isr(void);
void initialize_ezi2c_tuner(void);
#endif

#if (TUNER_INTERFACE_TYPE == USER_I2C)
void initialize_user_i2c_tuner(void);
#endif

/* RTT callback functions */
#if (TUNER_INTERFACE_TYPE == RTT)
void rtt_tuner_send(void *context);
void rtt_tuner_receive(uint8_t **packet, uint8_t **tuner_packet, void *context);
void initialize_rtt_tuner(void);
#endif

/* UART callback and ISR functions */
#if (TUNER_INTERFACE_TYPE == UART)
void uart_tuner_send(void *context);
void uart_tuner_receive(uint8_t **packet, uint8_t **tuner_packet, void *context);
void uart_command_receive(void);
void uart_interrupt(void);
void initialize_uart_tuner(void);
#endif

#endif /* TUNER_H */

/* [] END OF FILE */
