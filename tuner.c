/******************************************************************************
 * File Name: tuner.c
 *
 * Description: Source file for callback functions and ISR implementations used
 *              by EZI2C, UART, and RTT tuner interfaces.
 *
 *******************************************************************************/

#include "tuner.h"
#include "SEGGER_RTT/RTT/SEGGER_RTT.h"
#include "app_iic.h"
#include "sensor_algorithm.h"
#include <stdbool.h>

#define FW_UPGRADE_COMMAND_CODE (0xABu)
#define FW_UPGRADE_COMMAND_COUNTER (0xCDu)
#define UART_PROTOCOL_MAGIC_SIZE (4u)
#define UART_PROTOCOL_HEADER_SIZE (10u)
/* 命令既接受广播地址0，也接受当前编译变体的手指类型。 */
#define UART_PROTOCOL_DEVICE_TYPE DEVICE_FINGER_TYPE
#define UART_PROTOCOL_COMMAND_RECALIBRATE (0x80u)
#define UART_PROTOCOL_COMMAND_SET_OUTPUT_MODE (0x82u)
#define UART_PROTOCOL_COMMAND_READ_FORCE_CALIBRATION (0x83u)
#define UART_PROTOCOL_COMMAND_SET_FORCE_CALIBRATION (0x84u)
#define UART_PROTOCOL_COMMAND_SAVE_FORCE_CALIBRATION (0x85u)
#define UART_PROTOCOL_COMMAND_DISCARD_FORCE_CALIBRATION (0x86u)
#define UART_PROTOCOL_RESULT_SUCCESS (0u)
#define UART_PROTOCOL_RESULT_INVALID_PAYLOAD (1u)
#define UART_PROTOCOL_RESULT_UNSUPPORTED_COMMAND (2u)
/* 0x84完整20节点力标定载荷为8+20×8=168字节。 */
#define UART_PROTOCOL_MAX_PAYLOAD_SIZE SENSOR_ALGO_FORCE_CALIBRATION_COMMAND_MAX_PAYLOAD_SIZE
#define UART_PROTOCOL_MAX_FRAME_SIZE (UART_PROTOCOL_HEADER_SIZE + UART_PROTOCOL_MAX_PAYLOAD_SIZE)

/*******************************************************************************
 * Global Variables
 *******************************************************************************/

/* EZI2C-specific context */
#if (TUNER_INTERFACE_TYPE == EZI2C)
cy_stc_scb_ezi2c_context_t ezi2c_context;
#endif

/* RTT-specific buffers */
#if (TUNER_INTERFACE_TYPE == RTT)
typedef struct
{
    uint8_t header[2];
    uint8_t tuner_data[sizeof(cy_capsense_tuner)];
    uint8_t tail[3];
} rtt_tuner_data_t;

static uint8_t tuner_down_buf[32];
static rtt_tuner_data_t tuner_up_buf =
{
        .header = {TUNER_CMD_TX_HEADER0, TUNER_CMD_TX_HEADER1},
        .tuner_data = {0},
        .tail = {TUNER_CMD_TX_TAIL0, TUNER_CMD_TX_TAIL1, TUNER_CMD_TX_TAIL2}
};
#endif

/* UART-specific context */
#if (TUNER_INTERFACE_TYPE == UART)
cy_stc_scb_uart_context_t cybsp_uart_context;
extern volatile bool is_fw_upgrade_event_detected;

uint8_t uartRingBuffer[UART_RINGBUFFER_SIZE];
uint8_t uartTxHeader[] = {TUNER_CMD_TX_HEADER0, TUNER_CMD_TX_HEADER1};
uint8_t uartTxTail[] = {TUNER_CMD_TX_TAIL0, TUNER_CMD_TX_TAIL1, TUNER_CMD_TX_TAIL2};
#endif

/*******************************************************************************
 * Function Definitions
 *******************************************************************************/

#if (TUNER_INTERFACE_TYPE == EZI2C)
/* EZI2C ISR */
void ezi2c_isr(void)
{
    Cy_SCB_EZI2C_Interrupt(CYBSP_EZI2C_HW, &ezi2c_context);
}

/*******************************************************************************
 * Function Name: initialize_ezi2c_tuner
 ********************************************************************************
 * Summary:
 *  Initializes EZI2C Tuner communication.
 *******************************************************************************/
void initialize_ezi2c_tuner(void)
{
    cy_en_scb_ezi2c_status_t status = CY_SCB_EZI2C_SUCCESS;

    /* EZI2C interrupt configuration structure */
    const cy_stc_sysint_t ezi2c_intr_config =
    {
            .intrSrc = CYBSP_EZI2C_IRQ,
            .intrPriority = EZI2C_INTR_PRIORITY,
    };

    /* Initialize the EzI2C firmware module */
    status = Cy_SCB_EZI2C_Init(CYBSP_EZI2C_HW, &CYBSP_EZI2C_config, &ezi2c_context);
    if(status != CY_SCB_EZI2C_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    Cy_SysInt_Init(&ezi2c_intr_config, ezi2c_isr);
    NVIC_EnableIRQ(ezi2c_intr_config.intrSrc);

    /* Set the CAPSENSE data structure as the I2C buffer to be exposed to the
     * master on primary slave address interface. Any I2C host tools such as
     * the Tuner or the Bridge Control Panel can read this buffer but you can
     * connect only one tool at a time.
     */
    Cy_SCB_EZI2C_SetBuffer1(CYBSP_EZI2C_HW, (uint8_t *)&cy_capsense_tuner,
            sizeof(cy_capsense_tuner), sizeof(cy_capsense_tuner),
            &ezi2c_context);

    Cy_SCB_EZI2C_Enable(CYBSP_EZI2C_HW);
}
#endif

#if (TUNER_INTERFACE_TYPE == USER_I2C)
void initialize_user_i2c_tuner(void)
{
    AppIic_Init();
}
#endif

/* RTT-specific callback functions */
#if (TUNER_INTERFACE_TYPE == RTT)
/*******************************************************************************
 * Function Name: rtt_tuner_send
 ********************************************************************************
 * Summary:
 *  This function sends the CAPSENSE data to Tuner through RTT
 *
 *******************************************************************************/
void rtt_tuner_send(void * context)
{
    (void)context;

    SEGGER_RTT_LOCK();
    SEGGER_RTT_BUFFER_UP *buffer = _SEGGER_RTT.aUp + RTT_TUNER_CHANNEL;
    buffer->RdOff = 0u;
    buffer->WrOff = sizeof(tuner_up_buf);
    memcpy(tuner_up_buf.tuner_data, &cy_capsense_tuner, sizeof(cy_capsense_tuner));
    SEGGER_RTT_UNLOCK();
}

/*******************************************************************************
 * Function Name: rtt_tuner_receive
 ********************************************************************************
 * Summary:
 *  This function receives the CAPSENSE data from Tuner through RTT
 *
 *******************************************************************************/
void rtt_tuner_receive(uint8_t ** packet, uint8_t ** tuner_packet, void * context)
{
    uint32_t i;
    static uint32_t data_index = 0u;
    static uint8_t command_packet[16u] = {0u};
    while(0 != SEGGER_RTT_HasData(RTT_TUNER_CHANNEL))
    {
        uint8_t byte;
        SEGGER_RTT_Read(RTT_TUNER_CHANNEL, &byte, 1);
        command_packet[data_index++] = byte;
        if (CY_CAPSENSE_COMMAND_PACKET_SIZE == data_index)
        {
            if (CY_CAPSENSE_COMMAND_OK == Cy_CapSense_CheckTunerCmdIntegrity(&command_packet[0u]))
            {
                data_index = 0u;
                *tuner_packet = (uint8_t *)&cy_capsense_tuner;
                *packet = &command_packet[0u];
                break;
            }
            else
            {
                data_index--;
                for(i = 0u; i < (CY_CAPSENSE_COMMAND_PACKET_SIZE - 1u); i++)
                {
                    command_packet[i] = command_packet[i + 1u];
                }
            }
        }
    }
}

/*******************************************************************************
 * Function Name: initialize_rtt_tuner
 ********************************************************************************
 * Summary:
 *  Initializes SEGGER RTT communication for the Tuner.
 *******************************************************************************/
void initialize_rtt_tuner(void)
{
    /* Initializes the RTT Control Block */
    SEGGER_RTT_Init();
    /* Configure or add an up buffer by specifying its name, size and flags */
    SEGGER_RTT_ConfigUpBuffer(RTT_TUNER_CHANNEL, "tuner", &tuner_up_buf, sizeof(tuner_up_buf) + 1, SEGGER_RTT_MODE_NO_BLOCK_TRIM);
    /* Configure or add a down buffer by specifying its name, size and flags */
    SEGGER_RTT_ConfigDownBuffer(RTT_TUNER_CHANNEL, "tuner", tuner_down_buf, sizeof(tuner_down_buf), SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);

    /* Register communication callbacks */
    cy_capsense_context.ptrInternalContext->ptrTunerSendCallback    = rtt_tuner_send;
    cy_capsense_context.ptrInternalContext->ptrTunerReceiveCallback = rtt_tuner_receive;
}
#endif

/* UART-specific callback and ISR functions */
#if (TUNER_INTERFACE_TYPE == UART)
/*******************************************************************************
 * Function Name: uart_tuner_send
 ********************************************************************************
 * Summary:
 *  This function sends the CAPSENSE data to Tuner through UART
 *
 *******************************************************************************/
void uart_tuner_send(void * context)
{
    (void)context;
    Cy_SCB_UART_PutArrayBlocking(CYBSP_UART_HW, &(uartTxHeader[0u]), sizeof(uartTxHeader));
    Cy_SCB_UART_PutArrayBlocking(CYBSP_UART_HW, (uint8_t *)&cy_capsense_tuner, sizeof(cy_capsense_tuner));
    Cy_SCB_UART_PutArrayBlocking(CYBSP_UART_HW, &(uartTxTail[0u]), sizeof(uartTxTail));

    /* Blocking wait for transfer completion */
    while (!Cy_SCB_UART_IsTxComplete(CYBSP_UART_HW))
    {
    }
}


/*******************************************************************************
 * Function Name: uart_tuner_receive
 ********************************************************************************
 * Summary:
 *  This function receives the CAPSENSE data from Tuner through UART
 *
 *******************************************************************************/
void uart_tuner_receive(uint8_t ** packet, uint8_t ** tunerPacket, void * context)
{
    uint32_t i;
    (void)context;
    static uint32_t dataIndex = 0u;
    static uint8_t commandPacket[CY_CAPSENSE_COMMAND_PACKET_SIZE] = {0u};
    uint32_t numBytes;

    while(0u != Cy_SCB_UART_GetNumInRingBuffer(CYBSP_UART_HW, &cybsp_uart_context))
    {
        numBytes = Cy_SCB_UART_GetNumInRingBuffer(CYBSP_UART_HW, &cybsp_uart_context);
        /* Calculate number of bytes to read from ring buffer */
        if ((CY_CAPSENSE_COMMAND_PACKET_SIZE - dataIndex) < numBytes)
        {
            numBytes = CY_CAPSENSE_COMMAND_PACKET_SIZE - dataIndex;
        }
        /* Add received data to the end of commandPacket */
        Cy_SCB_UART_Receive(CYBSP_UART_HW, &commandPacket[dataIndex], numBytes, &cybsp_uart_context);
        dataIndex += numBytes;
        if (CY_CAPSENSE_COMMAND_PACKET_SIZE <= dataIndex)
        {
            if (CY_CAPSENSE_COMMAND_OK == Cy_CapSense_CheckTunerCmdIntegrity(&commandPacket[0u]))
            {
                /* Found a correct command, reset data index and assign pointers to buffers */
                dataIndex = 0u;
                *tunerPacket = (uint8_t *)&cy_capsense_tuner;
                *packet = &commandPacket[0u];
                break;
            }
            else
            {
                /* Command is not correct, remove the first byte in commandPacket buffer */
                dataIndex--;
                for (i = 0u; i < (CY_CAPSENSE_COMMAND_PACKET_SIZE - 1u); i++)
                {
                    commandPacket[i] = commandPacket[i + 1u];
                }
            }
        }
    }
}

static uint32_t uart_fw_upgrade_commandcheck(const uint8_t *commandPacket)
{
    uint32_t status = CY_CAPSENSE_WRONG_CODE;

    if ((FW_UPGRADE_COMMAND_CODE == commandPacket[CY_CAPSENSE_COMMAND_CODE_0_IDX]) &&
        (FW_UPGRADE_COMMAND_COUNTER == commandPacket[CY_CAPSENSE_COMMAND_CNTR_0_IDX]))
    {
        status = CY_CAPSENSE_COMMAND_OK;
    }

    return status;
}

static bool uart_protocol_magic_matches(const uint8_t *frame)
{
    static const uint8_t magic[UART_PROTOCOL_MAGIC_SIZE] = {'D', 'A', 'T', 'A'};
    uint32_t index;

    for (index = 0u; index < UART_PROTOCOL_MAGIC_SIZE; index++)
    {
        if (frame[index] != magic[index])
        {
            return false;
        }
    }

    return true;
}

static void uart_protocol_shift_command(uint8_t *frame, uint32_t *dataIndex)
{
    uint32_t index;

    (*dataIndex)--;
    for (index = 0u; index < *dataIndex; index++)
    {
        frame[index] = frame[index + 1u];
    }
}

static void uart_protocol_handle_command(const uint8_t *frame, uint16_t payloadLen)
{
    uint8_t frameType = frame[5u];
    uint8_t device = frame[6u];
    uint8_t channelCount = frame[7u];

    if ((frame[4u] != SENSOR_ALGO_PROTOCOL_VERSION) ||
        ((device != 0u) && (device != UART_PROTOCOL_DEVICE_TYPE)))
    {
        return;
    }

    if (channelCount != 0u)
    {
        SensorAlgorithm_SendCommandAck(UART_PROTOCOL_RESULT_INVALID_PAYLOAD);
        return;
    }

    SensorAlgorithm_HandleProtocolCommand(frameType, &frame[UART_PROTOCOL_HEADER_SIZE], payloadLen);
}

static void uart_protocol_command_receive(uint8_t byte)
{
    static uint8_t commandFrame[UART_PROTOCOL_MAX_FRAME_SIZE];
    static uint32_t dataIndex;
    uint16_t payloadLen;

    commandFrame[dataIndex++] = byte;

    while ((dataIndex >= UART_PROTOCOL_MAGIC_SIZE) &&
           !uart_protocol_magic_matches(commandFrame))
    {
        uart_protocol_shift_command(commandFrame, &dataIndex);
    }

    if (dataIndex < UART_PROTOCOL_HEADER_SIZE)
    {
        return;
    }

    payloadLen = (uint16_t)((uint16_t)commandFrame[8u] |
                            ((uint16_t)commandFrame[9u] << 8u));
    if (payloadLen > UART_PROTOCOL_MAX_PAYLOAD_SIZE)
    {
        uart_protocol_shift_command(commandFrame, &dataIndex);
        return;
    }

    if (dataIndex == (UART_PROTOCOL_HEADER_SIZE + payloadLen))
    {
        uart_protocol_handle_command(commandFrame, payloadLen);
        dataIndex = 0u;
    }
}

void uart_command_receive(void)
{
    uint32_t i;
    static uint32_t dataIndex = 0u;
    static uint8_t commandPacket[CY_CAPSENSE_COMMAND_PACKET_SIZE] = {0u};
    uint8_t receivedByte;

    while (0u != Cy_SCB_UART_GetNumInRingBuffer(CYBSP_UART_HW, &cybsp_uart_context))
    {
        Cy_SCB_UART_Receive(CYBSP_UART_HW, &receivedByte, 1u, &cybsp_uart_context);
        uart_protocol_command_receive(receivedByte);
        commandPacket[dataIndex++] = receivedByte;

        if (CY_CAPSENSE_COMMAND_PACKET_SIZE <= dataIndex)
        {
            if (CY_CAPSENSE_COMMAND_OK == uart_fw_upgrade_commandcheck(&commandPacket[0u]))
            {
                is_fw_upgrade_event_detected = true;
                dataIndex = 0u;
                break;
            }
            else
            {
                dataIndex--;
                for (i = 0u; i < (CY_CAPSENSE_COMMAND_PACKET_SIZE - 1u); i++)
                {
                    commandPacket[i] = commandPacket[i + 1u];
                }
            }
        }
    }
}
/*******************************************************************************
 * Function Name: initialize_uart_tuner
 ********************************************************************************
 * Summary:
 *  Initializes UART communication for the Tuner.
 *******************************************************************************/
void initialize_uart_tuner(void)
{
    /* UART interrupt configuration structure */
    const cy_stc_sysint_t uart_intr_config =
    {
            .intrSrc = CYBSP_UART_IRQ,
            .intrPriority = 2u,
    };

    /* Initialize, Configure and enable UART communication */
    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &cybsp_uart_context);
    Cy_SysInt_Init(&uart_intr_config, uart_interrupt);
    NVIC_EnableIRQ(uart_intr_config.intrSrc);
    Cy_SCB_UART_StartRingBuffer(CYBSP_UART_HW, uartRingBuffer, UART_RINGBUFFER_SIZE, &cybsp_uart_context);
    Cy_SCB_SetRxFifoLevel(CYBSP_UART_HW, 0u);
    Cy_SCB_UART_Enable(CYBSP_UART_HW);
}

/*******************************************************************************
 * Function Name: uart_interrupt
 ********************************************************************************
 * Summary:
 * Wrapper function for handling interrupts from UART block.
 *
 *******************************************************************************/
void uart_interrupt(void)
{
    Cy_SCB_UART_Interrupt(CYBSP_UART_HW, &cybsp_uart_context);
}

#endif

/* [] END OF FILE */
