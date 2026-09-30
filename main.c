/******************************************************************************
 * File Name: main.c
 *
 * Description: This is the source code for the PSOC 4 MSCLP self-capacitance
 * button code example for ModusToolbox with supporting multiple communication
 * interfaces with CAPSENSE Tuner.
 *
 * Related Document: See README.md
 *
 *******************************************************************************
 * Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company)
 * or an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_dfu.h"
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "app_iic.h"
#include "capsense_autorange.h"
/* 引入食指全通道固定CapSense档位设置接口。 */
#include "capsense_channel_config.h"
/* 引入只修改 RAM 的全通道 Nsub 手动控制接口。 */
#include "capsense_nsub_control.h"
#include "capsense_runtime.h"
/* 引入非阻塞芯片温度采样接口。 */
#include "die_temperature.h"
#include "force_vector_cal.h"
#include "iap_request.h"
#include "sensor_algorithm.h"
/* 使用统一状态语义，空闲、等待和恢复均不代表有效接触。 */
#include "channel_state_machine.h"
#include "sensor_bus_slave.h"
#include "stdio.h"
#include "tuner.h"
/*******************************************************************************
 * User configurable Macros
 ********************************************************************************/

/*******************************************************************************
 * Fixed Macros
 *******************************************************************************/
#define CAPSENSE_MSC0_INTR_PRIORITY (3u)
#define CY_ASSERT_FAILED (0u)
#define UART_TX_BUFFER_SIZE (16u)
/* 容纳完整48节点命令及帧头，避免扫描期间串口环形缓存溢出。 */
#define UART_RX_BUFFER_SIZE (1024u)
#define RAWCOUNT_SENSOR_COUNT CY_CAPSENSE_SENSOR_COUNT
#define RAWCOUNT_BYTES_PER_SENSOR (2u)
#define RAWCOUNT_MAGIC_SIZE (4u)
#define RAWCOUNT_FRAME_SIZE ((RAWCOUNT_SENSOR_COUNT * RAWCOUNT_BYTES_PER_SENSOR) + RAWCOUNT_MAGIC_SIZE)
#define ALGO_VALUES_PER_SENSOR (3u)
#define FORCE_VECTOR_VALUE_COUNT (3u)
#define FORCE_VECTOR_BYTES_PER_VALUE (4u)
#define ALGO_FRAME_SIZE                                                                                                                                                            \
	((RAWCOUNT_SENSOR_COUNT * ALGO_VALUES_PER_SENSOR * RAWCOUNT_BYTES_PER_SENSOR) + (FORCE_VECTOR_VALUE_COUNT * FORCE_VECTOR_BYTES_PER_VALUE) + RAWCOUNT_MAGIC_SIZE)
#if (RAWCOUNT_SENSOR_COUNT != SENSOR_ALGO_CHANNEL_COUNT)
#error "SENSOR_ALGO_CHANNEL_COUNT must match CY_CAPSENSE_SENSOR_COUNT"
#endif
#define UART_PERI_DIVIDER_NUM (3u)
#define UART_460800_DIVIDER (12u)
#define UART_1000000_DIVIDER (5u)
#define RAWCOUNT_INV_K_Q15_ONE (32768u)

#define BOOTLOADER_ID (0u)

/*******************************************************************************
 * Global Variables
 *******************************************************************************/
CY_SECTION(".cy_app_signature") __USED static const uint32_t cy_dfu_appSignature;
volatile bool is_fw_upgrade_event_detected;

#if CY_CAPSENSE_BIST_EN
uint32_t sensor_id;
uint32_t sense_cap[CY_CAPSENSE_SENSOR_COUNT];
cy_en_capsense_bist_status_t measure_status[CY_CAPSENSE_SENSOR_COUNT];
#endif
uint8_t timer1s_flag = 0;
/* 按当前传感器帧率累计约一秒后触发一次温度转换。 */
static uint16_t die_temperature_frame_count;

/* 矩阵以200帧为上限调度；连续累计SysTick差值，避免长扫描跨回绕丢失计时。 */
#if DEVICE_MATRIX20
static uint32_t matrix_last_tick;
static uint32_t matrix_frame_cycles;
static uint64_t matrix_timing_sum;
static uint32_t matrix_timing_count;
/* 开机和三参数生效后重新测量完整帧周期，测量期间由算法保持校准。 */
static void matrix_restart_timing(void) {
	matrix_last_tick = Cy_SysTick_GetValue();
	matrix_frame_cycles = 0u;
	matrix_timing_sum = 0u;
	matrix_timing_count = 0u;
}
/* 每次主循环更新时钟，等待期间继续处理串口及DFU命令。 */
static bool matrix_frame_ready(void) {
	uint32_t tick = Cy_SysTick_GetValue();
	matrix_frame_cycles += (matrix_last_tick - tick) & 0x00FFFFFFu;
	matrix_last_tick = tick;
	return matrix_frame_cycles >= CY_CAPSENSE_CPU_CLK / DEVICE_ALGORITHM_FRAME_RATE_HZ;
}
/* 前64帧锁定实际时基，随后重新校准通过统一入口清空所有状态历史。 */
static void matrix_frame_consumed(void) {
	if (matrix_timing_count < 64u) {
		matrix_timing_sum += matrix_frame_cycles;
		matrix_timing_count++;
		if (matrix_timing_count == 64u) {
			uint32_t rate = (uint32_t)(((uint64_t)CY_CAPSENSE_CPU_CLK * 64u +
				matrix_timing_sum / 2u) / matrix_timing_sum);
			ChannelState_SetSampleRateHz(rate);
			SensorAlgorithm_RequestRecalibration();
		}
	}
	matrix_frame_cycles = 0u;
}
#endif

#if (TUNER_INTERFACE_TYPE != UART)
cy_stc_scb_uart_context_t cybsp_uart_context;

uint8_t uartTxBuffer[UART_TX_BUFFER_SIZE];
uint8_t uartRxBuffer[UART_RX_BUFFER_SIZE];
uint8_t uartTxHeader[] = {0, 0};
uint8_t uartTxTail[] = {0, 0};
void bsp_uart_initialize(void);
#endif
/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/
static void initialize_capsense(void);
static void initialize_capsense_tuner(void);
static void capsense_msc0_isr(void);

#if CY_CAPSENSE_BIST_EN
static void measure_cp(void);
#endif
void SysTick_Callback(void);

void Sensor_Rawcount_Update(void);
void Sensor_Diffcount_Update(void);
static void Sensor_ReadRawCounts(uint16_t raw_values[RAWCOUNT_SENSOR_COUNT]);

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 *  System entrance point. This function performs
 *  - initial setup of device
 *  - initialize CAPSENSE
 *  - initialize tuner communication
 *  - scan touch input continuously
 *  - LED control for touch indication
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void) {

	cy_rslt_t result;

	/* Initialize the device and board peripherals */
	result = cybsp_init();

	/* Board init failed. Stop program execution */
	if (result != CY_RSLT_SUCCESS) {
		CY_ASSERT(CY_ASSERT_FAILED);
	}

	Cy_SysClk_PeriphDisableDivider(CY_SYSCLK_DIV_16_BIT, UART_PERI_DIVIDER_NUM);
	Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, UART_PERI_DIVIDER_NUM, UART_1000000_DIVIDER);
	Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, UART_PERI_DIVIDER_NUM);
	/* Enable global interrupts */
	__enable_irq();

	// Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_LF, 40000);
	// /* Register one of the SysTick callbacks */
	// Cy_SysTick_SetCallback(0UL, &SysTick_Callback);
	/* Initialize MSC CAPSENSE */
	initialize_capsense();
#if DEVICE_ENFORCE_FIXED_CAPSENSE_CONFIG
	/* 首次扫描前按设备默认三参数同步食指全部通道的配置与硬件镜像。 */
	if (!CapSenseChannelConfig_ApplyFixedAll(
			DEVICE_FIXED_CAPSENSE_NSUB,
			DEVICE_FIXED_CAPSENSE_TX_DIVIDER,
			DEVICE_FIXED_CAPSENSE_REF_CDAC)) {
		CY_ASSERT(CY_ASSERT_FAILED);
	}
#endif
	CapSenseRuntime_Init();
	/* 初始化只作用于 RAM 的全通道 Nsub 手动控制模块。 */
	CapSenseNsubControl_Init();
#if DEVICE_HAS_MATRIX_AUTORANGE
	/* 固定 MatrixButtons0 的 CIC2 配置，并从最高灵敏度的 Ref CDAC=12 启动自动量程。 */
	(void)CapSenseMatrixConfig_Request(
		CAPSENSE_AUTORANGE_FIXED_NSUB,
		CAPSENSE_AUTORANGE_FIXED_TX_DIV,
		CAPSENSE_AUTORANGE_START_REF);
	(void)CapSenseNsubControl_ApplyPending();
	(void)CapSenseAutoRange_Init();
#endif
	/* 温度模块初始化失败时保持主传感器运行，仅停止温度帧上传。 */
	(void)DieTemperature_Init();

	/* Initialize EZI2C */
	initialize_capsense_tuner();

	// 对算法做初始化.
	SensorAlgorithm_Init();
#if (TUNER_INTERFACE_TYPE != UART) && (TUNER_INTERFACE_TYPE != USER_I2C)
	SensorBusSlave_Init();
#endif

	// bsp_uart_initialize();

	// Cy_SCB_UART_PutString(CYBSP_UART_HW, "******application
	// started*******\r\n");

#if CY_CAPSENSE_BIST_EN
	measure_cp();
	/* Measure the sensor capacitance using BIST */
	Cy_CapSense_RunSelfTest(CY_CAPSENSE_BIST_SNS_CAP_MASK | CY_CAPSENSE_BIST_ELTD_CAP_MASK, &cy_capsense_context);
#endif

	/* Start the first scan */
#if DEVICE_MATRIX20
	/* 首次扫描前启动完整帧周期计时。 */
	matrix_restart_timing();
#endif
	CapSenseRuntime_MarkScanStarted();
	Cy_CapSense_ScanAllSlots(&cy_capsense_context);

	for (;;) {
		int16_t temperature_tenth_celsius;
#if DEVICE_MATRIX20
		/* 限速只延后算法处理，不阻塞命令接收或固件升级入口。 */
		bool frame_ready = matrix_frame_ready();
#endif

		SensorAlgorithm_UartTxPoll();
#if (TUNER_INTERFACE_TYPE == UART)
		/* 扫描和帧间等待期间也及时消费命令，避免DFU依赖下一帧完成。 */
		uart_command_receive();
#endif
		/* 轮询非阻塞SAR转换，完成后通过Type 0x03上传。 */
		if (DieTemperature_TryReadTenthDegreeC(&temperature_tenth_celsius)) {
			SensorAlgorithm_SendTemperatureFrame(temperature_tenth_celsius);
		}
#if (TUNER_INTERFACE_TYPE == USER_I2C)
		AppIic_Poll();
#endif

		if (is_fw_upgrade_event_detected) {
			is_fw_upgrade_event_detected = false;
#if defined(APP_I2C_IAP) && (APP_I2C_IAP)
			IapRequest_EnterBootloader(IAP_TRANSPORT_I2C);
#else
			IapRequest_EnterBootloader(IAP_TRANSPORT_UART);
#endif
		}

		/* 扫描一旦完成立即记时，使扫描耗时不包含后续帧间等待。 */
		bool scan_complete = CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context);
		if (scan_complete) {
			CapSenseRuntime_MarkScanComplete();
		}
		/* 矩阵扫描完成且到达帧周期后执行一次算法。 */
		if (scan_complete
#if DEVICE_MATRIX20
			&& frame_ready
#endif
		) {
#if DEVICE_MATRIX20
			/* 完整帧周期包含扫描、算法和串口服务开销。 */
			CapSenseRuntime_SetFramePeriodUs(matrix_frame_cycles / (CY_CAPSENSE_CPU_CLK / 1000000u));
			matrix_frame_consumed();
#endif
			/* 在新 Nsub 的第一帧完成后重置滤波历史和 CapSense 基线。 */
			if (CapSenseNsubControl_FinalizeAppliedScan()) {
#if DEVICE_MATRIX20
				/* 三参数会改变扫描耗时，生效后重新测量时基。 */
				matrix_restart_timing();
#endif
#if DEVICE_HAS_MATRIX_AUTORANGE
				/* 同步手动矩阵配置的实际值及当前Ref。 */
				CapSenseAutoRange_OnMatrixConfigApplied();
#endif
				/* 让算法基线和噪声同步适应新的 Raw 量级。 */
				SensorAlgorithm_RequestRecalibration();
			}
			if (CapSenseRuntime_UpdateBaselineIfPending()) {
				SensorAlgorithm_RestartBaselineInitialization();
			}

			/* Process all widgets */
			Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

#if (TUNER_INTERFACE_TYPE == UART)
			uart_command_receive();
#endif
			/* 每完成约一秒的CapSense扫描后启动一次独立温度转换。 */
			die_temperature_frame_count++;
			if (die_temperature_frame_count >= SENSOR_ALGO_FRAME_RATE_HZ) {
				die_temperature_frame_count = 0u;
				(void)DieTemperature_StartMeasurement();
			}
			Sensor_Rawcount_Update();
			CapSenseRuntime_ApplyPending();
			/* 在扫描空闲点应用全通道易失性 Nsub 请求，且不写 Flash。 */
			(void)CapSenseNsubControl_ApplyPending();
#if DEVICE_HAS_MATRIX_AUTORANGE
			/* 自动量程只在扫描空闲点装载下一帧使用的 Ref CDAC。 */
			(void)CapSenseAutoRange_ApplyPending();
#endif
			// Sensor_Diffcount_Update();

			/* Scan all Widget */
			CapSenseRuntime_MarkScanStarted();
			Cy_CapSense_ScanAllSlots(&cy_capsense_context);
		}
	}
}

/*******************************************************************************
 * Function Name: initialize_capsense
 ********************************************************************************
 * Summary:
 *  This function initializes the CAPSENSE and configures the CAPSENSE
 *  interrupt.
 *
 *******************************************************************************/
static void initialize_capsense(void) {
	cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;

	/* CAPSENSE interrupt configuration MSCLP 0 */
	const cy_stc_sysint_t capsense_msc0_interrupt_config = {
		.intrSrc = CY_MSCLP0_LP_IRQ,
		.intrPriority = CAPSENSE_MSC0_INTR_PRIORITY,
	};

	/* Capture the MSC HW block and initialize it to the default state. */
	status = Cy_CapSense_Init(&cy_capsense_context);

	if (CY_CAPSENSE_STATUS_SUCCESS == status) {
		/* Initialize CAPSENSE interrupt for MSCLP 0 */
		Cy_SysInt_Init(&capsense_msc0_interrupt_config, capsense_msc0_isr);
		NVIC_ClearPendingIRQ(capsense_msc0_interrupt_config.intrSrc);
		NVIC_EnableIRQ(capsense_msc0_interrupt_config.intrSrc);

		/* Initialize the CAPSENSE firmware modules. */
		status = Cy_CapSense_Enable(&cy_capsense_context);
	}

	if (status != CY_CAPSENSE_STATUS_SUCCESS) {
		/* This status could fail before tuning the sensors correctly.
		 * Ensure that this function passes after the CAPSENSE sensors are tuned
		 * as per procedure give in the Readme.md file */
	}
}

/*******************************************************************************
 * Function Name: initialize_capsense_tuner
 ********************************************************************************
 * Summary:
 * This function is for Initializing the communication with the CAPSENSE Tuner
 * tool.
 *
 *******************************************************************************/
static void initialize_capsense_tuner(void) {
#if (TUNER_INTERFACE_TYPE == EZI2C)
	initialize_ezi2c_tuner();

#elif (TUNER_INTERFACE_TYPE == USER_I2C)
	initialize_user_i2c_tuner();

#elif (TUNER_INTERFACE_TYPE == RTT)
	initialize_rtt_tuner();

#elif (TUNER_INTERFACE_TYPE == UART)
	initialize_uart_tuner();
#endif
}

/*******************************************************************************
 * Function Name: capsense_msc0_isr
 ********************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from CAPSENSE MSC0 block.
 *
 *******************************************************************************/
static void capsense_msc0_isr(void) { Cy_CapSense_InterruptHandler(CY_MSCLP0_HW, &cy_capsense_context); }

#if CY_CAPSENSE_BIST_EN
/*******************************************************************************
 * Function Name: measure_cp
 ********************************************************************************
 * Summary:
 * Measures the sensor capacitance of the Tx and Rx electrodes to determine Tx
 * clock frequency. The measured sensor capacitance (Cp) values are stored in
 * the array 'sense_cap[x]', where x is the sensor number.
 *
 *******************************************************************************/
static void measure_cp(void) {
	//    for (sensor_id = CY_CAPSENSE_BUTTON0_WDGT_ID; sensor_id <
	//    CY_CAPSENSE_SENSOR_COUNT; sensor_id++)
	//    {
	//        measure_status[sensor_id] =
	//        Cy_CapSense_MeasureCapacitanceSensorElectrode(
	//                                    CY_CAPSENSE_BUTTON0_WDGT_ID,
	//                                    sensor_id, &cy_capsense_context);
	//        sense_cap[sensor_id] =
	//        cy_capsense_context.ptrWdConfig[CY_CAPSENSE_BUTTON0_WDGT_ID].ptrEltdCapacitance[sensor_id];
	//    }
	//	for(sensor_id = CY_CAPSENSE_BUTTON0_WDGT_ID; sensor_id <
	// CY_CAPSENSE_SENSOR_COUNT;sensor_id++ )
	//	{
	//		measure_status[sensor_id] =
	// Cy_CapSense_MeasureCapacitanceSlotSensors(
	// sensor_id, 0, &cy_capsense_context); 		sense_cap[sensor_id] =
	// cy_capsense_context.ptrWdConfig[CY_CAPSENSE_SENSOR_COUNT].ptrSnsCapacitance[sensor_id];
	//
	//	}
}
#endif
/*******************************************************************************
 * Function Name: SysTick_Callback
 ****************************************************************************/
void SysTick_Callback(void) {
	/* Some action */
	timer1s_flag = 1;
}

#if 0 // (TUNER_INTERFACE_TYPE != UART)
/*******************************************************************************
 * Function Name: uart_tuner_send
 ********************************************************************************
 * Summary:
 *  This function sends the CAPSENSE data to Tuner through UART
 *
 *******************************************************************************/
void bsp_uart_send(uint8_t *databuffer, uint8_t size)
{

    //Cy_SCB_UART_PutArrayBlocking(CYBSP_UART_HW, &(uartTxHeader[0u]), sizeof(uartTxHeader));
    Cy_SCB_UART_PutArrayBlocking(CYBSP_UART_HW, databuffer, size);
    //Cy_SCB_UART_PutArrayBlocking(CYBSP_UART_HW, &(uartTxTail[0u]), sizeof(uartTxTail));

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
void bsp_uart_receive(void)
{
    uint32_t numBytes;

    while(0u != Cy_SCB_UART_GetNumInRingBuffer(CYBSP_UART_HW, &cybsp_uart_context))
    {
        numBytes = Cy_SCB_UART_GetNumInRingBuffer(CYBSP_UART_HW, &cybsp_uart_context);
       
        /* Add received data to the end of commandPacket */
        Cy_SCB_UART_Receive(CYBSP_UART_HW, &uartRxBuffer[0], numBytes, &cybsp_uart_context);

        
    }
}
/*******************************************************************************
 * Function Name: uart_interrupt
 ********************************************************************************
 * Summary:
 * Wrapper function for handling interrupts from UART block.
 *
 *******************************************************************************/
void bsp_uart_interrupt(void)
{
    Cy_SCB_UART_Interrupt(CYBSP_UART_HW, &cybsp_uart_context);
}
/*******************************************************************************
 * Function Name: initialize_uart_tuner
 ********************************************************************************
 * Summary:
 *  Initializes UART communication for the Tuner.
 *******************************************************************************/
void bsp_uart_initialize(void)
{
    /* UART interrupt configuration structure */
    const cy_stc_sysint_t uart_intr_config =
    {
            .intrSrc = CYBSP_UART_IRQ,
            .intrPriority = 2u,
    };


    /* Initialize, Configure and enable UART communication */
    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &cybsp_uart_context);
    Cy_SysInt_Init(&uart_intr_config, bsp_uart_interrupt);
    NVIC_EnableIRQ(uart_intr_config.intrSrc);
    Cy_SCB_UART_StartRingBuffer(CYBSP_UART_HW, uartRxBuffer, UART_RX_BUFFER_SIZE, &cybsp_uart_context);
    Cy_SCB_UART_Enable(CYBSP_UART_HW);
}

#endif

/* 读取一帧Hardware Raw，按当前变体完成输入处理后再送入传感算法。 */
void Sensor_Rawcount_Update(void) {

	/* 固定档位仅采集一份硬件Raw，算法与协议使用同一帧真实计数。 */
	uint16_t raw_values[RAWCOUNT_SENSOR_COUNT];
	Sensor_ReadRawCounts(raw_values);
	SensorAlgorithm_ProcessAndSend(raw_values);
#if (TUNER_INTERFACE_TYPE == USER_I2C)
	AppIic_OnScanComplete();
#elif (TUNER_INTERFACE_TYPE != UART)
	SensorBusSlave_Update();
#endif
	
	// 下面是旧的算法.

#if 0
	SensorAlgorithm_Process(raw_values, outputs);

	// float p_00_f_val, float p_01_f_val, float p_10_f_val, float p_11_f_val
	// force_vector_cal((float)outputs[1u].raw, (float)outputs[0u].raw, (float)outputs[3u].raw, (float)outputs[2u].raw);
	force_vector_cal((float)outputs[1u].signal, (float)outputs[0u].signal, (float)outputs[3u].signal, (float)outputs[2u].signal);
	// force_vector_cal((float)outputs[1u].signal, (float)outputs[0u].signal, (float)outputs[3u].signal, (float)outputs[2u].signal);
	Sensor_SendAlgorithmFrame(outputs);
#endif

}

static void Sensor_ReadRawCounts(uint16_t raw_values[RAWCOUNT_SENSOR_COUNT]) {
/* 矩阵按Configurator的RX优先、TX递增顺序映射全部20点。 */
#if DEVICE_MATRIX20 && (RAWCOUNT_SENSOR_COUNT == 20u)
	uint32_t index;
	for (index = 0u; index < RAWCOUNT_SENSOR_COUNT; ++index) {
		raw_values[index] = cy_capsense_tuner.sensorContext[index].raw;
	}
#elif defined(CY_CAPSENSE_BUTTON4_RX0_RAW0_VALUE) && (RAWCOUNT_SENSOR_COUNT == 5u)
	/* 按 Configurator 中 Button0～Button4 的顺序映射五个独立通道。 */
	raw_values[0u] = (uint16_t)CY_CAPSENSE_BUTTON0_RX0_RAW0_VALUE;
	raw_values[1u] = (uint16_t)CY_CAPSENSE_BUTTON1_RX0_RAW0_VALUE;
	raw_values[2u] = (uint16_t)CY_CAPSENSE_BUTTON2_RX0_RAW0_VALUE;
	raw_values[3u] = (uint16_t)CY_CAPSENSE_BUTTON3_RX0_RAW0_VALUE;
	raw_values[4u] = (uint16_t)CY_CAPSENSE_BUTTON4_RX0_RAW0_VALUE;
#elif defined(CY_CAPSENSE_BUTTON8_RX0_RAW0_VALUE) && defined(CY_CAPSENSE_MATRIXBUTTONS0_RX1_TX1_RAW0_VALUE) && (RAWCOUNT_SENSOR_COUNT == 8u)
	/* 保留原八通道配置的逻辑顺序，便于同一源码切回食指硬件。 */
	raw_values[0u] = (uint16_t)CY_CAPSENSE_BUTTON8_RX0_RAW0_VALUE;
	raw_values[1u] = (uint16_t)CY_CAPSENSE_BUTTON7_RX0_RAW0_VALUE;
	raw_values[2u] = (uint16_t)CY_CAPSENSE_BUTTON6_RX0_RAW0_VALUE;
	raw_values[3u] = (uint16_t)CY_CAPSENSE_BUTTON5_RX0_RAW0_VALUE;
	raw_values[4u] = (uint16_t)CY_CAPSENSE_MATRIXBUTTONS0_RX0_TX0_RAW0_VALUE;
	raw_values[5u] = (uint16_t)CY_CAPSENSE_MATRIXBUTTONS0_RX0_TX1_RAW0_VALUE;
	raw_values[6u] = (uint16_t)CY_CAPSENSE_MATRIXBUTTONS0_RX1_TX0_RAW0_VALUE;
	raw_values[7u] = (uint16_t)CY_CAPSENSE_MATRIXBUTTONS0_RX1_TX1_RAW0_VALUE;
#else
#error "Unsupported CAPSENSE sensor raw-count mapping. Update Sensor_ReadRawCounts() to match cycfg_capsense.h."
#endif
}

#if 0
void Sensor_Diffcount_Update(void) {
	uint8_t sensor_index;
	uint8_t widget_id;
	uint8_t sensor_id;
	uint16_t diff_count;
	uint8_t frame[RAWCOUNT_FRAME_SIZE];

	cy_stc_capsense_widget_config_t *widget_config;
	cy_stc_capsense_sensor_context_t *sensor_context;

	static const uint8_t diffcount_magic[RAWCOUNT_MAGIC_SIZE] = {'R', 'A', 'W', 'C'};
	static const uint8_t diffcount_widget_ids[RAWCOUNT_SENSOR_COUNT] = {
		CY_CAPSENSE_BUTTON8_WDGT_ID,		CY_CAPSENSE_BUTTON7_WDGT_ID,		CY_CAPSENSE_BUTTON6_WDGT_ID,		CY_CAPSENSE_BUTTON5_WDGT_ID,
		CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID, CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID, CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID, CY_CAPSENSE_MATRIXBUTTONS0_WDGT_ID,
	};

	for (sensor_index = 0u; sensor_index < RAWCOUNT_SENSOR_COUNT; sensor_index++) {
		widget_id = diffcount_widget_ids[sensor_index];
		sensor_id = (sensor_index < 4u) ? 0u : (sensor_index - 4u);
		widget_config = &(cy_capsense_context.ptrWdConfig[widget_id]);
		sensor_context = &(widget_config->ptrSnsContext[sensor_id]);

		diff_count = (uint16_t)sensor_context->diff;
		frame[sensor_index * RAWCOUNT_BYTES_PER_SENSOR] = (uint8_t)(diff_count & 0xFFu);
		frame[(sensor_index * RAWCOUNT_BYTES_PER_SENSOR) + 1u] = (uint8_t)(diff_count >> 8u);
	}

	for (sensor_index = 0u; sensor_index < RAWCOUNT_MAGIC_SIZE; sensor_index++) {
		frame[(RAWCOUNT_SENSOR_COUNT * RAWCOUNT_BYTES_PER_SENSOR) + sensor_index] = diffcount_magic[sensor_index];
	}

	Cy_SCB_UART_PutArrayBlocking(CYBSP_UART_HW, frame, RAWCOUNT_FRAME_SIZE);
	while (!Cy_SCB_UART_IsTxComplete(CYBSP_UART_HW)) {
	}
}

void Sensor_Rawcount_Update(void)
{
	uint8_t sensor_id;
    uint8_t eltdTxID;
	uint16_t tmp_rawdiff, tmp_currentraw, tmp_rawdiff_last;
    uint16_t sns_raw_last = 0;
	cy_stc_capsense_sensor_context_t * sensor_context;
    cy_stc_capsense_widget_config_t * widget_config;
	cy_stc_capsense_widget_context_t * widget_context;
    cy_capsense_status_t statrslt;


	widget_config = &(cy_capsense_context.ptrWdConfig[CY_CAPSENSE_TOUCHPAD0_WDGT_ID]);
	widget_context = widget_config->ptrWdContext;

	for(sensor_id = CY_CAPSENSE_TOUCHPAD0_SNS0_ID; sensor_id < CY_CAPSENSE_SENSOR_COUNT; sensor_id++)
	{        
        eltdTxID = CY_CAPSENSE_TOUCHPAD0_TX0_ID + (sensor_id % 8);
		sensor_context = &(widget_config->ptrSnsContext[sensor_id]);
        //tmp_currentraw = sensor_context->raw;
		if(sensor_context->raw < (sensor_context->bsln - widget_context->nNoiseTh ))
		{
            statrslt = Cy_CapSense_SlotPinState(sensor_id, &(widget_config->ptrEltdConfig[eltdTxID]),
                                                CY_CAPSENSE_HIGHZ, &cy_capsense_context);
            // if(CY_CAPSENSE_STATUS_SUCCESS != statrslt)
            // {
                // 
                // Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n");
   
            // }
            //Cy_SCB_UART_PutString(CYBSP_UART_HW, "******finger touch******\r\n");
            //tmp_rawdiff = sensor_context->bsln - sensor_context->raw;
            // if(tmp_rawdiff < tmp_rawdiff_last) //Cm will start to increase, reset baseline
            // {
                // sensor_context->bsln = sns_raw_last - tmp_rawdiff_last;
                // widget_config->ptrBslnInv[sensor_id] = ~(sensor_context->bsln);
            // }
			// else 
            // {
                // sensor_context->raw = tmp_rawdiff + sensor_context->bsln;
                // tmp_rawdiff_last = tmp_rawdiff;
            // }
			
			//sensor_context->diff = sensor_context->raw - sensor_context->bsln;
		}
		//sns_raw_last = tmp_currentraw;
	}
	
}
#endif
/* [] END OF FILE */
