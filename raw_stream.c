/* 20点矩阵临时采集入口：保留扫描，绕过基线更新、状态机及压力算法。 */
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
/* 复用原升级请求，软件复位后由Bootloader接管UART传输。 */
#include "iap_request.h"

/* 固定协议：四字节帧头、20个小端原始值、40字节载荷的16位累加和。 */
#define RAW20_SENSOR_COUNT (20u)
#define RAW20_PAYLOAD_SIZE (RAW20_SENSOR_COUNT * 2u)
#define RAW20_FRAME_SIZE (4u + RAW20_PAYLOAD_SIZE + 2u)

/* 配置不匹配时拒绝编译，防止漏传通道或沿用旧八点映射。 */
#if (CY_CAPSENSE_SENSOR_COUNT != RAW20_SENSOR_COUNT) || (CY_CAPSENSE_WIDGET_COUNT != 1u)
#error "RAW20 requires one 20-sensor matrix widget"
#endif

/* 保留原应用签名段，使现有构建流程继续生成可校验的CYACD2。 */
CY_SECTION(".cy_app_signature") __USED static const uint32_t cy_dfu_appSignature;

/* 串口阻塞发送原始帧；接收端只保留兼容原上位机的DFU入口。 */
static cy_stc_scb_uart_context_t raw20_uart_context;

/* 采集失败时停止发送；保留UART中断，使CapSense异常时仍可再次升级。 */
static void raw20_halt(void)
{
    for (;;)
    {
    }
}

/* 扫描中断由中间件搬运完整矩阵的硬件原始计数。 */
static void raw20_capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CY_MSCLP0_HW, &cy_capsense_context);
}

/* 在接收中断识别连续AB CD字节，扫描或发送阻塞期间也能进入DFU。 */
static void raw20_uart_isr(void)
{
    static bool previous_ab;
    const uint32_t errors = CY_SCB_UART_RX_OVERFLOW |
        CY_SCB_UART_RX_ERR_FRAME | CY_SCB_UART_RX_ERR_PARITY;
    uint32_t status = Cy_SCB_GetRxInterruptStatusMasked(CYBSP_UART_HW);

    /* 丢弃出错的接收片段，不把错误前后的字节拼成升级命令。 */
    if (0u != (status & errors))
    {
        previous_ab = false;
        Cy_SCB_UART_ClearRxFifo(CYBSP_UART_HW);
        Cy_SCB_ClearRxInterrupt(CYBSP_UART_HW, status);
        return;
    }
    while (0u != Cy_SCB_UART_GetNumInRxFifo(CYBSP_UART_HW))
    {
        uint8_t byte = (uint8_t)Cy_SCB_UART_Get(CYBSP_UART_HW);
        if (previous_ab && (byte == 0xCDu))
        {
            IapRequest_EnterBootloader(IAP_TRANSPORT_UART);
        }
        previous_ab = (byte == 0xABu);
    }
    Cy_SCB_ClearRxInterrupt(CYBSP_UART_HW, CY_SCB_UART_RX_NOT_EMPTY);
}

/* 沿用原板串口引脚及48MHz时钟，分频6、过采样8得到1000000波特率。 */
static void raw20_initialize_uart(void)
{
    /* UART优先于扫描中断，避免DFU入口依赖扫描完成或主循环进度。 */
    const cy_stc_sysint_t interrupt_config = {
        .intrSrc = CYBSP_UART_IRQ,
        .intrPriority = 2u,
    };
    Cy_SysClk_PeriphDisableDivider(CY_SYSCLK_DIV_16_BIT, 3u);
    Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 3u, 5u);
    Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, 3u);
    if (CY_SCB_UART_SUCCESS != Cy_SCB_UART_Init(
            CYBSP_UART_HW, &CYBSP_UART_config, &raw20_uart_context))
    {
        raw20_halt();
    }
    if (CY_SYSINT_SUCCESS != Cy_SysInt_Init(&interrupt_config, raw20_uart_isr))
    {
        raw20_halt();
    }
    Cy_SCB_SetRxInterruptMask(CYBSP_UART_HW, CY_SCB_UART_RX_NOT_EMPTY |
        CY_SCB_UART_RX_OVERFLOW | CY_SCB_UART_RX_ERR_FRAME | CY_SCB_UART_RX_ERR_PARITY);
    NVIC_ClearPendingIRQ(interrupt_config.intrSrc);
    NVIC_EnableIRQ(interrupt_config.intrSrc);
    Cy_SCB_UART_Enable(CYBSP_UART_HW);
}

/* 按Configurator生成配置初始化硬件，不加载Flash参数或覆盖三参数。 */
static void raw20_initialize_capsense(void)
{
    const cy_stc_sysint_t interrupt_config = {
        .intrSrc = CY_MSCLP0_LP_IRQ,
        .intrPriority = 3u,
    };
    if (CY_CAPSENSE_STATUS_SUCCESS != Cy_CapSense_Init(&cy_capsense_context))
    {
        raw20_halt();
    }
    if (CY_SYSINT_SUCCESS != Cy_SysInt_Init(&interrupt_config, raw20_capsense_isr))
    {
        raw20_halt();
    }
    NVIC_ClearPendingIRQ(interrupt_config.intrSrc);
    NVIC_EnableIRQ(interrupt_config.intrSrc);
    /* 标准Enable内部的一次初始化保留；运行循环不再处理或更新baseline。 */
    if (CY_CAPSENSE_STATUS_SUCCESS != Cy_CapSense_Enable(&cy_capsense_context))
    {
        raw20_halt();
    }
}

/* 按RX0/TX0至RX4/TX3顺序直接发送raw字段，发送期间不启动下一次扫描。 */
static void raw20_send_frame(void)
{
    uint8_t frame[RAW20_FRAME_SIZE] = {0xAAu, 0x55u, 0xAAu, 0x55u};
    uint16_t checksum = 0u;
    uint32_t index;
    for (index = 0u; index < RAW20_SENSOR_COUNT; ++index)
    {
        uint16_t raw = cy_capsense_tuner.sensorContext[index].raw;
        frame[4u + index * 2u] = (uint8_t)raw;
        frame[5u + index * 2u] = (uint8_t)(raw >> 8u);
        checksum += frame[4u + index * 2u];
        checksum += frame[5u + index * 2u];
    }
    frame[RAW20_FRAME_SIZE - 2u] = (uint8_t)checksum;
    frame[RAW20_FRAME_SIZE - 1u] = (uint8_t)(checksum >> 8u);
    Cy_SCB_UART_PutArrayBlocking(CYBSP_UART_HW, frame, sizeof(frame));
    while (!Cy_SCB_UART_IsTxComplete(CYBSP_UART_HW))
    {
    }
}

/* 主循环只执行完整扫描、等待完成和原始帧发送，不调用任何通道算法。 */
int main(void)
{
    if (CY_RSLT_SUCCESS != cybsp_init())
    {
        raw20_halt();
    }
    raw20_initialize_uart();
    __enable_irq();
    raw20_initialize_capsense();
    for (;;)
    {
        if (CY_CAPSENSE_STATUS_SUCCESS != Cy_CapSense_ScanAllSlots(&cy_capsense_context))
        {
            raw20_halt();
        }
        while (CY_CAPSENSE_NOT_BUSY != Cy_CapSense_IsBusy(&cy_capsense_context))
        {
        }
        raw20_send_frame();
    }
}
