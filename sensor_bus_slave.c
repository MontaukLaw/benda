#include "sensor_bus_slave.h"

#include "cy_pdl.h"
#include "sensor_bus_protocol.h"
#include "sensor_algorithm.h"
#include "force_vector_cal.h"

static uint16_t SensorBus_ScaleForceX100(float value)
{
    float scaled;

    if (!(value > 0.0f)) {
        return 0u;
    }

    scaled = (value * 100.0f) + 0.5f;
    if (scaled >= 65535.0f) {
        return 65535u;
    }

    return (uint16_t)scaled;
}

static uint32_t SensorBus_ChannelValue(const sensor_algorithm_snapshot_t *snapshot,
                                       uint32_t channel_index)
{
    uint32_t value = (uint32_t)snapshot->channels[channel_index].signal * 200u;
    /* 恢复状态的 Signal 已被清零，因此不能继续向总线报告接触有效。 */
    uint32_t active = SensorAlgorithm_IsContactActiveState(snapshot->states[channel_index]) ? 1u : 0u;

    if (value > SENSOR_BUS_CHANNEL_VALUE_MAX) {
        value = SENSOR_BUS_CHANNEL_VALUE_MAX;
    }
    return (value << 1u) | active;
}

static uint16_t SensorBus_DirectionDegrees(float theta_rad)
{
    float degrees = theta_rad * (180.0f / 3.14159265f);

    while (degrees < 0.0f) {
        degrees += 360.0f;
    }
    while (degrees >= 360.0f) {
        degrees -= 360.0f;
    }

    return (uint16_t)(degrees + 0.5f);
}

static uint16_t SensorBus_Checksum(const uint8_t *frame, uint32_t length)
{
    uint32_t i;
    uint16_t sum = 0u;

    for (i = 0u; i < length; i++) {
        sum = (uint16_t)(sum + frame[i]);
    }

    return sum;
}

#define SENSOR_SLAVE_HW                   SCB1
#define SENSOR_SLAVE_IRQ                  scb_1_interrupt_IRQn
#define SENSOR_SLAVE_SCL_PORT             GPIO_PRT5
#define SENSOR_SLAVE_SCL_PIN              (1u)
#define SENSOR_SLAVE_SCL_HSIOM            P5_1_SCB1_I2C_SCL
#define SENSOR_SLAVE_SDA_PORT             GPIO_PRT5
#define SENSOR_SLAVE_SDA_PIN              (2u)
#define SENSOR_SLAVE_SDA_HSIOM            P5_2_SCB1_I2C_SDA
#define SENSOR_SLAVE_CLOCK_DIV_NUM        (1u)
#define SENSOR_SLAVE_CLOCK_DIV_VALUE      (5u)

static cy_stc_scb_ezi2c_context_t slave_context;
static uint8_t slave_frames[2][SENSOR_BUS_MAX_FRAME_SIZE];
static uint8_t active_frame;
static uint8_t sequence;

static const cy_stc_scb_ezi2c_config_t slave_config = {
    .numberOfAddresses = CY_SCB_EZI2C_ONE_ADDRESS,
    .slaveAddress1 = SENSOR_SLAVE_I2C_ADDRESS,
    .slaveAddress2 = 0u,
    .subAddressSize = CY_SCB_EZI2C_SUB_ADDR8_BITS,
    .enableWakeFromSleep = false,
};

static void SensorBusSlave_Isr(void)
{
    Cy_SCB_EZI2C_Interrupt(SENSOR_SLAVE_HW, &slave_context);
}

void SensorBusSlave_Init(void)
{
    const cy_stc_gpio_pin_config_t scl_config = {
        .outVal = 1u, .driveMode = CY_GPIO_DM_OD_DRIVESLOW,
        .hsiom = SENSOR_SLAVE_SCL_HSIOM, .intEdge = CY_GPIO_INTR_DISABLE,
        .vtrip = CY_GPIO_VTRIP_CMOS, .slewRate = CY_GPIO_SLEW_FAST,
    };
    const cy_stc_gpio_pin_config_t sda_config = {
        .outVal = 1u, .driveMode = CY_GPIO_DM_OD_DRIVESLOW,
        .hsiom = SENSOR_SLAVE_SDA_HSIOM, .intEdge = CY_GPIO_INTR_DISABLE,
        .vtrip = CY_GPIO_VTRIP_CMOS, .slewRate = CY_GPIO_SLEW_FAST,
    };
    const cy_stc_sysint_t irq_config = {
        .intrSrc = SENSOR_SLAVE_IRQ, .intrPriority = 2u,
    };

    Cy_SysClk_PeriphAssignDivider(PCLK_SCB1_CLOCK, CY_SYSCLK_DIV_16_BIT,
                                  SENSOR_SLAVE_CLOCK_DIV_NUM);
    Cy_SysClk_PeriphDisableDivider(CY_SYSCLK_DIV_16_BIT, SENSOR_SLAVE_CLOCK_DIV_NUM);
    Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, SENSOR_SLAVE_CLOCK_DIV_NUM,
                               SENSOR_SLAVE_CLOCK_DIV_VALUE);
    Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, SENSOR_SLAVE_CLOCK_DIV_NUM);
    Cy_GPIO_Pin_Init(SENSOR_SLAVE_SCL_PORT, SENSOR_SLAVE_SCL_PIN, &scl_config);
    Cy_GPIO_Pin_Init(SENSOR_SLAVE_SDA_PORT, SENSOR_SLAVE_SDA_PIN, &sda_config);
    Cy_SCB_EZI2C_Init(SENSOR_SLAVE_HW, &slave_config, &slave_context);
    Cy_SysInt_Init(&irq_config, SensorBusSlave_Isr);
    NVIC_ClearPendingIRQ(SENSOR_SLAVE_IRQ);
    NVIC_EnableIRQ(SENSOR_SLAVE_IRQ);
    Cy_SCB_EZI2C_SetBuffer1(SENSOR_SLAVE_HW, slave_frames[0], SENSOR_BUS_MAX_FRAME_SIZE,
                            SENSOR_BUS_MAX_FRAME_SIZE, &slave_context);
    Cy_SCB_EZI2C_Enable(SENSOR_SLAVE_HW);
}

void SensorBusSlave_Update(void)
{
    sensor_algorithm_snapshot_t snapshot;
    uint8_t next = (uint8_t)(active_frame ^ 1u);
    uint8_t *frame = slave_frames[next];
    uint16_t offset;
    uint8_t i;
    float normal_force;
    float tangential_force;
    float direction_rad;
    uint16_t direction;
    uint16_t checksum;

    SensorAlgorithm_GetLatestSnapshot(&snapshot);
    SensorAlgorithm_GetCalibratedForces(&normal_force, &tangential_force,
                                        &direction_rad);

    memset(frame, 0, SENSOR_BUS_FRAME_SIZE);

    /* Header: 55 AA | 03 | 85 | seq | active_ch_count */
    frame[0] = SENSOR_BUS_MAGIC0;
    frame[1] = SENSOR_BUS_MAGIC1;
    frame[2] = SENSOR_BUS_VERSION;
    frame[3] = SENSOR_BUS_CMD_UP_DATA;
    frame[4] = sequence++;
    frame[5] = SENSOR_ALGO_CHANNEL_COUNT;

    offset = SENSOR_BUS_HEADER_SIZE;

    /* Channel data: ALL_SENSOR_NUM slots × 3 bytes each.
     * Active channels filled from algorithm snapshot; remaining zero-padded. */
    for (i = 0u; i < SENSOR_ALGO_CHANNEL_COUNT; i++) {
        SensorBus_WriteU24(&frame[offset],
                           SensorBus_ChannelValue(&snapshot, i));
        offset += 3u;
    }
    /* Remaining slots (SENSOR_ALGO_CHANNEL_COUNT .. ALL_SENSOR_NUM-1)
     * stay zero from memset. */
    offset = SENSOR_BUS_HEADER_SIZE + (SENSOR_BUS_ALL_SENSOR_NUM * 3u);

    /* Force slot 0: fingertip (normal, tangential, direction) — uint16 each */
    SensorBus_WriteU16(&frame[offset], SensorBus_ScaleForceX100(normal_force));
    offset += 2u;
    SensorBus_WriteU16(&frame[offset], SensorBus_ScaleForceX100(tangential_force));
    offset += 2u;
    if (g_force_vector_contact_active && (tangential_force > 0.0f)) {
        direction = SensorBus_DirectionDegrees(direction_rad);
    } else {
        direction = SENSOR_BUS_DIR_INVALID;
    }
    SensorBus_WriteU16(&frame[offset], direction);
    offset += 2u;

    /* Force slot 1: finger pad — all zeros (already cleared by memset) */
    offset += 6u;

    /* SPROX: 2 × 3 bytes — all zeros */
    offset += (SENSOR_BUS_SPROX_NUM * 3u);

    /* MPROX: 1 × 3 bytes — all zeros */
    offset += (SENSOR_BUS_MPROX_NUM * 3u);

    /* State: 1 byte — zero */
    offset += 1u;

    /* Checksum: uint16 sum of bytes 0..63 */
    checksum = SensorBus_Checksum(frame, SENSOR_BUS_FRAME_SIZE - 2u);

    SensorBus_WriteU16(&frame[offset], checksum);

    Cy_SCB_EZI2C_SetBuffer1(SENSOR_SLAVE_HW, frame, SENSOR_BUS_MAX_FRAME_SIZE,
                            SENSOR_BUS_MAX_FRAME_SIZE, &slave_context);
    active_frame = next;
}
