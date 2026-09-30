#ifndef SENSOR_BUS_PROTOCOL_H
#define SENSOR_BUS_PROTOCOL_H

#include <stdint.h>

/*
 * IIC slave data frame format — matching reference protocol (55 AA header).
 *
 * Frame layout (little-endian, 66 bytes total):
 *   [0-1]   Header:  55 AA
 *   [2]     Version: 03
 *   [3]     Command: 85 (UpData)
 *   [4]     Sequence number
 *   [5]     Active channel count
 *   [6-41]  Channel data: ALL_SENSOR_NUM × 3 bytes (uint24, 12 slots)
 *   [42-47] Force slot 0: fingertip nf(2) tf(2) dir(2) — uint16 each
 *   [48-53] Force slot 1: finger pad  nf(2) tf(2) dir(2) — uint16 each (all zero)
 *   [54-59] SPROX: 2 × 3 bytes
 *   [60-62] MPROX: 1 × 3 bytes
 *   [63]    State: 1 byte
 *   [64-65] Checksum: uint16 sum of bytes 0..63
 */

#define SENSOR_BUS_MAGIC0                 (0x55u)
#define SENSOR_BUS_MAGIC1                 (0xAAu)
#define SENSOR_BUS_VERSION                (0x03u)
#define SENSOR_BUS_CMD_UP_DATA            (0x85u)

#define SENSOR_BUS_HEADER_SIZE            (6u)
#define SENSOR_BUS_ALL_SENSOR_NUM         (12u)
#define SENSOR_BUS_N_FORCE_NUM            (2u)
#define SENSOR_BUS_SPROX_NUM              (2u)
#define SENSOR_BUS_MPROX_NUM              (1u)
#define SENSOR_BUS_CHANNEL_VALUE_MAX      (0xFFFFFFu)
#define SENSOR_BUS_DIR_INVALID            (0xFFFFu)

#define SENSOR_BUS_FRAME_SIZE \
    (SENSOR_BUS_HEADER_SIZE + \
     (SENSOR_BUS_ALL_SENSOR_NUM * 3u) + \
     (SENSOR_BUS_N_FORCE_NUM * 6u) + \
     (SENSOR_BUS_SPROX_NUM * 3u) + \
     (SENSOR_BUS_MPROX_NUM * 3u) + \
     1u + 2u)
/* = 6 + 36 + 12 + 6 + 3 + 1 + 2 = 66 */

/*
 * UART aggregate frame: wraps one or more IIC slave frames.
 *   55 AA | version | frame_count | payload_len(LE16) | payload
 */
#define SENSOR_UART_MAGIC0                (0x55u)
#define SENSOR_UART_MAGIC1                (0xAAu)
#define SENSOR_UART_HEADER_SIZE           (6u)
#define SENSOR_BUS_MAX_DEVICE_COUNT       (5u)
#define SENSOR_UART_MAX_FRAME_SIZE \
    (SENSOR_UART_HEADER_SIZE + (SENSOR_BUS_MAX_DEVICE_COUNT * SENSOR_BUS_FRAME_SIZE))

/* Largest frame buffer needed; EZI2C slave uses this for ping-pong buffers. */
#define SENSOR_BUS_MAX_FRAME_SIZE         SENSOR_BUS_FRAME_SIZE

/* Helper: write uint16 LE. */
static inline void SensorBus_WriteU16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8u);
}

/* Helper: write uint24 LE (3 bytes). */
static inline void SensorBus_WriteU24(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8u);
    dst[2] = (uint8_t)(value >> 16u);
}

#endif
