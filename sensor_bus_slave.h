#ifndef SENSOR_BUS_SLAVE_H
#define SENSOR_BUS_SLAVE_H

#include <stdint.h>

/* 7-bit I2C address and logical ID are intentionally independent. */
#define SENSOR_SLAVE_I2C_ADDRESS          (0x21u)
#define SENSOR_SLAVE_DEVICE_ID            (1u)

void SensorBusSlave_Init(void);
/* Publish the latest algorithm snapshot in the I2C live-data frame. */
void SensorBusSlave_Update(void);

#endif
