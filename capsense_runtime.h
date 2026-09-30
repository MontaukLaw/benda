#ifndef CAPSENSE_RUNTIME_H
#define CAPSENSE_RUNTIME_H

#include "cy_capsense.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * PSoC 4100TP uses the fifth-generation low-power MSCLP block. The
 * Configurator 11.0 range is 0..16383, while Middleware 9.0 encodes the
 * requested conversion count as (Nsub - 1). Therefore 0 is not a valid
 * runtime conversion count and is deliberately rejected.
 */
#define CAPSENSE_RUNTIME_NSUB_MIN (1u)
#define CAPSENSE_RUNTIME_NSUB_MAX (16383u)
#define CAPSENSE_RUNTIME_CREF_MIN (1u)
#define CAPSENSE_RUNTIME_CREF_MAX (255u)
#define CAPSENSE_RUNTIME_MATRIX_SENSOR_COUNT (4u)

typedef struct {
	uint16_t requested_nsub;
	uint16_t actual_nsub;
	cy_capsense_status_t set_status;
	cy_capsense_status_t get_status;
} capsense_runtime_nsub_status_t;

typedef struct {
	uint8_t requested_cref;
	uint8_t actual_cref;
	cy_capsense_status_t set_status;
	cy_capsense_status_t get_status;
} capsense_runtime_cref_status_t;

typedef struct {
	uint16_t measurement_nsub;
	uint16_t hardware_nsub;
	uint16_t init_nsub;
	uint8_t ref_cdac;
	uint8_t hardware_ref_cdac;
	uint8_t comp_cdac[CAPSENSE_RUNTIME_MATRIX_SENSOR_COUNT];
	uint8_t hardware_comp_cdac[CAPSENSE_RUNTIME_MATRIX_SENSOR_COUNT];
	uint16_t sense_clk_div;
	uint16_t hardware_sense_clk_div;
	uint16_t max_count;
	uint8_t smartsense_enabled;
	uint8_t auto_calibration_enabled;
	uint8_t capsense_busy;
	uint32_t scan_time_us;
	uint32_t fps_milli;
	cy_capsense_status_t get_status;
	uint32_t persistence_status;
	uint8_t persistent_config_loaded;
	cy_capsense_status_t baseline_status;
	uint8_t baseline_update_pending;
} capsense_runtime_config_t;

void CapSenseRuntime_Init(void);
bool CapSenseRuntime_RequestNsub(uint16_t nsub);
bool CapSenseRuntime_RequestCref(uint8_t cref);
void CapSenseRuntime_ApplyPending(void);
void CapSenseRuntime_GetNsubStatus(capsense_runtime_nsub_status_t *status);
void CapSenseRuntime_GetCrefStatus(capsense_runtime_cref_status_t *status);
void CapSenseRuntime_GetConfig(capsense_runtime_config_t *config);
void CapSenseRuntime_MarkScanStarted(void);
void CapSenseRuntime_MarkScanComplete(void);
/* 矩阵诊断使用包含算法和通信开销的完整帧周期计算FPS。 */
void CapSenseRuntime_SetFramePeriodUs(uint32_t frame_period_us);
bool CapSenseRuntime_UpdateBaselineIfPending(void);

#endif
