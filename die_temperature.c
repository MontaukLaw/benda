#include "die_temperature.h"

#include "cy_pdl.h"

#define DIE_TEMPERATURE_PERI_DIVIDER_NUM (4u)
#define DIE_TEMPERATURE_CLOCK_DIVIDER (29u)
#define DIE_TEMPERATURE_VREF_MV (1200u)
#define DIE_TEMPERATURE_SAMPLE_TIME (18u)

static const cy_stc_sar_channel_config_t die_temperature_channel_config = {
	.addr = CY_SAR_ADDR_SARMUX_DIE_TEMP,
	.differential = false,
	.resolution = CY_SAR_MAX_RES,
	.avgEn = true,
	.sampleTimeSel = CY_SAR_SAMPLE_TIME_0,
	.rangeIntrEn = false,
	.satIntrEn = false,
};

static const cy_stc_sar_routing_config_t die_temperature_routing_config = {
	.muxSwitch = CY_SAR_MUX_FW_TEMP_VPLUS | CY_SAR_MUX_FW_VSSA_VMINUS,
	.muxSwitchHwCtrl = CY_SAR_MUX_HW_CTRL_TEMP | CY_SAR_MUX_HW_CTRL_VSSA,
};

static int16_t die_temperature_tenth_celsius;
static bool die_temperature_initialized;
static bool die_temperature_conversion_pending;

static const cy_stc_sar_config_t die_temperature_sar_config = {
	.vrefSel = CY_SAR_VREF_SEL_BGR,
	.vrefBypCapEn = false,
	.negSel = CY_SAR_NEG_SEL_VSSA_KELVIN,
	.negVref = CY_SAR_NEGVREF_HW,
	.boostPump = false,
	.power = CY_SAR_NORMAL_PWR,
	.sarMuxDsEn = false,
	.switchDisable = false,
	.subResolution = CY_SAR_SUB_RESOLUTION_8B,
	.leftAlign = false,
	.singleEndedSigned = true,
	.differentialSigned = true,
	.avgCnt = CY_SAR_AVG_CNT_256,
	.avgShift = true,
	.trigMode = CY_SAR_TRIGGER_MODE_FW_ONLY,
	.eosEn = false,
	.sampleTime0 = DIE_TEMPERATURE_SAMPLE_TIME,
	.sampleTime1 = DIE_TEMPERATURE_SAMPLE_TIME,
	.sampleTime2 = DIE_TEMPERATURE_SAMPLE_TIME,
	.sampleTime3 = DIE_TEMPERATURE_SAMPLE_TIME,
	.rangeThresLow = 0u,
	.rangeThresHigh = 0u,
	.rangeCond = CY_SAR_RANGE_COND_BELOW,
	.chanEn = 0u,
	.channelConfig = {
		[CY_SAR_INJ_CHANNEL] = &die_temperature_channel_config,
	},
	.routingConfig = &die_temperature_routing_config,
	.vrefMvValue = DIE_TEMPERATURE_VREF_MV,
};

/* 初始化独立SAR时钟和芯片内部温度注入通道。 */
bool DieTemperature_Init(void) {
	cy_en_sar_status_t status;

	Cy_SysClk_PeriphDisableDivider(CY_SYSCLK_DIV_16_BIT, DIE_TEMPERATURE_PERI_DIVIDER_NUM);
	Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, DIE_TEMPERATURE_PERI_DIVIDER_NUM, DIE_TEMPERATURE_CLOCK_DIVIDER);
	Cy_SysClk_PeriphAssignDivider(PCLK_PASS0_CLOCK_SAR, CY_SYSCLK_DIV_16_BIT, DIE_TEMPERATURE_PERI_DIVIDER_NUM);
	Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, DIE_TEMPERATURE_PERI_DIVIDER_NUM);

	status = Cy_SAR_Init(SAR0, &die_temperature_sar_config);
	if (status != CY_SAR_SUCCESS) {
		die_temperature_initialized = false;
		die_temperature_conversion_pending = false;
		return false;
	}

	Cy_SAR_Enable(SAR0);
	Cy_SAR_ClearInterrupt(SAR0, CY_SAR_INTR_INJ_EOC);
	die_temperature_initialized = true;
	die_temperature_conversion_pending = false;
	return true;
}

/* 在没有未完成转换时触发一次非阻塞温度采样。 */
bool DieTemperature_StartMeasurement(void) {
	if (!die_temperature_initialized || die_temperature_conversion_pending) {
		return false;
	}

	Cy_SAR_ClearInterrupt(SAR0, CY_SAR_INTR_INJ_EOC);
	Cy_SAR_EnableInjection(SAR0, false);
	die_temperature_conversion_pending = true;
	return true;
}

/* 轮询SAR完成状态，未完成时立即返回而不阻塞CapSense主循环。 */
bool DieTemperature_TryReadTenthDegreeC(int16_t *temperature_tenth_celsius) {
	int16_t adc_counts;

	if (!die_temperature_conversion_pending ||
		(Cy_SAR_IsEndConversion(SAR0, CY_SAR_RETURN_STATUS_INJ) != CY_SAR_SUCCESS)) {
		return false;
	}

	adc_counts = Cy_SAR_GetResult16(SAR0, CY_SAR_INJ_CHANNEL);
	die_temperature_tenth_celsius = Cy_SAR_CountsTo_tenthDegreeC(SAR0, CY_SAR_INJ_CHANNEL, adc_counts);
	die_temperature_conversion_pending = false;
	if (temperature_tenth_celsius != NULL) {
		*temperature_tenth_celsius = die_temperature_tenth_celsius;
	}
	return true;
}

/* 返回最近一次成功转换值，尚未完成首帧时为零。 */
int16_t DieTemperature_GetLatestTenthDegreeC(void) {
	return die_temperature_tenth_celsius;
}
