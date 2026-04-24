#ifndef __ADC_MONITOR_H
#define __ADC_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialize ADC for supercapacitor voltage monitoring on IO8 (PA8).
 *         Call once before using adc_monitor_read_voltage().
 */
void adc_monitor_init(void);

/**
 * @brief  Read the supercapacitor voltage on configured ADC pin/channel
 *         (see lora_config.h CONFIG_VCAP_ADC_*).
 *         Averages multiple samples and applies factory calibration.
 * @return Voltage in volts at the ADC pin.
 *         Returns -1.0f on timeout error.
 */
float adc_monitor_read_voltage(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_MONITOR_H */
