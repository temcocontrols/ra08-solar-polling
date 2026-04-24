#include "adc_monitor.h"
#include "lora_config.h"
#include "tremo_rcc.h"
#include "tremo_gpio.h"
#include "tremo_adc.h"
#include <stdint.h>
#include <stdio.h>

/* ADC monitor source is configured in lora_config.h */
#define ADC_MONITOR_GPIOX       CONFIG_VCAP_ADC_GPIOX
#define ADC_MONITOR_PIN         CONFIG_VCAP_ADC_PIN
#define ADC_MONITOR_ADC_CHAN    ((adc_sample_chan_t)CONFIG_VCAP_ADC_CHAN)

/* Number of samples to average per reading */
#define ADC_MONITOR_SAMPLES     8U

/* Timeout guard for EOC polling (~1 ms at typical clock rates) */
#define ADC_MONITOR_TIMEOUT     100000U

/* Cached calibration coefficients (populated in adc_monitor_init) */
static float s_gain  = 1.0f;
static float s_dco   = 0.0f;

/* Read one settled raw sample for a channel using SDK-style 3-step sequence.
 * Returns true on success and writes the sample to out_raw. */
static bool adc_monitor_read_one_raw(adc_sample_chan_t chan, uint16_t *out_raw)
{
    uint32_t timeout;
    uint16_t raw0;
    uint16_t raw1;

    if (out_raw == NULL)
    {
        return false;
    }

    adc_start(true);

    timeout = ADC_MONITOR_TIMEOUT;
    while (!adc_get_interrupt_status(ADC_ISR_EOC) && (timeout > 0U))
    {
        timeout--;
    }
    if (timeout == 0U)
    {
        return false;
    }
    raw0 = adc_get_data();

    timeout = ADC_MONITOR_TIMEOUT;
    while (!adc_get_interrupt_status(ADC_ISR_EOC) && (timeout > 0U))
    {
        timeout--;
    }
    if (timeout == 0U)
    {
        return false;
    }
    raw1 = adc_get_data();

    timeout = ADC_MONITOR_TIMEOUT;
    while (!adc_get_interrupt_status(ADC_ISR_EOC) && (timeout > 0U))
    {
        timeout--;
    }
    if (timeout == 0U)
    {
        return false;
    }
    (void)adc_get_data(); /* pipeline tail sample */

    /* For repeated same-channel sequence, raw1 is usually the more settled point. */
    (void)raw0;
    *out_raw = raw1;
    return true;
}

/* Read one ADC channel and return averaged raw count; 0xFFFF on timeout. */
static uint16_t adc_monitor_read_raw_channel(adc_sample_chan_t chan, uint32_t samples)
{
    uint32_t sum;
    uint32_t i;
    uint16_t raw;

    sum = 0U;

    adc_init();
    adc_config_ref_voltage(ADC_INTERNAL_REF_VOLTAGE);
    adc_config_clock_division(20);
    adc_config_conv_mode(ADC_CONV_MODE_SINGLE);
    adc_config_sample_sequence(ADC_SAMPLE_SEQ_CHAN_0, chan);
    adc_config_sample_sequence(ADC_SAMPLE_SEQ_CHAN_1, chan);
    adc_config_sample_sequence(ADC_SAMPLE_SEQ_CHAN_2, chan);
    adc_enable(true);

    for (i = 0U; i < samples; i++)
    {
        if (!adc_monitor_read_one_raw(chan, &raw))
        {
            adc_start(false);
            adc_enable(false);
            return 0xFFFFU;
        }

        sum += raw;
    }

    adc_start(false);
    adc_enable(false);

    return (uint16_t)(sum / samples);
}

/* One-time diagnostic: print ADC channel map to locate IO8 actual channel. */
static void adc_monitor_scan_channels_once(void)
{
    uint8_t chan;

    printf("[ADC] scan begin: gain=%.6f dco=%.6f\r\n", s_gain, s_dco);
    for (chan = 1U; chan <= 15U; chan++)
    {
        uint16_t raw;
        float voltage;

        raw = adc_monitor_read_raw_channel((adc_sample_chan_t)chan, 4U);
        if (raw == 0xFFFFU)
        {
            printf("[ADC] chan=%u timeout\r\n", (unsigned)chan);
            continue;
        }

        voltage = ((1.2f / 4096.0f) * (float)raw - s_dco) / s_gain;
        printf("[ADC] chan=%u raw=%u v=%.3fV\r\n", (unsigned)chan, (unsigned)raw, voltage);
    }
    printf("[ADC] scan end\r\n");
}

void adc_monitor_init(void)
{
    /* Enable required peripheral clocks */
    rcc_set_adc_clk_source(RCC_ADC_CLK_SOURCE_RCO48M);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_ADC,   true);

    /* Force pin back to GPIO function before analog mode to avoid stale iomux. */
    gpio_set_iomux(ADC_MONITOR_GPIOX, ADC_MONITOR_PIN, 0);

    /* Configure selected pin as analog input (no pull, no drive) */
    gpio_init(ADC_MONITOR_GPIOX, ADC_MONITOR_PIN, GPIO_MODE_ANALOG);

    /* Read factory calibration values for single-ended mode */
    adc_get_calibration_value(false, &s_gain, &s_dco);

    /* Print one-time channel scan to verify IO8->ADC channel mapping in hardware. */
    adc_monitor_scan_channels_once();
}

float adc_monitor_read_voltage(void)
{
    uint32_t sum;
    uint32_t i;
    uint16_t raw_sample;
    float raw;
    float voltage;

    sum = 0U;

    adc_init();
    adc_config_ref_voltage(ADC_INTERNAL_REF_VOLTAGE);
    adc_config_clock_division(20);  /* ~150 kHz sample rate with RCO48M */
    adc_config_conv_mode(ADC_CONV_MODE_SINGLE);
    adc_config_sample_sequence(ADC_SAMPLE_SEQ_CHAN_0, ADC_MONITOR_ADC_CHAN);
    adc_config_sample_sequence(ADC_SAMPLE_SEQ_CHAN_1, ADC_MONITOR_ADC_CHAN);
    adc_config_sample_sequence(ADC_SAMPLE_SEQ_CHAN_2, ADC_MONITOR_ADC_CHAN);
    adc_enable(true);

    for (i = 0U; i < ADC_MONITOR_SAMPLES; i++)
    {
        if (!adc_monitor_read_one_raw(ADC_MONITOR_ADC_CHAN, &raw_sample))
        {
            /* Timeout: abort and return error sentinel */
            adc_start(false);
            adc_enable(false);
            return -1.0f;
        }

        sum += raw_sample;
    }

    adc_start(false);
    adc_enable(false);

    /* Average raw counts */
    raw = (float)(sum / ADC_MONITOR_SAMPLES);

    /* Apply factory calibration: V = (raw * Vref/4096 - dco) / gain
     * Internal reference Vref = 1.2 V, 12-bit resolution (4096 counts) */
    voltage = ((1.2f / 4096.0f) * raw - s_dco) / s_gain;

    return voltage;
}
