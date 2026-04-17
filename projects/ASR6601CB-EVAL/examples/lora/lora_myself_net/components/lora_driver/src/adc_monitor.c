#include "adc_monitor.h"
#include "tremo_rcc.h"
#include "tremo_gpio.h"
#include "tremo_adc.h"
#include <stdint.h>

/* IO8 on Ra-08 = PA8, which maps to ADC channel 1 */
#define ADC_MONITOR_GPIOX       GPIOA
#define ADC_MONITOR_PIN         GPIO_PIN_8
#define ADC_MONITOR_ADC_CHAN    ADC_SAMPLE_CHAN_1

/* Number of samples to average per reading */
#define ADC_MONITOR_SAMPLES     8U

/* Timeout guard for EOC polling (~1 ms at typical clock rates) */
#define ADC_MONITOR_TIMEOUT     100000U

/* Cached calibration coefficients (populated in adc_monitor_init) */
static float s_gain  = 1.0f;
static float s_dco   = 0.0f;

void adc_monitor_init(void)
{
    /* Enable required peripheral clocks */
    rcc_set_adc_clk_source(RCC_ADC_CLK_SOURCE_RCO48M);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_ADC,   true);

    /* Configure PA8 as analog input (no pull, no drive) */
    gpio_init(ADC_MONITOR_GPIOX, ADC_MONITOR_PIN, GPIO_MODE_ANALOG);

    /* Read factory calibration values for single-ended mode */
    adc_get_calibration_value(false, &s_gain, &s_dco);
}

float adc_monitor_read_voltage(void)
{
    uint32_t sum;
    uint32_t timeout;
    uint32_t i;
    float raw;
    float voltage;

    sum = 0U;
    timeout = 0U;

    adc_init();
    adc_config_clock_division(20);  /* ~150 kHz sample rate with RCO48M */
    adc_config_conv_mode(ADC_CONV_MODE_SINGLE);
    adc_config_sample_sequence(ADC_SAMPLE_SEQ_CHAN_0, ADC_MONITOR_ADC_CHAN);
    adc_enable(true);

    for (i = 0U; i < ADC_MONITOR_SAMPLES; i++)
    {
        adc_start(true);

        timeout = ADC_MONITOR_TIMEOUT;
        while (!adc_get_interrupt_status(ADC_ISR_EOC) && (timeout > 0U))
        {
            timeout--;
        }

        if (timeout == 0U)
        {
            /* Timeout: abort and return error sentinel */
            adc_start(false);
            adc_enable(false);
            return -1.0f;
        }

        sum += adc_get_data();
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
