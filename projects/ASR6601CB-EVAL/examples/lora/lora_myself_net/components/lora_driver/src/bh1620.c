#include "bh1620.h"

#include "lora_config.h"
#include "tremo_delay.h"
#include "tremo_adc.h"
#include "tremo_gpio.h"
#include "tremo_rcc.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define BH1620_ADC_TIMEOUT       100000U
#define BH1620_ADC_AVG_SAMPLES   8U

static float s_gain = 1.0f;
static float s_dco = 0.0f;

static uint16_t bh1620_enable_level(void)
{
#if CONFIG_BH1620_EN_ACTIVE_HIGH
    return GPIO_LEVEL_HIGH;
#else
    return GPIO_LEVEL_LOW;
#endif
}

static uint16_t bh1620_disable_level(void)
{
#if CONFIG_BH1620_EN_ACTIVE_HIGH
    return GPIO_LEVEL_LOW;
#else
    return GPIO_LEVEL_HIGH;
#endif
}

static bool bh1620_read_one_raw(uint16_t *out_raw)
{
    unsigned int timeout;

    if (out_raw == NULL) {
        return false;
    }

    adc_start(true);

    timeout = BH1620_ADC_TIMEOUT;
    while (!adc_get_interrupt_status(ADC_ISR_EOC) && (timeout > 0U)) {
        timeout--;
    }
    if (timeout == 0U) {
        return false;
    }
    (void)adc_get_data();

    timeout = BH1620_ADC_TIMEOUT;
    while (!adc_get_interrupt_status(ADC_ISR_EOC) && (timeout > 0U)) {
        timeout--;
    }
    if (timeout == 0U) {
        return false;
    }
    *out_raw = adc_get_data();

    timeout = BH1620_ADC_TIMEOUT;
    while (!adc_get_interrupt_status(ADC_ISR_EOC) && (timeout > 0U)) {
        timeout--;
    }
    if (timeout == 0U) {
        return false;
    }
    (void)adc_get_data();

    return true;
}

static uint16_t bh1620_read_raw_average(void)
{
    unsigned int i;
    unsigned int sum = 0U;
    uint16_t raw;

    adc_init();
    adc_config_ref_voltage(ADC_INTERNAL_REF_VOLTAGE);
    adc_config_clock_division(20);
    adc_config_conv_mode(ADC_CONV_MODE_SINGLE);
    adc_config_sample_sequence(ADC_SAMPLE_SEQ_CHAN_0, (adc_sample_chan_t)CONFIG_BH1620_ADC_CHAN);
    adc_config_sample_sequence(ADC_SAMPLE_SEQ_CHAN_1, (adc_sample_chan_t)CONFIG_BH1620_ADC_CHAN);
    adc_config_sample_sequence(ADC_SAMPLE_SEQ_CHAN_2, (adc_sample_chan_t)CONFIG_BH1620_ADC_CHAN);
    adc_enable(true);

    for (i = 0U; i < BH1620_ADC_AVG_SAMPLES; i++) {
        if (!bh1620_read_one_raw(&raw)) {
            adc_start(false);
            adc_enable(false);
            return 0xFFFFU;
        }
        sum += raw;
    }

    adc_start(false);
    adc_enable(false);

    return (uint16_t)(sum / BH1620_ADC_AVG_SAMPLES);
}

bool bh1620_init(void)
{
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOC, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOD, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_ADC, true);
    rcc_set_adc_clk_source(RCC_ADC_CLK_SOURCE_RCO48M);

    gpio_set_iomux(CONFIG_BH1620_EN_GPIOX, CONFIG_BH1620_EN_PIN, 0);
    gpio_init(CONFIG_BH1620_EN_GPIOX, CONFIG_BH1620_EN_PIN, GPIO_MODE_OUTPUT_PP_HIGH);
    gpio_write(CONFIG_BH1620_EN_GPIOX, CONFIG_BH1620_EN_PIN, bh1620_disable_level());

    gpio_set_iomux(CONFIG_BH1620_ADC_GPIOX, CONFIG_BH1620_ADC_PIN, 0);
    gpio_init(CONFIG_BH1620_ADC_GPIOX, CONFIG_BH1620_ADC_PIN, GPIO_MODE_ANALOG);

    adc_get_calibration_value(false, &s_gain, &s_dco);

    return true;
}

bool bh1620_read_lux(uint16_t *lux)
{
    uint16_t raw;
    float voltage;
    float current_ua;
    float lux_f;

    if (lux == NULL) {
        return false;
    }

    gpio_write(CONFIG_BH1620_EN_GPIOX, CONFIG_BH1620_EN_PIN, bh1620_enable_level());
    delay_ms(CONFIG_BH1620_POWER_ON_DELAY_MS);

    raw = bh1620_read_raw_average();
    gpio_write(CONFIG_BH1620_EN_GPIOX, CONFIG_BH1620_EN_PIN, bh1620_disable_level());

    if (raw == 0xFFFFU) {
        return false;
    }

    voltage = ((1.2f / 4096.0f) * (float)raw - s_dco) / s_gain;
    if (voltage < 0.0f) {
        voltage = 0.0f;
    }

    /* BH1620FVC is analog current-source output. The external resistor converts
     * current to voltage: V = Iout * R. Lux conversion depends on Rsense and
     * sensor sensitivity (uA/lx), exposed as board-level macros for calibration. */
    current_ua = (voltage * 1000000.0f) / CONFIG_BH1620_RSENSE_OHM;
    lux_f = current_ua / CONFIG_BH1620_UA_PER_LUX;

    if (lux_f < 0.0f) {
        lux_f = 0.0f;
    }
    if (lux_f > 65535.0f) {
        lux_f = 65535.0f;
    }

    *lux = (uint16_t)lux_f;

    return true;
}
