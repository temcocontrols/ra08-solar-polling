#include "sht40.h"
#include "tremo_rcc.h"
#include "tremo_gpio.h"
#include "tremo_i2c.h"
#include "tremo_delay.h"
#include "lora_config.h"
#include <stdio.h>

#define SHT40_ADDR 0x44
#define SHT40_I2C_WAIT_TIMEOUT 200000U

/* SHT4x single-shot high-repeatability command */
#define SHT40_CMD_MEASURE_HIGH 0xFD

/* Alternate preferred HUM_EN level across calls. */
static bool g_sht40_next_hum_en_high = true;

static uint8_t sht40_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static bool sht40_wait_flag(i2c_t *i2c, i2c_flag_t flag)
{
    uint32_t timeout = SHT40_I2C_WAIT_TIMEOUT;
    while ((i2c_get_flag_status(i2c, flag) != SET) && (timeout > 0U)) {
        timeout--;
    }
    return (timeout > 0U);
}

static bool sht40_read_once(float *temperature_c, float *rh_percent)
{
    uint8_t buf[6];

    i2c_master_send_start(I2C0, SHT40_ADDR, I2C_WRITE);
    i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
    if (!sht40_wait_flag(I2C0, I2C_FLAG_TRANS_EMPTY)) {
        i2c_master_send_stop(I2C0);
        return false;
    }
    i2c_send_data(I2C0, SHT40_CMD_MEASURE_HIGH);

    i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
    if (!sht40_wait_flag(I2C0, I2C_FLAG_TRANS_EMPTY)) {
        i2c_master_send_stop(I2C0);
        return false;
    }
    i2c_master_send_stop(I2C0);

    /* SHT40 high-repeatability conversion time is typically < 10 ms. */
    delay_ms(10);

    i2c_master_send_start(I2C0, SHT40_ADDR, I2C_READ);
    i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
    if (!sht40_wait_flag(I2C0, I2C_FLAG_TRANS_EMPTY)) {
        i2c_master_send_stop(I2C0);
        return false;
    }

    for (int i = 0; i < 6; ++i) {
        i2c_set_receive_mode(I2C0, (i == 5) ? I2C_NAK : I2C_ACK);
        if (!sht40_wait_flag(I2C0, I2C_FLAG_RECV_FULL)) {
            i2c_master_send_stop(I2C0);
            return false;
        }
        buf[i] = i2c_receive_data(I2C0);
        i2c_clear_flag_status(I2C0, I2C_FLAG_RECV_FULL);
    }
    i2c_master_send_stop(I2C0);

    if (sht40_crc8(buf, 2) != buf[2]) return false;
    if (sht40_crc8(buf + 3, 2) != buf[5]) return false;

    {
        uint16_t raw_t = (uint16_t)buf[0] << 8 | buf[1];
        uint16_t raw_r = (uint16_t)buf[3] << 8 | buf[4];
        *temperature_c = -45.0f + 175.0f * ((float)raw_t / 65535.0f);
        *rh_percent = 100.0f * ((float)raw_r / 65535.0f);
    }

    return true;
}

bool sht40_init(void)
{
    i2c_config_t config;

    rcc_enable_peripheral_clk(RCC_PERIPHERAL_I2C0, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);

    gpio_set_iomux(CONFIG_HUM_EN_GPIOX, CONFIG_HUM_EN_PIN, 0);
    gpio_init(CONFIG_HUM_EN_GPIOX, CONFIG_HUM_EN_PIN, GPIO_MODE_OUTPUT_PP_HIGH);
    delay_ms(5);

    gpio_set_iomux(CONFIG_RA08_I2C_GPIOX, CONFIG_RA08_I2C_SCL_PIN, 3);
    gpio_set_iomux(CONFIG_RA08_I2C_GPIOX, CONFIG_RA08_I2C_SDA_PIN, 3);

    i2c_config_init(&config);
    config.mode = I2C_MODE_MASTER;
    config.fifo_mode_en = false;
    config.settings.master.speed = I2C_SPEED_STANDARD;
    i2c_init(I2C0, &config);
    i2c_cmd(I2C0, true);
    return true;
}

bool sht40_read(float *temperature_c, float *rh_percent)
{
    if (temperature_c == NULL || rh_percent == NULL) return false;

    {
        bool first_high = g_sht40_next_hum_en_high;
        g_sht40_next_hum_en_high = !g_sht40_next_hum_en_high;

        for (int attempt = 0; attempt < 2; ++attempt) {
            bool hum_en_high = (attempt == 0) ? first_high : !first_high;
            gpio_write(CONFIG_HUM_EN_GPIOX, CONFIG_HUM_EN_PIN,
                       hum_en_high ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW);
            printf("[SHT40] HUM_EN=%s for this read (attempt %d)\r\n",
                   hum_en_high ? "HIGH" : "LOW", attempt + 1);
            delay_ms(10);

            if (sht40_read_once(temperature_c, rh_percent)) {
                return true;
            }
        }
    }

    printf("[SHT40] read failed with both HUM_EN polarities\r\n");
    return false;
}