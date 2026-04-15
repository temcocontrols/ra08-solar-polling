#include "sht40.h"
#include "tremo_rcc.h"
#include "tremo_gpio.h"
#include "tremo_i2c.h"
#include "tremo_delay.h"
#include <stdio.h>
#include <string.h>

// SHT40 I2C address
#define SHT40_ADDR 0x44

/* Single shot high repeatability command for SHT40: 0x2C06 */
static const uint8_t SHT40_CMD_SINGLE_HIGH[2] = {0x2C, 0x06};

/* CRC8 calculation (polynomial 0x31) */
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

bool sht40_init(void)
{
    /* Enable clocks and configure I2C0 pins (PA14=SCL, PA15=SDA) */
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_I2C0, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);

    gpio_set_iomux(GPIOA, GPIO_PIN_14, 3);
    gpio_set_iomux(GPIOA, GPIO_PIN_15, 3);

    i2c_config_t config;
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

    // send single shot command
    i2c_master_send_start(I2C0, SHT40_ADDR << 1, I2C_WRITE);
    i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
    while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET) ;
    i2c_send_data(I2C0, SHT40_CMD_SINGLE_HIGH[0]);
    i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
    while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET) ;
    i2c_send_data(I2C0, SHT40_CMD_SINGLE_HIGH[1]);
    i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
    while (i2c_get_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY) != SET) ;
    i2c_master_send_stop(I2C0);

    // conversion time ~15 ms
    delay_ms(20);

    // read 6 bytes
    i2c_master_send_start(I2C0, SHT40_ADDR << 1, I2C_READ);
    uint8_t buf[6];
    for (int i = 0; i < 6; ++i) {
        if (i == 5) i2c_set_receive_mode(I2C0, I2C_NAK);
        else i2c_set_receive_mode(I2C0, I2C_ACK);
        while (i2c_get_flag_status(I2C0, I2C_FLAG_RECV_FULL) != SET) ;
        buf[i] = i2c_receive_data(I2C0);
        i2c_clear_flag_status(I2C0, I2C_FLAG_RECV_FULL);
    }
    i2c_master_send_stop(I2C0);

    // check CRCs
    if (sht40_crc8(buf, 2) != buf[2]) return false;
    if (sht40_crc8(buf + 3, 2) != buf[5]) return false;

    uint16_t raw_t = (uint16_t)buf[0] << 8 | buf[1];
    uint16_t raw_r = (uint16_t)buf[3] << 8 | buf[4];

    *temperature_c = -45.0f + 175.0f * ((float)raw_t / 65535.0f);
    *rh_percent = 100.0f * ((float)raw_r / 65535.0f);

    return true;
}