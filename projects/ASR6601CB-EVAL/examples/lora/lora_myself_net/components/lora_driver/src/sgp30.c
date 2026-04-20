#include "sgp30.h"

#include "lora_config.h"
#include "tremo_delay.h"
#include "tremo_gpio.h"
#include "tremo_i2c.h"
#include "tremo_rcc.h"
#include <stddef.h>

#define SGP30_I2C_ADDR          0x58
#define SGP30_I2C_WAIT_TIMEOUT  200000U

#define SGP30_CMD_IAQ_INIT      0x2003
#define SGP30_CMD_IAQ_MEASURE   0x2008

static bool sgp30_write_cmd_u16(uint16_t cmd);

static void sgp30_i2c_prepare(void)
{
    i2c_config_t config;

    rcc_enable_peripheral_clk(RCC_PERIPHERAL_I2C0, true);
    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true);

    gpio_set_iomux(CONFIG_RA08_I2C_GPIOX, CONFIG_RA08_I2C_SCL_PIN, 3);
    gpio_set_iomux(CONFIG_RA08_I2C_GPIOX, CONFIG_RA08_I2C_SDA_PIN, 3);

    i2c_config_init(&config);
    config.mode = I2C_MODE_MASTER;
    config.fifo_mode_en = false;
    config.settings.master.speed = I2C_SPEED_STANDARD;
    i2c_init(I2C0, &config);
    i2c_cmd(I2C0, true);
}

static bool sgp30_i2c_wait_flag(i2c_t *i2c, i2c_flag_t flag)
{
    uint32_t timeout = SGP30_I2C_WAIT_TIMEOUT;
    while ((i2c_get_flag_status(i2c, flag) != SET) && (timeout > 0U)) {
        timeout--;
    }
    return (timeout > 0U);
}

static uint8_t sgp30_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    uint8_t i;
    uint8_t b;

    for (i = 0; i < len; ++i) {
        crc ^= data[i];
        for (b = 0; b < 8; ++b) {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x31U) : (uint8_t)(crc << 1);
        }
    }

    return crc;
}

static bool sgp30_write_cmd_u16(uint16_t cmd)
{
    i2c_master_send_start(I2C0, SGP30_I2C_ADDR, I2C_WRITE);
    i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
    if (!sgp30_i2c_wait_flag(I2C0, I2C_FLAG_TRANS_EMPTY)) {
        i2c_master_send_stop(I2C0);
        return false;
    }

    i2c_send_data(I2C0, (uint8_t)((cmd >> 8) & 0xFF));
    i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
    if (!sgp30_i2c_wait_flag(I2C0, I2C_FLAG_TRANS_EMPTY)) {
        i2c_master_send_stop(I2C0);
        return false;
    }

    i2c_send_data(I2C0, (uint8_t)(cmd & 0xFF));
    i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
    if (!sgp30_i2c_wait_flag(I2C0, I2C_FLAG_TRANS_EMPTY)) {
        i2c_master_send_stop(I2C0);
        return false;
    }

    i2c_master_send_stop(I2C0);
    return true;
}

static bool sgp30_read_bytes(uint8_t *buf, uint8_t len)
{
    int i;

    i2c_master_send_start(I2C0, SGP30_I2C_ADDR, I2C_READ);
    i2c_clear_flag_status(I2C0, I2C_FLAG_TRANS_EMPTY);
    if (!sgp30_i2c_wait_flag(I2C0, I2C_FLAG_TRANS_EMPTY)) {
        i2c_master_send_stop(I2C0);
        return false;
    }

    for (i = 0; i < len; ++i) {
        i2c_set_receive_mode(I2C0, (i == (len - 1)) ? I2C_NAK : I2C_ACK);
        if (!sgp30_i2c_wait_flag(I2C0, I2C_FLAG_RECV_FULL)) {
            i2c_master_send_stop(I2C0);
            return false;
        }
        buf[i] = i2c_receive_data(I2C0);
        i2c_clear_flag_status(I2C0, I2C_FLAG_RECV_FULL);
    }

    i2c_master_send_stop(I2C0);
    return true;
}

bool sgp30_init(void)
{
    sgp30_i2c_prepare();

    if (!sgp30_write_cmd_u16(SGP30_CMD_IAQ_INIT)) {
        return false;
    }

    delay_ms(20);
    return true;
}

bool sgp30_read_tvoc(uint16_t *tvoc_ppb)
{
    uint8_t buf[6];

    if (tvoc_ppb == NULL) {
        return false;
    }

    if (!sgp30_write_cmd_u16(SGP30_CMD_IAQ_MEASURE)) {
        return false;
    }

    delay_ms(20);

    if (!sgp30_read_bytes(buf, 6)) {
        return false;
    }

    if (sgp30_crc8(&buf[0], 2) != buf[2]) {
        return false;
    }
    if (sgp30_crc8(&buf[3], 2) != buf[5]) {
        return false;
    }

    *tvoc_ppb = (uint16_t)(((uint16_t)buf[3] << 8) | buf[4]);
    return true;
}
