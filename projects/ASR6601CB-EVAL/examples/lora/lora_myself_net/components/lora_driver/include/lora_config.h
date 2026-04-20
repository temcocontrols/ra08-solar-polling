/*
 * @Author: your name
 * @Date: 2022-04-22 15:53:39
 * @LastEditTime: 2022-04-26 16:57:14
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \ASR6601_AT_LoRaWAN\projects\ASR6601CB-EVAL\examples\my_example\lora_myself_net\components\lora_driver\include\lora_config.h
 */
#ifndef __LORA_CONFIG_H
#define __LORA_CONFIG_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "tremo_gpio.h"

#define CONFIG_GATEWAY (1)

#define CONFIG_LORA_RFSW_CTRL_GPIOX GPIOD
#define CONFIG_LORA_RFSW_CTRL_PIN GPIO_PIN_11

#define CONFIG_LORA_RFSW_VDD_GPIOX GPIOA
#define CONFIG_LORA_RFSW_VDD_PIN GPIO_PIN_10

/* RA08 board pin mapping (project baseline) */
#define CONFIG_RA08_I2C_GPIOX GPIOA
#define CONFIG_RA08_I2C_SCL_PIN GPIO_PIN_14
#define CONFIG_RA08_I2C_SDA_PIN GPIO_PIN_15

/* STCC4 CO2 sensor enable */
#define CONFIG_CO2_EN_GPIOX GPIOA
#define CONFIG_CO2_EN_PIN GPIO_PIN_5

/* SHT40 humidity sensor enable (HUM_EN) */
/* HUM_EN is wired to SWDIO (PA13) */
#define CONFIG_HUM_EN_GPIOX GPIOA
#define CONFIG_HUM_EN_PIN GPIO_PIN_13

#ifdef __cplusplus
}
#endif

#endif /* __LORA_CONFIG_H */
