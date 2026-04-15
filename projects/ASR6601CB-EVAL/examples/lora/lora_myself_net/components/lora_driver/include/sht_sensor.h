#ifndef __SHT_SENSOR_H__
#define __SHT_SENSOR_H__

#include <stdbool.h>

#ifndef SHT_SENSOR_TYPE
#define SHT_SENSOR_TYPE 3  /* 3 = SHT3X (默认), 40 = SHT40 */
#endif

#ifdef __cplusplus
extern "C" {
#endif

bool sht_sensor_init(void);
/* 读取温度(°C)和相对湿度(%)，返回 true 表示读到有效值；失败时调用者可上报 0/0 */
bool sht_sensor_read(float *temperature_c, float *rh_percent);

#ifdef __cplusplus
}
#endif

#endif // __SHT_SENSOR_H__