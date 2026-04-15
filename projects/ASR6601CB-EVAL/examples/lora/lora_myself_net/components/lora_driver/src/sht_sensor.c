#include "sht_sensor.h"

#if SHT_SENSOR_TYPE == 3
#include "sht3x.h"
#elif SHT_SENSOR_TYPE == 40
#include "sht40.h"
#else
#error "Unsupported SHT_SENSOR_TYPE"
#endif

bool sht_sensor_init(void)
{
#if SHT_SENSOR_TYPE == 3
    /* sht3x_init has void return type; call it and return true */
    sht3x_init();
    return true;
#elif SHT_SENSOR_TYPE == 40
    return sht40_init();
#endif
}

bool sht_sensor_read(float *temperature_c, float *rh_percent)
{
#if SHT_SENSOR_TYPE == 3
    return sht3x_read(temperature_c, rh_percent);
#elif SHT_SENSOR_TYPE == 40
    return sht40_read(temperature_c, rh_percent);
#endif
}
