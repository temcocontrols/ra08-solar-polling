#ifndef __SHT40_H__
#define __SHT40_H__

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SHT40 驱动接口（模板）：
   请在此实现基于 tremo_i2c 的具体写/读逻辑，或者告知我使用的 tremo_i2c API，我可替你实现完整驱动。 */
bool sht40_init(void);
bool sht40_read(float *temperature_c, float *rh_percent);

#ifdef __cplusplus
}
#endif

#endif // __SHT40_H__