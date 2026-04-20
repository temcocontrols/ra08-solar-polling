#ifndef __STCC4_H__
#define __STCC4_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool stcc4_init(void);
bool stcc4_read_co2(uint16_t *co2_ppm);

#ifdef __cplusplus
}
#endif

#endif /* __STCC4_H__ */
