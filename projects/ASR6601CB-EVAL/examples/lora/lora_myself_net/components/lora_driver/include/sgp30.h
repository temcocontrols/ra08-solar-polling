#ifndef __SGP30_H__
#define __SGP30_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool sgp30_init(void);
bool sgp30_read_tvoc(uint16_t *tvoc_ppb);

#ifdef __cplusplus
}
#endif

#endif /* __SGP30_H__ */
