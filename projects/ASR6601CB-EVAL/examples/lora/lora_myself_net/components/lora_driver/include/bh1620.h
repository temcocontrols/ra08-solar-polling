#ifndef __BH1620_H__
#define __BH1620_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool bh1620_init(void);
bool bh1620_read_lux(uint16_t *lux);

#ifdef __cplusplus
}
#endif

#endif /* __BH1620_H__ */
