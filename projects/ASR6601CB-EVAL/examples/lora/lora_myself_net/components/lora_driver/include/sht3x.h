#ifndef SHT3X_H
#define SHT3X_H

#include <stdint.h>
#include <stdbool.h>

// Initialize SHT3x (configures I2C0 pins and enables peripheral)
void sht3x_init(void);

// Read temperature (°C) and relative humidity (%) from SHT3x
// Returns true on success, false on communication/CRC error.
bool sht3x_read(float *temperature_c, float *relative_humidity);

// Optional test helper: prints measurement to UART
void sht3x_print_measurement(void);

#endif // SHT3X_H
