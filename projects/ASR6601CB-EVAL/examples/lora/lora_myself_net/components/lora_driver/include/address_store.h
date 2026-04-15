#ifndef ADDRESS_STORE_H
#define ADDRESS_STORE_H

#include <stdint.h>
#include <stdbool.h>

// Load stored node address from flash. Returns true if a valid address was read.
bool load_node_address(uint8_t *addr);

// Store node address to flash. Returns true on success.
bool store_node_address(uint8_t addr);

#endif // ADDRESS_STORE_H
