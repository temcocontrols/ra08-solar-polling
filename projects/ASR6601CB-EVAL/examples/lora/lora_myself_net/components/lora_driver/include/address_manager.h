#ifndef ADDRESS_MANAGER_H
#define ADDRESS_MANAGER_H

#include <stdint.h>

// Initialize address manager (call at gateway startup)
void init_address_manager(void);

// Handle a JOIN request; uid points to 4-byte unique id from node
void handle_join_req(const uint8_t *uid);

#endif // ADDRESS_MANAGER_H
