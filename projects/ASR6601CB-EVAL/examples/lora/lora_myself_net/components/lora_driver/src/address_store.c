#include "address_store.h"
#include "tremo_flash.h"
#include <string.h>

// NOTE: Adjust this address to a safe, unused flash page for your board.
// Default here is a placeholder near flash end; change before production.
#ifndef NODE_ADDR_FLASH_ADDR
#define NODE_ADDR_FLASH_ADDR 0x0803F800UL
#endif

// flash page size used for erase; tremo_flash provides page erase by addr
#define FLASH_ERESE_PAGE_ADDR NODE_ADDR_FLASH_ADDR

static uint8_t calc_crc8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0xFF;
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; ++b)
            crc = (crc & 1) ? (uint8_t)((crc >> 1) ^ 0x8C) : (uint8_t)(crc >> 1);
    }
    return crc;
}

bool load_node_address(uint8_t *addr)
{
    if (addr == NULL) return false;
    uint8_t buf[2];
    // flash is memory mapped; read directly
    memcpy(buf, (const void *)NODE_ADDR_FLASH_ADDR, sizeof(buf));
    // a blank flash usually reads 0xFF; treat 0xFF as invalid
    if (buf[0] == 0xFF || buf[1] == 0xFF) return false;
    if (calc_crc8(&buf[0], 1) != buf[1]) return false;
    *addr = buf[0];
    return true;
}

bool store_node_address(uint8_t addr)
{
    uint8_t buf[2];
    buf[0] = addr;
    buf[1] = calc_crc8(&buf[0], 1);

    // Erase page first
    if (flash_erase_page(FLASH_ERESE_PAGE_ADDR) < 0) {
        return false;
    }

    if (flash_program_bytes(NODE_ADDR_FLASH_ADDR, buf, sizeof(buf)) < 0) {
        return false;
    }

    // verify
    uint8_t verify[2];
    memcpy(verify, (const void *)NODE_ADDR_FLASH_ADDR, sizeof(verify));
    if (verify[0] != buf[0] || verify[1] != buf[1]) return false;
    return true;
}

