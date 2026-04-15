#include "address_manager.h"
#include <string.h>
#include <stdio.h>
#include "radio.h"

// Simple in-RAM mapping table; keep small to save memory
typedef struct {
    uint8_t uid[4];
    uint8_t addr;
    uint8_t used;
} mapping_t;

// Support up to 256 entries (address 0 reserved)
static mapping_t mappings[256];

void init_address_manager(void)
{
    memset(mappings, 0, sizeof(mappings));
}

static int find_by_uid(const uint8_t *uid)
{
    for (int i = 1; i < 256; ++i) {
        if (mappings[i].used && memcmp(mappings[i].uid, uid, 4) == 0) return i;
    }
    return -1;
}

static int allocate_addr_for_uid(const uint8_t *uid)
{
    for (int i = 1; i < 255; ++i) {
        if (!mappings[i].used) {
            mappings[i].used = 1;
            memcpy(mappings[i].uid, uid, 4);
            mappings[i].addr = (uint8_t)i;
            return mappings[i].addr;
        }
    }
    return -1; // no free
}

static void send_assign_packet(const uint8_t *uid, uint8_t addr)
{
    uint8_t pkt[8];
    pkt[0] = 0xA0;
    pkt[1] = 0x11; // ASSIGN_ADDR
    memcpy(&pkt[2], uid, 4);
    pkt[6] = addr;
    pkt[7] = 0xA1;
    Radio.Send(pkt, sizeof(pkt));
    printf("[ADDRMGR] Sent ASSIGN for UID %02X%02X%02X%02X -> %d\r\n", uid[0], uid[1], uid[2], uid[3], addr);
}

void handle_join_req(const uint8_t *uid)
{
    if (uid == NULL) return;
    int idx = find_by_uid(uid);
    if (idx >= 0) {
        // already assigned
        uint8_t addr = mappings[idx].addr;
        send_assign_packet(uid, addr);
        return;
    }
    int newaddr = allocate_addr_for_uid(uid);
    if (newaddr > 0) {
        send_assign_packet(uid, (uint8_t)newaddr);
    } else {
        printf("[ADDRMGR] No free address to assign\r\n");
    }
}

