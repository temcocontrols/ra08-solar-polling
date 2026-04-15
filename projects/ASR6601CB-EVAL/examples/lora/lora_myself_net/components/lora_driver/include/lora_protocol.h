/* lora_protocol.h
 * Shared protocol constants and helpers for Ra-08 example
 */
#ifndef LORA_PROTOCOL_H
#define LORA_PROTOCOL_H

#include <stdint.h>

/* Opcode definitions */
#define OPCODE_FIND       0xF9
#define OPCODE_FIND_ACK   0xFA

/* Simple checksum: sum of three bytes, low 8 bits */
static inline uint8_t lora_simple_checksum(uint8_t a, uint8_t b, uint8_t c)
{
    return (uint8_t)((a + b + c) & 0xFF);
}

#endif /* LORA_PROTOCOL_H */
