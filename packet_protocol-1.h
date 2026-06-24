/**
 * @file    packet_protocol.h
 * @brief   Prosthetic Knee — Wire Protocol (Start | Data | CRC | Stop)
 * @author  Mavis
 *
 * Wire format
 * -----------
 *   Byte 0      : START   = 0xAA  (synchronisation + first byte of magic)
 *   Byte 1      : LEN     = number of payload bytes (0..255)
 *   Byte 2..N+1 : PAYLOAD (LEN bytes)
 *   Byte N+2    : CMD     = command / type identifier
 *   Byte N+3    : CRC8    = checksum over LEN + PAYLOAD + CMD
 *                          polynomial 0x07, init 0x00
 *   Byte N+4    : STOP    = 0x55
 *
 * The CRC is the classic CRC-8/SAE-J1850 used by most automotive /
 * medical embedded busses because it catches all single-bit errors,
 * all two-bit errors within a window of 8 bits and most burst errors.
 *
 * Example — Telemetry frame (10 payload bytes):
 *
 *   AA 0A 64 FE 03 00 32 00 64 01 2C 11 A0 55
 *   |  |  |_________________________|  |  |  |  |
 *   |  |            payload           cmd CRC stop
 *   |  len = 10
 *   start
 *
 * Why START + STOP bytes and not just START?
 *   - A single start byte leaves the receiver vulnerable to noise
 *     flipping a data byte into the start value. With STOP = 0x55
 *     the bus idles between frames and a corrupted frame is dropped
 *     immediately.
 *   - The 0xAA / 0x55 pair is also what classic CPython struct
 *     tests use to detect endianness.
 */

#ifndef PACKET_PROTOCOL_H
#define PACKET_PROTOCOL_H

#include "project_types.h"

/* -------------------------------------------------------------------- */
/*  Wire constants                                                       */
/* -------------------------------------------------------------------- */

#define PK_PROTO_START_BYTE       (0xAAu)
#define PK_PROTO_STOP_BYTE        (0x55u)
#define PK_PROTO_MAX_PAYLOAD      (32u)   /* big enough for telemetry  */
#define PK_PROTO_MAX_FRAME        (PK_PROTO_MAX_PAYLOAD + 5u) /* incl. hdr */

/* -------------------------------------------------------------------- */
/*  Command IDs                                                          */
/* -------------------------------------------------------------------- */

typedef enum {
    PK_CMD_TELEMETRY_PUSH   = 0x01,  /* host <- device, periodic      */
    PK_CMD_FORCE_STREAM     = 0x02,  /* host <- device, 50 Hz         */
    PK_CMD_GET_STATE        = 0x03,
    PK_CMD_SET_STATE        = 0x04,
    PK_CMD_CALIBRATE_ZERO   = 0x05,
    PK_CMD_CALIBRATE_SCALE  = 0x06,
    PK_CMD_FAULT_REPORT     = 0x07,
    PK_CMD_BATTERY_QUERY    = 0x08,
    PK_CMD_DIAGNOSTIC       = 0x09,
    PK_CMD_ACK              = 0x10,
    PK_CMD_NACK             = 0x11,
    PK_CMD_BOOTLOADER_JUMP  = 0xF0
} pk_cmd_t;

/* -------------------------------------------------------------------- */
/*  Decoder state machine                                                */
/* -------------------------------------------------------------------- */

typedef enum {
    PK_DECODER_IDLE = 0,
    PK_DECODER_GOT_START,
    PK_DECODER_GOT_LEN,
    PK_DECODER_GOT_PAYLOAD,
    PK_DECODER_GOT_CMD,
    PK_DECODER_GOT_CRC,
    PK_DECODER_GOT_STOP,
    PK_DECODER_ERROR
} pk_decoder_state_t;

/* -------------------------------------------------------------------- */
/*  Public API                                                           */
/* -------------------------------------------------------------------- */

/**
 * @brief  Compute CRC-8 over a buffer.
 * @param  buf   pointer to data
 * @param  len   number of bytes
 * @return 8-bit CRC value (polynomial 0x07, init 0x00, no reflection,
 *         no final XOR — i.e. CRC-8/SAE-J1850 zero).
 */
uint8_t pk_crc8(const uint8_t *buf, uint32_t len);

/**
 * @brief  Encode a packet into a byte buffer ready for DMA TX.
 * @param  cmd       command ID
 * @param  payload   pointer to payload bytes (may be NULL if len==0)
 * @param  plen      payload length
 * @param  out       destination buffer, must be at least
 *                   PK_PROTO_MAX_FRAME bytes
 * @return total frame length on success, 0 on overflow
 */
uint32_t pk_packet_encode(pk_cmd_t cmd,
                          const uint8_t *payload,
                          uint8_t plen,
                          uint8_t *out);

/**
 * @brief  Feed one received byte into the decoder.
 * @param  byte  incoming byte
 * @param  cmd   out — command ID when a complete frame is received
 * @param  pld   out — payload buffer, must be >= PK_PROTO_MAX_PAYLOAD
 * @param  plen  out — payload length
 * @retval true  a complete frame was decoded and CRC verified
 * @retval false still waiting for more bytes
 */
bool pk_packet_decode_byte(uint8_t byte,
                           pk_cmd_t *cmd,
                           uint8_t *pld,
                           uint8_t *plen);

/**
 * @brief  Reset the decoder FSM (call after handling an error).
 */
void pk_packet_decoder_reset(void);

#endif /* PACKET_PROTOCOL_H */
