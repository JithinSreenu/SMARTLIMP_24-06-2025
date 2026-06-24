/**

 * @file    packet_protocol.c

 * @brief   Wire-protocol implementation for the prosthetic knee.

 * @author  Mavis

 *

 * Educational note

 * ---------------

 * This file is intentionally self-contained. The packet format

 * is hand-rolled rather than using a third-party serializer

 * (protobuf, COBS, SLIP) because:

 *   1. We are on an offline workstation — we cannot pull packages.

 *   2. The frame is short and fixed, so a 30-line custom coder

 *      is more auditable for a safety-critical medical device

 *      than a generic library.

 *   3. Every byte of the on-air format is documented in the header.

 */


#include "packet_protocol.h"

#include <string.h>


/* -------------------------------------------------------------------- */

/*  CRC-8 lookup table (polynomial 0x07, no reflection)                  */

/* -------------------------------------------------------------------- */


static const uint8_t crc8_table[256] = {

    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,

    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,

    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,

    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,

    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,

    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,

    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,

    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,

    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,

    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,

    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,

    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,

    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,

    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,

    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,

    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3

};


uint8_t pk_crc8(const uint8_t *buf, uint32_t len)

{

    uint8_t crc = 0x00u;

    for (uint32_t i = 0u; i < len; i++) {

        /* XOR-in byte, then look up the running remainder.

         * The XOR with the *upper* nibble is implicit in the table. */

        crc = crc8_table[crc ^ buf[i]];

    }

    return crc;

}


/* -------------------------------------------------------------------- */

/*  Encoder                                                              */

/* -------------------------------------------------------------------- */


uint32_t pk_packet_encode(pk_cmd_t cmd,

                          const uint8_t *payload,

                          uint8_t plen,

                          uint8_t *out)

{

    if (plen > PK_PROTO_MAX_PAYLOAD || out == NULL) {

        return 0u;

    }


    uint32_t idx = 0u;

    out[idx++] = PK_PROTO_START_BYTE;

    out[idx++] = plen;

    if (plen > 0u && payload != NULL) {

        memcpy(&out[idx], payload, plen);

        idx += plen;

    }

    out[idx++] = (uint8_t)cmd;

    /* CRC covers LEN + PAYLOAD + CMD = plen + 2 bytes starting at out[1]. */

    out[idx++] = pk_crc8(&out[1], (uint32_t)plen + 2u);

    out[idx++] = PK_PROTO_STOP_BYTE;

    return idx;

}


/* -------------------------------------------------------------------- */

/*  Decoder FSM                                                          */

/* -------------------------------------------------------------------- */


/* The actual state. Static = file-private. We do NOT expose it. */

static struct {

    pk_decoder_state_t state;

    uint8_t            len;

    uint8_t            idx;

    pk_cmd_t           cmd;

    uint8_t            crc;

    uint8_t            payload[PK_PROTO_MAX_PAYLOAD];

} s_dec = {0};


void pk_packet_decoder_reset(void)

{

    (void)memset(&s_dec, 0, sizeof(s_dec));

    s_dec.state = PK_DECODER_IDLE;

}


bool pk_packet_decode_byte(uint8_t byte,

                           pk_cmd_t *cmd,

                           uint8_t *pld,

                           uint8_t *plen)

{

    switch (s_dec.state) {

    case PK_DECODER_IDLE:

        if (byte == PK_PROTO_START_BYTE) {

            s_dec.state = PK_DECODER_GOT_START;

            s_dec.idx   = 0u;

        }

        return false;


    case PK_DECODER_GOT_START:

        if (byte > PK_PROTO_MAX_PAYLOAD) {

            pk_packet_decoder_reset();   /* bad length — drop frame */

            return false;

        }

        s_dec.len = byte;

        s_dec.state = (byte == 0u) ? PK_DECODER_GOT_CMD : PK_DECODER_GOT_PAYLOAD;

        return false;


    case PK_DECODER_GOT_PAYLOAD:

        s_dec.payload[s_dec.idx++] = byte;

        if (s_dec.idx >= s_dec.len) {

            s_dec.state = PK_DECODER_GOT_CMD;

            s_dec.idx   = 0u;

        }

        return false;


    case PK_DECODER_GOT_CMD:

        /* CRC must cover LEN + PAYLOAD + CMD, exactly like the encoder. */

        s_dec.cmd = (pk_cmd_t)byte;

        {

            uint8_t crc_buf[PK_PROTO_MAX_PAYLOAD + 2];

            uint32_t n = 0u;

            crc_buf[n++] = s_dec.len;

            if (s_dec.len > 0u) {

                memcpy(&crc_buf[n], s_dec.payload, s_dec.len);

                n += s_dec.len;

            }

            crc_buf[n++] = byte;       /* the CMD byte we just received */

            s_dec.crc    = pk_crc8(crc_buf, n);

        }

        s_dec.state = PK_DECODER_GOT_CRC;

        return false;


    case PK_DECODER_GOT_CRC:

        if (byte != s_dec.crc) {

            pk_packet_decoder_reset();

            return false;

        }

        s_dec.state = PK_DECODER_GOT_STOP;

        return false;


    case PK_DECODER_GOT_STOP:

        if (byte != PK_PROTO_STOP_BYTE) {

            pk_packet_decoder_reset();

            return false;

        }

        /* Frame complete */

        if (cmd != NULL)  *cmd  = s_dec.cmd;

        if (plen != NULL) *plen = s_dec.len;

        if (pld != NULL && s_dec.len > 0u) {

            (void)memcpy(pld, s_dec.payload, s_dec.len);

        }

        pk_packet_decoder_reset();

        return true;


    default:

        pk_packet_decoder_reset();

        return false;

    }

}

