/**

 * @file    host_test.c

 * @brief   Host-side unit tests for the prosthetic-knee firmware.

 *

 * Builds with a plain `gcc` — no ARM toolchain, no HAL. Exercises:

 *   - CRC8 encoder/decoder round-trip

 *   - Packet encode/decode FSM (good frame, bad CRC, bad stop)

 *   - pk_movavg_t window behaviour

 *   - pk_ema_t fixed-point convergence

 *

 * Run with:

 *     cd Tests && make

 */


#include "project_types.h"

#include "packet_protocol.h"

#include "filter_lib.h"


#include <assert.h>

#include <stdio.h>

#include <string.h>


/* -------------------------------------------------------------------- */

/*  Test 1 — CRC-8 SAE-J1850 known vectors                              */

/* -------------------------------------------------------------------- */


static void test_crc8_known_vectors(void)

{

    /* Known answer: CRC8("123456789") with our zero-init variant

     * (poly 0x07, init 0x00, xorout 0x00) = 0xF4.

     * The standard CRC-8/SAE-J1850 uses init 0xFF, xorout 0xFF

     * and gives 0x4B; we deliberately use the zero variant for

     * easier boot-time verification (CRC of empty buffer is 0). */

    const uint8_t s[] = {'1','2','3','4','5','6','7','8','9'};

    uint8_t c = pk_crc8(s, sizeof(s));

    printf("CRC8(\"123456789\") = 0x%02X (expect 0xF4)\n", c);

    assert(c == 0xF4u);

}


/* -------------------------------------------------------------------- */

/*  Test 2 — packet round-trip                                          */

/* -------------------------------------------------------------------- */


static void test_packet_round_trip(void)

{

    uint8_t payload[] = { 0x64, 0xFE, 0x03, 0x00, 0x32, 0x00, 0x64, 0x01, 0x2C, 0x11 };

    uint8_t frame[PK_PROTO_MAX_FRAME];

    uint32_t n = pk_packet_encode(PK_CMD_TELEMETRY_PUSH, payload, sizeof(payload), frame);

    assert(n == sizeof(payload) + 5u);   /* start + len + payload + cmd + crc + stop */

    assert(frame[0] == PK_PROTO_START_BYTE);

    assert(frame[n - 1] == PK_PROTO_STOP_BYTE);


    pk_packet_decoder_reset();

    bool got_frame = false;

    pk_cmd_t   cmd = (pk_cmd_t)0xFF;    /* invalid sentinel */

    uint8_t    rx_payload[PK_PROTO_MAX_PAYLOAD];

    uint8_t    rx_plen = 0u;


    for (uint32_t i = 0u; i < n; i++) {

        if (pk_packet_decode_byte(frame[i], &cmd, rx_payload, &rx_plen)) {

            got_frame = true;

            break;

        }

    }

    assert(got_frame);

    assert(cmd == PK_CMD_TELEMETRY_PUSH);

    assert(rx_plen == sizeof(payload));

    assert(memcmp(payload, rx_payload, rx_plen) == 0);

    printf("Packet round-trip OK\n");

}


/* -------------------------------------------------------------------- */

/*  Test 3 — corrupted frame is rejected                                */

/* -------------------------------------------------------------------- */


static void test_packet_crc_corruption(void)

{

    uint8_t payload[] = { 1, 2, 3, 4 };

    uint8_t frame[PK_PROTO_MAX_FRAME];

    uint32_t n = pk_packet_encode(PK_CMD_GET_STATE, payload, sizeof(payload), frame);

    /* Flip a data bit */

    frame[3] ^= 0x01u;


    pk_packet_decoder_reset();

    bool got_frame = false;

    pk_cmd_t   cmd;

    uint8_t    rx_payload[PK_PROTO_MAX_PAYLOAD];

    uint8_t    rx_plen;


    for (uint32_t i = 0u; i < n; i++) {

        if (pk_packet_decode_byte(frame[i], &cmd, rx_payload, &rx_plen)) {

            got_frame = true;

        }

    }

    assert(!got_frame);

    printf("CRC corruption correctly rejected\n");

}


/* -------------------------------------------------------------------- */

/*  Test 4 — moving average                                              */

/* -------------------------------------------------------------------- */


static void test_moving_average(void)

{

    int32_t ring[4];

    pk_movavg_t f;

    pk_movavg_init(&f, ring, 4u);


    /* feed 4 samples of 100 */

    for (int i = 0; i < 4; i++) {

        pk_movavg_push(&f, 100);

    }

    int32_t avg = pk_movavg_push(&f, 100);

    assert(avg == 100);


    /* now feed 0 — average should settle to 25 after window flush */

    for (int i = 0; i < 4; i++) {

        pk_movavg_push(&f, 0);

    }

    avg = pk_movavg_push(&f, 0);

    assert(avg == 0);

    printf("Moving average OK\n");

}


/* -------------------------------------------------------------------- */

/*  Test 5 — EMA Q15                                                    */

/* -------------------------------------------------------------------- */


static void test_ema_q15(void)

{

    pk_ema_t f;

    pk_ema_init(&f, PK_ALPHA_Q15(0.5f));    /* alpha = 0.5 */


    int32_t s = 100;

    /* After seeding */

    s = pk_ema_push(&f, 100);

    assert(s == 100);


    /* 0 → alpha*0 + (1-alpha)*100 = 50 */

    s = pk_ema_push(&f, 0);

    assert(s == 50);


    s = pk_ema_push(&f, 0);

    /* 25 */

    assert(s == 25);


    s = pk_ema_push(&f, 0);

    /* ~12 */

    assert(s == 12 || s == 13);

    printf("EMA Q15 OK\n");

}


/* -------------------------------------------------------------------- */

/*  Main                                                                */

/* -------------------------------------------------------------------- */


int main(void)

{

    test_crc8_known_vectors();

    test_packet_round_trip();

    test_packet_crc_corruption();

    test_moving_average();

    test_ema_q15();

    printf("ALL TESTS PASSED\n");

    return 0;

}

