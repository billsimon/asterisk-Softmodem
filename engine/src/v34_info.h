/* The information sequences V.34 start-up exchanges: INFO0, INFO1c, INFO1a
 * (10.1.2.3) and MP (10.1.3.9), as bit arrays in transmission order - one
 * bit per byte, bit 0 of the table first - so that the modulator and the
 * parser deal in exactly what the tables say. */
#ifndef DATAMODEM_V34_INFO_H
#define DATAMODEM_V34_INFO_H

#include <stdbool.h>
#include <stdint.h>

#include "v34_codec.h"

/* 10.1.2.3.2: x^16 + x^12 + x^5 + 1, register loaded with ones, bits shifted
 * in first to last, the result sent bit 0 first. Run over a frame's
 * information bits followed by its CRC, it comes out zero. */
uint16_t v34_crc_bits(const uint8_t *bits, int n, uint16_t crc);

/* Table 14 */
typedef struct
{
    bool sr2743, sr2800, sr3429;
    bool low3000, high3000, low3200, high3200;
    bool allow3429;
    bool power_reduction;
    int asym_steps;           /* 0..5 */
    bool cme;
    bool c1664;               /* constellations up to 1664 points */
    int clock_source;         /* 0 internal */
    bool ack;                 /* bit 28: an INFO0 from the far end has been received */
} v34_info0_t;

/* Table 15 */
typedef struct
{
    int min_power_reduction;  /* 0..7 dB, for the answering modem's transmitter */
    int additional_power_reduction;
    int md;                   /* 35 ms units */
    struct
    {
        bool high;            /* the high carrier */
        int pre_emphasis;     /* 0..10 */
        int max_rate;         /* x 2400, 0 = this symbol rate cannot be used */
    } sr[V34_NUM_SR];
    int freq_offset;          /* 0.02 Hz, -512 = not measured */
} v34_info1c_t;

/* Table 16 */
typedef struct
{
    int min_power_reduction;  /* for the calling modem's transmitter */
    int additional_power_reduction;
    int md;
    bool high;                /* carrier, calling to answering */
    int pre_emphasis;         /* calling to answering */
    int max_rate;             /* x 2400, calling to answering */
    int sr_a_to_c;
    int sr_c_to_a;
    int freq_offset;
} v34_info1a_t;

/* Tables 20 and 21 */
typedef struct
{
    int type;                 /* 0, or 1 with precoding coefficients */
    int rate_c_to_a;          /* x 2400 */
    int rate_a_to_c;
    bool aux;
    int trellis;              /* 0 16 states, 1 32, 2 64 */
    bool nonlinear;
    bool expanded;
    bool ack;
    unsigned rate_mask;       /* bit 0 2400 ... bit 13 33600 */
    bool asymmetric;
    int16_t h[3][2];
} v34_mp_t;

#define V34_INFO0_BITS 49
#define V34_INFO1C_BITS 109
#define V34_INFO1A_BITS 70
#define V34_MP0_BITS 88
#define V34_MP1_BITS 188

int v34_info0_pack(const v34_info0_t *in, uint8_t *bits);
bool v34_info0_unpack(const uint8_t *bits, int n, v34_info0_t *out);
int v34_info1c_pack(const v34_info1c_t *in, uint8_t *bits);
bool v34_info1c_unpack(const uint8_t *bits, int n, v34_info1c_t *out);
int v34_info1a_pack(const v34_info1a_t *in, uint8_t *bits);
bool v34_info1a_unpack(const uint8_t *bits, int n, v34_info1a_t *out);
int v34_mp_pack(const v34_mp_t *in, uint8_t *bits);
/* bits starts at the frame sync. False on a bad CRC or a malformed frame. */
bool v34_mp_unpack(const uint8_t *bits, int n, v34_mp_t *out);

/* Tables 18 and 19, left-most bit first in time. */
#define V34_J4 0x0991u        /* 0000100110010001 */
#define V34_J16 0x0D91u       /* 0000110110010001 */
#define V34_JPRIME 0xF991u    /* 1111100110010001 */

/* ------------------------------------------------------------------ V.8 */

/* What CM and JM say, in the parts V.34 cares about. */
typedef struct
{
    int call_function;        /* 6: V-series data */
    bool v34;                 /* duplex */
    bool v34hdx;
    bool v32;                 /* V.32/V.32bis */
    bool v22;
    bool v21;
    bool lapm;                /* protocols octet: V.42 LAPM */
    bool pcm;                 /* a V.90/V.92 modem availability octet was seen */
} v8_msg_t;

/* Octets of a CM or JM, without the preamble and sync. */
int v8_build(const v8_msg_t *m, uint8_t *oct, int max);
bool v8_parse(const uint8_t *oct, int n, v8_msg_t *m);

#endif /* DATAMODEM_V34_INFO_H */
