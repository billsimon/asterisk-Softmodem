/* ITU-T V.32 (03/93): 9600 and 4800 bit/s duplex, by echo cancellation, and
 * ITU-T V.32 bis (02/91), which adds 14 400, 12 000 and 7200 bit/s and a way
 * of changing rate without retraining.
 *
 * Section and table numbers in the comments are V.32's unless they say "bis".
 * The two share almost everything: the line signal, the scramblers, the
 * start-up and retrain procedures, the trellis code (Figure 1/V.32 bis is
 * Figure 2/V.32) and the 9600 bit/s constellation (Figure 2-3/V.32 bis is
 * Figure 3/V.32, point for point). V.32 bis differs in the rate signal's
 * meaning (Table 5/V.32 bis), in three more constellations, and in the rate
 * renegotiation procedure of its section 8. Which rules a call runs under is
 * settled by the rate signals: a V.32 bis modem marks its own with B4 and B8
 * both set, and the moment either end's lacks them, both work to V.32 (Table
 * 5/V.32 bis, Note 1).
 *
 * The signal path, transmit side:
 *
 *   bits -> scrambler (4) -> differential coding (Table 1 or 2)
 *        -> [trellis encoder, Figure 2] -> constellation (Figures 1 and 3)
 *        -> root raised cosine at 2400 baud -> 1800 Hz carrier -> line
 *
 * and receive side:
 *
 *   line -> echo canceller -> down to baseband -> matched filter, resampled
 *        at T/2 under Gardner timing control -> fractionally spaced
 *        equaliser -> carrier phase correction -> slicer or Viterbi decoder
 *        -> differential decoding -> descrambler -> bits
 *
 * Everything here runs at 8000 samples a second, where a symbol is 10/3
 * samples long. The transmitter handles the fraction with ten polyphase
 * branches of its pulse shaping filter, one for each place a sample can fall
 * within a symbol; the receiver with a finely divided matched filter that it
 * can evaluate at any instant it likes.
 *
 * TIME. Both directions count samples from the moment the pump starts, and
 * "line time" means a sample index: transmit sample k and receive sample k
 * are taken to be the same instant at the line terminals. That is what lets
 * the start-up procedure measure round trips and turn signals round 64
 * symbols after hearing them, and what gives the echo canceller a reference
 * to align our own signal against when it comes back. */
#include "datamodem/v32.h"
#include "datamodem/log.h"

#include <complex.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef float complex cf_t;

#define V32_PI 3.14159265358979323846

/* 2400 baud at 8000 samples a second. */
#define SPS (10.0 / 3.0)
#define T_HALF (5.0 / 3.0)

/* 0 dBm0 is a sine 3.14 dB below digital full scale, the same reference
 * spandsp's level settings use, so --tx-power means one thing everywhere. */
#define DBM0_RMS (32767.0 * 0.69654 / 1.41421356)

/* The pulse shape: root raised cosine. 2.2 of the Recommendation only asks
 * that 600 and 3000 Hz sit 4.5 +/- 2.5 dB below the peak, and any root
 * raised cosine puts the band edges at -3 dB; the roll-off decides how far
 * past them the signal spreads. 25% keeps it inside 300-3300 Hz. */
#define RRC_ALPHA 0.25

#define TX_L 6                      /* transmit filter half-span, symbols */
#define TX_TAPS (2 * TX_L + 1)
#define TX_HIST 16

#define MF_HALF 20                  /* matched filter half-span, samples (6 symbols) */
#define MF_TAPS (2 * MF_HALF + 1)
#define MF_PHASES 64
#define ZBUF 512
#define ZMASK (ZBUF - 1)

/* T/2 spaced. Sixteen taps ahead of the main one for the far end's
 * precursors, thirty-one behind it for the longer tail a telephone channel
 * leaves after each symbol. */
#define EQ_LEN 48
#define EQ_CENTER 16

#define VIT_DEPTH 24

/* Tone detection. A 40 sample boxcar has its nulls every 200 Hz, so the
 * 600, 1800 and 3000 Hz detectors are all deaf to each other and to the
 * images that mixing a real signal down produces. */
#define TONE_WIN 40
#define TONE_HIST 128
#define TONE_LAG 80

/* The echo canceller. The echo that matters is the far one - our own signal
 * reflected at the far end's hybrid, a whole RTP round trip later - so the
 * reference has to reach a long way back. */
#define EC_TAPS 128
#define EC_PRE 24                   /* taps placed ahead of the strongest echo */
#define TXH_LEN 32768
#define TXH_MASK (TXH_LEN - 1)
#define EC_CORR_MAX 16000           /* the longest echo delay looked for: 2 s */
#define EC_CORR_WIN 2048
#define EC_CONVERGE 2400            /* samples of uninterrupted training after finding it */
#define EC_MU_FAST 0.5f
#define EC_MU_SETTLED 0.03f
#define EC_MU_SLOW 0.002f
#define EC_MU_FLOOR 0.0002f

/* Timeouts are kept in receive samples. */
#define SECONDS(s) ((long long) ((s) * 8000.0))

/* How long a tone has to be there before it counts. */
#define RUN_64T 213
#define RUN_128T 427
#define S_RUN 64

/* The data-aided part of receiver training ends this many symbols into TRN,
 * safely before the shortest TRN the far end may send (1280). */
#define TRN_DA_END 1200

/* ------------------------------------------------------------ the tables */

enum { ST_A = 0, ST_B, ST_C, ST_D };

/* Figure 1/V.32: A, B, C and D are a quarter turn apart, in that order, which
 * is what makes "advance one quadrant" and "rotate by 90 degrees" the same
 * thing - the whole basis of the differential coding. */
static const int8_t ABCD_POINTS[4][2] = { { -3, -1 }, { 1, -3 }, { 3, 1 }, { -1, 3 } };

/* Figure 1/V.32, indexed by Y1 Y2 Q3 Q4. */
static const int8_t MAP16[16][2] = {
    { -1, -1 }, { -3, -1 }, { -1, -3 }, { -3, -3 },
    {  1, -1 }, {  1, -3 }, {  3, -1 }, {  3, -3 },
    { -1,  1 }, { -1,  3 }, { -3,  1 }, { -3,  3 },
    {  1,  1 }, {  3,  1 }, {  1,  3 }, {  3,  3 },
};

/* Figure 3/V.32, indexed by Y0 Y1 Y2 Q3 Q4. */
static const int8_t MAP32[32][2] = {
    { -4,  1 }, {  0, -3 }, {  0,  1 }, {  4,  1 },
    {  4, -1 }, {  0,  3 }, {  0, -1 }, { -4, -1 },
    { -2,  3 }, { -2, -1 }, {  2,  3 }, {  2, -1 },
    {  2, -3 }, {  2,  1 }, { -2, -3 }, { -2,  1 },
    { -3, -2 }, {  1, -2 }, { -3,  2 }, {  1,  2 },
    {  3,  2 }, { -1,  2 }, {  3, -2 }, { -1, -2 },
    {  1,  4 }, { -3,  0 }, {  1,  0 }, {  1, -4 },
    { -1, -4 }, {  3,  0 }, { -1,  0 }, { -1,  4 },
};

/* V.32 bis Figures 2-4, 2-2 and 2-1: 7200, 12 000 and 14 400 bit/s, indexed
 * by Y0 Y1 Y2 Q3 ... with Y0 the most significant bit, as MAP32 is. These are
 * in half units - Figure 2-1 needs them - so A, B, C and D sit at (-6, -2)
 * and its rotations here, where the figures put them, and every constellation
 * comes out with the same mean energy as the others to within 0.2 dB. Points
 * checked against the figures; the same constellations are V.17's, and these
 * agree with spandsp's tables for it. */
static const int8_t MAP16T[16][2] = {
    {  6, -6 }, { -2,  2 }, { -6,  6 }, {  2, -2 }, {  6,  2 }, { -2, -6 }, { -6, -2 }, {  2,  6 },
    { -2,  6 }, {  6, -2 }, {  2, -6 }, { -6,  2 }, { -6, -6 }, {  2,  2 }, {  6,  6 }, { -2, -2 },
};

static const int8_t MAP64[64][2] = {
    {  7,  1 }, {  3,  5 }, {  7, -7 }, { -5,  5 }, {  3, -3 }, { -1,  1 }, { -1, -7 }, { -5, -3 },
    { -7, -1 }, { -3, -5 }, { -7,  7 }, {  5, -5 }, { -3,  3 }, {  1, -1 }, {  1,  7 }, {  5,  3 },
    { -1,  5 }, { -5,  1 }, {  7,  5 }, { -5, -7 }, {  3,  1 }, { -1, -3 }, {  7, -3 }, {  3, -7 },
    {  1, -5 }, {  5, -1 }, { -7, -5 }, {  5,  7 }, { -3, -1 }, {  1,  3 }, { -7,  3 }, { -3,  7 },
    { -5, -1 }, { -1, -5 }, { -5,  7 }, {  7, -5 }, { -1,  3 }, {  3, -1 }, {  3,  7 }, {  7,  3 },
    {  5,  1 }, {  1,  5 }, {  5, -7 }, { -7,  5 }, {  1, -3 }, { -3,  1 }, { -3, -7 }, { -7, -3 },
    {  1, -7 }, {  5, -3 }, { -7, -7 }, {  5,  5 }, { -3, -3 }, {  1,  1 }, { -7,  1 }, { -3,  5 },
    { -1,  7 }, { -5,  3 }, {  7,  7 }, { -5, -5 }, {  3,  3 }, { -1, -1 }, {  7, -1 }, {  3, -5 },
};

static const int8_t MAP128[128][2] = {
    { -8, -3 }, {  8, -3 }, {  4, -3 }, {  4, -7 }, { -4, -3 }, { -4, -7 }, {  0, -3 }, {  0, -7 },
    { -8,  1 }, {  8,  1 }, {  4,  1 }, {  4,  5 }, { -4,  1 }, { -4,  5 }, {  0,  1 }, {  0,  5 },
    {  8,  3 }, { -8,  3 }, { -4,  3 }, { -4,  7 }, {  4,  3 }, {  4,  7 }, {  0,  3 }, {  0,  7 },
    {  8, -1 }, { -8, -1 }, { -4, -1 }, { -4, -5 }, {  4, -1 }, {  4, -5 }, {  0, -1 }, {  0, -5 },
    {  2, -9 }, {  2,  7 }, {  2,  3 }, {  6,  3 }, {  2, -5 }, {  6, -5 }, {  2, -1 }, {  6, -1 },
    { -2, -9 }, { -2,  7 }, { -2,  3 }, { -6,  3 }, { -2, -5 }, { -6, -5 }, { -2, -1 }, { -6, -1 },
    { -2,  9 }, { -2, -7 }, { -2, -3 }, { -6, -3 }, { -2,  5 }, { -6,  5 }, { -2,  1 }, { -6,  1 },
    {  2,  9 }, {  2, -7 }, {  2, -3 }, {  6, -3 }, {  2,  5 }, {  6,  5 }, {  2,  1 }, {  6,  1 },
    {  9,  2 }, { -7,  2 }, { -3,  2 }, { -3,  6 }, {  5,  2 }, {  5,  6 }, {  1,  2 }, {  1,  6 },
    {  9, -2 }, { -7, -2 }, { -3, -2 }, { -3, -6 }, {  5, -2 }, {  5, -6 }, {  1, -2 }, {  1, -6 },
    { -9, -2 }, {  7, -2 }, {  3, -2 }, {  3, -6 }, { -5, -2 }, { -5, -6 }, { -1, -2 }, { -1, -6 },
    { -9,  2 }, {  7,  2 }, {  3,  2 }, {  3,  6 }, { -5,  2 }, { -5,  6 }, { -1,  2 }, { -1,  6 },
    { -3,  8 }, { -3, -8 }, { -3, -4 }, { -7, -4 }, { -3,  4 }, { -7,  4 }, { -3,  0 }, { -7,  0 },
    {  1,  8 }, {  1, -8 }, {  1, -4 }, {  5, -4 }, {  1,  4 }, {  5,  4 }, {  1,  0 }, {  5,  0 },
    {  3, -8 }, {  3,  8 }, {  3,  4 }, {  7,  4 }, {  3, -4 }, {  7, -4 }, {  3,  0 }, {  7,  0 },
    { -1, -8 }, { -1,  8 }, { -1,  4 }, { -5,  4 }, { -1, -4 }, { -5, -4 }, { -1,  0 }, { -5,  0 },
};

/* Y1 Y2 names a quadrant, and at 4800 the point in it (Table 1, last
 * column): 00 A, 01 B, 11 C, 10 D. */
static const uint8_t QUAD_OF_Y[4] = { ST_A, ST_B, ST_D, ST_C };
static const uint8_t Y_OF_QUAD[4] = { 0, 1, 3, 2 };

/* Table 1/V.32, [Q1 Q2][previous Y1 Y2] -> Y1 Y2: a quadrant rotation of
 * +90, 0, +180 or +270 degrees. For 4800 and the nonredundant 9600. V.32 bis
 * numbers the two tables the other way round: this is its Table 2. */
static const uint8_t TABLE1[4][4] = {
    { 1, 3, 0, 2 },
    { 0, 1, 2, 3 },
    { 3, 2, 1, 0 },
    { 2, 0, 3, 1 },
};

/* Table 2/V.32, the same for the trellis coded 9600 - and Table 1/V.32 bis,
 * for every trellis coded rate. */
static const uint8_t TABLE2[4][4] = {
    { 0, 1, 2, 3 },
    { 1, 0, 3, 2 },
    { 2, 3, 1, 0 },
    { 3, 2, 0, 1 },
};

/* Table 6 and 7: the 16-bit rate sequence, B0 first in time. */
#define RW_B(k) ((uint16_t) (1u << (15 - (k))))
#define RW_SYNC (RW_B(7) | RW_B(11) | RW_B(15))
#define RW_HEAD (RW_B(0) | RW_B(1) | RW_B(2) | RW_B(3))
/* Table 5/V.32 bis, Note 1: both set, or the call runs as V.32. */
#define RW_BIS (RW_B(4) | RW_B(8))

/* ----------------------------------------------------------------- types */

typedef enum
{
    SEG_SILENCE = 0,
    SEG_AA,
    SEG_CC,
    SEG_AC,
    SEG_CA,
    SEG_S,
    SEG_SBAR,
    SEG_TRN,       /* from here on the signal is scrambled */
    SEG_RATE,
    SEG_E,
    SEG_DATA
} seg_t;

/* In order of preference: where two ends have more than one in common, the
 * highest numbered one wins. */
typedef enum
{
    CODE_4800 = 0,
    CODE_7200T,    /* V.32 bis: 16 points, trellis coded */
    CODE_9600U,    /* V.32: 16 points, nonredundant */
    CODE_9600T,    /* 32 points, trellis coded */
    CODE_12000T,   /* V.32 bis: 64 points */
    CODE_14400T,   /* V.32 bis: 128 points */
    CODE_COUNT
} coding_t;

#define CBIT(c) (1u << (c))
#define SET_V32 (CBIT(CODE_4800) | CBIT(CODE_9600U) | CBIT(CODE_9600T))
#define SET_BIS (CBIT(CODE_4800) | CBIT(CODE_7200T) | CBIT(CODE_9600T) | CBIT(CODE_12000T) | CBIT(CODE_14400T))

typedef struct
{
    int rate;
    int bits;                 /* data bits per symbol, Q1 Q2 included */
    bool tcm;
    const char *name;
    /* The SNR, in dB, that our receiver has to have trained at for us to
     * offer the coding, and the SNR below which, once it is running, it has
     * stopped being worth having. Measured back to back through G.711 with
     * a -10 dB echo, a minute each way: the offer is a dB above where the
     * coding stopped making errors at all, and "poor" where it reaches about
     * one in ten thousand. Each is no more than 0.3 dB from what training
     * predicted. The trellis code is worth 3 dB over the 16-point 9600, and
     * each further bit per symbol costs 3. */
    float offer_db;
    float poor_db;
} coding_info_t;

static const coding_info_t CODING[CODE_COUNT] = {
    [CODE_4800] = { 4800, 2, false, "4800", 0.0f, 11.0f },
    [CODE_7200T] = { 7200, 3, true, "7200 (trellis coded)", 15.0f, 13.0f },
    [CODE_9600U] = { 9600, 4, false, "9600 (16 points, uncoded)", 22.0f, 19.5f },
    [CODE_9600T] = { 9600, 4, true, "9600 (trellis coded)", 19.0f, 17.0f },
    [CODE_12000T] = { 12000, 5, true, "12000 (trellis coded)", 22.0f, 20.0f },
    [CODE_14400T] = { 14400, 6, true, "14400 (trellis coded)", 25.5f, 23.0f },
};

typedef struct
{
    seg_t seg;
    long long len;
} seg_step_t;

typedef struct
{
    seg_t seg;
    long long count;          /* symbols sent in this segment */
    long long len;            /* < 0: until told otherwise */
    seg_step_t prog[6];
    int nprog;
    int iprog;
    long long switch_at;      /* symbol index of a scheduled change, -1 if none */
    seg_t switch_seg;
    long long switch_len;

    long long nsym;           /* symbols generated */
    long long nsamp;          /* samples generated */
    cf_t hist[TX_HIST];

    uint32_t scr;
    int y;                    /* last Y1 Y2, for the differential coding */
    int conv;                 /* trellis encoder delay elements S1 S2 S3 */
    uint16_t word;            /* the rate signal being sent */
    int wbit;
    bool e_pending;
    long long e_min;          /* E not before this many symbols of the rate signal */
    uint16_t eword;
    coding_t coding;
    bool data_on;
    bool scrambled;
    bool rn;                  /* the rate signal is renegotiation's R4 or R5 (8/V.32 bis) */
} tx_t;

typedef enum
{
    RX_TONES = 0,  /* nothing but the tone detectors */
    RX_ACQ,        /* measuring S to find the timing, phase and level */
    RX_S,          /* tracking S, waiting for S-bar */
    RX_TRN_ALIGN,  /* the start of TRN, finding exactly where it began */
    RX_TRN_DA,     /* training against the known TRN sequence */
    RX_RATE,       /* rate signals, 4800 bit/s */
    RX_DATA,
    RX_RN_PRE      /* a renegotiation preamble (8/V.32 bis), finding where it ends */
} rx_mode_t;

typedef struct
{
    float pm[8];
    uint8_t prev[VIT_DEPTH][8];
    uint8_t bits[VIT_DEPTH][8];
    int pos;
    int count;
} vit_t;

typedef struct
{
    rx_mode_t mode;
    cf_t z[ZBUF];             /* baseband, before the matched filter */

    /* Acquisition */
    int acq_count;
    cf_t acq_m1, acq_m2, acq_pp, acq_pm;

    /* Resampler and timing */
    double tau;               /* when the next T/2 sample is due, in samples */
    bool on_time;
    bool have_prev;
    cf_t y_prev, y_mid;
    float py;
    float k_timing;
    float t_freq;             /* the far end's symbol clock against ours, samples per symbol */

    /* Equaliser and carrier */
    cf_t line[2 * EQ_LEN];
    int lpos;
    int halves;
    cf_t c[EQ_LEN];
    float beta;
    float theta, nu, a1, a2;
    float mse;
    bool center_only;
    float leak;

    long long osym;           /* equaliser outputs since acquisition */
    int s_good;
    int last_q;
    long long trn_first;
    cf_t abuf[48];
    int abuf_n;
    uint32_t trn_scr;
    long long trn_j;

    /* Decoding */
    int prev_q;
    int prev_y;
    uint32_t dscr;
    uint32_t rw;
    long long rw_bits;
    bool rw_locked;
    int rw_phase;
    int rw_bad;
    coding_t coding;
    long long data_rcvd;
    bool data_on;
    vit_t vit;
    cf_t vit_u[VIT_DEPTH];    /* what went into the decoder, to measure against what came out */
    float vit_mse;
    float snr_trained;

    /* Listening for the far end's renegotiation preamble while in data. */
    cf_t pre_prev;
    int pre_run;
    int pre_count;
    int pre_tail;
} rx_t;

typedef struct
{
    double sre[3], sim[3], spw;
    float pre[3][TONE_WIN], pim[3][TONE_WIN], ppw[TONE_WIN];
    int pos;
    cf_t hist[3][TONE_HIST];
    float p[3];               /* 600, 1800, 3000 Hz */
    float ptot;
    uint8_t present[TONE_HIST]; /* PRESENT_AC, PRESENT_AA: which tone was there, sample by sample */
} tones_t;

#define PRESENT_AC 1u
#define PRESENT_AA 2u

typedef enum
{
    EC_IDLE = 0,
    EC_WAIT,                  /* our TRN is going out; its echo is not back yet */
    EC_CORR,                  /* looking for where it comes back */
    EC_DONE
} ec_state_t;

typedef struct
{
    ec_state_t state;
    bool enabled;
    int delay;
    float w[EC_TAPS];
    float mu;
    long long fast_from;
    long long slow_from;
    long long corr_start;
    int corr_lmax;
    double *corr;
    double corr_erx;
    float in_avg, res_avg;    /* while the far end is silent */
    float erl_db, erle_db;
} ec_t;

typedef enum
{
    STG_C_LISTEN = 0,   /* calling: silent, waiting to be told ANS was heard, or for AC */
    STG_C_AA,           /* AA out, waiting for AC and its reversal */
    STG_C_CC,           /* CC out, waiting for the second reversal */
    STG_C_TRAIN1,       /* silent, training on the answerer's S S-bar TRN, waiting for R1 */
    STG_C_TRAIN2,       /* our S S-bar TRN R2 out; training again, waiting for R3 */
    STG_C_WAIT_E,       /* E sent; waiting for the answerer's */
    STG_C_B1,
    STG_A_AC,           /* answering: AC out, waiting for AA */
    STG_A_CA,           /* CA out, waiting for the reversal */
    STG_A_AC2,          /* AC again, waiting for the caller to fall silent */
    STG_A_R1,           /* S S-bar TRN R1 out, waiting for the caller's S */
    STG_A_TRAIN,        /* silent, training on the caller's S S-bar TRN, waiting for R2 */
    STG_A_WAIT_E,       /* S S-bar TRN R3 out, waiting for the caller's E */
    STG_A_B1,
    STG_DATA,
    STG_RN_SEND,        /* V.32 bis rate renegotiation: we asked; preamble and R4 out, waiting for R5 */
    STG_RN_RESP,        /* the far end asked; its preamble heard, waiting for its R4 */
    STG_RN_WAIT_E,      /* rate signals exchanged, E sent or on its way; waiting for the far end's */
    STG_RN_B1,          /* the new rate agreed; 24 symbols of scrambled ones each way */
    STG_DEAD
} stage_t;

static const char *const STAGE_NAMES[] = {
    "listening for the answering modem", "AA, waiting for AC", "CC, measuring the round trip",
    "training on the answerer",
    "training again, rates offered", "waiting for E", "scrambled ones",
    "AC, waiting for AA", "CA, measuring the round trip", "AC, waiting for silence",
    "R1, waiting for S", "training on the caller", "R3, waiting for E", "scrambled ones",
    "data", "changing rate, waiting for R5", "changing rate, waiting for R4",
    "changing rate, waiting for E", "changing rate, scrambled ones", "cleared down"
};

struct dm_v32
{
    bool calling;
    bool bis;                 /* we are a V.32 bis modem */
    bool far_bis;             /* and so is the far end, by its rate signals */
    unsigned enabled;         /* the codings we will run, CBIT()s */
    unsigned rn_offer;        /* our R4 or R5 */
    bool rn_refused;          /* the far end ignored a renegotiation; do not ask again */
    bool rn_deaf;             /* test hook: never hear the far end's renegotiation */
    unsigned renegotiations;
    int rn_from;              /* the coding a renegotiation we asked for started from */
    long long up_since;       /* reception good enough for a higher rate since then, 0 if not */
    long long up_next;        /* no asking for a higher rate before then */
    double up_backoff;        /* seconds, doubled each time asking for more got nothing */
    bool trellis;
    char tag[48];
    int (*get_bit)(void *user);
    void (*put_bit)(void *user, int bit);
    void (*event)(void *user, dm_v32_event_t ev);
    void *user;
    int scr_tap;              /* our scrambler: 1 + x^-18 + x^-23 calling, x^-5 answering */
    int dscr_tap;             /* the far end's */

    cf_t abcd[4];
    cf_t map16[16];
    cf_t map32[32];
    cf_t map16t[16];
    cf_t map64[64];
    cf_t map128[128];
    const cf_t *cmap[CODE_COUNT]; /* the trellis coded ones, by coding */
    uint8_t idx16[4][4];
    float gtx[10][TX_TAPS];
    float hr[MF_PHASES + 1][MF_TAPS];
    float cos40[40], sin40[40];
    uint8_t inv1[4][4], inv2[4][4];
    uint8_t vnext[8][4];
    float tx_gain;
    float rx_scale;
    float pmin;               /* -48 dBm0, as 16-bit sample power */
    float ploss;              /* below this the far end has gone */
    float p0;                 /* 0 dBm0 */

    tx_t tx;
    rx_t rx;
    tones_t tn;
    ec_t ec;
    float txh[TXH_LEN];       /* what we sent, for the echo canceller */
    uint8_t txflag[TXH_LEN];  /* and whether it was scrambled enough to learn from */

    long long n;              /* receive samples = line time */
    stage_t stage;
    long long deadline;
    bool deadline_warned;
    int ac_loose, ac_strict, aa_loose, aa_strict, s_run;
    bool rev_armed;
    long long rev_rearm;
    double t_mark;
    double rtd;               /* samples; < 0 until measured */
    double nt;                /* the round trip as V.32 counts it, turn-round included */
    long long ca_start;
    float p18_avg, p18_ref;
    int drop_run;
    bool want_train;
    long long train_gate;
    bool ec_request;
    uint16_t sent_rate;
    int rate;
    coding_t coding;
    bool tcm;
    bool trained_once;
    unsigned retrains;
    long long freeze_until;
    float pwr;
    long long low_since;
    long long poor_since;
};

/* -------------------------------------------------------------- helpers */

static void emit(dm_v32_t *v, dm_v32_event_t ev)
{
    if (v->event != NULL)
        v->event(v->user, ev);
}

static cf_t pt(const int8_t p[2])
{
    return (float) p[0] + I * (float) p[1];
}

static float mag2(cf_t z)
{
    return crealf(z) * crealf(z) + cimagf(z) * cimagf(z);
}

static int scramble(uint32_t *reg, int tap, int in)
{
    int out = (in ^ (int) (*reg >> (tap - 1)) ^ (int) (*reg >> 22)) & 1;

    *reg = ((*reg << 1) | (uint32_t) out) & 0x7FFFFFu;
    return out;
}

static int descramble(uint32_t *reg, int tap, int in)
{
    int out = (in ^ (int) (*reg >> (tap - 1)) ^ (int) (*reg >> 22)) & 1;

    *reg = ((*reg << 1) | (uint32_t) (in & 1)) & 0x7FFFFFu;
    return out;
}

static double rrc(double t)
{
    const double a = RRC_ALPHA;
    double x;

    if (fabs(t) < 1e-9)
        return 1.0 - a + 4.0 * a / V32_PI;
    if (fabs(fabs(t) - 1.0 / (4.0 * a)) < 1e-9)
        return a / sqrt(2.0) * ((1.0 + 2.0 / V32_PI) * sin(V32_PI / (4.0 * a)) +
                                (1.0 - 2.0 / V32_PI) * cos(V32_PI / (4.0 * a)));
    x = 4.0 * a * t;
    return (sin(V32_PI * t * (1.0 - a)) + 4.0 * a * t * cos(V32_PI * t * (1.0 + a))) /
           (V32_PI * t * (1.0 - x * x));
}

/* Root raised cosine, t in symbols, Hann windowed to +/- span symbols. */
static double pulse(double t, double span)
{
    if (fabs(t) >= span)
        return 0.0;
    return rrc(t) * 0.5 * (1.0 + cos(V32_PI * t / span));
}

/* Where the boundary between symbol k-1 and symbol k crosses the line, and
 * which symbol would put a boundary nearest to line time t. */
static double boundary_time(long long k)
{
    return ((double) k + TX_L - 0.5) * SPS;
}

static long long boundary_symbol(double t)
{
    return llround(t / SPS - TX_L + 0.5);
}

static bool rate_word_ok(uint16_t w)
{
    return (w & RW_HEAD) == 0 && (w & RW_SYNC) == RW_SYNC;
}

static bool e_word_ok(uint16_t w)
{
    return (w & RW_HEAD) == RW_HEAD && (w & RW_SYNC) == RW_SYNC;
}

static bool word_is_bis(uint16_t w)
{
    return (w & RW_BIS) == RW_BIS;
}

/* A rate signal offering the codings in set: in V.32's terms (Table 6), or
 * V.32 bis's (Table 5/V.32 bis).
 *
 * V.32: B5 4800, B6 9600, B8 trellis coding at 9600. B9-B14 are 0 0 1 0 0 0,
 * "no special operational modes"; B11 is also a sync bit, so RW_SYNC already
 * has it. B4, 2400 bit/s, is for further study and never offered.
 *
 * V.32 bis: B4 and B8 always, then B5 4800, B9 7200, B6 9600, B10 12 000 and
 * B12 14 400, every one but 4800 trellis coded. */
static uint16_t make_word(bool e, unsigned set, bool bis)
{
    uint16_t w = RW_SYNC;

    if (e)
        w |= RW_HEAD;
    if (set & CBIT(CODE_4800))
        w |= RW_B(5);
    if (bis)
    {
        w |= RW_BIS;
        if (set & CBIT(CODE_7200T))
            w |= RW_B(9);
        if (set & CBIT(CODE_9600T))
            w |= RW_B(6);
        if (set & CBIT(CODE_12000T))
            w |= RW_B(10);
        if (set & CBIT(CODE_14400T))
            w |= RW_B(12);
    }
    else
    {
        if (set & (CBIT(CODE_9600U) | CBIT(CODE_9600T)))
            w |= RW_B(6);
        if (set & CBIT(CODE_9600T))
            w |= RW_B(8);
    }
    return w;
}

/* The codings a rate signal (or E) allows, read as V.32 bis if bis says this
 * end is one and the word carries the mark, and as V.32 otherwise - which is
 * how a V.32 modem reads a V.32 bis modem's R1: B6 9600, B8 trellis coding,
 * the rest ignored. In V.32 terms a 9600 offer with trellis coding still
 * allows the nonredundant code too: an end that cannot do the trellis code
 * answers with B6 alone, and the 16-point code is what they then run (V.32
 * 1e). */
static unsigned set_of(uint16_t w, bool bis)
{
    unsigned s = 0;

    if (w & RW_B(5))
        s |= CBIT(CODE_4800);
    if (bis && word_is_bis(w))
    {
        if (w & RW_B(9))
            s |= CBIT(CODE_7200T);
        if (w & RW_B(6))
            s |= CBIT(CODE_9600T);
        if (w & RW_B(10))
            s |= CBIT(CODE_12000T);
        if (w & RW_B(12))
            s |= CBIT(CODE_14400T);
    }
    else if (w & RW_B(6))
    {
        s |= (w & RW_B(8)) ? CBIT(CODE_9600T) | CBIT(CODE_9600U) : CBIT(CODE_9600U);
    }
    return s;
}

/* The best coding in a set, or -1 for none. */
static int best_of(unsigned set)
{
    for (int c = CODE_COUNT - 1; c >= 0; c--)
        if (set & CBIT(c))
            return c;
    return -1;
}

static void word_text(uint16_t w, char *out, size_t len)
{
    static const char *const short_names[CODE_COUNT] = { "4800", "7200", "9600 uncoded", "9600", "12000",
                                                         "14400" };
    unsigned set = set_of(w, true);
    size_t o;

    o = (size_t) snprintf(out, len, "%04x:%s", w, word_is_bis(w) ? " V.32bis" : "");
    if (!word_is_bis(w) && (w & RW_B(6)) && (w & RW_B(8)))
        set &= ~CBIT(CODE_9600U); /* "9600 trellis" says it better than both */
    for (int c = 0; c < CODE_COUNT && o < len; c++)
        if (set & CBIT(c))
            o += (size_t) snprintf(out + o, len - o, " %s", short_names[c]);
    if (!word_is_bis(w) && (w & RW_B(8)) && o < len)
        o += (size_t) snprintf(out + o, len - o, " trellis");
    if (set == 0 && o < len)
        snprintf(out + o, len - o, " cleardown");
}

/* --------------------------------------------------------------- set-up */

static void build_tables(dm_v32_t *v)
{
    double sum = 0.0;
    double k;
    double s;

    for (int i = 0; i < 4; i++)
        v->abcd[i] = pt(ABCD_POINTS[i]);
    for (int i = 0; i < 16; i++)
    {
        v->map16[i] = pt(MAP16[i]);
        v->idx16[(MAP16[i][0] + 3) / 2][(MAP16[i][1] + 3) / 2] = (uint8_t) i;
    }
    for (int i = 0; i < 32; i++)
        v->map32[i] = pt(MAP32[i]);
    for (int i = 0; i < 16; i++)
        v->map16t[i] = 0.5f * pt(MAP16T[i]);
    for (int i = 0; i < 64; i++)
        v->map64[i] = 0.5f * pt(MAP64[i]);
    for (int i = 0; i < 128; i++)
        v->map128[i] = 0.5f * pt(MAP128[i]);
    v->cmap[CODE_7200T] = v->map16t;
    v->cmap[CODE_9600T] = v->map32;
    v->cmap[CODE_12000T] = v->map64;
    v->cmap[CODE_14400T] = v->map128;
    for (int i = 0; i < 40; i++)
    {
        v->cos40[i] = (float) cos(2.0 * V32_PI * i / 40.0);
        v->sin40[i] = (float) sin(2.0 * V32_PI * i / 40.0);
    }
    for (int q = 0; q < 4; q++)
        for (int y = 0; y < 4; y++)
        {
            v->inv1[y][TABLE1[q][y]] = (uint8_t) q;
            v->inv2[y][TABLE2[q][y]] = (uint8_t) q;
        }

    /* Figure 2's convolutional encoder, as a next-state table. */
    for (int st = 0; st < 8; st++)
        for (int yy = 0; yy < 4; yy++)
        {
            int s1 = (st >> 2) & 1, s2 = (st >> 1) & 1, s3 = st & 1;
            int ya = (yy >> 1) & 1, yb = yy & 1;
            int n1 = s3;
            int n2 = s1 ^ ya ^ yb ^ (s3 & (s2 ^ yb));
            int n3 = s2 ^ yb ^ (ya & s3);

            v->vnext[st][yy] = (uint8_t) ((n1 << 2) | (n2 << 1) | n3);
        }

    /* Transmit pulse: ten polyphase branches, scaled so that independent
     * symbols of mean energy E come out at mean baseband power E. */
    for (int mu = 0; mu < 10; mu++)
        for (int m = 0; m < TX_TAPS; m++)
        {
            double g = pulse(mu / 10.0 + m - TX_L, TX_L);

            v->gtx[mu][m] = (float) g;
            sum += g * g;
        }
    k = sqrt(10.0 / sum);
    for (int mu = 0; mu < 10; mu++)
        for (int m = 0; m < TX_TAPS; m++)
            v->gtx[mu][m] = (float) (v->gtx[mu][m] * k);

    /* Matched filter, scaled so that the two in cascade deliver a lone
     * symbol at its own size. */
    s = 0.0;
    for (int n = -MF_HALF; n <= MF_HALF; n++)
        s += k * pulse(n / SPS, TX_L) * pulse(n / SPS, MF_HALF / SPS);
    for (int p = 0; p <= MF_PHASES; p++)
        for (int j = -MF_HALF; j <= MF_HALF; j++)
            v->hr[p][j + MF_HALF] = (float) (pulse(((double) p / MF_PHASES - j) / SPS, MF_HALF / SPS) / s);
}

/* ----------------------------------------------------------- transmitter */

static void ec_schedule(dm_v32_t *v, double trn_line_time);

static void tx_enter(dm_v32_t *v, seg_t seg, long long len)
{
    tx_t *t = &v->tx;

    t->seg = seg;
    t->len = len;
    t->count = 0;
    t->scrambled = (seg >= SEG_TRN);
    switch (seg)
    {
    case SEG_TRN:
        /* 5.2.3: the scrambler starts from all zeros, with ones going in. */
        t->scr = 0;
        if (v->ec_request)
            ec_schedule(v, boundary_time(t->nsym));
        break;
    case SEG_RATE:
        /* 8/V.32 bis: renegotiation's rate signals start the scrambler
         * from zero; the differential coder carries on from the preamble.
         * In start-up and retrain both carry on from TRN (5.3). */
        if (t->rn)
            t->scr = 0;
        t->wbit = 0;
        break;
    case SEG_E:
        t->wbit = 0;
        break;
    case SEG_DATA:
        /* 5.4.1: the convolutional encoder's delay elements start at zero. */
        t->conv = 0;
        t->rn = false;
        break;
    default:
        break;
    }
}

/* Replace whatever is going out with this, from the next symbol. */
static void tx_set(dm_v32_t *v, seg_t seg, long long len)
{
    v->tx.switch_at = -1;
    v->tx.nprog = v->tx.iprog = 0;
    v->tx.e_pending = false;
    v->tx.e_min = 0;
    v->tx.rn = false;
    tx_enter(v, seg, len);
}

/* And then this, once the current segment has run its length. */
static void tx_then(dm_v32_t *v, seg_t seg, long long len)
{
    tx_t *t = &v->tx;

    if (t->nprog < (int) (sizeof(t->prog) / sizeof(t->prog[0])))
    {
        t->prog[t->nprog].seg = seg;
        t->prog[t->nprog].len = len;
        t->nprog++;
    }
}

/* Change to this at symbol k exactly, which is how the turn-rounds land 64
 * symbols after what provoked them. */
static void tx_schedule(dm_v32_t *v, long long k, seg_t seg, long long len)
{
    if (k < v->tx.nsym)
        k = v->tx.nsym;
    v->tx.nprog = v->tx.iprog = 0;
    v->tx.switch_at = k;
    v->tx.switch_seg = seg;
    v->tx.switch_len = len;
}

static int tx_data_bit(dm_v32_t *v)
{
    int bit = 1;

    if (v->tx.data_on && v->get_bit != NULL)
        bit = v->get_bit(v->user) & 1;
    return scramble(&v->tx.scr, v->scr_tap, bit);
}

/* One rate signal dibit, scrambled and coded as Table 1 says (5.3). */
static cf_t tx_word_symbol(dm_v32_t *v, uint16_t word)
{
    tx_t *t = &v->tx;
    int b1 = (word >> (15 - t->wbit)) & 1;
    int b2 = (word >> (14 - t->wbit)) & 1;
    int q1;
    int q2;

    t->wbit += 2;
    /* Two statements: the scrambler is stateful, and C leaves the order of
     * the operands of | unspecified. GCC on x86_64 scrambles b2 first. */
    q1 = scramble(&t->scr, v->scr_tap, b1);
    q2 = scramble(&t->scr, v->scr_tap, b2);
    t->y = TABLE1[(q1 << 1) | q2][t->y];
    return v->abcd[QUAD_OF_Y[t->y]];
}

static cf_t tx_symbol(dm_v32_t *v)
{
    tx_t *t = &v->tx;
    cf_t s = 0.0f;

    if (t->switch_at >= 0 && t->nsym >= t->switch_at)
    {
        t->switch_at = -1;
        tx_enter(v, t->switch_seg, t->switch_len);
    }
    while (t->len >= 0 && t->count >= t->len)
    {
        if (t->iprog < t->nprog)
        {
            seg_step_t step = t->prog[t->iprog++];

            tx_enter(v, step.seg, step.len);
        }
        else
        {
            tx_enter(v, SEG_SILENCE, -1);
        }
    }

    switch (t->seg)
    {
    case SEG_SILENCE:
        break;
    /* These four keep the differential coder's state, because renegotiation
     * starts its rate signal from the last of them (8/V.32 bis). */
    case SEG_AA:
        t->y = Y_OF_QUAD[ST_A];
        s = v->abcd[ST_A];
        break;
    case SEG_CC:
        t->y = Y_OF_QUAD[ST_C];
        s = v->abcd[ST_C];
        break;
    case SEG_AC:
        t->y = Y_OF_QUAD[(t->count & 1) ? ST_C : ST_A];
        s = v->abcd[(t->count & 1) ? ST_C : ST_A];
        break;
    case SEG_CA:
        t->y = Y_OF_QUAD[(t->count & 1) ? ST_A : ST_C];
        s = v->abcd[(t->count & 1) ? ST_A : ST_C];
        break;
    case SEG_S:
        s = v->abcd[(t->count & 1) ? ST_B : ST_A];
        break;
    case SEG_SBAR:
        s = v->abcd[(t->count & 1) ? ST_D : ST_C];
        break;
    case SEG_TRN:
    {
        /* 5.2.3 and Table 5. No differential coding; the first 256 symbols
         * use only A and C, chosen by the first bit of each dibit. */
        int b1 = scramble(&t->scr, v->scr_tap, 1);
        int b2 = scramble(&t->scr, v->scr_tap, 1);

        t->y = (t->count < 256) ? (b1 ? 3 : 0) : ((b1 << 1) | b2);
        s = v->abcd[QUAD_OF_Y[t->y]];
        break;
    }
    case SEG_RATE:
        s = tx_word_symbol(v, t->word);
        if (t->wbit >= 16)
        {
            t->wbit = 0;
            /* 5.3.2: finish the sequence in hand, then send E - and in a
             * renegotiation, not before R4 has had 64 symbols (8.1/V.32 bis). */
            if (t->e_pending && t->count + 1 >= t->e_min)
            {
                t->e_pending = false;
                t->len = t->count + 1;
                t->nprog = t->iprog = 0;
                tx_then(v, SEG_E, 8);
                tx_then(v, SEG_DATA, -1);
            }
        }
        break;
    case SEG_E:
        s = tx_word_symbol(v, t->eword);
        break;
    case SEG_DATA:
    {
        /* Q1 and Q2 go through the differential coder; Q3 onwards, first
         * in time most significant, pick the point within its subset. */
        const coding_info_t *ci = &CODING[t->coding];
        int nr = ci->bits - 2;
        int q1 = tx_data_bit(v);
        int q2 = tx_data_bit(v);
        int q = (q1 << 1) | q2;
        int rest = 0;

        for (int i = 0; i < nr; i++)
            rest = (rest << 1) | tx_data_bit(v);
        if (t->coding == CODE_4800)
        {
            t->y = TABLE1[q][t->y];
            s = v->abcd[QUAD_OF_Y[t->y]];
        }
        else if (!ci->tcm)
        {
            t->y = TABLE1[q][t->y];
            s = v->map16[(t->y << 2) | rest];
        }
        else
        {
            int y0 = t->conv & 1;

            t->y = TABLE2[q][t->y];
            s = v->cmap[t->coding][(y0 << (nr + 2)) | (t->y << nr) | rest];
            t->conv = v->vnext[t->conv][t->y];
        }
        break;
    }
    }
    t->count++;
    return s;
}

int dm_v32_tx(dm_v32_t *v, int16_t *amp, int len)
{
    tx_t *t = &v->tx;

    for (int i = 0; i < len; i++)
    {
        long long n = t->nsamp;
        long long k = (3 * n) / 10;
        int mu = (int) ((3 * n) % 10);
        int ph = (int) ((9 * n) % 40);
        cf_t b = 0.0f;
        float x;

        while (t->nsym <= k)
        {
            cf_t s = tx_symbol(v);

            t->hist[t->nsym & (TX_HIST - 1)] = s;
            t->nsym++;
        }
        for (int m = 0; m < TX_TAPS; m++)
            b += t->hist[(k - m) & (TX_HIST - 1)] * v->gtx[mu][m];
        x = (crealf(b) * v->cos40[ph] - cimagf(b) * v->sin40[ph]) * v->tx_gain;
        if (x > 32767.0f)
            x = 32767.0f;
        else if (x < -32768.0f)
            x = -32768.0f;
        amp[i] = (int16_t) lrintf(x);
        v->txh[n & TXH_MASK] = (float) amp[i];
        v->txflag[n & TXH_MASK] = (uint8_t) t->scrambled;
        t->nsamp++;
    }
    return len;
}

/* ------------------------------------------------------- echo canceller
 *
 * Our own signal comes back to us from wherever there is a 2-wire hybrid on
 * the path - over SIP into the telephone network, that is the far end's line
 * card, a whole round trip away. V.32 puts both directions in the same band,
 * so nothing but subtraction will separate it from what the far end is
 * saying, and an echo only 10 dB down is enough to make 9600 impossible.
 *
 * The start-up procedure leaves each modem a stretch of its own TRN with the
 * far end silent. We cross-correlate what comes back against what we sent
 * to find where the echo is, place a 16 ms adaptive filter around it, and
 * let it learn while the far end is still quiet. Once the far end is talking
 * it keeps adapting, much more slowly, because the far end's signal is noise
 * as far as the canceller is concerned. */

static void ec_schedule(dm_v32_t *v, double trn_line_time)
{
    ec_t *ec = &v->ec;
    double rtd = (v->rtd >= 0.0) ? v->rtd : 4000.0;

    v->ec_request = false;
    ec->state = EC_WAIT;
    /* Our TRN's echo is back by one round trip; give it a little longer to
     * be sure the S that went before it has died away. */
    ec->corr_start = (long long) (trn_line_time + rtd) + 320;
    ec->corr_lmax = (int) rtd + 480;
    if (ec->corr_lmax > EC_CORR_MAX)
        ec->corr_lmax = EC_CORR_MAX;
}

static void ec_decide(dm_v32_t *v)
{
    ec_t *ec = &v->ec;
    double best = 0.0;
    int p = 0;
    double etx = 0.0;
    double rho;

    ec->state = EC_DONE;
    for (int lag = 0; lag < ec->corr_lmax; lag++)
        if (fabs(ec->corr[lag]) > best)
        {
            best = fabs(ec->corr[lag]);
            p = lag;
        }
    for (long long n = ec->corr_start; n < ec->corr_start + EC_CORR_WIN; n++)
    {
        float x = v->txh[(n - p) & TXH_MASK];

        etx += (double) x * x;
    }
    rho = (ec->corr_erx > 0.0 && etx > 0.0) ? best / sqrt(ec->corr_erx * etx) : 0.0;
    ec->erl_db = (ec->corr_erx > 0.0) ? (float) (10.0 * log10(etx / ec->corr_erx)) : 99.0f;

    /* A real echo correlates strongly with what we sent; a far end that is
     * genuinely silent leaves only the codec's noise, which does not. Under
     * -60 dBm0 there is nothing worth cancelling either way. */
    if (rho < 0.2 || ec->corr_erx / EC_CORR_WIN < v->p0 * 1e-6)
    {
        DM_DEBUG("v32", "no echo of our own signal came back (correlation %.2f, %.1f dBm0) "
                        "(tag=%s)",
                 rho, 10.0 * log10(ec->corr_erx / EC_CORR_WIN / v->p0 + 1e-12), v->tag);
        if (ec->enabled)
        {
            /* The path has changed since the last training. */
            ec->enabled = false;
        }
        return;
    }
    ec->enabled = true;
    ec->delay = (p > EC_PRE) ? p - EC_PRE : 0;
    memset(ec->w, 0, sizeof(ec->w));
    ec->mu = EC_MU_FAST;
    ec->fast_from = v->n;
    ec->in_avg = ec->res_avg = 0.0f;
    ec->erle_db = 0.0f;
    DM_DEBUG("v32", "our own signal comes back %.1f ms later, %.1f dB down (correlation %.2f); "
                    "cancelling it (tag=%s)",
             p / 8.0, ec->erl_db, rho, v->tag);
}

/* The far end has started talking: from now on, learn slowly. */
static void ec_slow(dm_v32_t *v)
{
    ec_t *ec = &v->ec;

    if (!ec->enabled || ec->mu == EC_MU_SLOW)
        return;
    if (ec->res_avg > 0.0f && ec->in_avg > 0.0f)
        ec->erle_db = 10.0f * log10f(ec->in_avg / ec->res_avg);
    ec->mu = EC_MU_SLOW;
    ec->slow_from = v->n;
    DM_DEBUG("v32", "echo canceller trained: %.1f dB of cancellation (tag=%s)", ec->erle_db, v->tag);
}

static float ec_run(dm_v32_t *v, float x, long long n)
{
    ec_t *ec = &v->ec;
    long long i0;
    float est = 0.0f;
    float norm = 0.0f;
    float e;

    if (ec->state == EC_WAIT && n >= ec->corr_start)
    {
        ec->state = EC_CORR;
        memset(ec->corr, 0, sizeof(double) * EC_CORR_MAX);
        ec->corr_erx = 0.0;
    }
    if (ec->state == EC_CORR)
    {
        double *c = ec->corr;

        for (int lag = 0; lag < ec->corr_lmax; lag++)
            c[lag] += (double) x * v->txh[(n - lag) & TXH_MASK];
        ec->corr_erx += (double) x * x;
        if (n >= ec->corr_start + EC_CORR_WIN - 1)
            ec_decide(v);
    }

    if (!ec->enabled)
        return x;

    i0 = n - ec->delay;
    for (int k = 0; k < EC_TAPS; k++)
    {
        float r = v->txh[(i0 - k) & TXH_MASK];

        est += ec->w[k] * r;
        norm += r * r;
    }
    e = x - est;

    if (ec->mu > 0.0f && n >= v->freeze_until && v->txflag[i0 & TXH_MASK] && norm > 1.0f)
    {
        float mu = ec->mu;
        float g;

        /* A large step finds the echo quickly, and then learns the line
         * noise along with it - a third of the noise power, at 0.5. So
         * start large and anneal, and leave it small for the far end. */
        if (mu == EC_MU_FAST)
        {
            mu = EC_MU_FAST * expf(-(float) (n - ec->fast_from) / 1000.0f);
            if (mu < EC_MU_SETTLED)
                mu = EC_MU_SETTLED;
        }
        else
        {
            /* With the far end talking, its signal is noise to the
             * canceller, and some of that noise stays in the taps: about
             * mu/2 of the far end's power. At 0.002 that was 30 dB down -
             * nothing at 9600, and the ceiling on SNR at 14 400. So
             * refine quickly for a few seconds, then settle to 0.0002,
             * which leaves it 40 dB down and still follows a slow drift. */
            mu = EC_MU_SLOW * expf(-(float) (n - ec->slow_from) / 16000.0f);
            if (mu < EC_MU_FLOOR)
                mu = EC_MU_FLOOR;
        }
        g = mu * e / (norm + 1000.0f);

        for (int k = 0; k < EC_TAPS; k++)
            ec->w[k] += g * v->txh[(i0 - k) & TXH_MASK];

        if (ec->mu == EC_MU_FAST)
        {
            /* While the far end is silent, everything left is echo we have
             * not cancelled yet. Once the canceller has had time to
             * converge, a sudden rise means the far end has started
             * talking - and learning from that at this rate would undo
             * the training. */
            bool settled = (n - ec->fast_from) > EC_CONVERGE;
            float e2 = e * e;

            if (settled && v->tn.ptot > 8.0f * ec->res_avg && v->tn.ptot > v->pmin)
            {
                ec_slow(v);
            }
            else
            {
                /* The reference must not be able to chase a sudden rise
                 * faster than the 40-sample window that is compared with
                 * it, or the far end's arrival is never noticed. */
                if (settled && e2 > 4.0f * ec->res_avg)
                    e2 = 4.0f * ec->res_avg;
                ec->in_avg += 0.002f * (x * x - ec->in_avg);
                ec->res_avg += 0.002f * (e2 - ec->res_avg);
            }
        }
    }
    return e;
}

/* -------------------------------------------------------- tone detection */

static void tones_update(dm_v32_t *v, float x, long long n)
{
    static const int step[3] = { 3, 9, 15 }; /* 600, 1800, 3000 Hz, in 40ths of 8000 */
    tones_t *t = &v->tn;
    int pos = t->pos;

    for (int f = 0; f < 3; f++)
    {
        int ph = (int) ((step[f] * n) % 40);
        float pr = x * v->cos40[ph];
        float pi = -x * v->sin40[ph];
        cf_t c;

        t->sre[f] += pr - t->pre[f][pos];
        t->sim[f] += pi - t->pim[f][pos];
        t->pre[f][pos] = pr;
        t->pim[f][pos] = pi;
        c = (float) t->sre[f] + I * (float) t->sim[f];
        t->hist[f][n & (TONE_HIST - 1)] = c;
        t->p[f] = 2.0f * mag2(c) / (TONE_WIN * TONE_WIN);
    }
    t->spw += x * x - t->ppw[pos];
    t->ppw[pos] = x * x;
    t->ptot = (float) (t->spw / TONE_WIN);
    if (t->ptot < 0.0f)
        t->ptot = 0.0f;

    if (++t->pos == TONE_WIN)
    {
        /* Rebuild the running sums now and then, so rounding cannot creep. */
        t->pos = 0;
        t->spw = 0.0;
        for (int f = 0; f < 3; f++)
            t->sre[f] = t->sim[f] = 0.0;
        for (int i = 0; i < TONE_WIN; i++)
        {
            t->spw += t->ppw[i];
            for (int f = 0; f < 3; f++)
            {
                t->sre[f] += t->pre[f][i];
                t->sim[f] += t->pim[f][i];
            }
        }
    }
}

#define M600 1u
#define M1800 2u
#define M3000 4u

/* A phase reversal in the tones named by mask (bit 0 600 Hz, bit 1 1800,
 * bit 2 3000). Comparing each detector with itself 10 ms earlier tolerates
 * the +/- 7 Hz of frequency offset 2.1 allows. *when gets the instant of the
 * reversal itself: the boxcar's output passes through zero when the reversal
 * sits in the middle of its window.
 *
 * The tone has to have been there, on its own, at both ends of the
 * comparison. Otherwise the onset of AC straight after ANS - whose 2100 Hz
 * leaks into both detectors at -25 dB in some phase or other - can look like
 * a reversal. Nothing else is required beforehand: an answering modem need
 * only send AC for 64 symbols after hearing AA, so on a short path the
 * reversal can come very soon after AC does. */
static bool reversal(dm_v32_t *v, unsigned mask, long long n, double *when)
{
    tones_t *t = &v->tn;
    float dot = 0.0f;
    float mag = 0.0f;
    float floor_mag = v->pmin * TONE_WIN * TONE_WIN / 2.0f;

    unsigned need = (mask == M1800) ? PRESENT_AA : PRESENT_AC;

    if (!v->rev_armed || n < v->rev_rearm || n < TONE_HIST)
        return false;
    if (!(t->present[n & (TONE_HIST - 1)] & need) ||
        !(t->present[(n - TONE_LAG) & (TONE_HIST - 1)] & need))
        return false;
    for (int f = 0; f < 3; f++)
    {
        cf_t a, b;

        if (!(mask & (1u << f)))
            continue;
        a = t->hist[f][n & (TONE_HIST - 1)];
        b = t->hist[f][(n - TONE_LAG) & (TONE_HIST - 1)];
        dot += crealf(a * conjf(b));
        mag += cabsf(a) * cabsf(b);
    }
    if (mag < floor_mag || dot > -0.5f * mag)
        return false;

    {
        float best = 1e30f;
        int bi = 0;

        for (int i = 0; i <= TONE_LAG; i++)
        {
            float m = 0.0f;

            for (int f = 0; f < 3; f++)
                if (mask & (1u << f))
                    m += mag2(t->hist[f][(n - i) & (TONE_HIST - 1)]);
            if (m < best)
            {
                best = m;
                bi = i;
            }
        }
        *when = (double) (n - bi) - (TONE_WIN - 1) / 2.0;
    }
    v->rev_rearm = n + 2 * TONE_LAG;
    return true;
}


/* --------------------------------------------------------------- slicing */

static int slice4(const dm_v32_t *v, cf_t u)
{
    int best = 0;
    float bd = 1e30f;

    for (int i = 0; i < 4; i++)
    {
        float d = mag2(u - v->abcd[i]);

        if (d < bd)
        {
            bd = d;
            best = i;
        }
    }
    return best;
}

static int axis4(float x)
{
    if (x < -2.0f)
        return 0;
    if (x < 0.0f)
        return 1;
    if (x < 2.0f)
        return 2;
    return 3;
}

static int slice16(const dm_v32_t *v, cf_t u)
{
    return v->idx16[axis4(crealf(u))][axis4(cimagf(u))];
}

/* The nearest point of a trellis coded constellation, as a tentative
 * decision for the equaliser and the phase loop. */
static int slice_tcm(const dm_v32_t *v, coding_t c, cf_t u)
{
    const cf_t *map = v->cmap[c];
    int n = 2 << CODING[c].bits;
    int best = 0;
    float bd = 1e30f;

    for (int i = 0; i < n; i++)
    {
        float d = mag2(u - map[i]);

        if (d < bd)
        {
            bd = d;
            best = i;
        }
    }
    return best;
}

/* --------------------------------------------------------------- Viterbi */

static void vit_reset(vit_t *vt)
{
    for (int s = 0; s < 8; s++)
        vt->pm[s] = 1e6f;
    /* The encoder starts at zero (5.4.1). */
    vt->pm[0] = 0.0f;
    vt->pos = 0;
    vt->count = 0;
}

/* One received point in; once the decoder is VIT_DEPTH symbols deep, one
 * decided Y1 Y2 Q3 ... out from that many symbols ago - Y0 left off, so 0..15
 * at 9600 and 0..63 at 14 400. */
static int vit_step(dm_v32_t *v, cf_t u)
{
    vit_t *vt = &v->rx.vit;
    const cf_t *map = v->cmap[v->rx.coding];
    int nr = CODING[v->rx.coding].bits - 2;
    float bm[8];
    int bq[8];
    float npm[8];
    uint8_t np[8] = { 0 };
    uint8_t nb[8] = { 0 };
    float lo = 1e30f;
    int best = 0;
    int s;

    /* Each of the eight subsets Y0 Y1 Y2 holds 2^nr points - two at 7200,
     * sixteen at 14 400; a branch through a subset costs the distance to
     * its nearest one. */
    for (int sub = 0; sub < 8; sub++)
    {
        bm[sub] = 1e30f;
        bq[sub] = 0;
        for (int q = 0; q < (1 << nr); q++)
        {
            float d = mag2(u - map[(sub << nr) | q]);

            if (d < bm[sub])
            {
                bm[sub] = d;
                bq[sub] = q;
            }
        }
    }
    for (int i = 0; i < 8; i++)
        npm[i] = 1e30f;
    for (int ps = 0; ps < 8; ps++)
        for (int yy = 0; yy < 4; yy++)
        {
            int ns = v->vnext[ps][yy];
            int sub = ((ps & 1) << 2) | yy; /* Y0 is S3 */
            float m = vt->pm[ps] + bm[sub];

            if (m < npm[ns])
            {
                npm[ns] = m;
                np[ns] = (uint8_t) ps;
                nb[ns] = (uint8_t) ((yy << nr) | bq[sub]);
            }
        }
    for (int i = 0; i < 8; i++)
        if (npm[i] < lo)
        {
            lo = npm[i];
            best = i;
        }
    for (int i = 0; i < 8; i++)
    {
        vt->pm[i] = npm[i] - lo;
        vt->prev[vt->pos][i] = np[i];
        vt->bits[vt->pos][i] = nb[i];
    }
    vt->pos = (vt->pos + 1) % VIT_DEPTH;
    if (++vt->count < VIT_DEPTH)
        return -1;

    /* Back from the best path's end to the oldest symbol still held. */
    s = best;
    for (int i = 1; i < VIT_DEPTH; i++)
        s = vt->prev[(vt->pos - i + VIT_DEPTH) % VIT_DEPTH][s];
    return vt->bits[vt->pos][s];
}

/* -------------------------------------------------------------- receiver */

static void ctl_rate_word(dm_v32_t *v, uint16_t w);
static bool ctl_e_word(dm_v32_t *v, uint16_t w);
static void ctl_rx_trained(dm_v32_t *v);
static void stage_enter(dm_v32_t *v, stage_t st, double timeout_s);

static void rx_reset(dm_v32_t *v)
{
    rx_t *r = &v->rx;

    r->mode = RX_TONES;
    r->data_on = false;
    r->rw_locked = false;
    r->rw_bits = 0;
}

static void rx_fail(dm_v32_t *v, const char *why)
{
    DM_DEBUG("v32", "receiver training failed: %s (tag=%s)", why, v->tag);
    rx_reset(v);
}

static void rx_start_acq(dm_v32_t *v)
{
    rx_t *r = &v->rx;

    r->mode = RX_ACQ;
    r->acq_count = 0;
    r->acq_m1 = r->acq_m2 = r->acq_pp = r->acq_pm = 0.0f;
    DM_DEBUG("v32", "S heard; training the receiver (tag=%s)", v->tag);
}

/* The timing loop is second order: the far modem's crystal is allowed to be
 * 100 ppm out (2.3), and a first order loop would answer a steady drift with
 * a steady error. The integral gain is set for critical damping. */
static void set_loops(rx_t *r, float beta, float a1, float a2, float k_timing)
{
    r->beta = beta;
    r->a1 = a1;
    r->a2 = a2;
    r->k_timing = k_timing;
}

static cf_t mf_at(const dm_v32_t *v, double tau)
{
    const rx_t *r = &v->rx;
    long long n0 = (long long) floor(tau);
    int p = (int) lrint((tau - (double) n0) * MF_PHASES);
    const float *h = v->hr[p];
    cf_t y = 0.0f;

    for (int j = -MF_HALF; j <= MF_HALF; j++)
        y += r->z[(n0 + j) & ZMASK] * h[j + MF_HALF];
    return y;
}

/* S, through the channel, is G (m + d cos(pi (t - tA) / T)) at baseband:
 * m = (A + B) / 2 a constant, d = (A - B) / 2 a 1200 Hz alternation. Over a
 * whole number of its cycles the mean gives the channel's gain and phase
 * outright, and the 1200 Hz component gives the instant an A peaks - the
 * symbol timing - without waiting for a loop to converge. */
#define ACQ_N 200

static void acq_finish(dm_v32_t *v, long long n, long long t_last)
{
    rx_t *r = &v->rx;
    const cf_t mref = (v->abcd[ST_A] + v->abcd[ST_B]) * 0.5f;
    const cf_t dref = (v->abcd[ST_A] - v->abcd[ST_B]) * 0.5f;
    cf_t mm = (r->acq_m1 + r->acq_m2) / (float) ACQ_N;
    cf_t gc = mm / mref;
    cf_t e1, e2, e;
    double ta, tau0, tmid, nu;
    long long kk;

    if (cabsf(gc) < 1e-4f)
    {
        rx_fail(v, "no signal where S should be");
        return;
    }
    e1 = 2.0f * (r->acq_pm / (float) ACQ_N) * mref / (mm * dref);
    e2 = 2.0f * (r->acq_pp / (float) ACQ_N) * mref / (mm * dref);
    e = e1 + conjf(e2);
    if (cabsf(e) < 1.0f)
    {
        rx_fail(v, "what was heard does not look like S");
        return;
    }
    ta = carg(e) * SPS / V32_PI;
    nu = carg(r->acq_m2 * conjf(r->acq_m1)) / (ACQ_N / 2.0);
    tmid = (double) t_last - ACQ_N / 2.0;

    /* Start the resampler on an A as far back as the buffer allows, so the
     * equaliser's delay line fills with S we have already heard. */
    kk = (long long) ceil(((double) n - 260.0 - ta) / (2.0 * SPS));
    tau0 = ta + 2.0 * SPS * (double) kk;

    memset(r->line, 0, sizeof(r->line));
    memset(r->c, 0, sizeof(r->c));
    r->c[EQ_CENTER] = 1.0f / gc;
    r->lpos = EQ_LEN;
    r->halves = 0;
    r->tau = tau0 - T_HALF;
    r->on_time = false;
    r->have_prev = false;
    r->t_freq = 0.0f;
    r->py = mag2(mm) * 1.5f;
    r->nu = (float) (nu * SPS);
    /* The phase as it will be at the first output, which waits for the
     * delay line to fill and looks at its centre tap. */
    r->theta = (float) (nu * (tau0 + (EQ_LEN / 2 - 1) * SPS - EQ_CENTER * T_HALF - tmid));
    r->mse = 1.0f;
    r->osym = 0;
    r->s_good = 0;
    r->last_q = -1;
    r->center_only = true;
    r->leak = 0.0f;
    set_loops(r, 0.05f, 0.1f, 0.003f, 0.05f);
    r->mode = RX_S;
    DM_DEBUG("v32", "S measured: level %.1f dB, frequency offset %.2f Hz (tag=%s)",
             20.0 * log10(cabsf(gc)), nu * 8000.0 / (2.0 * V32_PI), v->tag);
}

static void acq_sample(dm_v32_t *v, long long n)
{
    rx_t *r = &v->rx;
    long long t = n - MF_HALF;
    cf_t y = mf_at(v, (double) t);
    int ph = (int) ((6 * t) % 40); /* 1200 Hz */
    cf_t rot = v->cos40[ph] + I * v->sin40[ph];

    r->acq_pm += y * rot;
    r->acq_pp += y * conjf(rot);
    if (r->acq_count < ACQ_N / 2)
        r->acq_m1 += y;
    else
        r->acq_m2 += y;
    if (++r->acq_count == ACQ_N)
        acq_finish(v, n, t);
}

/* Phase and equaliser update towards target d, which is a decision or, in
 * training, the symbol known to have been sent. */
static void track(dm_v32_t *v, const cf_t *w, float norm, cf_t vout, cf_t u, cf_t d)
{
    rx_t *r = &v->rx;
    float dd = mag2(d);
    float pe = (dd > 0.0f) ? cimagf(u * conjf(d)) / dd : 0.0f;
    cf_t err = d * (cosf(r->theta) + I * sinf(r->theta)) - vout;

    r->mse += 0.005f * (mag2(u - d) - r->mse);
    if (v->n < v->freeze_until)
        return;

    r->theta += r->a1 * pe + r->nu;
    r->nu += r->a2 * pe;
    if (r->theta > V32_PI)
        r->theta -= (float) (2.0 * V32_PI);
    else if (r->theta < -V32_PI)
        r->theta += (float) (2.0 * V32_PI);

    if (r->center_only)
    {
        float wn = mag2(w[EQ_CENTER]);

        if (wn > 0.0f)
            r->c[EQ_CENTER] += r->beta * err * conjf(w[EQ_CENTER]) / wn;
    }
    else
    {
        cf_t g = r->beta * err / (norm + 1e-6f);

        for (int i = 0; i < EQ_LEN; i++)
            r->c[i] = r->c[i] * (1.0f - r->leak) + g * conjf(w[i]);
    }
}

static void deliver(dm_v32_t *v, int bit)
{
    int b = descramble(&v->rx.dscr, v->dscr_tap, bit);

    if (v->rx.data_on && v->put_bit != NULL)
        v->put_bit(v->user, b);
}

static void rate_dibit(dm_v32_t *v, int b1, int b2)
{
    rx_t *r = &v->rx;
    uint16_t w;

    r->rw = (r->rw << 2) | ((uint32_t) b1 << 1) | (uint32_t) b2;
    r->rw_bits += 2;
    w = (uint16_t) (r->rw & 0xFFFF);
    if (!r->rw_locked)
    {
        /* 5.3.1: two identical sequences in a row, sync bits correct. */
        if (r->rw_bits >= 32 && rate_word_ok(w) && w == (uint16_t) (r->rw >> 16))
        {
            r->rw_locked = true;
            r->rw_phase = (int) (r->rw_bits % 16);
            r->rw_bad = 0;
            ctl_rate_word(v, w);
        }
        return;
    }
    if ((r->rw_bits - r->rw_phase) % 16 != 0)
        return;
    if (e_word_ok(w))
    {
        if (ctl_e_word(v, w))
            r->rw_locked = false;
        return;
    }
    if (rate_word_ok(w))
        r->rw_bad = 0;
    else if (++r->rw_bad >= 2)
        r->rw_locked = false;
}

static void rx_enter_data(dm_v32_t *v, coding_t coding)
{
    rx_t *r = &v->rx;

    r->mode = RX_DATA;
    r->coding = coding;
    r->data_rcvd = 0;
    r->prev_y = Y_OF_QUAD[r->prev_q];
    vit_reset(&r->vit);
    r->vit_mse = r->mse;
    r->pre_run = 0;
    r->pre_prev = 0.0f;
    set_loops(r, 0.01f, 0.03f, 0.0005f, 0.01f);
    /* A little leakage keeps the fractionally spaced taps from wandering
     * off in the band edges over a long call. Not much: the taps settle
     * where the leak and the error balance, and at 1e-5 that held every rate
     * to 31 dB SNR - nothing at 9600, a third of the margin at 14 400. */
    r->leak = 1e-6f;
}

static cf_t trn_symbol(const dm_v32_t *v, uint32_t *reg, long long j)
{
    int b1 = scramble(reg, v->dscr_tap, 1);
    int b2 = scramble(reg, v->dscr_tap, 1);

    return v->abcd[(j < 256) ? (b1 ? ST_C : ST_A) : QUAD_OF_Y[(b1 << 1) | b2]];
}

static void trn_align(dm_v32_t *v)
{
    rx_t *r = &v->rx;
    cf_t ref[40];
    uint32_t reg = 0;
    double best = -1e30;
    int bo = 0;
    double norm = 0.0;

    for (int i = 0; i < 40; i++)
        ref[i] = trn_symbol(v, &reg, i);
    for (int o = -3; o <= 3; o++)
    {
        double sc = 0.0;

        for (int i = 0; i < 40; i++)
            sc += crealf(r->abuf[o + 3 + i] * conjf(ref[i]));
        if (sc > best)
        {
            best = sc;
            bo = o;
        }
    }
    for (int i = 0; i < 40; i++)
        norm += cabsf(r->abuf[bo + 3 + i]) * cabsf(ref[i]);
    if (best < 0.5 * norm)
    {
        rx_fail(v, "TRN did not match the sequence it should be");
        return;
    }
    /* The newest output is abuf[45], which is TRN symbol 42 - bo. */
    r->trn_scr = 0;
    r->trn_j = 0;
    while (r->trn_j < 43 - bo)
    {
        (void) trn_symbol(v, &r->trn_scr, r->trn_j);
        r->trn_j++;
    }
    r->center_only = false;
    set_loops(r, 0.15f, 0.08f, 0.002f, 0.03f);
    r->mode = RX_TRN_DA;
    if (bo != 0)
        DM_DEBUG("v32", "TRN found %d symbol%s from where S-bar put it (tag=%s)", bo,
                 (bo == 1 || bo == -1) ? "" : "s", v->tag);
}

/* 8/V.32 bis: the far end asking to change rate. Its preamble is 56 symbols
 * of AA from a caller or AC from an answerer, then 8 of CC or CA, then its
 * rate signal - sent in the middle of data, with nothing else to announce
 * it. So in data, every symbol is looked at for being the far end's preamble
 * symbol: a point of the 4800 bit/s set, the same as the one before (AA) or
 * opposite it (AC). Twenty-four in a row of those does not happen in
 * scrambled data at any rate, and leaves thirty-odd symbols of the preamble
 * still to come. Rotations by quarter turns change none of this, which
 * matters because a trellis coded receiver's phase may have slipped by one
 * without anything noticing. */
#define RN_PRE_RUN 24

static bool rn_pre_watch(dm_v32_t *v, cf_t u)
{
    rx_t *r = &v->rx;
    cf_t want = v->calling ? -r->pre_prev : r->pre_prev;
    float p = mag2(u);
    bool like = p > 5.0f && p < 16.0f && mag2(u - want) < 2.0f;

    r->pre_prev = u;
    if (!v->far_bis || v->rn_deaf || (v->stage != STG_DATA && v->stage != STG_RN_SEND))
        return false;
    r->pre_run = like ? r->pre_run + 1 : 0;
    if (r->pre_run < RN_PRE_RUN)
        return false;

    /* 8.1 and 8.2/V.32 bis: clamp circuit 104, and listen for the rate
     * signal. */
    r->mode = RX_RN_PRE;
    r->data_on = false;
    r->pre_count = 0;
    r->pre_tail = 0;
    r->last_q = slice4(v, u);
    if (v->stage == STG_DATA)
    {
        stage_enter(v, STG_RN_RESP, 3.0 + ((v->rtd >= 0.0) ? v->rtd / 8000.0 : 0.5));
        DM_INFO("v32", "the far end is asking to change rate (tag=%s)", v->tag);
    }
    else
    {
        DM_DEBUG("v32", "the far end's preamble heard; waiting for its rate signal (tag=%s)", v->tag);
    }
    return true;
}

/* Through the preamble to the instant its rate signal starts. The 56 symbols
 * end where the pattern changes - AA turning into C, or AC repeating a
 * symbol as it turns into CA - and the rate signal comes eight symbols after
 * that. Knowing where it starts lets the descrambler start where the far
 * end's scrambler does, from zero, rather than lose the first sequence to
 * resynchronising: the responder sends only four of them. */
static void rn_pre_symbol(dm_v32_t *v, const cf_t *w, float norm, cf_t vout, cf_t u)
{
    rx_t *r = &v->rx;
    int q = slice4(v, u);

    track(v, w, norm, vout, u, v->abcd[q]);
    r->pre_count++;
    if (r->pre_tail == 0)
    {
        bool same = (v->calling ? (q ^ 2) : q) == r->last_q;

        if (!same)
            r->pre_tail = 1;
    }
    else
    {
        r->pre_tail++;
    }
    r->last_q = q;
    r->prev_q = q;
    if (r->pre_tail >= 8 || r->pre_count > 200)
    {
        if (r->pre_tail < 8)
            DM_DEBUG("v32", "the end of the far end's preamble was not seen (tag=%s)", v->tag);
        r->mode = RX_RATE;
        r->dscr = 0;
        r->rw_locked = false;
        r->rw_bits = 0;
    }
}

static void symbol_out(dm_v32_t *v)
{
    rx_t *r = &v->rx;
    const cf_t *w = &r->line[r->lpos];
    cf_t vout = 0.0f;
    float norm = 0.0f;
    cf_t u;
    long long idx;

    for (int i = 0; i < EQ_LEN; i++)
    {
        vout += r->c[i] * w[i];
        norm += mag2(w[i]);
    }
    u = vout * (cosf(r->theta) - I * sinf(r->theta));
    idx = r->osym++;

    switch (r->mode)
    {
    case RX_S:
    {
        int q = slice4(v, u);

        track(v, w, norm, vout, u, v->abcd[q]);
        if (q == ST_A || q == ST_B)
            r->s_good++;
        /* S-bar is C D C D; its first C is where the reference is taken
         * from, and TRN begins sixteen symbols after it (5.2.2). */
        if (r->last_q == ST_C && q == ST_D && r->s_good >= 16)
        {
            r->trn_first = idx - 1 + 16;
            r->abuf_n = 0;
            r->mode = RX_TRN_ALIGN;
            set_loops(r, 0.05f, 0.1f, 0.003f, 0.03f);
            r->center_only = false;
        }
        else if (idx > 3000)
        {
            rx_fail(v, "S never turned into S-bar");
        }
        r->last_q = q;
        break;
    }

    case RX_TRN_ALIGN:
    {
        int q = slice4(v, u);

        track(v, w, norm, vout, u, v->abcd[q]);
        if (idx >= r->trn_first - 3)
        {
            r->abuf[r->abuf_n++] = u;
            if (r->abuf_n == 46)
                trn_align(v);
        }
        break;
    }

    case RX_TRN_DA:
    {
        cf_t ref = trn_symbol(v, &r->trn_scr, r->trn_j);

        track(v, w, norm, vout, u, ref);
        r->prev_q = slice4(v, ref);
        r->trn_j++;
        if (r->trn_j == 300)
            r->beta = 0.05f;
        if (r->trn_j >= TRN_DA_END)
        {
            r->mode = RX_RATE;
            r->rw_locked = false;
            r->rw_bits = 0;
            set_loops(r, 0.02f, 0.05f, 0.001f, 0.02f);
            r->snr_trained = 10.0f * log10f(10.0f / (r->mse + 1e-9f));
            ctl_rx_trained(v);
        }
        break;
    }

    case RX_RATE:
    {
        int q = slice4(v, u);
        int qq = v->inv1[Y_OF_QUAD[r->prev_q]][Y_OF_QUAD[q]];
        int b1;
        int b2;

        track(v, w, norm, vout, u, v->abcd[q]);
        r->prev_q = q;
        /* In order: as arguments, C would leave it to the compiler. */
        b1 = descramble(&r->dscr, v->dscr_tap, qq >> 1);
        b2 = descramble(&r->dscr, v->dscr_tap, qq & 1);
        rate_dibit(v, b1, b2);
        break;
    }

    case RX_DATA:
        if (rn_pre_watch(v, u))
        {
            track(v, w, norm, vout, u, v->abcd[slice4(v, u)]);
            break;
        }
        r->data_rcvd++;
        if (r->coding == CODE_4800)
        {
            int q = slice4(v, u);
            int qq = v->inv1[Y_OF_QUAD[r->prev_q]][Y_OF_QUAD[q]];

            track(v, w, norm, vout, u, v->abcd[q]);
            r->prev_q = q;
            deliver(v, qq >> 1);
            deliver(v, qq & 1);
        }
        else if (r->coding == CODE_9600U)
        {
            int i16 = slice16(v, u);
            int yy = i16 >> 2;
            int qq = v->inv1[r->prev_y][yy];

            track(v, w, norm, vout, u, v->map16[i16]);
            r->prev_y = yy;
            deliver(v, qq >> 1);
            deliver(v, qq & 1);
            deliver(v, (i16 >> 1) & 1);
            deliver(v, i16 & 1);
        }
        else
        {
            const cf_t *map = v->cmap[r->coding];
            int nr = CODING[r->coding].bits - 2;
            int out;

            track(v, w, norm, vout, u, map[slice_tcm(v, r->coding, u)]);
            r->vit_u[r->vit.pos] = u;
            out = vit_step(v, u);
            if (out >= 0)
            {
                int yy = out >> nr;
                int qq = v->inv2[r->prev_y][yy];
                cf_t held = r->vit_u[r->vit.pos];
                float e0 = mag2(held - map[out]);
                float e1 = mag2(held - map[(1 << (nr + 2)) | out]);

                /* The slicer's tentative decisions flatter the error at
                 * low SNR, where the nearest point is often the wrong one;
                 * measure against what the decoder chose. It decides Y1
                 * Y2 Q3 ..., which leaves two points, one per value of
                 * Y0, and the received one is near the right one.
                 *
                 * Not by re-running the encoder over the decoder's output
                 * to recover Y0, which is what this did: the decoder may
                 * switch to another path, the spliced output is then not
                 * one the encoder could have produced, and the copy of the
                 * encoder stays out of step for good - measuring every
                 * symbol against a point in the wrong subset, about 3 dB,
                 * while the data was perfect. That fired retrains on a
                 * line with nothing wrong with it. */
                r->vit_mse += 0.005f * ((e0 < e1 ? e0 : e1) - r->vit_mse);
                r->prev_y = yy;
                deliver(v, qq >> 1);
                deliver(v, qq & 1);
                for (int i = nr - 1; i >= 0; i--)
                    deliver(v, (out >> i) & 1);
            }
        }
        break;

    case RX_RN_PRE:
        rn_pre_symbol(v, w, norm, vout, u);
        break;

    default:
        break;
    }
}

static void half_sample(dm_v32_t *v, cf_t y)
{
    rx_t *r = &v->rx;

    r->lpos = (r->lpos == 0) ? EQ_LEN - 1 : r->lpos - 1;
    r->line[r->lpos] = y;
    r->line[r->lpos + EQ_LEN] = y;
    if (r->halves < EQ_LEN)
        r->halves++;

    if (!r->on_time)
    {
        r->y_mid = y;
        r->on_time = true;
        r->tau += T_HALF;
        return;
    }
    r->on_time = false;
    r->py += 0.01f * (mag2(y) - r->py);
    if (r->have_prev && v->n >= v->freeze_until)
    {
        /* Gardner: sampling late makes this negative, early positive. */
        float e = crealf((r->y_prev - y) * conjf(r->y_mid)) / (r->py + 1e-9f);
        float adj = r->k_timing * e;

        if (adj > 0.2f)
            adj = 0.2f;
        else if (adj < -0.2f)
            adj = -0.2f;
        r->t_freq += 0.125f * r->k_timing * r->k_timing * e;
        if (r->t_freq > 0.01f)
            r->t_freq = 0.01f;
        else if (r->t_freq < -0.01f)
            r->t_freq = -0.01f;
        r->tau += adj + r->t_freq;
    }
    r->y_prev = y;
    r->have_prev = true;
    r->tau += T_HALF;
    if (r->halves >= EQ_LEN)
        symbol_out(v);
}

static void demod(dm_v32_t *v, long long n)
{
    rx_t *r = &v->rx;

    if (r->mode == RX_TONES)
        return;
    if (r->mode == RX_ACQ)
    {
        acq_sample(v, n);
        return;
    }
    while (r->tau + MF_HALF <= (double) n)
    {
        half_sample(v, mf_at(v, r->tau));
        if (r->mode == RX_TONES || r->mode == RX_ACQ)
            return;
    }
}

/* ------------------------------------------------------------- handshake */

static void stage_enter(dm_v32_t *v, stage_t st, double timeout_s)
{
    v->stage = st;
    v->deadline = (timeout_s > 0.0) ? v->n + SECONDS(timeout_s) : 0;
    v->deadline_warned = false;
}

/* Long enough for the echo canceller to find our echo and learn it before
 * the far end starts talking, and never shorter than 1280 symbols (5.2.3). */
static long long trn_length(const dm_v32_t *v)
{
    double rtd = (v->rtd >= 0.0) ? v->rtd : 4000.0;
    long long k = (long long) ceil((rtd + 320.0 + EC_CORR_WIN + EC_CONVERGE) / SPS);

    if (k < 1280)
        k = 1280;
    if (k > 8192)
        k = 8192;
    return k;
}

static const char *coding_name(coding_t c)
{
    return CODING[c].name;
}

/* What our receiver could cope with at a given SNR: as it trained, when
 * choosing what to offer, or as it is now, in data, when renegotiating. */
static bool rx_can(coding_t c, float snr)
{
    return snr >= CODING[c].offer_db;
}

/* What we can offer in reply to the far end's rate signal: the codings we
 * are enabled for, restricted to the ones it allowed (5.4.1 and 6.1/V.32
 * bis), less any our receiver would not bear. */
static unsigned offer_set(const dm_v32_t *v, unsigned far, const char *verb)
{
    unsigned s = v->enabled & far & (v->far_bis ? SET_BIS : SET_V32);
    int lost = -1;
    int kept;

    for (int c = 0; c < CODE_COUNT; c++)
        if ((s & CBIT(c)) && !rx_can((coding_t) c, v->rx.snr_trained))
        {
            s &= ~CBIT(c);
            lost = c;
        }
    /* Worth saying only if it cost speed: dropping the uncoded 9600 while
     * the trellis coded one stays costs nothing. */
    kept = best_of(s);
    if (lost >= 0 && (kept < 0 || CODING[lost].rate > CODING[kept].rate))
        DM_INFO("v32", "the line trained at only %.1f dB SNR; not %s %d (tag=%s)", v->rx.snr_trained, verb,
                CODING[lost].rate, v->tag);
    return s;
}

/* Back to the start of 5.4.1 or 5.4.2 at the third paragraph - which is
 * also 5.5, a retrain. */
static void restart(dm_v32_t *v, const char *why, bool failed)
{
    if (failed)
    {
        DM_INFO("v32", "%s; starting the handshake again (tag=%s)", why, v->tag);
        emit(v, DM_V32_TRAINING_FAILED);
    }
    else
    {
        DM_INFO("v32", "%s; %s (tag=%s)", why, v->trained_once ? "retraining" : "starting the handshake again",
                v->tag);
    }
    rx_reset(v);
    v->rev_armed = false;
    v->want_train = false;
    v->drop_run = 0;
    v->p18_ref = v->p18_avg = 0.0f;
    v->tx.data_on = false;
    v->low_since = v->poor_since = 0;
    if (v->calling)
    {
        tx_set(v, SEG_AA, -1);
        stage_enter(v, STG_C_AA, 20.0);
    }
    else
    {
        tx_set(v, SEG_AC, -1);
        stage_enter(v, STG_A_AC, 20.0);
    }
}

/* The receiver's signal to noise ratio, against the decoder's decisions
 * when there is a decoder. */
static float rx_snr(const dm_v32_t *v)
{
    float mse = (v->rx.mode == RX_DATA && CODING[v->rx.coding].tcm && v->rx.vit_mse > 0.0f)
                    ? v->rx.vit_mse
                    : v->rx.mse;

    return 10.0f * log10f(10.0f / (mse + 1e-9f));
}

static void go_data(dm_v32_t *v)
{
    stage_enter(v, STG_DATA, 0.0);
    v->tx.data_on = true;
    v->rx.data_on = true;
    v->low_since = v->poor_since = 0;
    v->up_since = 0;
    v->up_next = v->n + SECONDS(10.0);
    v->rn_from = -1;
    DM_INFO("v32", "trained at %s%s: SNR %.1f dB, round trip %.0f ms, echo canceller %s (tag=%s)",
            coding_name(v->rx.coding), (v->bis && !v->far_bis) ? ", as V.32" : "", rx_snr(v),
            v->rtd >= 0.0 ? v->rtd / 8.0 : -1.0,
            v->ec.enabled ? "on" : "off - no echo", v->tag);
    if (v->trained_once)
        v->retrains++;
    v->trained_once = true;
    emit(v, DM_V32_TRAINED);
}

static void cleardown(dm_v32_t *v, const char *why)
{
    DM_WARN("v32", "%s (tag=%s)", why, v->tag);
    tx_set(v, SEG_SILENCE, -1);
    rx_reset(v);
    stage_enter(v, STG_DEAD, 0.0);
    emit(v, DM_V32_CLEARDOWN);
}

static void set_coding(dm_v32_t *v, coding_t c)
{
    v->coding = c;
    v->rate = CODING[c].rate;
    v->tcm = CODING[c].tcm;
}

/* --------------------------------------------- rate renegotiation (V.32 bis)
 *
 * Section 8/V.32 bis: either end may ask to change rate in the middle of data,
 * with no retrain - a preamble, a rate signal (R4 from the end that asks, R5
 * from the one that answers), E naming the best rate the two have in common,
 * and 24 symbols of scrambled ones at it. The equaliser, the phase and timing
 * loops and the echo canceller all carry on as they were, which is what makes
 * it take a fraction of a second where a retrain takes several on a long
 * path. */

static double rn_timeout(const dm_v32_t *v)
{
    return 3.0 + ((v->rtd >= 0.0) ? v->rtd / 8000.0 : 0.5);
}

/* The rate signals of a renegotiation offer "the desired rate ... and all
 * lower data signalling rates at which the modem is enabled to operate". */
static unsigned rn_set_upto(const dm_v32_t *v, int want)
{
    unsigned s = 0;

    for (int c = 0; c <= want; c++)
        if (v->enabled & SET_BIS & CBIT(c))
            s |= CBIT(c);
    return s;
}

/* The best coding our receiver would bear now, by the SNR it has in data. */
static int rn_desired(const dm_v32_t *v)
{
    float snr = rx_snr(v);
    int best = CODE_4800;

    for (int c = 0; c < CODE_COUNT; c++)
        if ((v->enabled & SET_BIS & CBIT(c)) && rx_can((coding_t) c, snr))
            best = c;
    return best;
}

/* The preamble: AA for 56 symbols then CC for 8 from the caller, AC then CA
 * from the answerer. The rate signal that follows starts its differential
 * coder from the last of them, which tx_symbol() keeps. */
static void rn_tx_preamble(dm_v32_t *v)
{
    if (v->calling)
    {
        tx_set(v, SEG_AA, 56);
        tx_then(v, SEG_CC, 8);
    }
    else
    {
        tx_set(v, SEG_AC, 56);
        tx_then(v, SEG_CA, 8);
    }
}

/* 8.1/V.32 bis: ask for a change to want, or the best below it the far end
 * will take. */
static bool rn_start(dm_v32_t *v, int want, const char *why)
{
    unsigned offer;

    if (!v->bis || !v->far_bis || v->stage != STG_DATA)
        return false;
    offer = rn_set_upto(v, want);
    if (offer == 0)
        return false;
    v->rn_offer = offer;
    v->rn_from = (int) v->coding;
    rn_tx_preamble(v);
    tx_then(v, SEG_RATE, -1);
    v->tx.word = make_word(false, offer, true);
    v->tx.e_min = 64;
    v->tx.rn = true;
    v->tx.data_on = false;
    stage_enter(v, STG_RN_SEND, rn_timeout(v));
    DM_INFO("v32", "%s; asking the far end to change to %d (tag=%s)", why, CODING[best_of(offer)].rate,
            v->tag);
    return true;
}

/* The far end's R4 (when it asked) or R5 (when we did) is in. */
static void rn_rate_word(dm_v32_t *v, uint16_t w, const char *wt)
{
    char ot[64];
    int c;

    if (v->stage == STG_RN_RESP)
    {
        /* 8.2/V.32 bis: turn 106 off, our preamble, R5 for 64 symbols, E,
         * and 24 symbols of scrambled ones at the new rate. R5 says what we
         * want irrespective of R4, and E the best the two have in common. */
        v->rn_offer = rn_set_upto(v, rn_desired(v));
        c = best_of(set_of(w, true) & v->rn_offer);
        word_text(make_word(false, v->rn_offer, true), ot, sizeof(ot));
        DM_DEBUG("v32", "R4 heard (%s); sending R5 (%s) (tag=%s)", wt, ot, v->tag);
        if (c < 0)
        {
            cleardown(v, "a rate renegotiation found no rate in common");
            return;
        }
        rn_tx_preamble(v);
        tx_then(v, SEG_RATE, 64);
        tx_then(v, SEG_E, 8);
        tx_then(v, SEG_DATA, -1);
        v->tx.word = make_word(false, v->rn_offer, true);
        v->tx.eword = make_word(true, CBIT(c), true);
        v->tx.coding = (coding_t) c;
        v->tx.rn = true;
        v->tx.data_on = false;
    }
    else
    {
        /* 8.1/V.32 bis: once R4 has run 64 symbols, finish the sequence in
         * hand and send E. */
        c = best_of(set_of(w, true) & v->rn_offer);
        DM_DEBUG("v32", "R5 heard (%s) (tag=%s)", wt, v->tag);
        if (c < 0)
        {
            cleardown(v, "a rate renegotiation found no rate in common");
            return;
        }
        v->tx.eword = make_word(true, CBIT(c), true);
        v->tx.coding = (coding_t) c;
        v->tx.e_pending = true;
    }
    stage_enter(v, STG_RN_WAIT_E, rn_timeout(v));
}

/* The receiver has a rate signal: R1, R2 or R3 depending on who we are and
 * how far along - or R4 or R5 in a renegotiation. */
static void ctl_rate_word(dm_v32_t *v, uint16_t w)
{
    char wt[64];
    char ot[64];

    word_text(w, wt, sizeof(wt));
    switch (v->stage)
    {
    case STG_C_TRAIN1:
    {
        /* R1. Answer with S for NT, the conditioning signal, and R2: what
         * we can do, restricted to what it offered (5.4.1). If either rate
         * signal lacks V.32 bis's mark, the call is V.32. */
        unsigned offer;
        long long nt_syms;

        v->far_bis = v->bis && word_is_bis(w);
        offer = offer_set(v, set_of(w, v->bis), "offering");
        v->sent_rate = make_word(false, offer, v->far_bis);
        word_text(v->sent_rate, ot, sizeof(ot));
        DM_DEBUG("v32", "R1 heard (%s); sending R2 (%s) (tag=%s)", wt, ot, v->tag);
        if (offer == 0)
        {
            cleardown(v, "the answering modem offers no rate we can use");
            return;
        }
        nt_syms = (long long) (((v->nt > 0.0) ? v->nt : 4000.0) / SPS);
        nt_syms = (nt_syms + 1) & ~1LL;
        if (nt_syms < 2)
            nt_syms = 2;
        tx_set(v, SEG_S, nt_syms);
        tx_then(v, SEG_S, 256);
        tx_then(v, SEG_SBAR, 16);
        tx_then(v, SEG_TRN, trn_length(v));
        tx_then(v, SEG_RATE, -1);
        v->tx.word = v->sent_rate;
        v->ec_request = true;
        rx_reset(v);
        v->want_train = false;
        stage_enter(v, STG_C_TRAIN2, (nt_syms + 272 + trn_length(v)) / 2400.0 + 10.0);
        break;
    }

    case STG_C_TRAIN2:
    {
        /* R3: the rate to use. Finish the R2 sequence in hand, send E. */
        int c = best_of(set_of(w, v->far_bis) & (v->far_bis ? SET_BIS : SET_V32));

        if (c < 0)
        {
            cleardown(v, "the answering modem called for a cleardown in R3");
            return;
        }
        DM_DEBUG("v32", "R3 heard (%s); sending E (tag=%s)", wt, v->tag);
        v->tx.eword = make_word(true, CBIT(c), v->far_bis);
        v->tx.e_pending = true;
        v->tx.coding = (coding_t) c;
        stage_enter(v, STG_C_WAIT_E, 6.0);
        break;
    }

    case STG_A_TRAIN:
    {
        /* R2. Second conditioning signal, then R3 naming one rate from it. */
        int c;

        v->far_bis = v->bis && word_is_bis(w);
        c = best_of(offer_set(v, set_of(w, v->bis), "taking"));
        if (c < 0)
        {
            DM_DEBUG("v32", "R2 heard (%s) (tag=%s)", wt, v->tag);
            cleardown(v, "the calling modem offers no rate we can use");
            return;
        }
        v->sent_rate = make_word(false, CBIT(c), v->far_bis);
        word_text(v->sent_rate, ot, sizeof(ot));
        DM_DEBUG("v32", "R2 heard (%s); sending R3 (%s) (tag=%s)", wt, ot, v->tag);
        v->tx.coding = (coding_t) c;
        v->tx.word = v->sent_rate;
        tx_set(v, SEG_S, 256);
        tx_then(v, SEG_SBAR, 16);
        tx_then(v, SEG_TRN, 1280);
        tx_then(v, SEG_RATE, -1);
        stage_enter(v, STG_A_WAIT_E, 10.0);
        break;
    }

    case STG_RN_RESP:
    case STG_RN_SEND:
        rn_rate_word(v, w, wt);
        break;

    default:
        DM_TRACE("v32", "rate signal %s ignored in stage '%s' (tag=%s)", wt, STAGE_NAMES[v->stage],
                 v->tag);
        break;
    }
}

/* E: the far end's last word before scrambled ones at the agreed rate. */
static bool ctl_e_word(dm_v32_t *v, uint16_t w)
{
    int c = best_of(set_of(w, v->far_bis) & (v->far_bis ? SET_BIS : SET_V32));
    char wt[64];

    word_text(w, wt, sizeof(wt));
    if (v->stage != STG_C_WAIT_E && v->stage != STG_A_WAIT_E && v->stage != STG_RN_WAIT_E)
    {
        DM_TRACE("v32", "E (%s) ignored in stage '%s' (tag=%s)", wt, STAGE_NAMES[v->stage], v->tag);
        return false;
    }
    if (c < 0)
        c = CODE_4800;
    if (c != (int) v->tx.coding)
        DM_WARN("v32", "the far end's E (%s) names a different rate from the one agreed; "
                       "receiving at its rate (tag=%s)",
                wt, v->tag);
    DM_DEBUG("v32", "E heard (%s); receiving at %s (tag=%s)", wt, coding_name((coding_t) c), v->tag);
    rx_enter_data(v, (coding_t) c);
    set_coding(v, (coding_t) c);
    if (v->stage == STG_RN_WAIT_E)
    {
        /* 8.1 and 8.2/V.32 bis: 104 stays clamped for 24 symbols more. */
        stage_enter(v, STG_RN_B1, rn_timeout(v));
    }
    else if (v->stage == STG_A_WAIT_E)
    {
        /* 5.4.2: finish R3's sequence, E, 128 symbols of scrambled ones. */
        v->tx.eword = make_word(true, CBIT(v->tx.coding), v->far_bis);
        v->tx.e_pending = true;
        stage_enter(v, STG_A_B1, 4.0);
    }
    else
    {
        stage_enter(v, STG_C_B1, 4.0);
    }
    return true;
}

static void ctl_rx_trained(dm_v32_t *v)
{
    DM_DEBUG("v32", "receiver trained: SNR %.1f dB (tag=%s)", v->rx.snr_trained, v->tag);
}

static void stage_timeout(dm_v32_t *v)
{
    switch (v->stage)
    {
    case STG_C_AA:
    case STG_A_AC:
        /* Still waiting for the far end to say anything at all. Saying so
         * once is enough; the session's own timeout decides when to stop. */
        if (!v->deadline_warned)
        {
            DM_WARN("v32", "%s for 20 seconds and heard nothing back - is the far end a V.32 "
                           "modem? (tag=%s)",
                    v->calling ? "sending AA" : "sending AC", v->tag);
            v->deadline_warned = true;
            emit(v, DM_V32_TRAINING_FAILED);
        }
        v->deadline = 0;
        break;
    case STG_RN_SEND:
        /* Note 3 to 8/V.32 bis: a far end that never answers may be a V.32
         * modem that uses B4 for something else. Retrain, and do not ask
         * again. */
        v->rn_refused = true;
        emit(v, DM_V32_RETRAINING);
        restart(v, "the far end did not answer a request to change rate", false);
        break;
    case STG_RN_RESP:
    case STG_RN_WAIT_E:
    case STG_RN_B1:
    {
        char why[96];

        /* A far end that starts the procedure and does not finish it is no
         * more to be relied on than one that never starts it. */
        v->rn_refused = true;
        snprintf(why, sizeof(why), "a rate renegotiation stalled at '%s'", STAGE_NAMES[v->stage]);
        emit(v, DM_V32_RETRAINING);
        restart(v, why, false);
        break;
    }
    default:
    {
        char why[96];

        snprintf(why, sizeof(why), "timed out at '%s'", STAGE_NAMES[v->stage]);
        restart(v, why, true);
        break;
    }
    }
}

static void control(dm_v32_t *v, long long n)
{
    const tones_t *t = &v->tn;
    float pt6 = t->p[0] + t->p[2];
    float p18 = t->p[1];
    float ptot = t->ptot;
    bool live = ptot > v->pmin;
    double r;

    /* The 600 and 3000 Hz pair is AC; 1800 alone is AA. S has all three,
     * two thirds of it at 1800. The loose tests are for listening while we
     * transmit, when the far end's tone may share the line with our own
     * echo in the other band; the strict ones tell the far end starting
     * over apart from S. */
    v->ac_loose = (live && pt6 > 0.5f * ptot) ? v->ac_loose + 1 : 0;
    v->aa_loose = (live && p18 > 0.5f * ptot) ? v->aa_loose + 1 : 0;
    v->ac_strict = (live && pt6 > 0.8f * ptot && p18 < 0.08f * ptot) ? v->ac_strict + 1 : 0;
    v->aa_strict = (live && p18 > 0.85f * ptot && pt6 < 0.08f * ptot) ? v->aa_strict + 1 : 0;
    v->s_run = (live && p18 > 0.45f * ptot && p18 < 0.85f * ptot && pt6 > 0.12f * ptot &&
                p18 + pt6 > 0.75f * ptot)
                   ? v->s_run + 1
                   : 0;
    v->tn.present[n & (TONE_HIST - 1)] =
        (uint8_t) ((v->ac_loose > 0 ? PRESENT_AC : 0) | (v->aa_loose > 0 ? PRESENT_AA : 0));

    if (v->stage == STG_DEAD)
        return;

    /* 5.5: the far end starting a retrain, or starting over. A renegotiation
     * preamble's 56 symbols of AC or AA are too short to be taken for one. */
    {
        bool in_data = v->stage == STG_DATA || v->stage == STG_RN_SEND || v->stage == STG_RN_RESP ||
                       v->stage == STG_RN_WAIT_E || v->stage == STG_RN_B1;

        if (v->calling && v->ac_strict >= RUN_128T &&
            (v->stage == STG_C_TRAIN2 || v->stage == STG_C_WAIT_E || v->stage == STG_C_B1 || in_data))
        {
            if (in_data)
                emit(v, DM_V32_RETRAINING);
            restart(v, "the answering modem is sending AC", false);
            return;
        }
        if (!v->calling && v->aa_strict >= RUN_128T &&
            (v->stage == STG_A_R1 || v->stage == STG_A_TRAIN || v->stage == STG_A_WAIT_E ||
             v->stage == STG_A_B1 || in_data))
        {
            if (in_data)
                emit(v, DM_V32_RETRAINING);
            restart(v, "the calling modem is sending AA", false);
            return;
        }
    }

    switch (v->stage)
    {
    case STG_C_LISTEN:
        /* An answering modem allowed to omit ANS sends AC at once. The
         * strict test: 2100 Hz leaks into the 600 and 3000 Hz detectors,
         * and must not be taken for AC. */
        if (v->ac_strict >= RUN_64T)
        {
            DM_DEBUG("v32", "AC heard while still listening; starting (tag=%s)", v->tag);
            dm_v32_start(v);
            /* Already hearing AC, so the reversal may be close behind. */
            v->rev_armed = true;
            v->rev_rearm = n;
        }
        break;

    case STG_C_AA:
        if (v->ac_loose >= S_RUN && !v->rev_armed)
        {
            v->rev_armed = true;
            v->rev_rearm = n;
            DM_DEBUG("v32", "AC heard (tag=%s)", v->tag);
        }
        if (reversal(v, M600 | M3000, n, &r))
        {
            long long k = boundary_symbol(r + 64.0 * SPS);

            if (k < v->tx.nsym)
            {
                DM_DEBUG("v32", "turning round %.1f symbols late (tag=%s)",
                         (boundary_time(v->tx.nsym) - r) / SPS - 64.0, v->tag);
                k = v->tx.nsym;
            }
            tx_schedule(v, k, SEG_CC, -1);
            v->t_mark = boundary_time(k);
            stage_enter(v, STG_C_CC, 6.0);
            DM_DEBUG("v32", "AC reversed; answering with CC (tag=%s)", v->tag);
        }
        break;

    case STG_C_CC:
        if (reversal(v, M600 | M3000, n, &r) || v->s_run >= S_RUN)
        {
            if (v->s_run >= S_RUN)
            {
                r = (double) n - 100.0;
                DM_DEBUG("v32", "S arrived without the second reversal (tag=%s)", v->tag);
            }
            v->nt = r - v->t_mark;
            v->rtd = v->nt - 64.0 * SPS;
            if (v->rtd < 0.0)
                v->rtd = 0.0;
            tx_set(v, SEG_SILENCE, -1);
            v->want_train = true;
            v->train_gate = n;
            stage_enter(v, STG_C_TRAIN1, 8.0 + v->rtd / 8000.0);
            DM_DEBUG("v32", "round trip %.1f ms; listening for S (tag=%s)", v->rtd / 8.0, v->tag);
        }
        break;

    case STG_C_TRAIN2:
        /* Not until our own S has gone and the canceller knows our echo,
         * or we would train on ourselves. */
        if (!v->want_train && v->tx.seg == SEG_RATE &&
            (v->ec.state == EC_DONE || v->ec.state == EC_IDLE))
        {
            v->want_train = true;
            v->train_gate = n;
        }
        if (v->rx.mode == RX_ACQ)
            ec_slow(v);
        break;

    case STG_C_B1:
        if (v->rx.data_rcvd >= 128)
            go_data(v);
        break;

    case STG_A_AC:
        if (v->tx.seg == SEG_AC && v->tx.count >= 128 && v->aa_loose >= RUN_64T)
        {
            long long k = v->tx.nsym + (v->tx.count & 1);

            tx_schedule(v, k, SEG_CA, -1);
            v->ca_start = k;
            v->t_mark = boundary_time(k);
            v->rev_armed = true;
            v->rev_rearm = n;
            stage_enter(v, STG_A_CA, 5.0);
            DM_DEBUG("v32", "AA heard; sending CA (tag=%s)", v->tag);
        }
        break;

    case STG_A_CA:
        if (reversal(v, M1800, n, &r))
        {
            double target = r + 64.0 * SPS;
            long long k = boundary_symbol(target);

            v->nt = r - v->t_mark;
            v->rtd = v->nt - 64.0 * SPS;
            if (v->rtd < 0.0)
                v->rtd = 0.0;
            /* Back to AC after an A, which in CA means an even boundary. */
            if ((k - v->ca_start) & 1)
                k += (boundary_time(k + 1) - target < target - boundary_time(k - 1)) ? 1 : -1;
            while (k < v->tx.nsym || ((k - v->ca_start) & 1))
                k++;
            tx_schedule(v, k, SEG_AC, -1);
            v->p18_ref = v->p18_avg = 0.0f;
            v->drop_run = 0;
            stage_enter(v, STG_A_AC2, 5.0 + v->rtd / 8000.0);
            DM_DEBUG("v32", "CC heard; round trip %.1f ms (tag=%s)", v->rtd / 8.0, v->tag);
        }
        break;

    case STG_A_AC2:
        v->p18_avg += 0.1f * (p18 - v->p18_avg);
        if (v->p18_avg > v->p18_ref)
            v->p18_ref = v->p18_avg;
        v->drop_run = (v->p18_ref > 4.0f * v->pmin && v->p18_avg < 0.1f * v->p18_ref) ? v->drop_run + 1 : 0;
        if (v->drop_run >= 40)
        {
            /* 5.4.2: 16 symbols of silence, then S, S-bar, TRN and R1. */
            long long trn = trn_length(v);

            v->sent_rate = make_word(false, v->enabled, v->bis);
            v->tx.word = v->sent_rate;
            tx_set(v, SEG_SILENCE, 16);
            tx_then(v, SEG_S, 256);
            tx_then(v, SEG_SBAR, 16);
            tx_then(v, SEG_TRN, trn);
            tx_then(v, SEG_RATE, -1);
            v->ec_request = true;
            stage_enter(v, STG_A_R1, (288 + trn) / 2400.0 + 12.0);
            DM_DEBUG("v32", "the caller has gone quiet; sending S, S-bar, %lld symbols of TRN, "
                            "then R1 (tag=%s)",
                     trn, v->tag);
        }
        break;

    case STG_A_R1:
        if (v->tx.seg == SEG_RATE && (v->ec.state == EC_DONE || v->ec.state == EC_IDLE) &&
            v->s_run >= S_RUN)
        {
            /* 5.4.2: cease transmitting, and wait out MT before training,
             * so that our own R1 has left the line. */
            double mt = (v->nt > 0.0) ? v->nt : 4000.0;

            tx_set(v, SEG_SILENCE, -1);
            ec_slow(v);
            v->want_train = true;
            v->train_gate = n + (long long) mt;
            stage_enter(v, STG_A_TRAIN, mt / 8000.0 + 8.0);
            DM_DEBUG("v32", "S heard; silent for MT, %.1f ms, then training (tag=%s)", mt / 8.0, v->tag);
        }
        break;

    case STG_A_B1:
        if (v->tx.seg == SEG_DATA && v->tx.count >= 128 && v->rx.data_rcvd >= 128)
            go_data(v);
        break;

    case STG_RN_B1:
        /* 8.1 and 8.2/V.32 bis: 24 symbols of scrambled ones, then data -
         * each direction on its own clock. A step up that got nowhere -
         * the far end's receiver would not have it - waits twice as long
         * before the next. */
        if (!v->tx.data_on && v->tx.seg == SEG_DATA && v->tx.count >= 24)
            v->tx.data_on = true;
        if (!v->rx.data_on && v->rx.mode == RX_DATA && v->rx.data_rcvd >= 24)
            v->rx.data_on = true;
        if (v->tx.data_on && v->rx.data_on)
        {
            stage_enter(v, STG_DATA, 0.0);
            v->low_since = v->poor_since = 0;
            v->renegotiations++;
            v->up_since = 0;
            if (v->rn_from >= 0 && (int) v->coding <= v->rn_from && v->up_backoff > 0.0)
                v->up_backoff = (v->up_backoff * 2.0 > 600.0) ? 600.0 : v->up_backoff * 2.0;
            else
                v->up_backoff = 30.0;
            v->up_next = n + SECONDS(v->up_backoff);
            v->rn_from = -1;
            DM_INFO("v32", "changed rate to %s without retraining: SNR %.1f dB (tag=%s)",
                    coding_name(v->rx.coding), rx_snr(v), v->tag);
            emit(v, DM_V32_RATE_CHANGED);
        }
        break;

    case STG_DATA:
    {
        float snr = rx_snr(v);
        /* Where each coding stops being worth having (CODING): measured
         * back to back over G.711, the 9600 trellis code is at about 1e-2
         * bit errors by 15 dB and useless by 13; the 16-point code needs 3
         * dB more; 4800 is still clean at 14. Between two V.32 bis modems
         * the cure is to step down to a rate the line will bear (8/V.32
         * bis); otherwise a retrain picks the rate again. */
        float poor = CODING[v->coding].poor_db;

        if (v->pwr < v->ploss)
        {
            if (v->low_since == 0)
                v->low_since = n;
            else if (n - v->low_since > SECONDS(0.4))
            {
                emit(v, DM_V32_CARRIER_DOWN);
                restart(v, "the far end's signal has gone", false);
                return;
            }
        }
        else
        {
            v->low_since = 0;
        }
        if (snr < poor)
        {
            if (v->poor_since == 0)
                v->poor_since = n;
            else if (n - v->poor_since > SECONDS(2.0))
            {
                char why[80];
                int want = rn_desired(v);

                snprintf(why, sizeof(why), "reception has been poor (%.1f dB SNR) for 2 seconds", snr);
                /* Stepping down only helps where the line is the trouble -
                 * not where the signal has all but gone, or the far end
                 * ignored the last request. */
                if (!v->rn_refused && want < (int) v->coding && snr >= CODING[CODE_4800].poor_db + 2.0f &&
                    rn_start(v, want, why))
                    break;
                emit(v, DM_V32_RETRAINING);
                restart(v, why, false);
                return;
            }
        }
        else
        {
            v->poor_since = 0;
        }

        /* And the other way: reception good enough for a higher rate, with
         * a dB to spare, for ten seconds - after a step down that turned out
         * more cautious than it needed to be, or a bad patch that passed. */
        if (v->bis && v->far_bis && !v->rn_refused)
        {
            int up = rn_desired(v);

            if (up > (int) v->coding && snr >= CODING[up].offer_db + 1.0f)
            {
                if (v->up_since == 0)
                    v->up_since = n;
                else if (n - v->up_since > SECONDS(10.0) && n >= v->up_next)
                {
                    char why[80];

                    snprintf(why, sizeof(why), "reception is good enough for %d (%.1f dB SNR)", CODING[up].rate,
                             snr);
                    if (v->up_backoff <= 0.0)
                        v->up_backoff = 30.0;
                    if (rn_start(v, up, why))
                        break;
                }
            }
            else
            {
                v->up_since = 0;
            }
        }
        break;
    }

    default:
        break;
    }

    if (v->want_train && v->rx.mode == RX_TONES && n >= v->train_gate && v->s_run >= S_RUN)
        rx_start_acq(v);

    if (v->deadline > 0 && n >= v->deadline)
        stage_timeout(v);
}

/* ---------------------------------------------------------------- driving */

/* A missing sample is silence after the canceller, not silence before it:
 * the far end said nothing we heard, and neither did our echo. */
static void rx_sample(dm_v32_t *v, float x, bool missing)
{
    long long n = v->n;
    float e = missing ? 0.0f : ec_run(v, x, n);
    int ph = (int) ((9 * n) % 40);
    float s = 2.0f * e * v->rx_scale;

    tones_update(v, e, n);
    v->pwr += 0.005f * (e * e - v->pwr);
    v->rx.z[n & ZMASK] = s * v->cos40[ph] - I * (s * v->sin40[ph]);
    v->n++;
    control(v, n);
    demod(v, n);
}

void dm_v32_rx(dm_v32_t *v, const int16_t *amp, int len)
{
    /* Nothing to measure time against until the transmitter has started. */
    if (v->tx.nsamp == 0)
        return;
    for (int i = 0; i < len; i++)
        rx_sample(v, (float) amp[i], false);
}

void dm_v32_rx_fillin(dm_v32_t *v, int len)
{
    if (v->tx.nsamp == 0 || len <= 0)
        return;
    v->freeze_until = v->n + len + 2 * MF_HALF + EQ_LEN * 2;
    for (int i = 0; i < len; i++)
        rx_sample(v, 0.0f, true);
}

dm_v32_t *dm_v32_create(const dm_v32_params_t *p)
{
    dm_v32_t *v = calloc(1, sizeof(*v));
    double rms;

    if (v == NULL)
        return NULL;
    v->ec.corr = calloc(EC_CORR_MAX, sizeof(double));
    if (v->ec.corr == NULL)
    {
        free(v);
        return NULL;
    }
    v->calling = p->calling;
    v->bis = p->v32bis;
    v->rn_deaf = p->deaf_to_renegotiation;
    v->trellis = p->trellis || p->v32bis;
    /* 4800 always. V.32 bis keeps the nonredundant 9600 for a V.32 far end
     * that has no trellis coder (1e/V.32 bis: "compatibility with V.32"). */
    v->enabled = CBIT(CODE_4800);
    if (p->max_rate >= 9600 || p->max_rate == 0)
    {
        v->enabled |= CBIT(CODE_9600U);
        if (v->trellis)
            v->enabled |= CBIT(CODE_9600T);
    }
    if (v->bis)
    {
        int max = (p->max_rate == 0) ? 14400 : p->max_rate;

        if (max >= 7200)
            v->enabled |= CBIT(CODE_7200T);
        if (max >= 12000)
            v->enabled |= CBIT(CODE_12000T);
        if (max >= 14400)
            v->enabled |= CBIT(CODE_14400T);
    }
    snprintf(v->tag, sizeof(v->tag), "%s", p->tag ? p->tag : (p->calling ? "out" : "in"));
    v->get_bit = p->get_bit;
    v->put_bit = p->put_bit;
    v->event = p->event;
    v->user = p->user;
    /* 4.1.1: the caller scrambles with GPC, the answerer with GPA. */
    v->scr_tap = p->calling ? 18 : 5;
    v->dscr_tap = p->calling ? 5 : 18;

    build_tables(v);
    rms = DBM0_RMS * pow(10.0, p->tx_power / 20.0);
    /* Every V.32 constellation has a mean energy of 10, which comes out of
     * the carrier at half that. */
    v->tx_gain = (float) (rms / sqrt(5.0));
    v->rx_scale = 1.0f / v->tx_gain;
    v->p0 = (float) (DBM0_RMS * DBM0_RMS);
    v->pmin = v->p0 * powf(10.0f, -48.0f / 10.0f);
    v->ploss = v->p0 * powf(10.0f, -45.0f / 10.0f);

    v->tx.switch_at = -1;
    v->rtd = -1.0;
    v->nt = -1.0;
    v->rx.snr_trained = 99.0f;
    rx_reset(v);

    if (v->calling && p->listen_first)
    {
        tx_set(v, SEG_SILENCE, -1);
        stage_enter(v, STG_C_LISTEN, 0.0);
    }
    else if (v->calling)
    {
        /* 5.4.1: the answer tone has been heard; AA. */
        tx_set(v, SEG_AA, -1);
        stage_enter(v, STG_C_AA, 20.0);
    }
    else
    {
        /* 5.4.2: the 75 ms of silence that ends V.25's answer sequence,
         * then AC. */
        tx_set(v, SEG_SILENCE, 180);
        tx_then(v, SEG_AC, -1);
        stage_enter(v, STG_A_AC, 20.0);
    }
    return v;
}

void dm_v32_free(dm_v32_t *v)
{
    if (v == NULL)
        return;
    free(v->ec.corr);
    free(v);
}

void dm_v32_start(dm_v32_t *v)
{
    if (v->stage != STG_C_LISTEN)
        return;
    tx_set(v, SEG_AA, -1);
    stage_enter(v, STG_C_AA, 20.0);
}

bool dm_v32_started(const dm_v32_t *v)
{
    return v->stage != STG_C_LISTEN;
}

bool dm_v32_engaged(const dm_v32_t *v)
{
    return v->trained_once ||
           (v->stage != STG_C_LISTEN && v->stage != STG_C_AA && v->stage != STG_A_AC && v->stage != STG_DEAD);
}

int dm_v32_bit_rate(const dm_v32_t *v)
{
    return v->rate;
}

bool dm_v32_renegotiate(dm_v32_t *v, int rate)
{
    int want = -1;

    for (int c = 0; c < CODE_COUNT; c++)
        if ((SET_BIS & CBIT(c)) && CODING[c].rate <= rate)
            want = c;
    if (want < 0)
        return false;
    return rn_start(v, want, "asked to change rate");
}

float dm_v32_rx_power(const dm_v32_t *v)
{
    return 10.0f * log10f(v->pwr / v->p0 + 1e-12f);
}

void dm_v32_stats(const dm_v32_t *v, dm_v32_stats_t *out)
{
    memset(out, 0, sizeof(*out));
    out->stage = STAGE_NAMES[v->stage];
    out->bit_rate = v->rate;
    out->trellis = v->tcm;
    out->snr_db = rx_snr(v);
    out->rx_power = dm_v32_rx_power(v);
    out->round_trip_ms = (v->rtd >= 0.0) ? (int) lrint(v->rtd / 8.0) : -1;
    out->echo_canceller = v->ec.enabled;
    out->echo_delay_ms = v->ec.enabled ? (float) (v->ec.delay + EC_PRE) / 8.0f : 0.0f;
    out->echo_return_loss_db = v->ec.erl_db;
    out->echo_cancelled_db = v->ec.erle_db;
    out->retrains = v->retrains;
    out->v32bis = v->bis && v->far_bis;
    out->renegotiations = v->renegotiations;
}
