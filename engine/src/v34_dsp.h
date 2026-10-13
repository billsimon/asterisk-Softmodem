/* Signal processing blocks for V.34 that are not tied to any one stage of
 * the start-up: the QAM modulator and its pre-emphasis filters (5.4), the
 * Phase 2 transmitter - 600 bit/s DPSK, tones A and B and the line probing
 * signals L1 and L2 (10.1.2) - and the Phase 2 receiver, including the
 * analysis of a received L2 that picks the far end's symbol rate.
 *
 * Everything runs at 8000 samples a second. Line time is a sample count, as
 * in v32.c: transmit sample k and receive sample k are taken to be the same
 * instant at the line terminals. */
#ifndef DATAMODEM_V34_DSP_H
#define DATAMODEM_V34_DSP_H

#include <stdbool.h>
#include <stdint.h>

#include "v34_codec.h"

#define V34_FS 8000

/* 0 dBm0 is a sine 3.14 dB below digital full scale, as in v32.c. */
#define V34_DBM0_RMS (32767.0 * 0.69654 / 1.41421356)

double v34_rrc(double t, double alpha);

/* --------------------------------------------------------- QAM modulator */

#define V34_QTX_L 10                  /* half-span of the pulse, symbols */
#define V34_QTX_TAPS (2 * V34_QTX_L + 1)
#define V34_PE_TAPS 41

typedef v34_cf_t (*v34_symbol_fn)(void *user);

typedef struct
{
    int sr;
    bool high;
    int num, den;                     /* symbols per sample, num/den */
    float (*g)[V34_QTX_TAPS];         /* [den][taps] */
    int cnum, cden;                   /* carrier cycles per sample */
    float *cosv, *sinv;
    long long n;                      /* samples since the modulator started */
    long long k;                      /* symbols taken */
    v34_cf_t hist[32];
    float gain;                       /* applied to symbols of unit mean power */
    float pe[V34_PE_TAPS];
    float pe_hist[V34_PE_TAPS];
    int pe_pos;
    bool pe_on;
    v34_symbol_fn next;
    void *user;
} v34_qtx_t;

bool v34_qtx_init(v34_qtx_t *t, int sr, bool high, int pre_emphasis, double power_dbm0, v34_symbol_fn next,
                  void *user);
void v34_qtx_free(v34_qtx_t *t);
float v34_qtx_sample(v34_qtx_t *t);

/* The magnitude, in dB, that pre-emphasis filter idx asks for at f Hz for
 * symbol rate sr (Figures 1 and 2, Tables 3 and 4). Only the shape matters:
 * the modulator normalises the power over the band. */
double v34_pre_emphasis_db(int idx, int sr, double f);

/* ------------------------------------------------------- Phase 2 signals */

/* 10.1.2.3.1: binary DPSK at 600 bit/s, a 1 a phase reversal; the answering
 * modem on 2400 Hz at 1 dB below nominal with a 1800 Hz guard tone 7 dB
 * below it, the calling modem on 1200 Hz at nominal. Tones A and B are the
 * same carriers unmodulated, and a reversal of one is scheduled to the
 * sample, because 11.2.1.1.3 wants it 40 +/- 1 ms after the far end's. */
#define V34_P2_QUEUE 512

typedef struct
{
    bool answerer;
    float amp;                        /* carrier amplitude, in samples */
    float guard_amp;
    int cden;                         /* carrier period in samples, 1200 Hz: 20, 2400 Hz: 10 */
    float cosv[40];
    long long n;
    long long k;                      /* symbols */
    float hist[16];                   /* DPSK symbols, +/-1 */
    float phase;                      /* the current symbol */
    float g[40][16];
    uint8_t q[V34_P2_QUEUE];          /* bits waiting to go */
    int qh, qt;
    long long flip_at;                /* line time of a scheduled tone reversal, -1 if none */
    float sign;
    bool on;                          /* carrier on (otherwise silence) */
    long long probe_n;                /* L1/L2 */
    float probe_amp;
    bool probe;
} v34_p2tx_t;

void v34_p2tx_init(v34_p2tx_t *t, bool answerer, double power_dbm0);
/* Queue bits for DPSK; until they are sent the carrier is a tone. */
void v34_p2tx_bits(v34_p2tx_t *t, const uint8_t *bits, int n);
int v34_p2tx_pending(const v34_p2tx_t *t);
void v34_p2tx_carrier(v34_p2tx_t *t, bool on);
/* A tone reversal at line time `at`, which must be in the future. */
void v34_p2tx_reverse_at(v34_p2tx_t *t, long long at);
/* L1 (+6 dB) or L2 (nominal), or neither. */
void v34_p2tx_probe(v34_p2tx_t *t, bool on, bool l1, double power_dbm0);
float v34_p2tx_sample(v34_p2tx_t *t, long long line_time);

/* The receiving half: the far end's 2400 or 1200 Hz carrier brought down to
 * baseband, then a DPSK demodulator, and narrow detectors for its tone and
 * for reversals of it. */
#define V34_P2_BOX 160                /* 20 ms: nulls at multiples of 50 Hz */
#define V34_P2_REV 80                 /* 10 ms, for the reversals */
#define V34_P2_HIST 512
#define V34_P2_PHASES 4

typedef struct
{
    int cden;
    float cosv[40], sinv[40];
    float mf[V34_P2_HIST];            /* matched filter, centred */
    int mf_half;
    v34_cf_t mix[V34_P2_HIST];        /* mixed down, before filtering */
    v34_cf_t zf[V34_P2_HIST];         /* matched filter output */
    v34_cf_t box[V34_P2_HIST];        /* 20 ms boxcar */
    v34_cf_t rev[V34_P2_HIST];        /* 10 ms boxcar */
    v34_cf_t sbox, srev;
    float pbox[V34_P2_HIST];
    double spow;
    long long n;
    /* DPSK: four slicers a quarter symbol apart. An INFO sequence is too
     * short for a timing loop to settle in, and one of the four is always
     * within an eighth of a symbol of the eye's centre. */
    double tau[V34_P2_PHASES];
    v34_cf_t zprev[V34_P2_PHASES];
    /* outputs */
    float tone;                       /* 0..1: how much of the band is a steady tone */
    float level;                      /* in-band power, samples^2 */
    long long rev_rearm;
    long long tone_since;             /* line time the tone has been there since, -1 if not */
    long long tone_last;              /* the last line time it was there */
    long long prev_since, prev_until; /* the run of tone before this one */
    /* bits for the caller to collect, one stream per slicer */
    uint8_t bits[V34_P2_PHASES][64];
    int nbits[V34_P2_PHASES];
} v34_p2rx_t;

void v34_p2rx_init(v34_p2rx_t *r, bool far_is_answerer);
/* One sample at line time n. Returns true if a tone reversal has just been
 * recognised; *when is the instant it happened at the line terminals. */
bool v34_p2rx_sample(v34_p2rx_t *r, float x, long long n, double *when);

/* ----------------------------------------------------------- line probing */

/* L1 and L2: 150 Hz to 3750 Hz in 150 Hz steps, less 900, 1200, 1800 and
 * 2400 (Table 17). Exactly periodic in 160 samples. */
#define V34_PROBE_TONES 21
extern const int V34_PROBE_HZ[V34_PROBE_TONES];

/* Collects 20 ms blocks of a received L2 and works out what it says about
 * the line. */
#define V34_PROBE_BLOCKS 32

typedef struct
{
    int blocks;
    v34_cf_t x[V34_PROBE_BLOCKS][80]; /* DFT bins at 50 Hz spacing */
    float buf[V34_P2_BOX];
    int pos;
    /* results */
    float gain_db[V34_PROBE_TONES];   /* received level of each tone, relative to sent */
    float snr_db[V34_PROBE_TONES];    /* per-tone SNR as a QAM signal of the same power would see it */
    bool valid;
} v34_probe_t;

void v34_probe_init(v34_probe_t *p);
/* Feed samples; returns true once enough blocks are in. */
bool v34_probe_sample(v34_probe_t *p, float x);
/* Work out per-tone gain and noise. power_dbm0 is what the far end sends
 * L2 at - its nominal power, which is also what it will train at. */
void v34_probe_analyse(v34_probe_t *p);

/* What a linear equaliser would make of the line at symbol rate sr, carrier
 * high or low and pre-emphasis idx: the SNR it would see, in dB. */
float v34_probe_snr(const v34_probe_t *p, int sr, bool high, int pre_emphasis);

/* ---------------------------------------------------------- echo canceller
 *
 * As in v32.c: the echo that matters is our own signal reflected at the far
 * end's hybrid, a whole round trip later, so a short adaptive filter is
 * placed around wherever cross-correlation finds it. */
#define V34_EC_TAPS 128
#define V34_EC_PRE 24
#define V34_TXH_LEN 32768
#define V34_TXH_MASK (V34_TXH_LEN - 1)
#define V34_EC_CORR_MAX 16000
#define V34_EC_CORR_WIN 2048
#define V34_EC_CONVERGE 2400

typedef enum
{
    V34_EC_IDLE = 0,
    V34_EC_WAIT,
    V34_EC_CORR,
    V34_EC_DONE
} v34_ec_state_t;

typedef struct
{
    v34_ec_state_t state;
    bool enabled;
    int delay;
    float w[V34_EC_TAPS];
    float mu;
    long long fast_from;
    long long slow_from;
    long long corr_start;
    int corr_lmax;
    double *corr;
    double corr_erx;
    float in_avg, res_avg;
    float erl_db, erle_db;
    float txh[V34_TXH_LEN];
    uint8_t txflag[V34_TXH_LEN];
    float p0;
    long long freeze_until;
    const char *tag;
} v34_ec_t;

bool v34_ec_init(v34_ec_t *ec, const char *tag);
void v34_ec_free(v34_ec_t *ec);
/* What we sent at line time n, and whether it is wideband and scrambled
 * enough to learn the echo from. */
void v34_ec_tx(v34_ec_t *ec, long long n, float x, bool learn);
/* Look for our echo: our own training started going out at line time t0 and
 * the round trip is rtd samples. */
void v34_ec_schedule(v34_ec_t *ec, long long t0, double rtd);
/* The far end has started talking: learn slowly from now on. */
void v34_ec_slow(v34_ec_t *ec, long long n);
/* The residual at line time n. far_power is the recent received power,
 * which is how the canceller notices the far end arriving. */
float v34_ec_run(v34_ec_t *ec, float x, long long n, float far_power, float pmin);

#endif /* DATAMODEM_V34_DSP_H */
