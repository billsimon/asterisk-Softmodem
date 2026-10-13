/* ITU-T V.34 (02/98), duplex: the start-up of clause 11 and the modem around
 * the data-mode coding in v34_codec.c. Section, table and figure numbers are
 * V.34's unless they say otherwise.
 *
 * The transmit side is one of four things at any moment: V.8's signals
 * (spandsp's answer tone and V.21 FSK), the Phase 2 transmitter of
 * v34_dsp.c (600 bit/s DPSK, tones A and B, the L1 and L2 probes), the QAM
 * modulator at the symbol rate Phase 2 chose, or silence. The QAM modulator
 * takes its symbols from a little program of segments - S, S-bar, PP, TRN,
 * J, MP, E, and data - in the same shape as v32.c's.
 *
 * The receive side, after the echo canceller, is the matching V.8 or Phase
 * 2 receiver, or a QAM receiver: down to baseband, a matched filter resampled
 * at T/2, a fractionally spaced equaliser and a phase-locked loop, and then
 * whatever the stage needs - looking for S and S-bar, recognising J, J',
 * MP and E in the descrambled bit stream, or the data-mode decoder.
 *
 * TIME. As in v32.c, line time is a sample count: transmit sample k and
 * receive sample k are the same instant at the line terminals, which is
 * what the 40 ms turn-rounds of Phase 2 and the round trip measurement rely
 * on. */
#include "datamodem/v34.h"
#include "datamodem/log.h"

#include "v34_codec.h"
#include "v34_dsp.h"
#include "v34_info.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <spandsp.h>

#define PI 3.14159265358979323846
#define MS(x) ((long long) ((x) * 8))

/* ANSam is 2100 Hz amplitude modulated by a 15 Hz sine, 20% deep (V.8
 * 7.1). A second of its envelope at 100 samples a second is fifteen whole
 * cycles, so the 15 Hz bin stands alone. */
#define ENV_BLOCK 80
#define ENV_BLOCKS 100

/* ----------------------------------------------------------------- stages */

typedef enum
{
    ST_V8_C_LISTEN = 0,   /* calling: waiting for ANSam */
    ST_V8_C_TE,           /* ANSam heard; silent for Te */
    ST_V8_C_CM,           /* CM out, waiting for two identical JMs */
    ST_V8_C_CJ,           /* CJ out, then 75 ms of silence */
    ST_V8_A_ANSAM,        /* answering: ANSam out, waiting for two identical CMs */
    ST_V8_A_JM,           /* JM out, waiting for CJ */
    ST_V8_DONE,           /* 75 ms of silence before Phase 2 */

    ST_C2_INFO0,          /* INFO0c and tone B out; waiting for INFO0a and tone A's reversal */
    ST_C2_REV2,           /* B reversed; waiting for A's second reversal */
    ST_C2_PROBE,          /* receiving the answerer's L1 and L2 */
    ST_C2_B,              /* tone B; waiting for A and its reversal */
    ST_C2_L,              /* L1 and L2 out; waiting for tone A */
    ST_C2_INFO1,          /* INFO1c out; waiting for INFO1a */

    ST_A2_INFO0,          /* INFO0a and tone A out; waiting for INFO0c and tone B */
    ST_A2_REV1,           /* A reversed; waiting for B's reversal */
    ST_A2_L,              /* L1 and L2 out; waiting for tone B */
    ST_A2_A50,            /* tone A and a reversal; waiting for B's reversal */
    ST_A2_PROBE,          /* receiving the caller's L1 and L2 */
    ST_A2_INFO1,          /* tone A; waiting for INFO1c, then INFO1a out */

    ST_P3_TX_FIRST,       /* answerer: S, S-bar, PP, TRN, J out; waiting for the caller's S-bar */
    ST_P3_RX,             /* training on the far end's PP and TRN; waiting for its J */
    ST_P3_TX_SECOND,      /* caller: S, S-bar, PP, TRN, J out; waiting for the answerer's S */
    ST_P4_TRN,            /* TRN out; waiting for our receiver to train on the far end's */
    ST_P4_MP,             /* MP and MP' exchange */
    ST_P4_B1,             /* E and B1 out; waiting for the far end's B1 */
    ST_DATA,
    ST_RN,                /* rate renegotiation: S, S-bar, TRN, then MP as Phase 4 */
    ST_RETRAIN,           /* 70 ms of silence, then into Phase 2 as 11.5 */
    ST_DEAD
} stage_t;

static const char *const STAGE_NAMES[] = {
    "V.8: listening for ANSam",
    "V.8: waiting out Te",
    "V.8: CM, waiting for JM",
    "V.8: CJ",
    "V.8: ANSam, waiting for CM",
    "V.8: JM, waiting for CJ",
    "V.8 done",
    "INFO0c, waiting for INFO0a",
    "measuring the round trip",
    "probing: receiving L1/L2",
    "tone B, waiting for tone A",
    "probing: sending L1/L2",
    "INFO1c, waiting for INFO1a",
    "INFO0a, waiting for INFO0c",
    "measuring the round trip",
    "probing: sending L1/L2",
    "tone A, waiting for tone B",
    "probing: receiving L1/L2",
    "waiting for INFO1c",
    "training: S, PP, TRN, J",
    "training on the far end",
    "training: S, PP, TRN, J",
    "final training: TRN",
    "exchanging MP",
    "B1",
    "data",
    "changing rate",
    "retraining",
    "cleared down",
};

/* ---------------------------------------------------------- transmitter */

typedef enum
{
    TXM_SILENCE = 0,
    TXM_V8,
    TXM_P2,
    TXM_QAM
} txm_t;

typedef enum
{
    TS_SILENCE = 0,
    TS_S,
    TS_SBAR,
    TS_PP,
    TS_TRN,
    TS_J,
    TS_JP,
    TS_MP,
    TS_E,
    TS_DATA
} tseg_t;

typedef struct
{
    tseg_t seg;
    long long len;
} tstep_t;

/* ------------------------------------------------------------- receiver */

#define MF_PHASES 64
#define MF_MAX 64
#define ZBUF 512
#define ZMASK (ZBUF - 1)
#define NEQ 48
#define EQ_D 16              /* the main tap */
#define HBUF 4096            /* T/2 samples held while looking for S and PP */
#define HMASK (HBUF - 1)

typedef enum
{
    RQ_OFF = 0,
    RQ_HUNT,                 /* untrained: looking for S, then S-bar */
    RQ_MD,                   /* not listening yet, or the far end's MD */
    RQ_ALIGN,                /* gathering PP to find exactly where it starts */
    RQ_TRAIN,                /* data-aided, PP then TRN */
    RQ_DD,                   /* trained: 4-point decisions, J, J', MP, E, S */
    RQ_B1,                   /* B1: one data frame, known in advance */
    RQ_DATA
} rq_t;

typedef struct
{
    int sr;
    bool high;
    double sps, th;
    int cnum, cden;
    float *cosv, *sinv;
    int mf_half;
    float hr[MF_PHASES + 1][2 * MF_MAX + 1];
    v34_cf_t z[ZBUF];
    double tau;

    /* T/2 samples */
    v34_cf_t h[HBUF];
    long long nh;            /* T/2 samples produced */
    long long eq_next;       /* the next one the equaliser takes */
    long long sym0;          /* T/2 index whose sample is symbol 0's main tap */

    /* equaliser */
    v34_cf_t line[2 * NEQ];
    int lpos;
    v34_cf_t c[NEQ];
    double complex *P;       /* RLS inverse correlation, NEQ x NEQ */
    double lambda;
    float beta;
    float theta, nu, a1, a2;
    float mse;
    float mse_fast;          /* the same, quicker, to start mse from once training is done */
    float centroid0;
    double tfreq;            /* timing loop's integrator: samples per symbol the far end's clock is off by */
    long long k;             /* equaliser outputs since symbol 0 */
    rq_t mode;

    /* hunting */
    v34_cf_t hq;             /* running mean of r(i) conj r(i - 4) */
    float hp;
    int s_run;
    bool s_seen;
    int neg_run;
    float prod3[3], p3[3];
    long long s_bar_at;      /* T/2 index of the S-to-S-bar transition */
    long long md_until;      /* line time */
    bool far_md;

    /* training reference */
    uint32_t trn_scr;

    /* decisions */
    int q_prev;
    int alt_run;             /* S: alternating label differences */
    int last_diff;
    bool s_dd_seen;
    long long s_dd_at;
    v34_cf_t u_prev;
    uint32_t scr_a, scr_b;
    int ones_a;
    uint32_t sr16;           /* the last 32 bits of stream B */
    int ones_b;
    uint8_t mpbuf[200];
    int mpn;
    bool in_mp;
    float pwr;               /* T/2 sample power, smoothed */
    float pwr_ref;           /* the far end's level when S arrived */
    int hold;                /* T/2 samples before adapting again, after the far end was quiet */
    float pwr_q;             /* T/2 sample power over the last few, to see a silence start at once */
} qrx_t;

struct dm_v34
{
    bool calling;
    char tag[48];
    int max_rate;
    unsigned sr_allow;
    int force_carrier;
    int force_pe;
    int req_trellis;          /* 0, 1, 2: 16, 32, 64 states */
    bool req_shaping;
    bool lapm;
    float nominal_dbm0;
    int (*get_bit)(void *user);
    void (*put_bit)(void *user, int bit);
    void (*event)(void *user, dm_v34_event_t ev);
    void *user;
    int scr_tap, dscr_tap;
    float p0, pmin;

    stage_t stage;
    long long deadline;
    long long t_stage;
    long long n;              /* receive line time */
    long long ntx;            /* transmit line time */

    /* transmit */
    txm_t txm;
    fsk_tx_state_t *fsk_tx;
    modem_connect_tones_tx_state_t *ansam_tx;
    long long ansam_done_at;
    uint8_t v8q[1024];
    int v8h, v8t;
    uint8_t v8msg[256];       /* one CM or JM, as bits */
    int v8msg_len;
    bool v8_repeat;
    long long v8_done_at;     /* transmit time CJ's last stop bit was out, -1 until then */
    bool v8_quiet;            /* stop V.8 at the next transmit sample */
    long long p2_due;         /* transmit time Phase 2 starts, -1 if not due */
    v34_p2tx_t p2tx;
    long long p2_off_at;      /* carrier off at this line time, -1 if not */
    long long p2_probe_at;    /* L1 starts at this line time */
    long long p2_l2_at;       /* and L2 */
    bool p2_off_after_bits;
    long long p2_tone_from;   /* when the carrier last became a plain tone */
    v34_qtx_t qtx;
    tseg_t seg;
    long long seg_count, seg_len;
    tstep_t prog[10];
    int nprog, iprog;
    uint32_t tscr;
    int tz;
    bool tx16;                /* TRN, MP and E on the 16-point set */
    bool p4_16;               /* what the far end's J asked of our Phase 4 */
    uint16_t jpat;
    bool j_stop;
    long long j_start;        /* line time our J began */
    uint8_t mpbits[V34_MP1_BITS];
    int mp_len, mp_pos;
    bool mp_ack_pending, mp_e_pending;
    unsigned mp_sent, mpp_sent;
    int e_pos;
    v34_enc_t enc;
    v34_cf_t dbuf[8];
    int dpos;
    float data_scale;
    bool tx_data;             /* user data may go */
    long long trn_started;    /* line time our TRN started */
    long long trn_symbols;    /* how many we have sent */

    /* receive */
    v34_ec_t ec;
    fsk_rx_state_t *fsk_rx;
    modem_connect_tones_rx_state_t *ansam_rx;
    bool ansam_claimed;       /* spandsp says ANSam; ansam_verify() has the last word */
    /* The answer tone's envelope, every 10 ms, for ansam_verify(). */
    float env_re, env_im;
    float env_pwr;            /* the whole block's power, to tell a tone from noise */
    double env_phase;
    int env_n;
    int tone_run;             /* blocks in a row that were the 2100 Hz tone */
    int tone_gap;             /* allowance for the block a phase reversal empties */
    float env[ENV_BLOCKS];
    int env_count;
    uint32_t v8_sr;
    int v8_bitcnt;
    bool v8_synced;
    uint8_t v8_rx[64];
    int v8_rxn;
    uint8_t v8_last[64];
    int v8_lastn;
    int v8_zero_octets;
    bool v8_got_msg;
    bool v8_got_cj;
    v8_msg_t v8_far;
    v34_p2rx_t p2rx;
    v34_p2rx_t tonerx;        /* the far end's Phase 2 tone, listened for from Phase 3 on */
    uint8_t ibits[V34_P2_PHASES][128];
    int ibits_n[V34_P2_PHASES];
    uint32_t isr[V34_P2_PHASES];
    bool i_collect[V34_P2_PHASES];
    int i_expect;
    v34_probe_t probe;
    bool probing;
    long long probe_from;
    qrx_t q;
    bool far_trn_on;          /* the far end's Phase 4 TRN is arriving */
    long long far_trn;        /* symbols of it */
    double trn_err;           /* squared error over a block of it, for the rate */
    int trn_err_n;
    v34_dec_t dec;
    v34_enc_t b1ref;
    v34_cf_t b1u[256];
    v34_cf_t b1r[256];
    int b1n;
    v34_cf_t rx_gain;         /* equaliser output to Figure 5 units */
    float rx_energy;
    float pwr;                /* received power after the canceller */
    float pwr_fast;           /* the same, quicker, for noticing the far end has gone */

    /* what has been learnt */
    v34_info0_t info0_far;
    bool have_info0;
    unsigned info0_count;
    bool ack_info0;           /* the far end has acknowledged ours */
    v34_info1c_t info1c;      /* ours, if calling; theirs, if answering */
    v34_info1a_t info1a;
    v34_mp_t mp_far;
    bool have_mp, far_ack, far_e;
    long long rtd;            /* samples; -1 until measured */
    long long rev_sent;       /* line time of our reversal */
    double rev_cand;          /* a reversal not yet known to be followed by L1, -1 if none */
    int sr_tx, sr_rx;
    bool high_tx, high_rx;
    int pe_tx;                /* the filter the far end chose for us */
    float pr_db;              /* power reduction the far end asked for */
    int md_far;               /* 35 ms units */
    int rate_tx, rate_rx;
    int want_rx;              /* what we will ask for */
    v34_data_params_t dp_tx, dp_rx;
    bool tx_in_data, rx_in_data;
    bool trained_once;
    bool renegotiating;
    bool rn_initiator;
    int rn_want;
    unsigned retrains, renegotiations;
    float snr_db;
    long long poor_since, low_since;
    long long freeze_until;
    unsigned failures;
};

/* ------------------------------------------------------------- helpers */

static void emit(dm_v34_t *v, dm_v34_event_t ev)
{
    if (v->event != NULL)
        v->event(v->user, ev);
}

static const char *const STAGE_NAMES[];

static void stage_enter(dm_v34_t *v, stage_t st, double timeout_s)
{
    DM_TRACE("v34", "%.3f s: '%s' (tag=%s)", v->n / 8000.0, STAGE_NAMES[st], v->tag);
    v->stage = st;
    v->t_stage = v->n;
    v->deadline = (timeout_s > 0.0) ? v->n + (long long) (timeout_s * 8000.0) : 0;
}

static double rtd_s(const dm_v34_t *v)
{
    return (v->rtd >= 0) ? v->rtd / 8000.0 : 0.5;
}

static float snr_from_mse(float mse)
{
    return 10.0f * log10f(1.0f / (mse + 1e-9f));
}

/* The SNR a data rate needs at a symbol rate: what the 16-state code and a
 * linear equaliser need for an error rate around 1e-6, with a little in
 * hand. */
static float rate_needs_db(int rate, int sr)
{
    double b = rate / v34_symbol_rate(sr);

    return (float) (10.0 * log10(pow(2.0, b) - 1.0) + 7.0);
}

static int best_rate_for(float snr, int sr, int cap)
{
    int best = 0;

    for (int r = (sr == V34_S2400) ? 2400 : 4800; r <= V34_SR_MAX_RATE[sr] && r <= cap; r += 2400)
        if (snr >= rate_needs_db(r, sr))
            best = r;
    return best;
}

/* ------------------------------------------------------------------ V.8 */

static int v8_get_bit(void *user)
{
    dm_v34_t *v = user;
    int b;

    if (v->v8h == v->v8t)
    {
        if (!v->v8_repeat || v->v8msg_len <= 0)
        {
            /* Asked for the bit after the last one queued: CJ's final stop
             * bit has just gone out whole. */
            if (v->v8_done_at < 0)
                v->v8_done_at = v->ntx;
            return 1;
        }
        for (int i = 0; i < v->v8msg_len; i++)
        {
            v->v8q[v->v8t] = v->v8msg[i];
            v->v8t = (v->v8t + 1) % (int) sizeof(v->v8q);
        }
    }
    b = v->v8q[v->v8h];
    v->v8h = (v->v8h + 1) % (int) sizeof(v->v8q);
    return b;
}

static void v8_octet(uint8_t *dst, int *n, int o)
{
    dst[(*n)++] = 0;
    for (int i = 0; i < 8; i++)
        dst[(*n)++] = (uint8_t) ((o >> i) & 1);
    dst[(*n)++] = 1;
}

/* A CM or JM: ten ones, the CM/JM sync octet 0xE0 - 0000001111 with its
 * start and stop bits - then the octets, each sent least significant bit
 * first between a start bit and a stop bit. */
static void v8_build_msg(dm_v34_t *v, const v8_msg_t *m)
{
    uint8_t oct[16];
    int no = v8_build(m, oct, (int) sizeof(oct));
    int n = 0;

    for (int i = 0; i < 10; i++)
        v->v8msg[n++] = 1;
    v8_octet(v->v8msg, &n, 0xE0);
    for (int i = 0; i < no; i++)
        v8_octet(v->v8msg, &n, oct[i]);
    v->v8msg_len = n;
}

static void v8_start_fsk(dm_v34_t *v, bool repeat)
{
    v->v8h = v->v8t = 0;
    v->v8_repeat = repeat;
    v->v8_done_at = -1;
    if (v->fsk_tx == NULL)
        v->fsk_tx = fsk_tx_init(NULL, &preset_fsk_specs[v->calling ? FSK_V21CH1 : FSK_V21CH2], v8_get_bit, v);
    else
        fsk_tx_restart(v->fsk_tx, &preset_fsk_specs[v->calling ? FSK_V21CH1 : FSK_V21CH2]);
    if (v->fsk_tx != NULL)
        fsk_tx_power(v->fsk_tx, v->nominal_dbm0);
    v->txm = TXM_V8;
}

/* What the far end said in V.8, bit by bit: ten ones and the sync octet
 * start a message, then octets follow between start and stop bits. Two
 * identical messages in a row are what counts (V.8 7.4), and three zero
 * octets in a row are CJ. */
static void v8_put_bit(void *user, int bit)
{
    dm_v34_t *v = user;

    if (bit < 0)
        return;
    v->v8_sr = ((v->v8_sr << 1) | (uint32_t) (bit & 1)) & 0xFFFFFu;
    /* 1111111111 0 00000 111 1, in time order */
    if (v->v8_sr == 0xFFC0Fu)
    {
        if (v->v8_synced && v->v8_rxn > 0)
        {
            if (v->v8_rxn == v->v8_lastn && memcmp(v->v8_rx, v->v8_last, (size_t) v->v8_rxn) == 0 &&
                !v->v8_got_msg)
            {
                v8_msg_t m;

                if (v8_parse(v->v8_rx, v->v8_rxn, &m))
                {
                    v->v8_far = m;
                    v->v8_got_msg = true;
                }
            }
            memcpy(v->v8_last, v->v8_rx, (size_t) v->v8_rxn);
            v->v8_lastn = v->v8_rxn;
        }
        v->v8_synced = true;
        v->v8_rxn = 0;
        v->v8_bitcnt = 0;
        v->v8_zero_octets = 0;
        return;
    }
    if (!v->v8_synced)
        return;
    if (++v->v8_bitcnt < 10)
        return;
    if (((v->v8_sr >> 9) & 1) == 0 && (v->v8_sr & 1) == 1)
    {
        uint8_t o = 0;

        for (int i = 0; i < 8; i++)
            o |= (uint8_t) (((v->v8_sr >> (8 - i)) & 1) << i);
        v->v8_bitcnt = 0;
        if (o == 0)
        {
            /* CJ is three zero octets. No CM or JM octet is zero, so two
             * are already unmistakable, and the third is the one most at
             * risk - the caller falls silent straight after it. */
            if (++v->v8_zero_octets >= 2)
                v->v8_got_cj = true;
        }
        else
        {
            v->v8_zero_octets = 0;
            if (v->v8_rxn < (int) sizeof(v->v8_rx))
                v->v8_rx[v->v8_rxn++] = o;
        }
    }
    else if (v->v8_bitcnt > 20)
    {
        v->v8_synced = false;
    }
}

static void v8_start_rx(dm_v34_t *v)
{
    if (v->fsk_rx != NULL)
        fsk_rx_free(v->fsk_rx);
    v->fsk_rx = fsk_rx_init(NULL, &preset_fsk_specs[v->calling ? FSK_V21CH2 : FSK_V21CH1], FSK_FRAME_MODE_ASYNC,
                            v8_put_bit, v);
    if (v->fsk_rx != NULL)
        fsk_rx_signal_cutoff(v->fsk_rx, -45.5f);
    v->v8_synced = false;
    v->v8_lastn = 0;
    v->v8_got_msg = false;
    v->v8_got_cj = false;
}

static void v8_describe(const v8_msg_t *m, char *out, size_t len)
{
    snprintf(out, len, "%s%s%s%s%s%s", m->v34 ? " V.34" : "", m->v34hdx ? " V.34hdx" : "", m->v32 ? " V.32bis" : "",
             m->v22 ? " V.22bis" : "", m->v21 ? " V.21" : "", m->lapm ? ", LAPM" : "");
}

/* --------------------------------------------------------- Phase 2: TX */

static void p2_carrier(dm_v34_t *v, bool on)
{
    v34_p2tx_carrier(&v->p2tx, on);
    v->txm = TXM_P2;
    v->p2_off_at = -1;
    v->p2_probe_at = -1;
    v->p2_l2_at = -1;
    v->p2_off_after_bits = false;
    if (on)
        v->p2_tone_from = v->ntx;
}

static void p2_send_info(dm_v34_t *v, const uint8_t *bits, int n)
{
    if (!v->p2tx.on || v->txm != TXM_P2)
        p2_carrier(v, true);
    v34_p2tx_bits(&v->p2tx, bits, n);
}

static void send_info0(dm_v34_t *v)
{
    v34_info0_t i = { 0 };
    uint8_t bits[V34_INFO0_BITS];
    int n;

    i.sr2743 = (v->sr_allow >> V34_S2743) & 1;
    i.sr2800 = (v->sr_allow >> V34_S2800) & 1;
    i.sr3429 = (v->sr_allow >> V34_S3429) & 1;
    i.low3000 = i.high3000 = (v->sr_allow >> V34_S3000) & 1;
    i.low3200 = i.high3200 = (v->sr_allow >> V34_S3200) & 1;
    i.allow3429 = i.sr3429;
    i.power_reduction = true;
    i.asym_steps = 5;
    i.c1664 = true;
    i.ack = v->have_info0;
    n = v34_info0_pack(&i, bits);
    p2_send_info(v, bits, n);
}

/* At line time t the tone reverses; tail_ms after that the carrier stops,
 * and if probe, L1 and then L2 follow. */
static void p2_reverse(dm_v34_t *v, long long t, int tail_ms, bool probe)
{
    if (t < v->ntx)
        t = v->ntx;
    v34_p2tx_reverse_at(&v->p2tx, t);
    v->rev_sent = t;
    v->p2_off_at = (tail_ms > 0) ? t + MS(tail_ms) : -1;
    v->p2_probe_at = probe ? v->p2_off_at : -1;
    v->p2_l2_at = probe ? v->p2_off_at + MS(160) : -1;
}

static void info_bits_reset(dm_v34_t *v, int expect)
{
    for (int k = 0; k < V34_P2_PHASES; k++)
    {
        v->isr[k] = 0;
        v->i_collect[k] = false;
        v->ibits_n[k] = 0;
    }
    v->i_expect = expect;
}

static void p2_start_rx(dm_v34_t *v)
{
    v34_p2rx_init(&v->p2rx, v->calling);
    info_bits_reset(v, V34_INFO0_BITS);
}

/* ------------------------------------------------------- QAM transmitter */

static v34_cf_t pt4(int rot)
{
    static const v34_cf_t p[4] = { 0.70710678f + 0.70710678f * I, 0.70710678f - 0.70710678f * I,
                                   -0.70710678f - 0.70710678f * I, -0.70710678f + 0.70710678f * I };

    return p[rot & 3];
}

/* The 16-point set: points 0 to 3 of Figure 5 turned clockwise, at unit
 * mean power. */
static v34_cf_t pt16(int idx, int rot)
{
    static const int8_t base[4][2] = { { 1, 1 }, { -3, 1 }, { 1, -3 }, { -3, -3 } };
    int x = base[idx & 3][0], y = base[idx & 3][1];
    int ox, oy;

    switch (rot & 3)
    {
    case 0:
        ox = x;
        oy = y;
        break;
    case 1:
        ox = y;
        oy = -x;
        break;
    case 2:
        ox = -x;
        oy = -y;
        break;
    default:
        ox = -y;
        oy = x;
        break;
    }
    return ((float) ox + I * (float) oy) * 0.31622777f;
}

static long long tx_symbols_ahead(const dm_v34_t *v)
{
    return (long long) (V34_QTX_L * 8000.0 / v34_symbol_rate(v->sr_tx));
}

static void tx_enter(dm_v34_t *v, tseg_t seg, long long len)
{
    v->seg = seg;
    v->seg_len = len;
    v->seg_count = 0;
    switch (seg)
    {
    case TS_TRN:
        /* 10.1.3.8: the scrambler starts from zero. */
        v->tscr = 0;
        v->trn_started = v->ntx + tx_symbols_ahead(v);
        v->trn_symbols = 0;
        break;
    case TS_J:
        v->j_start = v->ntx + tx_symbols_ahead(v);
        break;
    case TS_MP:
        v->mp_pos = 0;
        break;
    case TS_E:
        v->e_pos = 0;
        break;
    case TS_DATA:
    {
        double ex, exn;

        /* 10.1.3.1: B1 starts the scrambler, the trellis and differential
         * encoders and the precoder afresh. The Note to 10.1.3: data goes
         * out at the power training did. */
        v34_enc_init(&v->enc, &v->dp_tx, v->scr_tap);
        v34_data_energy(&v->dp_tx, &ex, &exn);
        v34_enc_set_energy(&v->enc, ex);
        v->data_scale = (float) (1.0 / sqrt(exn));
        v->dpos = 8;
        v->tx_data = false;
        break;
    }
    default:
        break;
    }
}

static void tx_set(dm_v34_t *v, tseg_t seg, long long len)
{
    v->nprog = v->iprog = 0;
    v->j_stop = false;
    tx_enter(v, seg, len);
}

static void tx_then(dm_v34_t *v, tseg_t seg, long long len)
{
    if (v->nprog < (int) (sizeof(v->prog) / sizeof(v->prog[0])))
    {
        v->prog[v->nprog].seg = seg;
        v->prog[v->nprog].len = len;
        v->nprog++;
    }
}

static int tx_bit(dm_v34_t *v, int bit)
{
    return v34_scramble(&v->tscr, v->scr_tap, bit);
}

/* Two bits, differentially encoded onto the 4-point set (10.1.3.3), or four
 * onto the 16-point one (10.1.3.9). */
static v34_cf_t tx_diff(dm_v34_t *v, const int *b, bool sixteen)
{
    int i1 = tx_bit(v, b[0]), i2 = tx_bit(v, b[1]);

    v->tz = (v->tz + i1 + 2 * i2) & 3;
    if (!sixteen)
        return pt4(v->tz);
    {
        int q1 = tx_bit(v, b[2]), q2 = tx_bit(v, b[3]);

        return pt16(q1 + 2 * q2, v->tz);
    }
}

static void build_mp(dm_v34_t *v, bool ack);

static int get_data_bit(void *user)
{
    dm_v34_t *v = user;

    if (!v->tx_data || v->get_bit == NULL)
        return 1;
    return v->get_bit(v->user) & 1;
}

static v34_cf_t tx_symbol(void *user)
{
    dm_v34_t *v = user;
    v34_cf_t s = 0.0f;

    while (v->seg_len >= 0 && v->seg_count >= v->seg_len)
    {
        if (v->iprog < v->nprog)
        {
            tstep_t st = v->prog[v->iprog++];

            tx_enter(v, st.seg, st.len);
        }
        else
        {
            tx_enter(v, TS_SILENCE, -1);
        }
    }
    switch (v->seg)
    {
    case TS_SILENCE:
        break;
    /* 10.1.3.7: S alternates point 0 and point 0 turned a quarter
     * anticlockwise, ending on the latter; S-bar is S turned half way round,
     * starting on point 0's opposite. */
    case TS_S:
        s = pt4((v->seg_count & 1) ? 3 : 0);
        break;
    case TS_SBAR:
        s = pt4((v->seg_count & 1) ? 1 : 2);
        break;
    case TS_PP:
    {
        /* (10-1) */
        int i = (int) (v->seg_count % 48);
        int k = i / 4, l = i % 4;
        double a = (k % 3 == 1) ? PI * (k * l + 4) / 6.0 : PI * k * l / 6.0;

        s = (float) cos(a) + I * (float) sin(a);
        break;
    }
    case TS_TRN:
    {
        int i1 = tx_bit(v, 1), i2 = tx_bit(v, 1);
        int rot = i1 + 2 * i2;

        if (v->tx16)
        {
            int q1 = tx_bit(v, 1), q2 = tx_bit(v, 1);

            s = pt16(q1 + 2 * q2, rot);
        }
        else
        {
            s = pt4(rot);
        }
        /* J and MP start their differential encoders from the last of it. */
        v->tz = rot;
        v->trn_symbols++;
        break;
    }
    case TS_J:
    case TS_JP:
    {
        uint16_t pat = (v->seg == TS_JP) ? V34_JPRIME : v->jpat;
        int pos = (int) (v->seg_count % 8);
        int b[2] = { (pat >> (15 - 2 * pos)) & 1, (pat >> (14 - 2 * pos)) & 1 };

        s = tx_diff(v, b, false);
        if (pos == 7 && (v->seg == TS_JP || v->j_stop))
            v->seg_len = v->seg_count + 1;
        break;
    }
    case TS_MP:
    {
        int b[4] = { 0 };
        int per = v->tx16 ? 4 : 2;

        for (int k = 0; k < per; k++)
            b[k] = (v->mp_pos < v->mp_len) ? v->mpbits[v->mp_pos++] : 0;
        s = tx_diff(v, b, v->tx16);
        if (v->mp_pos >= v->mp_len)
        {
            /* A whole sequence is out. 11.4.1: MP' once the far end's MP is
             * in, and E once both MP' are. */
            v->mp_pos = 0;
            if (v->mpbits[33])
                v->mpp_sent++;
            else
                v->mp_sent++;
            if (v->mp_e_pending && v->mpbits[33])
            {
                v->mp_e_pending = false;
                v->seg_len = v->seg_count + 1;
                v->nprog = v->iprog = 0;
                tx_then(v, TS_E, -1);
                tx_then(v, TS_DATA, -1);
            }
            else if (v->mp_ack_pending)
            {
                v->mp_ack_pending = false;
                build_mp(v, true);
            }
        }
        break;
    }
    case TS_E:
    {
        int b[4] = { 1, 1, 1, 1 };

        s = tx_diff(v, b, v->tx16);
        v->e_pos += v->tx16 ? 4 : 2;
        if (v->e_pos >= 20)
            v->seg_len = v->seg_count + 1;
        break;
    }
    case TS_DATA:
        if (v->dpos >= 8)
        {
            bool b1 = v->enc.i < v->enc.f.P;

            if (!b1)
                v->tx_data = true;
            v34_enc_frame(&v->enc, b1 ? NULL : get_data_bit, v, NULL, NULL, v->dbuf);
            v->dpos = 0;
        }
        s = v->dbuf[v->dpos++] * v->data_scale;
        break;
    }
    v->seg_count++;
    return s;
}

static void qtx_start(dm_v34_t *v)
{
    double p = v->nominal_dbm0 - v->pr_db;

    v34_qtx_init(&v->qtx, v->sr_tx, v->high_tx, v->pe_tx, p, tx_symbol, v);
    v->txm = TXM_QAM;
}

/* -------------------------------------------------------- QAM receiver */

static double pulse(double t, double a, double span)
{
    if (fabs(t) >= span)
        return 0.0;
    return v34_rrc(t, a) * 0.5 * (1.0 + cos(PI * t / span));
}

static void qrx_free(qrx_t *q)
{
    free(q->cosv);
    free(q->sinv);
    free(q->P);
    q->cosv = q->sinv = NULL;
    q->P = NULL;
}

static int gcd_i(int a, int b)
{
    while (b)
    {
        int t = a % b;

        a = b;
        b = t;
    }
    return a;
}

static bool qrx_start(dm_v34_t *v, int sr, bool high)
{
    qrx_t *q = &v->q;
    int a = V34_SR_A[sr], c = V34_SR_C[sr];
    int d = V34_CAR_D[sr][high], e = V34_CAR_E[sr][high];
    const double span = 8.0;
    double se = 0.0;
    int g;

    qrx_free(q);
    memset(q, 0, sizeof(*q));
    q->sr = sr;
    q->high = high;
    q->sps = 8000.0 / v34_symbol_rate(sr);
    q->th = q->sps / 2.0;
    q->cnum = 3 * a * d;
    q->cden = 10 * c * e;
    g = gcd_i(q->cnum, q->cden);
    q->cnum /= g;
    q->cden /= g;
    q->cosv = calloc((size_t) q->cden, sizeof(float));
    q->sinv = calloc((size_t) q->cden, sizeof(float));
    q->P = calloc(NEQ * NEQ, sizeof(double complex));
    if (q->cosv == NULL || q->sinv == NULL || q->P == NULL)
        return false;
    for (int i = 0; i < q->cden; i++)
    {
        q->cosv[i] = (float) cos(2.0 * PI * i / q->cden);
        q->sinv[i] = (float) sin(2.0 * PI * i / q->cden);
    }
    q->mf_half = (int) ceil(span * q->sps);
    if (q->mf_half > MF_MAX)
        q->mf_half = MF_MAX;
    /* Matched to a 10% root raised cosine. With unit-power symbols the
     * transmit pulse is sqrt(sps / E) times it, E its energy in samples, so
     * dividing by sqrt(sps E) puts a lone symbol out at its own size. */
    for (int n = -q->mf_half; n <= q->mf_half; n++)
    {
        double p = pulse(n / q->sps, 0.10, span);

        se += p * p;
    }
    for (int p = 0; p <= MF_PHASES; p++)
        for (int j = -q->mf_half; j <= q->mf_half; j++)
            q->hr[p][j + q->mf_half] =
                (float) (pulse(((double) p / MF_PHASES - j) / q->sps, 0.10, span) / sqrt(q->sps * se));
    q->tau = (double) v->n + 2.0 * q->mf_half + 4.0;
    q->mode = RQ_HUNT;
    q->lpos = NEQ;
    return true;
}

static v34_cf_t mf_at(const qrx_t *q, double tau)
{
    long long n0 = (long long) floor(tau);
    int p = (int) lrint((tau - (double) n0) * MF_PHASES);
    const float *h = q->hr[p];
    v34_cf_t y = 0.0f;

    for (int j = -q->mf_half; j <= q->mf_half; j++)
        y += q->z[(n0 + j) & ZMASK] * h[j + q->mf_half];
    return y;
}

static v34_cf_t pp_symbol(int k)
{
    int i = k % 48;
    int kk = i / 4, l = i % 4;
    double a = (kk % 3 == 1) ? PI * (kk * l + 4) / 6.0 : PI * kk * l / 6.0;

    return (float) cos(a) + I * (float) sin(a);
}

/* The clockwise quarter turns from point 0. */
static int quad_of(v34_cf_t u)
{
    if (crealf(u) >= 0.0f)
        return (cimagf(u) >= 0.0f) ? 0 : 1;
    return (cimagf(u) < 0.0f) ? 2 : 3;
}

static void rx_dd_reset(qrx_t *q)
{
    q->alt_run = 0;
    q->last_diff = -1;
    q->s_dd_seen = false;
    q->ones_a = 0;
    q->ones_b = 0;
    q->sr16 = 0;
    q->in_mp = false;
    q->mpn = 0;
}

static void rls_init(qrx_t *q, double delta)
{
    memset(q->P, 0, sizeof(double complex) * NEQ * NEQ);
    for (int i = 0; i < NEQ; i++)
        q->P[i * NEQ + i] = 1.0 / delta;
}

/* Recursive least squares, for the output y = sum c w. In the usual form
 * y = theta^H w, theta = conj(c). */
static void rls_update(qrx_t *q, const v34_cf_t *w, v34_cf_t err)
{
    double complex pu[NEQ];
    double complex g[NEQ];
    double den = q->lambda;

    for (int i = 0; i < NEQ; i++)
    {
        double complex s = 0.0;

        for (int j = 0; j < NEQ; j++)
            s += q->P[i * NEQ + j] * w[j];
        pu[i] = s;
    }
    for (int i = 0; i < NEQ; i++)
        den += creal(conj(w[i]) * pu[i]);
    for (int i = 0; i < NEQ; i++)
        g[i] = pu[i] / den;
    for (int i = 0; i < NEQ; i++)
        q->c[i] += (float complex) (conj(g[i]) * err);
    /* P = (P - g (P w)^H) / lambda, P being Hermitian. */
    for (int i = 0; i < NEQ; i++)
        for (int j = 0; j < NEQ; j++)
            q->P[i * NEQ + j] = (q->P[i * NEQ + j] - g[i] * conj(pu[j])) / q->lambda;
}

static float tap_centroid(const qrx_t *q)
{
    float s = 0.0f, m = 0.0f;

    for (int i = 0; i < NEQ; i++)
    {
        float e = crealf(q->c[i]) * crealf(q->c[i]) + cimagf(q->c[i]) * cimagf(q->c[i]);

        s += e;
        m += e * (float) i;
    }
    return (s > 0.0f) ? m / s : (float) EQ_D;
}

static void rx_symbol(dm_v34_t *v, const v34_cf_t *w, v34_cf_t vout, v34_cf_t u);

/* Equalise one T/2 sample; every second one is a symbol. */
static void eq_half(dm_v34_t *v, v34_cf_t y, long long idx)
{
    qrx_t *q = &v->q;
    const v34_cf_t *w;
    v34_cf_t vout = 0.0f;
    v34_cf_t u;

    q->lpos = (q->lpos == 0) ? NEQ - 1 : q->lpos - 1;
    q->line[q->lpos] = y;
    q->line[q->lpos + NEQ] = y;
    if (idx < q->sym0 || ((idx - q->sym0) & 1))
        return;
    w = &q->line[q->lpos];
    for (int i = 0; i < NEQ; i++)
        vout += q->c[i] * w[i];
    u = vout * (cosf(q->theta) - I * sinf(q->theta));
    rx_symbol(v, w, vout, u);
    q->k++;
    /* What the timing loop has learnt of the far end's clock goes on applying
     * through silences and missing samples, when nothing is learnt: the drift
     * does not stop. The carrier's offset is not carried on the same way:
     * its estimate is noisier, and over a second of silence a small error in
     * it turns the phase round, where leaving the phase alone loses only what
     * a real offset - a fraction of a hertz - adds up to. */
    q->tau += q->tfreq;
}

static void timing_track(qrx_t *q, const v34_cf_t *w, v34_cf_t vout, v34_cf_t err);

/* Phase and equaliser update towards d - what was sent, or a decision. */
static void track(dm_v34_t *v, const v34_cf_t *w, v34_cf_t vout, v34_cf_t u, v34_cf_t d)
{
    qrx_t *q = &v->q;
    float dd = crealf(d) * crealf(d) + cimagf(d) * cimagf(d);
    float pe = (dd > 0.0f) ? cimagf(u * conjf(d)) / dd : 0.0f;
    v34_cf_t err = d * (cosf(q->theta) + I * sinf(q->theta)) - vout;
    v34_cf_t ue = u - d;

    /* Nothing is learnt while samples are missing or the far end is quiet,
     * and nothing measured either; nor is anything learnt while our own
     * Phase 3 signal is coming back at us (see ST_P3_TX_SECOND). */
    if (v->n < v->freeze_until || q->pwr < 0.3f * q->pwr_ref || q->hold > 0)
        return;
    {
        float e2 = (crealf(ue) * crealf(ue) + cimagf(ue) * cimagf(ue)) / (dd > 0 ? dd : 1.0f);

        q->mse += 0.01f * (e2 - q->mse);
        q->mse_fast += 0.05f * (e2 - q->mse_fast);
        /* A decision this far from what was received is a guess - the far
         * end fading out, a burst of noise - and the loops must not steer by
         * it: a handful of them at the start of a silence once turned the
         * clock estimate round, and the silence then carried it half a
         * symbol off. */
        if (q->mode != RQ_TRAIN && e2 > 0.15f)
        {
            float uu = crealf(u) * crealf(u) + cimagf(u) * cimagf(u);

            /* ... except the phase itself, while what arrives is a signal
             * of the right size: a phase that has slipped after a silence
             * makes every decision look like this, and would never be
             * pulled back otherwise. */
            if (uu > 0.5f * dd && uu < 2.0f * dd)
            {
                q->theta += q->a1 * pe;
                if (q->theta > PI)
                    q->theta -= (float) (2.0 * PI);
                else if (q->theta < -PI)
                    q->theta += (float) (2.0 * PI);
            }
            return;
        }
    }
    if (q->mode == RQ_TRAIN)
    {
        rls_update(q, w, err);
        /* The phase loop runs alongside, gently, once the first periods of
         * PP have set the equaliser up: least squares on its own trails a
         * carrier that rotates - a far end some ppm out - by most of its
         * memory, which at 100 ppm was a fifth of a radian. */
        if (q->k < 96)
            return;
    }
    q->theta += q->a1 * pe + q->nu;
    q->nu += q->a2 * pe;
    /* A frequency offset beyond what the far end's carrier may have (2.1 of
     * V.34 allows 0.01%, under 0.4 Hz) is the loop chasing noise. */
    if (q->nu > 0.003f)
        q->nu = 0.003f;
    else if (q->nu < -0.003f)
        q->nu = -0.003f;
    if (q->theta > PI)
        q->theta -= (float) (2.0 * PI);
    else if (q->theta < -PI)
        q->theta += (float) (2.0 * PI);
    /* While our own Phase 3 signal is coming back and the canceller has not
     * yet found it, the phase loop may follow the far end but the equaliser
     * must not learn our echo. */
    if (q->mode == RQ_TRAIN)
    {
        if (q->k >= 288)
            timing_track(q, w, vout, err);
        return;
    }
    timing_track(q, w, vout, err);
    if (v->stage == ST_P3_TX_SECOND && (v->ec.state == V34_EC_WAIT || v->ec.state == V34_EC_CORR))
        return;
    {
        float norm = 0.0f;
        v34_cf_t g;

        for (int i = 0; i < NEQ; i++)
            norm += crealf(w[i]) * crealf(w[i]) + cimagf(w[i]) * cimagf(w[i]);
        g = q->beta * err / (norm + 1e-6f);
        for (int i = 0; i < NEQ; i++)
            q->c[i] += g * conjf(w[i]);
    }
}

static void rx_half(dm_v34_t *v, v34_cf_t y);

static void qrx_sample(dm_v34_t *v, float e, long long n)
{
    qrx_t *q = &v->q;
    int ph = (int) ((n * q->cnum) % q->cden);

    q->z[n & ZMASK] = (2.0f * e / (float) V34_DBM0_RMS) * (q->cosv[ph] - I * q->sinv[ph]);
    while (q->tau + q->mf_half <= (double) n)
    {
        v34_cf_t y = mf_at(q, q->tau);

        q->tau += q->th;
        rx_half(v, y);
        if (q->mode == RQ_OFF)
            return;
    }
}

/* ----------------------------------------------------------- the stages */

static void restart_phase2(dm_v34_t *v, bool initiate, const char *why);
static void cleardown(dm_v34_t *v, const char *why);
static void p3_heard_j(dm_v34_t *v, bool sixteen);
static void p3_heard_sbar(dm_v34_t *v);
static void p4_heard_s(dm_v34_t *v);
static void p4_heard_sbar(dm_v34_t *v);
static void p4_heard_jprime(dm_v34_t *v);
static void heard_mp(dm_v34_t *v, const v34_mp_t *mp);
static void heard_e(dm_v34_t *v);
static void go_data(dm_v34_t *v);
static void rx_data_symbol(dm_v34_t *v, const v34_cf_t *w, v34_cf_t vout, v34_cf_t u);

/* The far end's S (10.1.3.7) arrives with nothing trained: it has period 2T,
 * so each T/2 sample matches the one four before; and S-bar is S turned
 * round, so across the boundary they are opposite. */
static void hunt_half(dm_v34_t *v, long long idx)
{
    qrx_t *q = &v->q;
    v34_cf_t r = q->h[idx & HMASK];
    v34_cf_t r4, prod;
    float p;

    if (idx < 8)
        return;
    r4 = q->h[(idx - 4) & HMASK];
    prod = r * conjf(r4);
    p = 0.5f * (crealf(r) * crealf(r) + cimagf(r) * cimagf(r) + crealf(r4) * crealf(r4) + cimagf(r4) * cimagf(r4));
    q->hq += 0.1f * (prod - q->hq);
    q->hp += 0.1f * (p - q->hp);
    if (q->mode == RQ_MD)
    {
        if (v->n >= q->md_until)
        {
            q->mode = RQ_HUNT;
            q->s_seen = false;
            q->s_run = q->neg_run = 0;
        }
        return;
    }
    if (q->hp > 1e-4f && crealf(q->hq) > 0.75f * q->hp)
    {
        if (++q->s_run >= 40 && !q->s_seen)
        {
            q->s_seen = true;
            q->pwr_ref = q->hp;
            DM_DEBUG("v34", "S heard at T/2 %lld (tag=%s)", idx, v->tag);
        }
    }
    else
    {
        q->s_run = 0;
    }
    if (!q->s_seen)
        return;
    /* Three samples together: at T/2 the ones between symbol centres are
     * smaller, and on their own can miss. */
    q->prod3[idx % 3] = crealf(prod);
    q->p3[idx % 3] = p;
    if (q->prod3[0] + q->prod3[1] + q->prod3[2] < -0.5f * (q->p3[0] + q->p3[1] + q->p3[2]) &&
        p > 0.2f * q->pwr_ref)
    {
        if (++q->neg_run == 1)
        {
            q->s_bar_at = idx - 2;
            if (q->far_md)
            {
                /* 11.3.1.1.1: MD follows; then S and S-bar again. */
                q->far_md = false;
                q->mode = RQ_MD;
                q->md_until = v->n + MS(35 * v->md_far);
                DM_DEBUG("v34", "S-bar heard; waiting out %d ms of MD (tag=%s)", 35 * v->md_far, v->tag);
                return;
            }
            q->mode = RQ_ALIGN;
            p3_heard_sbar(v);
        }
    }
    else
    {
        q->neg_run = 0;
    }
}

/* PP (10.1.3.6) begins 16 symbols after S-bar does. Correlating against its
 * first two periods finds the sample its first symbol lands on. */
static void align_pp(dm_v34_t *v)
{
    qrx_t *q = &v->q;
    long long c0 = q->s_bar_at + 32;
    double best = -1.0;
    long long bc = c0;
    v34_cf_t bC = 0.0f;
    double pw = 0.0;
    double rho;

    for (long long c = c0 - 40; c <= c0 + 40; c++)
    {
        v34_cf_t C = 0.0f;

        for (int k = 0; k < 96; k++)
            C += q->h[(c + 2 * k) & HMASK] * conjf(pp_symbol(k));
        if (cabsf(C) > best)
        {
            best = cabsf(C);
            bc = c;
            bC = C;
        }
    }
    for (int k = 0; k < 96; k++)
    {
        v34_cf_t r = q->h[(bc + 2 * k) & HMASK];

        pw += crealf(r) * crealf(r) + cimagf(r) * cimagf(r);
    }
    rho = best / sqrt(96.0 * pw + 1e-12);
    if (rho < 0.5)
    {
        DM_DEBUG("v34", "PP is not where S-bar put it (correlation %.2f); listening again (tag=%s)", rho, v->tag);
        q->mode = RQ_HUNT;
        q->s_seen = false;
        q->s_run = q->neg_run = 0;
        return;
    }
    /* Symbol k's main-tap sample is bc + 2k; the equaliser's output for it
     * is formed once the line holds EQ_D more samples. */
    q->sym0 = bc + EQ_D;
    q->eq_next = bc + EQ_D - NEQ + 1;
    memset(q->c, 0, sizeof(q->c));
    memset(q->line, 0, sizeof(q->line));
    q->lpos = NEQ;
    q->c[EQ_D] = 96.0f / bC;
    q->k = 0;
    q->lambda = 0.995;
    rls_init(q, 0.02 * pw / 96.0 + 1e-6);
    q->theta = q->nu = 0.0f;
    q->a1 = 0.02f;
    q->a2 = 0.0002f;
    q->mse = q->mse_fast = 1.0f;
    q->mode = RQ_TRAIN;
    q->trn_scr = 0;
    DM_DEBUG("v34", "PP found (correlation %.2f, %+lld T/2 from where S-bar put it); training (tag=%s)", rho,
             bc - c0, v->tag);
}

static void rx_half(dm_v34_t *v, v34_cf_t y)
{
    qrx_t *q = &v->q;
    long long idx = q->nh++;

    q->h[idx & HMASK] = y;
    q->pwr += 0.02f * (crealf(y) * crealf(y) + cimagf(y) * cimagf(y) - q->pwr);
    /* After a silence, or samples that never came, nothing is learnt until
     * the equaliser's delay line holds the far end's signal again: what it
     * decides meanwhile is decided from half a line of nothing. */
    q->pwr_q += 0.3f * (crealf(y) * crealf(y) + cimagf(y) * cimagf(y) - q->pwr_q);
    if (q->pwr < 0.3f * q->pwr_ref || q->pwr_q < 0.05f * q->pwr_ref || v->n < v->freeze_until)
        q->hold = NEQ + 8;
    else if (q->hold > 0)
        q->hold--;
    switch (q->mode)
    {
    case RQ_HUNT:
    case RQ_MD:
        hunt_half(v, idx);
        break;
    case RQ_ALIGN:
        if (idx >= q->s_bar_at + 32 + 40 + 2 * 96 + 2)
        {
            align_pp(v);
            /* Catch up on what is already in hand. */
            while (q->mode == RQ_TRAIN && q->eq_next <= idx)
            {
                eq_half(v, q->h[q->eq_next & HMASK], q->eq_next);
                q->eq_next++;
            }
        }
        break;
    case RQ_OFF:
        break;
    default:
        if (q->eq_next <= idx)
        {
            eq_half(v, y, idx);
            q->eq_next = idx + 1;
        }
        break;
    }
}

/* Two bits from a 4-point decision: stream A as if absolute (TRN), stream B
 * differential (J, MP, E). While A descrambles to ones, TRN is still coming
 * and A's scrambler state is the true one, so B takes it over - and so is
 * right from the first symbol of whatever follows TRN. */
static void dd_bits(dm_v34_t *v, int quad)
{
    qrx_t *q = &v->q;
    int diff = (quad - q->q_prev) & 3;
    int a[2] = { quad & 1, quad >> 1 };
    int b[2] = { diff & 1, diff >> 1 };

    q->q_prev = quad;
    if (q->ones_a >= 24)
        q->scr_b = q->scr_a;
    for (int k = 0; k < 2; k++)
    {
        int da = v34_descramble(&q->scr_a, v->dscr_tap, a[k]);
        int db = v34_descramble(&q->scr_b, v->dscr_tap, b[k]);

        q->ones_a = da ? q->ones_a + 1 : 0;
        q->sr16 = (q->sr16 << 1) | (uint32_t) db;
        q->ones_b = db ? q->ones_b + 1 : 0;

        /* J and J' (Tables 18 and 19) */
        if (v->stage == ST_P3_RX)
        {
            uint16_t lo = (uint16_t) (q->sr16 & 0xFFFF), hi = (uint16_t) (q->sr16 >> 16);

            if (lo == hi && (lo == V34_J4 || lo == V34_J16))
            {
                p3_heard_j(v, lo == V34_J16);
                return;
            }
        }
        else if (v->stage == ST_P4_TRN && !v->calling && !v->far_trn_on && (q->sr16 & 0xFFFF) == V34_JPRIME)
        {
            p4_heard_jprime(v);
        }

        /* MP: seventeen ones and a zero start one. */
        if (v->stage == ST_P4_TRN || v->stage == ST_P4_MP || v->stage == ST_P4_B1 || v->stage == ST_RN)
        {
            if (q->in_mp)
            {
                q->mpbuf[q->mpn++] = (uint8_t) db;
                if ((q->mpn == V34_MP0_BITS && !q->mpbuf[18]) || q->mpn == V34_MP1_BITS)
                {
                    v34_mp_t mp;

                    q->in_mp = false;
                    if (v34_mp_unpack(q->mpbuf, q->mpn, &mp))
                        heard_mp(v, &mp);
                    if (q->mode != RQ_DD)
                        return;
                }
            }
            else if (!db && (q->sr16 & 0x3FFFF) == 0x3FFFE)
            {
                memset(q->mpbuf, 1, 17);
                q->mpbuf[17] = 0;
                q->mpn = 18;
                q->in_mp = true;
            }
            /* E: twenty ones after an MP (10.1.3.2). The symbol after this
             * one is B1's first. */
            if (v->have_mp && q->ones_b == 20 && !q->in_mp)
            {
                heard_e(v);
                return;
            }
        }
    }
}

/* S (and the S-bar after it) in a trained receiver: 4-point decisions that
 * alternate between point 0 and its quarter turn, then turn round. In label
 * terms the differences alternate 3, 1, 3, 1; the turn makes two 3s. */
static bool dd_watch_s(dm_v34_t *v, v34_cf_t u, int quad)
{
    qrx_t *q = &v->q;
    int diff = (quad - q->q_prev) & 3;
    v34_cf_t e = u - pt4(quad);
    v34_cf_t step = u * conjf(q->u_prev);
    float mu = crealf(u) * crealf(u) + cimagf(u) * cimagf(u);
    /* A clean quarter turn from the last symbol, at about the right size:
     * whatever the phase, which may well be off after a silence. */
    bool near = mu > 0.5f && mu < 1.6f && fabsf(crealf(step)) < 0.35f * cabsf(step);
    bool turned = false;

    q->u_prev = u;
    (void) e;
    /* Quarter turns clockwise, as the labels count them. */
    if (near)
        diff = (cimagf(step) < 0.0f) ? 1 : 3;

    /* Our own S coming back is not the far end's (11.3.1.1.7). */
    if (v->stage == ST_P3_TX_SECOND && v->n < v->j_start + (v->rtd > 0 ? v->rtd : 0) + MS(20))
    {
        q->alt_run = 0;
        q->last_diff = -1;
        return false;
    }
    if (near && (diff == 1 || diff == 3) && q->last_diff >= 0 && diff != q->last_diff)
        q->alt_run++;
    else if (near && diff == 3 && q->last_diff == 3 && (q->alt_run >= 20 || q->s_dd_seen))
        turned = true;
    else
        q->alt_run = 0;
    q->last_diff = near ? diff : -1;
    if (q->alt_run == 20 && !q->s_dd_seen)
    {
        q->s_dd_seen = true;
        q->s_dd_at = q->k;
        p4_heard_s(v);
    }
    /* S is 128 symbols: once it has been seen, the turn need not have a
     * clean run right before it - a decision spoiled near the end of S must
     * not hide it - but it must come soon. */
    if (q->s_dd_seen && !turned && q->k - q->s_dd_at > 200)
        q->s_dd_seen = false;
    if (turned)
    {
        q->alt_run = 0;
        if (q->s_dd_seen)
        {
            q->s_dd_seen = false;
            p4_heard_sbar(v);
            return true;
        }
    }
    return false;
}

/* The far end's symbol clock against ours, which 2.1 allows to be 100 ppm
 * out. The equaliser could follow a drifting clock by moving its taps, but
 * only as fast as it learns, and badly: so the sampling instant is steered
 * directly. The error a timing offset makes is the output's slope times the
 * offset, and the slope is the difference between the output and what the
 * same taps make of the line half a symbol older. A second-order loop, so
 * that a steady drift leaves no steady error. */
static void timing_track(qrx_t *q, const v34_cf_t *w, v34_cf_t vout, v34_cf_t err)
{
    v34_cf_t older = 0.0f;
    v34_cf_t slope;
    float sp, e;

    for (int i = 0; i < NEQ - 1; i++)
        older += q->c[i] * w[i + 1];
    slope = vout - older;
    sp = crealf(slope) * crealf(slope) + cimagf(slope) * cimagf(slope);
    if (sp < 1e-6f)
        return;
    /* Positive when a later sample would have been better. */
    e = crealf(conjf(err) * slope) / sp;
    if (e > 0.5f)
        e = 0.5f;
    else if (e < -0.5f)
        e = -0.5f;
    /* Quick through training, to have found a clock offset by the time the
     * rate is chosen; then gentler, because data has more noise in it.
     * Critically damped either way. */
    {
        double kp = (q->mode == RQ_DATA) ? 0.004 : 0.008;

        q->tfreq += kp * kp / 4.0 * e * q->th;
        q->tau += kp * e * q->th;
    }
    if (q->tfreq > 0.0005 * q->sps)
        q->tfreq = 0.0005 * q->sps;
    else if (q->tfreq < -0.0005 * q->sps)
        q->tfreq = -0.0005 * q->sps;
}

static void rx_symbol(dm_v34_t *v, const v34_cf_t *w, v34_cf_t vout, v34_cf_t u)
{
    qrx_t *q = &v->q;

    switch (q->mode)
    {
    case RQ_TRAIN:
    {
        v34_cf_t d;

        if (q->k < 288)
        {
            d = pp_symbol((int) q->k);
        }
        else
        {
            int i1 = v34_scramble(&q->trn_scr, v->dscr_tap, 1);
            int i2 = v34_scramble(&q->trn_scr, v->dscr_tap, 1);

            d = pt4(i1 + 2 * i2);
            if (q->k == 288 + 200)
                q->lambda = 0.998;
        }
        track(v, w, vout, u, d);
        if (q->k >= 288 + 480)
        {
            /* The far end's TRN is at least 512 symbols. Hand over to
             * decisions now, which carry on through the rest of it and J. */
            q->mode = RQ_DD;
            q->beta = 0.02f;
            q->a1 = 0.05f;
            q->a2 = 0.0004f;
            q->q_prev = quad_of(u);
            q->centroid0 = tap_centroid(q);
            rx_dd_reset(q);
            q->scr_a = q->scr_b = q->trn_scr;
            q->ones_a = 24;
            q->mse = q->mse_fast;
            v->snr_db = snr_from_mse(q->mse);
            DM_DEBUG("v34", "receiver trained on PP and TRN: SNR %.1f dB (tag=%s)", v->snr_db, v->tag);
        }
        break;
    }

    case RQ_DD:
    {
        int quad = quad_of(u);

        track(v, w, vout, u, pt4(quad));
        v->snr_db = snr_from_mse(q->mse);
        if (q->pwr < 0.05f * q->pwr_ref)
        {
            /* The far end is silent. */
            q->alt_run = 0;
            q->last_diff = -1;
            q->q_prev = quad;
            break;
        }
        if (v->far_trn_on)
        {
            /* The block the rate is chosen from: clear of the S-bar to TRN
             * turn, and before MP. */
            if (v->far_trn >= 100 && v->far_trn < 600)
            {
                v34_cf_t e = u - pt4(quad);

                v->trn_err += crealf(e) * crealf(e) + cimagf(e) * cimagf(e);
                v->trn_err_n++;
            }
            v->far_trn++;
        }
        if (dd_watch_s(v, u, quad))
        {
            q->q_prev = quad;
            break;
        }
        dd_bits(v, quad);
        break;
    }

    case RQ_B1:
    case RQ_DATA:
        rx_data_symbol(v, w, vout, u);
        break;

    default:
        break;
    }
}

/* ----------------------------------------------------------- data mode */

static void dec_put_bit(void *user, int bit)
{
    dm_v34_t *v = user;

    if (v->rx_in_data && v->put_bit != NULL)
        v->put_bit(v->user, bit);
}

static void rx_data_symbol(dm_v34_t *v, const v34_cf_t *w, v34_cf_t vout, v34_cf_t u)
{
    qrx_t *q = &v->q;

    if (q->mode == RQ_B1)
    {
        /* B1 is one data frame of scrambled ones from a fresh encoder
         * (10.1.3.1), so it is known before it arrives, and gives the scale
         * and phase of the far end's data constellation exactly. */
        int nb = 8 * v->dec.f.P;

        if (v->b1n % 8 == 0)
            v34_enc_frame(&v->b1ref, NULL, NULL, NULL, NULL, &v->b1r[v->b1n]);
        v->b1u[v->b1n++] = u;
        if (v->b1n >= nb)
        {
            v34_cf_t num = 0.0f;
            float den = 0.0f, res = 0.0f, ref = 0.0f;
            v34_cf_t g;

            for (int i = 0; i < nb; i++)
            {
                num += v->b1r[i] * conjf(v->b1u[i]);
                den += crealf(v->b1u[i]) * crealf(v->b1u[i]) + cimagf(v->b1u[i]) * cimagf(v->b1u[i]);
            }
            g = num / (den + 1e-9f);
            for (int i = 0; i < nb; i++)
            {
                v34_cf_t e = g * v->b1u[i] - v->b1r[i];

                res += crealf(e) * crealf(e) + cimagf(e) * cimagf(e);
                ref += crealf(v->b1r[i]) * crealf(v->b1r[i]) + cimagf(v->b1r[i]) * cimagf(v->b1r[i]);
            }
            if (res < 0.1f * ref && fabsf(cabsf(g) / sqrtf(v->rx_energy) - 1.0f) < 0.3f)
            {
                v->rx_gain = g;
                DM_DEBUG("v34", "B1 as expected: gain %.3f of nominal, phase %.1f degrees, SNR %.1f dB (tag=%s)",
                         cabsf(g) / sqrtf(v->rx_energy), carg(g) * 180.0 / PI, 10.0 * log10(ref / (res + 1e-9)),
                         v->tag);
            }
            else
            {
                DM_INFO("v34", "B1 is not what was expected (residual %.1f dB, gain %.2f of nominal); carrying "
                               "on at the nominal scale (tag=%s)",
                        10.0 * log10(res / (ref + 1e-9)), cabsf(g) / sqrtf(v->rx_energy), v->tag);
            }
            for (int i = 0; i < nb; i++)
                v34_dec_symbol(&v->dec, v->b1u[i] * v->rx_gain);
            q->mode = RQ_DATA;
            q->alt_run = 0;
            q->last_diff = -1;
            v->rx_in_data = true;
            if (v->tx_in_data && v->stage != ST_DATA)
                go_data(v);
        }
        return;
    }

    {
        v34_cf_t y = u * v->rx_gain;
        v34_cf_t d = v34_dec_slice(&v->dec, y);
        int quad = quad_of(u);

        track(v, w, vout, u, d / v->rx_gain);
        v34_dec_symbol(&v->dec, y);
        /* Now and then, how reception is going: what to look at first when a
         * real far end misbehaves. */
        if (q->k % (10 * 3429) == 0 && dm_log_enabled(DM_LOG_DEBUG))
            DM_DEBUG("v34", "receiving at %d: SNR %.1f dB, the far end's clock %+.1f ppm and carrier %+.2f Hz from "
                            "ours, %u frames the shell mapper could not have made (tag=%s)",
                     v->rate_rx, 10.0f * log10f(v->rx_energy / (v->dec.err + 1e-9f)), -q->tfreq / q->sps * 1e6,
                     q->nu * v34_symbol_rate(q->sr) / (2.0 * PI), v->dec.bad_frames, v->tag);
        /* Watch for the far end starting a rate renegotiation (11.6), or
         * answering ours. */
        if (v->stage == ST_DATA || v->stage == ST_RN)
            dd_watch_s(v, u, quad);
        q->q_prev = quad;
    }
}

/* ---------------------------------------------------------- Phase 3, 4 */

static long long trn_length(const dm_v34_t *v, bool second)
{
    double S = v34_symbol_rate(v->sr_tx);
    double rtd = (v->rtd >= 0) ? (double) v->rtd : 4000.0;
    /* Long enough for the canceller to find our echo and learn it with the
     * far end quiet, no shorter than 512T, and within the 2 s plus one or
     * two round trips 11.3.1 allows. */
    long long k = (long long) ceil((rtd + 320.0 + V34_EC_CORR_WIN + V34_EC_CONVERGE) / 8000.0 * S);
    long long lim = (long long) ((1.8 + (second ? 2.0 : 1.0) * rtd / 8000.0) * S);

    if (k < 512)
        k = 512;
    if (k > lim)
        k = lim;
    return k;
}

static void phase3_tx(dm_v34_t *v)
{
    long long trn = trn_length(v, v->calling);
    double S = v34_symbol_rate(v->sr_tx);
    long long silence = v->calling ? 0 : (long long) (0.07 * S);

    qtx_start(v);
    /* Phase 3's TRN is always 4 points (11.3.1.1.6); we ask for the
     * 4-point set in Phase 4 too. */
    v->tx16 = false;
    v->jpat = V34_J4;
    tx_set(v, TS_SILENCE, silence);
    tx_then(v, TS_S, 128);
    tx_then(v, TS_SBAR, 16);
    tx_then(v, TS_PP, 288);
    tx_then(v, TS_TRN, trn);
    tx_then(v, TS_J, -1);
    v->ec.state = V34_EC_IDLE;
    v34_ec_schedule(&v->ec, v->ntx + (long long) ((V34_QTX_L + 432 + silence) * 8000.0 / S), (double) v->rtd);
    DM_DEBUG("v34", "Phase 3: S, S-bar, PP, %lld symbols of TRN, then J (tag=%s)", trn, v->tag);
}

/* The S-bar of the far end's Phase 3. The answerer stops J and goes quiet
 * (11.3.1.2.4); either way, PP comes next. */
static void p3_heard_sbar(dm_v34_t *v)
{
    DM_DEBUG("v34", "S-bar heard; training on PP (tag=%s)", v->tag);
    if (v->stage == ST_P3_TX_FIRST)
    {
        tx_set(v, TS_SILENCE, -1);
        v34_ec_slow(&v->ec, v->n);
        stage_enter(v, ST_P3_RX, 2.6 + 2.0 * rtd_s(v) + 2.0);
    }
}

static void p3_heard_j(dm_v34_t *v, bool sixteen)
{
    DM_DEBUG("v34", "J heard: the far end wants %d points in Phase 4 (tag=%s)", sixteen ? 16 : 4, v->tag);
    v->p4_16 = sixteen;
    if (v->calling)
    {
        /* 11.3.1.1.3: now our own S, S-bar, PP, TRN and J. */
        phase3_tx(v);
        stage_enter(v, ST_P3_TX_SECOND, 10.0 + 2.0 * rtd_s(v));
    }
    else
    {
        /* 11.4.1.2.1: S, S-bar, then TRN. */
        v->tx16 = v->p4_16;
        tx_set(v, TS_S, 128);
        tx_then(v, TS_SBAR, 16);
        tx_then(v, TS_TRN, -1);
        v->far_trn_on = false;
        v->far_trn = 0;
        rx_dd_reset(&v->q);
        stage_enter(v, ST_P4_TRN, 2.5 + 3.0 * rtd_s(v) + 2.0);
        DM_DEBUG("v34", "Phase 4: S, S-bar, TRN (tag=%s)", v->tag);
    }
}

static void p4_heard_s(dm_v34_t *v)
{
    if (v->stage == ST_DATA && !v->renegotiating)
    {
        /* 11.6.1.2.1: the far end wants to renegotiate. Clamp, and wait for
         * S-bar. */
        DM_INFO("v34", "the far end is asking to change rate (tag=%s)", v->tag);
        v->renegotiating = true;
        v->rn_initiator = false;
        v->rn_want = 0;
        v->rx_in_data = false;
        v->q.mode = RQ_DD;
        v->q.beta = 0.02f;
        emit(v, DM_V34_RETRAINING);
    }
    else if (v->stage == ST_RN && v->rn_initiator && v->q.mode == RQ_DATA)
    {
        /* 11.6.1.1.2: the far end's answer to ours. */
        v->rx_in_data = false;
        v->q.mode = RQ_DD;
        v->q.beta = 0.02f;
    }
}

static void build_mp(dm_v34_t *v, bool ack)
{
    v34_mp_t mp = { 0 };
    int my_tx_max = (v->max_rate < V34_SR_MAX_RATE[v->sr_tx]) ? v->max_rate : V34_SR_MAX_RATE[v->sr_tx];
    int rx = v->want_rx;

    /* Note to Tables 20 and 21 */
    if (!v->info0_far.c1664)
    {
        if (my_tx_max > 28800)
            my_tx_max = 28800;
        if (rx > 28800)
            rx = 28800;
    }
    mp.type = 1;
    mp.rate_c_to_a = (v->calling ? my_tx_max : rx) / 2400;
    mp.rate_a_to_c = (v->calling ? rx : my_tx_max) / 2400;
    mp.trellis = v->req_trellis;
    mp.nonlinear = false;
    mp.expanded = v->req_shaping;
    mp.ack = ack;
    for (int r = 2400; r <= 33600; r += 2400)
        if (r <= v->max_rate)
            mp.rate_mask |= 1u << (r / 2400 - 1);
    mp.asymmetric = true;
    v->mp_len = v34_mp_pack(&mp, v->mpbits);
}

/* MP from the far end (10.1.3.9). */
static void heard_mp(dm_v34_t *v, const v34_mp_t *mp)
{
    bool pre = mp->type == 1 &&
               (mp->h[0][0] || mp->h[0][1] || mp->h[1][0] || mp->h[1][1] || mp->h[2][0] || mp->h[2][1]);

    if (!v->have_mp || mp->ack != v->mp_far.ack)
        DM_DEBUG("v34", "MP%s heard: call->answer %d, answer->call %d, %d-state trellis%s%s%s%s (tag=%s)",
                 mp->ack ? "'" : "", mp->rate_c_to_a * 2400, mp->rate_a_to_c * 2400, 16 << mp->trellis,
                 mp->expanded ? ", expanded shaping" : "", mp->nonlinear ? ", non-linear encoding" : "",
                 pre ? ", precoding" : "", mp->asymmetric ? "" : ", symmetric", v->tag);
    if (mp->type == 1)
    {
        v->mp_far = *mp;
    }
    else
    {
        int16_t h[3][2];

        /* A type 0 leaves the precoding coefficients as they were - even the
         * first MP of a rate renegotiation, which is usually a type 0: the
         * far end goes on un-precoding with the coefficients it sent in
         * Phase 4, so we must go on precoding with them. Only a retrain
         * zeroes them (restart_phase2). */
        memcpy(h, v->mp_far.h, sizeof(h));
        v->mp_far = *mp;
        memcpy(v->mp_far.h, h, sizeof(h));
    }
    v->have_mp = true;
    if (mp->ack)
        v->far_ack = true;
    if (mp->rate_c_to_a == 0 && mp->rate_a_to_c == 0)
    {
        cleardown(v, "the far end is clearing down (11.7)");
        return;
    }
    if (v->stage == ST_P4_MP && v->mpbits[33] == 0)
        v->mp_ack_pending = true;
}

/* 11.4.1.1.3: the data rates. */
static int pick_rate(unsigned mask, int limit)
{
    int best = 0;

    for (int r = 2400; r <= 33600; r += 2400)
        if ((mask & (1u << (r / 2400 - 1))) && r <= limit)
            best = r;
    return best;
}

static bool settle_rates(dm_v34_t *v)
{
    v34_mp_t mine;
    unsigned mask;
    int ca, ac;

    v34_mp_unpack(v->mpbits, v->mp_len, &mine);
    mask = mine.rate_mask & v->mp_far.rate_mask;
    if (!mine.asymmetric || !v->mp_far.asymmetric)
    {
        int lim = mine.rate_c_to_a;

        if (mine.rate_a_to_c < lim)
            lim = mine.rate_a_to_c;
        if (v->mp_far.rate_c_to_a < lim)
            lim = v->mp_far.rate_c_to_a;
        if (v->mp_far.rate_a_to_c < lim)
            lim = v->mp_far.rate_a_to_c;
        ca = ac = pick_rate(mask, lim * 2400);
    }
    else
    {
        int lca = (mine.rate_c_to_a < v->mp_far.rate_c_to_a) ? mine.rate_c_to_a : v->mp_far.rate_c_to_a;
        int lac = (mine.rate_a_to_c < v->mp_far.rate_a_to_c) ? mine.rate_a_to_c : v->mp_far.rate_a_to_c;

        ca = pick_rate(mask, 2400 * lca);
        ac = pick_rate(mask, 2400 * lac);
    }
    v->rate_tx = v->calling ? ca : ac;
    v->rate_rx = v->calling ? ac : ca;
    /* A rate the symbol rate cannot carry is no rate (Table 8). */
    if (v->rate_tx > V34_SR_MAX_RATE[v->sr_tx])
        v->rate_tx = V34_SR_MAX_RATE[v->sr_tx];
    if (v->rate_rx > V34_SR_MAX_RATE[v->sr_rx])
        v->rate_rx = V34_SR_MAX_RATE[v->sr_rx];
    if (v->rate_tx == 2400 && v->sr_tx != V34_S2400)
        v->rate_tx = 0;
    if (v->rate_rx == 2400 && v->sr_rx != V34_S2400)
        v->rate_rx = 0;
    if (v->rate_tx <= 0 || v->rate_rx <= 0)
        return false;

    memset(&v->dp_tx, 0, sizeof(v->dp_tx));
    v->dp_tx.sr = v->sr_tx;
    v->dp_tx.rate = v->rate_tx;
    v->dp_tx.trellis = (v->mp_far.trellis <= 2) ? 16 << v->mp_far.trellis : 16;
    v->dp_tx.expanded = v->mp_far.expanded;
    v->dp_tx.nonlinear = v->mp_far.nonlinear;
    memcpy(v->dp_tx.h, v->mp_far.h, sizeof(v->dp_tx.h));
    memset(&v->dp_rx, 0, sizeof(v->dp_rx));
    v->dp_rx.sr = v->sr_rx;
    v->dp_rx.rate = v->rate_rx;
    v->dp_rx.trellis = 16 << mine.trellis;
    v->dp_rx.expanded = mine.expanded;
    return true;
}

static void start_mp(dm_v34_t *v)
{
    build_mp(v, v->have_mp);
    v->mp_ack_pending = false;
    v->mp_e_pending = false;
    v->mp_sent = v->mpp_sent = 0;
    v->seg_len = v->seg_count;     /* end TRN with this symbol */
    v->nprog = v->iprog = 0;
    tx_then(v, TS_MP, -1);
    stage_enter(v, ST_P4_MP, 2.5 + 3.0 * rtd_s(v) + 2.0);
}

/* Our receiver has had a good look at the far end's training: what it can
 * take goes in our MP. */
static void choose_rx_rate(dm_v34_t *v)
{
    int cap = (v->max_rate < V34_SR_MAX_RATE[v->sr_rx]) ? v->max_rate : V34_SR_MAX_RATE[v->sr_rx];

    if (v->rn_want > 0 && v->rn_want < cap)
        cap = v->rn_want;
    if (v->trn_err_n > 100)
        v->snr_db = snr_from_mse((float) (v->trn_err / v->trn_err_n));
    v->want_rx = best_rate_for(v->snr_db, v->sr_rx, cap);
    if (v->want_rx == 0)
        v->want_rx = (v->sr_rx == V34_S2400) ? 2400 : 4800;
    DM_DEBUG("v34", "receiver at %.1f dB SNR; asking for %d bit/s (tag=%s)", v->snr_db, v->want_rx, v->tag);
}

static void p4_heard_sbar(dm_v34_t *v)
{
    if (v->stage == ST_P3_TX_SECOND)
    {
        /* 11.4.1.1.1: stop J, send J', then TRN; the answerer's TRN follows
         * its S-bar. */
        DM_DEBUG("v34", "the answerer's S-bar heard; J', then TRN (tag=%s)", v->tag);
        v->j_stop = true;
        v->nprog = v->iprog = 0;
        v->tx16 = v->p4_16;
        tx_then(v, TS_JP, -1);
        tx_then(v, TS_TRN, -1);
        rx_dd_reset(&v->q);
        v->q.ones_a = 0;
        v->far_trn_on = true;
        v->far_trn = 0;
        v->trn_err = 0.0;
        v->trn_err_n = 0;
        stage_enter(v, ST_P4_TRN, 2.5 + 2.0 * rtd_s(v) + 2.0);
    }
    else if (v->renegotiating && (v->stage == ST_DATA || v->stage == ST_RN))
    {
        /* 11.6.1.2.2: the responder's S, S-bar and TRN; TRN, MP and E all on
         * the 4-point set. The far end's TRN is on its way. */
        rx_dd_reset(&v->q);
        v->far_trn_on = true;
        v->far_trn = 0;
        v->trn_err = 0.0;
        v->trn_err_n = 0;
        v->have_mp = v->far_ack = v->far_e = false;
        if (v->stage == ST_DATA)
        {
            v->tx_in_data = false;
            v->tx_data = false;
            v->tx16 = false;
            tx_set(v, TS_S, 128);
            tx_then(v, TS_SBAR, 16);
            tx_then(v, TS_TRN, -1);
            stage_enter(v, ST_RN, 2.5 + 3.0 * rtd_s(v) + 2.0);
        }
    }
}

static void p4_heard_jprime(dm_v34_t *v)
{
    DM_DEBUG("v34", "J' heard; the caller's TRN follows (tag=%s)", v->tag);
    v->far_trn_on = true;
    v->far_trn = 0;
    v->trn_err = 0.0;
    v->trn_err_n = 0;
}

static void heard_e(dm_v34_t *v)
{
    qrx_t *q = &v->q;
    double ex, exn;

    if (v->far_e)
        return;
    v->far_e = true;
    v->far_ack = true;
    DM_DEBUG("v34", "E heard; B1 follows (tag=%s)", v->tag);
    if (!settle_rates(v))
    {
        cleardown(v, "the two modems have no data rate in common");
        return;
    }
    v34_dec_init(&v->dec, &v->dp_rx, v->dscr_tap, dec_put_bit, v);
    v34_enc_init(&v->b1ref, &v->dp_rx, v->dscr_tap);
    v34_data_energy(&v->dp_rx, &ex, &exn);
    v->rx_energy = (float) exn;
    v->rx_gain = sqrtf(v->rx_energy);
    v->b1n = 0;
    q->mode = RQ_B1;
    q->beta = 0.005f;
    q->a1 = 0.03f;
    q->a2 = 0.0005f;
}

static void go_data(dm_v34_t *v)
{
    bool retrain = v->trained_once;

    stage_enter(v, ST_DATA, 0.0);
    v->poor_since = v->low_since = 0;
    v->failures = 0;
    if (v->renegotiating)
    {
        v->renegotiating = false;
        v->renegotiations++;
        DM_INFO("v34", "rates changed without a retrain: %d bit/s out, %d in, SNR %.1f dB (tag=%s)", v->rate_tx,
                v->rate_rx, v->snr_db, v->tag);
        emit(v, DM_V34_RATE_CHANGED);
        return;
    }
    DM_INFO("v34", "trained: %d bit/s out at %s symbols/s on %.0f Hz, %d bit/s in at %s on %.0f Hz; SNR %.1f dB, "
                   "round trip %.0f ms, echo canceller %s (tag=%s)",
            v->rate_tx, V34_SR_NAME[v->sr_tx], v34_carrier(v->sr_tx, v->high_tx), v->rate_rx, V34_SR_NAME[v->sr_rx],
            v34_carrier(v->sr_rx, v->high_rx), v->snr_db, v->rtd >= 0 ? v->rtd / 8.0 : -1.0,
            v->ec.enabled ? "on" : "off - no echo", v->tag);
    if (retrain)
        v->retrains++;
    v->trained_once = true;
    emit(v, DM_V34_TRAINED);
}

/* ---------------------------------------------------------- Phase 2: RX */

/* Feeds a Phase 2 bit through the INFO frame detector; true when a whole
 * frame of i_expect bits is in ibits. */
static bool info_bit(dm_v34_t *v, int k, int bit)
{
    v->isr[k] = ((v->isr[k] << 1) | (uint32_t) bit) & 0xFFFu;
    if (!v->i_collect[k])
    {
        /* 1111 01110010 */
        if (v->i_expect > 0 && v->isr[k] == 0xF72u)
        {
            static const uint8_t head[12] = { 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 0 };

            memcpy(v->ibits[k], head, 12);
            v->ibits_n[k] = 12;
            v->i_collect[k] = true;
        }
        return false;
    }
    v->ibits[k][v->ibits_n[k]++] = (uint8_t) bit;
    if (v->ibits_n[k] >= v->i_expect - 4)
    {
        v->i_collect[k] = false;
        return true;
    }
    return false;
}

static void describe_info0(const v34_info0_t *i, char *out, size_t len)
{
    snprintf(out, len, "2400 3000%s 3200%s%s%s%s%s%s",
             (i->low3000 && i->high3000) ? "" : i->low3000 ? "(low)" : i->high3000 ? "(high)" : "(no tx)",
             (i->low3200 && i->high3200) ? "" : i->low3200 ? "(low)" : i->high3200 ? "(high)" : "(no tx)",
             i->sr2743 ? " 2743" : "", i->sr2800 ? " 2800" : "", (i->sr3429 && i->allow3429) ? " 3429" : "",
             i->c1664 ? ", 1664 points" : "", i->cme ? ", CME" : "");
}

/* Can the far end transmit at symbol rate sr on that carrier? (Table 14) */
static bool far_can_tx(const dm_v34_t *v, int sr, bool high)
{
    const v34_info0_t *i = &v->info0_far;

    switch (sr)
    {
    case V34_S2400:
        return true;
    case V34_S2743:
        return i->sr2743;
    case V34_S2800:
        return i->sr2800;
    case V34_S3000:
        return high ? i->high3000 : i->low3000;
    case V34_S3200:
        return high ? i->high3200 : i->low3200;
    case V34_S3429:
        return i->sr3429 && i->allow3429;
    }
    return false;
}

/* From our probe of the far end's L2: for each symbol rate, the carrier and
 * pre-emphasis the far end should transmit with, and what that would get. */
static void probe_results(dm_v34_t *v, v34_info1c_t *out)
{
    memset(out, 0, sizeof(*out));
    out->freq_offset = -512;
    for (int sr = 0; sr < V34_NUM_SR; sr++)
    {
        float best = -100.0f;

        if (!((v->sr_allow >> sr) & 1))
            continue;
        for (int hi = 0; hi < 2; hi++)
        {
            if (!far_can_tx(v, sr, hi) || (v->force_carrier && hi != v->force_carrier - 1))
                continue;
            for (int pe = 0; pe <= 10; pe++)
            {
                if (v->force_pe >= 0 && pe != v->force_pe)
                    continue;
                float snr = v34_probe_snr(&v->probe, sr, hi, pe);

                if (snr > best + 0.2f)
                {
                    best = snr;
                    out->sr[sr].high = hi;
                    out->sr[sr].pre_emphasis = pe;
                }
            }
        }
        if (best > -100.0f)
        {
            int cap = v->max_rate;

            if (!v->info0_far.c1664 && cap > 28800)
                cap = 28800;
            /* The probe is a forecast; leave a couple of dB for what it
             * cannot see. The rate is settled for real in Phase 4. */
            out->sr[sr].max_rate = best_rate_for(best - 2.0f, sr, cap) / 2400;
            DM_DEBUG("v34", "probing: %s symbols/s, %s carrier, pre-emphasis %d: %.1f dB, %d bit/s (tag=%s)",
                     V34_SR_NAME[sr], out->sr[sr].high ? "high" : "low", out->sr[sr].pre_emphasis, best,
                     out->sr[sr].max_rate * 2400, v->tag);
        }
    }
}

/* 10.1.2.3.5, as the answerer: the two symbol rates. The slower direction
 * first, then the total. */
static void choose_info1a(dm_v34_t *v, v34_info1a_t *a)
{
    v34_info1c_t mine;
    int steps = 5;
    int best_ac = -1, best_ca = -1;
    int best_score = -1;

    probe_results(v, &mine);
    if (v->info0_far.asym_steps < steps)
        steps = v->info0_far.asym_steps;
    for (int ac = 0; ac < V34_NUM_SR; ac++)
    {
        if (!((v->sr_allow >> ac) & 1) || v->info1c.sr[ac].max_rate == 0)
            continue;
        for (int ca = 0; ca < V34_NUM_SR; ca++)
        {
            int lo, score;

            if (mine.sr[ca].max_rate == 0 || abs(ac - ca) > steps)
                continue;
            lo = (v->info1c.sr[ac].max_rate < mine.sr[ca].max_rate) ? v->info1c.sr[ac].max_rate
                                                                     : mine.sr[ca].max_rate;
            score = 1000 * lo + v->info1c.sr[ac].max_rate + mine.sr[ca].max_rate;
            if (score > best_score)
            {
                best_score = score;
                best_ac = ac;
                best_ca = ca;
            }
        }
    }
    if (best_ac < 0)
    {
        best_ac = best_ca = V34_S2400;
        mine.sr[V34_S2400].max_rate = 1;
    }
    memset(a, 0, sizeof(*a));
    a->sr_a_to_c = best_ac;
    a->sr_c_to_a = best_ca;
    a->high = mine.sr[best_ca].high;
    a->pre_emphasis = mine.sr[best_ca].pre_emphasis;
    a->max_rate = mine.sr[best_ca].max_rate;
    a->freq_offset = -512;
}

static void enter_phase3(dm_v34_t *v)
{
    DM_INFO("v34", "probed: sending at %s symbols/s on %.0f Hz with pre-emphasis %d, receiving at %s on %.0f Hz; "
                   "round trip %.0f ms (tag=%s)",
            V34_SR_NAME[v->sr_tx], v34_carrier(v->sr_tx, v->high_tx), v->pe_tx, V34_SR_NAME[v->sr_rx],
            v34_carrier(v->sr_rx, v->high_rx), v->rtd / 8.0, v->tag);
    v34_p2rx_init(&v->tonerx, v->calling);
    qrx_start(v, v->sr_rx, v->high_rx);
    v->q.far_md = v->md_far > 0;
    v->far_trn_on = false;
    if (v->calling)
    {
        v->txm = TXM_SILENCE;
        stage_enter(v, ST_P3_RX, 2.8 + 2.0 * rtd_s(v) + 2.0);
    }
    else
    {
        phase3_tx(v);
        /* Our own S comes back as echo before the canceller has learnt it;
         * the caller's does not come until it has had our J. */
        v->q.mode = RQ_MD;
        v->q.md_until = v->ntx + (long long) ((0.07 + (128 + 16 + 288 + trn_length(v, false)) /
                                                         v34_symbol_rate(v->sr_tx)) * 8000.0) +
                        ((v->rtd > 0) ? v->rtd : 0);
        stage_enter(v, ST_P3_TX_FIRST, 4.0 + 2.0 * rtd_s(v) + 4.0);
    }
}

/* One INFO sequence, from slicer k, whose CRC is still to be checked. */
static bool phase2_frame(dm_v34_t *v, int k);

static void phase2_bits(dm_v34_t *v)
{
    v34_p2rx_t *r = &v->p2rx;

    for (int k = 0; k < V34_P2_PHASES; k++)
    {
        int nb = r->nbits[k];

        r->nbits[k] = 0;
        for (int i = 0; i < nb; i++)
            if (info_bit(v, k, r->bits[k][i]) && phase2_frame(v, k))
            {
                /* One slicer has it; the others must not have it again. */
                for (int j = 0; j < V34_P2_PHASES; j++)
                    r->nbits[j] = 0;
                return;
            }
    }
}

static bool phase2_frame(dm_v34_t *v, int k)
{
    const uint8_t *bits = v->ibits[k];
    int n = v->ibits_n[k];

    if (v->i_expect == V34_INFO0_BITS)
    {
        v34_info0_t i0;
        char d[128];

        if (!v34_info0_unpack(bits, n, &i0))
        {
            DM_TRACE("v34", "an INFO0 failed its CRC (slicer %d) (tag=%s)", k, v->tag);
            return false;
        }
        v->info0_count++;
        if (!v->have_info0)
        {
            describe_info0(&i0, d, sizeof(d));
            DM_DEBUG("v34", "INFO0%c: %s (tag=%s)", v->calling ? 'a' : 'c', d, v->tag);
        }
        v->info0_far = i0;
        v->have_info0 = true;
        if (i0.ack)
            v->ack_info0 = true;
        info_bits_reset(v, V34_INFO0_BITS);
        return true;
    }
    if (v->i_expect == V34_INFO1A_BITS && v->stage == ST_C2_INFO1)
    {
        v34_info1a_t a;

        if (!v34_info1a_unpack(bits, n, &a))
            return false;
        v->info1a = a;
        /* The calling direction is what the answerer chose; the answering
         * one, its symbol rate on what we said for it. */
        v->sr_tx = a.sr_c_to_a;
        v->high_tx = a.high;
        v->pe_tx = a.pre_emphasis;
        v->pr_db = (float) a.min_power_reduction;
        v->md_far = a.md;
        v->sr_rx = a.sr_a_to_c;
        v->high_rx = v->info1c.sr[a.sr_a_to_c].high;
        DM_DEBUG("v34", "INFO1a: answer->call %s, call->answer %s/%s pre-emphasis %d up to %d (tag=%s)",
                 V34_SR_NAME[a.sr_a_to_c], V34_SR_NAME[a.sr_c_to_a], a.high ? "high" : "low", a.pre_emphasis,
                 a.max_rate * 2400, v->tag);
        info_bits_reset(v, 0);
        enter_phase3(v);
        return true;
    }
    if (v->i_expect == V34_INFO1C_BITS && v->stage == ST_A2_INFO1)
    {
        v34_info1c_t c;
        v34_info1a_t a;
        uint8_t out[V34_INFO1A_BITS];
        int no;

        if (!v34_info1c_unpack(bits, n, &c))
            return false;
        v->info1c = c;
        choose_info1a(v, &a);
        v->info1a = a;
        v->sr_tx = a.sr_a_to_c;
        v->high_tx = c.sr[a.sr_a_to_c].high;
        v->pe_tx = c.sr[a.sr_a_to_c].pre_emphasis;
        v->pr_db = (float) c.min_power_reduction;
        v->md_far = c.md;
        v->sr_rx = a.sr_c_to_a;
        v->high_rx = a.high;
        DM_DEBUG("v34", "INFO1c heard; INFO1a: answer->call %s, call->answer %s (tag=%s)",
                 V34_SR_NAME[a.sr_a_to_c], V34_SR_NAME[a.sr_c_to_a], v->tag);
        no = v34_info1a_pack(&a, out);
        p2_send_info(v, out, no);
        v->p2_off_after_bits = true;
        info_bits_reset(v, 0);
        return true;
    }
    return false;
}

/* ------------------------------------------------------------- retrains */

static void restart_phase2(dm_v34_t *v, bool initiate, const char *why)
{
    if (++v->failures > 12)
    {
        cleardown(v, "too many failed attempts at training");
        return;
    }
    DM_INFO("v34", "%s; %s (tag=%s)", why, initiate ? "retraining" : "answering a retrain", v->tag);
    if (v->stage >= ST_P4_MP && v->stage <= ST_RN)
        emit(v, DM_V34_RETRAINING);
    v->tx_in_data = v->rx_in_data = false;
    v->tx_data = false;
    v->renegotiating = false;
    v->rate_tx = v->rate_rx = 0;
    v->have_mp = v->far_ack = v->far_e = false;
    /* 10.1.3.9: the coefficients are zero until the first MP of Phase 4. */
    memset(v->mp_far.h, 0, sizeof(v->mp_far.h));
    v->probing = false;
    v->q.mode = RQ_OFF;
    /* 11.5: 70 ms of silence, then our tone. */
    v->txm = TXM_SILENCE;
    p2_start_rx(v);
    stage_enter(v, ST_RETRAIN, 0.07);
}

static void cleardown(dm_v34_t *v, const char *why)
{
    DM_WARN("v34", "%s (tag=%s)", why, v->tag);
    v->txm = TXM_SILENCE;
    v->q.mode = RQ_OFF;
    v->tx_in_data = v->rx_in_data = false;
    stage_enter(v, ST_DEAD, 0.0);
    emit(v, DM_V34_CLEARDOWN);
}

static void not_v34(dm_v34_t *v, bool no_v8)
{
    v->txm = TXM_SILENCE;
    stage_enter(v, ST_DEAD, 0.0);
    emit(v, no_v8 ? DM_V34_NO_V8 : DM_V34_NOT_V34);
}

/* Whether what spandsp took for ANSam really is: a clean 15 Hz sine on the
 * tone's envelope, about 20% deep. spandsp has twice taken a 2400 bps
 * modem's plain answer tone for ANSam on a real call - and CM sent at an
 * older modem is FSK in its own calling band, which it may well answer as
 * if we were a V.21 caller. Returns -1 while there is not yet a second of
 * tone to judge, else 1 for ANSam and 0 for not. */
static int ansam_verify(dm_v34_t *v)
{
    float x[ENV_BLOCKS];
    float sorted[ENV_BLOCKS];
    float median;
    float mean = 0.0f;
    float ac = 0.0f;
    double re = 0.0;
    double im = 0.0;
    float a15;
    float depth;
    float purity;
    bool yes;

    if (v->env_count < ENV_BLOCKS)
        return -1;
    for (int k = 0; k < ENV_BLOCKS; k++)
        x[k] = sorted[k] = v->env[(v->env_count + k) % ENV_BLOCKS];
    for (int i = 1; i < ENV_BLOCKS; i++)
        for (int j = i; j > 0 && sorted[j - 1] > sorted[j]; j--)
        {
            float t = sorted[j];

            sorted[j] = sorted[j - 1];
            sorted[j - 1] = t;
        }
    median = sorted[ENV_BLOCKS / 2];
    if (median <= 0.0f)
        return 0;
    /* A phase reversal - ANSam's, every 450 ms - empties the block it falls
     * in. 20% of AM never takes the envelope below 0.8 of its mean, so
     * anything far below is a reversal, and is not modulation. */
    for (int k = 0; k < ENV_BLOCKS; k++)
    {
        if (x[k] < 0.6f * median)
            x[k] = median;
        mean += x[k];
    }
    mean /= ENV_BLOCKS;
    for (int k = 0; k < ENV_BLOCKS; k++)
    {
        double w = 2.0 * M_PI * 15.0 * k / 100.0;

        re += (x[k] - mean) * cos(w);
        im -= (x[k] - mean) * sin(w);
        ac += (x[k] - mean) * (x[k] - mean);
    }
    ac /= ENV_BLOCKS;
    a15 = (float) (2.0 * sqrt(re * re + im * im) / ENV_BLOCKS);
    depth = a15 / mean;
    purity = (ac > 0.0f) ? (a15 * a15 / 2.0f) / ac : 0.0f;
    yes = depth >= 0.1f && depth <= 0.35f && purity >= 0.5f;
    DM_INFO("v34", "answer tone: 15 Hz modulation %.0f%% deep, %.0f%% of its fluctuation - %s (tag=%s)",
            100.0f * depth, 100.0f * purity, yes ? "ANSam" : "not ANSam, whatever spandsp says", v->tag);
    return yes ? 1 : 0;
}

/* --------------------------------------------------------- control loop */

static void begin_phase2(dm_v34_t *v)
{
    v34_p2tx_init(&v->p2tx, !v->calling, v->nominal_dbm0);
    v->have_info0 = false;
    v->ack_info0 = false;
    v->info0_count = 0;
    send_info0(v);
    stage_enter(v, v->calling ? ST_C2_INFO0 : ST_A2_INFO0, 10.0);
    DM_DEBUG("v34", "Phase 2: INFO0%c (tag=%s)", v->calling ? 'c' : 'a', v->tag);
}

static void control(dm_v34_t *v, long long n, double rev, bool reversed)
{
    v34_p2rx_t *r = &v->p2rx;
    double rt = (v->rtd >= 0) ? v->rtd / 8000.0 : 0.0;
    bool tone = r->tone_since >= 0;

    switch (v->stage)
    {
    /* ---------------------------------------------------------- V.8 */
    case ST_V8_C_LISTEN:
    {
        int t = modem_connect_tones_rx_get(v->ansam_rx);

        /* spandsp's detector takes two and a half seconds to call ANSam -
         * which on a real call left an answerer one second of our CM before
         * its ANSam ran out. A second of the tone is enough to measure. */
        if (t == MODEM_CONNECT_TONES_ANSAM || t == MODEM_CONNECT_TONES_ANSAM_PR || v->tone_run >= ENV_BLOCKS)
            v->ansam_claimed = true;
        if (v->ansam_claimed)
        {
            int ok = ansam_verify(v);

            if (ok == 1)
            {
                DM_DEBUG("v34", "ANSam heard (tag=%s)", v->tag);
                stage_enter(v, ST_V8_C_TE, 0.5);
            }
            else if (ok == 0)
            {
                DM_INFO("v34", "the far end answered with a plain answer tone and no V.8, so it is not a V.34 "
                               "modem (tag=%s)",
                        v->tag);
                not_v34(v, true);
            }
        }
        else if (t == MODEM_CONNECT_TONES_ANS || t == MODEM_CONNECT_TONES_ANS_PR)
        {
            DM_INFO("v34", "the far end answered with a plain answer tone and no V.8, so it is not a V.34 modem "
                           "(tag=%s)",
                    v->tag);
            not_v34(v, true);
        }
        break;
    }
    case ST_V8_C_CM:
        if (v->v8_got_msg)
        {
            char d[96];

            v8_describe(&v->v8_far, d, sizeof(d));
            DM_INFO("v34", "V.8: the answering modem offers%s (tag=%s)", d, v->tag);
            if (!v->v8_far.v34)
            {
                not_v34(v, false);
                break;
            }
            /* 11.1.1.1: complete the CM octet in hand, then CJ. Messages are
             * whole multiples of ten bits. */
            {
                int left = (v->v8t - v->v8h + (int) sizeof(v->v8q)) % (int) sizeof(v->v8q);
                uint8_t cj[30];
                int nb = 0;

                v->v8t = (v->v8h + left % 10) % (int) sizeof(v->v8q);
                v->v8_repeat = false;
                for (int k = 0; k < 3; k++)
                    v8_octet(cj, &nb, 0);
                for (int k = 0; k < nb; k++)
                {
                    v->v8q[v->v8t] = cj[k];
                    v->v8t = (v->v8t + 1) % (int) sizeof(v->v8q);
                }
            }
            stage_enter(v, ST_V8_C_CJ, 2.0);
        }
        break;
    case ST_V8_C_CJ:
        /* dm_v34_tx goes quiet once CJ's last stop bit is out, and starts
         * Phase 2 75 ms later (11.1.1.1). 11.2.1.1.1: the receiver listens
         * for INFO0a through the silence. */
        if (v->p2_due >= 0)
        {
            p2_start_rx(v);
            stage_enter(v, ST_V8_DONE, 0.5);
        }
        break;
    case ST_V8_A_ANSAM:
        if (v->v8_got_msg)
        {
            char d[96];
            v8_msg_t jm = { 0 };

            v8_describe(&v->v8_far, d, sizeof(d));
            DM_INFO("v34", "V.8: the calling modem offers%s (tag=%s)", d, v->tag);
            if (!v->v8_far.v34)
            {
                not_v34(v, false);
                break;
            }
            jm.call_function = v->v8_far.call_function;
            jm.v34 = true;
            jm.lapm = v->lapm && v->v8_far.lapm;
            v8_build_msg(v, &jm);
            v8_start_fsk(v, true);
            v->v8_got_cj = false;
            stage_enter(v, ST_V8_A_JM, 5.0);
        }
        else if (v->ansam_done_at > 0 && n - v->ansam_done_at > MS(1000))
        {
            DM_INFO("v34", "no CM in answer to ANSam: the calling modem does not do V.8 (tag=%s)", v->tag);
            not_v34(v, true);
        }
        break;
    case ST_V8_A_JM:
        if (v->v8_got_cj)
        {
            /* 11.1.2.2: silence, then Phase 2 75 ms later - both timed by
             * dm_v34_tx. */
            v->v8_quiet = true;
            p2_start_rx(v);
            stage_enter(v, ST_V8_DONE, 0.5);
        }
        break;
    case ST_V8_DONE:
        break;

    /* ------------------------------------------------ Phase 2, calling */
    case ST_C2_INFO0:
        /* 11.2.2.1.1: tone A without INFO0a, or INFO0a again, means the
         * answerer did not get ours. */
        if (v34_p2tx_pending(&v->p2tx) == 0 && v->txm == TXM_P2 && !v->ack_info0 &&
            ((tone && n - r->tone_since > MS(100) && !v->have_info0) || v->info0_count >= 2))
        {
            v->info0_count = 1;
            send_info0(v);
        }
        if (reversed && v->have_info0 && v34_p2tx_pending(&v->p2tx) == 0)
        {
            /* 11.2.1.1.3: our reversal 40 ms after theirs. */
            p2_reverse(v, (long long) rev + MS(40), 10, false);
            v->rev_cand = -1.0;
            stage_enter(v, ST_C2_REV2, 2.1);
            DM_DEBUG("v34", "tone A reversed; reversing B (tag=%s)", v->tag);
        }
        break;
    case ST_C2_REV2:
        if (v->info0_count >= 2 && !v->ack_info0)
        {
            /* 11.2.2.1.1: the answerer is repeating INFO0a, so it never had
             * ours. */
            v->info0_count = 1;
            p2_carrier(v, true);
            send_info0(v);
            stage_enter(v, ST_C2_INFO0, 10.0);
            break;
        }
        /* 11.2.1.2.5: the answerer's second reversal is followed, 10 ms
         * later, by L1 - which has no 2400 Hz in it, so tone A goes. A
         * reversal with tone A still there 50 ms on was something else: a
         * MICA has been heard to reverse A several times over when our last
         * INFO0c was still reaching it, taking its DPSK for tone B's
         * reversal, and timing the round trip from one of those put the
         * probing window in the wrong place. We send nothing at this
         * reversal, so there is time to look. */
        if (reversed && rev > (double) v->rev_sent)
            v->rev_cand = rev;
        if (v->rev_cand >= 0.0 && (double) n >= v->rev_cand + MS(50))
        {
            rev = v->rev_cand;
            v->rev_cand = -1.0;
            if (tone)
            {
                DM_DEBUG("v34", "tone A reversed %.1f ms after ours but went on, so that was not the reversal before "
                                "L1; still waiting (tag=%s)",
                         (rev - (double) v->rev_sent) / 8.0, v->tag);
                break;
            }
            /* 11.2.1.1.4 */
            v->rtd = (long long) (rev - (double) v->rev_sent) - MS(40);
            if (v->rtd < 0)
                v->rtd = 0;
            DM_DEBUG("v34", "round trip %.1f ms; receiving L1 and L2 (tag=%s)", v->rtd / 8.0, v->tag);
            v34_probe_init(&v->probe);
            v->probing = true;
            /* L1 after 10 ms more of A; L2 160 ms after that. */
            v->probe_from = (long long) rev + MS(10 + 160 + 20);
            stage_enter(v, ST_C2_PROBE, 1.5);
        }
        break;
    case ST_C2_PROBE:
        if (!v->probing)
        {
            v34_probe_analyse(&v->probe);
            probe_results(v, &v->info1c);
            p2_carrier(v, true);    /* tone B */
            stage_enter(v, ST_C2_B, 0.9 + rt + 0.5);
        }
        break;
    case ST_C2_B:
        if (reversed)
        {
            /* 11.2.1.1.6 */
            p2_reverse(v, (long long) rev + MS(40), 10, true);
            stage_enter(v, ST_C2_L, 0.06 + 0.16 + 0.65 + rt + 0.5);
        }
        break;
    case ST_C2_L:
        /* 11.2.1.1.7: tone A while L2 goes out. */
        if (v->p2_l2_at < 0 && v->p2tx.probe && tone && n - r->tone_since > MS(30) &&
            n > v->rev_sent + MS(10 + 160 + 100))
        {
            uint8_t bits[V34_INFO1C_BITS];
            int nb = v34_info1c_pack(&v->info1c, bits);

            p2_carrier(v, true);
            v34_p2tx_bits(&v->p2tx, bits, nb);
            v->p2_off_after_bits = true;
            info_bits_reset(v, V34_INFO1A_BITS);
            stage_enter(v, ST_C2_INFO1, 0.7 + rt + 0.6);
            DM_DEBUG("v34", "tone A heard; INFO1c (tag=%s)", v->tag);
        }
        break;
    case ST_C2_INFO1:
        break;

    /* ---------------------------------------------- Phase 2, answering */
    case ST_A2_INFO0:
        if (v34_p2tx_pending(&v->p2tx) == 0 && v->txm == TXM_P2 && !v->ack_info0 &&
            ((tone && n - r->tone_since > MS(100) && !v->have_info0) || v->info0_count >= 2))
        {
            v->info0_count = 1;
            send_info0(v);
        }
        /* 11.2.1.2.3: tone B heard with INFO0c in, and A sent for 50 ms. */
        if (v->have_info0 && v34_p2tx_pending(&v->p2tx) == 0 && v->txm == TXM_P2 && tone &&
            n - r->tone_since > MS(20) && v->ntx - v->p2_tone_from > MS(50) && v->p2tx.flip_at < 0)
        {
            p2_reverse(v, v->ntx + 8, 0, false);
            stage_enter(v, ST_A2_REV1, 2.0);
            DM_DEBUG("v34", "tone B heard; reversing A (tag=%s)", v->tag);
        }
        break;
    case ST_A2_REV1:
        if (v->info0_count >= 2 && !v->ack_info0)
        {
            /* 11.2.2.2.1: the caller is repeating INFO0c, so it never had
             * ours. */
            v->info0_count = 1;
            send_info0(v);
            stage_enter(v, ST_A2_INFO0, 10.0);
            break;
        }
        if (reversed && rev > (double) v->rev_sent)
        {
            /* 11.2.1.2.4 and 5 */
            v->rtd = (long long) (rev - (double) v->rev_sent) - MS(40);
            if (v->rtd < 0)
                v->rtd = 0;
            DM_DEBUG("v34", "round trip %.1f ms; L1 and L2 (tag=%s)", v->rtd / 8.0, v->tag);
            p2_reverse(v, (long long) rev + MS(40), 10, true);
            stage_enter(v, ST_A2_L, 0.05 + 0.16 + 0.6 + rt + 0.5);
        }
        break;
    case ST_A2_L:
        if (v->p2_l2_at < 0 && v->p2tx.probe && tone && n - r->tone_since > MS(30) &&
            n > v->rev_sent + MS(10 + 160 + 50))
        {
            /* 11.2.1.2.6: A for 50 ms, a reversal, 10 ms more, silence. */
            p2_carrier(v, true);
            p2_reverse(v, v->ntx + MS(50), 10, false);
            stage_enter(v, ST_A2_A50, 2.0 + rt);
        }
        break;
    case ST_A2_A50:
        if (reversed && rev > (double) v->rev_sent)
        {
            /* 11.2.1.2.7 and 8 */
            v34_probe_init(&v->probe);
            v->probing = true;
            v->probe_from = (long long) rev + MS(10 + 160 + 20);
            stage_enter(v, ST_A2_PROBE, 1.5);
        }
        break;
    case ST_A2_PROBE:
        if (!v->probing)
        {
            v34_probe_analyse(&v->probe);
            p2_carrier(v, true);    /* tone A */
            info_bits_reset(v, V34_INFO1C_BITS);
            stage_enter(v, ST_A2_INFO1, 2.0 + 2.0 * rt + 0.5);
        }
        break;
    case ST_A2_INFO1:
        /* INFO1a out, then Phase 3 (11.2.1.2.9). */
        if (v->i_expect == 0 && v->txm == TXM_SILENCE)
            enter_phase3(v);
        break;

    case ST_RETRAIN:
        break;

    /* --------------------------------------------------- Phase 3 and 4 */
    case ST_P4_TRN:
        /* 11.4.1: TRN for at least 512T, and until our receiver has had 512T
         * of the far end's. */
        if (v->seg == TS_TRN && v->trn_symbols >= 512 && v->q.mode == RQ_DD && v->far_trn >= 600)
        {
            choose_rx_rate(v);
            start_mp(v);
        }
        break;
    case ST_RN:
        if (v->seg == TS_TRN && v->trn_symbols >= 512 && v->q.mode == RQ_DD && v->far_trn >= 400)
        {
            choose_rx_rate(v);
            start_mp(v);
        }
        break;
    case ST_P4_MP:
        /* 11.4.1.1.3 and 11.4.1.2.3: E once we have sent MP' and heard MP'
         * or E. */
        if (v->far_ack && v->mpbits[33] && v->mpp_sent >= 1 && !v->mp_e_pending && v->seg == TS_MP)
        {
            if (!settle_rates(v))
            {
                cleardown(v, "the two modems have no data rate in common");
                break;
            }
            v->mp_e_pending = true;
            stage_enter(v, ST_P4_B1, 3.0 + 2.0 * rt);
            DM_DEBUG("v34", "MP' exchanged: %d bit/s out, %d in; E and B1 (tag=%s)", v->rate_tx, v->rate_rx,
                     v->tag);
        }
        break;
    case ST_P4_B1:
        if (v->seg == TS_DATA && v->tx_data && !v->tx_in_data)
        {
            v->tx_in_data = true;
            if (v->rx_in_data)
                go_data(v);
        }
        break;
    case ST_DATA:
        if (v->renegotiating)
            break;
        {
            float snr = 10.0f * log10f(v->rx_energy / (v->dec.err + 1e-9f));

            v->snr_db = snr;
            if (v->pwr_fast < v->pmin)
            {
                if (v->low_since == 0)
                    v->low_since = n;
                else if (n - v->low_since > MS(300))
                {
                    emit(v, DM_V34_CARRIER_DOWN);
                    restart_phase2(v, true, "the far end's signal has gone");
                    return;
                }
            }
            else
            {
                v->low_since = 0;
            }
            if (snr < rate_needs_db(v->rate_rx, v->sr_rx) - 4.0f)
            {
                if (v->poor_since == 0)
                    v->poor_since = n;
                else if (n - v->poor_since > MS(2000))
                {
                    char why[96];

                    snprintf(why, sizeof(why), "reception has been poor (%.1f dB SNR) for 2 seconds", snr);
                    restart_phase2(v, true, why);
                    return;
                }
            }
            else
            {
                v->poor_since = 0;
            }
        }
        break;
    default:
        break;
    }

    /* A tone from the far end in Phases 3, 4 or data is a retrain (11.5). */
    if (v->stage >= ST_P3_RX && v->stage <= ST_RN)
    {
        if (v->tonerx.tone_since >= 0 && n - v->tonerx.tone_since > MS(60) && v->tonerx.level > 4.0f * v->pmin)
        {
            restart_phase2(v, false, v->calling ? "the answerer is sending tone A" : "the caller is sending tone B");
            return;
        }
    }

    if (v->deadline > 0 && n >= v->deadline)
    {
        char why[96];

        switch (v->stage)
        {
        case ST_V8_C_TE:
        {
            v8_msg_t cm = { 0 };

            /* 11.1.1.1: CM, offering V.34 duplex. */
            cm.call_function = 6;
            cm.v34 = true;
            cm.lapm = v->lapm;
            v8_build_msg(v, &cm);
            v8_start_fsk(v, true);
            v8_start_rx(v);
            stage_enter(v, ST_V8_C_CM, 6.0);
            return;
        }
        case ST_V8_DONE:
            /* Only if dm_v34_tx never got there itself. */
            v->p2_due = -1;
            v->txm = TXM_SILENCE;
            begin_phase2(v);
            return;
        case ST_V8_C_CM:
        case ST_V8_A_JM:
        case ST_V8_C_CJ:
            DM_WARN("v34", "V.8 stalled at '%s' (tag=%s)", STAGE_NAMES[v->stage], v->tag);
            not_v34(v, false);
            return;
        case ST_RETRAIN:
            /* 11.5.1 and 11.5.2: our tone, then Phase 2 from the reversal. */
            v34_p2tx_init(&v->p2tx, !v->calling, v->nominal_dbm0);
            p2_carrier(v, true);
            v->info0_count = 1;
            v->ack_info0 = true;
            stage_enter(v, v->calling ? ST_C2_INFO0 : ST_A2_INFO0, 6.0 + 2.0 * rt);
            return;
        default:
            break;
        }
        snprintf(why, sizeof(why), "timed out at '%s'", STAGE_NAMES[v->stage]);
        emit(v, DM_V34_TRAINING_FAILED);
        restart_phase2(v, true, why);
    }
}

/* ---------------------------------------------------------------- driving */

int dm_v34_tx(dm_v34_t *v, int16_t *amp, int len)
{
    for (int i = 0; i < len; i++)
    {
        float x = 0.0f;
        bool learn = false;

        switch (v->txm)
        {
        case TXM_SILENCE:
            if (v->p2_due >= 0 && v->ntx >= v->p2_due)
            {
                v->p2_due = -1;
                begin_phase2(v);
            }
            break;
        case TXM_V8:
        {
            int16_t s = 0;

            if (v->stage == ST_V8_A_ANSAM && v->ansam_tx != NULL)
            {
                if (modem_connect_tones_tx(v->ansam_tx, &s, 1) < 1 && v->ansam_done_at == 0)
                    v->ansam_done_at = v->ntx;
            }
            else if (v->fsk_tx != NULL)
            {
                fsk_tx(v->fsk_tx, &s, 1);
            }
            x = s;
            /* The end of V.8, timed here in transmit time and not by the
             * stage machine: that runs on receive time, a frame behind, and
             * used to cut CJ off before its last stop bit, which left the
             * far end without a third octet and still sending JM. A few bits
             * of mark after CJ, then 75 ms of silence (11.1.1.1, 11.1.2.2). */
            if (v->v8_quiet || (v->v8_done_at >= 0 && v->ntx >= v->v8_done_at + MS(10)))
            {
                v->v8_quiet = false;
                v->v8_done_at = -1;
                v->txm = TXM_SILENCE;
                v->p2_due = v->ntx + MS(75);
                x = 0.0f;
            }
            break;
        }
        case TXM_P2:
            if (v->p2_off_at >= 0 && v->ntx >= v->p2_off_at)
            {
                v34_p2tx_carrier(&v->p2tx, false);
                v->p2_off_at = -1;
                if (v->p2_probe_at >= 0)
                {
                    v34_p2tx_probe(&v->p2tx, true, true, v->nominal_dbm0);
                    v->p2_probe_at = -1;
                }
            }
            if (v->p2_l2_at >= 0 && v->ntx >= v->p2_l2_at)
            {
                v34_p2tx_probe(&v->p2tx, true, false, v->nominal_dbm0);
                v->p2_l2_at = -1;
            }
            x = v34_p2tx_sample(&v->p2tx, v->ntx);
            if (v->p2_off_after_bits && v34_p2tx_pending(&v->p2tx) == 0)
            {
                /* Let the last symbol finish before the carrier stops. */
                v->p2_off_after_bits = false;
                v->p2_off_at = v->ntx + MS(8);
            }
            if (!v->p2tx.on && !v->p2tx.probe && v->p2_off_at < 0 && v->p2_l2_at < 0)
                v->txm = TXM_SILENCE;
            break;
        case TXM_QAM:
            x = v34_qtx_sample(&v->qtx);
            learn = v->seg == TS_TRN || v->seg == TS_DATA || v->seg == TS_MP || v->seg == TS_PP ||
                    v->seg == TS_J;
            break;
        }
        if (x > 32767.0f)
            x = 32767.0f;
        else if (x < -32768.0f)
            x = -32768.0f;
        amp[i] = (int16_t) lrintf(x);
        v34_ec_tx(&v->ec, v->ntx, (float) amp[i], learn);
        v->ntx++;
    }
    return len;
}

static void rx_sample(dm_v34_t *v, float x, bool missing)
{
    long long n = v->n;
    double rev = 0.0;
    bool reversed = false;
    float e = missing ? 0.0f : v34_ec_run(&v->ec, x, n, v->pwr, v->pmin);

    v->pwr += 0.005f * (e * e - v->pwr);
    v->pwr_fast += 0.05f * (e * e - v->pwr_fast);
    v->n++;
    switch (v->stage)
    {
    case ST_V8_C_LISTEN:
    case ST_V8_C_TE:
    {
        int16_t s = (int16_t) lrintf(e);

        modem_connect_tones_rx(v->ansam_rx, &s, 1);
        /* The tone's envelope: mixed down from 2100 Hz and averaged over
         * 10 ms, which passes 15 Hz and a tone up to V.25's 15 Hz off. */
        v->env_re += e * (float) cos(v->env_phase);
        v->env_im -= e * (float) sin(v->env_phase);
        v->env_pwr += e * e;
        v->env_phase += 2.0 * M_PI * 2100.0 / 8000.0;
        if (v->env_phase > 2.0 * M_PI)
            v->env_phase -= 2.0 * M_PI;
        if (++v->env_n == ENV_BLOCK)
        {
            float a = sqrtf(v->env_re * v->env_re + v->env_im * v->env_im) / ENV_BLOCK;
            float ms = v->env_pwr / ENV_BLOCK;

            v->env[v->env_count++ % ENV_BLOCKS] = 2.0f * a;
            /* A tone of amplitude A mixes down to A/2 and has A^2/2 of mean
             * square: most of the block, at a level worth hearing, is the
             * answer tone. One block that is not - where a phase reversal
             * fell - does not end the run. */
            if (ms > v->pmin && 2.0f * a * a >= 0.6f * ms)
            {
                v->tone_run++;
                v->tone_gap = 0;
            }
            else if (v->tone_run > 0 && v->tone_gap == 0)
            {
                v->tone_run++;
                v->tone_gap = 1;
            }
            else
            {
                v->tone_run = 0;
            }
            v->env_re = v->env_im = v->env_pwr = 0.0f;
            v->env_n = 0;
        }
        break;
    }
    case ST_V8_C_CM:
    case ST_V8_C_CJ:
    case ST_V8_A_ANSAM:
    case ST_V8_A_JM:
    {
        int16_t s = (int16_t) lrintf(e);

        if (v->fsk_rx != NULL)
            fsk_rx(v->fsk_rx, &s, 1);
        break;
    }
    case ST_DEAD:
        break;
    default:
        if (v->stage <= ST_A2_INFO1 || v->stage == ST_RETRAIN)
        {
            reversed = v34_p2rx_sample(&v->p2rx, e, n, &rev);
            phase2_bits(v);
            if (v->probing && n >= v->probe_from)
            {
                if (v34_probe_sample(&v->probe, e) || v->probe.blocks >= 20)
                    v->probing = false;
            }
        }
        else
        {
            double dummy;

            (void) v34_p2rx_sample(&v->tonerx, e, n, &dummy);
            memset(v->tonerx.nbits, 0, sizeof(v->tonerx.nbits));
            if (v->q.mode != RQ_OFF)
                qrx_sample(v, e, n);
        }
        break;
    }
    control(v, n, rev, reversed);
}

void dm_v34_rx(dm_v34_t *v, const int16_t *amp, int len)
{
    if (v->ntx == 0)
        return;
    for (int i = 0; i < len; i++)
        rx_sample(v, (float) amp[i], false);
}

void dm_v34_rx_fillin(dm_v34_t *v, int len)
{
    if (v->ntx == 0 || len <= 0)
        return;
    v->freeze_until = v->n + len + 400;
    v->ec.freeze_until = v->freeze_until;
    for (int i = 0; i < len; i++)
        rx_sample(v, 0.0f, true);
}

dm_v34_t *dm_v34_create(const dm_v34_params_t *p)
{
    dm_v34_t *v = calloc(1, sizeof(*v));

    if (v == NULL)
        return NULL;
    v->v8_done_at = v->p2_due = -1;
    v->calling = p->calling;
    snprintf(v->tag, sizeof(v->tag), "%s", p->tag ? p->tag : (p->calling ? "out" : "in"));
    if (!v34_ec_init(&v->ec, v->tag))
    {
        free(v);
        return NULL;
    }
    v->max_rate = (p->max_rate <= 0 || p->max_rate > 33600) ? 33600 : p->max_rate;
    v->sr_allow = p->symbol_rates ? (p->symbol_rates | 1u) & 0x3Fu : 0x3Fu;
    v->force_carrier = p->carrier;
    v->force_pe = p->pre_emphasis;
    v->req_trellis = (p->trellis == 64) ? 2 : (p->trellis == 32) ? 1 : 0;
    v->req_shaping = p->shaping;
    v->lapm = p->lapm;
    v->nominal_dbm0 = p->tx_power;
    v->get_bit = p->get_bit;
    v->put_bit = p->put_bit;
    v->event = p->event;
    v->user = p->user;
    /* Clause 7: GPC calling, GPA answering. */
    v->scr_tap = p->calling ? 18 : 5;
    v->dscr_tap = p->calling ? 5 : 18;
    v->p0 = (float) (V34_DBM0_RMS * V34_DBM0_RMS);
    v->pmin = v->p0 * powf(10.0f, -48.0f / 10.0f);
    v->rtd = -1;
    v->p2_off_at = v->p2_probe_at = v->p2_l2_at = -1;
    v->rx_gain = 1.0f;
    v->rx_energy = 1.0f;

    if (v->calling)
    {
        /* 11.1.1.1: listen for ANSam, sending nothing. */
        v->ansam_rx = modem_connect_tones_rx_init(NULL, MODEM_CONNECT_TONES_ANS_PR, NULL, NULL);
        v->txm = TXM_SILENCE;
        stage_enter(v, ST_V8_C_LISTEN, 0.0);
        if (v->ansam_rx == NULL)
        {
            dm_v34_free(v);
            return NULL;
        }
    }
    else
    {
        /* 11.1.2.1: 200 ms of silence and then ANSam - spandsp's generator
         * starts with the silence - listening for CM all the while. */
        v->ansam_tx = modem_connect_tones_tx_init(NULL, MODEM_CONNECT_TONES_ANSAM_PR);
        v->txm = TXM_V8;
        v8_start_rx(v);
        stage_enter(v, ST_V8_A_ANSAM, 0.0);
        if (v->ansam_tx == NULL || v->fsk_rx == NULL)
        {
            dm_v34_free(v);
            return NULL;
        }
    }
    return v;
}

void dm_v34_free(dm_v34_t *v)
{
    if (v == NULL)
        return;
    if (v->fsk_tx != NULL)
        fsk_tx_free(v->fsk_tx);
    if (v->fsk_rx != NULL)
        fsk_rx_free(v->fsk_rx);
    if (v->ansam_tx != NULL)
        modem_connect_tones_tx_free(v->ansam_tx);
    if (v->ansam_rx != NULL)
        modem_connect_tones_rx_free(v->ansam_rx);
    v34_qtx_free(&v->qtx);
    qrx_free(&v->q);
    v34_ec_free(&v->ec);
    free(v);
}

int dm_v34_tx_rate(const dm_v34_t *v)
{
    return (v->stage == ST_DATA || v->tx_in_data) ? v->rate_tx : 0;
}

bool dm_v34_engaged(const dm_v34_t *v)
{
    /* Calling, ANSam alone is not enough: an older modem's answer tone can
     * be taken for it, and the CM that follows then goes unanswered. JM is
     * the far end saying V.8. */
    if (v->calling)
        return v->stage != ST_V8_C_LISTEN && v->stage != ST_V8_C_TE && v->stage != ST_V8_C_CM &&
               v->stage != ST_DEAD;
    return v->stage != ST_V8_A_ANSAM && v->stage != ST_DEAD;
}

int dm_v34_rx_rate(const dm_v34_t *v)
{
    return (v->stage == ST_DATA || v->rx_in_data) ? v->rate_rx : 0;
}

bool dm_v34_renegotiate(dm_v34_t *v, int rx_rate)
{
    if (v->stage != ST_DATA || v->renegotiating)
        return false;
    /* 11.6.1.1.1: S, S-bar, TRN, then MP, all on the 4-point set. */
    v->renegotiating = true;
    v->rn_initiator = true;
    v->rn_want = rx_rate;
    v->tx_in_data = false;
    v->tx_data = false;
    v->tx16 = false;
    v->have_mp = v->far_ack = v->far_e = false;
    v->far_trn_on = false;
    tx_set(v, TS_S, 128);
    tx_then(v, TS_SBAR, 16);
    tx_then(v, TS_TRN, -1);
    stage_enter(v, ST_RN, 2.5 + 2.0 * rtd_s(v) + 2.0);
    emit(v, DM_V34_RETRAINING);
    DM_INFO("v34", "asking the far end to change rate (to %d bit/s inbound at most) (tag=%s)", rx_rate, v->tag);
    return true;
}

float dm_v34_rx_power(const dm_v34_t *v)
{
    return 10.0f * log10f(v->pwr / v->p0 + 1e-12f);
}

void dm_v34_stats(const dm_v34_t *v, dm_v34_stats_t *out)
{
    memset(out, 0, sizeof(*out));
    out->stage = STAGE_NAMES[v->stage];
    out->tx_rate = dm_v34_tx_rate(v);
    out->rx_rate = dm_v34_rx_rate(v);
    if (v->stage >= ST_P3_TX_FIRST && v->stage <= ST_RN)
    {
        out->tx_symbol_rate = (int) lrint(v34_symbol_rate(v->sr_tx));
        out->rx_symbol_rate = (int) lrint(v34_symbol_rate(v->sr_rx));
        out->tx_carrier = (int) lrint(v34_carrier(v->sr_tx, v->high_tx));
        out->rx_carrier = (int) lrint(v34_carrier(v->sr_rx, v->high_rx));
    }
    out->tx_pre_emphasis = v->pe_tx;
    out->tx_trellis = v->dp_tx.trellis;
    out->tx_precoding = v->dp_tx.h[0][0] || v->dp_tx.h[0][1] || v->dp_tx.h[1][0] || v->dp_tx.h[1][1] ||
                        v->dp_tx.h[2][0] || v->dp_tx.h[2][1];
    out->tx_nonlinear = v->dp_tx.nonlinear;
    out->tx_shaping = v->dp_tx.expanded;
    out->snr_db = v->snr_db;
    out->rx_power = dm_v34_rx_power(v);
    out->round_trip_ms = (v->rtd >= 0) ? (int) (v->rtd / 8) : -1;
    out->echo_canceller = v->ec.enabled;
    out->echo_delay_ms = v->ec.enabled ? (float) (v->ec.delay + V34_EC_PRE) / 8.0f : 0.0f;
    out->echo_return_loss_db = v->ec.erl_db;
    out->echo_cancelled_db = v->ec.erle_db;
    out->retrains = v->retrains;
    out->renegotiations = v->renegotiations;
}
