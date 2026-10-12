/* The data-mode half of ITU-T V.34 (02/98), clauses 7 to 9: everything
 * between the scrambled bit stream and the 2D signal points, and back.
 *
 * Kept apart from the signal processing in v34.c because none of it is
 * signal processing. It is integer bookkeeping - framing, shell mapping,
 * differential coding, the precoder, the trellis code - that has to match the
 * far end's bit for bit, and it can be tested on its own, without a line in
 * between, by feeding the encoder's output straight to the decoder.
 *
 * Signal points here are in the units of Figure 5: the odd-integer grid,
 * point 0 at 1 + j. Scaling to a transmit level is the modulator's business.
 *
 * Section, table and figure numbers are V.34's throughout. */
#ifndef DATAMODEM_V34_CODEC_H
#define DATAMODEM_V34_CODEC_H

#include <complex.h>
#include <stdbool.h>
#include <stdint.h>

typedef float complex v34_cf_t;

/* Symbol rates, in the order INFO sequences number them (Tables 14 to 16). */
enum
{
    V34_S2400 = 0,
    V34_S2743,
    V34_S2800,
    V34_S3000,
    V34_S3200,
    V34_S3429,
    V34_NUM_SR
};

/* Table 1: S = 2400 a/c. */
extern const int V34_SR_A[V34_NUM_SR];
extern const int V34_SR_C[V34_NUM_SR];
/* Table 2: carrier = S d/e, [sr][0 low, 1 high]. */
extern const int V34_CAR_D[V34_NUM_SR][2];
extern const int V34_CAR_E[V34_NUM_SR][2];
/* The highest data rate each symbol rate carries (Table 8). */
extern const int V34_SR_MAX_RATE[V34_NUM_SR];
extern const char *const V34_SR_NAME[V34_NUM_SR];

double v34_symbol_rate(int sr);
double v34_carrier(int sr, bool high);

/* The quarter superconstellation of Figure 5: label -> point. */
#define V34_QUARTER 416
extern int8_t v34_super[V34_QUARTER][2];

/* The 3-bit subset label of Figure 9 for any point of the odd-integer grid. */
int v34_subset_label(int re, int im);

/* Everything the data mode needs to know about one direction of
 * transmission. The transmitter takes the trellis code, the shaping, the
 * non-linear encoder and the precoding coefficients from the far end's MP;
 * the receiver asks for them in its own. */
typedef struct
{
    int sr;                   /* V34_S2400 ... V34_S3429 */
    int rate;                 /* primary channel, bit/s, 2400 ... 33600 */
    bool aux;                 /* the 200 bit/s auxiliary channel is on */
    int trellis;              /* 16, 32 or 64 states */
    bool expanded;            /* shaping: the larger M of Table 10 */
    bool nonlinear;           /* Theta = 0.3125 (9.7) */
    int16_t h[3][2];          /* precoding coefficients h(1..3), re/im, 2.14 two's complement */
} v34_data_params_t;

/* Derived from the parameters: 8.1 to 9.2. */
typedef struct
{
    int J;                    /* data frames per superframe */
    int P;                    /* mapping frames per data frame */
    int W;                    /* auxiliary bits per data frame */
    int N;                    /* bits per data frame */
    int b;                    /* bits in a high mapping frame */
    int r;                    /* high mapping frames per data frame */
    int K;                    /* shell mapping bits */
    int q;                    /* uncoded bits per 2D symbol */
    int M;                    /* rings */
    int L;                    /* points in the 2D constellation */
    int w;                    /* 2^w is the precoder's quantum (9-29) */
} v34_frame_t;

bool v34_frame_params(const v34_data_params_t *p, v34_frame_t *f);

/* Frame switching and auxiliary multiplexing patterns as Table 8 and 9 show
 * them, left-most bit the first mapping frame. For tests and logs. */
unsigned v34_swp(const v34_frame_t *f);
unsigned v34_amp(const v34_frame_t *f);

/* The scrambler of clause 7, which V.32 shares: 1 + x^-18 + x^-23 for the
 * calling modem (tap 18), 1 + x^-5 + x^-23 for the answering one (tap 5). */
int v34_scramble(uint32_t *reg, int tap, int in);
int v34_descramble(uint32_t *reg, int tap, int in);

/* ------------------------------------------------------------ shell map */

typedef struct
{
    int M;
    uint64_t g2[64];
    uint64_t g4[128];
    uint64_t g8[256];
    uint64_t z8[257];
} v34_shell_t;

void v34_shell_init(v34_shell_t *s, int M);
void v34_shell_map(const v34_shell_t *s, uint32_t r0, int m[8]);
/* The inverse, for the receiver. False if the ring indices are not ones the
 * mapper could have produced from K bits. */
bool v34_shell_unmap(const v34_shell_t *s, const int m[8], int K, uint32_t *r0);

/* -------------------------------------------------------------- encoder */

typedef int (*v34_get_bit_t)(void *user);

typedef struct
{
    v34_data_params_t p;
    v34_frame_t f;
    v34_shell_t shell;
    int scr_tap;
    uint32_t scr;
    long long i;              /* mapping frame since the start of B1 */
    int z;                    /* differential encoder, Z(m - 1) */
    int state;                /* convolutional encoder */
    int y0;                   /* Y0(m) */
    int32_t xh[3][2];         /* precoder delay line, x(n-1) first, 9.7 fixed point */
    int32_t pq[2];            /* p(n), 9.7 */
    int32_t c[2];             /* c(n), integers */
    double ex;                /* mean |x|^2, for the non-linear encoder */
    bool b1;                  /* sending B1: every data bit a one */
} v34_enc_t;

void v34_enc_init(v34_enc_t *e, const v34_data_params_t *p, int scr_tap);

/* Sets ex for the non-linear encoder (9-35), which needs the average energy
 * of the precoded signal before it can run. v34_data_energy() measures it. */
void v34_enc_set_energy(v34_enc_t *e, double ex);

/* One mapping frame: eight 2D symbols, x'(n) of Figure 4. get_bit may be
 * NULL, which sends ones - which is B1, if the encoder is fresh. aux_bit
 * likewise, which sends zeros. */
void v34_enc_frame(v34_enc_t *e, v34_get_bit_t get_bit, void *user, v34_get_bit_t aux_bit, void *aux_user,
                   v34_cf_t out[8]);

/* The mean energy of x(n) and of x'(n), for these parameters, measured by
 * running the encoder over random data. The transmitter scales by the second
 * so that data goes out at the power of the training that preceded it (Note
 * to 10.1.3); the receiver needs it for the far end's signal. */
void v34_data_energy(const v34_data_params_t *p, double *ex, double *exn);

/* -------------------------------------------------------------- decoder */

/* Viterbi decoder for any of the three codes. It assumes what our MP asks
 * for besides: no precoding and no non-linear encoding. */
#define V34_VIT_DEPTH 48

typedef void (*v34_put_bit_t)(void *user, int bit);

typedef struct
{
    v34_data_params_t p;
    v34_frame_t f;
    v34_shell_t shell;
    int dscr_tap;
    uint32_t dscr;

    /* Viterbi */
    int states;
    float pm[64];
    uint8_t prev[V34_VIT_DEPTH][64];
    uint8_t pair[V34_VIT_DEPTH][64];  /* s(2m) << 3 | s(2m+1) */
    v34_cf_t rx[V34_VIT_DEPTH][2];
    int vpos;
    long long m_in;           /* 4D symbols into the decoder */
    v34_cf_t half;            /* the first 2D symbol of a 4D pair */
    bool have_half;

    /* Demapping, behind the decoder */
    long long m_out;          /* 4D symbols out of it */
    int zprev;
    int ring[8];
    int ibits[4];
    int qbits[8];
    float err;                /* running mean squared error against the decisions */
    unsigned bad_frames;      /* ring indices the shell mapper could not have made */
    bool deliver;
    v34_put_bit_t put_bit;
    void *user;
} v34_dec_t;

void v34_dec_init(v34_dec_t *d, const v34_data_params_t *p, int dscr_tap, v34_put_bit_t put_bit, void *user);
/* One 2D symbol, in Figure 5 units. */
void v34_dec_symbol(v34_dec_t *d, v34_cf_t y);
/* The nearest point of the constellation in use: a tentative decision, for
 * the equaliser and the phase loop to work from ahead of the decoder. */
v34_cf_t v34_dec_slice(const v34_dec_t *d, v34_cf_t y);

/* V0(m) for 4D symbol m counted from the start of B1 (9.6.3, Table 12). */
int v34_v0(const v34_frame_t *f, long long m);

#endif /* DATAMODEM_V34_CODEC_H */
