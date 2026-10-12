/* ITU-T V.34 (02/98): duplex at up to 33 600 bit/s.
 *
 * A complete implementation of its own, as v32.c is of V.32, in the same
 * shape - samples in, samples out, bits through callbacks - so that modem.c
 * drives it the same way. What it adds to V.32 is everything that lets V.34
 * get twice the speed out of the same telephone channel:
 *
 *   Phase 1, V.8: the answer tone carries a 15 Hz modulation that says "V.8
 *   spoken here", and the two modems agree on V.34 in a 300 bit/s exchange
 *   (CM, JM, CJ) before anything else happens.
 *
 *   Phase 2, probing and ranging: each modem measures the round trip from
 *   reversals of two tones, sends the other a comb of 21 tones (L1, L2), and
 *   from what arrives works out which symbol rate (2400 to 3429 a second),
 *   carrier and pre-emphasis the line will bear, and how fast.
 *
 *   Phases 3 and 4: equaliser and echo canceller training at that symbol
 *   rate, then the MP exchange that settles the data rate in each direction,
 *   which need not be the same.
 *
 *   Data: shell mapping onto constellations of up to 1664 points, a 4D
 *   trellis code, and - on the far end's request - precoding and a
 *   non-linear encoder, in superframes of 280 ms.
 *
 * Implemented: duplex operation (clause 11) at every symbol rate and data
 * rate, both carriers, all eleven pre-emphasis filters, and in the
 * transmitter all three trellis codes, precoding, non-linear encoding and
 * shaping, whichever the far end's receiver asks for; retrains (11.5), rate
 * renegotiation (11.6) in both directions, and cleardown (11.7). Our own
 * receiver asks for the 16-state code, no precoding and no non-linear
 * encoding, so it can stay a linear equaliser and a 16-state Viterbi
 * decoder. Not implemented: half-duplex operation (clause 12), the
 * auxiliary channel and Annex A's control channel, which are offered to
 * nobody. A far end that turns out not to do V.8 is reported (DM_V34_NO_V8)
 * for the user of this to fall back to V.32 bis, which is not this file's
 * business. */
#ifndef DATAMODEM_V34_H
#define DATAMODEM_V34_H

#include <stdbool.h>
#include <stdint.h>

typedef struct dm_v34 dm_v34_t;

typedef enum
{
    DM_V34_TRAINED = 0,      /* start-up or retrain complete; data is crossing */
    DM_V34_RETRAINING,       /* a retrain has begun, from either end; data is paused */
    DM_V34_CARRIER_DOWN,     /* the far end's signal has gone; a retrain is being tried */
    DM_V34_TRAINING_FAILED,  /* a stage of the handshake timed out; trying again */
    DM_V34_CLEARDOWN,        /* the far end cleared down, or the two found no rate in common */
    DM_V34_RATE_CHANGED,     /* rates renegotiated without a retrain; data is crossing again */
    DM_V34_NOT_V34,          /* V.8 found the far end does not do V.34, or V.8 stalled */
    DM_V34_NO_V8             /* the far end does not do V.8 at all: a plain answer tone, or no CM
                                in answer to ANSam - an older modem, which may well do V.32 bis */
} dm_v34_event_t;

typedef struct
{
    bool calling;
    int max_rate;             /* 2400 ... 33600; 0 = 33600 */
    float tx_power;           /* nominal transmit power, dBm0 */
    bool lapm;                /* say in V.8 that we would like V.42 */
    unsigned symbol_rates;    /* test hook: which of the six to allow, bit 0 = 2400; 0 = all */
    int carrier;              /* test hook: what to ask the far end for, 0 = whatever probing says, 1 low, 2 high */
    int pre_emphasis;         /* test hook: likewise, -1 = whatever probing says, else 0 to 10 */
    int trellis;              /* the code our receiver asks for: 16 (0 means 16), 32 or 64 */
    bool shaping;             /* ask for expanded shaping */
    const char *tag;          /* for logs */

    /* Called on whichever thread drives dm_v34_tx / dm_v34_rx. get_bit is
     * only asked once data may flow. */
    int (*get_bit)(void *user);
    void (*put_bit)(void *user, int bit);
    void (*event)(void *user, dm_v34_event_t ev);
    void *user;
} dm_v34_params_t;

typedef struct
{
    const char *stage;        /* where the handshake is, for diagnostics */
    int tx_rate;              /* 0 until agreed */
    int rx_rate;
    int tx_symbol_rate;       /* symbols a second, rounded; 0 until agreed */
    int rx_symbol_rate;
    int tx_carrier;           /* Hz, rounded */
    int rx_carrier;
    int tx_pre_emphasis;      /* the filter the far end chose for us */
    int tx_trellis;           /* 16, 32 or 64: what the far end asked for */
    bool tx_precoding;
    bool tx_nonlinear;
    bool tx_shaping;
    float snr_db;             /* the receiver's own estimate */
    float rx_power;           /* dBm0, after echo cancellation */
    int round_trip_ms;        /* as measured in Phase 2, -1 if not yet */
    bool echo_canceller;
    float echo_delay_ms;
    float echo_return_loss_db;
    float echo_cancelled_db;
    unsigned retrains;
    unsigned renegotiations;
} dm_v34_stats_t;

dm_v34_t *dm_v34_create(const dm_v34_params_t *params);
void dm_v34_free(dm_v34_t *v);

/* 16-bit linear, 8000 Hz. As with V.32, the receiver keeps the
 * transmitter's sample count as its time reference, so feed the two the same
 * number of samples, transmit first. */
int dm_v34_tx(dm_v34_t *v, int16_t *amp, int len);
void dm_v34_rx(dm_v34_t *v, const int16_t *amp, int len);
/* Audio that never arrived. */
void dm_v34_rx_fillin(dm_v34_t *v, int len);

/* V.8 is under way: JM heard by a calling modem, CM by an answering one.
 * Until then the far end may be something older that never will be - even
 * after a calling modem has heard what it took for ANSam. */
bool dm_v34_engaged(const dm_v34_t *v);

/* Data signalling rate in each direction; 0 until data mode. */
int dm_v34_tx_rate(const dm_v34_t *v);
int dm_v34_rx_rate(const dm_v34_t *v);

/* Ask the far end to change to at most this rate in our receive direction
 * (11.6). False if not in data. DM_V34_RATE_CHANGED says when it is done. */
bool dm_v34_renegotiate(dm_v34_t *v, int rx_rate);

float dm_v34_rx_power(const dm_v34_t *v);
void dm_v34_stats(const dm_v34_t *v, dm_v34_stats_t *out);

#endif /* DATAMODEM_V34_H */
