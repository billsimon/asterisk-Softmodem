/* ITU-T V.32: 9600 and 4800 bit/s, full duplex, on one pair of wires - and
 * V.32 bis, which takes the same modem to 14 400.
 *
 * Both directions occupy the same band - a 2400 baud QAM carrier at 1800 Hz -
 * so each modem has to subtract its own echo from what it hears before it
 * can demodulate the other one. That echo canceller, and the start-up
 * procedure that measures the round trip and trains it, are what separate
 * V.32 from the half-duplex fax modems that share its constellation.
 *
 * spandsp has no V.32, so this is a complete implementation of its own:
 * transmitter, receiver, the 8-state trellis code, the start-up and retrain
 * sequences of section 5, and the echo canceller. It knows nothing about
 * spandsp or pjmedia - samples in, samples out, bits through callbacks - in
 * the same shape as spandsp's data pumps so that modem.c can drive it the
 * same way.
 *
 * Implemented: 9600 bit/s trellis coded (32 points) and nonredundant (16
 * points), 4800 bit/s (4 points), and the full rate negotiation between them;
 * and with v32bis set, V.32 bis's 14 400, 12 000 and 7200 bit/s (128, 64 and
 * 16 points, all trellis coded) and its rate renegotiation, answering the far
 * end's requests and making its own when reception degrades. A V.32 bis
 * modem works to V.32's rules with a far end that is only V.32, as the
 * Recommendation requires. Not implemented: 2400 bit/s, which V.32 leaves for
 * further study. Annex A's automode fallback to V.22bis is modem.c's
 * business, as the other steps down are. */
#ifndef DATAMODEM_V32_H
#define DATAMODEM_V32_H

#include <stdbool.h>
#include <stdint.h>

typedef struct dm_v32 dm_v32_t;

typedef enum
{
    DM_V32_TRAINED = 0,      /* start-up or retrain complete; data is crossing */
    DM_V32_RETRAINING,       /* a retrain has begun, from either end; data is paused */
    DM_V32_CARRIER_DOWN,     /* the far end's signal has gone; a retrain is being tried */
    DM_V32_TRAINING_FAILED,  /* a stage of the handshake timed out; starting it over */
    DM_V32_CLEARDOWN,        /* the rate exchange agreed on no rate at all */
    DM_V32_RATE_CHANGED      /* V.32 bis renegotiated the rate; data is crossing again */
} dm_v32_event_t;

typedef struct
{
    bool calling;
    bool v32bis;              /* a V.32 bis modem; otherwise V.32 */
    int max_rate;             /* 14400, 12000, 9600, 7200 or 4800; V.32 stops at 9600 and skips 7200 */
    bool trellis;             /* V.32: offer the trellis-coded 9600; the 16-point one is always there */
    bool listen_first;        /* calling: stay silent until dm_v32_start(), or until AC is heard */
    float tx_power;           /* dBm0 */
    bool deaf_to_renegotiation; /* test hook: act like a far end that ignores 8/V.32 bis */
    const char *tag;          /* for logs */

    /* Called on whichever thread drives dm_v32_tx / dm_v32_rx. get_bit is
     * only asked once data may flow; before that the transmitter sends the
     * scrambled ones the procedure calls for. */
    int (*get_bit)(void *user);
    void (*put_bit)(void *user, int bit);
    void (*event)(void *user, dm_v32_event_t ev);
    void *user;
} dm_v32_params_t;

typedef struct
{
    const char *stage;        /* where the handshake is, for diagnostics */
    int bit_rate;             /* 0 until a rate has been agreed */
    bool trellis;
    float snr_db;             /* receiver's estimate, from the equaliser error */
    float rx_power;           /* dBm0, after echo cancellation */
    int round_trip_ms;        /* as measured during start-up, -1 if not yet */
    bool echo_canceller;      /* an echo was found and is being cancelled */
    float echo_delay_ms;
    float echo_return_loss_db; /* how far below our own signal the echo came back */
    float echo_cancelled_db;  /* and how much further the canceller took it down */
    unsigned retrains;
    bool v32bis;              /* both ends are V.32 bis; false when either is only V.32 */
    unsigned renegotiations;  /* rate changes without a retrain (V.32 bis) */
} dm_v32_stats_t;

dm_v32_t *dm_v32_create(const dm_v32_params_t *params);
void dm_v32_free(dm_v32_t *v);

/* 16-bit linear, 8000 Hz. dm_v32_tx always fills the buffer. The receiver
 * keeps the transmitter's sample count as its time reference - the echo
 * canceller and the round-trip measurement both depend on it - so feed the
 * two the same number of samples, transmit first. */
int dm_v32_tx(dm_v32_t *v, int16_t *amp, int len);
void dm_v32_rx(dm_v32_t *v, const int16_t *amp, int len);

/* Audio that never arrived. Keeps the clocks running without letting the
 * adaptive parts learn from the gap. */
void dm_v32_rx_fillin(dm_v32_t *v, int len);

/* A calling modem created with listen_first sends nothing until it is told
 * the answer tone has been heard (5.4.1) - or until it hears AC itself,
 * which an answering modem that skips the answer tone sends straight away
 * (5.1, and 5.4 Note 1). dm_v32_started() says which has happened. */
void dm_v32_start(dm_v32_t *v);
bool dm_v32_started(const dm_v32_t *v);

/* The far end has answered in V.32's terms - AC heard by a calling modem, AA
 * by an answering one - so it is a V.32 modem and the handshake is under
 * way. Until then it may be something else altogether. */
bool dm_v32_engaged(const dm_v32_t *v);

int dm_v32_bit_rate(const dm_v32_t *v);

/* V.32 bis, in data: ask the far end to change to rate, or the best below it
 * that both ends will take (8/V.32 bis). False if this is not a V.32 bis
 * call, or not in data. DM_V32_RATE_CHANGED says when it has happened. */
bool dm_v32_renegotiate(dm_v32_t *v, int rate);
float dm_v32_rx_power(const dm_v32_t *v);
void dm_v32_stats(const dm_v32_t *v, dm_v32_stats_t *out);

#endif /* DATAMODEM_V32_H */
