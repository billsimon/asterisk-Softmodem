/* The softmodem: a data pump wrapped so that audio can be pumped at it from a
 * pjmedia thread while the main loop moves bytes to and from the terminal.
 *
 * spandsp supplies the duplex data modems it implements: V.22bis
 * (2400/1200), V.22 (1200), V.21 and Bell 103 (300), and V.23 (1200/75
 * split). V.32 (9600/4800) and V.32bis (to 14400) are our own - spandsp has
 * neither - and live in v32.c; so is V.34 (to 33600), in v34.c and the
 * v34_*.c files beside it. There is no V.90.
 *
 * V.22bis needs the sources in third_party/spandsp-v22bis to be built in;
 * the V.22bis in most packaged libspandsp builds trains and then carries
 * nothing. See that directory's README.
 *
 * V.42 error correction and V.42bis compression are available on top of any
 * of them, via --v42 and --v42bis, and are off by default. When they are off
 * the link is direct async - the same thing you got by dialling a modem with
 * &Q0 set - and a far end offering LAPM falls back to that when we do not
 * answer its ODP pattern.
 *
 * Three things about spandsp's V.42 shape this design. It hardcodes the bit
 * rate its timers are derived from, so they have to be corrected or nothing
 * interoperates; its detection window assumes a copper pair and is too short
 * for an audio path with a jitter buffer at each end; and it did not really
 * negotiate V.42bis in its XID exchange, which the vendored copy now does -
 * so what --v42bis asks for is an offer, and the link runs whatever the far
 * end agreed to. */
#ifndef DATAMODEM_MODEM_H
#define DATAMODEM_MODEM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "datamodem/config.h"

typedef struct dm_modem dm_modem_t;

typedef enum
{
    DM_PHASE_IDLE = 0,      /* created, no audio pumped yet */
    DM_PHASE_ANSWER_TONE,   /* answerer sends ANS; caller listens for it */
    DM_PHASE_TRAINING,      /* data pump running, carrier not yet good */
    DM_PHASE_DATA,          /* trained; bytes are crossing */
    DM_PHASE_DOWN           /* carrier gone */
} dm_modem_phase_t;

typedef struct
{
    bool calling;              /* true when we placed the call (originate) */
    const char *modulation;    /* v34 | v32bis | v32 | v22bis | v22 | v21 | bell103 | v23: with
                                * step_down, the fastest to try */
    bool step_down;            /* fall back to slower modulations, down to bell103 */
    int bit_rate;              /* V.34 2400 .. 33600 in steps of 2400 (a ceiling),
                                * V.32bis 14400 | 12000 | 9600 | 7200 | 4800, V.32 9600 | 4800,
                                * V.22bis 2400 | 1200; 0 = the most it can do */
    const char *guard_tone;    /* none | 550 | 1800 */
    int data_bits;             /* 5..8 */
    const char *parity;        /* none | even | odd */
    int stop_bits;             /* 1 | 2 */
    bool v14;                  /* V.14 rate adaption when framing received characters */
    int tx_power;              /* dBm0, e.g. -13 */
    int answer_tone_ms;        /* answerer: length of the 2100 Hz ANS burst, 0 = none */
    int answer_wait_s;         /* caller: seconds to listen for ANS before training anyway */
    int answer_tail_ms;        /* caller: how long to let a heard ANS finish */
    const char *v42;           /* off | detect | require */
    int v42_timeout_s;         /* detect: seconds before falling back to raw async */
    int path_delay_ms;         /* round trip through the audio path, for V.42 detection */
    bool v42bis;               /* V.42bis compression; requires V.42 */
    int v42bis_dict;           /* P1, codewords */
    int v42bis_max_string;     /* P2 */
    bool calling_tone;         /* caller: send the V.25 1300 Hz calling tone while waiting */
    const char *tag;           /* short label for logs */
} dm_modem_params_t;

typedef struct
{
    dm_modem_phase_t phase;
    const char *phase_text;
    const char *modulation;    /* what we are running: after a step down, not what we started with */
    int bit_rate;              /* negotiated rate once trained, else the offered one; V.34: receive */
    int tx_bit_rate;           /* V.34 only: what we transmit at, which may differ */
    int symbol_rate;           /* V.34 only: the far end's symbol rate, and ours */
    int tx_symbol_rate;
    int offered_rate;          /* what we asked for; below it means a down-negotiation */
    bool connected;            /* training succeeded at least once */
    bool carrier_lost;         /* carrier came up and then went away */
    bool answer_tone_seen;     /* caller heard the far end's answer tone */

    /* A carrier is not the same thing as a usable link. With V.42 asked for,
     * the data path is only settled once LAPM has come up or we have given
     * up on it, and it is that - not the carrier - that CONNECT should wait
     * for. */
    bool data_ready;
    const char *protocol;      /* "async", "V.42", "V.42/V.42bis" */
    bool lapm_up;
    bool v42_fell_back;        /* V.42 was asked for, the far end did not answer */
    unsigned frame_errors;     /* LAPM frames that arrived damaged */

    int64_t connect_ms;        /* monotonic ms at which training succeeded, 0 if never */
    uint64_t bytes_tx;         /* characters to and from the terminal */
    uint64_t bytes_rx;
    uint64_t wire_tx;          /* bytes handed to and taken from the link layer, so
                                * wire vs bytes is the compression actually achieved */
    uint64_t wire_rx;
    unsigned retrains;
    float rx_power;            /* received signal level, dBm0 */

    /* V.32 and V.32bis only; train_stage is NULL for everything else. */
    const char *train_stage;   /* where the handshake is */
    float snr_db;              /* the receiver's own estimate */
    bool line_trellis;         /* the rate in use is trellis coded */
    bool line_v32bis;          /* both ends are V.32bis; false when the far end is only V.32 */
    unsigned renegotiations;   /* V.32bis rate changes without a retrain */
    int round_trip_ms;         /* measured during start-up, -1 if not yet */
    bool echo_cancelling;      /* an echo of our own signal was found and is being removed */
    float echo_delay_ms;
    float echo_return_loss_db; /* how far below our own level it came back */
    float echo_cancelled_db;   /* and how much further the canceller took it */
    size_t tx_dropped;
    size_t rx_dropped;
} dm_modem_status_t;

/* Validates a params set without building anything, so bad settings are
 * reported before a call is placed. */
bool dm_modem_params_check(const dm_modem_params_t *params, char *err, size_t err_len);

dm_modem_t *dm_modem_create(const dm_modem_params_t *params);
void dm_modem_destroy(dm_modem_t *m);

/* Nothing happens - no answer tone, no listening for one, no training, no
 * clock of any kind - until the modem is armed. It must not be, until the
 * call is genuinely answered.
 *
 * This matters because pjmedia's conference bridge starts clocking a port
 * the moment it joins, which is well before the far end picks up. Left
 * ungated, the modem spends the ringing burning down its answer-tone wait
 * against ringback, then trains against whatever noise is on the line and
 * reports a carrier that was never there. */
void dm_modem_arm(dm_modem_t *m);
bool dm_modem_armed(dm_modem_t *m);

/* Audio pumps. 16-bit signed linear, 8000 Hz, mono. Called from the media
 * thread; all three are internally locked. dm_modem_tx returns the number of
 * samples written, which is always max_count once a pump is running. Before
 * the modem is armed they produce silence and discard input. */
int dm_modem_tx(dm_modem_t *m, int16_t *samples, int max_count);
void dm_modem_rx(dm_modem_t *m, const int16_t *samples, int count);
void dm_modem_rx_missing(dm_modem_t *m, int count); /* lost or stretched frame */

/* Byte interface, from the terminal side.
 *
 * dm_modem_send takes what fits in the transmit queue and returns that count.
 * A short return means the queue is full - at 2400 bps it drains at 240 bytes
 * per second, so a paste will hit this - and the caller must stop reading its
 * input until dm_modem_tx_space() recovers.
 *
 * dm_modem_recv drains received characters. */
size_t dm_modem_send(dm_modem_t *m, const void *data, size_t len);
size_t dm_modem_recv(dm_modem_t *m, void *out, size_t max);
size_t dm_modem_tx_pending(dm_modem_t *m);

/* True once everything handed over has actually had time to go out on the
 * line, allowing for the far end's jitter buffer. Not the same as the queue
 * being empty: the link layer takes bytes from us far faster than the line
 * carries them, and with V.42 it will swallow the whole queue at once and
 * then spend half a minute transmitting it. Anything that decides when a
 * call may be cleared has to ask this, not dm_modem_tx_pending. */
bool dm_modem_drained(dm_modem_t *m);
size_t dm_modem_tx_space(dm_modem_t *m);
size_t dm_modem_rx_pending(dm_modem_t *m);
void dm_modem_tx_clear(dm_modem_t *m);

/* Called from the media thread whenever received bytes land, so a main loop
 * blocked in poll() can be woken rather than polled at a fixed rate.
 *
 * The callback runs on the media thread with the modem locked, so it must
 * not block and must not call back into dm_modem_*. Writing a byte to a
 * non-blocking pipe is the intended use. Passing NULL is guaranteed to
 * return only once no callback is in flight, so it is safe to tear down
 * whatever `user` points at immediately afterwards. */
void dm_modem_set_wake(dm_modem_t *m, void (*wake)(void *user), void *user);

void dm_modem_status(dm_modem_t *m, dm_modem_status_t *out);

/* Carrier up. Not the same as ready to carry data - see dm_modem_data_ready. */
bool dm_modem_connected(dm_modem_t *m);

/* Carrier up and the link layer settled: either V.42 was never asked for,
 * or LAPM is established, or we gave up waiting and fell back to raw async.
 * This is what a session should wait for before it sends anything. */
bool dm_modem_data_ready(dm_modem_t *m);
dm_modem_phase_t dm_modem_phase(dm_modem_t *m);
const char *dm_modem_phase_name(dm_modem_phase_t phase);

/* Time since the far end last delivered a character. Drives the idle
 * timeout. Counts from the moment the modems trained. */
int64_t dm_modem_since_rx_ms(dm_modem_t *m);

/* Routes spandsp's own logging into our stream at a verbosity derived from
 * cfg->log_level (or cfg->spandsp_log_level when set). Call once at startup. */
void dm_modem_init_logging(const dm_config_t *cfg);

/* Builds modem params from a config, filling in the per-modulation defaults
 * that the config layer cannot know about. */
void dm_modem_params_from_config(const dm_config_t *cfg, bool calling, const char *tag,
                                 dm_modem_params_t *out);

/* Runs two modems back to back in memory: no SIP, no audio device. Pushes a
 * test pattern each way and checks it arrives. Returns a process exit code. */
int dm_modem_selftest(const dm_config_t *cfg);

#endif /* DATAMODEM_MODEM_H */
