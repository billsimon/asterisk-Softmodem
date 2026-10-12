/*
 * Softmodem for Asterisk: everything between a channel's audio and a TCP
 * socket that does not need Asterisk to run.
 *
 * The data pump is datamodem's (engine/, see engine/UPSTREAM): V.34, V.32bis,
 * V.32, V.22bis, V.22, V.21, Bell 103 and V.23, with V.42/V.42bis, an
 * answering modem that offers each in turn to a caller that has not said
 * what it is, and a calling modem that listens to what answers and steps
 * down to it. This file wraps it in what an Asterisk application needs:
 *
 *   - options, in the vocabulary of the dialplan application;
 *   - a clock, because Asterisk hands us frames as they arrive rather than
 *     on a timer: we transmit one sample for every one received, account
 *     for lost packets from their sequence numbers, and keep transmitting on
 *     our own clock if the far end goes quiet (see sm_clock_t);
 *   - the byte pump between the modem and the socket, with the old
 *     application's Btx extras (ULM header, NUL byte, bit order); and
 *   - when to give up: no training, a lost carrier, a link that will not
 *     hold, the server going away.
 *
 * app_softmodem.c is the thin layer that ties this to a channel; the tests
 * drive it directly, two of them back to back.
 */
#ifndef SOFTMODEM_BRIDGE_H
#define SOFTMODEM_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "datamodem/log.h"
#include "datamodem/modem.h"

#define SM_SAMPLE_RATE 8000

/* ------------------------------------------------------------------ logging */

typedef void (*sm_log_sink_t)(dm_log_level_t level, const char *component, const char *msg);
typedef bool (*sm_log_enabled_t)(dm_log_level_t level);

/* Where the engine's log lines go, and which levels are worth formatting.
 * NULL for either restores the default: stderr, at sm_log_set_level(). */
void sm_log_set_sink(sm_log_sink_t sink, sm_log_enabled_t enabled);
void sm_log_set_level(dm_log_level_t level);

/* Once per process, before the first modem: hooks spandsp's own logging
 * into the sink and checks the spandsp it is linked against. */
void sm_init(dm_log_level_t spandsp_level);

/* ------------------------------------------------------------------ options */

typedef enum {
	SM_ROLE_AUTO = 0,     /* answer, unless the channel is one Asterisk placed */
	SM_ROLE_CALLING,
	SM_ROLE_ANSWERING
} sm_role_t;

typedef struct {
	sm_role_t role;
	char modulation[16];      /* the fastest to try: v34 ... bell103, or v23 */
	bool step_down;           /* fall back to slower modulations the far end wants */
	int bit_rate;             /* ceiling; 0 = the most the modulation can do */
	char v42[16];             /* off | detect | require */
	bool v42bis;
	int data_bits;            /* 5..8 */
	char parity[8];           /* none | even | odd */
	int stop_bits;            /* 1 | 2 */
	int tx_power;             /* dBm0 */
	char guard_tone[8];       /* none | 550 | 1800: a V.22bis answerer's */
	int train_timeout_s;      /* hang up if no carrier by then */
	int max_retrains;         /* hang up on a link that will not hold; 0 = never */
	bool msb_first;           /* reverse the bit order of each character (async only) */
	bool ulm_header;          /* tell the server the line speeds first (Btx) */
	bool send_null;           /* send a NUL down the line on connecting (Btx) */
	int drain_timeout_s;      /* once the server has closed: most to wait for the line to drain */
} sm_options_t;

/* datamodem's defaults: V.34 stepping down to whatever answers or calls,
 * V.42 with V.42bis when the far end will, 8N1, -13 dBm0. */
void sm_options_defaults(sm_options_t *o);

/* Applies one option of the dialplan application - its letter and, for those
 * that take one, its argument - in the same words the documentation uses.
 * False, with err filled, for a bad value. */
bool sm_options_apply(sm_options_t *o, char letter, const char *arg, char *err, size_t err_len);

/* Fills in the engine's parameters for a role that is now known. */
void sm_options_to_params(const sm_options_t *o, bool calling, const char *tag, dm_modem_params_t *out);

/* Checks everything the engine would refuse, before a call is touched. */
bool sm_options_check(const sm_options_t *o, char *err, size_t err_len);

/* ------------------------------------------------------------------- bridge */

typedef enum {
	SM_RUNNING = 0,
	SM_END_NO_CARRIER,      /* the modems never trained */
	SM_END_CARRIER_LOST,    /* trained, then the carrier went and stayed gone */
	SM_END_UNSTABLE,        /* retrained too often to be worth keeping */
	SM_END_SERVER_CLOSED,   /* the server hung up, and what it said has gone out */
	SM_END_SOCKET_ERROR,    /* the connection to the server failed mid-call */
	SM_END_HANGUP,          /* the channel went away (set by the host) */
	SM_END_ERROR            /* something of ours broke */
} sm_end_t;

/* As the dialplan sees it, in ${SOFTMODEM_RESULT}. */
const char *sm_end_name(sm_end_t end);

typedef struct sm_bridge sm_bridge_t;

/* sock is a connected, non-blocking stream socket to the server, which the
 * bridge now owns. The tag names this call in the log. */
sm_bridge_t *sm_bridge_create(const sm_options_t *o, bool calling, int sock, const char *tag);
void sm_bridge_destroy(sm_bridge_t *b);

/* n samples of the far end in, n samples of ours out - ours first, which is
 * the order V.32 and V.34 time their echo cancellers by. */
void sm_bridge_audio(sm_bridge_t *b, const int16_t *in, int16_t *out, int n);

/* n samples of the far end that never arrived; n of ours still go out. */
void sm_bridge_missing(sm_bridge_t *b, int16_t *out, int n);

/* Moves bytes between the modem and the socket and decides whether the call
 * goes on. Call it after every frame and whenever the socket is readable;
 * once it returns anything but SM_RUNNING the call is over. */
sm_end_t sm_bridge_service(sm_bridge_t *b);

/* The socket the bridge owns, for the host to wait on; -1 when none. */
int sm_bridge_fd(const sm_bridge_t *b);

/* How the call ended, once sm_bridge_service has said; the host records
 * SM_END_HANGUP here itself when the channel goes first. */
sm_end_t sm_bridge_end(const sm_bridge_t *b);
void sm_bridge_set_end(sm_bridge_t *b, sm_end_t end);

typedef struct {
	bool connected;             /* the link came up at some point */
	bool data_ready;            /* and is carrying data now */
	const char *modulation;     /* what is running, after any step down */
	int rx_rate;                /* bit/s, each way */
	int tx_rate;
	const char *protocol;       /* async | V.42 | V.42/V.42bis ... */
	uint64_t bytes_to_line;     /* characters, before any compression */
	uint64_t bytes_from_line;
	unsigned retrains;
	int64_t connect_ms;         /* how long training took; 0 if it never did */
} sm_status_t;

void sm_bridge_status(sm_bridge_t *b, sm_status_t *out);

/* ------------------------------------------------------------------- clock
 *
 * Asterisk delivers the far end's audio as it arrives - every 20 ms while
 * RTP is flowing, and not at all when it is not. The modem wants a sample
 * out for every sample in, so:
 *
 *   - each frame received is answered with a frame of ours, the same length,
 *     so our transmit clock is the far end's and never drifts from it;
 *   - packets missing from the sequence are given to the modem as missing,
 *     and answered as if they had arrived;
 *   - when nothing arrives for SM_STALL_MS, our own clock takes over and the
 *     modem is told that what it is not hearing is missing - so the far end
 *     goes on hearing a carrier through a stall of ours - and once audio
 *     comes back, as much of it as was already covered that way is dropped,
 *     as a jitter buffer drops a late packet, so that the two directions
 *     stay sample for sample in step.
 */
#define SM_STALL_MS 60
#define SM_MAX_GAP_SAMPLES SM_SAMPLE_RATE /* never fill more than a second at once */

typedef struct {
	bool have_seq;
	int last_seq;
	int frame_samples;        /* the size of the last frame, which the gaps are made of */
	int64_t last_audio_ms;    /* when our output last caught up with real time */
	long covered;             /* samples already sent on our own clock, owed by late audio */
} sm_clock_t;

void sm_clock_init(sm_clock_t *c, int64_t now_ms);

/* A frame of n samples, with its RTP sequence number or -1 if it has none.
 * Writes what to transmit into out (room for out_max samples, which should
 * be at least n + SM_MAX_GAP_SAMPLES) and returns how many. */
int sm_clock_frame(sm_clock_t *c, sm_bridge_t *b, const int16_t *in, int n, int seq, int64_t now_ms,
		   int16_t *out, int out_max);

/* Nothing arrived: if the far end has been quiet for SM_STALL_MS, what to
 * transmit to keep our side going, else 0. */
int sm_clock_idle(sm_clock_t *c, sm_bridge_t *b, int64_t now_ms, int16_t *out, int out_max);

/* ------------------------------------------------------------------ network */

/* Connects to host:port (a name, IPv4 or IPv6) within timeout_ms, and returns
 * a non-blocking socket, or -1 with err filled. */
int sm_connect_tcp(const char *host, int port, int timeout_ms, char *err, size_t err_len);

#endif /* SOFTMODEM_BRIDGE_H */
