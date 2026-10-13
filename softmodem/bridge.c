/*
 * Softmodem for Asterisk: the part that does not need Asterisk. See bridge.h.
 */
#include "softmodem/bridge.h"
#include "datamodem/config.h"
#include "datamodem/util.h"

#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0 /* macOS: SO_NOSIGPIPE on the socket instead */
#endif

/* datamodem's own defaults for the parts of the handshake there is no
 * option for here; see datamodem's config.c for why each is what it is. */
#define SM_ANSWER_TONE_MS 3300
#define SM_ANSWER_WAIT_S 5
#define SM_ANSWER_TAIL_MS 3300
#define SM_V42_TIMEOUT_S 10
#define SM_PATH_DELAY_MS 300
#define SM_V42BIS_DICT 2048
#define SM_V42BIS_MAX_STRING 32

/* What the server has said and the line has not yet taken. Small, so that
 * the server is held off by TCP rather than by us: a caller's ^C at a BBS
 * should stop the output in seconds, not after everything already buffered
 * has crawled out at 300 bps. When the server goes - closing, or resetting
 * the connection - what it sent before is still read out of the socket, and
 * still goes down the line before the call is cleared. */
#define SM_FROM_SOCK 4096

void sm_init(dm_log_level_t spandsp_level)
{
	static dm_config_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	cfg.log_level = spandsp_level;
	cfg.spandsp_log_level = -1;
	dm_modem_init_logging(&cfg);
}

/* ------------------------------------------------------------------ options */

void sm_options_defaults(sm_options_t *o)
{
	memset(o, 0, sizeof(*o));
	o->role = SM_ROLE_AUTO;
	snprintf(o->modulation, sizeof(o->modulation), "v34");
	o->step_down = true;
	snprintf(o->v42, sizeof(o->v42), "detect");
	o->v42bis = true;
	o->data_bits = 8;
	snprintf(o->parity, sizeof(o->parity), "none");
	o->stop_bits = 1;
	o->tx_power = -13;
	snprintf(o->guard_tone, sizeof(o->guard_tone), "none");
	o->train_timeout_s = 45;
	o->max_retrains = 4;
	o->drain_timeout_s = 30;
}

/* "V22bis", "v.22bis", "Bell103" and so on, to the engine's names. */
static bool canonical_modulation(const char *in, char *out, size_t out_len)
{
	static const char *const names[] = { "v34", "v32bis", "v32", "v22bis", "v22", "v21", "bell103", "v23" };
	char flat[16];
	size_t n = 0;

	for (const char *p = in; *p != '\0' && n < sizeof(flat) - 1; p++) {
		if (*p == '.' || *p == '-' || *p == ' ')
			continue;
		flat[n++] = (char) (*p >= 'A' && *p <= 'Z' ? *p - 'A' + 'a' : *p);
	}
	flat[n] = '\0';
	if (strcmp(flat, "auto") == 0) {
		snprintf(out, out_len, "v34");
		return true;
	}
	for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
		if (strcmp(flat, names[i]) == 0) {
			snprintf(out, out_len, "%s", names[i]);
			return true;
		}
	}
	return false;
}

static bool parse_int(const char *arg, int lo, int hi, int *out)
{
	char *end;
	long v;

	if (arg == NULL || *arg == '\0')
		return false;
	errno = 0;
	v = strtol(arg, &end, 10);
	if (errno != 0 || *end != '\0' || v < lo || v > hi)
		return false;
	*out = (int) v;
	return true;
}

bool sm_options_apply(sm_options_t *o, char letter, const char *arg, char *err, size_t err_len)
{
	const char *a = arg ? arg : "";

	switch (letter) {
	case 'a':
		o->role = SM_ROLE_ANSWERING;
		return true;
	case 'c':
		o->role = SM_ROLE_CALLING;
		return true;
	case 'v':
		if (!canonical_modulation(a, o->modulation, sizeof(o->modulation))) {
			snprintf(err, err_len, "v(%s): the modulations are V34, V32bis, V32, V22bis, V22, V21, "
					       "Bell103 and V23", a);
			return false;
		}
		return true;
	case 'f':
		o->step_down = false;
		return true;
	case 'b':
		if (!parse_int(a, 300, 33600, &o->bit_rate)) {
			snprintf(err, err_len, "b(%s): a bit rate is 300 to 33600", a);
			return false;
		}
		return true;
	case 'e':
		if (strcasecmp(a, "off") == 0 || strcasecmp(a, "none") == 0 || strcasecmp(a, "no") == 0)
			snprintf(o->v42, sizeof(o->v42), "off");
		else if (strcasecmp(a, "detect") == 0 || strcasecmp(a, "auto") == 0 || strcasecmp(a, "on") == 0 ||
			 *a == '\0')
			snprintf(o->v42, sizeof(o->v42), "detect");
		else if (strcasecmp(a, "require") == 0 || strcasecmp(a, "required") == 0)
			snprintf(o->v42, sizeof(o->v42), "require");
		else {
			snprintf(err, err_len, "e(%s): V.42 is off, detect or require", a);
			return false;
		}
		return true;
	case 'k':
		o->v42bis = false;
		return true;
	case 'd':
		if (!parse_int(a, 5, 8, &o->data_bits)) {
			snprintf(err, err_len, "d(%s): only 5-8 data bits are supported", a);
			return false;
		}
		return true;
	case 'p':
		if (strcasecmp(a, "none") == 0 || strcasecmp(a, "n") == 0)
			snprintf(o->parity, sizeof(o->parity), "none");
		else if (strcasecmp(a, "even") == 0 || strcasecmp(a, "e") == 0)
			snprintf(o->parity, sizeof(o->parity), "even");
		else if (strcasecmp(a, "odd") == 0 || strcasecmp(a, "o") == 0)
			snprintf(o->parity, sizeof(o->parity), "odd");
		else {
			snprintf(err, err_len, "p(%s): parity is none, even or odd", a);
			return false;
		}
		return true;
	case 's':
		if (!parse_int(a, 1, 2, &o->stop_bits)) {
			snprintf(err, err_len, "s(%s): only 1-2 stop bits are supported", a);
			return false;
		}
		return true;
	case 't': {
		char *end;
		double v = strtod(a, &end);

		if (*a == '\0' || *end != '\0' || v < -60.0 || v > 0.0) {
			snprintf(err, err_len, "t(%s): the transmit level is in dBm0, -60 to 0", a);
			return false;
		}
		o->tx_power = (int) (v < 0 ? v - 0.5 : v + 0.5);
		return true;
	}
	case 'r':
		/* The old FSK receiver's cutoff. The engine sets its own thresholds,
		 * per modulation; accepted so that old dialplans still run. */
		return true;
	case 'g':
		if (strcasecmp(a, "none") == 0 || strcmp(a, "0") == 0 || strcmp(a, "550") == 0 ||
		    strcmp(a, "1800") == 0) {
			snprintf(o->guard_tone, sizeof(o->guard_tone), "%s", strcmp(a, "0") == 0 ? "none" : a);
			return true;
		}
		snprintf(err, err_len, "g(%s): the guard tone is none, 550 or 1800", a);
		return false;
	case 'w':
		if (!parse_int(a, 5, 600, &o->train_timeout_s)) {
			snprintf(err, err_len, "w(%s): the training timeout is 5 to 600 seconds", a);
			return false;
		}
		return true;
	case 'l':
		o->msb_first = false;
		return true;
	case 'm':
		o->msb_first = true;
		return true;
	case 'u':
		o->ulm_header = true;
		return true;
	case 'n':
		o->send_null = true;
		return true;
	}
	snprintf(err, err_len, "unknown option '%c'", letter);
	return false;
}

void sm_options_to_params(const sm_options_t *o, bool calling, const char *tag, dm_modem_params_t *out)
{
	memset(out, 0, sizeof(*out));
	out->calling = calling;
	out->modulation = o->modulation;
	out->step_down = o->step_down;
	out->bit_rate = o->bit_rate;
	out->guard_tone = o->guard_tone;
	out->data_bits = o->data_bits;
	out->parity = o->parity;
	out->stop_bits = o->stop_bits;
	out->v14 = true;
	out->tx_power = o->tx_power;
	out->answer_tone_ms = SM_ANSWER_TONE_MS;
	out->answer_wait_s = SM_ANSWER_WAIT_S;
	out->answer_tail_ms = SM_ANSWER_TAIL_MS;
	out->v42 = o->v42;
	out->v42_timeout_s = SM_V42_TIMEOUT_S;
	out->path_delay_ms = SM_PATH_DELAY_MS;
	out->v42bis = o->v42bis && strcmp(o->v42, "off") != 0;
	out->v42bis_dict = SM_V42BIS_DICT;
	out->v42bis_max_string = SM_V42BIS_MAX_STRING;
	out->calling_tone = false;
	out->tag = tag;
}

bool sm_options_check(const sm_options_t *o, char *err, size_t err_len)
{
	dm_modem_params_t p;

	/* Whichever role it turns out to be. */
	sm_options_to_params(o, true, "check", &p);
	if (!dm_modem_params_check(&p, err, err_len))
		return false;
	sm_options_to_params(o, false, "check", &p);
	return dm_modem_params_check(&p, err, err_len);
}

/* ------------------------------------------------------------------- bridge */

struct sm_bridge {
	sm_options_t opts;
	bool calling;
	char tag[64];
	int sock;
	dm_modem_t *modem;

	int64_t started_ms;
	int64_t connected_ms;
	bool announced;        /* the link came up and the server has been told */

	/* Off the line, on its way to the server. */
	unsigned char to_sock[4096];
	size_t to_sock_off;
	size_t to_sock_len;

	/* From the server, on its way to the line. */
	unsigned char *from_sock;
	size_t from_sock_off;
	size_t from_sock_len;

	bool sock_eof;         /* the server has gone: closed, reset or failed */
	bool sock_failed;      /* ... and not with a clean close */
	int64_t emptied_ms;    /* when what it had said had all gone to the line */
	sm_end_t end;
};

const char *sm_end_name(sm_end_t end)
{
	switch (end) {
	case SM_RUNNING:
		return "RUNNING";
	case SM_END_NO_CARRIER:
		return "NOCARRIER";
	case SM_END_CARRIER_LOST:
		return "CARRIERLOST";
	case SM_END_UNSTABLE:
		return "UNSTABLE";
	case SM_END_SERVER_CLOSED:
		return "SERVERCLOSED";
	case SM_END_SOCKET_ERROR:
		return "SOCKETERROR";
	case SM_END_HANGUP:
		return "HANGUP";
	case SM_END_ERROR:
		break;
	}
	return "ERROR";
}

sm_bridge_t *sm_bridge_create(const sm_options_t *o, bool calling, int sock, const char *tag)
{
	sm_bridge_t *b;
	dm_modem_params_t p;

	b = calloc(1, sizeof(*b));
	if (b == NULL)
		return NULL;
	b->from_sock = malloc(SM_FROM_SOCK);
	if (b->from_sock == NULL) {
		free(b);
		return NULL;
	}
	b->opts = *o;
	b->calling = calling;
	b->sock = sock;
	snprintf(b->tag, sizeof(b->tag), "%s", tag ? tag : (calling ? "out" : "in"));
	/* The tag ends up in file names under DATAMODEM_RECORD. */
	for (char *c = b->tag; *c != '\0'; c++)
		if (*c == '/' || *c == ' ')
			*c = '_';

	sm_options_to_params(&b->opts, calling, b->tag, &p);
	b->modem = dm_modem_create(&p);
	if (b->modem == NULL) {
		free(b->from_sock);
		free(b);
		return NULL;
	}
	/* The call is answered by the time there is a bridge, which is the
	 * moment the engine waits for before anything - tones, timers - starts. */
	dm_modem_arm(b->modem);
	b->started_ms = dm_now_ms();
	return b;
}

void sm_bridge_destroy(sm_bridge_t *b)
{
	if (b == NULL)
		return;
	dm_modem_destroy(b->modem);
	if (b->sock >= 0)
		close(b->sock);
	free(b->from_sock);
	free(b);
}

int sm_bridge_fd(const sm_bridge_t *b)
{
	return b->sock;
}

sm_end_t sm_bridge_end(const sm_bridge_t *b)
{
	return b->end;
}

void sm_bridge_set_end(sm_bridge_t *b, sm_end_t end)
{
	if (b->end == SM_RUNNING)
		b->end = end;
}

void sm_bridge_audio(sm_bridge_t *b, const int16_t *in, int16_t *out, int n)
{
	int got = dm_modem_tx(b->modem, out, n);

	if (got < n)
		memset(out + (got > 0 ? got : 0), 0, (size_t) (n - (got > 0 ? got : 0)) * sizeof(int16_t));
	dm_modem_rx(b->modem, in, n);
}

void sm_bridge_missing(sm_bridge_t *b, int16_t *out, int n)
{
	int got = dm_modem_tx(b->modem, out, n);

	if (got < n)
		memset(out + (got > 0 ? got : 0), 0, (size_t) (n - (got > 0 ? got : 0)) * sizeof(int16_t));
	dm_modem_rx_missing(b->modem, n);
}

/* The rates each way: the engine reports what it receives at, and for V.34
 * what it sends at too; V.23 is lopsided by role. */
static void rates(const sm_bridge_t *b, const dm_modem_status_t *st, int *rx, int *tx)
{
	*rx = st->bit_rate;
	*tx = st->bit_rate;
	if (strcmp(st->modulation, "v34") == 0 && st->tx_bit_rate > 0)
		*tx = st->tx_bit_rate;
	else if (strcmp(st->modulation, "v23") == 0) {
		*rx = b->calling ? 1200 : 75;
		*tx = b->calling ? 75 : 1200;
	}
}

void sm_bridge_status(sm_bridge_t *b, sm_status_t *out)
{
	dm_modem_status_t st;

	dm_modem_status(b->modem, &st);
	memset(out, 0, sizeof(*out));
	out->connected = b->announced;
	out->data_ready = st.data_ready;
	out->modulation = st.modulation;
	rates(b, &st, &out->rx_rate, &out->tx_rate);
	out->protocol = st.protocol ? st.protocol : "async";
	out->bytes_to_line = st.bytes_tx;
	out->bytes_from_line = st.bytes_rx;
	out->retrains = st.retrains;
	out->connect_ms = b->connected_ms ? b->connected_ms - b->started_ms : 0;
}

/* MSB-first characters are LSB-first ones with their data bits reversed; the
 * engine frames LSB-first, as every standard modem does. The same reversal
 * undoes itself, so it serves both directions. */
static void reverse_bits(unsigned char *buf, size_t n, int data_bits)
{
	for (size_t i = 0; i < n; i++) {
		unsigned in = buf[i];
		unsigned out = 0;

		for (int bit = 0; bit < data_bits; bit++)
			if (in & (1u << bit))
				out |= 1u << (data_bits - 1 - bit);
		buf[i] = (unsigned char) out;
	}
}

static bool queue_to_sock(sm_bridge_t *b, const void *data, size_t len)
{
	if (b->to_sock_off > 0 && b->to_sock_len > 0)
		memmove(b->to_sock, b->to_sock + b->to_sock_off, b->to_sock_len);
	b->to_sock_off = 0;
	if (len > sizeof(b->to_sock) - b->to_sock_len)
		return false;
	memcpy(b->to_sock + b->to_sock_len, data, len);
	b->to_sock_len += len;
	return true;
}

/* The server has gone. What it said before it went still goes down the line;
 * nothing more goes to it. */
static void server_gone(sm_bridge_t *b, int err)
{
	if (b->sock_eof)
		return;
	b->sock_eof = true;
	b->sock_failed = (err != 0);
	b->to_sock_len = 0;
	b->emptied_ms = dm_now_ms();
	if (err != 0)
		DM_WARN("softmodem", "the connection to the server failed (%s); %zu bytes it sent are still to "
				     "go down the line (tag=%s)", strerror(err), b->from_sock_len, b->tag);
	else
		DM_INFO("softmodem", "the server closed the connection; %zu bytes it sent are still to go down "
				     "the line (tag=%s)", b->from_sock_len, b->tag);
}

static void flush_to_sock(sm_bridge_t *b)
{
	while (b->to_sock_len > 0 && !b->sock_eof) {
		ssize_t n = send(b->sock, b->to_sock + b->to_sock_off, b->to_sock_len, MSG_NOSIGNAL);

		if (n < 0) {
			if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
				server_gone(b, errno);
			return;
		}
		b->to_sock_off += (size_t) n;
		b->to_sock_len -= (size_t) n;
	}
	b->to_sock_off = 0;
}

/* Everything the server has sent, into our own buffer. */
static void read_from_sock(sm_bridge_t *b)
{
	while (!b->sock_eof) {
		size_t room;
		ssize_t n;

		if (b->from_sock_off > 0) {
			memmove(b->from_sock, b->from_sock + b->from_sock_off, b->from_sock_len);
			b->from_sock_off = 0;
		}
		room = SM_FROM_SOCK - b->from_sock_len;
		if (room == 0)
			return;
		n = recv(b->sock, b->from_sock + b->from_sock_len, room, 0);
		if (n > 0) {
			b->from_sock_len += (size_t) n;
			continue;
		}
		if (n == 0)
			server_gone(b, 0);
		else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
			server_gone(b, errno);
		return;
	}
}

static void announce(sm_bridge_t *b, const dm_modem_status_t *st)
{
	int rx, tx;

	b->announced = true;
	b->connected_ms = dm_now_ms();
	rates(b, st, &rx, &tx);
	DM_INFO("softmodem", "connected: %s, %d bps received and %d sent, %s, after %.1fs (tag=%s)",
		st->modulation, rx, tx, st->protocol, (double) (b->connected_ms - b->started_ms) / 1000.0, b->tag);
	/* The Ulm Relay Protocol's header: the speeds in characters a second,
	 * so the server can tell how long what it sends will take. */
	if (b->opts.ulm_header) {
		int bits = 1 + b->opts.data_bits + b->opts.stop_bits + (strcmp(b->opts.parity, "none") != 0);
		char header[96];
		int len = snprintf(header, sizeof(header), "Version: 1\r\nTXspeed: %.2f\r\nRXspeed: %.2f\r\n\r\n",
				   (double) tx / bits, (double) rx / bits);

		queue_to_sock(b, header, (size_t) len);
	}
	/* What a calling softmodem has always told its server on connecting. */
	if (b->calling)
		queue_to_sock(b, "CONNECTED\r\n", 11);
	/* The Btx centre's request for the terminal's identity, first of all.
	 * Answering on an async link, the engine holds it, and the server's
	 * first words, until the caller is listening (DM_ANSWER_HOLD_MS). */
	if (b->opts.send_null) {
		static const unsigned char nul = 0;

		dm_modem_send(b->modem, &nul, 1);
	}
}

static sm_end_t finish(sm_bridge_t *b, sm_end_t end)
{
	b->end = end;
	return end;
}

sm_end_t sm_bridge_service(sm_bridge_t *b)
{
	dm_modem_status_t st;
	int64_t now = dm_now_ms();
	bool async;

	if (b->end != SM_RUNNING)
		return b->end;

	dm_modem_status(b->modem, &st);
	if (!b->announced && st.data_ready)
		announce(b, &st);
	async = (st.protocol == NULL || strcmp(st.protocol, "async") == 0);

	/* Off the line and on to the server, no faster than it takes it; to
	 * nowhere once it has gone. */
	flush_to_sock(b);
	while (b->to_sock_len == 0) {
		unsigned char buf[2048];
		size_t got = dm_modem_recv(b->modem, buf, sizeof(buf));

		if (got == 0 || b->sock_eof)
			break;
		if (b->opts.msb_first && async)
			reverse_bits(buf, got, b->opts.data_bits);
		queue_to_sock(b, buf, got);
		flush_to_sock(b);
	}

	/* From the server, and down the line once there is a line to go down,
	 * no faster than it takes it. */
	read_from_sock(b);
	if (b->announced && st.data_ready && b->from_sock_len > 0) {
		size_t n = dm_modem_tx_space(b->modem);
		unsigned char buf[2048];

		if (n > b->from_sock_len)
			n = b->from_sock_len;
		if (n > sizeof(buf))
			n = sizeof(buf);
		if (n > 0) {
			memcpy(buf, b->from_sock + b->from_sock_off, n);
			if (b->opts.msb_first && async)
				reverse_bits(buf, n, b->opts.data_bits);
			n = dm_modem_send(b->modem, buf, n);
			b->from_sock_off += n;
			b->from_sock_len -= n;
			if (b->from_sock_len == 0)
				b->emptied_ms = now;
		}
	}

	/* The server has gone. Whatever it said last still reaches the far end
	 * before the call is cleared - which with V.42 means the line, not just
	 * the queue - and a server that gives up before the modems have trained
	 * with nothing to say ends the call there and then. */
	if (b->sock_eof) {
		sm_end_t how = b->sock_failed ? SM_END_SOCKET_ERROR : SM_END_SERVER_CLOSED;

		if (b->from_sock_len == 0 && !b->announced) {
			DM_INFO("softmodem", "the server went before the modems trained (tag=%s)", b->tag);
			return finish(b, how);
		}
		if (st.carrier_lost)
			return finish(b, how);
		if (b->from_sock_len == 0 && b->announced && dm_modem_drained(b->modem))
			return finish(b, how);
		if (b->from_sock_len == 0 && now - b->emptied_ms > (int64_t) b->opts.drain_timeout_s * 1000) {
			DM_WARN("softmodem", "the line did not drain within %ds of the server going (tag=%s)",
				b->opts.drain_timeout_s, b->tag);
			return finish(b, how);
		}
	}

	if (!b->announced && now - b->started_ms > (int64_t) b->opts.train_timeout_s * 1000) {
		DM_WARN("softmodem", "the modems did not train within %ds (phase: %s, last tried %s) (tag=%s)",
			b->opts.train_timeout_s, st.phase_text, st.modulation, b->tag);
		return finish(b, SM_END_NO_CARRIER);
	}
	if (st.carrier_lost) {
		DM_INFO("softmodem", "the far end's carrier has gone (tag=%s)", b->tag);
		return finish(b, SM_END_CARRIER_LOST);
	}
	if (b->opts.max_retrains > 0 && st.retrains >= (unsigned) b->opts.max_retrains) {
		DM_WARN("softmodem", "the link retrained %u times at %d bps and will not hold; the audio path is "
				     "too poor for %s - v() names a slower modulation to start at (tag=%s)",
			st.retrains, st.bit_rate, st.modulation, b->tag);
		return finish(b, SM_END_UNSTABLE);
	}
	return SM_RUNNING;
}

/* -------------------------------------------------------------------- clock */

void sm_clock_init(sm_clock_t *c, int64_t now_ms)
{
	memset(c, 0, sizeof(*c));
	c->frame_samples = SM_SAMPLE_RATE / 50;
	c->last_audio_ms = now_ms;
}

int sm_clock_frame(sm_clock_t *c, sm_bridge_t *b, const int16_t *in, int n, int seq, int64_t now_ms,
		   int16_t *out, int out_max)
{
	int w = 0;
	long skip;

	if (n <= 0 || n > out_max)
		return 0;

	if (seq >= 0 && c->have_seq) {
		int d = (seq - c->last_seq) & 0xffff;
		long lost;

		/* A repeat, or older than one already played: a late packet whose
		 * moment has been and gone. */
		if (d == 0 || d >= 0x8000)
			return 0;
		lost = (long) (d - 1) * c->frame_samples;
		/* A jump that long is a new stream, not a loss. */
		if (lost > SM_MAX_GAP_SAMPLES)
			lost = 0;
		/* What our own clock already covered while it was missing. */
		skip = lost < c->covered ? lost : c->covered;
		c->covered -= skip;
		lost -= skip;
		if (lost > out_max - n)
			lost = out_max - n;
		if (lost > 0) {
			sm_bridge_missing(b, out, (int) lost);
			w += (int) lost;
		}
	}
	if (seq >= 0) {
		c->have_seq = true;
		c->last_seq = seq & 0xffff;
	}
	c->frame_samples = n;

	/* Audio that arrives after our clock has already stood in for it. */
	skip = n < c->covered ? n : c->covered;
	c->covered -= skip;
	if (n - skip > 0) {
		sm_bridge_audio(b, in + skip, out + w, (int) (n - skip));
		w += (int) (n - skip);
	}
	c->last_audio_ms = now_ms;
	return w;
}

int sm_clock_idle(sm_clock_t *c, sm_bridge_t *b, int64_t now_ms, int16_t *out, int out_max)
{
	int64_t elapsed = now_ms - c->last_audio_ms;
	int frame_ms = c->frame_samples * 1000 / SM_SAMPLE_RATE;
	int64_t frames;
	int n;

	if (elapsed < SM_STALL_MS)
		return 0;
	if (frame_ms <= 0)
		frame_ms = 20;
	/* Keep a frame behind real time, so that audio which is merely late
	 * still gets its turn. */
	frames = (elapsed - frame_ms) / frame_ms;
	if (frames <= 0)
		return 0;
	if (frames * c->frame_samples > out_max)
		frames = out_max / c->frame_samples;
	n = (int) frames * c->frame_samples;
	sm_bridge_missing(b, out, n);
	c->covered += n;
	c->last_audio_ms += frames * frame_ms;
	return n;
}

/* ------------------------------------------------------------------ network */

int sm_connect_tcp(const char *host, int port, int timeout_ms, char *err, size_t err_len)
{
	struct addrinfo hints, *res, *ai;
	char service[16];
	int rc, fd = -1;

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	snprintf(service, sizeof(service), "%d", port);
	rc = getaddrinfo(host, service, &hints, &res);
	if (rc != 0) {
		snprintf(err, err_len, "cannot resolve %s: %s", host, gai_strerror(rc));
		return -1;
	}
	snprintf(err, err_len, "cannot connect to %s port %d", host, port);
	for (ai = res; ai != NULL; ai = ai->ai_next) {
		struct pollfd pfd;
		int one = 1;
		int soerr = 0;
		socklen_t len = sizeof(soerr);

		fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
		if (fd < 0)
			continue;
		fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
		fcntl(fd, F_SETFD, FD_CLOEXEC);
#ifdef SO_NOSIGPIPE
		setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif
		/* Characters typed one at a time should not wait for company. */
		setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
		if (connect(fd, ai->ai_addr, ai->ai_addrlen) == 0)
			break;
		if (errno == EINPROGRESS) {
			pfd.fd = fd;
			pfd.events = POLLOUT;
			if (poll(&pfd, 1, timeout_ms) == 1 &&
			    getsockopt(fd, SOL_SOCKET, SO_ERROR, &soerr, &len) == 0 && soerr == 0)
				break;
			if (soerr != 0)
				snprintf(err, err_len, "cannot connect to %s port %d: %s", host, port, strerror(soerr));
			else
				snprintf(err, err_len, "cannot connect to %s port %d: timed out", host, port);
		} else {
			snprintf(err, err_len, "cannot connect to %s port %d: %s", host, port, strerror(errno));
		}
		close(fd);
		fd = -1;
	}
	freeaddrinfo(res);
	return fd;
}
