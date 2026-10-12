/*
 * Tests for the Softmodem, without Asterisk.
 *
 * Each call puts the Softmodem's bridge - exactly what app_softmodem.c runs
 * on a channel, clock and all - on one end of a simulated RTP path, and
 * another modem on the other: the far end, set up as an older or a
 * differently configured modem would be. Both ends' TCP servers are socket
 * pairs the test reads and writes, and everything has to arrive byte for
 * byte, in both directions.
 *
 * The path is 20 ms packets with sequence numbers, as Asterisk delivers them,
 * and can be made worse: G.711 either way, delay, lost packets, and stalls
 * in which nothing arrives and then everything does at once.
 *
 *   build/softmodem-test                 everything
 *   build/softmodem-test inbound v22     only tests whose names contain all of those
 *   SOFTMODEM_TEST_VERBOSE=1 ...         the engine's log as it goes, not just on failure
 *
 * The far end is the same engine as the Softmodem, which is datamodem's; how
 * that engine fares against other manufacturers' modems is datamodem's
 * business, and its DEVNOTES. What is tested here is everything the
 * Softmodem adds: who calls and who answers, stepping down to whatever the
 * other end is, the clock that turns Asterisk's frames into a sample stream,
 * the server connection and when to hang up.
 */
#include "softmodem/bridge.h"
#include "datamodem/util.h"

#include <spandsp.h>

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define TICK 160 /* samples: one 20 ms packet */
#define TICK_MS 20
#define OUT_MAX (SM_MAX_GAP_SAMPLES + 4 * TICK)

/* ------------------------------------------------------------- reporting */

static bool g_verbose;
static int g_run, g_failed;
static char **g_filters;
static int g_nfilters;

/* Each test's log, kept to print if it fails. Per thread, for the test that
 * runs calls side by side. */
static __thread char *t_log;
static __thread size_t t_log_len, t_log_cap;

static void log_append(const char *fmt, ...)
{
	char line[4096];
	va_list ap;
	int n;

	va_start(ap, fmt);
	n = vsnprintf(line, sizeof(line), fmt, ap);
	va_end(ap);
	if (n < 0)
		return;
	if ((size_t) n >= sizeof(line))
		n = sizeof(line) - 1;
	if (t_log_len + (size_t) n + 1 > t_log_cap) {
		size_t cap = t_log_cap ? t_log_cap * 2 : 65536;

		while (cap < t_log_len + (size_t) n + 1)
			cap *= 2;
		/* Keep it bounded: a failing call's last few hundred kB say enough. */
		if (cap > (8u << 20)) {
			size_t keep = t_log_len / 2;

			memmove(t_log, t_log + t_log_len - keep, keep);
			t_log_len = keep;
			cap = t_log_cap;
		} else {
			t_log = realloc(t_log, cap);
			t_log_cap = cap;
		}
	}
	memcpy(t_log + t_log_len, line, (size_t) n);
	t_log_len += (size_t) n;
	t_log[t_log_len] = '\0';
}

static void sink(dm_log_level_t level, const char *component, const char *msg)
{
	if (g_verbose)
		fprintf(stderr, "    %-5s [%s] %s\n", dm_log_level_name(level), component, msg);
	else
		log_append("    %-5s [%s] %s\n", dm_log_level_name(level), component, msg);
}

static bool enabled(dm_log_level_t level)
{
	return level <= (g_verbose ? DM_LOG_DEBUG : DM_LOG_INFO);
}

static bool selected(const char *name)
{
	for (int i = 0; i < g_nfilters; i++)
		if (strstr(name, g_filters[i]) == NULL)
			return false;
	return true;
}

static void begin(void)
{
	t_log_len = 0;
	if (t_log)
		t_log[0] = '\0';
}

static void report(const char *name, bool ok, const char *detail)
{
	g_run++;
	if (!ok)
		g_failed++;
	printf("%s %d - %s%s%s\n", ok ? "ok" : "not ok", g_run, name, detail && *detail ? ": " : "",
	       detail ? detail : "");
	if (!ok && !g_verbose && t_log_len > 0) {
		/* The end of it, which is where it went wrong. */
		const char *from = t_log_len > 60000 ? t_log + t_log_len - 60000 : t_log;

		printf("  --- log%s ---\n%s  --- end of log ---\n", from != t_log ? " (last 60 kB)" : "", from);
	}
	fflush(stdout);
}

/* --------------------------------------------------------------- options */

/* Options as the dialplan writes them: letters, some with (arguments). */
static bool parse_options(sm_options_t *o, const char *s, char *err, size_t err_len)
{
	while (s && *s) {
		char letter = *s++;
		char arg[64] = "";

		if (*s == '(') {
			const char *end = strchr(s, ')');
			size_t n;

			if (end == NULL) {
				snprintf(err, err_len, "unbalanced ( in options");
				return false;
			}
			n = (size_t) (end - s - 1);
			if (n >= sizeof(arg))
				n = sizeof(arg) - 1;
			memcpy(arg, s + 1, n);
			arg[n] = '\0';
			s = end + 1;
		}
		if (!sm_options_apply(o, letter, arg[0] ? arg : NULL, err, err_len))
			return false;
	}
	return true;
}

static sm_options_t options(const char *s)
{
	sm_options_t o;
	char err[256];

	sm_options_defaults(&o);
	if (!parse_options(&o, s, err, sizeof(err))) {
		fprintf(stderr, "bad test options \"%s\": %s\n", s, err);
		exit(2);
	}
	return o;
}

/* ------------------------------------------------------------------ line */

typedef enum { CODEC_SLIN, CODEC_ULAW, CODEC_ALAW } codec_t;

typedef struct {
	codec_t codec;
	int delay_ms;           /* one way */
	int loss_per_mille;     /* packets lost, each way */
	int stall_ms;           /* far to us: nothing for this long ... */
	int stall_after_ms;     /* ... starting this long after the link comes up ... */
	bool stall_drop;        /* ... and then what was held back is lost, rather than late */
} line_t;

typedef struct {
	int16_t s[TICK];
	int n;
	int seq;
	long arrive;            /* tick */
	bool missing;
} pkt_t;

typedef struct {
	pkt_t *p;
	size_t head, len, cap;
	long last_arrive;
} pktq_t;

static void q_push(pktq_t *q, const pkt_t *pk)
{
	if (q->len == q->cap) {
		size_t cap = q->cap ? q->cap * 2 : 256;
		pkt_t *p = malloc(cap * sizeof(*p));

		for (size_t i = 0; i < q->len; i++)
			p[i] = q->p[(q->head + i) % q->cap];
		free(q->p);
		q->p = p;
		q->cap = cap;
		q->head = 0;
	}
	q->p[(q->head + q->len) % q->cap] = *pk;
	/* RTP over one path does not overtake itself. */
	if (q->p[(q->head + q->len) % q->cap].arrive < q->last_arrive)
		q->p[(q->head + q->len) % q->cap].arrive = q->last_arrive;
	q->last_arrive = q->p[(q->head + q->len) % q->cap].arrive;
	q->len++;
}

static pkt_t *q_ready(pktq_t *q, long tick)
{
	if (q->len == 0 || q->p[q->head].arrive > tick)
		return NULL;
	return &q->p[q->head];
}

static void q_pop(pktq_t *q)
{
	q->head = (q->head + 1) % q->cap;
	q->len--;
}

static void q_free(pktq_t *q)
{
	free(q->p);
	memset(q, 0, sizeof(*q));
}

static void codec_apply(codec_t c, int16_t *s, int n)
{
	for (int i = 0; i < n; i++) {
		if (c == CODEC_ULAW)
			s[i] = ulaw_to_linear(linear_to_ulaw(s[i]));
		else if (c == CODEC_ALAW)
			s[i] = alaw_to_linear(linear_to_alaw(s[i]));
	}
}

/* A small deterministic generator, so a failure can be run again. */
static uint32_t rnd(uint32_t *state)
{
	*state = *state * 1664525u + 1013904223u;
	return *state >> 8;
}

/* ------------------------------------------------------------------ calls */

typedef enum {
	END_NONE,
	END_SERVER_CLOSE,       /* the server says its last and closes; the line has to drain first */
	END_SERVER_RESET,       /* the same, with a TCP reset straight after its last */
	END_FAR_DROPS,          /* the far end drops its carrier */
	END_SERVER_EARLY,       /* the server goes before the modems train, saying nothing */
	END_SERVER_EARLY_SAYS   /* the server says something and goes before the modems train */
} ending_t;

typedef struct {
	const char *name;
	const char *us;          /* the Softmodem's options */
	bool us_calling;         /* the role it ends up in */
	const char *far;         /* the far end's options; NULL for a line with nobody on it */
	line_t line;
	size_t down;             /* bytes from our server to the far end's terminal */
	size_t up;               /* and from that terminal to our server */
	const char *expect_mod;  /* "|"-separated; NULL for any */
	int expect_rx;           /* our receive rate, 0 for any */
	int expect_tx;
	const char *expect_proto;
	ending_t ending;
	const char *expect_end;  /* how our end of the call ends; NULL: still running is fine */
	int limit_s;             /* simulated seconds before giving up */
	bool lockstep;           /* run alongside other calls: see run_concurrent() */
	bool tcp;                /* our server over TCP on loopback, not a socket pair */
} call_t;

typedef struct {
	sm_bridge_t *b;
	int srv;                 /* the test's end of the socket pair */
	unsigned char *to_send;
	size_t to_send_len, sent;
	unsigned char *got;
	size_t got_len, got_cap;
	bool closed;             /* the test closed its end */
} end_t;

/* Lockstep across threads: every call's tick happens, then the shared clock
 * moves on. */
typedef struct {
	pthread_mutex_t lock;
	pthread_cond_t cond;
	int parties, waiting;
	unsigned long generation;
} gate_t;

static gate_t *g_gate;

static void gate_wait(gate_t *g, bool advance_clock)
{
	pthread_mutex_lock(&g->lock);
	if (++g->waiting == g->parties) {
		if (advance_clock)
			dm_clock_advance_ms(TICK_MS);
		g->waiting = 0;
		g->generation++;
		pthread_cond_broadcast(&g->cond);
	} else {
		unsigned long gen = g->generation;

		while (gen == g->generation)
			pthread_cond_wait(&g->cond, &g->lock);
	}
	pthread_mutex_unlock(&g->lock);
}

static void gate_leave(gate_t *g)
{
	pthread_mutex_lock(&g->lock);
	g->parties--;
	if (g->waiting == g->parties && g->parties > 0) {
		dm_clock_advance_ms(TICK_MS);
		g->waiting = 0;
		g->generation++;
		pthread_cond_broadcast(&g->cond);
	}
	pthread_mutex_unlock(&g->lock);
}

/* A connected pair of TCP sockets on loopback: what a reset does to data in
 * flight is TCP's, which a socket pair does not imitate. */
static bool tcp_pair(int sv[2])
{
	struct sockaddr_in sa;
	socklen_t len = sizeof(sa);
	int lfd = socket(AF_INET, SOCK_STREAM, 0);

	memset(&sa, 0, sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	if (lfd < 0 || bind(lfd, (struct sockaddr *) &sa, sizeof(sa)) != 0 || listen(lfd, 1) != 0 ||
	    getsockname(lfd, (struct sockaddr *) &sa, &len) != 0) {
		if (lfd >= 0)
			close(lfd);
		return false;
	}
	sv[0] = socket(AF_INET, SOCK_STREAM, 0);
	if (sv[0] < 0 || connect(sv[0], (struct sockaddr *) &sa, sizeof(sa)) != 0) {
		close(lfd);
		return false;
	}
	sv[1] = accept(lfd, NULL, NULL);
	close(lfd);
	return sv[1] >= 0;
}

static bool end_open(end_t *e, const sm_options_t *o, bool calling, const char *tag, bool tcp)
{
	int sv[2];

	memset(e, 0, sizeof(*e));
	if (tcp ? !tcp_pair(sv) : socketpair(AF_UNIX, SOCK_STREAM, 0, sv) != 0)
		return false;
	fcntl(sv[0], F_SETFL, O_NONBLOCK);
	fcntl(sv[1], F_SETFL, O_NONBLOCK);
#ifdef SO_NOSIGPIPE
	{
		int one = 1;

		setsockopt(sv[0], SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
		setsockopt(sv[1], SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
	}
#endif
	e->b = sm_bridge_create(o, calling, sv[0], tag);
	if (e->b == NULL) {
		close(sv[0]);
		close(sv[1]);
		return false;
	}
	e->srv = sv[1];
	return true;
}

/* The modem and its server go; what the server received stays, to check. */
static void end_drop(end_t *e)
{
	sm_bridge_destroy(e->b);
	e->b = NULL;
	if (e->srv >= 0 && !e->closed)
		close(e->srv);
	e->srv = -1;
}

static void end_close(end_t *e)
{
	end_drop(e);
	free(e->got);
	e->got = NULL;
}

/* The test's side of a server: write what is due, read what has come. */
static void end_io(end_t *e, size_t send_upto)
{
	if (e->srv < 0 || e->closed)
		return;
	if (send_upto > e->to_send_len)
		send_upto = e->to_send_len;
	while (e->sent < send_upto) {
		ssize_t n = send(e->srv, e->to_send + e->sent, send_upto - e->sent, 0);

		if (n <= 0)
			break;
		e->sent += (size_t) n;
	}
	for (;;) {
		unsigned char buf[4096];
		ssize_t n = recv(e->srv, buf, sizeof(buf), 0);

		if (n <= 0)
			break;
		if (e->got_len + (size_t) n > e->got_cap) {
			e->got_cap = (e->got_len + (size_t) n) * 2;
			e->got = realloc(e->got, e->got_cap);
		}
		memcpy(e->got + e->got_len, buf, (size_t) n);
		e->got_len += (size_t) n;
	}
}

static void log_hex(const char *what, const unsigned char *p, size_t n)
{
	char line[3 * 32 + 1];
	size_t shown = n < 96 ? n : 96;

	log_append("  %s (%zu bytes, the first %zu):\n", what, n, shown);
	for (size_t i = 0; i < shown; i += 32) {
		size_t k = 0;

		for (size_t j = i; j < shown && j < i + 32; j++)
			k += (size_t) snprintf(line + k, sizeof(line) - k, "%02x ", p[j]);
		log_append("    %s\n", line);
	}
}

static unsigned char *pattern(size_t n, uint32_t seed, unsigned mask)
{
	unsigned char *p = malloc(n ? n : 1);

	for (size_t i = 0; i < n; i++)
		p[i] = (unsigned char) (rnd(&seed) & mask);
	return p;
}

/* Where a and b first differ, or -1. */
static long first_difference(const unsigned char *a, size_t alen, const unsigned char *b, size_t blen)
{
	size_t n = alen < blen ? alen : blen;

	for (size_t i = 0; i < n; i++)
		if (a[i] != b[i])
			return (long) i;
	return alen == blen ? -1 : (long) n;
}

static bool in_set(const char *set, const char *s)
{
	const char *p = set;
	size_t n = strlen(s);

	while (p && *p) {
		const char *bar = strchr(p, '|');
		size_t len = bar ? (size_t) (bar - p) : strlen(p);

		if (len == n && strncmp(p, s, n) == 0)
			return true;
		p = bar ? bar + 1 : NULL;
	}
	return false;
}

static unsigned data_mask(const sm_options_t *o)
{
	return (1u << o->data_bits) - 1;
}

/* What our server should see before the far end's first byte. */
static size_t preamble_len(const sm_options_t *o, bool calling, const unsigned char *got, size_t got_len)
{
	size_t n = 0;

	if (o->ulm_header) {
		size_t i;

		for (i = 0; i + 4 <= got_len; i++)
			if (memcmp(got + i, "\r\n\r\n", 4) == 0)
				break;
		if (i + 4 > got_len)
			return (size_t) -1;
		n = i + 4;
	}
	if (calling)
		n += 11; /* CONNECTED\r\n */
	return n;
}

/* What the far end's server gets before our server's first byte: CONNECTED
 * from its own bridge when it called, our NUL if we send one - and, from a
 * calling end running V.42 detection at a far end that does not, its ODP:
 * DC1s, which that far end takes for characters until we give up and fall
 * back (see datamodem's DEVNOTES). */
static size_t far_prefix(const call_t *c, const sm_options_t *uo, const sm_options_t *fo, const end_t *far,
			 size_t *odp)
{
	size_t n = (!c->us_calling ? 11 : 0) + (uo->send_null ? 1 : 0);
	size_t dc1 = 0;

	if (c->us_calling && strcmp(fo->v42, "off") == 0 && strcmp(uo->v42, "off") != 0)
		while (n + dc1 < far->got_len && (far->got[n + dc1] & 0x7f) == 0x11)
			dc1++;
	if (odp)
		*odp = dc1;
	return n + dc1;
}

typedef struct {
	char detail[512];
	bool ok;
	sm_status_t us;
	sm_end_t us_end;
	unsigned char head[128];  /* the first of what our server got */
	size_t head_len;
	unsigned char far_head[16];
	size_t far_head_len;
} result_t;

static bool fail(result_t *r, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(r->detail, sizeof(r->detail), fmt, ap);
	va_end(ap);
	r->ok = false;
	return false;
}

/* Runs one call. */
static bool run_call(const call_t *c, result_t *r)
{
	sm_options_t uo = options(c->us);
	sm_options_t fo = c->far ? options(c->far) : options("");
	end_t us, far;
	pktq_t f2u = { 0 }, u2f = { 0 };
	sm_clock_t clock;
	int16_t *out = malloc(OUT_MAX * sizeof(int16_t));
	int16_t silence[TICK] = { 0 };
	uint32_t seed = 12345;
	long limit = (long) (c->limit_s ? c->limit_s : 120) * 1000 / TICK_MS;
	long tick, connected_tick = -1, done_tick = -1, stall_from = -1, stall_to = -1;
	int far_seq = 0;
	bool far_alive = (c->far != NULL);
	bool down_ok = false, up_ok = false;
	bool closing = false;
	size_t hold_back = 0;
	size_t pre = 0;
	unsigned char *down_ref, *up_ref;
	char tag_us[64], tag_far[64];

	memset(r, 0, sizeof(*r));
	r->ok = true;
	snprintf(tag_us, sizeof(tag_us), "us:%s", c->name);
	snprintf(tag_far, sizeof(tag_far), "far:%s", c->name);

	if (!end_open(&us, &uo, c->us_calling, tag_us, c->tcp))
		return fail(r, "could not create our modem");
	if (far_alive && !end_open(&far, &fo, !c->us_calling, tag_far, false)) {
		end_close(&us);
		return fail(r, "could not create the far end's modem");
	}
	if (!far_alive)
		memset(&far, 0, sizeof(far)), far.srv = -1;

	/* What crosses: random bytes, as wide as the framing carries. When both
	 * ends have an MSB-first setting that differs, the far end sees each
	 * character reversed - checked separately. */
	us.to_send = pattern(c->down, 1, data_mask(&uo) & data_mask(&fo));
	us.to_send_len = c->down;
	far.to_send = pattern(c->up, 2, data_mask(&uo) & data_mask(&fo));
	far.to_send_len = c->up;
	down_ref = us.to_send;
	up_ref = far.to_send;
	/* A server closing as soon as it has said its last: keep the last
	 * part back until everything else has crossed. */
	if (c->ending == END_SERVER_CLOSE)
		hold_back = c->down > 64 ? 64 : c->down;
	/* Or all of it: everything at once, and then gone. */
	if (c->ending == END_SERVER_RESET || c->ending == END_SERVER_EARLY_SAYS)
		hold_back = c->down;

	sm_clock_init(&clock, 0);

	for (tick = 0; tick < limit; tick++) {
		int64_t sim_ms = (int64_t) tick * TICK_MS;
		int16_t far_out[TICK];
		bool got_any = false;
		pkt_t pk;
		sm_end_t ue;

		/* The far end: one packet in, one packet out, on its own clock. */
		if (far_alive) {
			pkt_t *in = q_ready(&u2f, tick);

			if (in != NULL && !in->missing)
				sm_bridge_audio(far.b, in->s, far_out, TICK);
			else
				sm_bridge_missing(far.b, far_out, TICK);
			if (in != NULL)
				q_pop(&u2f);
		} else {
			memcpy(far_out, silence, sizeof(far_out));
		}

		/* ... across the path to us. */
		memset(&pk, 0, sizeof(pk));
		memcpy(pk.s, far_out, sizeof(far_out));
		codec_apply(c->line.codec, pk.s, TICK);
		pk.n = TICK;
		pk.seq = far_seq++ & 0xffff;
		pk.arrive = tick + c->line.delay_ms / TICK_MS;
		if (stall_from >= 0 && tick >= stall_from && tick < stall_to) {
			if (c->line.stall_drop)
				pk.n = 0;
			else
				pk.arrive = stall_to;
		}
		if (c->line.loss_per_mille > 0 && (int) (rnd(&seed) % 1000) < c->line.loss_per_mille)
			pk.n = 0;
		if (pk.n > 0)
			q_push(&f2u, &pk);

		/* Us, as app_softmodem.c runs it. */
		for (pkt_t *in; (in = q_ready(&f2u, tick)) != NULL; q_pop(&f2u)) {
			int n = sm_clock_frame(&clock, us.b, in->s, in->n, in->seq, sim_ms, out, OUT_MAX);

			got_any = true;
			for (int off = 0; off < n; off += TICK) {
				pkt_t op;

				memset(&op, 0, sizeof(op));
				op.n = n - off < TICK ? n - off : TICK;
				memcpy(op.s, out + off, (size_t) op.n * sizeof(int16_t));
				codec_apply(c->line.codec, op.s, op.n);
				op.arrive = tick + c->line.delay_ms / TICK_MS;
				if (c->line.loss_per_mille > 0 && (int) (rnd(&seed) % 1000) < c->line.loss_per_mille)
					op.missing = true;
				q_push(&u2f, &op);
			}
		}
		if (!got_any) {
			int n = sm_clock_idle(&clock, us.b, sim_ms, out, OUT_MAX);

			for (int off = 0; off < n; off += TICK) {
				pkt_t op;

				memset(&op, 0, sizeof(op));
				op.n = n - off < TICK ? n - off : TICK;
				memcpy(op.s, out + off, (size_t) op.n * sizeof(int16_t));
				codec_apply(c->line.codec, op.s, op.n);
				op.arrive = tick + c->line.delay_ms / TICK_MS;
				q_push(&u2f, &op);
			}
		}

		/* Both servers. */
		ue = sm_bridge_service(us.b);
		if (far_alive)
			sm_bridge_service(far.b);
		end_io(&us, us.to_send_len - (closing ? 0 : hold_back));
		end_io(&far, far.to_send_len);

		if (c->lockstep)
			gate_wait(g_gate, true);
		else
			dm_clock_advance_ms(TICK_MS);

		if (connected_tick < 0) {
			sm_status_t st;

			sm_bridge_status(us.b, &st);
			if (st.connected) {
				connected_tick = tick;
				if (c->line.stall_ms > 0) {
					stall_from = tick + c->line.stall_after_ms / TICK_MS;
					stall_to = stall_from + c->line.stall_ms / TICK_MS;
				}

			}
		}

		/* A server that hangs up before anything has happened - having
		 * said something first, or not. */
		if ((c->ending == END_SERVER_EARLY || c->ending == END_SERVER_EARLY_SAYS) && tick == 50 && !us.closed) {
			closing = true;
			end_io(&us, us.to_send_len);
			close(us.srv);
			us.closed = true;
		}
		/* A server that sends everything it has and resets the connection
		 * straight away, before the line has taken a fraction of it. */
		if (c->ending == END_SERVER_RESET && !us.closed) {
			if (!closing && up_ok && connected_tick >= 0 && tick > connected_tick + 100) {
				closing = true;
				end_io(&us, us.to_send_len);
			} else if (closing && us.sent == us.to_send_len) {
				struct linger lg = { 1, 0 };

				setsockopt(us.srv, SOL_SOCKET, SO_LINGER, &lg, sizeof(lg));
				close(us.srv);
				us.closed = true;
			} else if (closing) {
				end_io(&us, us.to_send_len);
			}
		}

		/* Has everything crossed? */
		if (!down_ok && far_alive) {
			if (far.got_len >= far_prefix(c, &uo, &fo, &far, NULL) + c->down)
				down_ok = true;
		}
		if (!up_ok) {
			pre = preamble_len(&uo, c->us_calling, us.got, us.got_len);
			if (pre != (size_t) -1 && us.got_len >= pre + c->up)
				up_ok = true;
		}

		if (c->ending == END_SERVER_CLOSE && !closing && up_ok && far_alive &&
		    far.got_len >= far_prefix(c, &uo, &fo, &far, NULL) + c->down - hold_back) {
			/* The last words, and the server is gone. */
			closing = true;
			end_io(&us, us.to_send_len);
			if (us.sent == us.to_send_len) {
				close(us.srv);
				us.closed = true;
			} else {
				closing = false; /* try again next tick */
			}
		}

		if (done_tick < 0 && down_ok && up_ok) {
			done_tick = tick;
			sm_bridge_status(us.b, &r->us);
			if (c->ending == END_FAR_DROPS) {
				/* The far end drops its carrier and the line goes quiet. */
				end_drop(&far);
				far_alive = false;
			}
			if (c->ending == END_NONE)
				break;
		}
		if (ue != SM_RUNNING)
			break;
	}

	r->us_end = sm_bridge_end(us.b);
	if (done_tick < 0)
		sm_bridge_status(us.b, &r->us);
	r->head_len = us.got_len < sizeof(r->head) ? us.got_len : sizeof(r->head);
	memcpy(r->head, us.got, r->head_len);
	r->far_head_len = far.got_len < sizeof(r->far_head) ? far.got_len : sizeof(r->far_head);
	if (far.got)
		memcpy(r->far_head, far.got, r->far_head_len);

	/* What has to be true. */
	if (c->expect_end != NULL) {
		if (r->us_end == SM_RUNNING)
			fail(r, "still running after %lds (expected %s)", tick * TICK_MS / 1000, c->expect_end);
		else if (strcmp(sm_end_name(r->us_end), c->expect_end) != 0)
			fail(r, "ended %s, expected %s", sm_end_name(r->us_end), c->expect_end);
	}
	if (r->ok && c->far != NULL && c->ending != END_SERVER_EARLY) {
		if (!r->us.connected)
			fail(r, "never connected (%s)", sm_end_name(r->us_end));
		else if (!down_ok || !up_ok) {
			size_t far_pre = far_prefix(c, &uo, &fo, &far, NULL);
			long dd = far.got_len >= far_pre
					  ? first_difference(far.got + far_pre, far.got_len - far_pre, down_ref, c->down)
					  : 0;
			long du = us.got_len >= pre ? first_difference(us.got + pre, us.got_len - pre, up_ref, c->up) : 0;

			log_hex("our server sent", down_ref, c->down);
			log_hex("the far end's server received", far.got, far.got_len);
			log_hex("the far end's server sent", up_ref, c->up);
			log_hex("our server received", us.got, us.got_len);
			fail(r, "data incomplete after %lds: %zu of %zu down (first difference at %ld), "
				"%zu of %zu up (at %ld)", tick * TICK_MS / 1000, far.got_len - far_pre, c->down, dd,
			     us.got_len - pre, c->up, du);
		}
	}
	if (r->ok && c->far != NULL && c->ending != END_SERVER_EARLY && down_ok && up_ok) {
		size_t odp;
		size_t far_pre = far_prefix(c, &uo, &fo, &far, &odp);
		long d;

		if (odp > 0)
			log_append("  %zu ODP characters reached the far end before the fall back\n", odp);

		if (!c->us_calling && (far.got_len < 11 || memcmp(far.got, "CONNECTED\r\n", 11) != 0))
			fail(r, "the far end's server was not told CONNECTED");
		else if (c->us_calling && memcmp(us.got + pre - 11, "CONNECTED\r\n", 11) != 0)
			fail(r, "our server was not told CONNECTED");
		/* With one end MSB first and the other not, each character
		 * arrives with its bits the other way round - in async only, since
		 * under V.42 the bit order of a character means nothing. */
		bool reversed = uo.msb_first != fo.msb_first && r->us.protocol != NULL &&
				strcmp(r->us.protocol, "async") == 0;

		if (r->ok && !reversed) {
			d = first_difference(far.got + far_pre, far.got_len - far_pre, down_ref, c->down);
			if (d >= 0)
				fail(r, "down: byte %ld differs (%zu received, %zu sent)", d, far.got_len - far_pre, c->down);
			d = first_difference(us.got + pre, us.got_len - pre, up_ref, c->up);
			if (r->ok && d >= 0)
				fail(r, "up: byte %ld differs (%zu received, %zu sent)", d, us.got_len - pre, c->up);
		} else if (r->ok) {
			for (size_t i = 0; i < c->down && r->ok; i++) {
				unsigned want = 0, in = down_ref[i];

				for (int bit = 0; bit < uo.data_bits; bit++)
					if (in & (1u << bit))
						want |= 1u << (uo.data_bits - 1 - bit);
				if (far.got[far_pre + i] != want)
					fail(r, "down: byte %zu is %02x, not %02x reversed (%02x)", i, far.got[far_pre + i], in,
					     want);
			}
		}
	}
	if (r->ok && c->expect_mod && r->us.modulation && !in_set(c->expect_mod, r->us.modulation))
		fail(r, "ran %s, expected %s", r->us.modulation, c->expect_mod);
	if (r->ok && c->expect_rx && r->us.rx_rate != c->expect_rx)
		fail(r, "received at %d, expected %d", r->us.rx_rate, c->expect_rx);
	if (r->ok && c->expect_tx && r->us.tx_rate != c->expect_tx)
		fail(r, "sent at %d, expected %d", r->us.tx_rate, c->expect_tx);
	if (r->ok && c->expect_proto && r->us.protocol && strcmp(r->us.protocol, c->expect_proto) != 0)
		fail(r, "protocol %s, expected %s", r->us.protocol, c->expect_proto);
	if (r->ok && r->us.connected)
		snprintf(r->detail, sizeof(r->detail), "%s %d/%d %s, connected in %.1fs, %s", r->us.modulation,
			 r->us.rx_rate, r->us.tx_rate, r->us.protocol, (double) r->us.connect_ms / 1000.0,
			 sm_end_name(r->us_end) );
	else if (r->ok)
		snprintf(r->detail, sizeof(r->detail), "%s after %lds", sm_end_name(r->us_end), tick * TICK_MS / 1000);

	end_close(&far);
	end_close(&us);
	free(down_ref);
	free(up_ref);
	q_free(&f2u);
	q_free(&u2f);
	free(out);
	return r->ok;
}

static void test_call(const call_t *c)
{
	result_t r;

	if (!selected(c->name))
		return;
	begin();
	run_call(c, &r);
	report(c->name, r.ok, r.detail);
}

/* ----------------------------------------------------------- the matrix */

typedef struct {
	const char *name;      /* as v() takes it */
	const char *expect;    /* what our end may say it is running */
	int rate;              /* both ways, on a clean line */
	size_t bytes;          /* how much to send each way */
	const char *proto;     /* with V.42 on offer at both ends */
} mod_t;

static const mod_t MODS[] = {
	{ "V34", "v34", 33600, 4096, "V.42/V.42bis" },
	{ "V32bis", "v32bis", 14400, 4096, "V.42/V.42bis" },
	/* A V.32bis modem talks to a V.32 one as V.32, and V.22bis to V.22 as
	 * V.22: the engine names the modem it is, the rate says the rest. */
	{ "V32", "v32|v32bis", 9600, 2048, "V.42/V.42bis" },
	{ "V22bis", "v22bis", 2400, 1024, "V.42/V.42bis" },
	{ "V22", "v22|v22bis", 1200, 512, "V.42/V.42bis" },
	{ "V21", "v21", 300, 128, "async" },
	{ "Bell103", "bell103", 300, 128, "async" },
};

static void matrix(void)
{
	char name[96], far[64];

	for (size_t i = 0; i < sizeof(MODS) / sizeof(MODS[0]); i++) {
		const mod_t *m = &MODS[i];
		call_t c;

		/* A caller of each kind dials in and the Softmodem, at its defaults,
		 * works out what it is. */
		memset(&c, 0, sizeof(c));
		snprintf(name, sizeof(name), "inbound %s caller, Softmodem defaults", m->name);
		snprintf(far, sizeof(far), "v(%s)f", m->name);
		c.name = name;
		c.us = "";
		c.us_calling = false;
		c.far = far;
		c.down = c.up = m->bytes;
		c.expect_mod = m->expect;
		c.expect_rx = c.expect_tx = m->rate;
		c.expect_proto = m->proto;
		test_call(&c);

		/* The Softmodem, at its defaults, dials out to each kind of modem. */
		snprintf(name, sizeof(name), "outbound to %s answerer, Softmodem defaults", m->name);
		c.us_calling = true;
		test_call(&c);

		/* Both ends told the modulation. */
		snprintf(name, sizeof(name), "inbound %s, both fixed", m->name);
		{
			char us[64];

			snprintf(us, sizeof(us), "v(%s)f", m->name);
			c.us = us;
			c.us_calling = false;
			test_call(&c);
			snprintf(name, sizeof(name), "outbound %s, both fixed", m->name);
			c.us_calling = true;
			test_call(&c);
		}
	}

	/* V.23, which nothing falls back to, so v() has to say so: the
	 * videotex centre answering at 1200 forward, 75 back, and the terminal
	 * calling it. */
	{
		call_t c;

		memset(&c, 0, sizeof(c));
		c.name = "inbound V23 caller, Softmodem v(V23)";
		c.us = "v(V23)";
		c.far = "v(V23)f";
		c.down = 512;
		c.up = 32;
		c.expect_mod = "v23";
		c.expect_rx = 75;
		c.expect_tx = 1200;
		c.expect_proto = "async";
		test_call(&c);

		c.name = "outbound to V23 answerer, Softmodem v(V23)";
		c.us_calling = true;
		c.down = 32;
		c.up = 512;
		c.expect_rx = 1200;
		c.expect_tx = 75;
		test_call(&c);
	}
}

/* ----------------------------------------------------- the link layer */

static void link_layer(void)
{
	static const call_t calls[] = {
		{ .name = "V.42: far end without V.42, inbound V32bis", .us = "", .far = "v(V32bis)fe(off)",
		  .down = 2048, .up = 2048, .expect_mod = "v32bis", .expect_proto = "async" },
		{ .name = "V.42: far end without V.42, outbound V22bis", .us = "", .us_calling = true,
		  .far = "v(V22bis)fe(off)", .down = 512, .up = 512, .expect_mod = "v22bis", .expect_proto = "async" },
		{ .name = "V.42: far end without V.42bis, inbound V34", .us = "", .far = "v(V34)fk", .down = 4096,
		  .up = 4096, .expect_mod = "v34", .expect_proto = "V.42" },
		{ .name = "V.42: Softmodem e(off), outbound V32bis", .us = "e(off)", .us_calling = true,
		  .far = "v(V32bis)f", .down = 2048, .up = 2048, .expect_proto = "async" },
		{ .name = "V.42: Softmodem k, inbound V22bis", .us = "k", .far = "v(V22bis)f", .down = 512, .up = 512,
		  .expect_proto = "V.42" },
		{ .name = "V.42: Softmodem e(require), inbound V32bis", .us = "e(require)", .far = "v(V32bis)f",
		  .down = 2048, .up = 2048, .expect_proto = "V.42/V.42bis" },
		/* 7E2 and 5N1 framing need async at both ends. */
		{ .name = "framing 7E2, inbound V22bis", .us = "e(off)d(7)p(even)s(2)", .far = "v(V22bis)fe(off)d(7)p(even)s(2)",
		  .down = 512, .up = 512, .expect_proto = "async" },
		{ .name = "framing 7O1, outbound V21", .us = "d(7)p(odd)", .us_calling = true, .far = "v(V21)fd(7)p(odd)",
		  .down = 128, .up = 128, .expect_mod = "v21" },
		{ .name = "framing 5N2, inbound Bell103", .us = "d(5)s(2)", .far = "v(Bell103)fd(5)s(2)",
		  .down = 128, .up = 128, .expect_mod = "bell103" },
		/* Capping the rate, at both ends of the handshake. */
		{ .name = "b(9600) inbound V34 caller", .us = "b(9600)", .far = "v(V34)f", .down = 2048, .up = 2048,
		  .expect_rx = 9600, .expect_tx = 9600 },
		{ .name = "b(1200) outbound V22bis", .us = "b(1200)", .us_calling = true, .far = "v(V22bis)f",
		  .down = 512, .up = 512, .expect_rx = 1200 },
		/* Starting below the top. */
		{ .name = "v(V22bis) inbound, finds a V21 caller", .us = "v(V22bis)", .far = "v(V21)f", .down = 128,
		  .up = 128, .expect_mod = "v21" },
		{ .name = "v(V32bis) outbound, finds a Bell103 answerer", .us = "v(V32bis)", .us_calling = true,
		  .far = "v(Bell103)f", .down = 128, .up = 128, .expect_mod = "bell103" },
		/* A far end that steps down too, from somewhere else. */
		{ .name = "inbound V32bis caller that steps down, Softmodem v(V22bis)", .us = "v(V22bis)",
		  .far = "v(V32bis)", .down = 512, .up = 512, .expect_mod = "v22bis", .expect_rx = 2400 },
	};

	for (size_t i = 0; i < sizeof(calls) / sizeof(calls[0]); i++)
		test_call(&calls[i]);
}

/* ------------------------------------------------------------ Btx extras */

static void btx(void)
{
	call_t c;
	result_t r;
	char detail[600];

	/* The Btx centre: V.23 answering, a NUL to the terminal, the ULM
	 * header to the server. */
	memset(&c, 0, sizeof(c));
	c.name = "Btx: v(V23)un answering, NUL and ULM header";
	c.us = "v(V23)ld(8)s(1)un";
	c.far = "v(V23)f";
	c.down = 256;
	c.up = 16;
	c.expect_mod = "v23";
	if (selected(c.name)) {
		static const char header[] = "Version: 1\r\nTXspeed: 120.00\r\nRXspeed: 7.50\r\n\r\n";

		begin();
		run_call(&c, &r);
		if (r.ok && (r.head_len < sizeof(header) - 1 || memcmp(r.head, header, sizeof(header) - 1) != 0)) {
			r.ok = false;
			snprintf(r.detail, sizeof(r.detail), "the ULM header was \"%.*s\"", (int) r.head_len, r.head);
		}
		/* The far end hears CONNECTED from its own server, then our NUL. */
		if (r.ok && (r.far_head_len < 12 || r.far_head[11] != 0)) {
			r.ok = false;
			snprintf(r.detail, sizeof(r.detail), "the far end did not get a NUL first");
		}
		snprintf(detail, sizeof(detail), "%s", r.detail);
		report(c.name, r.ok, detail);
	}

	/* MSB first at both ends is a round trip; at one end, every character
	 * arrives reversed. */
	memset(&c, 0, sizeof(c));
	c.name = "bit order: m at both ends, inbound V23";
	c.us = "v(V23)m";
	c.far = "v(V23)fm";
	c.down = 256;
	c.up = 16;
	test_call(&c);

	c.name = "bit order: m at our end only, inbound V21";
	c.us = "v(V21)m";
	c.far = "v(V21)f";
	c.down = 64;
	c.up = 0;
	test_call(&c);

	/* Under V.42 the bit order of a character means nothing: m leaves it alone. */
	c.name = "bit order: m ignored under V.42, inbound V22bis";
	c.us = "m";
	c.far = "v(V22bis)f";
	c.down = 512;
	c.up = 512;
	c.expect_proto = "V.42/V.42bis";
	test_call(&c);
}

/* --------------------------------------------------------------- the line */

static void impairments(void)
{
	static const call_t calls[] = {
		{ .name = "line: ulaw, inbound V34", .us = "", .far = "v(V34)f", .line = { .codec = CODEC_ULAW },
		  .down = 4096, .up = 4096, .expect_mod = "v34" },
		{ .name = "line: alaw, outbound V34", .us = "", .us_calling = true, .far = "v(V34)f",
		  .line = { .codec = CODEC_ALAW }, .down = 4096, .up = 4096, .expect_mod = "v34" },
		{ .name = "line: ulaw 150 ms, inbound V32bis", .us = "", .far = "v(V32bis)f",
		  .line = { .codec = CODEC_ULAW, .delay_ms = 150 }, .down = 4096, .up = 4096, .expect_mod = "v32bis" },
		{ .name = "line: ulaw 150 ms, outbound V22bis", .us = "", .us_calling = true, .far = "v(V22bis)f",
		  .line = { .codec = CODEC_ULAW, .delay_ms = 150 }, .down = 1024, .up = 1024, .expect_mod = "v22bis" },
		{ .name = "line: ulaw 100 ms, inbound V21", .us = "", .far = "v(V21)f",
		  .line = { .codec = CODEC_ULAW, .delay_ms = 100 }, .down = 128, .up = 128, .expect_mod = "v21" },
		/* Lost packets cost bits; V.42 puts them back. */
		{ .name = "line: 0.5% loss, inbound V32bis with V.42", .us = "", .far = "v(V32bis)f",
		  .line = { .codec = CODEC_ULAW, .delay_ms = 60, .loss_per_mille = 5 }, .down = 8192, .up = 8192,
		  .expect_mod = "v32bis", .expect_proto = "V.42/V.42bis", .limit_s = 240 },
		{ .name = "line: 0.5% loss, outbound V22bis with V.42", .us = "", .us_calling = true,
		  .far = "v(V22bis)f", .line = { .codec = CODEC_ULAW, .delay_ms = 60, .loss_per_mille = 5 },
		  .down = 2048, .up = 2048, .expect_mod = "v22bis", .expect_proto = "V.42/V.42bis", .limit_s = 240 },
		/* Nothing for 300 ms, then all of it at once: our clock covers
		 * the gap and drops what arrives too late to be used. */
		{ .name = "line: 300 ms stall then a burst, inbound V32bis", .us = "", .far = "v(V32bis)f",
		  .line = { .codec = CODEC_ULAW, .delay_ms = 40, .stall_ms = 300, .stall_after_ms = 2000 },
		  .down = 8192, .up = 8192, .expect_mod = "v32bis", .limit_s = 240 },
		{ .name = "line: 300 ms stall then a burst, inbound V22bis", .us = "", .far = "v(V22bis)f",
		  .line = { .codec = CODEC_ULAW, .delay_ms = 40, .stall_ms = 300, .stall_after_ms = 2000 },
		  .down = 2048, .up = 2048, .expect_mod = "v22bis", .limit_s = 240 },
		{ .name = "line: 400 ms of lost packets, outbound V34", .us = "", .us_calling = true,
		  .far = "v(V34)f",
		  .line = { .codec = CODEC_ULAW, .delay_ms = 40, .stall_ms = 400, .stall_after_ms = 2000,
			    .stall_drop = true },
		  .down = 16384, .up = 16384, .expect_mod = "v34", .limit_s = 240 },
	};

	for (size_t i = 0; i < sizeof(calls) / sizeof(calls[0]); i++)
		test_call(&calls[i]);
}

/* --------------------------------------------------------- hanging up */

static void endings(void)
{
	static const call_t calls[] = {
		{ .name = "end: server closes, inbound V32bis drains first", .us = "", .far = "v(V32bis)f",
		  .down = 8192, .up = 1024, .ending = END_SERVER_CLOSE, .expect_end = "SERVERCLOSED" },
		{ .name = "end: server closes, outbound V22bis drains first", .us = "", .us_calling = true,
		  .far = "v(V22bis)f", .down = 2048, .up = 256, .ending = END_SERVER_CLOSE,
		  .expect_end = "SERVERCLOSED" },
		{ .name = "end: server closes, inbound V21 drains first", .us = "", .far = "v(V21)f", .down = 256,
		  .up = 64, .ending = END_SERVER_CLOSE, .expect_end = "SERVERCLOSED" },
		{ .name = "end: far end drops carrier, inbound V22bis", .us = "", .far = "v(V22bis)f", .down = 256,
		  .up = 256, .ending = END_FAR_DROPS, .expect_end = "CARRIERLOST" },
		{ .name = "end: far end drops carrier, outbound V21", .us = "", .us_calling = true, .far = "v(V21)f",
		  .down = 64, .up = 64, .ending = END_FAR_DROPS, .expect_end = "CARRIERLOST" },
		{ .name = "end: far end drops carrier, inbound V34", .us = "", .far = "v(V34)f", .down = 1024,
		  .up = 1024, .ending = END_FAR_DROPS, .expect_end = "CARRIERLOST", .limit_s = 120 },
		{ .name = "end: nobody there, answering w(10)", .us = "w(10)", .far = NULL,
		  .expect_end = "NOCARRIER", .limit_s = 30 },
		{ .name = "end: nobody there, calling w(10)", .us = "w(10)", .us_calling = true, .far = NULL,
		  .expect_end = "NOCARRIER", .limit_s = 30 },
		{ .name = "end: server gives up before training", .us = "", .far = "v(V22bis)f",
		  .ending = END_SERVER_EARLY, .expect_end = "SERVERCLOSED", .limit_s = 30 },
		/* Its parting words still reach the far end once the modems train. */
		{ .name = "end: server says its piece and goes before training, inbound V22bis", .us = "",
		  .far = "v(V22bis)f", .down = 64, .up = 0, .ending = END_SERVER_EARLY_SAYS,
		  .expect_end = "SERVERCLOSED", .limit_s = 60 },
		/* 2 kB is eight seconds at 2400 bps, and the reset comes a moment
		 * after it is sent: it all has to arrive anyway. */
		{ .name = "end: server resets the connection, what it sent still arrives, async V22bis",
		  .us = "e(off)", .far = "v(V22bis)fe(off)", .down = 2048, .up = 64, .ending = END_SERVER_RESET,
		  .expect_end = "SOCKETERROR", .tcp = true },
		{ .name = "end: server resets the connection, what it sent still arrives, outbound V21",
		  .us = "", .us_calling = true, .far = "v(V21)f", .down = 256, .up = 32, .ending = END_SERVER_RESET,
		  .expect_end = "SOCKETERROR", .tcp = true },
	};

	for (size_t i = 0; i < sizeof(calls) / sizeof(calls[0]); i++)
		test_call(&calls[i]);
}

/* --------------------------------------------------------- side by side */

typedef struct {
	call_t call;
	result_t result;
	char *log;
} job_t;

static void *job_main(void *arg)
{
	job_t *j = arg;

	run_call(&j->call, &j->result);
	j->log = t_log ? strdup(t_log) : NULL;
	gate_leave(g_gate);
	return NULL;
}

/* Asterisk runs a modem per channel, each on its own thread, all at once.
 * Four calls in lockstep on four threads, with V.42 in each. */
static void concurrent(void)
{
	static const call_t calls[] = {
		{ .name = "concurrent: V34", .us = "", .far = "v(V34)f", .down = 4096, .up = 4096,
		  .expect_mod = "v34", .lockstep = true },
		{ .name = "concurrent: V32bis", .us = "", .us_calling = true, .far = "v(V32bis)f", .down = 4096,
		  .up = 4096, .expect_mod = "v32bis", .lockstep = true },
		{ .name = "concurrent: V22bis", .us = "", .far = "v(V22bis)f", .down = 1024, .up = 1024,
		  .expect_mod = "v22bis", .lockstep = true },
		{ .name = "concurrent: V22bis without V.42bis", .us = "", .us_calling = true, .far = "v(V22bis)fk",
		  .down = 1024, .up = 1024, .expect_mod = "v22bis", .expect_proto = "V.42", .lockstep = true },
	};
	enum { N = sizeof(calls) / sizeof(calls[0]) };
	job_t jobs[N];
	pthread_t th[N];
	gate_t gate;

	if (!selected("concurrent"))
		return;
	memset(&gate, 0, sizeof(gate));
	pthread_mutex_init(&gate.lock, NULL);
	pthread_cond_init(&gate.cond, NULL);
	gate.parties = N;
	g_gate = &gate;
	begin();
	for (int i = 0; i < N; i++) {
		jobs[i].call = calls[i];
		pthread_create(&th[i], NULL, job_main, &jobs[i]);
	}
	for (int i = 0; i < N; i++)
		pthread_join(th[i], NULL);
	for (int i = 0; i < N; i++) {
		/* Each call's own log, if it failed. */
		t_log_len = 0;
		if (jobs[i].log != NULL)
			log_append("%s", jobs[i].log);
		report(jobs[i].call.name, jobs[i].result.ok, jobs[i].result.detail);
		free(jobs[i].log);
	}
	g_gate = NULL;
}

/* -------------------------------------------------------------- units */

static void unit_options(void)
{
	sm_options_t o;
	char err[256];
	bool ok = true;
	char detail[256] = "";

	if (!selected("options"))
		return;
	begin();

	sm_options_defaults(&o);
	if (strcmp(o.modulation, "v34") != 0 || !o.step_down || o.role != SM_ROLE_AUTO ||
	    strcmp(o.v42, "detect") != 0 || !o.v42bis || o.data_bits != 8 || o.stop_bits != 1 ||
	    o.tx_power != -13) {
		ok = false;
		snprintf(detail, sizeof(detail), "defaults are not V.34 stepping down, V.42bis, 8N1, -13 dBm0");
	}
	report("options: defaults", ok, detail);

	{
		static const struct {
			const char *in, *out;
		} names[] = { { "V21", "v21" },       { "V23", "v23" },       { "Bell103", "bell103" },
			      { "V22", "v22" },       { "V22bis", "v22bis" }, { "v.22bis", "v22bis" },
			      { "V32", "v32" },       { "V32bis", "v32bis" }, { "V.34", "v34" },
			      { "auto", "v34" },      { "BELL-103", "bell103" } };

		ok = true;
		detail[0] = '\0';
		for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
			sm_options_defaults(&o);
			if (!sm_options_apply(&o, 'v', names[i].in, err, sizeof(err)) ||
			    strcmp(o.modulation, names[i].out) != 0) {
				ok = false;
				snprintf(detail, sizeof(detail), "v(%s) gave %s", names[i].in, o.modulation);
			}
		}
		report("options: every modulation name the old and new dialplans use", ok, detail);
	}

	{
		static const struct {
			char letter;
			const char *arg;
		} bad[] = { { 'v', "V90" }, { 'd', "9" },   { 'd', "4" },      { 's', "3" },   { 'p', "mark" },
			    { 'e', "maybe" }, { 'b', "100" }, { 't', "+3" },     { 't', "x" },   { 'g', "2100" },
			    { 'w', "1" },   { 'z', NULL } };

		ok = true;
		detail[0] = '\0';
		for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
			sm_options_defaults(&o);
			if (sm_options_apply(&o, bad[i].letter, bad[i].arg, err, sizeof(err))) {
				ok = false;
				snprintf(detail, sizeof(detail), "%c(%s) was accepted", bad[i].letter,
					 bad[i].arg ? bad[i].arg : "");
			}
		}
		report("options: bad values are refused", ok, detail);
	}

	{
		ok = true;
		detail[0] = '\0';
		o = options("v(V23)ld(7)s(2)p(even)unt(-28)r(-35)amw(20)fe(off)kb(2400)g(1800)");
		if (o.role != SM_ROLE_ANSWERING || strcmp(o.modulation, "v23") != 0 || o.data_bits != 7 ||
		    o.stop_bits != 2 || strcmp(o.parity, "even") != 0 || !o.ulm_header || !o.send_null ||
		    o.tx_power != -28 || !o.msb_first || o.train_timeout_s != 20 || o.step_down ||
		    strcmp(o.v42, "off") != 0 || o.v42bis || o.bit_rate != 2400 || strcmp(o.guard_tone, "1800") != 0) {
			ok = false;
			snprintf(detail, sizeof(detail), "a full option string did not all apply");
		}
		report("options: a full option string", ok, detail);
	}

	{
		/* What the engine refuses, refused before the call: V.42 required
		 * where it never runs. */
		o = options("v(V21)fe(require)");
		ok = !sm_options_check(&o, err, sizeof(err));
		report("options: V.42 required over V.21 is refused up front", ok, ok ? err : "accepted");
		o = options("");
		ok = sm_options_check(&o, err, sizeof(err));
		report("options: the defaults pass the engine's checks", ok, ok ? "" : err);
		o = options("v(V23)un");
		ok = sm_options_check(&o, err, sizeof(err));
		report("options: the Btx example from the README passes", ok, ok ? "" : err);
	}
}

/* The clock, counted: what goes out for what comes in. */
static void unit_clock(void)
{
	sm_options_t o;
	end_t e;
	sm_clock_t c;
	int16_t in[TICK] = { 0 };
	int16_t *out;
	bool ok = true;
	char detail[256] = "";
	int n;

	if (!selected("clock"))
		return;
	begin();
	out = malloc(OUT_MAX * sizeof(int16_t));
	o = options("v(V22bis)f");
	if (!end_open(&e, &o, false, "clock", false)) {
		report("clock", false, "could not create a modem");
		free(out);
		return;
	}
	sm_clock_init(&c, 0);

#define EXPECT(cond, ...)                                                                                     \
	do {                                                                                                  \
		if (ok && !(cond)) {                                                                          \
			ok = false;                                                                           \
			snprintf(detail, sizeof(detail), __VA_ARGS__);                                        \
		}                                                                                             \
	} while (0)

	n = sm_clock_frame(&c, e.b, in, TICK, 100, 0, out, OUT_MAX);
	EXPECT(n == TICK, "a frame in gave %d out, not %d", n, TICK);
	n = sm_clock_frame(&c, e.b, in, TICK, 101, 20, out, OUT_MAX);
	EXPECT(n == TICK, "the next frame gave %d", n);
	/* Two packets lost: answered as if they had come. */
	n = sm_clock_frame(&c, e.b, in, TICK, 104, 80, out, OUT_MAX);
	EXPECT(n == 3 * TICK, "after two lost packets, %d out, not %d", n, 3 * TICK);
	/* A repeat, and one from the past: nothing. */
	n = sm_clock_frame(&c, e.b, in, TICK, 104, 80, out, OUT_MAX);
	EXPECT(n == 0, "a repeated packet gave %d", n);
	n = sm_clock_frame(&c, e.b, in, TICK, 102, 80, out, OUT_MAX);
	EXPECT(n == 0, "a stale packet gave %d", n);
	/* Quiet, but not for long enough to count. */
	n = sm_clock_idle(&c, e.b, 80 + SM_STALL_MS - 1, out, OUT_MAX);
	EXPECT(n == 0, "%d ms of quiet gave %d", SM_STALL_MS - 1, n);
	/* A stall: our clock takes over, a frame behind real time. */
	n = sm_clock_idle(&c, e.b, 80 + 100, out, OUT_MAX);
	EXPECT(n == 4 * TICK, "100 ms of quiet gave %d, not %d", n, 4 * TICK);
	n = sm_clock_idle(&c, e.b, 80 + 140, out, OUT_MAX);
	EXPECT(n == 2 * TICK, "40 ms more gave %d, not %d", n, 2 * TICK);
	/* The audio was only late: all six frames of it arrive, and are
	 * dropped, having been covered; the seventh is new. */
	for (int s = 105; s <= 110; s++) {
		n = sm_clock_frame(&c, e.b, in, TICK, s, 230, out, OUT_MAX);
		EXPECT(n == 0, "late packet %d gave %d, not 0", s, n);
	}
	n = sm_clock_frame(&c, e.b, in, TICK, 111, 240, out, OUT_MAX);
	EXPECT(n == TICK, "the first new packet after a stall gave %d", n);
	/* Another stall, in which the packets were lost rather than late: the
	 * gap is covered once, not twice. */
	n = sm_clock_idle(&c, e.b, 240 + 100, out, OUT_MAX);
	EXPECT(n == 4 * TICK, "100 ms of quiet gave %d", n);
	n = sm_clock_frame(&c, e.b, in, TICK, 117, 345, out, OUT_MAX);
	EXPECT(n == 2 * TICK, "after 5 lost packets of which 4 were covered: %d, not %d", n, 2 * TICK);
	/* No sequence numbers at all - not every channel has them. */
	n = sm_clock_frame(&c, e.b, in, 80, -1, 400, out, OUT_MAX);
	EXPECT(n == 80, "a 10 ms frame without a sequence number gave %d", n);
	/* A jump of minutes is a new stream, not a loss to fill. */
	n = sm_clock_frame(&c, e.b, in, TICK, 30000, 420, out, OUT_MAX);
	EXPECT(n == TICK, "a new stream gave %d", n);
	n = sm_clock_frame(&c, e.b, in, TICK, 30001, 440, out, OUT_MAX);
	EXPECT(n == TICK, "and its next packet %d", n);
	/* Sequence numbers wrap. */
	n = sm_clock_frame(&c, e.b, in, TICK, 65535, 460, out, OUT_MAX);
	EXPECT(n == 0 || n == TICK, "-");
	sm_clock_init(&c, 0);
	sm_clock_frame(&c, e.b, in, TICK, 65534, 0, out, OUT_MAX);
	sm_clock_frame(&c, e.b, in, TICK, 65535, 20, out, OUT_MAX);
	n = sm_clock_frame(&c, e.b, in, TICK, 0, 40, out, OUT_MAX);
	EXPECT(n == TICK, "across the wrap: %d", n);
	n = sm_clock_frame(&c, e.b, in, TICK, 2, 80, out, OUT_MAX);
	EXPECT(n == 2 * TICK, "one lost just after the wrap: %d", n);
#undef EXPECT

	report("clock: lost, repeated, late and missing packets", ok, detail);
	end_close(&e);
	free(out);
}

static void unit_connect(void)
{
	int lfd, fd;
	struct sockaddr_in sa;
	socklen_t len = sizeof(sa);
	char err[256];
	int port;

	if (!selected("connect"))
		return;
	begin();
	lfd = socket(AF_INET, SOCK_STREAM, 0);
	memset(&sa, 0, sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	if (lfd < 0 || bind(lfd, (struct sockaddr *) &sa, sizeof(sa)) != 0 || listen(lfd, 1) != 0 ||
	    getsockname(lfd, (struct sockaddr *) &sa, &len) != 0) {
		report("connect: to a listening server", false, "could not listen");
		return;
	}
	port = ntohs(sa.sin_port);
	fd = sm_connect_tcp("localhost", port, 2000, err, sizeof(err));
	report("connect: to a listening server, by name", fd >= 0 && (fcntl(fd, F_GETFL) & O_NONBLOCK),
	       fd >= 0 ? "" : err);
	if (fd >= 0)
		close(fd);
	close(lfd);
	/* Nothing listens there now. */
	fd = sm_connect_tcp("127.0.0.1", port, 2000, err, sizeof(err));
	report("connect: refused", fd < 0, fd < 0 ? err : "connected to nothing");
	if (fd >= 0)
		close(fd);
	fd = sm_connect_tcp("no-such-host.invalid", 23, 2000, err, sizeof(err));
	report("connect: a name that does not resolve", fd < 0, fd < 0 ? err : "connected");
	if (fd >= 0)
		close(fd);
}

int main(int argc, char **argv)
{
	g_verbose = getenv("SOFTMODEM_TEST_VERBOSE") != NULL;
	g_filters = argv + 1;
	g_nfilters = argc - 1;
	sm_log_set_sink(sink, enabled);
	sm_init(g_verbose ? DM_LOG_DEBUG : DM_LOG_WARN);
	setvbuf(stdout, NULL, _IOLBF, 0);

	unit_options();
	unit_clock();
	unit_connect();
	matrix();
	link_layer();
	btx();
	impairments();
	endings();
	concurrent();

	printf("1..%d\n", g_run);
	if (g_run == 0)
		printf("# no test matched\n");
	printf("# %d passed, %d failed\n", g_run - g_failed, g_failed);
	return g_failed == 0 && g_run > 0 ? 0 : 1;
}
