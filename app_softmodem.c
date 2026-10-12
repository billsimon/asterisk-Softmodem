/*
 * Softmodem for Asterisk
 *
 * 2010, Christian Groeger <code@proquari.at>
 *
 * Based on app_fax.c by Dmitry Andrianov <asterisk@dima.spb.ru>
 * and Steve Underwood <steveu@coppice.org>
 *
 * The data pump is datamodem's (https://github.com/billsimon/datamodem),
 * vendored in engine/ - see engine/UPSTREAM.
 *
 * This program is free software, distributed under the terms of
 * the GNU General Public License
 *
 */

/*** MODULEINFO
	<depend>spandsp</depend>
	<support_level>extended</support_level>
***/

/* Needed for spandsp headers */
#define ASTMM_LIBC ASTMM_IGNORE

#include "asterisk.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "asterisk/app.h"
#include "asterisk/channel.h"
#include "asterisk/format_cache.h"
#include "asterisk/format_cap.h"
#include "asterisk/frame.h"
#include "asterisk/logger.h"
#include "asterisk/module.h"
#include "asterisk/pbx.h"
#include "asterisk/strings.h"
#include "asterisk/time.h"
#include "asterisk/utils.h"

#include "softmodem/bridge.h"

/*** DOCUMENTATION
	<application name="Softmodem" language="en_US">
		<synopsis>
			A data modem on the channel, connected to a TCP server.
		</synopsis>
		<syntax>
			<parameter name="hostname">
				<para>The server to connect the modem to. Default: localhost.</para>
			</parameter>
			<parameter name="port">
				<para>Its TCP port. Default: 23.</para>
			</parameter>
			<parameter name="options">
				<optionlist>
					<option name="a">
						<para>This is the answering modem: the far end called us.</para>
					</option>
					<option name="c">
						<para>This is the calling modem: we called the far end.</para>
						<para>Without <literal>a</literal> or <literal>c</literal> the modem answers,
						unless the channel is one Asterisk placed (Originate, a call file, or the
						called side of Dial), in which case it calls.</para>
					</option>
					<option name="v">
						<argument name="modulation" required="true" />
						<para>The fastest modulation to try: <literal>V34</literal> (the default),
						<literal>V32bis</literal>, <literal>V32</literal>, <literal>V22bis</literal>,
						<literal>V22</literal>, <literal>V21</literal>, <literal>Bell103</literal> or
						<literal>V23</literal>. Below it, the modem falls back to whatever the far end
						turns out to be. V23 (1200/75, as used by videotex) is never fallen back to and
						has to be asked for.</para>
					</option>
					<option name="f">
						<para>Fixed: only the modulation <literal>v</literal> names, no falling back.</para>
					</option>
					<option name="b">
						<argument name="rate" required="true" />
						<para>The highest bit rate to offer, e.g. <literal>2400</literal>.</para>
					</option>
					<option name="e">
						<argument name="mode" required="true" />
						<para>V.42 error correction: <literal>detect</literal> (the default: when the far end
						will), <literal>off</literal>, or <literal>require</literal>. Never over V.21, Bell 103
						or V.23.</para>
					</option>
					<option name="k">
						<para>No V.42bis compression.</para>
					</option>
					<option name="d">
						<argument name="bits" required="true" />
						<para>Data bits, 5 to 8. Default: 8.</para>
					</option>
					<option name="p">
						<argument name="parity" required="true" />
						<para><literal>none</literal> (the default), <literal>even</literal> or
						<literal>odd</literal>.</para>
					</option>
					<option name="s">
						<argument name="bits" required="true" />
						<para>Stop bits, 1 or 2. Default: 1.</para>
					</option>
					<option name="l">
						<para>Least significant bit first (the default, and the standard).</para>
					</option>
					<option name="m">
						<para>Most significant bit first.</para>
					</option>
					<option name="t">
						<argument name="level" required="true" />
						<para>Transmit level in dBm0. Default: -13.</para>
					</option>
					<option name="r">
						<argument name="cutoff" required="true" />
						<para>Accepted for compatibility and ignored.</para>
					</option>
					<option name="g">
						<argument name="tone" required="true" />
						<para>Guard tone for an answering V.22bis modem: <literal>550</literal> or
						<literal>1800</literal>. Default: none.</para>
					</option>
					<option name="w">
						<argument name="seconds" required="true" />
						<para>Hang up if the modems have not trained in this long. Default: 45.</para>
					</option>
					<option name="u">
						<para>Send the server an Ulm Relay Protocol header with the line speeds (Btx).</para>
					</option>
					<option name="n">
						<para>Send a NUL down the line once connected (Btx).</para>
					</option>
				</optionlist>
			</parameter>
		</syntax>
		<description>
			<para>Runs a data modem on the channel - V.34, V.32bis, V.32, V.22bis, V.22, V.21, Bell 103 or
			V.23, with V.42 error correction and V.42bis compression - and connects what it carries to a TCP
			server, as a telnet client would. The call ends when the far end's carrier goes or the server
			closes the connection.</para>
			<para>Use G.711 (ulaw or alaw) end to end, and switch off anything that alters the audio:
			inband DTMF detection, fax detection, echo cancellation, jitter buffers that conceal loss.</para>
			<para>This application sets the following channel variables:</para>
			<variablelist>
				<variable name="SOFTMODEM_RESULT">
					<value name="SERVERCLOSED">The server closed the connection.</value>
					<value name="CARRIERLOST">The far end's carrier went away.</value>
					<value name="HANGUP">The channel hung up.</value>
					<value name="NOCARRIER">The modems never trained.</value>
					<value name="UNSTABLE">The link kept retraining.</value>
					<value name="CONNECTFAILED">The server could not be reached.</value>
					<value name="SOCKETERROR">The connection to the server failed.</value>
					<value name="ERROR">Something else went wrong.</value>
				</variable>
				<variable name="SOFTMODEM_MODULATION">What the link ran, e.g. v32bis.</variable>
				<variable name="SOFTMODEM_RATE">The bit rate received at.</variable>
				<variable name="SOFTMODEM_TX_RATE">The bit rate sent at.</variable>
				<variable name="SOFTMODEM_PROTOCOL">async, V.42 or V.42/V.42bis.</variable>
				<variable name="SOFTMODEM_BYTES_SENT">Characters sent down the line.</variable>
				<variable name="SOFTMODEM_BYTES_RECEIVED">Characters received from the line.</variable>
			</variablelist>
		</description>
	</application>
 ***/

static const char app[] = "Softmodem";

enum {
	OPT_ANSWER =        (1 << 0),
	OPT_CALL =          (1 << 1),
	OPT_MODULATION =    (1 << 2),
	OPT_FIXED =         (1 << 3),
	OPT_BIT_RATE =      (1 << 4),
	OPT_V42 =           (1 << 5),
	OPT_NO_V42BIS =     (1 << 6),
	OPT_DATABITS =      (1 << 7),
	OPT_PARITY =        (1 << 8),
	OPT_STOPBITS =      (1 << 9),
	OPT_LSB_FIRST =     (1 << 10),
	OPT_MSB_FIRST =     (1 << 11),
	OPT_TX_POWER =      (1 << 12),
	OPT_RX_CUTOFF =     (1 << 13),
	OPT_GUARD_TONE =    (1 << 14),
	OPT_TRAIN_TIMEOUT = (1 << 15),
	OPT_ULM_HEADER =    (1 << 16),
	OPT_NULL =          (1 << 17),
};

enum {
	OPT_ARG_MODULATION,
	OPT_ARG_BIT_RATE,
	OPT_ARG_V42,
	OPT_ARG_DATABITS,
	OPT_ARG_PARITY,
	OPT_ARG_STOPBITS,
	OPT_ARG_TX_POWER,
	OPT_ARG_RX_CUTOFF,
	OPT_ARG_GUARD_TONE,
	OPT_ARG_TRAIN_TIMEOUT,

	/* Must be the last element */
	OPT_ARG_ARRAY_SIZE,
};

AST_APP_OPTIONS(additional_options, BEGIN_OPTIONS
	AST_APP_OPTION('a', OPT_ANSWER),
	AST_APP_OPTION('c', OPT_CALL),
	AST_APP_OPTION_ARG('v', OPT_MODULATION, OPT_ARG_MODULATION),
	AST_APP_OPTION('f', OPT_FIXED),
	AST_APP_OPTION_ARG('b', OPT_BIT_RATE, OPT_ARG_BIT_RATE),
	AST_APP_OPTION_ARG('e', OPT_V42, OPT_ARG_V42),
	AST_APP_OPTION('k', OPT_NO_V42BIS),
	AST_APP_OPTION_ARG('d', OPT_DATABITS, OPT_ARG_DATABITS),
	AST_APP_OPTION_ARG('p', OPT_PARITY, OPT_ARG_PARITY),
	AST_APP_OPTION_ARG('s', OPT_STOPBITS, OPT_ARG_STOPBITS),
	AST_APP_OPTION('l', OPT_LSB_FIRST),
	AST_APP_OPTION('m', OPT_MSB_FIRST),
	AST_APP_OPTION_ARG('t', OPT_TX_POWER, OPT_ARG_TX_POWER),
	AST_APP_OPTION_ARG('r', OPT_RX_CUTOFF, OPT_ARG_RX_CUTOFF),
	AST_APP_OPTION_ARG('g', OPT_GUARD_TONE, OPT_ARG_GUARD_TONE),
	AST_APP_OPTION_ARG('w', OPT_TRAIN_TIMEOUT, OPT_ARG_TRAIN_TIMEOUT),
	AST_APP_OPTION('u', OPT_ULM_HEADER),
	AST_APP_OPTION('n', OPT_NULL),
END_OPTIONS );

/* Each option's letter, flag and argument slot (-1 for none), in the order
 * they are applied - which only matters for l/m and a/c, refused together. */
static const struct {
	char letter;
	unsigned int flag;
	int arg;
} option_table[] = {
	{ 'a', OPT_ANSWER, -1 },
	{ 'c', OPT_CALL, -1 },
	{ 'v', OPT_MODULATION, OPT_ARG_MODULATION },
	{ 'f', OPT_FIXED, -1 },
	{ 'b', OPT_BIT_RATE, OPT_ARG_BIT_RATE },
	{ 'e', OPT_V42, OPT_ARG_V42 },
	{ 'k', OPT_NO_V42BIS, -1 },
	{ 'd', OPT_DATABITS, OPT_ARG_DATABITS },
	{ 'p', OPT_PARITY, OPT_ARG_PARITY },
	{ 's', OPT_STOPBITS, OPT_ARG_STOPBITS },
	{ 'l', OPT_LSB_FIRST, -1 },
	{ 'm', OPT_MSB_FIRST, -1 },
	{ 't', OPT_TX_POWER, OPT_ARG_TX_POWER },
	{ 'r', OPT_RX_CUTOFF, OPT_ARG_RX_CUTOFF },
	{ 'g', OPT_GUARD_TONE, OPT_ARG_GUARD_TONE },
	{ 'w', OPT_TRAIN_TIMEOUT, OPT_ARG_TRAIN_TIMEOUT },
	{ 'u', OPT_ULM_HEADER, -1 },
	{ 'n', OPT_NULL, -1 },
};

/* How long to wait for the server to accept the connection. */
#define SOFTMODEM_CONNECT_TIMEOUT_MS 10000

/* Room for a frame and the biggest gap the clock will fill at once. */
#define SOFTMODEM_OUT_SAMPLES (SM_MAX_GAP_SAMPLES + 2 * SM_SAMPLE_RATE / 10)

/* ------------------------------------------------------------------ logging */

static void ast_sink(dm_log_level_t level, const char *component, const char *msg)
{
	switch (level) {
	case DM_LOG_ERROR:
		ast_log(LOG_ERROR, "[%s] %s\n", component, msg);
		break;
	case DM_LOG_WARN:
		ast_log(LOG_WARNING, "[%s] %s\n", component, msg);
		break;
	case DM_LOG_INFO:
		ast_verb(3, "Softmodem [%s] %s\n", component, msg);
		break;
	case DM_LOG_DEBUG:
		ast_debug(1, "[%s] %s\n", component, msg);
		break;
	default:
		ast_debug(4, "[%s] %s\n", component, msg);
		break;
	}
}

static bool ast_enabled(dm_log_level_t level)
{
	switch (level) {
	case DM_LOG_ERROR:
	case DM_LOG_WARN:
		return true;
	case DM_LOG_INFO:
		return VERBOSITY_ATLEAST(3);
	case DM_LOG_DEBUG:
		return DEBUG_ATLEAST(1);
	default:
		return DEBUG_ATLEAST(4);
	}
}

/* ------------------------------------------------------------------ channel */

static int write_audio(struct ast_channel *chan, const int16_t *samples, int n, int frame_samples)
{
	uint8_t buffer[AST_FRIENDLY_OFFSET + SM_SAMPLE_RATE / 10 * sizeof(int16_t)];
	int max = SM_SAMPLE_RATE / 10;

	if (frame_samples <= 0 || frame_samples > max)
		frame_samples = SM_SAMPLE_RATE / 50;
	/* In frames the size of the far end's, so that a gap filled in one go
	 * goes out as the packets it stands for. */
	while (n > 0) {
		int len = n < frame_samples ? n : frame_samples;
		struct ast_frame f = {
			.frametype = AST_FRAME_VOICE,
			.src = app,
		};

		f.subclass.format = ast_format_slin;
		memcpy(buffer + AST_FRIENDLY_OFFSET, samples, len * sizeof(int16_t));
		f.samples = len;
		AST_FRAME_SET_BUFFER(&f, buffer, AST_FRIENDLY_OFFSET, len * sizeof(int16_t));
		if (ast_write(chan, &f) < 0) {
			ast_log(LOG_WARNING, "Failed to write frame to %s: %s\n", ast_channel_name(chan), strerror(errno));
			return -1;
		}
		samples += len;
		n -= len;
	}
	return 0;
}

/* Anything that listens to the audio for tones of its own - DTMF, fax - can
 * mistake a modem for one and mute or divert the call, and an echo canceller
 * fights V.32's and V.34's own. Channel drivers that can switch these off do
 * it here (DAHDI); the rest have to be configured not to. */
static void set_dsp(struct ast_channel *chan, int on)
{
	char v = on ? 1 : 0;

	ast_channel_setoption(chan, AST_OPTION_DIGIT_DETECT, &v, sizeof(v), 0);
	if (!on) {
		ast_channel_setoption(chan, AST_OPTION_FAX_DETECT, &v, sizeof(v), 0);
		ast_channel_setoption(chan, AST_OPTION_ECHOCAN, &v, sizeof(v), 0);
	}
}

/* Modems need the audio as it was sent; a codec that compresses it destroys
 * anything faster than a fraction of 300 bps. */
static void check_codec(struct ast_channel *chan)
{
	struct ast_format_cap *caps = ast_channel_nativeformats(chan);

	if (caps == NULL)
		return;
	if (ast_format_cap_iscompatible_format(caps, ast_format_ulaw) == AST_FORMAT_CMP_NOT_EQUAL &&
	    ast_format_cap_iscompatible_format(caps, ast_format_alaw) == AST_FORMAT_CMP_NOT_EQUAL &&
	    ast_format_cap_iscompatible_format(caps, ast_format_slin) == AST_FORMAT_CMP_NOT_EQUAL) {
		struct ast_str *names = ast_str_alloca(AST_FORMAT_CAP_NAMES_LEN);

		ast_log(LOG_WARNING, "%s is using %s; a modem needs G.711 (ulaw or alaw) end to end\n",
			ast_channel_name(chan), ast_format_cap_get_names(caps, &names));
	}
}

static void set_result(struct ast_channel *chan, const char *result, sm_bridge_t *b)
{
	char num[32];

	pbx_builtin_setvar_helper(chan, "SOFTMODEM_RESULT", result);
	if (b == NULL)
		return;
	{
		sm_status_t st;

		sm_bridge_status(b, &st);
		pbx_builtin_setvar_helper(chan, "SOFTMODEM_MODULATION", st.connected ? st.modulation : "");
		pbx_builtin_setvar_helper(chan, "SOFTMODEM_PROTOCOL", st.connected ? st.protocol : "");
		snprintf(num, sizeof(num), "%d", st.connected ? st.rx_rate : 0);
		pbx_builtin_setvar_helper(chan, "SOFTMODEM_RATE", num);
		snprintf(num, sizeof(num), "%d", st.connected ? st.tx_rate : 0);
		pbx_builtin_setvar_helper(chan, "SOFTMODEM_TX_RATE", num);
		snprintf(num, sizeof(num), "%llu", (unsigned long long) st.bytes_to_line);
		pbx_builtin_setvar_helper(chan, "SOFTMODEM_BYTES_SENT", num);
		snprintf(num, sizeof(num), "%llu", (unsigned long long) st.bytes_from_line);
		pbx_builtin_setvar_helper(chan, "SOFTMODEM_BYTES_RECEIVED", num);
	}
}

static int64_t now_ms(void)
{
	struct timeval tv = ast_tvnow();

	return (int64_t) tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

/* The call itself: audio in, audio out, bytes to and from the server.
 * Returns -1 if the channel hung up, else 0. */
static int softmodem_run(struct ast_channel *chan, sm_bridge_t *b)
{
	int16_t *out;
	sm_clock_t clock;
	int frame_samples = SM_SAMPLE_RATE / 50;
	int res = 0;

	out = ast_malloc(SOFTMODEM_OUT_SAMPLES * sizeof(int16_t));
	if (out == NULL) {
		sm_bridge_set_end(b, SM_END_ERROR);
		return 0;
	}
	sm_clock_init(&clock, now_ms());

	for (;;) {
		struct ast_frame *f;
		int n = 0;
		int ready = ast_waitfor(chan, 20);

		if (ready < 0) {
			sm_bridge_set_end(b, SM_END_HANGUP);
			res = -1;
			break;
		}
		if (ready > 0) {
			f = ast_read(chan);
			if (f == NULL) {
				ast_debug(1, "%s hung up\n", ast_channel_name(chan));
				sm_bridge_set_end(b, SM_END_HANGUP);
				res = -1;
				break;
			}
			/* The format has to be checked as well: a frame in the old one
			 * may have been queued before the channel was switched to
			 * signed linear. */
			if (f->frametype == AST_FRAME_VOICE &&
			    ast_format_cmp(f->subclass.format, ast_format_slin) == AST_FORMAT_CMP_EQUAL &&
			    f->samples > 0 && f->samples <= SM_SAMPLE_RATE / 10 && f->data.ptr != NULL) {
				int seq = ast_test_flag(f, AST_FRFLAG_HAS_TIMING_INFO) ? (f->seqno & 0xffff) : -1;

				frame_samples = f->samples;
				n = sm_clock_frame(&clock, b, f->data.ptr, f->samples, seq, now_ms(), out,
						   SOFTMODEM_OUT_SAMPLES);
			}
			ast_frfree(f);
		}
		if (n == 0)
			n = sm_clock_idle(&clock, b, now_ms(), out, SOFTMODEM_OUT_SAMPLES);
		if (n > 0 && write_audio(chan, out, n, frame_samples) < 0) {
			sm_bridge_set_end(b, SM_END_HANGUP);
			res = -1;
			break;
		}
		if (sm_bridge_service(b) != SM_RUNNING)
			break;
	}
	ast_free(out);
	return res;
}

static int softmodem_exec(struct ast_channel *chan, const char *data)
{
	char *parse;
	char err[256];
	struct ast_flags flags = { 0, };
	char *option_args[OPT_ARG_ARRAY_SIZE] = { NULL, };
	sm_options_t opts;
	const char *host = "localhost";
	int port = 23;
	bool calling;
	struct ast_format *read_fmt = NULL, *write_fmt = NULL;
	sm_bridge_t *b = NULL;
	int sock;
	int res = 0;

	AST_DECLARE_APP_ARGS(args,
		AST_APP_ARG(host);
		AST_APP_ARG(port);
		AST_APP_ARG(options);
	);

	if (chan == NULL) {
		ast_log(LOG_ERROR, "Channel is NULL. Giving up.\n");
		return -1;
	}

	parse = ast_strdupa(S_OR(data, ""));
	AST_STANDARD_APP_ARGS(args, parse);

	if (!ast_strlen_zero(args.host))
		host = ast_strip(args.host);
	if (!ast_strlen_zero(args.port)) {
		if (sscanf(args.port, "%30d", &port) != 1 || port <= 0 || port > 65535) {
			ast_log(LOG_ERROR, "Please specify a valid port number.\n");
			return -1;
		}
	}

	sm_options_defaults(&opts);
	if (!ast_strlen_zero(args.options)) {
		if (ast_app_parse_options(additional_options, &flags, option_args, args.options)) {
			ast_log(LOG_ERROR, "Could not parse the options '%s'\n", args.options);
			return -1;
		}
		if (ast_test_flag(&flags, OPT_ANSWER) && ast_test_flag(&flags, OPT_CALL)) {
			ast_log(LOG_ERROR, "Please only set c or a flag, not both.\n");
			return -1;
		}
		if (ast_test_flag(&flags, OPT_LSB_FIRST) && ast_test_flag(&flags, OPT_MSB_FIRST)) {
			ast_log(LOG_ERROR, "Please only set l or m flag, not both.\n");
			return -1;
		}
		for (size_t i = 0; i < ARRAY_LEN(option_table); i++) {
			const char *arg;

			if (!ast_test_flag(&flags, option_table[i].flag))
				continue;
			arg = option_table[i].arg >= 0 ? option_args[option_table[i].arg] : NULL;
			if (option_table[i].arg >= 0 && ast_strlen_zero(arg))
				continue;
			if (!sm_options_apply(&opts, option_table[i].letter, arg, err, sizeof(err))) {
				ast_log(LOG_ERROR, "%s\n", err);
				return -1;
			}
		}
		if (ast_test_flag(&flags, OPT_RX_CUTOFF))
			ast_log(LOG_NOTICE, "Softmodem: the r() option no longer does anything\n");
	}
	if (!sm_options_check(&opts, err, sizeof(err))) {
		ast_log(LOG_ERROR, "%s\n", err);
		return -1;
	}

	/* Who dialled whom decides which end of the handshake we are. A
	 * channel Asterisk placed - Originate, a call file, the called side of
	 * Dial - is a call we made; anything else, one we are answering. */
	if (opts.role == SM_ROLE_AUTO) {
		calling = ast_test_flag(ast_channel_flags(chan), AST_FLAG_OUTGOING) ||
			  ast_test_flag(ast_channel_flags(chan), AST_FLAG_ORIGINATED);
	} else {
		calling = (opts.role == SM_ROLE_CALLING);
	}

	/* Nothing may start until the call is answered: an answer tone into
	 * ringback, or training against it, is the classic way to "connect" to
	 * nobody. */
	if (ast_channel_state(chan) != AST_STATE_UP) {
		if (ast_answer(chan)) {
			ast_log(LOG_WARNING, "Could not answer channel '%s'\n", ast_channel_name(chan));
			return -1;
		}
	}

	/* The server first: a call it cannot be connected to is not worth
	 * training for. */
	sock = sm_connect_tcp(host, port, SOFTMODEM_CONNECT_TIMEOUT_MS, err, sizeof(err));
	if (sock < 0) {
		ast_log(LOG_WARNING, "Softmodem on %s: %s\n", ast_channel_name(chan), err);
		set_result(chan, "CONNECTFAILED", NULL);
		return 0;
	}

	read_fmt = ao2_bump(ast_channel_readformat(chan));
	write_fmt = ao2_bump(ast_channel_writeformat(chan));
	if (ast_set_read_format(chan, ast_format_slin) < 0 || ast_set_write_format(chan, ast_format_slin) < 0) {
		ast_log(LOG_WARNING, "Unable to set %s to signed linear\n", ast_channel_name(chan));
		close(sock);
		set_result(chan, "ERROR", NULL);
		goto restore;
	}
	check_codec(chan);
	set_dsp(chan, 0);

	sm_init(DEBUG_ATLEAST(1) ? DM_LOG_DEBUG : DM_LOG_WARN);
	b = sm_bridge_create(&opts, calling, sock, ast_channel_name(chan));
	if (b == NULL) {
		close(sock);
		set_result(chan, "ERROR", NULL);
		goto restore_dsp;
	}
	ast_verb(3, "Softmodem on %s: %s modem, up to %s%s, connected to %s port %d\n", ast_channel_name(chan),
		 calling ? "calling" : "answering", opts.modulation, opts.step_down ? " and falling back" : " only",
		 host, port);

	res = softmodem_run(chan, b);
	{
		sm_status_t st;

		sm_bridge_status(b, &st);
		ast_verb(3, "Softmodem on %s: %s after %s%s%s, %llu bytes sent and %llu received\n",
			 ast_channel_name(chan), sm_end_name(sm_bridge_end(b)), st.connected ? st.modulation : "no carrier",
			 st.connected ? " " : "", st.connected ? st.protocol : "",
			 (unsigned long long) st.bytes_to_line, (unsigned long long) st.bytes_from_line);
	}
	/* Even for a channel that has hung up: an h extension can still read them. */
	set_result(chan, sm_end_name(sm_bridge_end(b)), b);
	sm_bridge_destroy(b);

restore_dsp:
	if (res == 0)
		set_dsp(chan, 1);
restore:
	if (res == 0 && read_fmt && ast_set_read_format(chan, read_fmt) < 0)
		ast_log(LOG_WARNING, "Unable to restore read format on '%s'\n", ast_channel_name(chan));
	if (res == 0 && write_fmt && ast_set_write_format(chan, write_fmt) < 0)
		ast_log(LOG_WARNING, "Unable to restore write format on '%s'\n", ast_channel_name(chan));
	ao2_cleanup(read_fmt);
	ao2_cleanup(write_fmt);
	return res;
}

static int unload_module(void)
{
	int res = ast_unregister_application(app);

	sm_log_set_sink(NULL, NULL);
	return res;
}

static int load_module(void)
{
	sm_log_set_sink(ast_sink, ast_enabled);
	sm_init(DM_LOG_WARN);
	return ast_register_application_xml(app, softmodem_exec) ? AST_MODULE_LOAD_DECLINE : AST_MODULE_LOAD_SUCCESS;
}

AST_MODULE_INFO_STANDARD_EXTENDED(ASTERISK_GPL_KEY, "Softmodem: V.34, V.32bis, V.22bis, V.21, Bell 103 and V.23 data modem");
