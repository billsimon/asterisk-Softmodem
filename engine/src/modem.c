#include "datamodem/modem.h"
#include "datamodem/log.h"
#include "datamodem/ring.h"
#include "datamodem/util.h"
#include "datamodem/v32.h"
#include "datamodem/v34.h"

#include <inttypes.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* V.42's transmit bit rate has to be corrected after init and there is no
 * API for it - see v42_set_bit_rate(). That needs the private structure. */
#define SPANDSP_EXPOSE_INTERNAL_STRUCTURES
#include <spandsp.h>

/* The transmit queue holds this many seconds of line time. It is deliberately
 * short: anything sitting in it is already committed, so a big queue only
 * means a longer wait between changing your mind and the line noticing. At
 * 300 bps four seconds is about 120 characters. The receive queue is fixed
 * and large, because nothing downstream of it is slow - it only has to cover
 * the stretch where command mode holds the screen. */
#define DM_TX_SECONDS 4
#define DM_TX_QUEUE_MIN 256
#define DM_RX_QUEUE 65536

/* Under V.42 the receive queue need never overflow: when it is this close to
 * full, LAPM tells the far end to wait (RNR), and it is let go again once
 * there is this much room. The gap stops it flapping; what arrives meanwhile
 * - a decompressed frame at most - is far less than either. Without V.42
 * there is no way to ask, and the oldest bytes go. */
#define DM_RX_BUSY_FREE 16384
#define DM_RX_READY_FREE 32768

#define DM_SAMPLE_RATE 8000

/* A dropped carrier is not automatically the end of the call: real lines
 * glitch, and a V.22bis pair will retrain in under two seconds when they do.
 * Only a carrier that stays away this long counts as NO CARRIER. Longer than
 * a retrain takes, shorter than anyone's patience.
 *
 * V.32 gets longer. Its retrain is the whole start-up procedure again - the
 * tone exchange, two training sequences each sized to the round trip, and
 * the rate exchange - which is about four seconds on a 200 ms path and more
 * on a longer one. */
#define DM_CARRIER_GRACE_MS 4000
#define DM_CARRIER_GRACE_V32_MS 12000
/* V.34's retrain goes back through Phase 2 - tones, probing, two training
 * sequences sized to the round trip, and the MP exchange - which is longer
 * again. */
#define DM_CARRIER_GRACE_V34_MS 15000

/* Staging for compressor output waiting to be packed into LAPM frames.
 * Generous: V.42bis in transparent mode can expand slightly, and a frame is
 * only ever 128 bytes, so this is many frames' worth of headroom. */
#define DM_COMP_QUEUE 8192
/* How much plaintext to push through the compressor in one go, and the free
 * space required in the staging queue before doing so. The gap between them
 * is the worst-case expansion, with a lot to spare. */
#define DM_COMP_CHUNK 512
#define DM_COMP_HEADROOM 2048

/* While V.42 detection is still undecided the far end may already be
 * talking - a host that does not do V.42 starts sending its banner the
 * moment it has a carrier. Those bits go to the detector, so unless they
 * are framed in parallel and held, everything said before we give up on
 * V.42 is lost. Ten seconds at 2400 bps is 3 kB; this is comfortably more,
 * and it fills from the front, so what survives is the beginning of the
 * banner rather than the end. */
#define DM_PENDING_RX 8192

/* Handing a byte to the link layer is not the same as putting it on the
 * line. With V.42 especially, LAPM swallows our whole queue into its window
 * in an instant and then spends the next half minute actually transmitting
 * it - so "our queue is empty" is not remotely the same question as "is it
 * safe to hang up now".
 *
 * spandsp can be asked how many frames LAPM still holds, but the answer is
 * useless for this: its acknowledgement bookkeeping lags the real delivery
 * by as much as a minute and a half, so waiting on it would keep a finished
 * call up long after the data had arrived.
 *
 * So the drain is estimated from the line rate instead, which is the thing
 * that actually governs it. Ten bits per byte covers start/stop framing and
 * HDLC overhead alike, and the tail covers packetisation and the far end's
 * jitter buffer - the last RTP packet has to be played out over there before
 * we clear the call. */
#define DM_LINE_BITS_PER_BYTE 10
#define DM_LINE_TAIL_MS 750

/* V.42's N401 - the most data one LAPM I-frame may carry. spandsp defaults
 * to this and caps it here too, but only says so in its private header, so
 * the number is repeated rather than reached for. The compressor is told not
 * to emit blocks bigger than this, so everything it produces fits a frame. */
#define DM_LAPM_MAX_FRAME 128

/* spandsp reports this through the V.42 status callback when its detection
 * phase gives up on the far end. The value is real API - lapm_status_to_str()
 * is exported and will name it - but the enum itself is file-local to
 * spandsp's v42.c and appears in no header, so the number is repeated here
 * and checked against the library at startup. */
#define DM_LAPM_V42_UNSUPPORTED 8
/* Likewise the two states of the same enum that say how far a handshake had
 * got: still sending ODP, or past detection with no link yet. */
#define DM_LAPM_DETECT 0
#define DM_LAPM_IDLE 1

/* V.42's detection phase recognises a repeating pattern, so what each end
 * needs is a number of bit periods, not a number of milliseconds. spandsp's
 * detector wants about 512 of them; this is double, for margin. */
#define DM_V42_DETECT_MIN_BITS 1024

#if defined(DATAMODEM_VENDORED_V42)
/* See third_party/spandsp-v42. */
extern __thread char dm_v42_disconnect_cause[80];
extern int dm_v42_reconnect(v42_state_t *s);
extern int dm_v42_refuse_sabme;
extern __thread unsigned dm_v42_fcs_errors;
extern __thread int dm_v42_adp_no_ec;
extern int dm_v42_answer_no_ec;
extern int dm_v42_no_xid;
extern int dm_v42_xid_done(const v42_state_t *s);
extern void dm_v42_set_t401(const v42_state_t *s, int ms);
#endif

/* How long to leave between attempts at establishment. Long enough that a
 * far end which said "not yet" has had a chance to become ready, short
 * enough to fit several tries inside a default --v42-timeout. */
#define DM_V42_RETRY_MS 1500

/* How many times to re-offer establishment to a far end that answers with a
 * point-blank DM. V.42 8.3.2.1 makes a DM response a legitimate "I cannot
 * enter the connected state", and a far end that says it once may well be
 * ready a moment later - but one that says it every time has made up its
 * mind, and there is no sense spending the whole of --v42-timeout finding
 * that out. Silence gets the full timeout, because silence may be a path
 * losing frames rather than an answer. */
#define DM_V42_MAX_REFUSALS 2

/* How long to watch an under-offer link before judging whether the two ends
 * actually agreed about the rate, and how bad the framing has to be to say
 * they did not. A link that is demodulating the far end at the wrong rate is
 * reading noise, so roughly half of its characters arrive with the stop bit
 * in the wrong place; one that is merely on a poor line is nothing like
 * that. */
#define DM_RATE_CHECK_MS 2500
#define DM_RATE_CHECK_MIN_CHARS 40

/* T400, the detection window, is 750 ms in the specification - which assumes
 * a copper pair where the far end's reply comes back more or less at once.
 * Over RTP it does not: there is a jitter buffer at each end, and the whole
 * exchange has to fit inside the window:
 *
 *     far end recognises our ODP     ~DM_V42_DETECT_MIN_BITS
 *   + its reply crosses the path     ~2 x jitter buffer
 *   + we recognise its ADP           ~DM_V42_DETECT_MIN_BITS
 *
 * At 2400 bps with the default 150 ms buffers that is about 1.15 s, so the
 * specified 750 ms expires while the far end's ADP is still in flight. We
 * then declare it non-V.42 and fall back - and it, having committed to
 * LAPM, carries on sending HDLC that we read as line noise until one side
 * gives up. Budget the window for the path it is actually running over.
 *
 * Erring long is cheap here: T400 governs only how long we keep listening
 * before giving up, it never makes the far end wait, and anything the far
 * end says meanwhile is held and delivered on fallback rather than lost. */
#define DM_V42_DETECT_SPEC_MS 750

typedef enum
{
    DM_MOD_V22BIS = 0,
    DM_MOD_V22,
    DM_MOD_V21,
    DM_MOD_BELL103,
    DM_MOD_V23,
    DM_MOD_V32,
    DM_MOD_V32BIS,
    DM_MOD_V34
} dm_mod_t;

/* What the far end sounds like while we are still finding out what it is.
 * See listen_block(). */
typedef enum
{
    HEARD_NONE = 0,
    HEARD_AC,        /* a V.32 answering modem: 600 and 3000 Hz */
    HEARD_AA,        /* a V.32 calling modem: 1800 Hz */
    HEARD_ANS,       /* an answer tone, V.25's or V.8's: 2100 Hz */
    HEARD_USB1,      /* a V.22 bis or V.22 answering modem's unscrambled ones */
    HEARD_V21_ANS,   /* V.21 channel 2, the answering modem's: mark 1650 Hz */
    HEARD_BELL_ANS,  /* Bell 103's answering band: mark 2225 Hz */
    HEARD_V21_ORIG,  /* V.21 channel 1, the calling modem's: mark 980 Hz */
    HEARD_BELL_ORIG, /* Bell 103's originating band: mark 1270 Hz */
    HEARD_V23_ANS,   /* V.23's forward channel, the answering modem's: mark 1300 Hz */
    HEARD_V23_ORIG,  /* V.23's backward channel, the calling modem's: mark 390 Hz */
    HEARD_LOW_FSK,   /* one or the other: which, takes more than one block to say */
    HEARD_COUNT
} heard_t;

/* 40 ms. Over exactly this many samples, Goertzel bins 25 Hz apart are
 * orthogonal - which is what separates Bell 103's 2225 Hz from the 2250 Hz
 * that a V.22 bis answering modem's unscrambled ones put most of their power
 * at. */
#define DM_LISTEN_BLOCK 320
#define DM_LISTEN_OWN_MAX 2

typedef struct
{
    int16_t buf[DM_LISTEN_BLOCK];
    int n;
    int run[HEARD_COUNT];      /* consecutive blocks each has been heard in */
    float own[DM_LISTEN_OWN_MAX]; /* what we are sending: its echo is not the far end */
    int n_own;
    float v21_tones;           /* HEARD_LOW_FSK: the power at each one's tones, this block */
    float bell_tones;
    float v21_sum;             /* and over the run of blocks it has lasted */
    float bell_sum;
    int fsk_blocks;
} dm_listen_t;

typedef enum
{
    DM_V42_OFF = 0,
    DM_V42_DETECT,   /* try, and fall back to raw async if the far end does not */
    DM_V42_REQUIRE   /* try, and drop the call if the far end does not */
} dm_v42_mode_t;

struct dm_modem
{
    pthread_mutex_t lock;

    /* Settings, fixed after create - except the modulation itself, and
     * what follows from it, which change when we step down. */
    dm_mod_t mod;
    const char *mod_name;
    bool calling;
    int offered_rate;
    int tx_bit_rate;         /* what we transmit at; V.23 differs by direction */
    int carrier_grace_ms;
    int guard;
    int data_bits;
    int parity;              /* ASYNC_PARITY_* */
    int stop_bits;
    bool v14;
    float tx_power;
    int path_delay_ms;
    char tag[64];

    /* Pre-carrier phase. */
    int answer_tone_samples;  /* answerer: ANS burst length, 0 = none */
    int wait_samples;         /* caller: samples left to listen for ANS */
    int ans_tail_samples;     /* caller: samples left to let the tone finish */
    bool ans_tail_running;
    modem_connect_tones_tx_state_t *tone_tx;
    modem_connect_tones_rx_state_t *tone_rx;

    /* Data pumps. Exactly one family is live. */
    dm_v32_t *v32;
    dm_v34_t *v34;
    v22bis_state_t *v22;
    fsk_tx_state_t *fsk_tx;
    fsk_rx_state_t *fsk_rx;
    async_rx_state_t *arx;
    /* A second framer, identical except that V.14 is off, whose only job is
     * to count stop bits that landed in the wrong place. V.14 rate adaption
     * exists precisely to tolerate that, so the real framer - which has it
     * on by default - reports no framing errors at all and cannot be asked
     * whether what it is framing is data or noise. See
     * check_rate_agreement(). */
    async_rx_state_t *arx_monitor;

    /* Error correction and compression. When v42 is running it replaces the
     * asynchronous character framing entirely: the pump's bits are LAPM's,
     * not a start/stop stream. */
    dm_v42_mode_t v42_mode;
    bool v42_peer_declined;    /* spandsp's detection gave up on the far end */
    v42_state_t *v42;
    v42bis_state_t *v42bis;
    /* What the XID exchange agreed, which is what V.42bis actually runs
     * with - possibly less than was offered, possibly one direction only,
     * possibly nothing. Until the link is up, the offer. */
    bool comp_tx;
    bool comp_rx;
    /* The far end never answered our XID, so nothing was agreed - but some
     * far ends compress anyway. Until it shows its hand, received data goes
     * to the terminal as it is and through the decompressor too, its output
     * thrown away, so that the dictionary is in step if the far end turns
     * out to be compressing. See v42bis_shadow(). */
    bool rx_shadow;
    bool rx_shadow_esc;       /* a 0x00 is held: the next octet decides */
    bool rx_discard;          /* the decompressor's output is not wanted */
    int v42bis_dict;
    int v42bis_max_string;
    bool lapm_up;
    bool lapm_lost;            /* LAPM was up and has closed: the call is ending */
    bool rx_busy;              /* LAPM told the far end to wait: our receive queue is full */
    bool v42_fell_back;        /* gave up on V.42 and went back to raw async */
    int v42_timeout_s;
    int64_t v42_deadline_ms;   /* 0 until the carrier is up */
    int64_t v42_retry_at_ms;   /* 0 = no establishment retry pending */
    unsigned v42_establish_tries;
    unsigned v42_refusals;     /* DM answers to our SABME */
    bool v42_establish_failed; /* the far end does V.42; we could not agree */
    bool v42_adp_declined;     /* its detection pattern asked for no error correction */
    int drop_ceiling_to;       /* >0: re-offer this rate at the next safe moment */
    bool rate_pinned;          /* the ceiling has been dropped once already */
    bool rate_proven;          /* and the framing came good afterwards */
    bool rate_complained;      /* and it did not, and we have said so once */
    int rate_bad_windows;      /* consecutive windows of bad framing */
    int64_t rate_check_ms;     /* when the current rate-agreement window opened */
    int rate_check_errors;     /* framing errors at the start of it */
    uint64_t rate_check_bytes;
    unsigned frame_errors;
    dm_ring_t pending_rx;      /* framed while V.42 is still undecided */
    uint64_t wire_tx;
    uint64_t wire_rx;
    dm_ring_t comp_out;        /* compressor output waiting for a frame */

    /* Our own asynchronous transmit framer, see tx_get_bit(). */
    int tx_idle_bits;          /* mark to send before the next character */
    int tx_bitpos;
    unsigned tx_byte;
    int tx_parity;

    /* State. */
    dm_modem_phase_t phase;
    bool armed;              /* the call is up; the clock may run */
    bool pump_started;
    bool connected;
    bool carrier_lost;
    bool answer_tone_seen;

    /* Stepping down. Until the far end has answered in some modulation's
     * terms we are hunting: listening for what it is, and on the answering
     * side offering one modulation after another. See hunt_tick(). */
    dm_mod_t ceiling;          /* --modulation: the fastest we will try */
    bool step_down;
    int rate_cap;              /* --bit-rate, 0 = none */
    bool hunting;
    dm_listen_t listen;
    long long samples;         /* line time since the modem was armed */
    bool switch_due;           /* change to switch_to at the next frame */
    dm_mod_t switch_to;
    char switch_why[96];
    bool switch_confirmed;     /* switch_to is FSK, and its far end has been heard already */
    bool no_v8;                /* V.34 found no V.8 at the far end */
    bool v8_stalled;           /* or V.8 started and went unanswered */
    bool switch_listen;        /* calling V.32: listen for AC, send no AA of our own */
    bool aa_heard;             /* answering: a V.32 caller's AA, at any point */
    int window;                /* answering: which of ANSWER_CYCLE is on offer, -1 = V.8 */
    long window_left;          /* samples before offering the next */
    bool offered_v21;          /* answering: V.21's channel 2 has been on the line */
    bool offered_bell;         /* and Bell 103's 2225 Hz */
    bool fsk_carrier;
    bool fsk_confirmed;        /* the far end's FSK tone has been heard, not just energy */
    bool fsk_announced;        /* calling: carrier up before the far end's, see hunt_heard() */
    long long fsk_mark_from;   /* calling: when our own carrier went out, -1 = not yet */
    long long answer_hold_until; /* answering: mark until then, with no V.42, see tx_get_bit() */
    size_t tx_limit;           /* DM_TX_SECONDS of the current line rate, in bytes */
    int bit_rate;
    unsigned retrains;
    int64_t connect_ms;
    /* DATAMODEM_V32_RENEGOTIATE, a test hook: ask for this rate this long
     * after the carrier came up. */
    int reneg_after_ms;
    int reneg_rate;
    int64_t carrier_down_ms;  /* 0 unless the carrier is currently missing */
    int64_t line_busy_until_ms; /* estimated: when the line finishes what it has */
    int64_t last_rx_ms;
    uint64_t bytes_tx;
    uint64_t bytes_rx;
    bool wake_pending;

    /* DATAMODEM_RECORD: the call's audio, both ways. See record_open(). */
    FILE *rec;
    uint32_t rec_frames;
    int16_t rec_tx[1024];
    int rec_tx_n;

    dm_ring_t tx;
    dm_ring_t rx;
    void (*wake)(void *user);
    void *wake_user;
};

static int g_spandsp_level = SPAN_LOG_WARNING;

/* ------------------------------------------------------------- plumbing */

void dm_modem_init_logging(const dm_config_t *cfg)
{
    if (cfg->spandsp_log_level >= 0)
    {
        g_spandsp_level = cfg->spandsp_log_level;
    }
    else
    {
        switch (cfg->log_level)
        {
        case DM_LOG_ERROR:
            g_spandsp_level = SPAN_LOG_ERROR;
            break;
        case DM_LOG_WARN:
            g_spandsp_level = SPAN_LOG_WARNING;
            break;
        case DM_LOG_INFO:
            g_spandsp_level = SPAN_LOG_PROTOCOL_WARNING;
            break;
        case DM_LOG_DEBUG:
            g_spandsp_level = SPAN_LOG_FLOW_2;
            break;
        default:
            g_spandsp_level = SPAN_LOG_DEBUG_2;
            break;
        }
    }
    span_set_message_handler(dm_log_spandsp_message);
    span_set_error_handler(dm_log_spandsp_error);

    /* DM_LAPM_V42_UNSUPPORTED is copied out of spandsp's internals. If a
     * future spandsp renumbers it, say so rather than silently stop
     * noticing that the far end declined V.42. */
    {
        const char *name = lapm_status_to_str(DM_LAPM_V42_UNSUPPORTED);

        if (name == NULL || strstr(name, "UNSUPPORTED") == NULL)
            DM_WARN("modem", "this spandsp reports status %d as \"%s\", not V.42-unsupported; "
                             "falling back on the --v42-timeout backstop instead",
                    DM_LAPM_V42_UNSUPPORTED, name ? name : "?");
        name = lapm_status_to_str(DM_LAPM_IDLE);
        if (name == NULL || strstr(name, "IDLE") == NULL || lapm_status_to_str(DM_LAPM_DETECT) == NULL ||
            strstr(lapm_status_to_str(DM_LAPM_DETECT), "DETECT") == NULL)
            DM_WARN("modem", "this spandsp numbers its LAPM states differently; V.42 cut short by a "
                             "retrain may restart from the wrong place");
    }
}

static void attach_logging(logging_state_t *lg, const char *tag)
{
    if (lg == NULL)
        return;
    span_log_set_message_handler(lg, dm_log_spandsp_message);
    span_log_set_error_handler(lg, dm_log_spandsp_error);
    span_log_set_level(lg, SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_PROTOCOL | SPAN_LOG_SHOW_TAG |
                               g_spandsp_level);
    if (tag != NULL && *tag != '\0')
        span_log_set_tag(lg, tag);
}

/* --------------------------------------------------------- option tables */

static bool parse_modulation(const char *s, dm_mod_t *out)
{
    if (s == NULL || *s == '\0' || strcasecmp(s, "v22bis") == 0 || strcasecmp(s, "v.22bis") == 0)
        *out = DM_MOD_V22BIS;
    else if (strcasecmp(s, "v22") == 0 || strcasecmp(s, "v.22") == 0)
        *out = DM_MOD_V22;
    else if (strcasecmp(s, "v21") == 0 || strcasecmp(s, "v.21") == 0)
        *out = DM_MOD_V21;
    else if (strcasecmp(s, "bell103") == 0 || strcasecmp(s, "bell-103") == 0)
        *out = DM_MOD_BELL103;
    else if (strcasecmp(s, "v23") == 0 || strcasecmp(s, "v.23") == 0)
        *out = DM_MOD_V23;
    else if (strcasecmp(s, "v32") == 0 || strcasecmp(s, "v.32") == 0)
        *out = DM_MOD_V32;
    else if (strcasecmp(s, "v32bis") == 0 || strcasecmp(s, "v.32bis") == 0)
        *out = DM_MOD_V32BIS;
    else if (strcasecmp(s, "v34") == 0 || strcasecmp(s, "v.34") == 0)
        *out = DM_MOD_V34;
    else
        return false;
    return true;
}

static const char *modulation_name(dm_mod_t m)
{
    switch (m)
    {
    case DM_MOD_V22BIS:
        return "v22bis";
    case DM_MOD_V22:
        return "v22";
    case DM_MOD_V21:
        return "v21";
    case DM_MOD_BELL103:
        return "bell103";
    case DM_MOD_V23:
        return "v23";
    case DM_MOD_V32:
        return "v32";
    case DM_MOD_V32BIS:
        return "v32bis";
    case DM_MOD_V34:
        return "v34";
    }
    return "?";
}

/* V.32 bis is V.32's pump with more rates; everything around it is the same. */
static bool is_v32(dm_mod_t m)
{
    return m == DM_MOD_V32 || m == DM_MOD_V32BIS;
}

static bool is_fsk(dm_mod_t m)
{
    return m == DM_MOD_V21 || m == DM_MOD_BELL103 || m == DM_MOD_V23;
}

/* Whether V.42 runs over this modulation at all. Not over V.23, whose 75 bps
 * back channel takes 14 seconds to send one LAPM frame against a one second
 * acknowledgement timer; and not over the 300 bps FSK modems either, where
 * detection alone puts several seconds of DC1s on the line - typed input, to
 * the many 300 bps hosts that do not do V.42, a Cisco MICA among them. */
static bool carries_v42(dm_mod_t m)
{
    return !is_fsk(m);
}

static bool parse_parity(const char *s, int *out)
{
    if (s == NULL || *s == '\0' || strcasecmp(s, "none") == 0 || strcasecmp(s, "n") == 0)
        *out = ASYNC_PARITY_NONE;
    else if (strcasecmp(s, "even") == 0 || strcasecmp(s, "e") == 0)
        *out = ASYNC_PARITY_EVEN;
    else if (strcasecmp(s, "odd") == 0 || strcasecmp(s, "o") == 0)
        *out = ASYNC_PARITY_ODD;
    else
        return false;
    return true;
}

static bool parse_v42_mode(const char *s, dm_v42_mode_t *out)
{
    if (s == NULL || *s == '\0' || strcasecmp(s, "off") == 0 || strcasecmp(s, "none") == 0 ||
        strcasecmp(s, "false") == 0 || strcasecmp(s, "no") == 0)
        *out = DM_V42_OFF;
    else if (strcasecmp(s, "detect") == 0 || strcasecmp(s, "auto") == 0 || strcasecmp(s, "on") == 0 ||
             strcasecmp(s, "true") == 0 || strcasecmp(s, "yes") == 0)
        *out = DM_V42_DETECT;
    else if (strcasecmp(s, "require") == 0 || strcasecmp(s, "required") == 0)
        *out = DM_V42_REQUIRE;
    else
        return false;
    return true;
}

static bool parse_guard(const char *s, int *out)
{
    if (s == NULL || *s == '\0' || strcasecmp(s, "none") == 0 || strcmp(s, "0") == 0)
        *out = V22BIS_GUARD_TONE_NONE;
    else if (strcmp(s, "550") == 0)
        *out = V22BIS_GUARD_TONE_550HZ;
    else if (strcmp(s, "1800") == 0)
        *out = V22BIS_GUARD_TONE_1800HZ;
    else
        return false;
    return true;
}

/* V.21, Bell 103 and V.23 all split the line into a channel per direction,
 * and which channel is yours depends on who dialled. V.23 is the odd one:
 * its channels are not the same speed, so the calling terminal transmits at
 * 75 bps and receives at 1200 - which is exactly right for videotex, and
 * useless for anything symmetric. */
static const fsk_spec_t *fsk_spec_for(dm_mod_t mod, bool calling, bool transmit)
{
    bool ours = (calling == transmit); /* our tx channel when calling, our rx when answering */

    switch (mod)
    {
    case DM_MOD_V21:
        /* V.21 channel 1 (980/1180 Hz) belongs to the calling modem. */
        return &preset_fsk_specs[ours ? FSK_V21CH1 : FSK_V21CH2];
    case DM_MOD_BELL103:
        /* Bell 103 numbers its channels the other way round from V.21, and
         * so does spandsp: its "Bell103 ch 1" is the ANSWERING band
         * (2025/2225 Hz) and "ch 2" is the originating one (1070/1270 Hz).
         * Getting this backwards is invisible in any test where both ends
         * are this program - both sides invert together and talk happily -
         * and produces two modems shouting into each other's bands the
         * first time one of them meets a real Bell 103. */
        return &preset_fsk_specs[ours ? FSK_BELL103CH2 : FSK_BELL103CH1];
    case DM_MOD_V23:
        /* Channel 1 is the 1200 bps forward channel, from the answering end. */
        return &preset_fsk_specs[ours ? FSK_V23CH2 : FSK_V23CH1];
    default:
        return NULL;
    }
}

static int fsk_rate_for(dm_mod_t mod, bool calling, bool transmit)
{
    if (mod != DM_MOD_V23)
        return 300;
    return (calling == transmit) ? 75 : 1200;
}

static int ans_tone_type(dm_mod_t mod)
{
    /* A Bell 103 answering modem has no separate answer tone: its 2225 Hz
     * mark carrier is the answer tone. */
    return (mod == DM_MOD_BELL103) ? MODEM_CONNECT_TONES_BELL_ANS : MODEM_CONNECT_TONES_ANS;
}

/* What --bit-rate 0, the default, means for each modulation: the most it
 * can do. */
static int default_rate(dm_mod_t mod)
{
    switch (mod)
    {
    case DM_MOD_V34:
        return 33600;
    case DM_MOD_V32BIS:
        return 14400;
    case DM_MOD_V32:
        return 9600;
    case DM_MOD_V22BIS:
        return 2400;
    case DM_MOD_V22:
        return 1200;
    default:
        return 300;
    }
}

/* The most a modulation will run at under a --bit-rate ceiling: the cap
 * itself when the modulation has that rate, else the next one down. 0 is no
 * ceiling. */
static int rate_for(dm_mod_t mod, int cap)
{
    static const int v32bis[] = { 14400, 12000, 9600, 7200, 4800 };
    int best = default_rate(mod);

    if (cap <= 0 || cap >= best)
        return best;
    switch (mod)
    {
    case DM_MOD_V34:
        return cap >= 2400 ? cap / 2400 * 2400 : 2400;
    case DM_MOD_V32BIS:
        for (size_t i = 0; i < sizeof(v32bis) / sizeof(v32bis[0]); i++)
            if (v32bis[i] <= cap)
                return v32bis[i];
        return 4800;
    case DM_MOD_V32:
        return 4800;
    case DM_MOD_V22BIS:
        return 1200;
    default:
        return best;
    }
}

/* ------------------------------------------------------------ stepping down
 *
 * Fastest first. V.32 bis and V.22 bis each already talk to the modulation
 * below them - a V.32 bis modem trains with a V.32 one at 9600, and V.22 bis
 * with V.22 at 1200 - so between them they cover V.32 and V.22 too; the two
 * are rungs of their own only for a --modulation that starts there. V.21
 * and Bell 103 are both 300 bps, in different bands. V.23 is on no ladder:
 * its 75 bps back channel is nothing anyone wants to fall back to, and
 * nothing falls back to it. */
static const dm_mod_t LADDER[] = { DM_MOD_V34, DM_MOD_V32BIS, DM_MOD_V32, DM_MOD_V22BIS,
                                   DM_MOD_V22, DM_MOD_V21,    DM_MOD_BELL103 };

static int rung(dm_mod_t mod)
{
    for (int i = 0; i < (int) (sizeof(LADDER) / sizeof(LADDER[0])); i++)
        if (LADDER[i] == mod)
            return i;
    return -1;
}

static bool may_run(const dm_modem_t *m, dm_mod_t mod)
{
    if (mod == m->ceiling)
        return true;
    return m->step_down && rung(m->ceiling) >= 0 && rung(mod) > rung(m->ceiling);
}

/* Which of a pair - the better first - this modem may run, or -1. */
static int pick(const dm_modem_t *m, dm_mod_t better, dm_mod_t other)
{
    if (may_run(m, better))
        return (int) better;
    if (may_run(m, other))
        return (int) other;
    return -1;
}

static int pick_v32(const dm_modem_t *m)
{
    return pick(m, DM_MOD_V32BIS, DM_MOD_V32);
}

static int pick_v22(const dm_modem_t *m)
{
    return pick(m, DM_MOD_V22BIS, DM_MOD_V22);
}

static bool is_v22(dm_mod_t mod)
{
    return mod == DM_MOD_V22BIS || mod == DM_MOD_V22;
}

/* What an answering modem offers, in turn, to a caller that has not said
 * what it is. V.32 bis Annex A's automode for the first two: V.22 bis's
 * unscrambled ones for Ta = 3 s, listening for S1 or SB1 in the low band,
 * then V.32's AC. Then V.21's and Bell 103's answering carriers, which are
 * what callers at 300 bps wait to hear before they say anything. Round again
 * after that, until the session's --train-timeout. Each offer is only
 * withdrawn while nobody is answering it. */
static const dm_mod_t ANSWER_CYCLE[] = { DM_MOD_V22BIS, DM_MOD_V32BIS, DM_MOD_V21, DM_MOD_BELL103 };
#define DM_ANSWER_CYCLE_LEN ((int) (sizeof(ANSWER_CYCLE) / sizeof(ANSWER_CYCLE[0])))
#define DM_WINDOW_MS 3000

/* The modulation this modem would run for one of ANSWER_CYCLE, or -1. */
static int cycle_mod(const dm_modem_t *m, int i)
{
    switch (ANSWER_CYCLE[i])
    {
    case DM_MOD_V22BIS:
        return pick_v22(m);
    case DM_MOD_V32BIS:
        return pick_v32(m);
    default:
        return may_run(m, ANSWER_CYCLE[i]) ? (int) ANSWER_CYCLE[i] : -1;
    }
}

static int cycle_index(dm_mod_t mod)
{
    if (is_v22(mod))
        return 0;
    if (is_v32(mod))
        return 1;
    return mod == DM_MOD_V21 ? 2 : 3;
}

/* Is there anywhere to step down to? */
static bool can_step_down(const dm_modem_t *m)
{
    if (!m->step_down || rung(m->ceiling) < 0)
        return false;
    return rung(m->ceiling) < rung(DM_MOD_BELL103);
}

/* --------------------------------------------------------------- listening
 *
 * Which modem is at the far end, from what it sends before it has heard
 * anything it recognises: tones, mostly, at frequencies that do not overlap.
 * A Goertzel filter per frequency over each 40 ms block, against the block's
 * whole power less the echo of whatever we are sending ourselves; a signal
 * counts once it holds for several blocks running. */

/* Power at f, scaled so that a tone of amplitude A reads A^2 - twice the mean
 * square it contributes. */
static float goertzel(const int16_t *x, int n, float f)
{
    float c = 2.0f * cosf(2.0f * (float) M_PI * f / DM_SAMPLE_RATE);
    float s1 = 0.0f;
    float s2 = 0.0f;

    for (int i = 0; i < n; i++)
    {
        float s0 = (float) x[i] + c * s1 - s2;

        s2 = s1;
        s1 = s0;
    }
    return (s1 * s1 + s2 * s2 - c * s1 * s2) * 4.0f / ((float) n * (float) n);
}

/* The strongest tone between lo and hi, in 5 Hz steps: a far end's crystal
 * or a long-distance carrier system may have moved it from nominal. */
static float tone_peak(const int16_t *x, int n, float lo, float hi, float *at)
{
    float best = 0.0f;

    *at = lo;
    for (float f = lo; f <= hi + 0.1f; f += 5.0f)
    {
        float p = goertzel(x, n, f);

        if (p > best)
        {
            best = p;
            *at = f;
        }
    }
    return best;
}

/* The power between lo and hi, less the echo of our own tones: over a
 * 320-sample block the Goertzel bins every 25 Hz are orthogonal, and between
 * them add up to all of it. */
static float band_power(const dm_listen_t *l, float lo, float hi)
{
    float sum = 0.0f;

    for (float f = lo; f <= hi + 0.1f; f += 25.0f)
    {
        bool own = false;

        for (int i = 0; i < l->n_own; i++)
            own = own || fabsf(f - l->own[i]) < 12.5f;
        if (!own)
            sum += goertzel(l->buf, DM_LISTEN_BLOCK, f);
    }
    return sum;
}

static void listen_reset(dm_listen_t *l)
{
    l->n = 0;
    memset(l->run, 0, sizeof(l->run));
    l->v21_sum = l->bell_sum = 0.0f;
    l->fsk_blocks = 0;
}

static void listen_set_own(dm_listen_t *l, float a, float b)
{
    l->n_own = 0;
    if (a > 0.0f)
        l->own[l->n_own++] = a;
    if (b > 0.0f)
        l->own[l->n_own++] = b;
}

/* Whether we are sending anything between lo and hi ourselves: a signal
 * there would only be our own echo, and is not looked for. */
static bool own_between(const dm_listen_t *l, float lo, float hi)
{
    for (int i = 0; i < l->n_own; i++)
        if (l->own[i] >= lo - 25.0f && l->own[i] <= hi + 25.0f)
            return true;
    return false;
}

/* A block quieter than -48 dBm0 has nothing in it worth naming. */
#define DM_LISTEN_FLOOR 4100.0f
/* How much of what is left once our own echo is set aside a signal's tones
 * have to account for. A clean tone is over 0.9; noise, speech and
 * wideband data are nowhere near half. */
#define DM_LISTEN_SHARE 0.5f

static heard_t listen_block(dm_listen_t *l)
{
    const int16_t *x = l->buf;
    const int n = DM_LISTEN_BLOCK;
    float ms = 0.0f;
    float whole;
    float at;
    float a;
    float b;

    for (int i = 0; i < n; i++)
        ms += (float) x[i] * (float) x[i];
    ms /= (float) n;
    if (ms < DM_LISTEN_FLOOR)
        return HEARD_NONE;
    whole = 2.0f * ms;
    for (int i = 0; i < l->n_own; i++)
        whole -= goertzel(x, n, l->own[i]);
    if (whole < 2.0f * DM_LISTEN_FLOOR)
        return HEARD_NONE;      /* nothing but our own echo */

    /* AC: both of its tones, neither of them incidental. */
    a = goertzel(x, n, 600.0f);
    b = goertzel(x, n, 3000.0f);
    if (!own_between(l, 600.0f, 600.0f) && a + b >= DM_LISTEN_SHARE * whole && a >= 0.15f * whole &&
        b >= 0.15f * whole)
        return HEARD_AC;
    if (!own_between(l, 1780.0f, 1820.0f) && tone_peak(x, n, 1780.0f, 1820.0f, &at) >= DM_LISTEN_SHARE * whole)
        return HEARD_AA;
    if (!own_between(l, 2080.0f, 2120.0f) && tone_peak(x, n, 2080.0f, 2120.0f, &at) >= DM_LISTEN_SHARE * whole)
        return HEARD_ANS;

    /* Between 2190 and 2275 Hz is either Bell 103's answering mark, a pure
     * tone at 2225, or V.22's unscrambled ones: 600 baud of the same phase
     * step, which is most of its power at 2250 and, from some modems, a
     * fourteenth of it 600 Hz higher. That second line settles it when it
     * is there - but it is only the pulse shaping's imperfection, and a
     * well-filtered modem's USB1 is a pure 2250 Hz tone (one answering for
     * Diamond Mine was, and went unanswered for two seconds). Without the
     * line, the frequency decides: a tone this long is found to within a
     * few hertz, and the two are 25 Hz apart. */
    a = own_between(l, 2005.0f, 2275.0f) ? 0.0f : tone_peak(x, n, 2190.0f, 2275.0f, &at);
    if (a >= DM_LISTEN_SHARE * whole)
    {
        float side = goertzel(x, n, at + 600.0f);

        if (at >= 2235.0f && at <= 2265.0f && side >= 0.02f * a)
            return HEARD_USB1;
        if (side < 0.01f * a)
        {
            if (at <= 2235.0f)
                return HEARD_BELL_ANS;
            if (at >= 2240.0f && at <= 2265.0f)
                return HEARD_USB1;
        }
        return HEARD_NONE;
    }
    /* Bell 103 carrying data, its power spread around 2025 and 2225 Hz. */
    if (!own_between(l, 2005.0f, 2275.0f) && band_power(l, 1925.0f, 2325.0f) >= DM_LISTEN_SHARE * whole &&
        goertzel(x, n, 2850.0f) < 0.01f * whole &&
        tone_peak(x, n, 2005.0f, 2045.0f, &at) + a >= 0.2f * whole)
        return HEARD_BELL_ANS;

    /* V.21 channel 2 idling at mark, or carrying data: then the power is
     * spread around both its tones rather than at them. */
    if (!own_between(l, 1630.0f, 1670.0f) && !own_between(l, 1830.0f, 1870.0f) &&
        band_power(l, 1550.0f, 1950.0f) >= DM_LISTEN_SHARE * whole &&
        (own_between(l, 1800.0f, 1800.0f) || goertzel(x, n, 1800.0f) < 0.4f * whole) &&
        tone_peak(x, n, 1630.0f, 1670.0f, &at) + tone_peak(x, n, 1830.0f, 1870.0f, &at) >= 0.2f * whole)
        return HEARD_V21_ANS;

    /* V.23 idling at mark: 1300 Hz forward, 390 Hz backward. Nothing else
     * either end sends sits at 390. 1300 is 30 Hz above Bell 103's calling
     * mark, which over a 40 ms block puts a quarter of its power at 1285, so
     * a tone peaking from there up is V.23's. Only the mark is looked for:
     * both ends idle at it until they are connected. */
    if (!own_between(l, 1285.0f, 1320.0f) && tone_peak(x, n, 1285.0f, 1320.0f, &at) >= DM_LISTEN_SHARE * whole)
        return HEARD_V23_ANS;
    if (!own_between(l, 370.0f, 410.0f) && tone_peak(x, n, 370.0f, 410.0f, &at) >= DM_LISTEN_SHARE * whole)
        return HEARD_V23_ORIG;

    /* The two calling bands overlap - V.21's 980 and 1180 Hz, Bell 103's
     * 1070 and 1270 - so the band says FSK and the tones say whose. Idling
     * at mark that is plain at once; carrying data, the power smears between
     * them, and listen_feed() decides over several blocks. */
    if (!own_between(l, 960.0f, 1290.0f) && band_power(l, 880.0f, 1370.0f) >= DM_LISTEN_SHARE * whole)
    {
        l->v21_tones = (tone_peak(x, n, 960.0f, 1000.0f, &at) + tone_peak(x, n, 1160.0f, 1200.0f, &at)) / whole;
        l->bell_tones = (tone_peak(x, n, 1050.0f, 1090.0f, &at) + tone_peak(x, n, 1250.0f, 1290.0f, &at)) / whole;
        return HEARD_LOW_FSK;
    }
    return HEARD_NONE;
}

static const char *heard_name(heard_t h)
{
    switch (h)
    {
    case HEARD_AC:
        return "V.32's AC";
    case HEARD_AA:
        return "V.32's AA";
    case HEARD_ANS:
        return "an answer tone";
    case HEARD_USB1:
        return "V.22's unscrambled ones";
    case HEARD_V21_ANS:
        return "V.21's answering carrier";
    case HEARD_BELL_ANS:
        return "Bell 103's answering carrier";
    case HEARD_V21_ORIG:
        return "V.21's calling carrier";
    case HEARD_BELL_ORIG:
        return "Bell 103's calling carrier";
    case HEARD_V23_ANS:
        return "V.23's answering carrier";
    case HEARD_V23_ORIG:
        return "V.23's calling carrier";
    default:
        return "nothing";
    }
}

/* Where to start. Stepping down, a --bit-rate below anything the
 * modulation can do starts at the fastest one that can: --bit-rate 1200 is
 * V.22 bis, not an error about V.34. */
static dm_mod_t start_mod(dm_mod_t mod, int cap, bool step_down)
{
    static const int slowest[] = { 2400, 4800, 4800, 1200, 1200, 300, 300 };
    int r = rung(mod);

    while (step_down && cap > 0 && r >= 0 && r + 1 < (int) (sizeof(LADDER) / sizeof(LADDER[0])) &&
           cap < slowest[r])
        r++;
    return r >= 0 ? LADDER[r] : mod;
}

bool dm_modem_params_check(const dm_modem_params_t *p, char *err, size_t err_len)
{
    dm_mod_t mod;
    int tmp;

    if (!parse_modulation(p->modulation, &mod))
    {
        snprintf(err, err_len, "unknown modulation '%s'; use v34, v32bis, v32, v22bis, v22, v21, bell103 or v23",
                 p->modulation ? p->modulation : "");
        return false;
    }
    mod = start_mod(mod, p->bit_rate, p->step_down);
    if (!parse_parity(p->parity, &tmp))
    {
        snprintf(err, err_len, "parity must be none, even or odd");
        return false;
    }
    if (!parse_guard(p->guard_tone, &tmp))
    {
        snprintf(err, err_len, "guard tone must be none, 550 or 1800");
        return false;
    }
    if (mod == DM_MOD_V22BIS && p->bit_rate != 0 && p->bit_rate != 1200 && p->bit_rate != 2400)
    {
        snprintf(err, err_len, "V.22bis runs at 1200 or 2400 bps, not %d", p->bit_rate);
        return false;
    }
    if (mod == DM_MOD_V32 && p->bit_rate != 0 && p->bit_rate != 4800 && p->bit_rate != 9600)
    {
        snprintf(err, err_len, "V.32 runs at 9600 or 4800 bps, not %d (2400 is left for further "
                               "study by the Recommendation)", p->bit_rate);
        return false;
    }
    if (mod == DM_MOD_V32BIS && p->bit_rate != 0 && p->bit_rate != 4800 && p->bit_rate != 7200 &&
        p->bit_rate != 9600 && p->bit_rate != 12000 && p->bit_rate != 14400)
    {
        snprintf(err, err_len, "V.32bis runs at 14400, 12000, 9600, 7200 or 4800 bps, not %d", p->bit_rate);
        return false;
    }
    if (mod == DM_MOD_V34 && p->bit_rate != 0 && (p->bit_rate < 2400 || p->bit_rate > 33600 || p->bit_rate % 2400))
    {
        snprintf(err, err_len, "V.34 runs at multiples of 2400 bps from 2400 to 33600, not %d", p->bit_rate);
        return false;
    }
    if (p->data_bits < 5 || p->data_bits > 8)
    {
        snprintf(err, err_len, "data bits must be 5, 6, 7 or 8");
        return false;
    }
    if (p->stop_bits != 1 && p->stop_bits != 2)
    {
        snprintf(err, err_len, "stop bits must be 1 or 2");
        return false;
    }

    {
        dm_v42_mode_t mode;

        if (!parse_v42_mode(p->v42, &mode))
        {
            snprintf(err, err_len, "--v42 must be off, detect or require");
            return false;
        }
        /* V.42 does not run over V.21, Bell 103 or V.23 - see carries_v42().
         * detect, the default, means "if it can", so dm_modem_create() runs
         * it as off there. Only require is a contradiction - and starting at
         * one of these, there is nothing faster to step down from. */
        if (mode == DM_V42_REQUIRE && !carries_v42(mod))
        {
            if (mod == DM_MOD_V23)
                snprintf(err, err_len,
                         "--v42 require cannot run over V.23: its 75 bps back channel takes 14 "
                         "seconds to send one LAPM frame, and the acknowledgement timer is one second");
            else
                snprintf(err, err_len, "--v42 require cannot run over %s: datamodem runs no V.42 at 300 bps",
                         modulation_name(mod));
            return false;
        }
        /* --v42bis with --v42 off is not an error: V.42bis is an offer made
         * inside V.42's XID exchange, and with no V.42 there is nothing to
         * make it in, so it is simply not offered. Compression without error
         * correction would not be safe anyway - V.42bis keeps a dictionary
         * that both ends build as they go, and one corrupted byte leaves the
         * two diverged for the rest of the call. */
        if (p->v42bis_dict < V42BIS_MIN_DICTIONARY_SIZE || p->v42bis_dict > V42BIS_MAX_CODEWORDS)
        {
            snprintf(err, err_len, "--v42bis-dict must be between %d and %d codewords",
                     V42BIS_MIN_DICTIONARY_SIZE, V42BIS_MAX_CODEWORDS);
            return false;
        }
        if (p->v42bis_max_string < V42BIS_MIN_STRING_SIZE ||
            p->v42bis_max_string > V42BIS_MAX_STRING_SIZE)
        {
            snprintf(err, err_len, "--v42bis-max-string must be between %d and %d",
                     V42BIS_MIN_STRING_SIZE, V42BIS_MAX_STRING_SIZE);
            return false;
        }
    }
    return true;
}

void dm_modem_params_from_config(const dm_config_t *cfg, bool calling, const char *tag,
                                 dm_modem_params_t *out)
{
    memset(out, 0, sizeof(*out));
    out->calling = calling;
    out->modulation = cfg->modulation;
    out->bit_rate = cfg->bit_rate;
    out->guard_tone = cfg->guard_tone;
    out->data_bits = cfg->data_bits;
    out->parity = cfg->parity;
    out->stop_bits = cfg->stop_bits;
    out->v14 = cfg->v14;
    out->tx_power = cfg->tx_power;
    out->answer_tone_ms = cfg->answer_tone_ms;
    out->answer_wait_s = cfg->answer_wait_s;
    out->answer_tail_ms = cfg->answer_tail_ms;
    out->v42 = cfg->v42;
    out->v42_timeout_s = cfg->v42_timeout_s;
    /* A jitter buffer at each end, so the round trip is twice one of them.
     * This is what V.42's detection window has to be stretched by. */
    out->path_delay_ms = cfg->jitter_buffer_ms * 2;
    out->v42bis = cfg->v42bis;
    out->v42bis_dict = cfg->v42bis_dict;
    out->v42bis_max_string = cfg->v42bis_max_string;
    out->calling_tone = cfg->calling_tone;
    out->step_down = cfg->step_down;
    out->tag = tag;
}

/* ------------------------------------------------------- data pump glue */

static bool v42_undecided(const dm_modem_t *m)
{
    return m->v42 != NULL && !m->lapm_up && !m->v42_fell_back;
}

/* Everything received ends up here: framed characters from the async path,
 * or the payload of a LAPM frame. Called on the media thread with the lock
 * held. */
static void deliver_rx(dm_modem_t *m, const unsigned char *data, size_t len)
{
    if (len == 0)
        return;
    m->bytes_rx += len;
    m->last_rx_ms = dm_now_ms();
    dm_ring_write_overwrite(&m->rx, data, len);
    m->wake_pending = true;
}

/* Push the "line is busy until" estimate out by however long these bytes
 * will take to transmit. Called on the media thread with the lock held,
 * every time bytes leave our queues for the pump or for LAPM. */
static void charge_line_time(dm_modem_t *m, size_t bytes)
{
    int64_t now = dm_now_ms();
    int rate = m->tx_bit_rate > 0 ? m->tx_bit_rate : 300;

    /* LAPM asks for a frame constantly, and mostly gets nothing. Charging
     * those empty calls would drag the estimate forward to "now" on every
     * one of them, and the line would never be considered drained at all. */
    if (bytes == 0)
        return;

    if (m->line_busy_until_ms < now)
        m->line_busy_until_ms = now;
    m->line_busy_until_ms += (int64_t) bytes * DM_LINE_BITS_PER_BYTE * 1000 / rate;
}

/* The transmit side frames characters itself rather than using spandsp's
 * async_tx. async_tx signals SIG_STATUS_END_OF_DATA as soon as its get_byte
 * callback comes up empty, which tells the data pump to shut the carrier
 * down - correct for a fax burst, fatal for an interactive session where the
 * queue is empty most of the time. A real async modem holds the line at mark
 * between characters instead, which is all the difference below amounts to.
 *
 * Called from the media thread with the modem lock held. */
static int async_framer_get_bit(dm_modem_t *m)
{
    int parity_slot = (m->parity != ASYNC_PARITY_NONE) ? 1 : 0;
    int bit;

    if (m->tx_bitpos == 0)
    {
        int c;

        if (m->tx_idle_bits > 0)
        {
            m->tx_idle_bits--;
            return 1;
        }
        c = dm_ring_getc(&m->tx);

        if (c < 0)
            return 1; /* idle mark */
        m->tx_byte = (unsigned) c & ((1u << m->data_bits) - 1u);
        m->tx_parity = 0;
        m->tx_bitpos = 1;
        m->bytes_tx++;
        m->wire_tx++;
        charge_line_time(m, 1);
        return 0; /* start bit */
    }
    if (m->tx_bitpos <= m->data_bits)
    {
        bit = (int) (m->tx_byte & 1u);
        m->tx_byte >>= 1;
        m->tx_parity ^= bit;
        m->tx_bitpos++;
        return bit;
    }
    if (parity_slot && m->tx_bitpos == m->data_bits + 1)
    {
        m->tx_bitpos++;
        return (m->parity == ASYNC_PARITY_ODD) ? (m->tx_parity ^ 1) : m->tx_parity;
    }
    /* Stop bits, then back round for the next character. */
    if (m->tx_bitpos >= m->data_bits + parity_slot + m->stop_bits)
        m->tx_bitpos = 0;
    else
        m->tx_bitpos++;
    return 1;
}

/* The framing monitor's output is discarded - see arx_monitor. */
static void rx_monitor_byte(void *user, int byte)
{
    (void) user;
    (void) byte;
}

static void rx_put_byte(void *user, int byte)
{
    dm_modem_t *m = user;
    unsigned char c;

    if (byte < 0)
        return; /* async_rx forwards status codes here; handled in rx_status */
    /* Training patterns are scrambled ones, and the character framer will
     * happily turn them into characters. Nothing counts as data until the
     * modems have agreed that they are connected. */
    if (m->phase != DM_PHASE_DATA)
        return;
    c = (unsigned char) byte;
    if (v42_undecided(m))
    {
        /* Might be the far end's banner, might be its ODP pattern. We will
         * know which shortly; hold it until then. */
        dm_ring_write(&m->pending_rx, &c, 1);
        return;
    }
    m->wire_rx++;
    deliver_rx(m, &c, 1);
}

/* ------------------------------------------------------ V.42 and V.42bis
 *
 * Layering, outermost first:
 *
 *   terminal  <->  V.42bis  <->  LAPM (V.42)  <->  data pump  <->  line
 *
 * When V.42 is running, its bit stream replaces the asynchronous character
 * framing entirely - there are no start and stop bits on the line any more,
 * there are HDLC frames. So tx_get_bit and rx_put_bit below are switches
 * rather than the framer itself, and falling back to raw async is a matter
 * of flipping one flag rather than rewiring the pump. */

/* The V.42bis compressor produced some output. It goes into a staging queue
 * rather than straight into a frame, because a flush can produce more than
 * one frame's worth at once. */
static void v42bis_encoded(void *user, const uint8_t *buf, int len)
{
    dm_modem_t *m = user;

    if (len <= 0)
        return;
    if (dm_ring_write(&m->comp_out, buf, (size_t) len) != (size_t) len)
    {
        /* Would mean the headroom calculation above is wrong. Losing bytes
         * here desynchronises the far end's dictionary, so say so loudly
         * rather than let the session quietly turn to noise. */
        DM_ERROR("modem", "compressed output overran its queue (tag=%s); the link is no longer "
                          "trustworthy",
                 m->tag);
    }
}

/* The V.42bis decompressor recovered some plaintext. */
static void v42bis_decoded(void *user, const uint8_t *buf, int len)
{
    dm_modem_t *m = user;

    if (len > 0 && !m->rx_discard)
        deliver_rx(m, buf, (size_t) len);
}

static void v42bis_feed(dm_modem_t *m, const uint8_t *buf, size_t len, bool discard)
{
    if (len == 0)
        return;
    m->rx_discard = discard;
    v42bis_decompress(m->v42bis, buf, (int) len);
    v42bis_decompress_flush(m->v42bis);
    m->rx_discard = false;
}

/* Received data from a far end that never answered our XID.
 *
 * V.42bis starts in transparent mode, where data passes through unchanged,
 * and the only way out of it is the escape character - initially 0x00 -
 * followed by a command: 0 to enter compressed mode, 1 for a literal escape
 * character, 2 to reset. So the first 0x00 settles it. Followed by a command
 * code, the far end is running V.42bis whatever it did not say, and from
 * then on what it sends goes through the decompressor, whose dictionary has
 * been kept in step all along. Followed by anything else, it is not, and the
 * shadow is dropped.
 *
 * Met in the field: a BBS whose modem never answered XID, sent its banner in
 * the clear, then "00 00" and compressed codewords - which, uncompressed,
 * was a screen of garbage two lines in. */
static void v42bis_shadow(dm_modem_t *m, const uint8_t *msg, size_t len)
{
    size_t start = 0;

    for (size_t i = 0; i < len; i++)
    {
        uint8_t c = msg[i];

        if (m->rx_shadow_esc)
        {
            static const uint8_t esc = 0x00;

            m->rx_shadow_esc = false;
            m->rx_shadow = false;
            if (c <= 2)
            {
                /* Everything before the escape was transparent, and has
                 * been delivered and taught to the dictionary already. */
                m->comp_rx = true;
                DM_WARN("modem", "the far end never answered our XID but is sending V.42bis anyway; "
                                 "decompressing it with the parameters we offered (dictionary %d, "
                                 "strings up to %d) (tag=%s)",
                        m->v42bis_dict, m->v42bis_max_string, m->tag);
                v42bis_feed(m, &esc, 1, false);
                v42bis_feed(m, msg + i, len - i, false);
                return;
            }
            DM_DEBUG("modem", "the far end sent a 0x00 that is not a V.42bis escape; it is not "
                              "compressing (tag=%s)", m->tag);
            deliver_rx(m, &esc, 1);
            deliver_rx(m, msg + i, len - i);
            return;
        }
        if (c == 0x00)
        {
            /* Hold it: alone it means nothing yet. The decompressor is fed
             * up to here, and the 0x00 with whatever follows it. */
            deliver_rx(m, msg + start, i - start);
            v42bis_feed(m, msg + start, i - start, true);
            m->rx_shadow_esc = true;
            start = i + 1;
        }
    }
    deliver_rx(m, msg + start, len - start);
    v42bis_feed(m, msg + start, len - start, true);
}

/* LAPM wants a frame to send. */
static int v42_iframe_get(void *user, uint8_t *msg, int max_len)
{
    dm_modem_t *m = user;
    size_t n;

    if (max_len <= 0)
        return 0;

    if (m->v42bis != NULL && m->comp_tx)
    {
        if (dm_ring_space(&m->comp_out) >= DM_COMP_HEADROOM)
        {
            unsigned char plain[DM_COMP_CHUNK];

            n = dm_ring_read(&m->tx, plain, sizeof(plain));
            if (n > 0)
            {
                m->bytes_tx += n;
                v42bis_compress(m->v42bis, plain, (int) n);
                /* Not optional. Without the flush the compressor sits on a
                 * short burst - which is to say, on everything anyone types -
                 * waiting for more input that will not arrive until the far
                 * end has replied to the thing still stuck in the buffer. */
                v42bis_compress_flush(m->v42bis);
            }
        }
        n = dm_ring_read(&m->comp_out, msg, (size_t) max_len);
        m->wire_tx += n;
        charge_line_time(m, n);
        return (int) n;
    }

    n = dm_ring_read(&m->tx, msg, (size_t) max_len);
    m->bytes_tx += n;
    m->wire_tx += n;
    charge_line_time(m, n);
    return (int) n;
}

/* LAPM delivered a frame, already checked and in order. */
static void v42_iframe_put(void *user, const uint8_t *msg, int len)
{
    dm_modem_t *m = user;

    if (len <= 0)
        return; /* status codes are reported through here too */
    m->wire_rx += (size_t) len;
    if (m->v42bis != NULL && m->comp_rx)
    {
        v42bis_feed(m, msg, (size_t) len, false);
    }
    else if (m->v42bis != NULL && m->rx_shadow)
    {
        v42bis_shadow(m, msg, (size_t) len);
    }
    else
    {
        deliver_rx(m, msg, (size_t) len);
    }
}

static const char *protocol_name(const dm_modem_t *m)
{
    if (m->v42 == NULL || m->v42_fell_back)
        return "async";
    if (!m->lapm_up && !m->lapm_lost)
        return "negotiating";
    if (m->v42bis == NULL || (!m->comp_tx && !m->comp_rx))
        return "V.42";
    if (m->comp_tx && m->comp_rx)
        return "V.42/V.42bis";
    return m->comp_tx ? "V.42/V.42bis (sending only)" : "V.42/V.42bis (receiving only)";
}

/* spandsp measures every V.42 timer in bit periods - the detection timeout
 * T400, the LAPM retransmission timer T401, the idle poll T403 - and
 * converts from milliseconds with a transmit rate it hardcodes to 28800 bps
 * at init, with no API to change it.
 *
 * At 2400 bps that makes every timer twelve times too long. At 300 bps,
 * ninety-six. Two spandsp instances inflate by the same factor and so talk
 * to each other quite happily, which is exactly why this hides until you
 * meet real equipment: the far end's T401 fires on schedule, it retransmits
 * into a peer that will not answer for another ten or ninety seconds, it
 * runs out of retries, and the link never comes up.
 *
 * The field is a plain int in a structure we allocate ourselves, and there
 * is no other way to reach it. */
static int v42_round_trip_ms(dm_modem_t *m);

/* T401 has to outlast everything between sending a frame and hearing it
 * acknowledged: the frame itself going out, the round trip, the far end
 * thinking about it, and - if it was part way through a frame of its own -
 * that frame, which the acknowledgement has to wait behind. V.42's 1 s
 * assumes a phone line and a fast modem. On a 300 ms RTP path at 9600 it
 * is about right; at 300 bps a single 128-octet frame takes 3.5 s to send,
 * and an acknowledgement could not possibly come back inside a second. */
static void v42_size_t401(dm_modem_t *m, int rate)
{
#if defined(DATAMODEM_VENDORED_V42)
    int rtd_ms = v42_round_trip_ms(m);
    /* 128 octets of information, address, control, two of FCS and a flag,
     * plus an allowance for bit stuffing. */
    int frame_ms = (int) ((int64_t) (128 + 6) * 8 * 1100 / rate);
    int t401 = rtd_ms + 500 + 2 * frame_ms;

    if (t401 < 1000)
        t401 = 1000;
    dm_v42_set_t401(m->v42, t401);
    DM_DEBUG("modem", "V.42 T401 %d ms, for a %d ms round trip at %d bps (tag=%s)", t401, rtd_ms, rate,
             m->tag);
#else
    (void) m;
    (void) rate;
#endif
}

static void v42_set_bit_rate(dm_modem_t *m, int rate)
{
    if (m->v42 == NULL || rate <= 0)
        return;
    m->v42->tx_bit_rate = rate;
    v42_size_t401(m, rate);
    DM_DEBUG("modem", "V.42 timers scaled to %d bps (tag=%s)", rate, m->tag);
}

/* Arms the whole V.42 handshake for a line rate: the LAPM timers, then the
 * detection window on top of them.
 *
 * This has to be redone once the carrier is up, because until then the rate
 * is only an offer. V.22bis offers 2400 and may settle at 1200, and timers
 * armed for 2400 then expire twice as fast as V.42 allows: T401 runs out of
 * retries while the far end is still within its rights to be thinking, and
 * the link dies having trained perfectly. That is why fixing the rate on the
 * command line worked and letting it negotiate did not - the fixed rate was
 * the only way the timers ever matched the line.
 *
 * Only safe before any data has been exchanged: v42_restart() resets the
 * detection state machine along with everything else. */
/* The round trip the V.42 timers have to allow for. V.32 measures the real
 * one during its start-up, and by the time the carrier is up it is known;
 * for everything else, and before then, the two jitter buffers are the best
 * estimate there is. RTP paths of 400-800 ms have been met in practice. */
static int v42_round_trip_ms(dm_modem_t *m)
{
    if (m->v32 != NULL)
    {
        dm_v32_stats_t vs;

        dm_v32_stats(m->v32, &vs);
        if (vs.round_trip_ms > m->path_delay_ms)
            return vs.round_trip_ms;
    }
    if (m->v34 != NULL)
    {
        dm_v34_stats_t vs;

        dm_v34_stats(m->v34, &vs);
        if (vs.round_trip_ms > m->path_delay_ms)
            return vs.round_trip_ms;
    }
    return m->path_delay_ms;
}

static void v42_arm(dm_modem_t *m, int rate)
{
    int rtd_ms;

    if (m->v42 == NULL || rate <= 0)
        return;
    rtd_ms = v42_round_trip_ms(m);
    /* Before the restart, not after: v42_restart latches T400 from the rate,
     * and a timer already loaded from the wrong one stays wrong. */
    v42_set_bit_rate(m, rate);
    /* T401 has to outlast the round trip, or every acknowledgement - and the
     * XID answer that V.42bis rides on - comes back after we have given up
     * on it. v42_set_bit_rate() just sized it; see v42_size_t401(). */
    v42_restart(m->v42);
    /* v42_restart has just armed T400 from the rate above. Replace it with a
     * window sized for this line rate and this audio path - see
     * DM_V42_DETECT_SPEC_MS. */
    if (m->v42->bit_timer > 0)
    {
        int path_ms = DM_V42_DETECT_SPEC_MS + rtd_ms;
        int bits = 2 * DM_V42_DETECT_MIN_BITS + (int) ((int64_t) path_ms * rate / 1000);

        DM_DEBUG("modem", "V.42 detection window %d -> %d bit periods (%.2fs at %d bps, "
                          "allowing %dms for the audio path) (tag=%s)",
                 (int) m->v42->bit_timer, bits, (double) bits / rate, rate, rtd_ms, m->tag);
        m->v42->bit_timer = bits;
    }
}

/* Schedules another attempt at LAPM establishment. Returns false when there
 * is no point: no vendored V.42 to drive it, or --v42-timeout has no room
 * left for an attempt plus a reasonable chance of it being answered. */
static bool v42_retry_establishment(dm_modem_t *m, const char *why)
{
#if defined(DATAMODEM_VENDORED_V42)
    int64_t now = dm_now_ms();
    bool refused = (why != NULL && strstr(why, "DM") != NULL);

    if (m->v42_deadline_ms == 0 || m->v42_retry_at_ms != 0)
        return false;
    if (refused && ++m->v42_refusals > DM_V42_MAX_REFUSALS)
    {
        DM_INFO("modem", "the far end has refused establishment %u times; it means it "
                         "(tag=%s)", m->v42_refusals, m->tag);
        return false;
    }
    /* Room for the wait and for the far end to answer within T401. */
    if (now + DM_V42_RETRY_MS + 1000 > m->v42_deadline_ms)
        return false;

    m->v42_retry_at_ms = now + DM_V42_RETRY_MS;
    DM_INFO("modem", "V.42 establishment attempt %u failed%s%s; trying again in %dms "
                     "(%.1fs of --v42-timeout left) (tag=%s)",
            m->v42_establish_tries + 1,
            (why != NULL && why[0] != '\0') ? " - " : "",
            (why != NULL) ? why : "", DM_V42_RETRY_MS,
            (double) (m->v42_deadline_ms - now) / 1000.0, m->tag);
    return true;
#else
    (void) m;
    (void) why;
    return false;
#endif
}

/* LAPM is up: make V.42bis whatever the XID exchange agreed, before a byte
 * has gone through it.
 *
 * V.42bis is negotiated there and nowhere else. The far end may take a
 * smaller dictionary or shorter strings than we offered, compress in one
 * direction only, or not at all - and if no XID was exchanged at all, V.42bis
 * says compression is off. Running the compressor on what we merely offered,
 * as this used to, hands a far end that agreed to less a stream it cannot
 * decode. Called from v42_status() with the lock held. */
static void apply_v42bis_agreement(dm_modem_t *m)
{
#if defined(DATAMODEM_VENDORED_V42)
    int p0;
    int p1;
    int p2;

    if (m->v42bis == NULL)
        return;
    if (!dm_v42_xid_done(m->v42))
    {
        m->comp_tx = m->comp_rx = false;
        /* Our own direction stays uncompressed: that is what V.42bis says
         * no negotiation means, and a far end that is decompressing anyway
         * passes it through untouched in transparent mode. The other way, be
         * ready for it to compress regardless - some do. */
        m->rx_shadow = true;
        m->rx_shadow_esc = false;
        if (getenv("DATAMODEM_V42BIS_REGARDLESS") != NULL)
        {
            /* Test hook: be that far end - compress, and expect compression,
             * on nothing but our own offer. */
            m->comp_tx = m->comp_rx = true;
            m->rx_shadow = false;
            DM_WARN("modem", "DATAMODEM_V42BIS_REGARDLESS: running V.42bis without an agreement. "
                             "This is a test hook.");
            return;
        }
        DM_WARN("modem", "the far end never answered our XID, so V.42bis was not negotiated and "
                         "this link runs uncompressed (tag=%s)", m->tag);
        /* Two very different situations look the same from here: a far end
         * that does not do XID, and one that answered over a path that
         * damaged the answer. The second will damage the data too. */
        if (dm_v42_fcs_errors > 0)
            DM_WARN("modem", "%u LAPM frame%s arrived damaged meanwhile, so it may well have "
                             "answered - this path is corrupting frames, which will hurt the data "
                             "as well (tag=%s)",
                    dm_v42_fcs_errors, dm_v42_fcs_errors == 1 ? "" : "s", m->tag);
        return;
    }
    p0 = m->v42->config.comp & 3;
    p1 = m->v42->config.comp_dict_size;
    p2 = m->v42->config.comp_max_string;
    /* P0 is written from the side of whoever sent the XID command, which is
     * the calling modem: bit 0 is caller to answerer, bit 1 the reverse. */
    m->comp_tx = (p0 & (m->calling ? 1 : 2)) != 0;
    m->comp_rx = (p0 & (m->calling ? 2 : 1)) != 0;
    if (!m->comp_tx && !m->comp_rx)
    {
        DM_WARN("modem", "the far end declined V.42bis in the XID exchange; error correction "
                         "only (tag=%s)", m->tag);
        return;
    }
    if (p1 != m->v42bis_dict || p2 != m->v42bis_max_string)
    {
        v42bis_state_t *agreed;

        if (p1 < V42BIS_MIN_DICTIONARY_SIZE || p1 > V42BIS_MAX_CODEWORDS ||
            p2 < V42BIS_MIN_STRING_SIZE || p2 > V42BIS_MAX_STRING_SIZE)
        {
            m->comp_tx = m->comp_rx = false;
            DM_WARN("modem", "the far end agreed V.42bis with a dictionary of %d and strings of %d, "
                             "which V.42bis does not allow; running uncompressed (tag=%s)",
                    p1, p2, m->tag);
            return;
        }
        agreed = v42bis_init(NULL, V42BIS_P0_BOTH_DIRECTIONS, p1, p2, v42bis_encoded, m,
                             DM_LAPM_MAX_FRAME, v42bis_decoded, m, V42BIS_MAX_OUTPUT_LENGTH);
        if (agreed == NULL)
        {
            m->comp_tx = m->comp_rx = false;
            DM_ERROR("modem", "could not set V.42bis up as agreed; running uncompressed (tag=%s)", m->tag);
            return;
        }
        v42bis_compression_control(agreed, V42BIS_COMPRESSION_MODE_DYNAMIC);
        v42bis_release(m->v42bis);
        v42bis_free(m->v42bis);
        m->v42bis = agreed;
        m->v42bis_dict = p1;
        m->v42bis_max_string = p2;
    }
    DM_INFO("modem", "V.42bis agreed: dictionary %d, strings up to %d, %s (tag=%s)", p1, p2,
            (m->comp_tx && m->comp_rx) ? "both directions"
                                       : (m->comp_tx ? "our direction only" : "their direction only"),
            m->tag);
#else
    /* The system libspandsp neither sends XID nor applies one, so there is
     * no agreement to apply; both ends must simply be configured alike. */
    m->comp_tx = m->comp_rx = (m->v42bis != NULL);
#endif
}

static void v42_status(void *user, int status)
{
    dm_modem_t *m = user;

    /* Every transition, at debug, because when V.42 goes wrong against real
     * equipment the sequence of these is the only thing that says where. */
    DM_DEBUG("modem", "v42 status %d (%s) (tag=%s)", status,
             lapm_status_to_str(status) ? lapm_status_to_str(status) : "?", m->tag);

    switch (status)
    {
    case SIG_STATUS_LINK_CONNECTED:
        if (!m->lapm_up)
        {
            apply_v42bis_agreement(m);
            m->lapm_up = true;
            /* What the parallel framer collected was V.42's own detection
             * pattern, not data. */
            dm_ring_clear(&m->pending_rx);
            dm_log_event(DM_LOG_INFO, "modem", "error correction established",
                         "tag=%s protocol=%s took_ms=%" PRId64, m->tag, protocol_name(m),
                         m->connect_ms ? dm_now_ms() - m->connect_ms : 0);
        }
        break;

    case SIG_STATUS_LINK_DISCONNECTED:
        /* Two quite different events arrive here.
         *
         * If LAPM was up, the far end has closed a working link. If it was
         * never up, this is spandsp giving up on establishment - T401 has
         * expired N400 times with no answer to our SABME - which is a
         * failure to agree on V.42 at all, not a failure of V.42.
         *
         * The second case is the one that matters against real modems, and
         * it used to fall through to nothing: the handler only looked at
         * the case where lapm_up was already true, so a failed negotiation
         * sat there until the --v42-timeout backstop noticed. Treat it as
         * what it is, and fall back at once. */
        if (m->lapm_up)
        {
            const char *why = "";

#if defined(DATAMODEM_VENDORED_V42)
            why = dm_v42_disconnect_cause;
#endif
            m->lapm_up = false;
            m->lapm_lost = true;
            m->rx_busy = false;
            /* A far end that has been running LAPM goes on running it, so
             * falling back to async now would only put its flags on the
             * screen as ~?~?~?. A real modem says NO CARRIER here; so do we. */
            m->carrier_lost = true;
            m->phase = DM_PHASE_DOWN;
            if (why != NULL && strcmp(why, "the far end sent DISC") == 0)
                DM_INFO("modem", "the far end closed the error-corrected link; hanging up (tag=%s)", m->tag);
            else
                DM_ERROR("modem", "the error-corrected link was lost%s%s, so the call is no use (tag=%s)",
                         (why != NULL && why[0] != '\0') ? " - " : "", (why != NULL) ? why : "", m->tag);
        }
        else if (!m->v42_peer_declined && !m->v42_fell_back)
        {
            const char *why = "";

#if defined(DATAMODEM_VENDORED_V42)
            why = dm_v42_disconnect_cause;
#endif
            /* Establishment failed, which is not the same as the far end
             * being unable to do V.42 - it answered detection, so it can.
             * spandsp has no more patience to offer here: whether it ran out
             * of SABME retries or was refused outright, it parks in
             * LAPM_IDLE and stays there. Being more patient than that is the
             * whole point of --v42-timeout, so keep trying until it runs
             * out. A far end that refused with DM was within its rights to
             * say "not yet" and is entitled to be ready a moment later. */
            if (v42_retry_establishment(m, why))
                break;
            m->v42_peer_declined = true;
            m->v42_establish_failed = true;
            DM_INFO("modem", "V.42 negotiation failed after %u attempt%s%s%s (tag=%s)",
                    m->v42_establish_tries + 1, m->v42_establish_tries == 0 ? "" : "s",
                    (why != NULL && why[0] != '\0') ? " - " : "",
                    (why != NULL) ? why : "", m->tag);
#if defined(DATAMODEM_VENDORED_V42)
            if (dm_v42_fcs_errors > 0)
                DM_WARN("modem", "%u LAPM frame%s arrived with a bad checksum during the "
                                 "negotiation - the audio path is corrupting frames, which is a "
                                 "different problem from the far end refusing (tag=%s)",
                        dm_v42_fcs_errors, dm_v42_fcs_errors == 1 ? "" : "s", m->tag);
            else
                DM_INFO("modem", "every LAPM frame arrived intact, so the audio path is not the "
                                 "problem - the far end simply would not establish (tag=%s)",
                        m->tag);
#endif
        }
        break;

    case SIG_STATUS_LINK_ERROR:
        m->frame_errors++;
        DM_DEBUG("modem", "LAPM frame error (tag=%s, %u so far)", m->tag, m->frame_errors);
        break;

    default:
        if (status == DM_LAPM_V42_UNSUPPORTED && !m->v42_peer_declined && !m->lapm_up)
        {
            /* Detection finished and found nobody. Waiting out our own
             * timeout on top of that achieves nothing - spandsp has already
             * parked the state machine and will not try again. */
            m->v42_peer_declined = true;
#if defined(DATAMODEM_VENDORED_V42)
            if (dm_v42_adp_no_ec)
            {
                m->v42_adp_declined = true;
                DM_INFO("modem", "the far end's detection pattern asked for no error-correcting "
                                 "protocol, so V.42 is not on offer here - falling back without "
                                 "asking (tag=%s)", m->tag);
            }
            else
#endif
            DM_INFO("modem", "the far end did not answer V.42 detection (tag=%s)", m->tag);
        }
        break;
    }
}

/* Once detection has given up: the rest of the ODP character under way and
 * the ones after it, so that an async far end is not left half a character,
 * then mark. spandsp loads the pair's second character only when it reaches
 * it (txbits == 18), which the tail does not do: a first character ends
 * there, already followed by its eight ones. */
static bool v42_detect_tail_pending(const v42_state_t *v)
{
    return v->calling_party && v->neg.txbits > 0 && v->neg.txbits != 18;
}

static int v42_detect_tail_bit(v42_state_t *v)
{
    int bit;

    if (!v42_detect_tail_pending(v))
    {
        if (v->calling_party)
            v->neg.txbits = 0;
        return 1;
    }
    bit = (int) (v->neg.txstream & 1);
    v->neg.txstream >>= 1;
    v->neg.txbits--;
    return bit;
}

/* An answering modem is connected before the calling one is listening.
 * Over FSK it connects the moment it hears the caller's carrier, which the
 * caller - ours, see DM_FSK_MARK_MS, and real ones - holds half a second or
 * more before it takes data; over V.22 and V.22bis the caller finishes
 * training most of half a second after it. A BBS that greets at once lost
 * the start of its banner to that: "OME TO" for "WELCOME TO" over Bell 103,
 * all but "BBS" over V.22bis. Over V.42 nothing is lost, as LAPM is up only
 * once both ends are; with no V.42, the answering modem holds mark this long
 * after connecting before it sends what it was given. */
#define DM_ANSWER_HOLD_MS 1000

static int tx_get_bit(void *user)
{
    dm_modem_t *m = user;

    /* An FSK transmitter asks for bits from the moment it starts, but until
     * the far end's carrier has been heard there is nobody to send them to:
     * hold the line at mark, as a modem not yet connected does. Anything
     * else - V.42's ODP especially - is mistaken for something else by a
     * far end still working out what we are. */
    if (is_fsk(m->mod) && !m->connected)
        return 1;
    /* Answering with no V.42 to resend it, what we say first would be lost
     * to a caller that is not yet listening - see DM_ANSWER_HOLD_MS. It
     * waits in the queue meanwhile. */
    if (m->samples < m->answer_hold_until && (m->v42 == NULL || m->v42_fell_back))
        return 1;
    /* Detection has given up on the far end, and a fall back to async is
     * moments away: mark, rather than the HDLC flags LAPM would idle with,
     * which an async far end would print. */
    if (m->v42 != NULL && !m->v42_fell_back)
    {
        int bit;

        if (m->v42->lapm.state == DM_LAPM_V42_UNSUPPORTED)
            return v42_detect_tail_bit(m->v42);
        /* T400 runs out inside this call, which then hands back the first
         * bit of a flag. */
        bit = v42_tx_bit(m->v42);
        return m->v42->lapm.state == DM_LAPM_V42_UNSUPPORTED ? v42_detect_tail_bit(m->v42) : bit;
    }
    /* The fall back comes a frame or so after detection gives up, and the
     * character under way may not be finished by then. Cut short, the rest
     * of it goes out as mark and the far end receives a byte that is
     * neither ODP nor anything we were asked to send - 0x91 arriving as
     * 0xf1, say. */
    if (m->v42 != NULL && m->v42_fell_back && m->v42->lapm.state == DM_LAPM_V42_UNSUPPORTED &&
        v42_detect_tail_pending(m->v42))
        return v42_detect_tail_bit(m->v42);
    return async_framer_get_bit(m);
}

static void settle(dm_modem_t *m, const char *why);
static void fsk_maybe_connect(dm_modem_t *m);

static void note_connected(dm_modem_t *m)
{
    bool first = !m->connected;

    if (m->carrier_lost)
        return; /* already given up on; a new call is the only way back */

    settle(m, "trained");
    m->connected = true;
    m->carrier_down_ms = 0;
    m->phase = DM_PHASE_DATA;
    if (m->v22 != NULL)
    {
        m->bit_rate = v22bis_get_current_bit_rate(m->v22);
        /* V.22bis is symmetric, so the negotiated rate is the transmit rate
         * too. It can settle below what was offered, and the V.42 timers
         * have to follow it down. */
        m->tx_bit_rate = m->bit_rate;
        v42_set_bit_rate(m, m->tx_bit_rate);
    }
    else if (m->v32 != NULL)
    {
        /* Likewise V.32, which may have settled on 4800, and a retrain can
         * change it in either direction. */
        m->bit_rate = dm_v32_bit_rate(m->v32);
        m->tx_bit_rate = m->bit_rate;
        v42_set_bit_rate(m, m->tx_bit_rate);
    }
    else if (m->v34 != NULL)
    {
        /* V.34's two directions need not run at the same rate. What we
         * report is what we receive at; V.42's timers go by what we send. */
        m->bit_rate = dm_v34_rx_rate(m->v34);
        m->tx_bit_rate = dm_v34_tx_rate(m->v34);
        v42_set_bit_rate(m, m->tx_bit_rate);
    }

    /* Stepped down to a modulation that runs no V.42 - see carries_v42().
     * Nothing has gone through it yet: until now the carrier was not up. */
    if (first && m->v42 != NULL && !carries_v42(m->mod))
    {
        if (m->v42_mode == DM_V42_REQUIRE)
        {
            m->carrier_lost = true;
            m->phase = DM_PHASE_DOWN;
            DM_ERROR("modem", "the far end is a %s modem, which runs no V.42, and --v42 require was "
                              "asked for, so the call is no use (tag=%s)", m->mod_name, m->tag);
            return;
        }
        DM_INFO("modem", "no V.42 over %s; this link has no error correction (tag=%s)", m->mod_name, m->tag);
        v42_release(m->v42);
        v42_free(m->v42);
        m->v42 = NULL;
        m->v42_mode = DM_V42_OFF;
    }

    if (first)
    {
        m->connect_ms = dm_now_ms();
        m->last_rx_ms = m->connect_ms;
        if (!m->calling)
            m->answer_hold_until = m->samples + (long long) DM_ANSWER_HOLD_MS * DM_SAMPLE_RATE / 1000;
        /* The clock on the V.42 handshake starts now: before the carrier
         * there was nobody to handshake with. And so do its timers - this is
         * the first moment the line rate is a fact rather than an offer. */
        if (m->v42 != NULL)
        {
            /* Anything detection concluded before now came from framing
             * training signals, or nothing, as if they were the far end. */
            m->v42_peer_declined = false;
            m->v42_establish_failed = false;
            m->v42_adp_declined = false;
            m->v42_deadline_ms = m->connect_ms + (int64_t) m->v42_timeout_s * 1000;
            v42_arm(m, m->tx_bit_rate);
        }
        dm_log_event(DM_LOG_INFO, "modem", "carrier established", "tag=%s modulation=%s rate=%d role=%s",
                     m->tag, m->mod_name, m->bit_rate, m->calling ? "originate" : "answer");
        if (m->bit_rate < m->offered_rate)
            DM_INFO("modem", "the far end would not take %d bps, so this call runs at %d "
                             "(tag=%s)", m->offered_rate, m->bit_rate, m->tag);
    }
    else
    {
        DM_INFO("modem", "carrier recovered at %d bps (tag=%s)", m->bit_rate, m->tag);
        /* A dropout in the middle of V.42 detection or establishment takes
         * the ODP, the ADP or the SABME with it, and spandsp, which has no
         * idea the line went away, concludes from the silence that the far
         * end does not do V.42. Give the handshake the time it would have
         * had on the new carrier. Where detection had already finished,
         * though, the far end may well have LAPM up and be deaf to
         * detection patterns - so start again only as far back as we had
         * got: detection if it was still going, SABME if it was past it. */
        if (m->v42 != NULL && !m->lapm_up && !m->v42_fell_back && !m->v42_adp_declined)
        {
            bool detecting = m->v42->lapm.state == DM_LAPM_DETECT ||
                             (m->v42_peer_declined && !m->v42_establish_failed);

            DM_INFO("modem", "V.42 was still %s; starting %s again (tag=%s)",
                    detecting ? "in detection" : "establishing",
                    detecting ? "detection" : "establishment", m->tag);
            m->v42_peer_declined = false;
            m->v42_establish_failed = false;
            m->v42_retry_at_ms = 0;
            m->v42_deadline_ms = dm_now_ms() + (int64_t) m->v42_timeout_s * 1000;
            if (detecting)
                v42_arm(m, m->tx_bit_rate);
            else if (m->v42->lapm.state == DM_LAPM_IDLE)
                m->v42_retry_at_ms = dm_now_ms();
        }
    }
    /* Whatever was measured before this carrier belongs to the last one. A
     * training pattern framed as characters is not evidence about the rate. */
    m->rate_check_ms = 0;
    m->rate_bad_windows = 0;
}

static void rx_status(void *user, int status)
{
    dm_modem_t *m = user;

    switch (status)
    {
    case SIG_STATUS_CARRIER_UP:
        DM_DEBUG("modem", "carrier detected (tag=%s)", m->tag);
        /* FSK has no training phase to succeed: the carrier is the
         * connection - once it is known to be the far end's. */
        if (is_fsk(m->mod))
        {
            m->fsk_carrier = true;
            /* Once connected the far end's tone is known; a carrier coming
             * back is a recovery, as it always was. */
            if (!m->connected)
                fsk_maybe_connect(m);
            else
                note_connected(m);
        }
        break;

    case SIG_STATUS_TRAINING_SUCCEEDED:
        note_connected(m);
        break;

    case SIG_STATUS_TRAINING_FAILED:
        DM_WARN("modem", "training failed, the far end will retry (tag=%s)", m->tag);
        break;

    case SIG_STATUS_MODEM_RETRAIN_OCCURRED:
        m->retrains++;
        if (m->v22 != NULL)
            m->bit_rate = v22bis_get_current_bit_rate(m->v22);
        else if (m->v32 != NULL)
            m->bit_rate = dm_v32_bit_rate(m->v32);
        else if (m->v34 != NULL)
            m->bit_rate = dm_v34_rx_rate(m->v34);
        DM_INFO("modem", "retrained at %d bps (tag=%s, %u so far)", m->bit_rate, m->tag, m->retrains);
        /* A link that settled below what we offered and will not hold is the
         * signature of the two ends having disagreed about the rate. V.22bis
         * settles that question once, during training, and has no way to ask
         * it again - so the only cure is to stop asking it at all. See
         * apply_rate_ceiling(). */
        if (m->v22 != NULL && !m->rate_pinned && m->bit_rate > 0 && m->bit_rate < m->offered_rate)
            m->drop_ceiling_to = m->bit_rate;
        break;

    case SIG_STATUS_CARRIER_DOWN:
        m->fsk_carrier = false;
        /* Not the end of the call yet. Stop treating what arrives as data,
         * start the clock, and let dm_modem_tx() decide if it stays away.
         * See DM_CARRIER_GRACE_MS. */
        if (m->connected && !m->carrier_lost && m->carrier_down_ms == 0)
        {
            m->carrier_down_ms = dm_now_ms();
            m->phase = DM_PHASE_TRAINING;
            DM_DEBUG("modem", "carrier dropped, allowing %dms for a retrain (tag=%s)",
                     m->carrier_grace_ms, m->tag);
        }
        break;

    case SIG_STATUS_POOR_SIGNAL_QUALITY:
        DM_DEBUG("modem", "poor signal quality (tag=%s)", m->tag);
        break;

    default:
        DM_TRACE("modem", "status %s (tag=%s)", signal_status_to_str(status), m->tag);
        break;
    }

    /* Keep the character framer in step: a carrier that came and went must
     * not leave half a character sitting in it. */
    if (m->arx != NULL)
        async_rx_put_bit(m->arx, status);
    if (m->arx_monitor != NULL)
        async_rx_put_bit(m->arx_monitor, status);
}

static void rx_put_bit(void *user, int bit)
{
    dm_modem_t *m = user;

    if (bit < 0)
    {
        rx_status(m, bit);
        return;
    }
    if (m->v42 != NULL && !m->v42_fell_back)
    {
        v42_rx_bit(m->v42, bit);
        /* In parallel, and only until V.42 is decided: if it turns out the
         * far end never wanted V.42, these are the characters it has been
         * sending us all along and they are owed to the terminal. */
        if (v42_undecided(m))
            async_rx_put_bit(m->arx, bit);
    }
    else
    {
        async_rx_put_bit(m->arx, bit);
    }
    async_rx_put_bit(m->arx_monitor, bit);
}

static void v32_test_renegotiate(dm_modem_t *m)
{
    if (m->reneg_after_ms <= 0 || !m->connected || m->carrier_lost ||
        dm_now_ms() - m->connect_ms < m->reneg_after_ms)
        return;
    m->reneg_after_ms = 0;
    if (!dm_v32_renegotiate(m->v32, m->reneg_rate))
        DM_WARN("modem", "DATAMODEM_V32_RENEGOTIATE: could not ask - not a V.32bis call, or not in data "
                         "(tag=%s)", m->tag);
}

/* DATAMODEM_V34_RENEGOTIATE: the same, for V.34's 11.6. */
static void v34_test_renegotiate(dm_modem_t *m)
{
    if (m->reneg_after_ms <= 0 || !m->connected || m->carrier_lost ||
        dm_now_ms() - m->connect_ms < m->reneg_after_ms)
        return;
    m->reneg_after_ms = 0;
    if (!dm_v34_renegotiate(m->v34, m->reneg_rate))
        DM_WARN("modem", "DATAMODEM_V34_RENEGOTIATE: could not ask - not in data (tag=%s)", m->tag);
}

/* V.32 reports its handshake in its own terms; this puts them in the ones
 * rx_status already understands. Called with the lock held, from inside
 * dm_v32_tx or dm_v32_rx. */
static void v32_event(void *user, dm_v32_event_t ev)
{
    dm_modem_t *m = user;

    switch (ev)
    {
    case DM_V32_TRAINED:
        /* The first time is a connection; after that, a retrain that came
         * good - possibly at a different rate. */
        if (m->connected)
            rx_status(m, SIG_STATUS_MODEM_RETRAIN_OCCURRED);
        rx_status(m, SIG_STATUS_TRAINING_SUCCEEDED);
        break;

    case DM_V32_RETRAINING:
    case DM_V32_CARRIER_DOWN:
        /* Either way no data crosses until the retrain completes, and
         * either way it gets the grace period to do so. */
        rx_status(m, SIG_STATUS_CARRIER_DOWN);
        break;

    case DM_V32_TRAINING_FAILED:
        rx_status(m, SIG_STATUS_TRAINING_FAILED);
        break;

    case DM_V32_RATE_CHANGED:
        /* V.32 bis changed rate without retraining. Nothing was lost and the
         * carrier never went; only the rate, and V.42's timers with it. */
        m->bit_rate = dm_v32_bit_rate(m->v32);
        m->tx_bit_rate = m->bit_rate;
        v42_set_bit_rate(m, m->tx_bit_rate);
        DM_INFO("modem", "rate changed to %d bps (tag=%s)", m->bit_rate, m->tag);
        break;

    case DM_V32_CLEARDOWN:
        /* The two ends found no rate in common, which no retrain will
         * change. */
        if (!m->carrier_lost)
        {
            m->carrier_lost = true;
            m->phase = DM_PHASE_DOWN;
            DM_ERROR("modem", "the V.32 rate exchange called for a cleardown (tag=%s)", m->tag);
        }
        break;
    }
}

/* V.34's events in the terms rx_status understands, as for V.32. */
static void v34_event(void *user, dm_v34_event_t ev)
{
    dm_modem_t *m = user;

    switch (ev)
    {
    case DM_V34_TRAINED:
        if (m->connected)
            rx_status(m, SIG_STATUS_MODEM_RETRAIN_OCCURRED);
        rx_status(m, SIG_STATUS_TRAINING_SUCCEEDED);
        break;
    case DM_V34_RETRAINING:
    case DM_V34_CARRIER_DOWN:
        rx_status(m, SIG_STATUS_CARRIER_DOWN);
        break;
    case DM_V34_TRAINING_FAILED:
        rx_status(m, SIG_STATUS_TRAINING_FAILED);
        break;
    case DM_V34_RATE_CHANGED:
        m->bit_rate = dm_v34_rx_rate(m->v34);
        m->tx_bit_rate = dm_v34_tx_rate(m->v34);
        m->carrier_down_ms = 0;
        m->phase = DM_PHASE_DATA;
        v42_set_bit_rate(m, m->tx_bit_rate);
        DM_INFO("modem", "rate changed to %d bps in, %d out (tag=%s)", m->bit_rate, m->tx_bit_rate, m->tag);
        break;
    case DM_V34_NO_V8:
        /* Dealt with at the next frame - see after_v8() - since it frees
         * the pump whose callback this is. */
        if (!m->carrier_lost)
            m->no_v8 = true;
        break;
    case DM_V34_NOT_V34:
        /* V.8 began and went nowhere - it stalled, or the far end's V.8
         * offered no V.34. Most often an older modem whose answer tone was
         * taken for ANSam, and is now waiting for us in its own terms. */
        if (!m->carrier_lost)
        {
            m->no_v8 = true;
            m->v8_stalled = true;
        }
        break;
    case DM_V34_CLEARDOWN:
        if (!m->carrier_lost)
        {
            m->carrier_lost = true;
            m->phase = DM_PHASE_DOWN;
            DM_ERROR("modem", "the V.34 connection was cleared down (tag=%s)", m->tag);
        }
        break;
    }
}

/* spandsp's V.42 never reports that detection has failed - point it at a far
 * end that has never heard of V.42 and it will send ODP patterns happily for
 * as long as you let it, with no callback and no give-up. So the deadline
 * lives here. This is also what real modems did: try for a few seconds, then
 * either drop to a direct async connection or give up on the call, depending
 * on how the user set &Q.
 *
 * Called from dm_modem_tx with the lock held. */
/* Which of the two quite different failures this was. They look the same
 * from the session's point of view and are not the same problem: a far end
 * that ignored detection does not do V.42, while one that answered detection
 * and then would not establish does, and is worth more patience. */
static const char *v42_failure_reason(const dm_modem_t *m)
{
    if (m->v42_adp_declined)
        return "its detection pattern asked for no error correction";
    if (m->v42_establish_failed)
        return "it answered detection, but LAPM would not establish";
    if (m->v42_peer_declined)
        return "it did not answer detection";
    return "it said nothing at all";
}

/* Did the two ends actually agree about the line rate?
 *
 * Only asked of a link that settled below what we offered, because that is
 * the only way V.22bis can end up asymmetric - see apply_rate_ceiling(). The
 * test is the async framer's own opinion of what it is being given: a stream
 * demodulated at the wrong rate is noise, and noise puts the stop bit in the
 * wrong place about half the time. Real data, even over a poor line, does
 * not do that. Content-independent, so it does not care what the far end was
 * trying to say. */
static void check_rate_agreement(dm_modem_t *m)
{
    int64_t now;
    int errors;
    uint64_t chars;

    if (m->v22 == NULL || m->arx_monitor == NULL || m->drop_ceiling_to > 0)
        return;
    if (!m->connected || m->phase != DM_PHASE_DATA || m->carrier_lost)
        return;
    /* Once the ceiling has been dropped the rates match by construction, so
     * there is nothing left to decide - but keep measuring, because if the
     * framing is still wrong afterwards then the rate was never the problem
     * and saying so saves somebody the same search. */
    if (!m->rate_pinned && (m->bit_rate <= 0 || m->bit_rate >= m->offered_rate))
        return;
    /* Only once the async framer is what is actually running. While V.42 is
     * still being decided it is framing HDLC in parallel, and complaining
     * about the stop bits in somebody else's flags proves nothing. */
    if (m->v42 != NULL && !m->v42_fell_back)
        return;
    if (m->lapm_up)
        return;

    now = dm_now_ms();
    if (m->rate_check_ms == 0)
    {
        m->rate_check_ms = now;
        m->rate_check_errors = m->arx_monitor->framing_errors;
        m->rate_check_bytes = m->bytes_rx;
        return;
    }
    if (now - m->rate_check_ms < DM_RATE_CHECK_MS)
        return;

    errors = m->arx_monitor->framing_errors - m->rate_check_errors;
    chars = m->bytes_rx - m->rate_check_bytes;
    m->rate_check_ms = now;
    m->rate_check_errors = m->arx_monitor->framing_errors;
    m->rate_check_bytes = m->bytes_rx;

    DM_DEBUG("modem", "rate agreement check: %d framing errors in %" PRIu64 " chars (tag=%s)",
             errors, chars + (uint64_t) errors, m->tag);
    if (chars + (uint64_t) errors < DM_RATE_CHECK_MIN_CHARS)
        return;                 /* too quiet to judge */
    if ((uint64_t) errors * 4 < chars)
    {
        m->rate_bad_windows = 0;
        if (m->rate_pinned && !m->rate_proven)
        {
            m->rate_proven = true;
            DM_INFO("modem", "the framing is clean now, so re-offering %d was the right call "
                             "(tag=%s)", m->bit_rate, m->tag);
        }
        return;                 /* framing is fine, so the rate is fine */
    }

    if (++m->rate_bad_windows < 2)
        return;                 /* one bad window could be a retrain settling */
    if (m->rate_pinned)
    {
        if (!m->rate_complained)
        {
            m->rate_complained = true;
            DM_WARN("modem", "%d of the last %" PRIu64 " characters are still badly framed with "
                             "both ends at %d bps, so the rate was not the problem - the audio "
                             "path is (tag=%s)",
                    errors, chars + (uint64_t) errors, m->bit_rate, m->tag);
        }
        return;
    }
    DM_WARN("modem", "%d of the last %" PRIu64 " characters arrived with the stop bit in the wrong "
                     "place, which is what demodulating the far end at the wrong rate looks like "
                     "(tag=%s)", errors, chars + (uint64_t) errors, m->tag);
    m->drop_ceiling_to = m->bit_rate;
}

static void check_v42_deadline(dm_modem_t *m)
{
    if (m->v42 == NULL || m->lapm_up || m->v42_fell_back || m->v42_deadline_ms == 0)
        return;
    if (m->carrier_lost)
        return;
    /* Nor while a retrain is under way: nothing can be said either way
     * until the carrier is back, and note_connected() starts again then. */
    if (m->carrier_down_ms != 0)
        return;

    /* An establishment retry that has come due. */
    if (m->v42_retry_at_ms != 0 && dm_now_ms() >= m->v42_retry_at_ms)
    {
        m->v42_retry_at_ms = 0;
        m->v42_establish_tries++;
#if defined(DATAMODEM_VENDORED_V42)
        if (dm_v42_reconnect(m->v42) == 0)
        {
            DM_INFO("modem", "V.42 establishment attempt %u: sending SABME again (tag=%s)",
                    m->v42_establish_tries + 1, m->tag);
            return;
        }
        DM_DEBUG("modem", "V.42 is not in a state to retry establishment (tag=%s)", m->tag);
#endif
        m->v42_peer_declined = true;
    }

    /* Either spandsp has told us detection failed, or our own backstop has
     * run out waiting for it to say anything at all. */
    if (!m->v42_peer_declined && dm_now_ms() < m->v42_deadline_ms)
        return;

    if (m->v42_mode == DM_V42_REQUIRE)
    {
        m->carrier_lost = true;
        m->phase = DM_PHASE_DOWN;
        DM_ERROR("modem", "V.42 could not be established (%s) and --v42 require was asked for, "
                          "so the call is no use (tag=%s)",
                 v42_failure_reason(m), m->tag);
        return;
    }

    /* Drop back to a plain start/stop stream. The pump keeps running and the
     * carrier is untouched; only the meaning of the bits changes. */
    m->v42_fell_back = true;
    m->tx_bitpos = 0;
    /* Two characters' worth of mark first. The far end was hearing ODP, or
     * LAPM, cut off mid-character; its framer needs the line idle for a
     * whole character to find the next start bit, or the first few come out
     * garbled. */
    m->tx_idle_bits = 2 * (1 + m->data_bits + (m->parity != ASYNC_PARITY_NONE) + m->stop_bits);
    DM_WARN("modem", "giving up on V.42 (%s); falling back to a direct async connection "
                     "(tag=%s)", v42_failure_reason(m), m->tag);

    /* Whatever the parallel framer collected while we were deciding is
     * either the far end talking plain async all along - worth keeping - or
     * its HDLC, framed by a receiver that had no business framing it. The
     * two are easy to tell apart: a stream of LAPM flags is a quarter 0x7e
     * or more once chopped into characters, and ordinary text is not. Giving
     * HDLC to the terminal is how a failed negotiation ends up looking like
     * line noise, so count first and deliver only if it reads as data. */
    {
        unsigned char held[512];
        size_t n;
        size_t total = 0;
        size_t flags = 0;
        unsigned char kept[DM_PENDING_RX];
        size_t kept_n = 0;

        while ((n = dm_ring_read(&m->pending_rx, held, sizeof(held))) > 0)
        {
            for (size_t i = 0; i < n; i++)
            {
                if (held[i] == 0x7e)
                    flags++;
                if (kept_n < sizeof(kept))
                    kept[kept_n++] = held[i];
            }
            total += n;
        }
        if (total == 0)
        {
            /* nothing was held back */
        }
        else if (flags * 4 >= total)
        {
            DM_INFO("modem", "discarded %zu byte%s of the far end's HDLC that arrived while V.42 "
                             "was still being decided - %zu of them flags, so it was LAPM and "
                             "not data (tag=%s)",
                    total, total == 1 ? "" : "s", flags, m->tag);
        }
        else
        {
            m->wire_rx += kept_n;
            deliver_rx(m, kept, kept_n);
            DM_INFO("modem", "recovered %zu byte%s the far end sent while V.42 was still being "
                             "decided (tag=%s)",
                    kept_n, kept_n == 1 ? "" : "s", m->tag);
        }
    }
}

/* ---------------------------------------------------------------- recording
 *
 * DATAMODEM_RECORD=path, a test hook rather than an option: the call's audio
 * from the moment it is answered, in path.<tag>.wav - stereo at 8 kHz, what
 * we heard on the left and what we sent on the right. A log says what the
 * modem concluded; this says what the far end actually sent, and can be
 * played back through the detectors afterwards. */

static void rec_u32(FILE *f, uint32_t v)
{
    unsigned char b[4] = { (unsigned char) v, (unsigned char) (v >> 8), (unsigned char) (v >> 16),
                           (unsigned char) (v >> 24) };

    fwrite(b, 1, 4, f);
}

static void rec_header(FILE *f, uint32_t frames)
{
    static const unsigned char fmt[] = { 1, 0, 2, 0, 0x40, 0x1f, 0, 0, 0x00, 0x7d, 0, 0, 4, 0, 16, 0 };

    fseek(f, 0, SEEK_SET);
    fwrite("RIFF", 1, 4, f);
    rec_u32(f, 36 + frames * 4);
    fwrite("WAVEfmt ", 1, 8, f);
    rec_u32(f, 16);
    fwrite(fmt, 1, sizeof(fmt), f);
    fwrite("data", 1, 4, f);
    rec_u32(f, frames * 4);
    fseek(f, 0, SEEK_END);
}

static void record_open(dm_modem_t *m)
{
    const char *path = getenv("DATAMODEM_RECORD");
    char name[1024];

    if (path == NULL || *path == '\0')
        return;
    snprintf(name, sizeof(name), "%s.%s.wav", path, m->tag);
    m->rec = fopen(name, "wb");
    if (m->rec == NULL)
    {
        DM_WARN("modem", "DATAMODEM_RECORD: cannot write %s", name);
        return;
    }
    rec_header(m->rec, 0);
    DM_WARN("modem", "DATAMODEM_RECORD: recording the call's audio to %s. This is a test hook.", name);
}

/* What we sent, held until the received samples of the same frame arrive. */
static void record_tx(dm_modem_t *m, const int16_t *x, int n)
{
    if (m->rec == NULL)
        return;
    for (int i = 0; i < n && m->rec_tx_n < (int) (sizeof(m->rec_tx) / sizeof(m->rec_tx[0])); i++)
        m->rec_tx[m->rec_tx_n++] = x[i];
}

/* x NULL: audio that never arrived. */
static void record_rx(dm_modem_t *m, const int16_t *x, int n)
{
    int used = n < m->rec_tx_n ? n : m->rec_tx_n;

    if (m->rec == NULL)
        return;
    for (int i = 0; i < n; i++)
    {
        uint16_t l = (uint16_t) (x != NULL ? x[i] : 0);
        uint16_t r = (uint16_t) (i < m->rec_tx_n ? m->rec_tx[i] : 0);
        unsigned char b[4] = { (unsigned char) l, (unsigned char) (l >> 8), (unsigned char) r,
                               (unsigned char) (r >> 8) };

        fwrite(b, 1, 4, m->rec);
    }
    m->rec_frames += (uint32_t) n;
    memmove(m->rec_tx, m->rec_tx + used, (size_t) (m->rec_tx_n - used) * sizeof(int16_t));
    m->rec_tx_n -= used;
}

static void record_close(dm_modem_t *m)
{
    if (m->rec == NULL)
        return;
    rec_header(m->rec, m->rec_frames);
    fclose(m->rec);
    m->rec = NULL;
}

/* ------------------------------------------------------ create / destroy */

static void start_pump(dm_modem_t *m, const char *why)
{
    if (m->pump_started)
        return;
    m->pump_started = true;
    if (m->v32 != NULL)
        dm_v32_start(m->v32);
    if (!m->calling && m->mod == DM_MOD_V21)
        m->offered_v21 = true;
    if (!m->calling && m->mod == DM_MOD_BELL103)
        m->offered_bell = true;
    m->phase = DM_PHASE_TRAINING;
    dm_log_event(DM_LOG_INFO, "modem", "training", "tag=%s modulation=%s role=%s offered_rate=%d why=\"%s\"",
                 m->tag, m->mod_name, m->calling ? "originate" : "answer", m->offered_rate, why);
    fsk_maybe_connect(m);
}

/* The V.32 or V.32 bis pump, for m->mod. listen_first has it running, silent,
 * through the wait for the answer tone, so that it can hear AC from an
 * answering modem that sends none. */
static bool create_v32_pump(dm_modem_t *m, bool listen_first)
{
    dm_v32_params_t vp;

    memset(&vp, 0, sizeof(vp));
    vp.calling = m->calling;
    vp.v32bis = (m->mod == DM_MOD_V32BIS);
    vp.max_rate = m->offered_rate;
    /* Test hook, not an option: every 9600 bit/s V.32 modem has to be
     * able to fall back to the nonredundant 16-point code (1e), and this
     * is how that path gets exercised against ourselves. */
    vp.trellis = (getenv("DATAMODEM_V32_NO_TRELLIS") == NULL);
    vp.listen_first = listen_first;
    if (!vp.trellis)
        DM_WARN("modem", "DATAMODEM_V32_NO_TRELLIS: this end will not offer trellis coding%s. "
                         "This is a test hook.",
                vp.v32bis ? " - ignored by V.32bis, whose rates are all trellis coded" : "");
    /* Test hook, not an option: "5:9600" has the calling end ask for
     * 9600 five seconds into the call, "5:9600:answer" the answering
     * end. It is how V.32 bis's rate renegotiation, which a clean line
     * never needs, gets exercised against ourselves. */
    {
        const char *rn = getenv("DATAMODEM_V32_RENEGOTIATE");
        double secs = 0.0;
        int rate = 0;
        char role[16] = "";

        if (rn != NULL && sscanf(rn, "%lf:%d:%15s", &secs, &rate, role) >= 2 && secs > 0.0 &&
            (strcmp(role, "answer") == 0) == !m->calling)
        {
            m->reneg_after_ms = (int) (secs * 1000.0);
            m->reneg_rate = rate;
            DM_WARN("modem", "DATAMODEM_V32_RENEGOTIATE: this end will ask for %d bps %.1f "
                             "seconds into the call. This is a test hook.", rate, secs);
        }
    }
    /* Test hook: "answer" or "call" makes that end ignore the far end's
     * requests to change rate, as a far end might that does not do 8/V.32
     * bis properly - so the asking end's give-up-and-retrain gets run. */
    {
        const char *deaf = getenv("DATAMODEM_V32_IGNORE_RENEGOTIATION");

        if (deaf != NULL && strcmp(deaf, m->calling ? "call" : "answer") == 0)
        {
            vp.deaf_to_renegotiation = true;
            DM_WARN("modem", "DATAMODEM_V32_IGNORE_RENEGOTIATION: this end will not hear requests to "
                             "change rate. This is a test hook.");
        }
    }
    vp.tx_power = m->tx_power;
    vp.tag = m->tag;
    vp.get_bit = tx_get_bit;
    vp.put_bit = rx_put_bit;
    vp.event = v32_event;
    vp.user = m;
    m->v32 = dm_v32_create(&vp);
    return m->v32 != NULL;
}

/* The data pump for m->mod. listen_first is for a calling V.32 pump: see
 * create_v32_pump(). */
static bool create_pump(dm_modem_t *m, bool listen_first)
{
    if (is_fsk(m->mod))
    {
        m->fsk_tx = fsk_tx_init(NULL, fsk_spec_for(m->mod, m->calling, true), tx_get_bit, m);
        m->fsk_rx = fsk_rx_init(NULL, fsk_spec_for(m->mod, m->calling, false), FSK_FRAME_MODE_ASYNC,
                                rx_put_bit, m);
        if (m->fsk_tx == NULL || m->fsk_rx == NULL)
        {
            DM_ERROR("modem", "could not start the %s data pump", m->mod_name);
            return false;
        }
        fsk_tx_power(m->fsk_tx, m->tx_power);
        fsk_rx_set_modem_status_handler(m->fsk_rx, rx_status, m);
    }
    else if (is_v32(m->mod))
    {
        if (!create_v32_pump(m, listen_first))
        {
            DM_ERROR("modem", "could not start the %s data pump", m->mod_name);
            return false;
        }
    }
    else if (m->mod == DM_MOD_V34)
    {
        dm_v34_params_t vp;

        memset(&vp, 0, sizeof(vp));
        vp.calling = m->calling;
        vp.max_rate = m->offered_rate;
        vp.tx_power = m->tx_power;
        vp.lapm = m->v42_mode != DM_V42_OFF;
        /* Test hook, not an option: a mask of the symbol rates to allow,
         * bit 0 2400 ... bit 5 3429, so that each can be exercised. */
        if (getenv("DATAMODEM_V34_SYMBOL_RATES") != NULL)
        {
            vp.symbol_rates = (unsigned) strtoul(getenv("DATAMODEM_V34_SYMBOL_RATES"), NULL, 0);
            DM_WARN("modem", "DATAMODEM_V34_SYMBOL_RATES=0x%x: only those symbol rates. This is a test hook.",
                    vp.symbol_rates);
        }
        /* Test hooks: "high" or "low", and 0 to 10 - what this end asks the
         * far end to transmit with, whatever probing says. */
        vp.pre_emphasis = -1;
        if (getenv("DATAMODEM_V34_CARRIER") != NULL)
        {
            vp.carrier = strcmp(getenv("DATAMODEM_V34_CARRIER"), "high") == 0 ? 2 : 1;
            DM_WARN("modem", "DATAMODEM_V34_CARRIER: asking for the %s carrier. This is a test hook.",
                    vp.carrier == 2 ? "high" : "low");
        }
        /* Test hooks: the trellis code and shaping our receiver asks the far
         * end's transmitter for. */
        if (getenv("DATAMODEM_V34_TRELLIS") != NULL)
        {
            vp.trellis = atoi(getenv("DATAMODEM_V34_TRELLIS"));
            DM_WARN("modem", "DATAMODEM_V34_TRELLIS: asking for the %d-state code. This is a test hook.",
                    vp.trellis);
        }
        if (getenv("DATAMODEM_V34_SHAPING") != NULL)
        {
            vp.shaping = true;
            DM_WARN("modem", "DATAMODEM_V34_SHAPING: asking for expanded shaping. This is a test hook.");
        }
        if (getenv("DATAMODEM_V34_PRE_EMPHASIS") != NULL)
        {
            vp.pre_emphasis = atoi(getenv("DATAMODEM_V34_PRE_EMPHASIS"));
            DM_WARN("modem", "DATAMODEM_V34_PRE_EMPHASIS: asking for filter %d. This is a test hook.",
                    vp.pre_emphasis);
        }
        vp.tag = m->tag;
        vp.get_bit = tx_get_bit;
        vp.put_bit = rx_put_bit;
        vp.event = v34_event;
        {
            const char *rn = getenv("DATAMODEM_V34_RENEGOTIATE");
            double secs = 0.0;
            int rate = 0;
            char role[16] = "";

            if (rn != NULL && sscanf(rn, "%lf:%d:%15s", &secs, &rate, role) >= 2 && secs > 0.0 &&
                (strcmp(role, "answer") == 0) == !m->calling)
            {
                m->reneg_after_ms = (int) (secs * 1000.0);
                m->reneg_rate = rate;
                DM_WARN("modem", "DATAMODEM_V34_RENEGOTIATE: this end will ask for at most %d bps inbound %.1f "
                                 "seconds into the call. This is a test hook.", rate, secs);
            }
        }
        vp.user = m;
        m->v34 = dm_v34_create(&vp);
        if (m->v34 == NULL)
        {
            DM_ERROR("modem", "could not start the %s data pump", m->mod_name);
            return false;
        }
    }
    else
    {
        m->v22 = v22bis_init(NULL, m->offered_rate, m->guard, m->calling ? TRUE : FALSE, tx_get_bit, m,
                             rx_put_bit, m);
        if (m->v22 == NULL)
        {
            DM_ERROR("modem", "could not start the %s data pump", m->mod_name);
            return false;
        }
        v22bis_tx_power(m->v22, m->tx_power);
        v22bis_set_modem_status_handler(m->v22, rx_status, m);
        attach_logging(v22bis_get_logging_state(m->v22), m->tag);

#if !defined(DATAMODEM_VENDORED_V22BIS)
        /* Built against the system libspandsp. Many packaged builds of it -
         * including the one Homebrew ships - complete V.22bis training,
         * report a connection, and then deliver a constant 0x55 forever,
         * because the receive equaliser has already diverged. Say so rather
         * than let it look like a bad line. See third_party/spandsp-v22bis.
         * `datamodem selftest --modulation v22bis` settles it either way. */
        DM_WARN("modem", "this build uses the system libspandsp for V.22bis; if it is one of "
                         "the many that train and then carry nothing, expect a constant 0x55. "
                         "Run 'datamodem selftest --modulation v22bis' to check.");
#endif
    }
    return true;
}

static void free_pump(dm_modem_t *m)
{
    dm_v32_free(m->v32);
    m->v32 = NULL;
    dm_v34_free(m->v34);
    m->v34 = NULL;
    if (m->v22 != NULL)
    {
        v22bis_release(m->v22);
        v22bis_free(m->v22);
        m->v22 = NULL;
    }
    if (m->fsk_tx != NULL)
    {
        fsk_tx_release(m->fsk_tx);
        fsk_tx_free(m->fsk_tx);
        m->fsk_tx = NULL;
    }
    if (m->fsk_rx != NULL)
    {
        fsk_rx_release(m->fsk_rx);
        fsk_rx_free(m->fsk_rx);
        m->fsk_rx = NULL;
    }
}

/* Everything that follows from the modulation: its rates, how long a retrain
 * may take, and how much the transmit queue may hold. */
static void set_modulation(dm_modem_t *m, dm_mod_t mod)
{
    size_t limit;

    m->mod = mod;
    m->mod_name = modulation_name(mod);
    m->offered_rate = rate_for(mod, m->rate_cap);
    m->carrier_grace_ms = is_v32(mod)             ? DM_CARRIER_GRACE_V32_MS
                          : (mod == DM_MOD_V34) ? DM_CARRIER_GRACE_V34_MS
                                                : DM_CARRIER_GRACE_MS;
    m->tx_bit_rate = m->offered_rate;
    if (is_fsk(mod))
    {
        m->offered_rate = fsk_rate_for(mod, m->calling, false);
        /* V.23 is not symmetric: the calling end receives at 1200 and
         * transmits at 75, so how long our own data takes to get out is a
         * different number from the one we report as the connection rate. */
        m->tx_bit_rate = fsk_rate_for(mod, m->calling, true);
    }
    m->bit_rate = m->offered_rate;
    /* Ten bits carries one 8N1 character, near enough for sizing. */
    limit = (size_t) m->offered_rate / 10 * DM_TX_SECONDS;
    if (limit < DM_TX_QUEUE_MIN)
        limit = DM_TX_QUEUE_MIN;
    __atomic_store_n(&m->tx_limit, limit, __ATOMIC_RELAXED);
}

/* What we are sending while we listen, whose echo is not the far end. */
static void listen_own(dm_modem_t *m)
{
    dm_listen_t *l = &m->listen;

    if (m->calling)
    {
        if (is_v32(m->mod))
            listen_set_own(l, 1800.0f, 0.0f);
        else if (m->mod == DM_MOD_V34)
            listen_set_own(l, 980.0f, 1180.0f); /* V.8's CM, when it is being sent */
        else if (m->mod == DM_MOD_V21)
            listen_set_own(l, 980.0f, 1180.0f);
        else if (m->mod == DM_MOD_BELL103)
            listen_set_own(l, 1270.0f, 1070.0f);
        else if (m->mod == DM_MOD_V23)
            listen_set_own(l, 390.0f, 450.0f);
        else
            listen_set_own(l, 0.0f, 0.0f);
    }
    else if (m->phase == DM_PHASE_ANSWER_TONE || m->mod == DM_MOD_V34)
        listen_set_own(l, 2100.0f, 0.0f);
    else if (is_v22(m->mod))
        listen_set_own(l, 2250.0f, 2850.0f);
    else if (is_v32(m->mod))
        listen_set_own(l, 600.0f, 3000.0f);
    else if (m->mod == DM_MOD_V21)
        listen_set_own(l, 1650.0f, 1850.0f);
    else if (m->mod == DM_MOD_BELL103)
        listen_set_own(l, 2225.0f, 2025.0f);
    else if (m->mod == DM_MOD_V23)
        listen_set_own(l, 1300.0f, 2100.0f);
    else
        listen_set_own(l, 0.0f, 0.0f);
}

static bool listening(const dm_modem_t *m)
{
    return m->hunting || (is_fsk(m->mod) && !m->fsk_confirmed);
}

/* The far end has answered in our terms; whatever happens now is this
 * modulation's handshake, and offering it anything else would only break
 * it. */
static void settle(dm_modem_t *m, const char *why)
{
    if (!m->hunting)
        return;
    m->hunting = false;
    DM_INFO("modem", "settled on %s: %s (tag=%s)", m->switch_due ? modulation_name(m->switch_to) : m->mod_name,
            why, m->tag);
}

/* Changes pump at the start of the next frame, in dm_modem_tx: not from
 * inside a pump's callbacks, which is where most of the reasons arise, and
 * not between a pump's transmit and receive halves of a frame. */
static void request_step(dm_modem_t *m, int mod, const char *why)
{
    if (mod < 0 || m->switch_due || m->carrier_lost || mod == (int) m->mod)
        return;
    m->switch_due = true;
    m->switch_to = (dm_mod_t) mod;
    m->switch_confirmed = false;
    m->switch_listen = false;
    snprintf(m->switch_why, sizeof(m->switch_why), "%s", why);
}

/* How long an answering modem offers one modulation before the next. */
static long window_samples(const dm_modem_t *m, dm_mod_t mod)
{
    int ms = DM_WINDOW_MS;

    /* AC has to reach the caller and its AA come back. */
    if (is_v32(mod))
        ms += m->path_delay_ms;
    return (long) ms * DM_SAMPLE_RATE / 1000;
}

static void step_now(dm_modem_t *m)
{
    dm_mod_t to = m->switch_to;
    const char *was = m->mod_name;

    m->switch_due = false;
    free_pump(m);
    set_modulation(m, to);
    m->pump_started = false;
    m->fsk_carrier = false;
    m->fsk_confirmed = m->switch_confirmed;
    m->fsk_announced = false;
    m->fsk_mark_from = -1;
    m->answer_hold_until = 0;
    listen_reset(&m->listen);
    if (!create_pump(m, m->calling && is_v32(to)))
    {
        m->carrier_lost = true;
        m->phase = DM_PHASE_DOWN;
        DM_ERROR("modem", "could not start the %s data pump (tag=%s)", m->mod_name, m->tag);
        return;
    }
    dm_log_event(DM_LOG_INFO, "modem", "modulation", "tag=%s from=%s to=%s rate=%d why=\"%s\"", m->tag, was,
                 m->mod_name, m->offered_rate, m->switch_why);
    v42_set_bit_rate(m, m->tx_bit_rate);
    if (!m->calling)
    {
        m->window = cycle_index(to);
        m->window_left = window_samples(m, to);
    }
    if (m->calling && is_v32(to) && m->switch_listen)
    {
        /* Silent until AC - the V.32 pump starts itself on hearing it - or
         * until listening hears something else to step to. */
        m->phase = DM_PHASE_ANSWER_TONE;
        m->answer_tone_seen = false;
        m->wait_samples = INT32_MAX;
        m->ans_tail_running = false;
    }
    else if (m->calling && is_v32(to))
    {
        /* V.32 bis Annex A's calling modem: AA once it has heard a second of
         * answer tone, and on with the handshake whenever AC comes. */
        m->phase = DM_PHASE_ANSWER_TONE;
        m->answer_tone_seen = true;
        m->wait_samples = 0;
        m->ans_tail_running = true;
        m->ans_tail_samples = DM_SAMPLE_RATE;
    }
    else
    {
        m->answer_tone_samples = 0;
        start_pump(m, m->switch_why);
    }
}

/* Answering: withdraw what is on offer and offer the next thing. */
static void next_offer(dm_modem_t *m, const char *why)
{
    for (int k = 1; k <= DM_ANSWER_CYCLE_LEN; k++)
    {
        int i = (m->window + k + DM_ANSWER_CYCLE_LEN) % DM_ANSWER_CYCLE_LEN;
        int mod = cycle_mod(m, i);

        if (mod < 0)
            continue;
        if (mod == (int) m->mod)
        {
            m->window = i;
            m->window_left = window_samples(m, m->mod);
        }
        else
        {
            request_step(m, mod, why);
        }
        return;
    }
}

/* V.34 found the far end has no V.8: it answered with a plain answer tone,
 * or did not answer our ANSam with CM. It is an older modem, and V.32 bis is
 * the place to start. An answering V.32 caller will have been sending AA at
 * our ANSam, taking it for ANS; anything older waits to hear its own
 * answering signal, which is Annex A's V.22 bis first. */
static void after_v8(dm_modem_t *m)
{
    int v32 = pick_v32(m);

    if (!m->step_down || v32 < 0)
    {
        m->carrier_lost = true;
        m->phase = DM_PHASE_DOWN;
        DM_ERROR("modem", "the far end %s, so it is no V.34 modem, and --no-step-down rules out "
                          "anything slower (tag=%s)",
                 m->v8_stalled ? "did not complete V.8" : "does not do V.8", m->tag);
        return;
    }
    /* Whatever had been settled on was V.8, and it is off. */
    m->hunting = true;
    if (m->calling && m->v8_stalled)
    {
        /* Its answer tone is long over, and it is offering whatever it does
         * next - AC, USB1, a 300 bps carrier - and waiting to be answered:
         * Annex A's late caller, which listens and says nothing until it
         * has heard one. */
        request_step(m, v32, "V.8 went unanswered: an older modem, its answer tone taken for ANSam");
        m->switch_listen = true;
    }
    else if (m->calling)
        request_step(m, v32, "a plain answer tone, without V.8");
    else if (m->aa_heard)
        request_step(m, v32, "no CM in answer to ANSam, but AA: a V.32 caller");
    else
        next_offer(m, "no CM in answer to ANSam");
}

/* The answering modem's answer tone is over. Annex A: a caller that sent AA
 * during it is V.32; one that did not may be V.22 bis, and gets unscrambled
 * ones first. */
static void answer_tone_done(dm_modem_t *m)
{
    if (m->switch_due)
        return;
    if (m->hunting && is_v32(m->mod) && !m->aa_heard && pick_v22(m) >= 0)
    {
        request_step(m, pick_v22(m), "no AA during the answer tone");
        return;
    }
    start_pump(m, "answer tone sent");
}

static const char *hunt_engaged(const dm_modem_t *m)
{
    if (m->v34 != NULL && dm_v34_engaged(m->v34))
        return m->calling ? "JM heard" : "CM heard";
    if (m->v32 != NULL && dm_v32_engaged(m->v32))
        return m->calling ? "AC heard" : "AA heard";
    /* The V.22 bis receiver has heard S1, or 270 ms of what really are
     * scrambled ones - not just something in its band, which a V.32
     * caller's AA or a 300 bps caller's carrier also is. */
    if (m->v22 != NULL && !m->calling &&
        (m->v22->negotiated_bit_rate == 2400 ||
         m->v22->rx.training == V22BIS_RX_TRAINING_STAGE_SCRAMBLED_ONES_AT_1200_SUSTAINING ||
         m->v22->rx.training == V22BIS_RX_TRAINING_STAGE_WAIT_FOR_SCRAMBLED_ONES_AT_2400 ||
         m->v22->rx.training == V22BIS_RX_TRAINING_STAGE_NORMAL_OPERATION))
        return "S1 or SB1 heard";
    if (m->fsk_rx != NULL && m->fsk_confirmed)
        return "its carrier heard";
    return NULL;
}

/* Once a frame, from dm_modem_tx with the lock held. */
static void hunt_tick(dm_modem_t *m, int samples)
{
    const char *why;

    if (m->no_v8)
    {
        m->no_v8 = false;
        after_v8(m);
    }
    if (m->switch_due)
        step_now(m);
    if (!m->hunting)
        return;
    if ((why = hunt_engaged(m)) != NULL)
    {
        settle(m, why);
        return;
    }
    /* A calling modem only listens - it is the answering one that offers. */
    if (m->calling || m->window < 0 || m->phase == DM_PHASE_ANSWER_TONE)
        return;
    m->window_left -= samples;
    if (m->window_left > 0)
        return;
    /* Something in the low band is being looked at as SB1 right now; let it
     * be decided, but not for long, since a 300 bps caller's carrier looks
     * like that for ever. */
    if (m->v22 != NULL && m->v22->rx.training == V22BIS_RX_TRAINING_STAGE_SCRAMBLED_ONES_AT_1200 &&
        m->window_left > -DM_SAMPLE_RATE / 2)
        return;
    {
        char what[64];

        snprintf(what, sizeof(what), "nobody answered %s", m->mod_name);
        next_offer(m, what);
    }
}

/* A calling FSK modem holds its carrier at mark this long before it passes
 * data, so that the answering one - which only connects once it has heard
 * that carrier - is listening by the time the first character arrives. */
#define DM_FSK_MARK_MS 500

/* An FSK link is connected once the far end's carrier is up, its tone has
 * been heard as well, and our own pump is running - and, calling, our own
 * carrier has had time to be heard. An FSK receiver takes any energy at all
 * for a carrier: a V.32 caller's AA, our own echo, or - to a calling V.23
 * modem, which then read 2100 Hz space for most of a minute - an answering
 * modem's USB1 at 2250 Hz. Only the far end's own tone in its band counts. */
static void fsk_maybe_connect(dm_modem_t *m)
{
    if (!is_fsk(m->mod) || !m->fsk_confirmed || !m->fsk_carrier || !m->pump_started || m->connected)
        return;
    if (m->calling &&
        (m->fsk_mark_from < 0 ||
         m->samples - m->fsk_mark_from < (long long) (DM_FSK_MARK_MS + m->path_delay_ms / 2) * DM_SAMPLE_RATE / 1000))
        return;
    note_connected(m);
}

static void fsk_heard(dm_modem_t *m, dm_mod_t mod, heard_t h)
{
    if (m->mod == mod)
    {
        if (!m->fsk_confirmed)
        {
            m->fsk_confirmed = true;
            DM_DEBUG("modem", "%s heard (tag=%s)", heard_name(h), m->tag);
        }
        settle(m, heard_name(h));
        /* A calling modem sitting out the rest of an answer tone that has
         * already finished. */
        if (m->calling && m->phase == DM_PHASE_ANSWER_TONE)
            start_pump(m, heard_name(h));
        fsk_maybe_connect(m);
        return;
    }
    if (!m->hunting || !may_run(m, mod))
        return;
    request_step(m, (int) mod, heard_name(h));
    m->switch_confirmed = true;
    settle(m, heard_name(h));
}

/* A signal has been heard for `run` blocks running. From dm_modem_rx, with
 * the lock held, after the pump has had the audio. */
static void hunt_heard(dm_modem_t *m, heard_t h, int run)
{
    /* Each needs 160 ms of itself: Annex A's 155 ms for USB1, and long
     * enough for the others that a passing coincidence does not count. */
    if (run < 4)
        return;
    /* A Bell 103 calling modem's receiver takes anything in its band for
     * the answering carrier, and an answer tone at 2100 Hz or USB1 at 2250
     * is in it. So a real one raises its 1270 Hz carrier during them, and an
     * automode answerer - a Cisco MICA is one - listens for exactly that
     * before it offers Bell 103: it plays its answer tone, USB1, V.21 and
     * V.23 in turn and never 2225 Hz unprompted. Ours goes on air the same
     * way, but still connects only on hearing 2225 Hz itself.
     *
     * A V.23 caller does the same, as ours always has. A V.21 caller does
     * not: V.21's answering band does not reach 2100 Hz, so a real one waits
     * for 1650 Hz, and speaking first gains nothing - the Level 29 BBS's
     * modem, a V.8 one offering USB1 alone, took a Bell 103 caller during it
     * and ignored 38 seconds of V.21's 980 Hz. */
    if (m->calling && is_fsk(m->mod) && m->mod != DM_MOD_V21 && !m->fsk_confirmed && !m->fsk_announced &&
        (h == HEARD_ANS || h == HEARD_USB1))
    {
        m->fsk_announced = true;
        DM_INFO("modem", "%s heard; raising our %s carrier so the far end can tell what we are "
                         "(tag=%s)", heard_name(h), m->mod_name, m->tag);
    }
    switch (h)
    {
    case HEARD_AC:
        if (m->calling && m->hunting && m->v32 == NULL)
            request_step(m, pick_v32(m), "AC heard");
        break;
    case HEARD_USB1:
        if (!m->calling || !m->hunting)
            break;
        if (m->v22 != NULL)
        {
            settle(m, heard_name(h));
            if (m->phase == DM_PHASE_ANSWER_TONE)
                start_pump(m, heard_name(h));
            break;
        }
        /* Answered at once, as a V.22 bis caller would, whatever was going
         * on - V.8's CM included, which no V.8 answerer would have met with
         * these.
         *
         * Annex A has a caller that has not sent AA wait Tc > 3.1 s first
         * (A.2.1.2), in case the answerer is an automode one that will offer
         * AC next. Its own Note 1 says what that costs: V.22 bis does not
         * say how long USB1 lasts, and modems that stop sooner will not
         * interwork. A real 2400 bps modem did exactly that - three seconds
         * of USB1 and on to V.21 - and was missed. An automode V.32 answerer
         * that hears S1 instead connects in V.22 bis, which is the lesser
         * loss; and it only gets as far as USB1 when it heard no AA during
         * its answer tone, which this end does send after a plain one. */
        if (pick_v22(m) >= 0)
        {
            request_step(m, pick_v22(m), m->v34 != NULL ? "V.22's unscrambled ones in answer to V.8: no V.8 there"
                                                        : heard_name(h));
            settle(m, heard_name(h));
        }
        break;
    case HEARD_V21_ANS:
        /* V.8's JM is V.21 channel 2 too, and V.8 decodes one in a few
         * hundred milliseconds. A second of channel 2 with no JM in it is a
         * V.21 answerer - which, if our CM is going out, has taken that for
         * a calling carrier and connected to it, so go to V.21 before the
         * CM stops and it gives up on us. */
        if (m->calling && (m->v34 == NULL || run >= 25))
            fsk_heard(m, DM_MOD_V21, h);
        break;
    case HEARD_BELL_ANS:
        if (m->calling)
            fsk_heard(m, DM_MOD_BELL103, h);
        break;
    case HEARD_AA:
        if (m->calling)
            break;
        if (!m->aa_heard)
        {
            m->aa_heard = true;
            DM_DEBUG("modem", "AA heard: the caller is a V.32 modem (tag=%s)", m->tag);
        }
        /* V.8 and the answer tone are seen out first; V.32's pump hears it
         * for itself. */
        if (m->hunting && m->v32 == NULL && m->v34 == NULL && m->phase != DM_PHASE_ANSWER_TONE)
            request_step(m, pick_v32(m), heard_name(h));
        break;
    case HEARD_V21_ORIG:
        /* A V.21 caller says nothing until it hears our channel 2, so before
         * that has been offered this is something else in its band - V.8's
         * CM, from a V.34 caller that took our answer tone for ANSam, is
         * V.21 channel 1 too. Once offered, a caller that answers late, after
         * we have moved on, is still taken. */
        if (!m->calling && m->offered_v21 && m->v34 == NULL)
            fsk_heard(m, DM_MOD_V21, h);
        break;
    case HEARD_BELL_ORIG:
        /* Unlike V.21's, a Bell 103 caller may speak first: it takes our
         * answer tone or USB1 for its carrier (see above). Nothing else a
         * caller sends sits at 1270 Hz - V.8's CM is at 980 and 1180. */
        if (!m->calling)
            fsk_heard(m, DM_MOD_BELL103, h);
        break;
    case HEARD_V23_ANS:
        if (m->calling)
            fsk_heard(m, DM_MOD_V23, h);
        break;
    case HEARD_V23_ORIG:
        if (!m->calling)
            fsk_heard(m, DM_MOD_V23, h);
        break;
    default:
        break;
    }
}

static void listen_feed(dm_modem_t *m, const int16_t *x, int count)
{
    dm_listen_t *l = &m->listen;

    while (count > 0 && listening(m))
    {
        int take = DM_LISTEN_BLOCK - l->n;
        heard_t h;

        if (take > count)
            take = count;
        memcpy(l->buf + l->n, x, (size_t) take * sizeof(int16_t));
        l->n += take;
        x += take;
        count -= take;
        if (l->n < DM_LISTEN_BLOCK)
            break;
        l->n = 0;
        listen_own(m);
        h = listen_block(l);
        if (h == HEARD_LOW_FSK)
        {
            /* FSK keeps a third or more of its power at its own two tones
             * even carrying data; V.22's low channel, also in this band,
             * spreads its evenly over 600 Hz. */
            float floor = 0.3f * (float) ++l->fsk_blocks;

            l->v21_sum += l->v21_tones;
            l->bell_sum += l->bell_tones;
            if (l->v21_sum >= 1.5f * l->bell_sum && l->v21_sum >= floor)
                h = HEARD_V21_ORIG;
            else if (l->bell_sum >= 1.5f * l->v21_sum && l->bell_sum >= floor)
                h = HEARD_BELL_ORIG;
            else
                h = HEARD_NONE;
        }
        else
        {
            l->v21_sum = l->bell_sum = 0.0f;
            l->fsk_blocks = 0;
        }
        for (int k = 1; k < HEARD_COUNT; k++)
            l->run[k] = (k == (int) h) ? l->run[k] + 1 : 0;
        if (h != HEARD_NONE)
        {
            if (l->run[h] == 1)
                DM_TRACE("modem", "hearing %s (tag=%s)", heard_name(h), m->tag);
            hunt_heard(m, h, l->run[h]);
        }
    }
}

dm_modem_t *dm_modem_create(const dm_modem_params_t *params)
{
    dm_modem_t *m;
    char err[256];

    if (!dm_modem_params_check(params, err, sizeof(err)))
    {
        DM_ERROR("modem", "%s", err);
        return NULL;
    }

    m = calloc(1, sizeof(*m));
    if (m == NULL)
        return NULL;

    pthread_mutex_init(&m->lock, NULL);
    parse_modulation(params->modulation, &m->mod);
    m->mod = start_mod(m->mod, params->bit_rate, params->step_down);
    parse_parity(params->parity, &m->parity);
    parse_guard(params->guard_tone, &m->guard);
    m->mod_name = modulation_name(m->mod);
    m->calling = params->calling;
    m->data_bits = params->data_bits;
    m->stop_bits = params->stop_bits;
    m->v14 = params->v14;
    m->tx_power = (float) params->tx_power;
    m->path_delay_ms = params->path_delay_ms;
    m->ceiling = m->mod;
    m->step_down = params->step_down;
    m->rate_cap = params->bit_rate;
    set_modulation(m, m->mod);
    m->hunting = can_step_down(m);
    m->fsk_mark_from = -1;
    m->answer_hold_until = 0;
    m->window = (m->mod == DM_MOD_V34) ? -1 : cycle_index(m->mod);
    m->window_left = window_samples(m, m->mod);
    m->phase = DM_PHASE_IDLE;
    m->last_rx_ms = dm_now_ms();
    snprintf(m->tag, sizeof(m->tag), "%s", params->tag ? params->tag : (params->calling ? "out" : "in"));

    /* The guard tone belongs to the answering modem; a calling modem that
     * sent one would be talking over the channel it needs to listen to. */
    if (m->calling && m->guard != V22BIS_GUARD_TONE_NONE)
    {
        DM_DEBUG("modem", "ignoring the guard tone setting: only the answering modem sends one");
        m->guard = V22BIS_GUARD_TONE_NONE;
    }

    /* Sized for the fastest we will run; stepping down only lowers the limit
     * on how much of it may be used. */
    if (!dm_ring_init(&m->tx, m->tx_limit) || !dm_ring_init(&m->rx, DM_RX_QUEUE))
    {
        dm_modem_destroy(m);
        return NULL;
    }

    m->arx = async_rx_init(NULL, m->data_bits, m->parity, m->stop_bits, m->v14 ? TRUE : FALSE,
                           rx_put_byte, m);
    m->arx_monitor = async_rx_init(NULL, m->data_bits, m->parity, m->stop_bits, FALSE,
                                   rx_monitor_byte, m);
    if (m->arx == NULL || m->arx_monitor == NULL)
    {
        DM_ERROR("modem", "async_rx_init failed");
        dm_modem_destroy(m);
        return NULL;
    }

    parse_v42_mode(params->v42, &m->v42_mode);
    if (m->v42_mode == DM_V42_DETECT && !carries_v42(m->mod))
    {
        DM_DEBUG("modem", "no V.42 over %s (tag=%s)", m->mod_name, params->tag ? params->tag : "");
        m->v42_mode = DM_V42_OFF;
    }
    m->v42_timeout_s = params->v42_timeout_s > 0 ? params->v42_timeout_s : 10;
#if defined(DATAMODEM_VENDORED_V42)
    /* Test hook, not an option: see third_party/spandsp-v42. Reproduces a far
     * end that answers the first few SABMEs with DM. */
    {
        const char *refuse = getenv("DATAMODEM_V42_REFUSE_SABME");

        if (refuse != NULL && *refuse != '\0')
        {
            dm_v42_refuse_sabme = atoi(refuse);
            DM_WARN("modem", "DATAMODEM_V42_REFUSE_SABME=%d: the answering end will refuse that "
                             "many SABMEs with DM. This is a test hook.", dm_v42_refuse_sabme);
        }
        {
            extern int dm_v22bis_deaf_to_s1;

            if (getenv("DATAMODEM_V22BIS_DEAF_TO_S1") != NULL)
            {
                dm_v22bis_deaf_to_s1 = 1;
                DM_WARN("modem", "DATAMODEM_V22BIS_DEAF_TO_S1: the calling end will not hear the "
                                 "answerer's S1, so the two will disagree about the rate. This is "
                                 "a test hook.");
            }
        }
        if (getenv("DATAMODEM_V42_NO_XID") != NULL)
        {
            dm_v42_no_xid = 1;
            DM_WARN("modem", "DATAMODEM_V42_NO_XID: this end will establish V.42 without an XID "
                             "exchange. This is a test hook.");
        }
        if (getenv("DATAMODEM_V42_ANSWER_NO_EC") != NULL)
        {
            dm_v42_answer_no_ec = 1;
            DM_WARN("modem", "DATAMODEM_V42_ANSWER_NO_EC: the answering end will send the "
                             "\"no error-correcting protocol desired\" ADP. This is a test hook.");
        }
    }
#endif
    if (m->v42_mode != DM_V42_OFF)
    {
        /* detect = TRUE: run the V.42 detection phase (ODP from the caller,
         * ADP from the answerer) rather than opening with SABME. It is the
         * standard handshake, and it is what a far end doing auto-reliable
         * is listening for. */
        m->v42 = v42_init(NULL, m->calling ? TRUE : FALSE, TRUE, v42_iframe_get, v42_iframe_put, m);
        if (m->v42 == NULL)
        {
            DM_ERROR("modem", "v42_init failed");
            dm_modem_destroy(m);
            return NULL;
        }
        if (!dm_ring_init(&m->pending_rx, DM_PENDING_RX))
        {
            dm_modem_destroy(m);
            return NULL;
        }
        v42_set_status_callback(m->v42, v42_status, m);
        /* What our XID offers. spandsp's own default is compression in one
         * direction only, with a 512 codeword dictionary and 6-character
         * strings, whatever --v42bis says; and with --v42bis off it should
         * offer none at all. */
        m->v42->config.comp = params->v42bis ? V42BIS_P0_BOTH_DIRECTIONS : 0;
        m->v42->config.comp_dict_size = params->v42bis_dict;
        m->v42->config.comp_max_string = params->v42bis_max_string;
        /* spandsp gives the V.42 context its own logging state and sets it
         * to silent, and there is no accessor to reach it. Without this,
         * every word V.42 has to say about why a negotiation went wrong -
         * its own flow messages and the detection diagnostics in the
         * vendored source - is discarded before anyone can read it. */
        attach_logging(&m->v42->logging, m->tag);
        /* Armed here with the rate we are offering, so the context is in a
         * valid state through training, and armed again for real in
         * note_connected() once the negotiated rate is known. */
        v42_arm(m, m->tx_bit_rate);

        if (params->v42bis)
        {
            if (!dm_ring_init(&m->comp_out, DM_COMP_QUEUE))
            {
                dm_modem_destroy(m);
                return NULL;
            }
            /* max_encode_len is capped at the LAPM frame size so the
             * compressor never hands us a block that cannot go out whole. */
            m->v42bis = v42bis_init(NULL, V42BIS_P0_BOTH_DIRECTIONS, params->v42bis_dict,
                                    params->v42bis_max_string, v42bis_encoded, m, DM_LAPM_MAX_FRAME,
                                    v42bis_decoded, m, V42BIS_MAX_OUTPUT_LENGTH);
            if (m->v42bis == NULL)
            {
                DM_ERROR("modem", "v42bis_init failed (dictionary %d, max string %d)",
                         params->v42bis_dict, params->v42bis_max_string);
                dm_modem_destroy(m);
                return NULL;
            }
            v42bis_compression_control(m->v42bis, V42BIS_COMPRESSION_MODE_DYNAMIC);
            /* An offer. The XID exchange decides what actually runs - see
             * apply_v42bis_agreement(). */
            m->comp_tx = m->comp_rx = true;
            m->v42bis_dict = params->v42bis_dict;
            m->v42bis_max_string = params->v42bis_max_string;
            DM_DEBUG("modem", "offering V.42bis with a dictionary of %d codewords and strings up to %d "
                              "(tag=%s)", params->v42bis_dict, params->v42bis_max_string, m->tag);
        }
    }

    if (!create_pump(m, m->calling && params->answer_wait_s > 0))
    {
        dm_modem_destroy(m);
        return NULL;
    }

    /* Pre-carrier signalling. The answering end announces itself with a 2100
     * Hz ANS burst before it starts training, the way V.25 says and the way
     * every piece of equipment on the far end expects; the calling end waits
     * to hear one before it bothers training. */
    m->answer_tone_samples = params->answer_tone_ms * DM_SAMPLE_RATE / 1000;
    if (m->mod == DM_MOD_BELL103 || m->mod == DM_MOD_V34)
    {
        /* Nothing to send: the answering carrier is the tone. */
        m->answer_tone_samples = 0;
    }
    m->wait_samples = params->answer_wait_s * DM_SAMPLE_RATE;
    m->ans_tail_samples = params->answer_tail_ms * DM_SAMPLE_RATE / 1000;
    /* V.32 does not have to sit the answer tone out. 5.4.1 has the caller
     * start AA once it has heard ANS for a second, and its receiver is
     * listening for AC at 600 and 3000 Hz, which 2100 Hz cannot be mistaken
     * for. An answering modem that does V.32 automode (Annex A) listens for
     * AA during its answer tone and is waiting for it. */
    if (is_v32(m->mod))
        m->ans_tail_samples = DM_SAMPLE_RATE;

    /* The answer tone has to finish before the calling modem starts, and
     * starting early does not work: 2100 Hz sits close enough to the
     * answering modem's band that the demodulator locks onto the tone,
     * declares itself trained, and drops again the moment the tone stops.
     *
     * spandsp's detector reports the tone as an event rather than a level,
     * so there is nothing to watch for it stopping - and V.25 does not fix
     * the length anyway, it allows 2.6 to 4 seconds. So the caller waits a
     * fixed tail after hearing it. Being late costs nothing: the answering
     * modem sits in its training pattern until we answer it.
     *
     * Bell 103 is the exception, and skips the whole phase. It has no
     * answer tone: the answering modem simply raises its 2225 Hz mark
     * carrier. There is nothing to detect and nothing to wait out, and an
     * FSK receiver cannot be fooled into a false connection by a tone the
     * way a QAM one can - it either sees a carrier or it does not. */
    if (m->calling)
    {
        /* V.34's answer tone is V.8's ANSam, and listening for it is the
         * data pump's own business. */
        if (m->wait_samples > 0 && m->mod != DM_MOD_BELL103 && m->mod != DM_MOD_V34)
        {
            m->tone_rx = modem_connect_tones_rx_init(NULL, ans_tone_type(m->mod), NULL, NULL);
            if (params->calling_tone)
            {
                m->tone_tx = modem_connect_tones_tx_init(NULL, MODEM_CONNECT_TONES_CALLING_TONE);
                if (m->tone_tx != NULL)
                    DM_DEBUG("modem", "sending the V.25 calling tone while waiting (tag=%s)", m->tag);
            }
            m->phase = DM_PHASE_ANSWER_TONE;
        }
    }
    else if (m->answer_tone_samples > 0)
    {
        /* V.32 has an echo canceller of its own, and the network's would
         * fight it: the phase reversals every 450 ms are V.25's signal for
         * echo cancellers along the path to stand aside. */
        int tone = is_v32(m->mod) ? MODEM_CONNECT_TONES_ANS_PR : MODEM_CONNECT_TONES_ANS;

        /* Test hook, not an option: an answer tone that a V.34 caller will
         * take for ANSam, as one from a real older modem has been - so that
         * the caller's V.8 goes unanswered and its fall back gets run. */
        if (getenv("DATAMODEM_ANSWER_ANSAM") != NULL)
        {
            tone = MODEM_CONNECT_TONES_ANSAM_PR;
            DM_WARN("modem", "DATAMODEM_ANSWER_ANSAM: sending ANSam as a %s answerer, which has no V.8. "
                             "This is a test hook.", m->mod_name);
        }
        m->tone_tx = modem_connect_tones_tx_init(NULL, tone);
        if (m->tone_tx == NULL)
        {
            DM_ERROR("modem", "could not start the answer tone generator");
            dm_modem_destroy(m);
            return NULL;
        }
        m->phase = DM_PHASE_ANSWER_TONE;
    }

    if (m->phase != DM_PHASE_ANSWER_TONE)
        start_pump(m, "no answer-tone phase");
    record_open(m);

    dm_log_event(DM_LOG_INFO, "modem", "engine started",
                 "tag=%s modulation=%s step_down=%s role=%s rate=%d format=%d%c%d guard=%s answer_tone_ms=%d "
                 "answer_wait_s=%d v42=%s v42bis=%s",
                 m->tag, m->mod_name, m->hunting ? "on" : "off", m->calling ? "originate" : "answer",
                 m->offered_rate, m->data_bits,
                 m->parity == ASYNC_PARITY_NONE ? 'N' : (m->parity == ASYNC_PARITY_EVEN ? 'E' : 'O'),
                 m->stop_bits, params->guard_tone ? params->guard_tone : "none", params->answer_tone_ms,
                 params->answer_wait_s,
                 m->v42_mode == DM_V42_OFF ? "off" : (m->v42_mode == DM_V42_REQUIRE ? "require"
                                                                                    : "detect"),
                 m->v42bis != NULL ? "on" : "off");
    return m;
}

void dm_modem_destroy(dm_modem_t *m)
{
    if (m == NULL)
        return;

    pthread_mutex_lock(&m->lock);
    record_close(m);
    free_pump(m);
    if (m->arx != NULL)
    {
        async_rx_release(m->arx);
        async_rx_free(m->arx);
    }
    if (m->arx_monitor != NULL)
    {
        async_rx_release(m->arx_monitor);
        async_rx_free(m->arx_monitor);
    }
    if (m->v42bis != NULL)
    {
        v42bis_release(m->v42bis);
        v42bis_free(m->v42bis);
    }
    if (m->v42 != NULL)
    {
        v42_release(m->v42);
        v42_free(m->v42);
    }
    if (m->tone_tx != NULL)
    {
        modem_connect_tones_tx_release(m->tone_tx);
        modem_connect_tones_tx_free(m->tone_tx);
    }
    if (m->tone_rx != NULL)
    {
        modem_connect_tones_rx_release(m->tone_rx);
        modem_connect_tones_rx_free(m->tone_rx);
    }
    pthread_mutex_unlock(&m->lock);
    pthread_mutex_destroy(&m->lock);

    dm_ring_destroy(&m->tx);
    dm_ring_destroy(&m->rx);
    dm_ring_destroy(&m->comp_out);
    dm_ring_destroy(&m->pending_rx);
    free(m);
}

void dm_modem_set_wake(dm_modem_t *m, void (*wake)(void *user), void *user)
{
    pthread_mutex_lock(&m->lock);
    m->wake = wake;
    m->wake_user = user;
    pthread_mutex_unlock(&m->lock);
}

/* --------------------------------------------------------- audio pumping */

void dm_modem_arm(dm_modem_t *m)
{
    pthread_mutex_lock(&m->lock);
    if (!m->armed)
    {
        m->armed = true;
        m->last_rx_ms = dm_now_ms();
        DM_DEBUG("modem", "call answered, starting the modem (tag=%s)", m->tag);
    }
    pthread_mutex_unlock(&m->lock);
}

bool dm_modem_armed(dm_modem_t *m)
{
    bool a;

    pthread_mutex_lock(&m->lock);
    a = m->armed;
    pthread_mutex_unlock(&m->lock);
    return a;
}

/* Re-offers the line rate we actually ended up at, and retrains.
 *
 * V.22bis negotiates 2400 with the S1 pattern: the calling modem sends it,
 * and the answering modem echoes it back if it agrees. Both decisions are
 * made from what each end heard, within a window that closes for good when
 * training ends. On a phone line that is safe. Over RTP it is not: lose the
 * 100 ms of audio carrying the far end's reply and we conclude 1200 while
 * the far end, having heard our S1 perfectly well, concludes 2400. Each then
 * demodulates the other at the wrong rate. There is no renegotiation in
 * V.22bis to recover with, so the link delivers structured garbage, retrains,
 * and does it again until somebody clears the call.
 *
 * Offering only the rate we settled at removes the question. We no longer
 * send S1, so the answering modem has nothing to agree to and must stay at
 * 1200 - which is exactly the configuration --bit-rate 1200 produces, and
 * which works. It costs one retrain, and only on a call that was failing.
 *
 * Deferred out of the status callback because spandsp raises that from
 * inside v22bis_rx(), on state this would reset underneath it. */
static void apply_rate_ceiling(dm_modem_t *m)
{
    int rate = m->drop_ceiling_to;
    int was = m->offered_rate;

    m->drop_ceiling_to = 0;
    if (m->v22 == NULL || rate <= 0 || m->rate_pinned)
        return;
    m->rate_pinned = true;
    DM_WARN("modem", "the link settled at %d having offered %d and will not hold, which means the "
                     "two ends disagreed about the rate; re-offering %d only, so there is nothing "
                     "left to disagree about (tag=%s)", rate, was, rate, m->tag);
    m->offered_rate = rate;
    m->tx_bit_rate = rate;
    m->bit_rate = rate;
    v42_set_bit_rate(m, rate);
    v22bis_restart(m->v22, rate);
    m->phase = DM_PHASE_TRAINING;
    m->connected = false;
    m->carrier_down_ms = 0;
    m->rate_check_ms = 0;
}

int dm_modem_tx(dm_modem_t *m, int16_t *samples, int max_count)
{
    int n = 0;

    if (m == NULL || max_count <= 0)
        return 0;

    pthread_mutex_lock(&m->lock);
    if (!m->armed)
    {
        /* Ringing, or early media. Stay silent and let no timer run. */
        memset(samples, 0, (size_t) max_count * sizeof(int16_t));
        pthread_mutex_unlock(&m->lock);
        return max_count;
    }
    m->samples += max_count;
    if (m->drop_ceiling_to > 0)
        apply_rate_ceiling(m);
    hunt_tick(m, max_count);
    if (m->phase == DM_PHASE_ANSWER_TONE)
    {
        if (m->v32 != NULL && m->calling)
        {
            /* Silence, unless a calling tone has been asked for - but it
             * keeps the V.32 receiver's clock running, so that it can hear
             * AC if the answering modem sends that without an answer tone. */
            int16_t tone[160];
            int k = 0;

            dm_v32_tx(m->v32, samples, max_count);
            while (m->tone_tx != NULL && k < max_count)
            {
                int chunk = (max_count - k > 160) ? 160 : max_count - k;
                int got = modem_connect_tones_tx(m->tone_tx, tone, chunk);

                for (int i = 0; i < got; i++)
                    samples[k + i] = saturate16((int32_t) samples[k + i] + tone[i]);
                k += chunk;
            }
            if (dm_v32_started(m->v32))
                start_pump(m, m->answer_tone_seen ? "AC heard" : "AC heard without an answer tone");
        }
        else
        {
            if (m->tone_tx != NULL)
                n = modem_connect_tones_tx(m->tone_tx, samples, max_count);
            if (n < max_count)
                memset(samples + n, 0, (size_t) (max_count - n) * sizeof(int16_t));
        }
        n = max_count;

        /* Transitions happen on frame boundaries. At 20 ms a frame that is
         * far finer than anything these timers measure. */
        if (!m->calling)
        {
            m->answer_tone_samples -= max_count;
            if (m->answer_tone_samples <= 0)
                answer_tone_done(m);
        }
        else
        {
            /* Hearing the tone can only ever push the start later, never
             * bring it forward. The detector missing a tone that is really
             * there is a survivable mistake - the fixed wait still covers
             * it. Training early because we wrongly believed the line was
             * quiet is not. */
            m->wait_samples -= max_count;
            if (m->ans_tail_running)
                m->ans_tail_samples -= max_count;

            /* V.32 is the exception: AA cannot be fooled by the tone, and
             * Annex A has it start once a second of ANS has been heard, so
             * that an automode answerer hears it during its answer tone. */
            if ((m->wait_samples <= 0 || (m->v32 != NULL && m->answer_tone_seen)) &&
                (!m->ans_tail_running || m->ans_tail_samples <= 0))
            {
                if (!m->answer_tone_seen)
                    DM_DEBUG("modem", "no answer tone heard; training anyway (tag=%s)", m->tag);
                start_pump(m, m->answer_tone_seen ? "answer tone finished" : "answer-tone wait elapsed");
            }
        }
    }
    else
    {
        /* The carrier has been gone longer than a retrain could take, so
         * it is not coming back. Keep transmitting either way: the far end
         * may still be trying, and cutting the signal only makes its
         * recovery harder. */
        check_v42_deadline(m);
        check_rate_agreement(m);

        if (m->carrier_down_ms != 0 && !m->carrier_lost &&
            dm_now_ms() - m->carrier_down_ms > m->carrier_grace_ms)
        {
            m->carrier_lost = true;
            m->phase = DM_PHASE_DOWN;
            dm_log_event(DM_LOG_INFO, "modem", "carrier lost",
                         "tag=%s bytes_tx=%" PRIu64 " bytes_rx=%" PRIu64, m->tag, m->bytes_tx,
                         m->bytes_rx);
        }

        if (m->v32 != NULL)
        {
            v32_test_renegotiate(m);
            n = dm_v32_tx(m->v32, samples, max_count);
        }
        else if (m->v34 != NULL)
        {
            v34_test_renegotiate(m);
            n = dm_v34_tx(m->v34, samples, max_count);
        }
        else if (m->v22 != NULL)
            n = v22bis_tx(m->v22, samples, max_count);
        else if (m->fsk_tx != NULL && m->calling && !m->fsk_confirmed && !m->fsk_announced)
            ; /* a calling FSK modem is silent until it hears the answering carrier */
        else if (m->fsk_tx != NULL)
        {
            if (m->fsk_mark_from < 0)
                m->fsk_mark_from = m->samples;
            n = fsk_tx(m->fsk_tx, samples, max_count);
            fsk_maybe_connect(m);
        }
    }

    /* RTP never stops, so silence has to be made explicit. */
    if (n < max_count)
    {
        memset(samples + n, 0, (size_t) (max_count - n) * sizeof(int16_t));
        n = max_count;
    }
    record_tx(m, samples, n);
    pthread_mutex_unlock(&m->lock);
    return n;
}

/* Caller holds m->lock.
 *
 * The callback runs under the lock deliberately. Dropping the lock first and
 * calling it afterwards would be tidier, but it opens a window in which the
 * session can deregister, close its pipe and return - leaving this thread
 * about to call a function pointer with a pointer to a stack frame that no
 * longer exists. Holding the lock across the call means dm_modem_set_wake()
 * cannot complete while a callback is in flight. It is safe to do because
 * the callback only writes one byte to a non-blocking pipe and never comes
 * back into the modem. */
static void fire_wake_locked(dm_modem_t *m)
{
    if (!m->wake_pending)
        return;
    m->wake_pending = false;
    if (m->wake != NULL)
        m->wake(m->wake_user);
}

/* Receive flow control under V.42: see DM_RX_BUSY_FREE. Once per audio
 * frame, on the media thread with the lock held - not from inside LAPM's own
 * delivery callback, which is where the queue fills. */
static void rx_flow_control(dm_modem_t *m)
{
    size_t free_space;

    if (m->v42 == NULL || !m->lapm_up)
        return;
    free_space = dm_ring_space(&m->rx);
    if (!m->rx_busy && free_space < DM_RX_BUSY_FREE)
    {
        m->rx_busy = true;
        v42_set_local_busy_status(m->v42, TRUE);
        DM_DEBUG("modem", "receive queue nearly full (%zu bytes free); asking the far end to wait "
                          "(tag=%s)",
                 free_space, m->tag);
    }
    else if (m->rx_busy && free_space >= DM_RX_READY_FREE)
    {
        m->rx_busy = false;
        v42_set_local_busy_status(m->v42, FALSE);
        DM_DEBUG("modem", "receive queue drained; letting the far end go on (tag=%s)", m->tag);
    }
}

void dm_modem_rx(dm_modem_t *m, const int16_t *samples, int count)
{
    if (m == NULL || count <= 0)
        return;

    pthread_mutex_lock(&m->lock);
    if (!m->armed)
    {
        /* Ringback is not a modem signal. Listening to it is how you end up
         * detecting a carrier that does not exist. */
        pthread_mutex_unlock(&m->lock);
        return;
    }
    record_rx(m, samples, count);
    if (m->phase == DM_PHASE_ANSWER_TONE && m->calling)
    {
        if (m->v32 != NULL)
            dm_v32_rx(m->v32, samples, count);
        if (m->tone_rx != NULL && !m->answer_tone_seen)
        {
            modem_connect_tones_rx(m->tone_rx, samples, count);
            if (modem_connect_tones_rx_get(m->tone_rx) != MODEM_CONNECT_TONES_NONE)
            {
                m->answer_tone_seen = true;
                if (m->ans_tail_samples > 0)
                {
                    m->ans_tail_running = true;
                    DM_INFO("modem", "answer tone from the far end; letting it finish (%dms) (tag=%s)",
                            m->ans_tail_samples * 1000 / DM_SAMPLE_RATE, m->tag);
                }
                else
                {
                    start_pump(m, "answer tone heard");
                }
            }
        }
    }
    else if (m->v32 != NULL)
    {
        dm_v32_rx(m->v32, samples, count);
    }
    else if (m->v34 != NULL)
    {
        dm_v34_rx(m->v34, samples, count);
    }
    else if (m->v22 != NULL)
    {
        v22bis_rx(m->v22, samples, count);
    }
    else if (m->fsk_rx != NULL)
    {
        fsk_rx(m->fsk_rx, samples, count);
    }
    if (listening(m))
        listen_feed(m, samples, count);
    rx_flow_control(m);
    fire_wake_locked(m);
    pthread_mutex_unlock(&m->lock);
}

void dm_modem_rx_missing(dm_modem_t *m, int count)
{
    if (m == NULL || count <= 0)
        return;

    pthread_mutex_lock(&m->lock);
    if (!m->armed)
    {
        pthread_mutex_unlock(&m->lock);
        return;
    }
    record_rx(m, NULL, count);
    if (m->v32 != NULL)
        dm_v32_rx_fillin(m->v32, count); /* its clock runs through the answer tone too */
    else if (m->v34 != NULL)
        dm_v34_rx_fillin(m->v34, count);
    else if (m->phase == DM_PHASE_ANSWER_TONE)
        ; /* nothing useful to fake into a tone detector */
    else if (m->v22 != NULL)
        v22bis_rx_fillin(m->v22, count);
    else if (m->fsk_rx != NULL)
        fsk_rx_fillin(m->fsk_rx, count);
    rx_flow_control(m);
    pthread_mutex_unlock(&m->lock);
}

/* ------------------------------------------------------- byte interface */

size_t dm_modem_send(dm_modem_t *m, const void *data, size_t len)
{
    size_t room = dm_modem_tx_space(m);

    return dm_ring_write(&m->tx, data, len < room ? len : room);
}

size_t dm_modem_recv(dm_modem_t *m, void *out, size_t max)
{
    return dm_ring_read(&m->rx, out, max);
}

size_t dm_modem_tx_pending(dm_modem_t *m)
{
    return dm_ring_len(&m->tx) + dm_ring_len(&m->comp_out);
}

bool dm_modem_drained(dm_modem_t *m)
{
    bool quiet;

    if (dm_modem_tx_pending(m) > 0)
        return false;

    pthread_mutex_lock(&m->lock);
    quiet = dm_now_ms() >= m->line_busy_until_ms + DM_LINE_TAIL_MS;
    pthread_mutex_unlock(&m->lock);
    return quiet;
}

size_t dm_modem_tx_space(dm_modem_t *m)
{
    size_t limit = __atomic_load_n(&m->tx_limit, __ATOMIC_RELAXED);
    size_t used = dm_ring_len(&m->tx);
    size_t space = dm_ring_space(&m->tx);
    size_t room = used >= limit ? 0 : limit - used;

    return room < space ? room : space;
}

size_t dm_modem_rx_pending(dm_modem_t *m)
{
    return dm_ring_len(&m->rx);
}

void dm_modem_tx_clear(dm_modem_t *m)
{
    dm_ring_clear(&m->tx);
    /* Not comp_out: those bytes are already inside the compressor's view of
     * the stream, and dropping them would leave the far end's dictionary
     * out of step with ours for the rest of the call. */
}

/* ----------------------------------------------------------------- state */

void dm_modem_status(dm_modem_t *m, dm_modem_status_t *out)
{
    memset(out, 0, sizeof(*out));
    if (m == NULL)
    {
        out->phase_text = "no modem";
        return;
    }
    pthread_mutex_lock(&m->lock);
    out->phase = m->phase;
    out->modulation = m->mod_name;
    out->bit_rate = m->bit_rate;
    out->offered_rate = m->offered_rate;
    out->connected = m->connected;
    out->carrier_lost = m->carrier_lost;
    out->answer_tone_seen = m->answer_tone_seen;
    out->connect_ms = m->connect_ms;
    out->bytes_tx = m->bytes_tx;
    out->bytes_rx = m->bytes_rx;
    out->wire_tx = m->wire_tx;
    out->wire_rx = m->wire_rx;
    out->retrains = m->retrains;
    if (m->v32 != NULL)
    {
        dm_v32_stats_t vs;

        dm_v32_stats(m->v32, &vs);
        out->rx_power = vs.rx_power;
        out->snr_db = vs.snr_db;
        out->line_trellis = vs.trellis;
        out->line_v32bis = vs.v32bis;
        out->renegotiations = vs.renegotiations;
        out->round_trip_ms = vs.round_trip_ms;
        out->echo_cancelling = vs.echo_canceller;
        out->echo_delay_ms = vs.echo_delay_ms;
        out->echo_return_loss_db = vs.echo_return_loss_db;
        out->echo_cancelled_db = vs.echo_cancelled_db;
        out->train_stage = vs.stage;
    }
    else if (m->v34 != NULL)
    {
        dm_v34_stats_t vs;

        dm_v34_stats(m->v34, &vs);
        out->rx_power = vs.rx_power;
        out->snr_db = vs.snr_db;
        out->line_trellis = true;
        out->renegotiations = vs.renegotiations;
        out->round_trip_ms = vs.round_trip_ms;
        out->echo_cancelling = vs.echo_canceller;
        out->echo_delay_ms = vs.echo_delay_ms;
        out->echo_return_loss_db = vs.echo_return_loss_db;
        out->echo_cancelled_db = vs.echo_cancelled_db;
        out->train_stage = vs.stage;
        out->tx_bit_rate = vs.tx_rate;
        out->symbol_rate = vs.rx_symbol_rate;
        out->tx_symbol_rate = vs.tx_symbol_rate;
    }
    else if (m->v22 != NULL)
        out->rx_power = v22bis_rx_signal_power(m->v22);
    else if (m->fsk_rx != NULL)
        out->rx_power = fsk_rx_signal_power(m->fsk_rx);
    out->lapm_up = m->lapm_up;
    out->v42_fell_back = m->v42_fell_back;
    out->frame_errors = m->frame_errors;
    out->protocol = protocol_name(m);
    out->data_ready = m->connected && !m->carrier_lost &&
                      (m->v42 == NULL || m->lapm_up || m->v42_fell_back);
    pthread_mutex_unlock(&m->lock);

    out->phase_text = dm_modem_phase_name(out->phase);
    out->tx_dropped = dm_ring_dropped(&m->tx);
    out->rx_dropped = dm_ring_dropped(&m->rx);
}

bool dm_modem_data_ready(dm_modem_t *m)
{
    bool ready;

    pthread_mutex_lock(&m->lock);
    ready = m->connected && !m->carrier_lost &&
            (m->v42 == NULL || m->lapm_up || m->v42_fell_back);
    pthread_mutex_unlock(&m->lock);
    return ready;
}

bool dm_modem_connected(dm_modem_t *m)
{
    bool c;

    pthread_mutex_lock(&m->lock);
    c = m->connected && !m->carrier_lost;
    pthread_mutex_unlock(&m->lock);
    return c;
}

dm_modem_phase_t dm_modem_phase(dm_modem_t *m)
{
    dm_modem_phase_t p;

    pthread_mutex_lock(&m->lock);
    p = m->phase;
    pthread_mutex_unlock(&m->lock);
    return p;
}

const char *dm_modem_phase_name(dm_modem_phase_t phase)
{
    switch (phase)
    {
    case DM_PHASE_IDLE:
        return "idle";
    case DM_PHASE_ANSWER_TONE:
        return "answer tone";
    case DM_PHASE_TRAINING:
        return "training";
    case DM_PHASE_DATA:
        return "data";
    case DM_PHASE_DOWN:
        return "carrier down";
    }
    return "?";
}

int64_t dm_modem_since_rx_ms(dm_modem_t *m)
{
    int64_t since;

    pthread_mutex_lock(&m->lock);
    since = dm_now_ms() - m->last_rx_ms;
    pthread_mutex_unlock(&m->lock);
    return since;
}

/* -------------------------------------------------------------- selftest */

#define SELFTEST_CHUNK 160 /* 20 ms at 8 kHz, the same frame pjmedia uses */
#define SELFTEST_MAX_SECONDS 120
#define SELFTEST_BYTES 512

/* The line between the two modems. By default a perfect one - each modem
 * hears exactly what the other sent, in the same 20 ms frame - which is
 * what tells a modulation problem from a line problem.
 *
 * DATAMODEM_SELFTEST_LINE describes a worse one, for the things a perfect
 * line cannot exercise: "delay=150,echo=-10,noise=-40,ulaw". delay is one
 * way, in ms, as two jitter buffers and a network would make it; echo puts
 * each modem's own signal back into its receiver that many dB down, a whole
 * round trip later, the way the far end's hybrid does on a call into the
 * telephone network; noise is white noise in dBm0; ulaw passes everything
 * through G.711; burst=20/3 adds 20 ms of loud noise every 3 seconds, which
 * damages a frame or two without making the line look bad on average - how
 * V.42's recovery gets exercised; cut=20/600 silences the line both ways for
 * 600 ms, 20 seconds in, which a modem can only recover from by retraining;
 * drift=100 makes the answerer's clock 100 ppm fast as the caller hears it. A test hook, not an option - it is how V.32's echo
 * canceller and round-trip measurement get tested without a phone line. */
#define SELFTEST_LINE_MAX (SELFTEST_CHUNK * 128) /* 2.5 s of each direction */

typedef struct
{
    int delay;                /* one way, samples */
    int echo_delay;           /* samples */
    float echo_gain;          /* 0 = no echo */
    bool ulaw;
    awgn_state_t *noise;
    awgn_state_t *burst;
    int burst_len;            /* samples */
    int burst_every;          /* samples */
    long cut_at;              /* line silent from here, both ways, 0 = never */
    int cut_len;
    double drift;             /* the answerer's clock against the caller's, parts per million */
    int16_t hist[2][SELFTEST_LINE_MAX];
    long pos;
} selftest_line_t;

static bool selftest_line_init(selftest_line_t *ln, char *desc, size_t desc_len)
{
    const char *spec = getenv("DATAMODEM_SELFTEST_LINE");
    char buf[256];
    char *tok;
    char *save = NULL;
    double delay_ms = 0.0;
    double echo_db = 0.0;
    double noise_db = 0.0;
    bool echo = false;

    memset(ln, 0, sizeof(*ln));
    snprintf(desc, desc_len, "perfect");
    if (spec == NULL || *spec == '\0')
        return true;

    snprintf(buf, sizeof(buf), "%s", spec);
    for (tok = strtok_r(buf, ", ", &save); tok != NULL; tok = strtok_r(NULL, ", ", &save))
    {
        if (strncmp(tok, "delay=", 6) == 0)
            delay_ms = atof(tok + 6);
        else if (strncmp(tok, "echo=", 5) == 0)
        {
            echo_db = atof(tok + 5);
            echo = true;
        }
        else if (strncmp(tok, "noise=", 6) == 0)
            noise_db = atof(tok + 6);
        else if (strcmp(tok, "ulaw") == 0)
            ln->ulaw = true;
        else if (strncmp(tok, "drift=", 6) == 0)
        {
            ln->drift = atof(tok + 6);
            if (fabs(ln->drift) > 500.0)
            {
                DM_ERROR("selftest", "DATAMODEM_SELFTEST_LINE: drift= is in parts per million, at most 500");
                return false;
            }
        }
        else if (strncmp(tok, "cut=", 4) == 0)
        {
            double at = 0.0, ms = 0.0;

            if (sscanf(tok + 4, "%lf/%lf", &at, &ms) != 2 || at <= 0.0 || ms <= 0.0)
            {
                DM_ERROR("selftest", "DATAMODEM_SELFTEST_LINE: cut= wants seconds/ms, as in cut=20/600");
                return false;
            }
            ln->cut_at = (long) (at * DM_SAMPLE_RATE);
            ln->cut_len = (int) (ms * DM_SAMPLE_RATE / 1000.0);
        }
        else if (strncmp(tok, "burst=", 6) == 0)
        {
            double ms = 0.0;
            double every = 0.0;

            if (sscanf(tok + 6, "%lf/%lf", &ms, &every) != 2 || ms <= 0.0 || every <= 0.0)
            {
                DM_ERROR("selftest", "DATAMODEM_SELFTEST_LINE: burst= wants ms/seconds, as in burst=20/3");
                return false;
            }
            ln->burst_len = (int) (ms * DM_SAMPLE_RATE / 1000.0);
            ln->burst_every = (int) (every * DM_SAMPLE_RATE);
            ln->burst = awgn_init_dbm0(NULL, 7654321, -10.0f);
        }
        else
        {
            DM_ERROR("selftest", "DATAMODEM_SELFTEST_LINE: '%s' is not delay=, echo=, noise=, burst=, cut=, drift= or ulaw",
                     tok);
            return false;
        }
    }
    ln->delay = (int) (delay_ms * DM_SAMPLE_RATE / 1000.0);
    /* Back from the far end's line card: twice the one-way path, plus a
     * millisecond of local loop. */
    ln->echo_delay = 2 * ln->delay + DM_SAMPLE_RATE / 1000;
    if (ln->echo_delay >= SELFTEST_LINE_MAX - SELFTEST_CHUNK)
    {
        DM_ERROR("selftest", "DATAMODEM_SELFTEST_LINE: a %.0f ms delay is longer than the selftest's "
                             "line can hold", delay_ms);
        return false;
    }
    if (echo)
        ln->echo_gain = powf(10.0f, (float) echo_db / 20.0f);
    if (noise_db < 0.0)
        ln->noise = awgn_init_dbm0(NULL, 1234567, (float) noise_db);
    if (ln->drift != 0.0 && ln->delay + 256 >= SELFTEST_LINE_MAX - SELFTEST_CHUNK)
    {
        DM_ERROR("selftest", "DATAMODEM_SELFTEST_LINE: too much delay to drift as well");
        return false;
    }
    snprintf(desc, desc_len, "delay %.0f ms each way, echo %s, noise %s, %s", delay_ms,
             echo ? "on" : "none", ln->noise ? "on" : "none", ln->ulaw ? "G.711 mu-law" : "linear");
    if (echo)
        snprintf(desc + strlen(desc), desc_len - strlen(desc), " (echo %.0f dB, noise %.0f dBm0)",
                 echo_db, noise_db);
    return true;
}

static void selftest_line_free(selftest_line_t *ln)
{
    if (ln->noise != NULL)
        awgn_free(ln->noise);
    if (ln->burst != NULL)
        awgn_free(ln->burst);
}

/* One 20 ms frame each way. Both transmit before either receives, which is
 * the order pjmedia's conference bridge calls a port in - and V.32 measures
 * the round trip on the assumption that it is. */
static void selftest_line_run(selftest_line_t *ln, dm_modem_t *caller, dm_modem_t *answerer)
{
    int16_t out[2][SELFTEST_CHUNK];
    int16_t in[2][SELFTEST_CHUNK];
    dm_modem_t *end[2] = { caller, answerer };

    for (int e = 0; e < 2; e++)
    {
        int n = dm_modem_tx(end[e], out[e], SELFTEST_CHUNK);

        if (n < SELFTEST_CHUNK)
            memset(out[e] + n, 0, (size_t) (SELFTEST_CHUNK - n) * sizeof(int16_t));
    }
    for (int i = 0; i < SELFTEST_CHUNK; i++)
    {
        long t = ln->pos + i;

        ln->hist[0][t % SELFTEST_LINE_MAX] = out[0][i];
        ln->hist[1][t % SELFTEST_LINE_MAX] = out[1][i];
        for (int e = 0; e < 2; e++)
        {
            float x = (t >= ln->delay) ? ln->hist[1 - e][(t - ln->delay) % SELFTEST_LINE_MAX] : 0.0f;

            /* What the caller hears from an answerer whose clock runs fast or
             * slow: read the answerer's samples at a drifting instant, by
             * windowed sinc interpolation. 128 samples further back, so that
             * a few minutes' drift either way stays in the history. */
            if (ln->drift != 0.0 && e == 0)
            {
                double at = (double) t - ln->delay - 128.0 + (double) t * ln->drift * 1e-6;
                long i0 = (long) floor(at);
                double fr = at - (double) i0;
                double acc = 0.0;

                for (int k = -24; k <= 24; k++)
                {
                    double u = (double) k - fr;
                    double w = 0.42 + 0.5 * cos(M_PI * u / 25.0) + 0.08 * cos(2.0 * M_PI * u / 25.0);
                    double sinc = (fabs(u) < 1e-9) ? 1.0 : sin(M_PI * u) / (M_PI * u);
                    long j = i0 + k;

                    if (j >= 0)
                        acc += w * sinc * ln->hist[1][j % SELFTEST_LINE_MAX];
                }
                x = (float) acc;
            }

            if (ln->echo_gain > 0.0f && t >= ln->echo_delay)
                x += ln->echo_gain * ln->hist[e][(t - ln->echo_delay) % SELFTEST_LINE_MAX];
            if (ln->noise != NULL)
                x += awgn(ln->noise);
            /* Away from the start, so as to hit data rather than training. */
            if (ln->burst != NULL && t > 15 * DM_SAMPLE_RATE && t % ln->burst_every < ln->burst_len)
                x += awgn(ln->burst);
            if (x > 32767.0f)
                x = 32767.0f;
            else if (x < -32768.0f)
                x = -32768.0f;
            in[e][i] = (int16_t) lrintf(x);
            if (ln->ulaw)
                in[e][i] = ulaw_to_linear(linear_to_ulaw(in[e][i]));
            /* The line going dead for a moment, which only a retrain cures. */
            if (ln->cut_at > 0 && t >= ln->cut_at && t < ln->cut_at + ln->cut_len)
                in[e][i] = 0;
        }
    }
    ln->pos += SELFTEST_CHUNK;
    dm_modem_rx(answerer, in[1], SELFTEST_CHUNK);
    dm_modem_rx(caller, in[0], SELFTEST_CHUNK);
}

static void fill_pattern(unsigned char *p, size_t n, unsigned seed)
{
    /* Printable, so a mismatch is readable in the log, and not a repeating
     * run that a framing slip could hide inside. */
    for (size_t i = 0; i < n; i++)
        p[i] = (unsigned char) (' ' + ((i * 7 + seed) % 95));
}

/* Says where and how the stream went wrong, because "corrupted" on its own
 * tells you nothing about whether it was a framing slip, a lost character or
 * training noise arriving before the data. */
static bool compare_stream(const char *what, const unsigned char *sent, size_t sent_len,
                           const unsigned char *got, size_t got_len)
{
    size_t n = (got_len < sent_len) ? got_len : sent_len;
    size_t i;
    char a[64];
    char b[64];
    size_t ao = 0;
    size_t bo = 0;

    for (i = 0; i < n && sent[i] == got[i]; i++)
        ;
    if (i == n && got_len == sent_len)
        return true;

    if (i == n)
    {
        DM_ERROR("selftest", "%s: %zu bytes sent, %zu received - the stream is the wrong length", what,
                 sent_len, got_len);
        return false;
    }

    for (size_t k = i; k < i + 12 && k < sent_len && ao < sizeof(a) - 5; k++)
        ao += (size_t) snprintf(a + ao, sizeof(a) - ao, "%02x ", sent[k]);
    for (size_t k = i; k < i + 12 && k < got_len && bo < sizeof(b) - 5; k++)
        bo += (size_t) snprintf(b + bo, sizeof(b) - bo, "%02x ", got[k]);

    {
        size_t bad = 0;

        for (size_t k = i; k < n; k++)
            bad += (sent[k] != got[k]);
        DM_ERROR("selftest", "%s: first difference at byte %zu of %zu (%zu received); %zu of the "
                             "first %zu differ",
                 what, i, sent_len, got_len, bad, n);
    }
    DM_ERROR("selftest", "  sent     %s", a);
    DM_ERROR("selftest", "  received %s", b);
    return false;
}

/* DATAMODEM_REPLAY=call.wav, a test hook: instead of two modems back to
 * back, one calling modem against a recording of a real far end - the left
 * channel of what DATAMODEM_RECORD wrote, or any 8 kHz 16-bit WAV. The far
 * end cannot answer what we send now, only what was sent on the day, so
 * past the first exchange the two drift apart; but everything up to it -
 * which answer tone this is, when CM goes out, what is heard after it and
 * what we step down to - is played out exactly, and said with the time into
 * the recording at which it happened. */
static int selftest_replay(const dm_config_t *cfg, const char *path)
{
    FILE *f = fopen(path, "rb");
    unsigned char hdr[12];
    unsigned char chunk[8];
    int channels = 0;
    int bits = 0;
    uint32_t rate = 0;
    long data_len = -1;
    dm_modem_params_t params;
    dm_modem_t *m;
    dm_modem_status_t st;
    char last_mod[16] = "";
    char last_stage[96] = "";
    int last_phase = -1;
    bool ready = false;
    long frames = 0;
    unsigned char got[4096];
    size_t got_n = 0;

    if (f == NULL || fread(hdr, 1, 12, f) != 12 || memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0)
    {
        DM_ERROR("selftest", "DATAMODEM_REPLAY: %s is not a WAV file", path);
        if (f != NULL)
            fclose(f);
        return DM_EXIT_CONFIG;
    }
    while (fread(chunk, 1, 8, f) == 8)
    {
        long len = (long) (chunk[4] | chunk[5] << 8 | chunk[6] << 16 | (uint32_t) chunk[7] << 24);

        if (memcmp(chunk, "fmt ", 4) == 0)
        {
            unsigned char fmt[16];

            if (len < 16 || fread(fmt, 1, 16, f) != 16)
                break;
            channels = fmt[2] | fmt[3] << 8;
            rate = (uint32_t) (fmt[4] | fmt[5] << 8 | fmt[6] << 16 | (uint32_t) fmt[7] << 24);
            bits = fmt[14] | fmt[15] << 8;
            fseek(f, len - 16, SEEK_CUR);
        }
        else if (memcmp(chunk, "data", 4) == 0)
        {
            data_len = len;
            break;
        }
        else
        {
            fseek(f, len, SEEK_CUR);
        }
    }
    if (data_len < 0 || rate != DM_SAMPLE_RATE || bits != 16 || channels < 1)
    {
        DM_ERROR("selftest", "DATAMODEM_REPLAY: %s must be 16-bit 8000 Hz PCM (it is %d-bit, %u Hz, %d channels)",
                 path, bits, rate, channels);
        fclose(f);
        return DM_EXIT_CONFIG;
    }

    dm_modem_params_from_config(cfg, true, "replay", &params);
    m = dm_modem_create(&params);
    if (m == NULL)
    {
        fclose(f);
        return DM_EXIT_CONFIG;
    }
    dm_modem_arm(m);
    DM_INFO("selftest", "replaying %s: %.1f seconds of a far end, to a calling modem", path,
            (double) data_len / (2.0 * channels) / DM_SAMPLE_RATE);

    for (;;)
    {
        int16_t out[SELFTEST_CHUNK];
        int16_t in[SELFTEST_CHUNK];
        int n = 0;
        double t = (double) frames * SELFTEST_CHUNK / DM_SAMPLE_RATE;

        while (n < SELFTEST_CHUNK)
        {
            unsigned char b[2 * 8];

            if (fread(b, 2, (size_t) channels, f) != (size_t) channels)
                break;
            in[n++] = (int16_t) (b[0] | b[1] << 8);
        }
        if (n < SELFTEST_CHUNK)
            break;
        dm_clock_advance_ms(SELFTEST_CHUNK * 1000 / DM_SAMPLE_RATE);
        dm_modem_tx(m, out, SELFTEST_CHUNK);
        dm_modem_rx(m, in, SELFTEST_CHUNK);
        frames++;

        dm_modem_status(m, &st);
        if (strcmp(st.modulation, last_mod) != 0 || (int) st.phase != last_phase ||
            strcmp(st.train_stage ? st.train_stage : "", last_stage) != 0)
        {
            DM_INFO("selftest", "%6.2f s  %-7s %-11s %s", t, st.modulation, st.phase_text,
                    st.train_stage ? st.train_stage : "");
            snprintf(last_mod, sizeof(last_mod), "%s", st.modulation);
            snprintf(last_stage, sizeof(last_stage), "%s", st.train_stage ? st.train_stage : "");
            last_phase = (int) st.phase;
        }
        if (!ready && dm_modem_data_ready(m))
        {
            ready = true;
            DM_INFO("selftest", "%6.2f s  link up: %s at %d bps, %s", t, st.modulation, st.bit_rate, st.protocol);
        }
        if (got_n < sizeof(got))
            got_n += dm_modem_recv(m, got + got_n, sizeof(got) - got_n);
    }
    fclose(f);

    dm_modem_status(m, &st);
    if (got_n > 0)
    {
        char text[256];
        size_t k = 0;

        for (size_t i = 0; i < got_n && k < sizeof(text) - 1; i++)
            text[k++] = (got[i] >= 0x20 && got[i] < 0x7f) ? (char) got[i] : '.';
        text[k] = '\0';
        DM_INFO("selftest", "received %zu bytes: %s", got_n, text);
    }
    dm_log_event(DM_LOG_INFO, "selftest", "replay", "modulation=%s rate=%d protocol=%s link=%s seconds=%.1f",
                 st.modulation, st.bit_rate, st.protocol, ready ? "up" : "never",
                 (double) frames * SELFTEST_CHUNK / DM_SAMPLE_RATE);
    dm_modem_destroy(m);
    return ready ? DM_EXIT_OK : DM_EXIT_NO_CARRIER;
}

int dm_modem_selftest(const dm_config_t *cfg)
{
    if (getenv("DATAMODEM_REPLAY") != NULL && *getenv("DATAMODEM_REPLAY") != '\0')
        return selftest_replay(cfg, getenv("DATAMODEM_REPLAY"));

    dm_modem_params_t call_params;
    dm_modem_params_t ans_params;
    dm_modem_t *caller = NULL;
    dm_modem_t *answerer = NULL;
    dm_modem_status_t cs;
    dm_modem_status_t as;
    /* DATAMODEM_SELFTEST_BYTES, a test hook: a longer transfer, to measure
     * error rates too low for 512 bytes to show. */
    size_t nbytes = SELFTEST_BYTES;
    /* DATAMODEM_SELFTEST_ONEWAY, a test hook: the answerer sends nothing.
     * Traffic both ways lets acknowledgements ride on I-frames, which is how
     * a LAPM that threw away every RR it was sent passed this test for as
     * long as it did; one way, like a download, they cannot. */
    size_t nbytes_in;
    /* DATAMODEM_SELFTEST_STALL=n, a test hook: two seconds after the link
     * comes up, stop reading what arrives at the answerer for n seconds - a
     * slow terminal, or the AT prompt - so that the receive queue fills and
     * V.42 has to hold the far end off rather than lose data. */
    long stall_from = -1;
    long stall_until = -1;
    unsigned char *sent_out;
    unsigned char *sent_in;
    unsigned char *got_out;
    unsigned char *got_in;
    size_t out_queued = 0;
    size_t in_queued = 0;
    size_t got_out_len = 0;
    size_t got_in_len = 0;
    static selftest_line_t line;
    char line_desc[160];
    long iterations = 0;
    long max_iterations = (long) SELFTEST_MAX_SECONDS * DM_SAMPLE_RATE / SELFTEST_CHUNK;
    long connected_at = -1;
    size_t handshake_bytes = 0;
    long settle_until = 0;    /* ignore what arrives while a rate change drains */
    int settled_offer = 0;
    int rc = DM_EXIT_OK;
    int64_t started;

    if (getenv("DATAMODEM_SELFTEST_BYTES") != NULL)
    {
        long b = atol(getenv("DATAMODEM_SELFTEST_BYTES"));

        if (b < 16 || b > 16 * 1024 * 1024)
        {
            DM_ERROR("selftest", "DATAMODEM_SELFTEST_BYTES must be 16 to 16777216");
            return DM_EXIT_CONFIG;
        }
        nbytes = (size_t) b;
        /* Time for it at 300 bps, plus the handshake. */
        max_iterations += (long) (nbytes * 10 / 300) * DM_SAMPLE_RATE / SELFTEST_CHUNK;
    }
    nbytes_in = (getenv("DATAMODEM_SELFTEST_ONEWAY") != NULL) ? 0 : nbytes;
    sent_out = malloc(nbytes);
    sent_in = malloc(nbytes);
    got_out = malloc(nbytes * 2);
    got_in = malloc(nbytes * 2);
    if (sent_out == NULL || sent_in == NULL || got_out == NULL || got_in == NULL)
    {
        free(sent_out);
        free(sent_in);
        free(got_out);
        free(got_in);
        return DM_EXIT_CONFIG;
    }
    if (!selftest_line_init(&line, line_desc, sizeof(line_desc)))
    {
        free(sent_out);
        free(sent_in);
        free(got_out);
        free(got_in);
        return DM_EXIT_CONFIG;
    }
    dm_modem_params_from_config(cfg, true, "selftest-call", &call_params);
    dm_modem_params_from_config(cfg, false, "selftest-answer", &ans_params);
    /* DATAMODEM_SELFTEST_FAR, a test hook: "answer:v22bis" makes the
     * answering end a modem that does V.22 bis and nothing else - an older
     * one, that has never heard of stepping down - so that the other end's
     * step down gets exercised; "call:bell103" the calling end. ":auto" on
     * the end leaves it stepping down too, from there. */
    {
        const char *far = getenv("DATAMODEM_SELFTEST_FAR");
        static char far_mod[16];
        char role[16] = "";
        char mode[16] = "";

        if (far != NULL && *far != '\0')
        {
            dm_modem_params_t *fp;

            if (sscanf(far, "%15[^:]:%15[^:]:%15s", role, far_mod, mode) < 2 ||
                (strcmp(role, "answer") != 0 && strcmp(role, "call") != 0) ||
                (mode[0] != '\0' && strcmp(mode, "auto") != 0))
            {
                DM_ERROR("selftest", "DATAMODEM_SELFTEST_FAR wants answer:<modulation> or "
                                     "call:<modulation>, and :auto after it to let that end step down");
                selftest_line_free(&line);
                free(sent_out);
                free(sent_in);
                free(got_out);
                free(got_in);
                return DM_EXIT_CONFIG;
            }
            fp = strcmp(role, "answer") == 0 ? &ans_params : &call_params;
            fp->modulation = far_mod;
            fp->bit_rate = 0;
            fp->step_down = mode[0] != '\0';
            DM_INFO("selftest", "the %s end runs %s%s", role, far_mod,
                    fp->step_down ? " and steps down from there" : " only");
        }
    }
    /* DATAMODEM_SELFTEST_V42, a test hook: "answer:off" makes the answering
     * end one with no V.42 at all, "call:no-v42bis" a calling end that does
     * V.42 but will not compress, "answer:require" one that insists - so
     * that each of the other end's fallbacks gets exercised. */
    {
        const char *v = getenv("DATAMODEM_SELFTEST_V42");
        char role[16] = "";
        char what[16] = "";

        if (v != NULL && *v != '\0')
        {
            dm_modem_params_t *fp;

            if (sscanf(v, "%15[^:]:%15s", role, what) != 2 ||
                (strcmp(role, "answer") != 0 && strcmp(role, "call") != 0) ||
                (strcmp(what, "off") != 0 && strcmp(what, "no-v42bis") != 0 && strcmp(what, "require") != 0))
            {
                DM_ERROR("selftest", "DATAMODEM_SELFTEST_V42 wants answer: or call:, then off, no-v42bis "
                                     "or require");
                selftest_line_free(&line);
                free(sent_out);
                free(sent_in);
                free(got_out);
                free(got_in);
                return DM_EXIT_CONFIG;
            }
            fp = strcmp(role, "answer") == 0 ? &ans_params : &call_params;
            if (strcmp(what, "no-v42bis") == 0)
                fp->v42bis = false;
            else
                fp->v42 = what;
            DM_INFO("selftest", "the %s end runs V.42 %s", role, what);
        }
    }

    caller = dm_modem_create(&call_params);
    answerer = dm_modem_create(&ans_params);
    if (caller == NULL || answerer == NULL)
    {
        dm_modem_destroy(caller);
        dm_modem_destroy(answerer);
        selftest_line_free(&line);
        free(sent_out);
        free(sent_in);
        free(got_out);
        free(got_in);
        return DM_EXIT_CONFIG;
    }

    /* There is no call here to answer, so stand in for one: this harness is
     * the moment the far end picks up. */
    dm_modem_arm(caller);
    dm_modem_arm(answerer);

    fill_pattern(sent_out, nbytes, 0);
    fill_pattern(sent_in, nbytes, 31);

    if (nbytes_in == 0)
        DM_INFO("selftest", "looping two %s modems back to back, %zu bytes from the caller only",
                cfg->modulation, nbytes);
    else
        DM_INFO("selftest", "looping two %s modems back to back, %zu bytes each way", cfg->modulation,
                nbytes);
    if (line.delay > 0 || line.echo_gain > 0.0f || line.noise != NULL || line.burst != NULL || line.ulaw)
        DM_INFO("selftest", "the line between them: %s", line_desc);
    started = dm_now_real_ms();

    while (iterations < max_iterations)
    {
        iterations++;
        /* Keep the clock in step with the audio, so that anything driven by
         * a deadline - --v42-timeout and the establishment retries under it,
         * the carrier grace period, the idle timeout - behaves here the way
         * it will on a call. */
        dm_clock_advance_ms(SELFTEST_CHUNK * 1000 / DM_SAMPLE_RATE);
        selftest_line_run(&line, caller, answerer);

        /* Only start pushing data once both ends have a usable link - which
         * with V.42 means LAPM is up, not merely that a carrier exists. That
         * is what a real session waits for, and it keeps the comparison
         * honest. */
        /* The line rate can change mid-call: a link that settled below what
         * was offered and would not hold gets the ceiling dropped and
         * retrains - see apply_rate_ceiling(). Everything collected so far
         * belongs to a link that was not working, so the measurement starts
         * again once the pipes have drained. Without this the harness scores
         * the recovery against the garbage that preceded it and a working
         * fix looks like a failure. */
        if (connected_at >= 0 && settle_until == 0)
        {
            dm_modem_status(caller, &cs);
            if (settled_offer == 0)
                settled_offer = cs.offered_rate;
            else if (cs.offered_rate != settled_offer && cs.offered_rate > 0)
            {
                DM_INFO("selftest", "the rate on offer changed from %d to %d; starting the "
                                    "transfer again", settled_offer, cs.offered_rate);
                settled_offer = cs.offered_rate;
                settle_until = iterations + 2 * DM_SAMPLE_RATE / SELFTEST_CHUNK;
                out_queued = in_queued = 0;
                got_out_len = got_in_len = 0;
            }
        }
        if (settle_until > 0)
        {
            unsigned char drain[512];

            while (dm_modem_recv(caller, drain, sizeof(drain)) > 0)
                ;
            while (dm_modem_recv(answerer, drain, sizeof(drain)) > 0)
                ;
            if (iterations >= settle_until)
                settle_until = 0;
            continue;
        }

        /* Before both ends have a usable link, what arrives is handshake: an
         * end with no V.42 hears the other's ODP - DC1s, which is what V.42
         * chose so that async hosts would take them for XON - for as long
         * as detection lasts. A session waits for the link the same way. */
        if (connected_at < 0 && !(dm_modem_data_ready(caller) && dm_modem_data_ready(answerer)))
        {
            unsigned char drain[512];
            size_t n;

            while ((n = dm_modem_recv(caller, drain, sizeof(drain))) > 0)
                handshake_bytes += n;
            while ((n = dm_modem_recv(answerer, drain, sizeof(drain))) > 0)
                handshake_bytes += n;
        }

        if (dm_modem_data_ready(caller) && dm_modem_data_ready(answerer))
        {
            if (connected_at < 0)
            {
                dm_modem_status(caller, &cs);
                dm_modem_status(answerer, &as);
                connected_at = iterations;
                /* One end gave up on V.42 just now. The last of its ODP is
                 * still on the way to the other, which has been async all
                 * along and takes it for what it is meant to look like:
                 * XON. Let it land before counting. */
                if (cs.v42_fell_back || as.v42_fell_back)
                {
                    settle_until = iterations + DM_SAMPLE_RATE / 2 / SELFTEST_CHUNK;
                    DM_INFO("selftest", "link up after %.1f simulated seconds (%s, V.42 fell back); "
                                        "letting the last of the handshake land",
                            (double) iterations * SELFTEST_CHUNK / DM_SAMPLE_RATE, cs.protocol);
                    continue;
                }
                if (getenv("DATAMODEM_SELFTEST_STALL") != NULL)
                {
                    stall_from = iterations + 2L * DM_SAMPLE_RATE / SELFTEST_CHUNK;
                    stall_until = stall_from +
                                  atol(getenv("DATAMODEM_SELFTEST_STALL")) * DM_SAMPLE_RATE / SELFTEST_CHUNK;
                    max_iterations += stall_until - stall_from;
                }
                DM_INFO("selftest", "link up after %.1f simulated seconds (%s)",
                        (double) iterations * SELFTEST_CHUNK / DM_SAMPLE_RATE, cs.protocol);
                if (handshake_bytes > 0)
                    DM_INFO("selftest", "%zu bytes arrived before the link was up, and were not counted",
                            handshake_bytes);
            }
            out_queued += dm_modem_send(caller, sent_out + out_queued, nbytes - out_queued);
            in_queued += dm_modem_send(answerer, sent_in + in_queued, nbytes_in - in_queued);
        }

        if (iterations == stall_from)
            DM_INFO("selftest", "the answerer stops reading");
        if (iterations == stall_until)
            DM_INFO("selftest", "the answerer reads again");
        if (got_out_len < nbytes * 2 && (iterations < stall_from || iterations >= stall_until))
            got_out_len += dm_modem_recv(answerer, got_out + got_out_len, nbytes * 2 - got_out_len);
        if (got_in_len < nbytes * 2)
            got_in_len += dm_modem_recv(caller, got_in + got_in_len, nbytes * 2 - got_in_len);

        if (got_out_len >= nbytes && got_in_len >= nbytes_in)
            break;
    }

    dm_modem_status(caller, &cs);
    dm_modem_status(answerer, &as);

    if (connected_at < 0)
    {
        DM_ERROR("selftest", "no usable link within %d simulated seconds (caller %s, answerer %s)",
                 SELFTEST_MAX_SECONDS, cs.phase_text, as.phase_text);
        rc = DM_EXIT_NO_CARRIER;
    }
    else if (got_out_len < nbytes || got_in_len < nbytes_in)
    {
        DM_ERROR("selftest", "only %zu/%zu bytes arrived at the answerer and %zu/%zu at the caller",
                 got_out_len, nbytes, got_in_len, nbytes_in);
        rc = DM_EXIT_TIMEOUT;
    }
    else
    {
        bool ok = compare_stream("caller -> answerer", sent_out, nbytes, got_out, got_out_len);
        if (!compare_stream("answerer -> caller", sent_in, nbytes_in, got_in, got_in_len))
            ok = false;
        if (!ok)
            rc = DM_EXIT_NO_CARRIER;
    }

    dm_log_event(rc == DM_EXIT_OK ? DM_LOG_INFO : DM_LOG_ERROR, "selftest", "result",
                 "modulation=%s rate=%d protocol=%s link_seconds=%.1f simulated_seconds=%.1f "
                 "wall_ms=%" PRId64 " out_bytes=%zu in_bytes=%zu wire_bytes=%" PRIu64
                 " compression=%.2f frame_errors=%u exit=%d",
                 cs.modulation, cs.bit_rate, cs.protocol,
                 connected_at < 0 ? -1.0 : (double) connected_at * SELFTEST_CHUNK / DM_SAMPLE_RATE,
                 (double) iterations * SELFTEST_CHUNK / DM_SAMPLE_RATE, dm_now_real_ms() - started, got_out_len,
                 got_in_len, cs.wire_tx, cs.wire_tx ? (double) cs.bytes_tx / cs.wire_tx : 1.0,
                 cs.frame_errors + as.frame_errors, rc);

    dm_modem_destroy(caller);
    dm_modem_destroy(answerer);
    selftest_line_free(&line);
    free(sent_out);
    free(sent_in);
    free(got_out);
    free(got_in);
    return rc;
}
