/*
 * SpanDSP - a series of DSP components for telephony
 *
 * v42.c
 *
 * Written by Steve Underwood <steveu@coppice.org>
 *
 * Copyright (C) 2004, 2011 Steve Underwood
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License version 2.1,
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/* THIS IS A WORK IN PROGRESS. IT IS NOT FINISHED. */

/*! \file */

#if defined(HAVE_CONFIG_H)
#include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <inttypes.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <ctype.h>
#include <assert.h>

#include "spandsp/telephony.h"
#include "spandsp/logging.h"
#include "spandsp/bit_operations.h"
#include "spandsp/async.h"
#include "spandsp/hdlc.h"
#include "spandsp/v42.h"

#include "spandsp/private/logging.h"
#include "spandsp/private/hdlc.h"
#include "spandsp/private/v42.h"

#define FALSE 0
#define TRUE (!FALSE)

/* Detection phase timer */
#define T_400                           750
/* Acknowledgement timer - 1 second between SABME's */
#define T_401                           1000
/* Replay delay timer (optional) */
#define T_402                           1000
/* Inactivity timer (optional). No default - use 10 seconds with no packets */
#define T_403                           10000

#define LAPM_DLCI_DTE_TO_DTE            0
#define LAPM_DLCI_LAYER2_MANAGEMENT     63

#define elements(a) (sizeof(a)/sizeof((a)[0]))

/* LAPM definitions */

#define LAPM_FRAMETYPE_MASK             0x03

enum
{
    LAPM_FRAMETYPE_I = 0x00,
    LAPM_FRAMETYPE_I_ALT = 0x02,
    LAPM_FRAMETYPE_S = 0x01,
    LAPM_FRAMETYPE_U = 0x03
};

/* Supervisory headers */
enum
{
    LAPM_S_RR = 0x00,       /* cr */
    LAPM_S_RNR = 0x04,      /* cr */
    LAPM_S_REJ = 0x08,      /* cr */
    LAPM_S_SREJ = 0x0C      /* cr */
};

#define LAPM_S_PF                       0x01

/* Unnumbered headers */
enum
{
    LAPM_U_UI = 0x00,       /* cr */
    LAPM_U_DM = 0x0C,       /*  r */
    LAPM_U_DISC = 0x40,     /* c  */
    LAPM_U_UA = 0x60,       /*  r */
    LAPM_U_SABME = 0x6C,    /* c  */
    LAPM_U_FRMR = 0x84,     /*  r */
    LAPM_U_XID = 0xAC,      /* cr */
    LAPM_U_TEST = 0xE0      /* c  */
};

#define LAPM_U_PF                       0x10

/* XID sub-field definitions */
#define FI_GENERAL                      0x82
#define GI_PARAM_NEGOTIATION            0x80
#define GI_PRIVATE_NEGOTIATION          0xF0
#define GI_USER_DATA                    0xFF

/* Param negotiation (Table 11a/V.42) */
enum
{
    PI_HDLC_OPTIONAL_FUNCTIONS = 0x03,
    PI_TX_INFO_MAXSIZE = 0x05,
    PI_RX_INFO_MAXSIZE = 0x06,
    PI_TX_WINDOW_SIZE = 0x07,
    PI_RX_WINDOW_SIZE = 0x08
};

/* Private param negotiation (Table 11b/V.42) */
enum
{
    PI_PARAMETER_SET_ID = 0x00,
    PI_V42BIS_COMPRESSION_REQUEST = 0x01,
    PI_V42BIS_NUM_CODEWORDS = 0x02,
    PI_V42BIS_MAX_STRING_LENGTH = 0x03
};

#define LAPM_DLCI_DTE_TO_DTE            0
#define LAPM_DLCI_LAYER2_MANAGEMENT     63

/* Type definitions */
enum
{
    LAPM_DETECT = 0,
    LAPM_IDLE = 1,
    LAPM_ESTABLISH = 2,
    LAPM_DATA = 3,
    LAPM_RELEASE = 4,
    LAPM_SIGNAL = 5,
    LAPM_SETPARM = 6,
    LAPM_TEST = 7,
    LAPM_V42_UNSUPPORTED = 8
};

/* Prototypes */
static int lapm_connect(v42_state_t *ss);
static int lapm_disconnect(v42_state_t *s);
static void reset_lapm(v42_state_t *s);
static void lapm_hdlc_underflow(void *user_data);

static int lapm_config(v42_state_t *ss);
static void restart_lapm(v42_state_t *s);

SPAN_DECLARE(const char *) lapm_status_to_str(int status)
{
    switch (status)
    {
    case LAPM_DETECT:
        return "LAPM_DETECT";
    case LAPM_IDLE:
        return "LAPM_IDLE";
    case LAPM_ESTABLISH:
        return "LAPM_ESTABLISH";
    case LAPM_DATA:
        return "LAPM_DATA";
    case LAPM_RELEASE:
        return "LAPM_RELEASE";
    case LAPM_SIGNAL:
        return "LAPM_SIGNAL";
    case LAPM_SETPARM:
        return "LAPM_SETPARM";
    case LAPM_TEST:
        return "LAPM_TEST";
    case LAPM_V42_UNSUPPORTED:
        return "LAPM_V42_UNSUPPORTED";
    }
    /*endswitch*/
    return "???";
}
/*- End of function --------------------------------------------------------*/

static void report_rx_status_change(v42_state_t *s, int status)
{
    if (s->lapm.status_handler)
        s->lapm.status_handler(s->lapm.status_user_data, status);
    else if (s->lapm.iframe_put)
        s->lapm.iframe_put(s->lapm.iframe_put_user_data, NULL, status);
}
/*- End of function --------------------------------------------------------*/

static inline uint32_t pack_value(const uint8_t *buf, int len)
{
    uint32_t val;

    val = 0;
    while (len--)
    {
        val <<= 8;
        val |= *buf++;
    }
    return val;
}
/*- End of function --------------------------------------------------------*/

static inline v42_frame_t *get_next_free_ctrl_frame(lapm_state_t *s)
{
    v42_frame_t *f;
    int ctrl_put_next;

    if ((ctrl_put_next = s->ctrl_put + 1) >= V42_CTRL_FRAMES)
        ctrl_put_next = 0;
    if (ctrl_put_next == s->ctrl_get)
        return NULL;
    f = &s->ctrl_buf[s->ctrl_put];
    s->ctrl_put = ctrl_put_next;
    return f;
}
/*- End of function --------------------------------------------------------*/

static int tx_unnumbered_frame(lapm_state_t *s, uint8_t addr, uint8_t ctrl, uint8_t *info, int len)
{
    v42_frame_t *f;
    uint8_t *buf;

    if ((f = get_next_free_ctrl_frame(s)) == NULL)
        return -1;
    buf = f->buf;
    buf[0] = addr;
    buf[1] = LAPM_FRAMETYPE_U | ctrl;
    f->len = 2;
    if (info  &&  len)
    {
        memcpy(buf + f->len, info, len);
        f->len += len;
    }
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int tx_supervisory_frame(lapm_state_t *s, uint8_t addr, uint8_t ctrl, uint8_t pf_mask)
{
    v42_frame_t *f;
    uint8_t *buf;
    
    if ((f = get_next_free_ctrl_frame(s)) == NULL)
        return -1;
    buf = f->buf;
    buf[0] = addr;
    buf[1] = LAPM_FRAMETYPE_S | ctrl;
    buf[2] = (s->vr << 1) | pf_mask;
    f->len = 3;
    return 0;
}
/*- End of function --------------------------------------------------------*/

static __inline__ int set_param(int param, int value, int def)
{
    if ((value < def  &&  param >= def)  ||  (value >= def  &&  param < def))
        return def;
    if ((value < def  &&  param < value)  ||  (value >= def  &&  param > value))
        return value;
    return param;
}
/*- End of function --------------------------------------------------------*/

/* datamodem: the outcome of the XID exchange, per V.42 context. spandsp has
   nowhere in its own state to keep it - the v42_update_config() that would
   have applied it is commented out - and selftest runs two contexts in one
   process, so a single global will not do. A slot is taken by a context's
   first lookup and given back when it is freed - see dm_xid_release(). */
#define DM_XID_SLOTS 8
/* app_softmodem: per thread, as each Asterisk channel's V.42 context lives
   and dies on its channel's thread, and two channels must not race for a
   slot. */
static __thread struct
{
    const v42_state_t *s;
    int done;
    int t401_ms;
    int busy_discarded;     /* I-frames thrown away while we were busy (patch 10) */
    int vs_high;            /* the highest N(S) sent before a REJ went back (patch 11), -1 none */
} dm_xid[DM_XID_SLOTS];

/* datamodem: test hook. When set, this end behaves like the far ends that
   know nothing of XID: it never sends one, establishing straight away with
   SABME, and silently ignores any it receives - so that the other end's
   give-up-on-XID path can be exercised. */
int dm_v42_no_xid = 0;

static int dm_xid_slot(const v42_state_t *s)
{
    int free_slot = -1;

    for (int i = 0; i < DM_XID_SLOTS; i++)
    {
        if (dm_xid[i].s == s)
            return i;
        if (dm_xid[i].s == NULL && free_slot < 0)
            free_slot = i;
    }
    if (free_slot < 0)
        free_slot = 0;
    dm_xid[free_slot].s = s;
    dm_xid[free_slot].done = 0;
    dm_xid[free_slot].t401_ms = 0;
    dm_xid[free_slot].busy_discarded = 0;
    dm_xid[free_slot].vs_high = -1;
    return free_slot;
}

static int *dm_xid_flag(const v42_state_t *s)
{
    return &dm_xid[dm_xid_slot(s)].done;
}

/* datamodem: forget a context. Slots were once never given back, so after
   eight V.42 links in one process - answer --calls 0 gets there, and a
   program running the engine for many lines at once much sooner - every
   slot was taken for good, and from then on every link shared slot 0 with
   whichever other link had looked itself up last, each wiping the other's
   XID outcome, and V.42bis went unused. And a context allocated where a
   freed one had been took over its slot as it stood: patch 11's highest
   N(S) and patch 10's discarded-while-busy flag from the call before. */
static void dm_xid_release(const v42_state_t *s)
{
    for (int i = 0; i < DM_XID_SLOTS; i++)
    {
        if (dm_xid[i].s == s)
            dm_xid[i].s = NULL;
    }
}

/* datamodem: test hook - how many contexts hold a slot. */
int dm_v42_xid_slots_in_use(void)
{
    int n = 0;

    for (int i = 0; i < DM_XID_SLOTS; i++)
    {
        if (dm_xid[i].s != NULL)
            n++;
    }
    return n;
}

/* datamodem: T401 for this context. V.42 means it to exceed the time from
   sending a frame to hearing it acknowledged, and spandsp fixes it at the
   1 s that does on a phone line. Over RTP the round trip alone can be most of
   that - 772 ms has been seen - and an XID answered a little after 1 s was
   given up on and then thrown away, while the far end, having answered,
   went ahead and compressed. src/modem.c sets it from the measured round
   trip; it is never less than V.42's 1 s. */
void dm_v42_set_t401(const v42_state_t *s, int ms)
{
    dm_xid[dm_xid_slot(s)].t401_ms = ms;
}

static int dm_t401_ms(const v42_state_t *s)
{
    int ms = dm_xid[dm_xid_slot(s)].t401_ms;

    return (ms > T_401) ? ms : T_401;
}

/* datamodem: did an XID exchange complete, so that what is in s->config is
   what both ends agreed rather than only what we offered? */
int dm_v42_xid_done(const v42_state_t *s)
{
    return *dm_xid_flag(s);
}

void dm_v42_xid_forget(const v42_state_t *s)
{
    *dm_xid_flag(s) = 0;
}

static int receive_xid(v42_state_t *ss, const uint8_t *frame, int len)
{
    lapm_state_t *s;
    v42_config_parameters_t config;
    const uint8_t *buf;
    uint8_t group_id;
    /* datamodem: int, not uint16_t. The checks below for running past the
       end of a group are "group_len < 0", which an unsigned type can never
       satisfy, so a malformed parameter length walked off the frame. */
    int group_len;
    uint32_t param_val;
    uint8_t param_id;
    uint8_t param_len;
    /* datamodem: what the far end said about V.42bis. A frame with no
       private group, or no P0 in it, offers no compression - which is
       V.42bis's default, and what the agreement must then be. */
    int far_p0 = 0;
    int far_p1 = 512;
    int far_p2 = 6;

    s = &ss->lapm;
    if (len < 3 || frame[2] != FI_GENERAL)
        return -1;
    memset(&config, 0, sizeof(config));
    /* Skip the header octets */
    frame += 3;
    len -= 3;
    while (len > 0)
    {
        group_id = frame[0];
        group_len = frame[1];
        group_len = (group_len << 8) | frame[2];
        frame += 3;
        len -= (3 + group_len);
        if (len < 0)
            break;
        buf = frame;
        frame += group_len;
        switch (group_id)
        {
        case GI_PARAM_NEGOTIATION:
            while (group_len > 0)
            {
                param_id = buf[0];
                param_len = buf[1];
                buf += 2;
                group_len -= (2 + param_len);
                if (group_len < 0)
                    break;
                switch (param_id)
                {
                case PI_HDLC_OPTIONAL_FUNCTIONS:
                    param_val = pack_value(buf, param_len);
                    break;
                case PI_TX_INFO_MAXSIZE:
                    param_val = pack_value(buf, param_len);
                    param_val >>= 3;
                    config.v42_tx_n401 =
                    s->tx_n401 = set_param(s->tx_n401, param_val, ss->config.v42_tx_n401);
                    break;
                case PI_RX_INFO_MAXSIZE:
                    param_val = pack_value(buf, param_len);
                    param_val >>= 3;
                    config.v42_rx_n401 =
                    s->rx_n401 = set_param(s->rx_n401, param_val, ss->config.v42_rx_n401);
                    break;
                case PI_TX_WINDOW_SIZE:
                    param_val = pack_value(buf, param_len);
                    config.v42_tx_window_size_k =
                    s->tx_window_size_k = set_param(s->tx_window_size_k, param_val, ss->config.v42_tx_window_size_k);
                    break;
                case PI_RX_WINDOW_SIZE:
                    param_val = pack_value(buf, param_len);
                    config.v42_rx_window_size_k =
                    s->rx_window_size_k = set_param(s->rx_window_size_k, param_val, ss->config.v42_rx_window_size_k);
                    break;
                default:
                    break;
                }
                buf += param_len;
            }
            break;
        case GI_PRIVATE_NEGOTIATION:
            while (group_len > 0)
            {
                param_id = buf[0];
                param_len = buf[1];
                buf += 2;
                group_len -= (2 + param_len);
                if (group_len < 0)
                    break;
                switch (param_id)
                {
                case PI_PARAMETER_SET_ID:
                    /* This might be worth monitoring, but it doesn't serve mnuch other purpose */
                    break;
                case PI_V42BIS_COMPRESSION_REQUEST:
                    config.comp = pack_value(buf, param_len);
                    far_p0 = config.comp & 3;
                    break;
                case PI_V42BIS_NUM_CODEWORDS:
                    config.comp_dict_size = pack_value(buf, param_len);
                    far_p1 = config.comp_dict_size;
                    break;
                case PI_V42BIS_MAX_STRING_LENGTH:
                    config.comp_max_string = pack_value(buf, param_len);
                    far_p2 = config.comp_max_string;
                    break;
                default:
                    break;
                }
                buf += param_len;
            }
            break;
        default:
            break;
        }
    }
    //v42_update_config(ss, &config);

    /* datamodem: V.42bis 7.2 - the responder answers with the directions both
       ends want and the smaller of each parameter, and that is what both then
       use. Upstream discarded the far end's values, answered with its own,
       and compressed regardless - so a far end that negotiated "no
       compression", or a smaller dictionary, received data it could not
       decode. Doing it here suits both roles: as responder, s->config is
       what the XID response we are about to send carries; as initiator,
       the response is already the agreement and taking it again changes
       nothing. */
    span_log(&ss->logging, SPAN_LOG_FLOW, "XID from the far end: V.42bis P0=%d P1=%d P2=%d\n",
             far_p0, far_p1, far_p2);
    ss->config.comp &= far_p0;
    if (far_p1 < ss->config.comp_dict_size)
        ss->config.comp_dict_size = far_p1;
    if (far_p2 < ss->config.comp_max_string)
        ss->config.comp_max_string = far_p2;
    span_log(&ss->logging, SPAN_LOG_FLOW, "XID agreed: V.42bis P0=%d P1=%d P2=%d\n",
             ss->config.comp, ss->config.comp_dict_size, ss->config.comp_max_string);
    *dm_xid_flag(ss) = 1;
    return 0;
}
/*- End of function --------------------------------------------------------*/

static void transmit_xid(v42_state_t *ss, uint8_t addr, int pf)
{
    lapm_state_t *s;
    uint8_t *buf;
    int len;
    int group_len;
    uint32_t param_val;
    v42_frame_t *f;

    s = &ss->lapm;
    if ((f = get_next_free_ctrl_frame(s)) == NULL)
        return;

    buf = f->buf;
    len = 0;

    /* Figure 11/V.42 */
    *buf++ = addr;
    /* datamodem: P set on the command, so the far end knows an answer is
       wanted, and F on the response mirroring it - as for SABME and UA. */
    *buf++ = LAPM_U_XID | LAPM_FRAMETYPE_U | pf;
    /* Format identifier subfield */
    *buf++ = FI_GENERAL;
    len += 3;

    /* Parameter negotiation group */
    group_len = 20;
    *buf++ = GI_PARAM_NEGOTIATION;
    *buf++ = (group_len >> 8) & 0xFF;
    *buf++ = group_len & 0xFF;
    len += 3;

    /* For conformance with the encoding rules in ISO/IEC 8885, the transmitter of an XID command frame shall
       set bit positions 2, 4, 8, 9, 12 and 16 to 1. (Table 11a/V.42)
       Optional bits are:
         3 Selective retransmission procedure (SREJ frame) single I frame request
        14 Loop-back test procedure (TEST frame)
        17
        Extended FCS procedure (32-bit FCS)
        24 Selective retransmission procedure (SREJ frame) multiple I frame request with span list
           capability. */
    *buf++ = PI_HDLC_OPTIONAL_FUNCTIONS;
    *buf++ = 4;
    *buf++ = 0x8A;  /* Bits 2, 4, and 8 set */
    *buf++ = 0x89;  /* Bits 9, 12, and 16 set */
    *buf++ = 0x00;
    *buf++ = 0x00;

    /* Send the maximum as a number of bits, rather than octets */
    param_val = ss->config.v42_tx_n401 << 3;
    *buf++ = PI_TX_INFO_MAXSIZE;
    *buf++ = 2;
    *buf++ = (param_val >> 8) & 0xFF;
    *buf++ = (param_val & 0xFF);

    /* Send the maximum as a number of bits, rather than octets */
    param_val = ss->config.v42_rx_n401 << 3;
    *buf++ = PI_RX_INFO_MAXSIZE;
    *buf++ = 2;
    *buf++ = (param_val >> 8) & 0xFF;
    *buf++ = (param_val & 0xFF);
    
    *buf++ = PI_TX_WINDOW_SIZE;
    *buf++ = 1;
    *buf++ = ss->config.v42_tx_window_size_k;

    *buf++ = PI_RX_WINDOW_SIZE;
    *buf++ = 1;
    *buf++ = ss->config.v42_rx_window_size_k;

    len += group_len;

    if (ss->config.comp)
    {
        /* Private parameter negotiation group */
        group_len = 15;
        *buf++ = GI_PRIVATE_NEGOTIATION;
        *buf++ = (group_len >> 8) & 0xFF;
        *buf++ = group_len & 0xFF;
        len += 3;

        /* Private parameter for V.42 (ASCII for V42). V.42 says ".42", but V.42bis says "V42",
           and that seems to be what should be used. */
        *buf++ = PI_PARAMETER_SET_ID;
        *buf++ = 3;
        *buf++ = 'V';
        *buf++ = '4';
        *buf++ = '2';

        /* V.42bis P0 
           00 Compression in neither direction (default);
           01 Negotiation initiator-responder direction only;
           10 Negotiation responder-initiator direction only;
           11 Both directions. */
        *buf++ = PI_V42BIS_COMPRESSION_REQUEST;
        *buf++ = 1;
        *buf++ = ss->config.comp;

        /* V.42bis P1 */
        param_val = ss->config.comp_dict_size;
        *buf++ = PI_V42BIS_NUM_CODEWORDS;
        *buf++ = 2;
        *buf++ = (param_val >> 8) & 0xFF;
        *buf++ = param_val & 0xFF;

        /* V.42bis P2 */
        *buf++ = PI_V42BIS_MAX_STRING_LENGTH;
        *buf++ = 1;
        *buf++ = ss->config.comp_max_string;

        len += group_len;
    }

    f->len = len;
}
/*- End of function --------------------------------------------------------*/

/* datamodem: why the link went down, for whoever handles the status change.
   SIG_STATUS_LINK_DISCONNECTED arrives with no reason attached, and there are
   six quite different ways to get here - from "we ran out of SABME retries"
   to "the far end answered our SABME with a point-blank refusal". Telling
   them apart from the outside is impossible, and they call for different
   responses. */
__thread char dm_v42_disconnect_cause[80] = "";

/* datamodem: test hook. Refuse the first N SABMEs with a DM response, which
   is what real far ends in the field do and what no spandsp ever does - two
   spandsp ends accept each other's SABME on the first try, so the retry path
   that copes with a refusal is otherwise unreachable from the test suite.
   Set from DATAMODEM_V42_REFUSE_SABME; zero in any normal run. */
int dm_v42_refuse_sabme = 0;

/* datamodem: frames whose FCS did not check out. lapm_receive() drops these
   without a word, so a far end that is answering perfectly well over a path
   that is mangling its answers looks identical to one that is saying nothing.
   The difference matters more than anything else in a failed negotiation. */
__thread unsigned dm_v42_fcs_errors = 0;

/* datamodem: which Answerer Detection Pattern came back.
   Table 3/V.42 defines two, and they mean opposite things:

     (E) (C)     - V.42 supported
     (E) (Null)  - no error-correcting protocol desired

   and 7.2.1.2 has the originator "take the appropriate action based upon the
   ADP received (e.g. if EC is received, initiate LAPM)". Upstream matches
   both and then initiates LAPM either way, its own comment calling the
   result "the V.42 supported pattern". So a far end that politely says it
   does not want error correction gets a SABME anyway, and answers it with a
   DM - which is the correct response to a request it already declined.
   Detection appears to succeed, establishment is refused every single time,
   and nothing anywhere says why. */
__thread int dm_v42_adp_no_ec = 0;

/* datamodem: test hook - make the answering end send the "no error-correcting
   protocol desired" ADP. Two spandsp ends only ever send the other one.
   Set from DATAMODEM_V42_ANSWER_NO_EC. */
int dm_v42_answer_no_ec = 0;

/* datamodem: what a LAPM control field is, for the log. */
static const char *dm_lapm_frame_name(const uint8_t *frame, int len)
{
    if (len < 2)
        return "runt";
    switch (frame[1] & LAPM_FRAMETYPE_MASK)
    {
    case LAPM_FRAMETYPE_I:
    case LAPM_FRAMETYPE_I_ALT:
        return "I";
    case LAPM_FRAMETYPE_S:
        switch (frame[1] & 0x0C)
        {
        case LAPM_S_RR:   return "RR";
        case LAPM_S_RNR:  return "RNR";
        case LAPM_S_REJ:  return "REJ";
        case LAPM_S_SREJ: return "SREJ";
        default:          return "S?";
        }
    default:
        switch (frame[1] & 0xEC)
        {
        case LAPM_U_SABME: return "SABME";
        case LAPM_U_UA:    return "UA";
        case LAPM_U_DM:    return "DM";
        case LAPM_U_DISC:  return "DISC";
        case LAPM_U_UI:    return "UI";
        case LAPM_U_XID:   return "XID";
        case LAPM_U_TEST:  return "TEST";
        case LAPM_U_FRMR:  return "FRMR";
        default:           return "U?";
        }
    }
}
/*- End of function --------------------------------------------------------*/

/* datamodem: every LAPM frame, both directions, as it goes on and comes off
   the wire. When a negotiation fails against real equipment this is the only
   thing that settles what actually happened, and spandsp logs none of it. */
static void dm_frame_log(v42_state_t *ss, const char *dir, const uint8_t *frame, int len, int ok)
{
    char hex[3 * 32 + 1];
    char txt[32 + 1];
    int i;
    int shown = (len < 8) ? len : 8;

    hex[0] = '\0';
    for (i = 0; i < shown; i++)
        sprintf(hex + i * 3, "%02x ", frame[i]);
    span_log(&ss->logging, SPAN_LOG_FLOW, "LAPM %s %-5s %s addr=0x%02x ctrl=0x%02x P/F=%d len=%d [%s%s]\n",
             dir, dm_lapm_frame_name(frame, len),
             ok ? "      " : "BAD FCS",
             (len > 0) ? frame[0] : 0, (len > 1) ? frame[1] : 0,
             /* datamodem: where P/F is depends on the frame: bit 4 of
                the control octet in a U frame, bit 0 of the second in I and
                S frames, whose bit 4 is part of N(S) or the S frame type. */
             (len > 1) ? (((frame[1] & 0x03) == 0x03) ? ((frame[1] >> 4) & 1)
                                                      : ((len > 2) ? (frame[2] & 1) : 0)) : 0,
             len, hex, (len > shown) ? "..." : "");

    /* datamodem: and then all of it, 32 octets a line with the printable
       ones alongside, for any frame carrying more than its header. Eight
       octets was enough to name a frame and not enough to see what was in
       it - whether an I-frame is text or compressed, or what an XID
       offered. Debug logs only. */
    if (len <= 3)
        return;
    for (int off = 0; off < len; off += 32)
    {
        int n = (len - off < 32) ? len - off : 32;

        for (i = 0; i < n; i++)
        {
            uint8_t c = frame[off + i];

            sprintf(hex + i * 3, "%02x ", c);
            txt[i] = (c >= 0x20 && c < 0x7F) ? (char) c : '.';
        }
        txt[n] = '\0';
        span_log(&ss->logging, SPAN_LOG_FLOW, "LAPM %s       %4d: %-96s %s\n", dir, off, hex, txt);
    }
}
/*- End of function --------------------------------------------------------*/

#define dm_cause(str) \
    do { strncpy(dm_v42_disconnect_cause, (str), sizeof(dm_v42_disconnect_cause) - 1); \
         dm_v42_disconnect_cause[sizeof(dm_v42_disconnect_cause) - 1] = '\0'; } while (0)

static int ms_to_bits(v42_state_t *s, int time)
{
    return ((time*s->tx_bit_rate)/1000);
}
/*- End of function --------------------------------------------------------*/

/* datamodem: detection diagnostics.
   Counters rather than a message per event: the interesting part of a
   failed negotiation is the steady state, and per-event logging with a
   sane rate limit only ever shows the first few milliseconds, which is
   the demodulator still settling. The bit window is circular for the same
   reason - what gets dumped is the last of the stream, not the first. */
#define DM_BITWIN 256
/* app_softmodem: per thread, as for dm_xid above. idle_run steers detection
   itself, and two channels detecting at once would count each other's
   ones. */
static __thread struct
{
    unsigned first_bad, gap1_bad, second_bad, gap2_bad, cycles_ok;
    int first_common, second_common;
    unsigned gap1_min, gap1_max;
    char win[DM_BITWIN];
    unsigned nbits;
    unsigned idle_run;      /* consecutive ones seen while resynchronising */
} dm_neg;

static void dm_neg_reset(void)
{
    memset(&dm_neg, 0, sizeof(dm_neg));
    dm_neg.gap1_min = 9999;
}

static void dm_neg_bit(int bit)
{
    dm_neg.win[dm_neg.nbits % DM_BITWIN] = (char) ('0' + (bit & 1));
    dm_neg.nbits++;
}

static void dm_neg_summary(v42_state_t *s)
{
    char out[DM_BITWIN + 1];
    unsigned start;
    unsigned n = (dm_neg.nbits < DM_BITWIN) ? dm_neg.nbits : DM_BITWIN;

    span_log(&s->logging, SPAN_LOG_FLOW,
             "detect gave up: %u bits seen, %u good cycles; "
             "first char wrong %u times (last 0x%03X), gap1 wrong %u times (%u..%u), "
             "second char wrong %u times (last 0x%03X), gap2 wrong %u times\n",
             dm_neg.nbits, dm_neg.cycles_ok,
             dm_neg.first_bad, dm_neg.first_common,
             dm_neg.gap1_bad, dm_neg.gap1_min == 9999 ? 0 : dm_neg.gap1_min, dm_neg.gap1_max,
             dm_neg.second_bad, dm_neg.second_common, dm_neg.gap2_bad);

    start = (dm_neg.nbits <= DM_BITWIN) ? 0 : (dm_neg.nbits % DM_BITWIN);
    for (unsigned i = 0; i < n; i++)
        out[i] = dm_neg.win[(start + i) % DM_BITWIN];
    out[n] = '\0';
    for (unsigned i = 0; i < n; i += 64)
        span_log(&s->logging, SPAN_LOG_FLOW, "detect last bits %3u: %.64s\n", i, out + i);
    span_log(&s->logging, SPAN_LOG_FLOW,
             "detect wanted: E=0101000101 then 6-24 ones then C=0110000101 then 6-24 ones\n");
}

static void t400_expired(v42_state_t *ss)
{
    /* Give up trying to detect a V.42 capable peer. */
    dm_neg_summary(ss);     /* datamodem: say what was actually on the wire */
    ss->bit_timer = 0;
    ss->lapm.state = LAPM_V42_UNSUPPORTED;
    report_rx_status_change(ss, ss->lapm.state);
}
/*- End of function --------------------------------------------------------*/

static __inline__ void t400_start(v42_state_t *s)
{
    s->bit_timer = ms_to_bits(s, T_400);
    s->bit_timer_func = t400_expired;
}
/*- End of function --------------------------------------------------------*/

static __inline__ void t400_stop(v42_state_t *s)
{
    s->bit_timer = 0;
}
/*- End of function --------------------------------------------------------*/

static void t401_expired(v42_state_t *ss)
{
    lapm_state_t *s;

    span_log(&ss->logging, SPAN_LOG_FLOW, "T.401 expired\n");
    s = &ss->lapm;
    if (s->retry_count > V42_DEFAULT_N_400)
    {
        s->retry_count = 0;
        switch (s->state)
        {
        case LAPM_ESTABLISH:
        case LAPM_RELEASE:
            s->state = LAPM_IDLE;
            dm_cause("no answer to SABME after N400 retries");
            report_rx_status_change(ss, SIG_STATUS_LINK_DISCONNECTED);
            break;
        case LAPM_DATA:
            dm_cause("the far end stopped answering our polls");
            lapm_disconnect(ss);
            break;
        }
        return ;
    }
    s->retry_count++;
    if (s->configuring)
    {
        /* datamodem: a far end that ignores XID altogether - there are some
           - would otherwise be asked N400 times and never sent the SABME it
           is waiting for, and since the LAPM state is still LAPM_IDLE the
           retry-exhausted path above reports nothing either. Two XIDs,
           then establish without negotiating: V.42bis is then off, which is
           what V.42bis says no negotiation means - unless the answer turns
           up late, before the link is up, which rx_unnumbered_rsp_frame
           still accepts. So the offer in s->config is left alone. */
        if (s->retry_count > 1)
        {
            span_log(&ss->logging, SPAN_LOG_FLOW, "No answer to XID; establishing without it\n");
            s->configuring = FALSE;
            lapm_connect(ss);
            return;
        }
        transmit_xid(ss, s->cmd_addr, LAPM_U_PF);
    }
    else
    {
        switch (s->state)
        {
        case LAPM_ESTABLISH:
            tx_unnumbered_frame(s, s->cmd_addr, LAPM_U_SABME | LAPM_U_PF, NULL, 0);
            break;
        case LAPM_RELEASE:
            tx_unnumbered_frame(s, s->cmd_addr, LAPM_U_DISC | LAPM_U_PF, NULL, 0);
            break;
        case LAPM_DATA:
            tx_supervisory_frame(s, s->cmd_addr, (s->local_busy)  ?  LAPM_S_RNR  :  LAPM_S_RR, 1);
            break;
        }
    }
    ss->bit_timer = ms_to_bits(ss, dm_t401_ms(ss));
    ss->bit_timer_func = t401_expired;
}
/*- End of function --------------------------------------------------------*/

static __inline__ void t401_start(v42_state_t *s)
{
    s->bit_timer = ms_to_bits(s, dm_t401_ms(s));
    s->bit_timer_func = t401_expired;
    s->lapm.retry_count = 0;
}
/*- End of function --------------------------------------------------------*/

static __inline__ void t401_stop(v42_state_t *s)
{
    s->bit_timer = 0;
    s->lapm.retry_count = 0;
}
/*- End of function --------------------------------------------------------*/

static void t403_expired(v42_state_t *ss)
{
    lapm_state_t *s;

    span_log(&ss->logging, SPAN_LOG_FLOW, "T.403 expired\n");
    if (ss->lapm.state != LAPM_DATA)
        return;
    s = &ss->lapm;
    tx_supervisory_frame(s, s->cmd_addr, (ss->lapm.local_busy)  ?  LAPM_S_RNR  :  LAPM_S_RR, 1);
    t401_start(ss);
    ss->lapm.retry_count = 1;
}
/*- End of function --------------------------------------------------------*/

static __inline__ void t401_stop_t403_start(v42_state_t *s)
{
    s->bit_timer = ms_to_bits(s, T_403);
    s->bit_timer_func = t403_expired;
    s->lapm.retry_count = 0;
}
/*- End of function --------------------------------------------------------*/

static void initiate_negotiation_expired(v42_state_t *s)
{
    /* Timer service routine */
    /* datamodem: this used to call lapm_config(), which transmits an XID
       and nothing else. XID is parameter negotiation: V.42 8.10 has it
       initiated by the control function, not as a precondition for getting
       connected, and 8.3 is clear that an error-corrected connection is
       established by sending SABME. A far end that is waiting for SABME
       never sees one, so it never answers; we retransmit XID N400 times
       and stop. Worse, while configuring the LAPM state is still
       LAPM_IDLE, which matches no case in the retry-exhausted switch in
       t401_expired, so nothing is even reported - the state machine just
       goes quiet until the far end gives up and clears the call.

       Two spandsp ends do not notice, because spandsp answers its own XID.
       Against anything else it is fatal. lapm_connect() is what was wanted
       here - it sends the SABME, and the author had already commented the
       XID out of it. Nothing is lost by skipping the negotiation: spandsp
       never applies the negotiated values anyway, its config fields being
       compile-time constants the XID exchange does not update. */
    /* datamodem, later: but the XID was right to send - it was only wrong
       to send nothing else. V.42bis is negotiated in it and nowhere else,
       and without it compression is off by definition, however both ends
       were configured; real V.42 modems open with it and some refuse a bare
       SABME with DM until it has happened. So: XID, and SABME as soon as
       the response arrives (rx_unnumbered_rsp_frame does that), or after
       two unanswered XIDs (t401_expired does that). */
    span_log(&s->logging, SPAN_LOG_FLOW, "Start negotiation\n");
    dm_v42_xid_forget(s);
    /* Count damage from here: what came before was detection, not frames. */
    dm_v42_fcs_errors = 0;
    if (dm_v42_no_xid)
        lapm_connect(s);
    else
        lapm_config(s);
    lapm_hdlc_underflow(s);
}
/*- End of function --------------------------------------------------------*/

static int tx_information_frame(v42_state_t *ss)
{
    lapm_state_t *s;
    v42_frame_t *f;
    uint8_t *buf;
    int n;
    int info_put_next;

    s = &ss->lapm;
    if (s->far_busy  ||  ((s->vs - s->va) & 0x7F) >= s->tx_window_size_k)
        return FALSE;
    if (s->info_get != s->info_put)
        return TRUE;
    if ((info_put_next = s->info_put + 1) >= V42_INFO_FRAMES)
        info_put_next = 0;
    if (info_put_next == s->info_get  ||  info_put_next == s->info_acked)
        return FALSE;
    f = &s->info_buf[s->info_put];
    buf = f->buf;
    if (s->iframe_get == NULL)
        return FALSE;
    n = s->iframe_get(s->iframe_get_user_data, buf + 3, s->tx_n401);
    if (n < 0)
    {
        /* Error */
        report_rx_status_change(ss, SIG_STATUS_LINK_ERROR);
        return FALSE;
    }
    if (n == 0)
        return FALSE;

    f->len = n + 3;
    s->info_put = info_put_next;
    return TRUE;
}
/*- End of function --------------------------------------------------------*/

static void tx_information_rr_rnr_response(v42_state_t *ss, const uint8_t *frame, int len)
{
    lapm_state_t *s;

    s = &ss->lapm;
    /* Respond with information frame, RR, or RNR, as appropriate */
    /* p = 1 may be used for status checking */
    /* datamodem: with F equal to the command's P, as HDLC's poll/final rule has it.
       Upstream set F=1 on every one, and the far end's
       rx_supervisory_rsp_frame() - ours included - throws away a final
       response it did not poll for, acknowledgement and all. Both ends
       sending data hides it, the acknowledgements riding on I-frames
       instead; a one-way transfer, a download, got no acknowledgements at
       all, filled its window and sat there until a timer let it go round
       again. */
    if ((frame[2] & 0x1)  ||  !tx_information_frame(ss))
        tx_supervisory_frame(s, frame[0], (s->local_busy)  ?  LAPM_S_RNR  :  LAPM_S_RR, frame[2] & 0x1);
}
/*- End of function --------------------------------------------------------*/

/* datamodem (patch 11): going back for a REJ forgets how far we had got,
   and the far end may yet acknowledge frames from beyond the point we went
   back to - ones it had in fact received, the REJ having been for an earlier
   one, or sent before ours arrived. ack_info() took such an N(R) for a
   protocol error and disconnected. It only takes enough frames in flight to
   happen: a V.34 retrain at 33 600 bit/s did it every time. So remember the
   highest N(S) that went out, and count N(R) up to it as valid. */
static int *dm_vs_high(lapm_state_t *s)
{
    v42_state_t *ss = (v42_state_t *) ((char *) s - offsetof(v42_state_t, lapm));

    return &dm_xid[dm_xid_slot(ss)].vs_high;
}

static int reject_info(lapm_state_t *s)
{
    uint8_t n;
    int *high = dm_vs_high(s);

    /* Reject all non-acked frames */
    if (s->state != LAPM_DATA)
        return 0;
    n = (s->vs - s->va) & 0x7F;
    if (*high < 0 || ((s->vs - s->va) & 0x7F) > ((*high - s->va) & 0x7F))
        *high = s->vs;
    s->vs = s->va;
    s->info_get = s->info_acked;
    return n;
}
/*- End of function --------------------------------------------------------*/

static int ack_info(v42_state_t *ss, uint8_t nr)
{
    lapm_state_t *s;
    int n;

    s = &ss->lapm;
    {
        int *high = dm_vs_high(s);
        uint8_t top = s->vs;

        /* patch 11: what was sent before a REJ took us back counts too -
           while it is still ahead of V(A) within the window, and so not
           something sequence numbers have since wrapped past. */
        if (*high >= 0 && ((*high - s->va) & 0x7F) > s->tx_window_size_k)
            *high = -1;
        if (*high >= 0 && ((*high - s->va) & 0x7F) > ((s->vs - s->va) & 0x7F))
            top = (uint8_t) *high;
        /* Check that NR is valid - i.e.  VA <= NR <= VS  &&  VS-VA <= k */
        if (!((((nr - s->va) & 0x7F) + ((top - nr) & 0x7F)) <= s->tx_window_size_k
             &&
             ((top - s->va) & 0x7F) <= s->tx_window_size_k))
        {
            span_log(&ss->logging, SPAN_LOG_FLOW, "N(R) %d invalid: V(A) %d V(S) %d\n", nr, s->va, s->vs);
            dm_cause("the far end acknowledged a frame we never sent");
            lapm_disconnect(ss);
            return -1;
        }
        if (((top - nr) & 0x7F) == 0 || ((nr - s->va) & 0x7F) >= ((top - s->va) & 0x7F))
            *high = -1;
    }
    n = 0;
    while (s->va != nr  &&  s->info_acked != s->info_put)
    {
        /* Acknowledged frames we had gone back to send again need not go. */
        if (s->info_acked == s->info_get)
        {
            if (++s->info_get >= V42_INFO_FRAMES)
                s->info_get = 0;
            s->vs = (s->vs + 1) & 0x7F;
        }
        if (++s->info_acked >= V42_INFO_FRAMES)
            s->info_acked = 0;
        s->va = (s->va + 1) & 0x7F;
        n++;
    }
    if (n > 0  &&  s->retry_count == 0)
    {
        t401_stop_t403_start(ss);
        /* 8.4.8 */
        if (((s->vs - s->va) & 0x7F))
            t401_start(ss);
    }
    return n;
}
/*- End of function --------------------------------------------------------*/

static int valid_data_state(v42_state_t *ss)
{
    lapm_state_t *s;

    s = &ss->lapm;
    switch (s->state)
    {
    case LAPM_DETECT:
    case LAPM_IDLE:
        break;
    case LAPM_ESTABLISH:
        reset_lapm(ss);
        s->state = LAPM_DATA;
        report_rx_status_change(ss, SIG_STATUS_LINK_CONNECTED);
        return 1;
    case LAPM_DATA:
        return 1;
    case LAPM_RELEASE:
        reset_lapm(ss);
        s->state = LAPM_IDLE;
        dm_cause("our DISC was acknowledged");
        report_rx_status_change(ss, SIG_STATUS_LINK_DISCONNECTED);
        break;
    case LAPM_SIGNAL:
    case LAPM_SETPARM:
    case LAPM_TEST:
    case LAPM_V42_UNSUPPORTED:
        break;
    }
    return 0;
}
/*- End of function --------------------------------------------------------*/

static void receive_information_frame(v42_state_t *ss, const uint8_t *frame, int len)
{
    lapm_state_t *s;

    s = &ss->lapm;
    if (!valid_data_state(ss))
        return;
    if (len > s->rx_n401 + 3)
        return;
    /* Ack I frames: NR - 1 */
    ack_info(ss, frame[2] >> 1);
    if (s->local_busy)
    {
        /* 8.4.7 */
        if ((frame[2] & 0x1))
            tx_supervisory_frame(s, s->rsp_addr, LAPM_S_RNR, 1);
        /* datamodem: remembered, so that leaving the busy condition asks
           for it again - see v42_set_local_busy_status(). */
        dm_xid[dm_xid_slot(ss)].busy_discarded = 1;
        return;
    }
    /* NS sequence error */
    if ((frame[1] >> 1) != s->vr)
    {
        if (!s->rejected)
        {
            tx_supervisory_frame(s, s->rsp_addr, LAPM_S_REJ, (frame[2] & 0x1));
            s->rejected = TRUE;
        }
        return;
    }
    s->rejected = FALSE;

    s->iframe_put(s->iframe_put_user_data, frame + 3, len - 3);
    /* Increment vr */
    s->vr = (s->vr + 1) & 0x7F;
    tx_information_rr_rnr_response(ss, frame, len);
}
/*- End of function --------------------------------------------------------*/

static void rx_supervisory_cmd_frame(v42_state_t *ss, const uint8_t *frame, int len)
{
    lapm_state_t *s;

    s = &ss->lapm;
    /* If l->local_busy each RR,RNR,REJ with p=1 should be replied by RNR with f=1 (8.4.7) */
    switch (frame[1] & 0x0C)
    {
    case LAPM_S_RR:
        s->far_busy = FALSE;
        ack_info(ss, frame[2] >> 1);
        /* If p = 1 may be used for status checking? */
        tx_information_rr_rnr_response(ss, frame, len);
        break;
    case LAPM_S_RNR:
        s->far_busy = TRUE;
        ack_info(ss, frame[2] >> 1);
        /* If p = 1 may be used for status checking? */
        if ((frame[2] & 0x1))
            tx_supervisory_frame(s, s->rsp_addr, (s->local_busy)  ?  LAPM_S_RNR  :  LAPM_S_RR, 1);
        break;
    case LAPM_S_REJ:
        s->far_busy = FALSE;
        ack_info(ss, frame[2] >> 1);
        if (s->retry_count == 0)
        {
            t401_stop_t403_start(ss);
            reject_info(s);
        }
        tx_information_rr_rnr_response(ss, frame, len);
        break;
    case LAPM_S_SREJ:
        /* TODO: */
        return;
    default:
        return;
    }
}
/*- End of function --------------------------------------------------------*/

static void rx_supervisory_rsp_frame(v42_state_t *ss, const uint8_t *frame, int len)
{
    lapm_state_t *s;

    s = &ss->lapm;
    /* datamodem: a final response nobody polled for. Upstream dropped the
       frame whole, acknowledgement included - and upstream's own receiver
       sent nothing else (see tx_information_rr_rnr_response()), as may any
       far end built on it. The F bit means nothing here, but N(R) still
       does, so take it as a plain response. */
    int f = frame[2] & 0x1;

    if (s->retry_count == 0  &&  f)
        f = 0;
    /* Ack I frames <= NR - 1 */
    switch (frame[1] & 0x0C)
    {
    case LAPM_S_RR:
        s->far_busy = FALSE;
        ack_info(ss, frame[2] >> 1);
        if (s->retry_count  &&  f)
        {
            reject_info(s);
            t401_stop_t403_start(ss);
        }
        break;
    case LAPM_S_RNR:
        s->far_busy = TRUE;
        ack_info(ss, frame[2] >> 1);
        if (s->retry_count  &&  f)
        {
            reject_info(s);
            t401_stop_t403_start(ss);
        }
        if (s->retry_count == 0)
            t401_start(ss);
        break;
    case LAPM_S_REJ:
        s->far_busy = FALSE;
        ack_info(ss, frame[2] >> 1);
        if (s->retry_count == 0  ||  f)
        {
            reject_info(s);
            t401_stop_t403_start(ss);
        }
        break;
    case LAPM_S_SREJ:
        /* TODO: */
        return;
    default:
        return;
    }
}
/*- End of function --------------------------------------------------------*/

static int rx_unnumbered_cmd_frame(v42_state_t *ss, const uint8_t *frame, int len)
{
    lapm_state_t *s;

    s = &ss->lapm;
    switch (frame[1] & 0xEC)
    {
    case LAPM_U_SABME:
        if (dm_v42_refuse_sabme > 0)
        {
            /* V.42 8.3.2.1: "If the control function is unable to accept
               establishment ... respond to the SABME command with a DM
               response with the F bit set to the same binary value as the P
               bit in the received SABME command." */
            dm_v42_refuse_sabme--;
            span_log(&ss->logging, SPAN_LOG_FLOW, "Refusing SABME with DM (test hook, %d left)\n",
                     dm_v42_refuse_sabme);
            tx_unnumbered_frame(s, s->rsp_addr, LAPM_U_DM | (frame[1] & 0x10), NULL, 0);
            break;
        }
        /* Discard un-acked I frames. Reset vs, vr, and va. Clear exceptions */
        reset_lapm(ss);
        /* Going to connected state */
        s->state = LAPM_DATA;
        /* Respond UA (or DM on error) */
        // fixme: why may be error and LAPM_U_DM ??
        tx_unnumbered_frame(s, s->rsp_addr, LAPM_U_UA | (frame[1] & 0x10), NULL, 0);
        t401_stop_t403_start(ss);
        report_rx_status_change(ss, SIG_STATUS_LINK_CONNECTED);
        break;
    case LAPM_U_UI:
        /* Break signal */
        /* TODO: */
        break;
    case LAPM_U_DISC:
        /* Respond UA (or DM) */
        if (s->state == LAPM_IDLE)
        {
            tx_unnumbered_frame(s, s->rsp_addr, LAPM_U_DM | LAPM_U_PF, NULL, 0);
        }
        else
        {
            /* Going to disconnected state, discard unacked I frames, reset all. */
            s->state = LAPM_IDLE;
            reset_lapm(ss);
            tx_unnumbered_frame(s, s->rsp_addr, LAPM_U_UA | (frame[1] & 0x10), NULL, 0);
            t401_stop(ss);
            /* TODO: notify CF */
            dm_cause("the far end sent DISC");
            report_rx_status_change(ss, SIG_STATUS_LINK_DISCONNECTED);
        }
        break;
    case LAPM_U_XID:
        if (dm_v42_no_xid)
            break;
        /* Exchange general ID info */
        receive_xid(ss, frame, len);
        transmit_xid(ss, s->rsp_addr, frame[1] & LAPM_U_PF);
        break;
    case LAPM_U_TEST:
        /* TODO: */
        break;
    default:
        return -1;
    }
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int rx_unnumbered_rsp_frame(v42_state_t *ss, const uint8_t *frame, int len)
{
    lapm_state_t *s;

    s = &ss->lapm;
    switch (frame[1] & 0xEC)
    {
    case LAPM_U_DM:
        switch (s->state)
        {
        case LAPM_IDLE:
            if (!(frame[1] & 0x10))
            {
                /* TODO: notify CF */
                report_rx_status_change(ss, SIG_STATUS_LINK_CONNECTED);
            }
            break;
        case LAPM_ESTABLISH:
        case LAPM_RELEASE:
            if ((frame[1] & 0x10))
            {
                s->state = LAPM_IDLE;
                reset_lapm(ss);
                t401_stop(ss);
                /* TODO: notify CF */
                /* V.42 8.3.2.1: a DM response to our SABME means the far end
                   is not able to enter the connected state. It is entitled to
                   say so, and entitled to be able to later - so this is the
                   one cause worth retrying rather than giving up on. */
                dm_cause("the far end refused our SABME with DM");
                report_rx_status_change(ss, SIG_STATUS_LINK_DISCONNECTED);
            }
            break;
        case LAPM_DATA:
            if (s->retry_count  ||  !(frame[1] & 0x10))
            {
                s->state = LAPM_IDLE;
                reset_lapm(ss);
                /* TODO: notify CF */
                dm_cause("the far end sent DM on an established link");
                report_rx_status_change(ss, SIG_STATUS_LINK_DISCONNECTED);
            }
            break;
        default:
            break;
        }
        break;
    case LAPM_U_UI:
        /* TODO: */
        break;
    case LAPM_U_UA:
        switch (s->state)
        {
        case LAPM_ESTABLISH:
            s->state = LAPM_DATA;
            reset_lapm(ss);
            t401_stop_t403_start(ss);
            report_rx_status_change(ss, SIG_STATUS_LINK_CONNECTED);
            break;
        case LAPM_RELEASE:
            s->state = LAPM_IDLE;
            reset_lapm(ss);
            t401_stop(ss);
            dm_cause("our DISC was answered with UA");
            report_rx_status_change(ss, SIG_STATUS_LINK_DISCONNECTED);
            break;
        default:
            /* Unsolicited UA */
            /* TODO: */
            break;
        }
        /* Clear all exceptions, busy states (self and peer) */
        /* Reset vars */
        break;
    case LAPM_U_FRMR:
        /* Non-recoverable error */
        /* TODO: */
        break;
    case LAPM_U_XID:
        if (s->configuring)
        {
            receive_xid(ss, frame, len);
            s->configuring = FALSE;
            t401_stop(ss);
            switch (s->state)
            {
            case LAPM_IDLE:
                lapm_connect(ss);
                break;
            case LAPM_DATA:
                s->local_busy = FALSE;
                tx_supervisory_frame(s, s->cmd_addr, LAPM_S_RR, 0);
                break;
            }
        }
        else if (s->state == LAPM_ESTABLISH && !dm_v42_xid_done(ss))
        {
            /* datamodem: the answer to an XID we had stopped waiting for.
               The far end sent it before it could have seen our SABME, so
               it arrives ahead of the UA, and the far end now believes the
               parameters agreed - it will compress if V.42bis was. Take it,
               or it sends compressed data to an end that is not expecting
               any. */
            span_log(&ss->logging, SPAN_LOG_FLOW, "XID answered late; taking it\n");
            receive_xid(ss, frame, len);
        }
        break;
    default:
        break;
    }
    return 0;
}
/*- End of function --------------------------------------------------------*/

static void lapm_hdlc_underflow(void *user_data)
{
    lapm_state_t *s;
    v42_state_t *ss;
    v42_frame_t *f;

    ss = (v42_state_t *) user_data;
    s = &ss->lapm;
    if (s->ctrl_get != s->ctrl_put)
    {
        /* Send control frame */
        f = &s->ctrl_buf[s->ctrl_get];
        if (++s->ctrl_get >= V42_CTRL_FRAMES)
            s->ctrl_get = 0;
    }
    else
    {
        if (s->far_busy  ||  s->configuring  ||  s->state != LAPM_DATA)
        {
            hdlc_tx_flags(&s->hdlc_tx, 10);
            return;
        }
        if (s->info_get == s->info_put  &&  !tx_information_frame(ss))
        {
            hdlc_tx_flags(&s->hdlc_tx, 10);
            return;
        }
        /* Send info frame */
        f = &s->info_buf[s->info_get];
        if (++s->info_get >= V42_INFO_FRAMES)
            s->info_get = 0;

        f->buf[0] = s->cmd_addr;
        f->buf[1] = s->vs << 1;
        f->buf[2] = s->vr << 1;
        s->vs = (s->vs + 1) & 0x7F;
        /* datamodem: T401 runs from the first I-frame that has not been
           acknowledged (8.4.8) - whatever else the one timer was doing.
           Upstream started it only if no timer was running, but T403, the
           idle poll, is running whenever T401 is not, from the moment the
           link comes up. So T401 never ran at all, and a frame lost or an
           acknowledgement missed was recovered only when T403 expired, ten
           seconds later. */
        if (ss->bit_timer == 0  ||  ss->bit_timer_func != t401_expired)
            t401_start(ss);
    }
    dm_frame_log(ss, "tx", f->buf, f->len, TRUE);
    hdlc_tx_frame(&s->hdlc_tx, f->buf, f->len);
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE_NONSTD(void) lapm_receive(void *user_data, const uint8_t *frame, int len, int ok)
{
    lapm_state_t *s;
    v42_state_t *ss;

    ss = (v42_state_t *) user_data;
    s = &ss->lapm;
    if (len < 0)
    {
        span_log(&ss->logging, SPAN_LOG_DEBUG, "V.42 rx status is %s (%d)\n", signal_status_to_str(len), len);
        return;
    }
    /* datamodem: a LAPM command while we are still in detection. V.42 lets
       an originator skip detection and go straight to establishment, and a
       far end that has been through it already - with us, before a retrain
       took our half of it away - will not go through it again. Either way it
       is plainly doing V.42, and the only sensible reply to its SABME or XID
       is the one it would get after detection. Anything else that arrives
       now, a damaged frame especially, is the receiver framing detection
       patterns or line noise and says nothing. */
    if (s->state == LAPM_DETECT)
    {
        if (!ok  ||  len < 2  ||  frame[0] != s->rsp_addr  ||  (frame[1] & 0x03) != LAPM_FRAMETYPE_U)
            return;
        if ((frame[1] & 0xEC) != LAPM_U_SABME  &&  (frame[1] & 0xEC) != LAPM_U_XID)
            return;
        span_log(&ss->logging, SPAN_LOG_FLOW, "LAPM command during detection: the far end is past it\n");
        t400_stop(ss);
        ss->neg.rx_negotiation_step = 5;
        s->state = LAPM_IDLE;
        report_rx_status_change(ss, s->state);
        restart_lapm(ss);
    }
    if (!ok)
    {
        /* datamodem: say so. A path that is corrupting the far end's answers
           is a completely different problem from a far end that is not
           answering, and these used to vanish without trace. */
        dm_v42_fcs_errors++;
        if (len >= 2)
            dm_frame_log(ss, "rx", frame, len, FALSE);
        else
            span_log(&ss->logging, SPAN_LOG_FLOW, "LAPM rx bad FCS, %d octets\n", len);
        return;
    }
    /* datamodem: a frame has to have an address and a control field before
       either can be looked at. Upstream indexes frame[1] regardless. */
    if (len < 2)
    {
        span_log(&ss->logging, SPAN_LOG_FLOW, "LAPM rx runt frame, %d octets\n", len);
        return;
    }
    dm_frame_log(ss, "rx", frame, len, TRUE);

    switch ((frame[1] & LAPM_FRAMETYPE_MASK))
    {
    case LAPM_FRAMETYPE_I:
    case LAPM_FRAMETYPE_I_ALT:
        receive_information_frame(ss, frame, len);
        break;
    case LAPM_FRAMETYPE_S:
        if (!valid_data_state(ss))
            return;
        if (frame[0] == s->rsp_addr)
            rx_supervisory_cmd_frame(ss, frame, len);
        else
            rx_supervisory_rsp_frame(ss, frame, len);
        break;
    case LAPM_FRAMETYPE_U:
        if (frame[0] == s->rsp_addr)
            rx_unnumbered_cmd_frame(ss, frame, len);
        else
            rx_unnumbered_rsp_frame(ss, frame, len);
        break;
    default:
        break;
    }
}
/*- End of function --------------------------------------------------------*/

static int lapm_connect(v42_state_t *ss)
{
    lapm_state_t *s;

    s = &ss->lapm;
    if (s->state != LAPM_IDLE)
        return -1;

    /* Negotiate params */
    //transmit_xid(s, s->cmd_addr);

    reset_lapm(ss);
    /* Connect */
    s->state = LAPM_ESTABLISH;
    tx_unnumbered_frame(s, s->cmd_addr, LAPM_U_SABME | LAPM_U_PF, NULL, 0);
    /* Start T401 (and not T403) */
    t401_start(ss);
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int lapm_disconnect(v42_state_t *ss)
{
    lapm_state_t *s;

    s = &ss->lapm;
    s->state = LAPM_RELEASE;
    tx_unnumbered_frame(s, s->cmd_addr, LAPM_U_DISC | LAPM_U_PF, NULL, 0);
    t401_start(ss);
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int lapm_config(v42_state_t *ss)
{
    lapm_state_t *s;

    s = &ss->lapm;
    s->configuring = TRUE;
    if (s->state == LAPM_DATA)
    {
        s->local_busy = TRUE;
        tx_supervisory_frame(s, s->cmd_addr, LAPM_S_RNR, 1);
    }
    transmit_xid(ss, s->cmd_addr, LAPM_U_PF);
    t401_start(ss);
    return 0;
}
/*- End of function --------------------------------------------------------*/

static void reset_lapm(v42_state_t *ss)
{
    lapm_state_t *s;
    
    s = &ss->lapm;
    /* Reset the LAP.M state */
    s->local_busy = FALSE;
    s->far_busy = FALSE;
    s->vs = 0;
    s->va = 0;
    s->vr = 0;
    /* Discard any info frames still queued for transmission */
    s->info_put = 0;
    s->info_acked = 0;
    s->info_get = 0;
    /* Discard any control frames */
    s->ctrl_put = 0;
    s->ctrl_get = 0;

    s->tx_window_size_k = ss->config.v42_tx_window_size_k;
    s->rx_window_size_k = ss->config.v42_rx_window_size_k;
    s->tx_n401 = ss->config.v42_tx_n401;
    s->rx_n401 = ss->config.v42_rx_n401;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v42_stop(v42_state_t *ss)
{
    lapm_state_t *s;

    s = &ss->lapm;
    ss->bit_timer = 0;
    s->packer_process = NULL;
    lapm_disconnect(ss);
}
/*- End of function --------------------------------------------------------*/

static void restart_lapm(v42_state_t *s)
{
    if (s->calling_party)
    {
        s->bit_timer = 48*8;
        s->bit_timer_func = initiate_negotiation_expired;
    }
    else
    {
        lapm_hdlc_underflow(s);
    }
    s->lapm.packer_process = NULL;
    s->lapm.state = LAPM_IDLE;
}
/*- End of function --------------------------------------------------------*/

static void negotiation_rx_bit(v42_state_t *s, int new_bit)
{
    if (new_bit >= 0)
        dm_neg_bit(new_bit);
    /* DC1 with even parity, 8-16 ones, DC1 with odd parity, 8-16 ones */
    /* uint8_t odp = "0100010001 11111111 0100010011 11111111"; */
    /* V.42 OK E , 8-16 ones, C, 8-16 ones */
    /* uint8_t adp_v42 = "0101000101  11111111  0110000101  11111111"; */
    /* V.42 disabled E, 8-16 ones, NULL, 8-16 ones */
    /* uint8_t adp_nov42 = "0101000101  11111111  0000000001  11111111"; */

    /* There may be no negotiation, so we need to process this data through the
       HDLC receiver as well */
    if (new_bit < 0)
    {
        /* Special conditions */
        span_log(&s->logging, SPAN_LOG_DEBUG, "V.42 rx status is %s (%d)\n", signal_status_to_str(new_bit), new_bit);
        return;
    }
    /*endif*/
    new_bit &= 1;
    s->neg.rxstream = (s->neg.rxstream << 1) | new_bit;
    switch (s->neg.rx_negotiation_step)
    {
    case 0:
        /* Look for some ones */
        /* datamodem: the comment says "look for some ones", but the code
           accepted any zero at all - including one of the zeros inside a
           character's data. Having misaligned once it would keep
           misaligning, one failed attempt per character, and never
           recover. Require a real idle run first, so what we latch onto is
           a start bit at the beginning of a pattern. The second character
           is found by case 3 from a known offset, so it does not need an
           idle in front of it and is unaffected. */
        if (new_bit)
        {
            if (dm_neg.idle_run < 255)
                dm_neg.idle_run++;
            break;
        }
        /*endif*/
        if (dm_neg.idle_run < 4)
            break;
        /*endif*/
        dm_neg.idle_run = 0;
        s->neg.rx_negotiation_step = 1;
        s->neg.rxbits = 0;
        s->neg.rxstream = ~1;
        s->neg.rxoks = 0;
        break;
    case 1:
        /* Look for the first character */
        if (++s->neg.rxbits < 9)
            break;
        /*endif*/
        s->neg.rxstream &= 0x3FF;
        if (s->calling_party  &&  s->neg.rxstream == 0x145)
        {
            s->neg.rx_negotiation_step++;
        }
        else if (!s->calling_party  &&  s->neg.rxstream == 0x111)
        {
            s->neg.rx_negotiation_step++;
        }
        else
        {
            dm_neg.first_bad++;
            dm_neg.first_common = s->neg.rxstream;
            dm_neg.idle_run = 0;
            s->neg.rx_negotiation_step = 0;
        }
        /*endif*/
        s->neg.rxbits = 0;
        s->neg.rxstream = ~0;
        break;
    case 2:
        /* Look for 8 to 16 ones */
        /* datamodem: the increment used to happen before this test, so the
           terminating zero was counted as one of the ones and the accepted
           range was really 7 to 15. A far end using 16 - the top of what
           V.42 permits - was rejected, and detection failed against
           perfectly conformant equipment. Count only the ones, and allow a
           little either side of the specified range. */
        if (new_bit)
        {
            s->neg.rxbits++;
            break;
        }
        /*endif*/
        /* datamodem: V.42 says 8 to 16 ones here, and spandsp's own
           off-by-one made that 7 to 15. Observed equipment sends the two
           characters back to back and puts the whole idle after the pair,
           so the first gap is zero. Accept anything up to a generous
           ceiling: the pattern being matched is still E then C then idle,
           which is specific enough, and a wrong guess costs only a failed
           LAPM establishment and a fallback. */
        if (s->neg.rxbits <= 64)
            s->neg.rx_negotiation_step++;
        else
        {
            dm_neg.gap1_bad++;
            if ((unsigned) s->neg.rxbits < dm_neg.gap1_min) dm_neg.gap1_min = s->neg.rxbits;
            if ((unsigned) s->neg.rxbits > dm_neg.gap1_max) dm_neg.gap1_max = s->neg.rxbits;
            s->neg.rx_negotiation_step = 0;
        }
        /*endif*/
        s->neg.rxbits = 0;
        s->neg.rxstream = ~1;
        break;
    case 3:
        /* Look for the second character */
        if (++s->neg.rxbits < 9)
            break;
        /*endif*/
        s->neg.rxstream &= 0x3FF;
        if (s->calling_party  &&  s->neg.rxstream == 0x185)
        {
            /* (C): V.42 supported. */
            dm_v42_adp_no_ec = FALSE;
            s->neg.rx_negotiation_step++;
        }
        else if (s->calling_party  &&  s->neg.rxstream == 0x001)
        {
            /* (Null): no error-correcting protocol desired. */
            dm_v42_adp_no_ec = TRUE;
            s->neg.rx_negotiation_step++;
        }
        else if (!s->calling_party  &&  s->neg.rxstream == 0x113)
        {
            s->neg.rx_negotiation_step++;
        }
        else
        {
            dm_neg.second_bad++;
            dm_neg.second_common = s->neg.rxstream;
            s->neg.rx_negotiation_step = 0;
        }
        /*endif*/
        s->neg.rxbits = 0;
        s->neg.rxstream = ~0;
        break;
    case 4:
        /* Look for 8 to 16 ones */
        /* datamodem: same off-by-one as case 2 above. */
        if (new_bit)
        {
            s->neg.rxbits++;
            break;
        }
        /*endif*/
        if (s->neg.rxbits <= 64)
        {
            if (++s->neg.rxoks >= 2)
            {
                /* HIT - we have found one of the two ADPs. Which one it was
                   decides what happens next; see dm_v42_adp_no_ec. */
                s->neg.rx_negotiation_step++;
                if (s->calling_party)
                {
                    t400_stop(s);
                    if (dm_v42_adp_no_ec)
                    {
                        /* V.42 7.2.1.2 and Table 3: the answerer's detection
                           pattern asked for no error-correcting protocol.
                           Sending SABME at it only earns a DM. */
                        span_log(&s->logging, SPAN_LOG_FLOW,
                                 "ADP says no error-correcting protocol desired\n");
                        s->lapm.state = LAPM_V42_UNSUPPORTED;
                        report_rx_status_change(s, s->lapm.state);
                    }
                    else
                    {
                        span_log(&s->logging, SPAN_LOG_FLOW, "ADP says V.42 supported\n");
                        s->lapm.state = LAPM_IDLE;
                        report_rx_status_change(s, s->lapm.state);
                        restart_lapm(s);
                    }
                    /*endif*/
                }
                else
                {
                    s->neg.odp_seen = TRUE;
                }
                /*endif*/
                break;
            }
            /*endif*/
            dm_neg.cycles_ok++;
            s->neg.rx_negotiation_step = 1;
            s->neg.rxbits = 0;
            s->neg.rxstream = ~1;
        }
        else
        {
            dm_neg.gap2_bad++;
            s->neg.rx_negotiation_step = 0;
            s->neg.rxbits = 0;
            s->neg.rxstream = ~0;
        }
        /*endif*/
        break;
    case 5:
        /* Parked */
        break;
    }
    /*endswitch*/
}
/*- End of function --------------------------------------------------------*/

static int v42_support_negotiation_tx_bit(v42_state_t *s)
{
    int bit;

    if (s->calling_party)
    {
        if (s->neg.txbits <= 0)
        {
            s->neg.txstream = 0x3FE22;
            s->neg.txbits = 36;
        }
        else if (s->neg.txbits == 18)
        {
            s->neg.txstream = 0x3FF22;
        }
        /*endif*/
        bit = s->neg.txstream & 1;
        s->neg.txstream >>= 1;
        s->neg.txbits--;
    }
    else
    {
        if (s->neg.odp_seen  &&  s->neg.txadps < 10)
        {
            if (s->neg.txbits <= 0)
            {
                if (++s->neg.txadps >= 10)
                {
                    t400_stop(s);
                    s->lapm.state = LAPM_IDLE;
                    report_rx_status_change(s, s->lapm.state);
                    s->neg.txstream = 1;
                    restart_lapm(s);
                }
                else
                {
                    s->neg.txstream = 0x3FE8A;
                    s->neg.txbits = 36;
                }
                /*endif*/
            }
            else if (s->neg.txbits == 18)
            {
                /* (C) = V.42 supported, or (Null) = no error-correcting
                   protocol desired. Both carry 8 ones above the character. */
                s->neg.txstream = dm_v42_answer_no_ec ? 0x3FE00 : 0x3FE86;
            }
            /*endif*/
            bit = s->neg.txstream & 1;
            s->neg.txstream >>= 1;
            s->neg.txbits--;
        }
        else
        {
            bit = 1;
        }
        /*endif*/
    }
    /*endif*/
    return bit;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v42_rx_bit(void *user_data, int bit)
{
    v42_state_t *s;

    s = (v42_state_t *) user_data;
    if (s->lapm.state == LAPM_DETECT)
    {
        negotiation_rx_bit(s, bit);
        /* datamodem: and for frames from a far end already past detection;
           see lapm_receive(). */
        if (s->lapm.state == LAPM_DETECT)
            hdlc_rx_put_bit(&s->lapm.hdlc_rx, bit);
    }
    else
    {
        hdlc_rx_put_bit(&s->lapm.hdlc_rx, bit);
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

/* datamodem: send another SABME, without re-running detection.
   Establishment and detection are separate phases, and only the first of
   them failed. v42_restart() would go back to the detection phase, which a
   far end that has already completed detection will not run again - so the
   only way to be more patient than spandsp is to repeat this one step.
   Valid exactly when establishment has fallen back to LAPM_IDLE. */
int dm_v42_reconnect(v42_state_t *ss)
{
    if (ss->lapm.state != LAPM_IDLE)
        return -1;
    dm_v42_disconnect_cause[0] = '\0';
    /* If the parameters were never agreed, agreeing them may be exactly
       what the far end is waiting for. */
    if (!dm_v42_no_xid && !dm_v42_xid_done(ss))
        return lapm_config(ss);
    return lapm_connect(ss);
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v42_tx_bit(void *user_data)
{
    v42_state_t *s;
    int bit;

    s = (v42_state_t *) user_data;
    if (s->bit_timer  &&  (--s->bit_timer) <= 0)
    {
        s->bit_timer = 0;
        s->bit_timer_func(s);
    }
    if (s->lapm.state == LAPM_DETECT)
        bit = v42_support_negotiation_tx_bit(s);
    else
        bit = hdlc_tx_get_bit(&s->lapm.hdlc_tx);
    /*endif*/
    return bit;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v42_set_local_busy_status(v42_state_t *s, int busy)
{
    lapm_state_t *l = &s->lapm;
    int previous_busy;
    int slot = dm_xid_slot(s);

    busy = (busy)  ?  TRUE  :  FALSE;
    previous_busy = l->local_busy;
    l->local_busy = busy;
    /* datamodem: and tell the far end (8.4.7). Upstream only set the flag, so
       the far end went on sending, every I-frame was thrown away unanswered,
       and it found out only by polling after T401 - and then, when the
       condition cleared, nothing said so until its next poll, and the frames
       thrown away were recovered only by timing out again. Now an RNR goes
       out on entering the busy condition, and on leaving it an RR - or a REJ
       if anything was discarded meanwhile, so that it is sent again at
       once. */
    if (l->state == LAPM_DATA  &&  busy != previous_busy)
    {
        if (busy)
        {
            tx_supervisory_frame(l, l->rsp_addr, LAPM_S_RNR, 0);
        }
        else if (dm_xid[slot].busy_discarded)
        {
            tx_supervisory_frame(l, l->rsp_addr, LAPM_S_REJ, 0);
            l->rejected = TRUE;
        }
        else
        {
            tx_supervisory_frame(l, l->rsp_addr, LAPM_S_RR, 0);
        }
        dm_xid[slot].busy_discarded = 0;
    }
    return previous_busy;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v42_get_far_busy_status(v42_state_t *s)
{
    return s->lapm.far_busy;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v42_set_status_callback(v42_state_t *s, modem_status_func_t status_handler, void *user_data)
{
    s->lapm.status_handler = status_handler;
    s->lapm.status_user_data = user_data;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v42_restart(v42_state_t *s)
{
    dm_neg_reset();         /* datamodem: fresh diagnostics per call */
    hdlc_tx_init(&s->lapm.hdlc_tx, FALSE, 1, TRUE, lapm_hdlc_underflow, s);
    /* datamodem: report_bad_frames TRUE. With it FALSE, as upstream has it,
       the HDLC receiver drops a frame that fails its FCS before LAPM sees
       it - so the bad-FCS counting and logging in lapm_receive() could never
       fire, and a damaged answer was indistinguishable from no answer. */
    hdlc_rx_init(&s->lapm.hdlc_rx, FALSE, TRUE, 1, lapm_receive, s);

    if (s->detect)
    {
        /* We need to do the V.42 support detection sequence */
        s->neg.txstream = ~0;
        s->neg.txbits = 0;
        s->neg.rxstream = ~0;
        s->neg.rxbits = 0;
        s->neg.rxoks = 0;
        s->neg.txadps = 0;
        s->neg.rx_negotiation_step = 0;
        s->neg.odp_seen = FALSE;
        t400_start(s);
        s->lapm.state = LAPM_DETECT;
    }
    else
    {
        /* Go directly to LAP.M mode */
        s->lapm.state = LAPM_IDLE;
        restart_lapm(s);
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(v42_state_t *) v42_init(v42_state_t *ss,
                                     int calling_party,
                                     int detect,
                                     get_msg_func_t iframe_get,
                                     put_msg_func_t iframe_put,
                                     void *user_data)
{
    lapm_state_t *s;

    if (ss == NULL)
    {
        if ((ss = (v42_state_t *) malloc(sizeof(*ss))) == NULL)
            return NULL;
    }
    memset(ss, 0, sizeof(*ss));
    dm_xid_release(ss);

    s = &ss->lapm;
    ss->calling_party = calling_party;
    ss->detect = detect;
    s->iframe_get = iframe_get;
    s->iframe_get_user_data = user_data;
    s->iframe_put = iframe_put;
    s->iframe_put_user_data = user_data;

    s->state = (ss->detect)  ?  LAPM_DETECT  :  LAPM_IDLE;
    s->local_busy = FALSE;
    s->far_busy = FALSE;

    /* The address octet is:
        Data link connection identifier (0)
        Command/response (0 if answerer, 1 if originator)
        Extended address (1) */
    s->cmd_addr = (LAPM_DLCI_DTE_TO_DTE << 2) | ((ss->calling_party)  ?  0x02  :  0x00) | 0x01;
    s->rsp_addr = (LAPM_DLCI_DTE_TO_DTE << 2) | ((ss->calling_party)  ?  0x00  :  0x02) | 0x01;

    /* Set default values for the LAP.M parameters. These can be modified later. */
    ss->config.v42_tx_window_size_k = V42_DEFAULT_WINDOW_SIZE_K;
    ss->config.v42_rx_window_size_k = V42_DEFAULT_WINDOW_SIZE_K;
    ss->config.v42_tx_n401 = V42_DEFAULT_N_401;
    ss->config.v42_rx_n401 = V42_DEFAULT_N_401;

    /* TODO: This should be part of the V.42bis startup */
    ss->config.comp = 1;
    ss->config.comp_dict_size = 512;
    ss->config.comp_max_string = 6;

    ss->tx_bit_rate = 28800;

    reset_lapm(ss);

    span_log_init(&ss->logging, SPAN_LOG_NONE, NULL);
    span_log_set_protocol(&ss->logging, "V.42");
    return ss;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v42_release(v42_state_t *s)
{
    reset_lapm(s);
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v42_free(v42_state_t *s)
{
    v42_release(s);
    dm_xid_release(s);
    free(s);
    return 0;
}
/*- End of function --------------------------------------------------------*/
/*- End of file ------------------------------------------------------------*/
