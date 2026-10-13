/* INFO and MP sequences, and the V.8 messages ahead of them. See v34_info.h. */
#include "v34_info.h"

#include <string.h>

/* Figure 14: the register shifts towards bit 0, and the feedback - bit 0
 * plus the incoming bit - goes into bits 15, 10 and 3. That is the reflected
 * form of the CCITT polynomial, 0x8408, with no inversion at either end. */
uint16_t v34_crc_bits(const uint8_t *bits, int n, uint16_t crc)
{
    for (int i = 0; i < n; i++)
    {
        int fb = (crc ^ bits[i]) & 1;

        crc >>= 1;
        if (fb)
            crc ^= 0x8408;
    }
    return crc;
}

static int put(uint8_t *bits, int pos, unsigned v, int len)
{
    for (int i = 0; i < len; i++)
        bits[pos + i] = (uint8_t) ((v >> i) & 1);
    return pos + len;
}

static unsigned get(const uint8_t *bits, int pos, int len)
{
    unsigned v = 0;

    for (int i = 0; i < len; i++)
        v |= (unsigned) (bits[pos + i] & 1) << i;
    return v;
}

static int sget(const uint8_t *bits, int pos, int len)
{
    unsigned v = get(bits, pos, len);

    if (v & (1u << (len - 1)))
        return (int) v - (1 << len);
    return (int) v;
}

/* 1111, then 01110010 left-most first. */
static int put_info_head(uint8_t *bits)
{
    static const uint8_t head[12] = { 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 0 };

    memcpy(bits, head, sizeof(head));
    return 12;
}

static int put_crc(uint8_t *bits, int from, int to)
{
    return put(bits, to, v34_crc_bits(bits + from, to - from, 0xFFFF), 16);
}

static bool info_crc_ok(const uint8_t *bits, int from, int crc_at)
{
    return v34_crc_bits(bits + from, crc_at + 16 - from, 0xFFFF) == 0;
}

int v34_info0_pack(const v34_info0_t *in, uint8_t *bits)
{
    int p = put_info_head(bits);

    p = put(bits, p, in->sr2743, 1);
    p = put(bits, p, in->sr2800, 1);
    p = put(bits, p, in->sr3429, 1);
    p = put(bits, p, in->low3000, 1);
    p = put(bits, p, in->high3000, 1);
    p = put(bits, p, in->low3200, 1);
    p = put(bits, p, in->high3200, 1);
    p = put(bits, p, in->allow3429, 1);
    p = put(bits, p, in->power_reduction, 1);
    p = put(bits, p, (unsigned) in->asym_steps, 3);
    p = put(bits, p, in->cme, 1);
    p = put(bits, p, in->c1664, 1);
    p = put(bits, p, (unsigned) in->clock_source, 2);
    p = put(bits, p, in->ack, 1);
    p = put_crc(bits, 12, p);
    return put(bits, p, 0xF, 4);
}

bool v34_info0_unpack(const uint8_t *bits, int n, v34_info0_t *o)
{
    if (n < V34_INFO0_BITS - 4 || !info_crc_ok(bits, 12, 29))
        return false;
    memset(o, 0, sizeof(*o));
    o->sr2743 = bits[12];
    o->sr2800 = bits[13];
    o->sr3429 = bits[14];
    o->low3000 = bits[15];
    o->high3000 = bits[16];
    o->low3200 = bits[17];
    o->high3200 = bits[18];
    o->allow3429 = bits[19];
    o->power_reduction = bits[20];
    o->asym_steps = (int) get(bits, 21, 3);
    o->cme = bits[24];
    o->c1664 = bits[25];
    o->clock_source = (int) get(bits, 26, 2);
    o->ack = bits[28];
    return true;
}

int v34_info1c_pack(const v34_info1c_t *in, uint8_t *bits)
{
    int p = put_info_head(bits);

    p = put(bits, p, (unsigned) in->min_power_reduction, 3);
    p = put(bits, p, (unsigned) in->additional_power_reduction, 3);
    p = put(bits, p, (unsigned) in->md, 7);
    for (int sr = 0; sr < V34_NUM_SR; sr++)
    {
        p = put(bits, p, in->sr[sr].high, 1);
        p = put(bits, p, (unsigned) in->sr[sr].pre_emphasis, 4);
        p = put(bits, p, (unsigned) in->sr[sr].max_rate, 4);
    }
    p = put(bits, p, (unsigned) in->freq_offset & 0x3FFu, 10);
    p = put_crc(bits, 12, p);
    return put(bits, p, 0xF, 4);
}

bool v34_info1c_unpack(const uint8_t *bits, int n, v34_info1c_t *o)
{
    int p = 25;

    if (n < V34_INFO1C_BITS - 4 || !info_crc_ok(bits, 12, 89))
        return false;
    memset(o, 0, sizeof(*o));
    o->min_power_reduction = (int) get(bits, 12, 3);
    o->additional_power_reduction = (int) get(bits, 15, 3);
    o->md = (int) get(bits, 18, 7);
    for (int sr = 0; sr < V34_NUM_SR; sr++)
    {
        o->sr[sr].high = bits[p];
        o->sr[sr].pre_emphasis = (int) get(bits, p + 1, 4);
        o->sr[sr].max_rate = (int) get(bits, p + 5, 4);
        p += 9;
    }
    o->freq_offset = sget(bits, 79, 10);
    return true;
}

int v34_info1a_pack(const v34_info1a_t *in, uint8_t *bits)
{
    int p = put_info_head(bits);

    p = put(bits, p, (unsigned) in->min_power_reduction, 3);
    p = put(bits, p, (unsigned) in->additional_power_reduction, 3);
    p = put(bits, p, (unsigned) in->md, 7);
    p = put(bits, p, in->high, 1);
    p = put(bits, p, (unsigned) in->pre_emphasis, 4);
    p = put(bits, p, (unsigned) in->max_rate, 4);
    p = put(bits, p, (unsigned) in->sr_a_to_c, 3);
    p = put(bits, p, (unsigned) in->sr_c_to_a, 3);
    p = put(bits, p, (unsigned) in->freq_offset & 0x3FFu, 10);
    p = put_crc(bits, 12, p);
    return put(bits, p, 0xF, 4);
}

bool v34_info1a_unpack(const uint8_t *bits, int n, v34_info1a_t *o)
{
    if (n < V34_INFO1A_BITS - 4 || !info_crc_ok(bits, 12, 50))
        return false;
    memset(o, 0, sizeof(*o));
    o->min_power_reduction = (int) get(bits, 12, 3);
    o->additional_power_reduction = (int) get(bits, 15, 3);
    o->md = (int) get(bits, 18, 7);
    o->high = bits[25];
    o->pre_emphasis = (int) get(bits, 26, 4);
    o->max_rate = (int) get(bits, 30, 4);
    o->sr_a_to_c = (int) get(bits, 34, 3);
    o->sr_c_to_a = (int) get(bits, 37, 3);
    o->freq_offset = sget(bits, 40, 10);
    return true;
}

/* An MP sequence is a frame sync of seventeen ones and then 16-bit fields,
 * each after a start bit of zero: (Tables 20 and 21). The CRC covers the
 * fields and none of the start bits. */
int v34_mp_pack(const v34_mp_t *in, uint8_t *bits)
{
    int fields = (in->type == 1) ? 9 : 3;
    uint16_t crc = 0xFFFF;
    int p = 0;

    p = put(bits, p, 0x1FFFF, 17);
    /* 17:33 */
    p = put(bits, p, 0, 1);
    p = put(bits, p, (unsigned) in->type, 1);
    p = put(bits, p, 0, 1);
    p = put(bits, p, (unsigned) in->rate_c_to_a, 4);
    p = put(bits, p, (unsigned) in->rate_a_to_c, 4);
    p = put(bits, p, in->aux, 1);
    p = put(bits, p, (unsigned) in->trellis, 2);
    p = put(bits, p, in->nonlinear, 1);
    p = put(bits, p, in->expanded, 1);
    p = put(bits, p, in->ack, 1);
    /* 34:50 */
    p = put(bits, p, 0, 1);
    p = put(bits, p, in->rate_mask & 0x3FFFu, 15);
    p = put(bits, p, in->asymmetric, 1);
    /* 51:67, and in type 1 the six coefficients and a reserved field */
    for (int f = 2; f < fields; f++)
    {
        unsigned v = 0;

        if (in->type == 1 && f >= 2 && f < 8)
            v = (uint16_t) in->h[(f - 2) / 2][(f - 2) % 2];
        p = put(bits, p, 0, 1);
        p = put(bits, p, v, 16);
    }
    for (int f = 0; f < fields; f++)
        crc = v34_crc_bits(bits + 18 + 17 * f, 16, crc);
    p = put(bits, p, 0, 1);
    p = put(bits, p, crc, 16);
    return put(bits, p, 0, (in->type == 1) ? 1 : 3);
}

bool v34_mp_unpack(const uint8_t *bits, int n, v34_mp_t *o)
{
    int type;
    int fields;
    uint16_t crc = 0xFFFF;

    if (n < V34_MP0_BITS - 3)
        return false;
    for (int i = 0; i < 17; i++)
        if (!bits[i])
            return false;
    type = bits[18];
    fields = type ? 9 : 3;
    if (type && n < V34_MP1_BITS - 1)
        return false;
    /* Every start bit is a zero. */
    for (int f = 0; f <= fields; f++)
        if (bits[17 + 17 * f])
            return false;
    for (int f = 0; f < fields; f++)
        crc = v34_crc_bits(bits + 18 + 17 * f, 16, crc);
    if (crc != get(bits, 18 + 17 * fields, 16))
        return false;
    memset(o, 0, sizeof(*o));
    o->type = type;
    o->rate_c_to_a = (int) get(bits, 20, 4);
    o->rate_a_to_c = (int) get(bits, 24, 4);
    o->aux = bits[28];
    o->trellis = (int) get(bits, 29, 2);
    o->nonlinear = bits[31];
    o->expanded = bits[32];
    o->ack = bits[33];
    o->rate_mask = get(bits, 35, 15) & 0x3FFFu;
    o->asymmetric = bits[50];
    if (type)
        for (int k = 0; k < 6; k++)
            o->h[k / 2][k % 2] = (int16_t) get(bits, 52 + 17 * k, 16);
    return true;
}

/* ------------------------------------------------------------------ V.8 */

/* Tags in the low five bits of an octet; extension octets have 010 in
 * bits 3 to 5. */
#define V8_TAG_CALL 0x01
#define V8_TAG_MOD 0x05
#define V8_TAG_PCM 0x07
#define V8_TAG_PROT 0x0A
#define V8_TAG_ACCESS 0x0D
#define V8_EXT(o) (((o) & 0x38) == 0x10)

int v8_build(const v8_msg_t *m, uint8_t *oct, int max)
{
    int n = 0;

    if (max < 6)
        return 0;
    oct[n++] = (uint8_t) ((m->call_function << 5) | V8_TAG_CALL);
    oct[n++] = (uint8_t) (V8_TAG_MOD | (m->v34 ? 0x40 : 0) | (m->v34hdx ? 0x80 : 0));
    oct[n++] = (uint8_t) (0x10 | (m->v32 ? 0x01 : 0) | (m->v22 ? 0x02 : 0));
    oct[n++] = (uint8_t) (0x10 | (m->v21 ? 0x80 : 0));
    if (m->lapm)
        oct[n++] = (uint8_t) ((1 << 5) | V8_TAG_PROT);
    return n;
}

bool v8_parse(const uint8_t *oct, int n, v8_msg_t *m)
{
    int i = 0;
    bool call = false;
    bool mod = false;

    memset(m, 0, sizeof(*m));
    while (i < n)
    {
        uint8_t o = oct[i++];

        switch (o & 0x1F)
        {
        case V8_TAG_CALL:
            m->call_function = (o >> 5) & 7;
            call = true;
            break;
        case V8_TAG_MOD:
            m->v34 = (o & 0x40) != 0;
            m->v34hdx = (o & 0x80) != 0;
            if (i < n && V8_EXT(oct[i]))
            {
                m->v32 = (oct[i] & 0x01) != 0;
                m->v22 = (oct[i] & 0x02) != 0;
                i++;
                if (i < n && V8_EXT(oct[i]))
                {
                    m->v21 = (oct[i] & 0x80) != 0;
                    i++;
                }
            }
            mod = true;
            break;
        case V8_TAG_PROT:
            m->lapm = ((o >> 5) & 7) == 1;
            break;
        case V8_TAG_PCM:
            m->pcm = true;
            break;
        default:
            break;
        }
        /* Extensions of anything not understood are skipped. */
        while (i < n && V8_EXT(oct[i]))
            i++;
    }
    /* CM and JM both carry a call function and modulation modes (V.8
     * 7.3, 7.4). Anything less is noise that happened to frame - the far
     * end's FSK, read by a V.8 receiver it was never meant for. */
    return call && mod;
}
