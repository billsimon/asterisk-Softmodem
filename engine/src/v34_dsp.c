/* V.34 signal processing blocks. See v34_dsp.h. */
#include "v34_dsp.h"
#include "datamodem/log.h"

#include <math.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.14159265358979323846

static int gcd(int a, int b)
{
    while (b)
    {
        int t = a % b;

        a = b;
        b = t;
    }
    return a;
}

double v34_rrc(double t, double a)
{
    double x;

    if (fabs(t) < 1e-9)
        return 1.0 - a + 4.0 * a / PI;
    if (fabs(fabs(t) - 1.0 / (4.0 * a)) < 1e-9)
        return a / sqrt(2.0) * ((1.0 + 2.0 / PI) * sin(PI / (4.0 * a)) + (1.0 - 2.0 / PI) * cos(PI / (4.0 * a)));
    x = 4.0 * a * t;
    return (sin(PI * t * (1.0 - a)) + 4.0 * a * t * cos(PI * t * (1.0 + a))) / (PI * t * (1.0 - x * x));
}

/* Root raised cosine, t in symbols, Hann windowed to +/- span. */
static double pulse(double t, double a, double span)
{
    if (fabs(t) >= span)
        return 0.0;
    return v34_rrc(t, a) * 0.5 * (1.0 + cos(PI * t / span));
}

/* --------------------------------------------------------- QAM modulator */

/* 5.4: the template runs flat to 0.45 S either side of the carrier, which is
 * a roll-off of 10%. */
#define QAM_ALPHA 0.10

double v34_pre_emphasis_db(int idx, int sr, double f)
{
    static const double alpha[6] = { 0, 2, 4, 6, 8, 10 };
    static const double beta[5] = { 0.5, 1.0, 1.5, 2.0, 2.5 };
    static const double gamma[5] = { 1, 2, 3, 4, 5 };
    double nu = f / v34_symbol_rate(sr);

    if (idx < 0 || idx > 10)
        return 0.0;
    if (idx <= 5)
        return alpha[idx] * nu;
    /* Figure 2: flat to 0.8 S, then a step of beta rising by gamma more by
     * 1.2 S. The template leaves a gap just below 0.8; the step is spread
     * over it. */
    idx -= 6;
    if (nu <= 0.72)
        return 0.0;
    if (nu < 0.8)
        return beta[idx] * (nu - 0.72) / 0.08;
    if (nu > 1.2)
        nu = 1.2;
    return beta[idx] + gamma[idx] * (nu - 0.8) / 0.4;
}

/* A linear-phase FIR with the magnitude of filter idx, normalised to unity
 * power gain across the band the signal occupies. */
static void design_pe(v34_qtx_t *t, int idx)
{
    const int c = V34_PE_TAPS / 2;
    const int K = 256;
    double h[V34_PE_TAPS] = { 0 };
    double S = v34_symbol_rate(t->sr);
    double fc = v34_carrier(t->sr, t->high);
    double pw = 0.0;
    int np = 0;

    for (int k = 0; k < K; k++)
    {
        double f = (k + 0.5) * 4000.0 / K;
        double ff = (f < fc - 0.6 * S) ? fc - 0.6 * S : (f > fc + 0.6 * S) ? fc + 0.6 * S : f;
        double a = pow(10.0, v34_pre_emphasis_db(idx, t->sr, ff) / 20.0);

        for (int n = 0; n < V34_PE_TAPS; n++)
            h[n] += 2.0 / K * a * cos(2.0 * PI * f * (n - c) / V34_FS) * 0.5 * (1.0 + cos(PI * (n - c) / (c + 1)));
    }
    for (double f = fc - 0.5 * S; f <= fc + 0.5 * S; f += 25.0)
    {
        double re = 0.0, im = 0.0;

        for (int n = 0; n < V34_PE_TAPS; n++)
        {
            re += h[n] * cos(2.0 * PI * f * n / V34_FS);
            im -= h[n] * sin(2.0 * PI * f * n / V34_FS);
        }
        pw += re * re + im * im;
        np++;
    }
    pw = sqrt(pw / np);
    for (int n = 0; n < V34_PE_TAPS; n++)
        t->pe[n] = (float) (h[n] / pw);
}

bool v34_qtx_init(v34_qtx_t *t, int sr, bool high, int pre_emphasis, double power_dbm0, v34_symbol_fn next,
                  void *user)
{
    int a = V34_SR_A[sr], c = V34_SR_C[sr];
    int d = V34_CAR_D[sr][high], e = V34_CAR_E[sr][high];
    int gd;
    double sum = 0.0, k;

    v34_qtx_free(t);
    memset(t, 0, sizeof(*t));
    t->sr = sr;
    t->high = high;
    t->num = 3 * a;
    t->den = 10 * c;
    gd = gcd(t->num, t->den);
    t->num /= gd;
    t->den /= gd;
    t->cnum = 3 * a * d;
    t->cden = 10 * c * e;
    gd = gcd(t->cnum, t->cden);
    t->cnum /= gd;
    t->cden /= gd;
    t->g = calloc((size_t) t->den, sizeof(*t->g));
    t->cosv = calloc((size_t) t->cden, sizeof(float));
    t->sinv = calloc((size_t) t->cden, sizeof(float));
    if (t->g == NULL || t->cosv == NULL || t->sinv == NULL)
    {
        v34_qtx_free(t);
        return false;
    }
    for (int mu = 0; mu < t->den; mu++)
        for (int m = 0; m < V34_QTX_TAPS; m++)
        {
            double g = pulse((double) mu / t->den + m - V34_QTX_L, QAM_ALPHA, V34_QTX_L);

            t->g[mu][m] = (float) g;
            sum += g * g;
        }
    /* Independent symbols of unit power come out at unit baseband power. */
    k = sqrt(t->den / sum);
    for (int mu = 0; mu < t->den; mu++)
        for (int m = 0; m < V34_QTX_TAPS; m++)
            t->g[mu][m] = (float) (t->g[mu][m] * k);
    for (int i = 0; i < t->cden; i++)
    {
        t->cosv[i] = (float) cos(2.0 * PI * i / t->cden);
        t->sinv[i] = (float) sin(2.0 * PI * i / t->cden);
    }
    t->gain = (float) (sqrt(2.0) * V34_DBM0_RMS * pow(10.0, power_dbm0 / 20.0));
    t->pe_on = pre_emphasis > 0 && pre_emphasis <= 10;
    if (t->pe_on)
        design_pe(t, pre_emphasis);
    t->next = next;
    t->user = user;
    return true;
}

void v34_qtx_free(v34_qtx_t *t)
{
    free(t->g);
    free(t->cosv);
    free(t->sinv);
    t->g = NULL;
    t->cosv = t->sinv = NULL;
}

float v34_qtx_sample(v34_qtx_t *t)
{
    long long kk = (t->n * t->num) / t->den;
    int mu = (int) ((t->n * t->num) % t->den);
    int ph = (int) ((t->n * t->cnum) % t->cden);
    v34_cf_t b = 0.0f;
    float x;

    while (t->k <= kk)
    {
        t->hist[t->k & 31] = t->next(t->user);
        t->k++;
    }
    for (int m = 0; m < V34_QTX_TAPS; m++)
        b += t->hist[(kk - m) & 31] * t->g[mu][m];
    x = (crealf(b) * t->cosv[ph] - cimagf(b) * t->sinv[ph]) * t->gain;
    t->n++;
    if (t->pe_on)
    {
        float y = 0.0f;

        t->pe_hist[t->pe_pos] = x;
        for (int i = 0; i < V34_PE_TAPS; i++)
            y += t->pe[i] * t->pe_hist[(t->pe_pos - i + V34_PE_TAPS) % V34_PE_TAPS];
        t->pe_pos = (t->pe_pos + 1) % V34_PE_TAPS;
        x = y;
    }
    return x;
}

/* ------------------------------------------------------- Phase 2 signals */

#define DPSK_ALPHA 0.75
#define DPSK_L 4
#define DPSK_TAPS (2 * DPSK_L + 1)

const int V34_PROBE_HZ[V34_PROBE_TONES] = { 150,  300,  450,  600,  750,  1050, 1350, 1500, 1650, 1950, 2100,
                                            2250, 2550, 2700, 2850, 3000, 3150, 3300, 3450, 3600, 3750 };
/* Table 17: which of them start at 180 degrees. */
static const bool PROBE_INVERT[V34_PROBE_TONES] = { 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                                     1, 0, 1, 0, 1, 1, 1, 1, 0, 0 };
static float probe_table[V34_P2_BOX];
/* app_softmodem: built once for all threads, as v34_codec.c's tables are.
   Two channels starting V.34 at once both built it, and on a weakly ordered
   CPU one could see the flag before the table. */
static pthread_once_t probe_once = PTHREAD_ONCE_INIT;

static void build_probe_once(void)
{
    for (int n = 0; n < V34_P2_BOX; n++)
    {
        double s = 0.0;

        for (int k = 0; k < V34_PROBE_TONES; k++)
            s += cos(2.0 * PI * V34_PROBE_HZ[k] * n / V34_FS + (PROBE_INVERT[k] ? PI : 0.0));
        probe_table[n] = (float) s;
    }
}

static void build_probe(void)
{
    pthread_once(&probe_once, build_probe_once);
}

void v34_p2tx_init(v34_p2tx_t *t, bool answerer, double power_dbm0)
{
    double rms = V34_DBM0_RMS * pow(10.0, power_dbm0 / 20.0);
    double sum = 0.0, k;

    memset(t, 0, sizeof(*t));
    build_probe();
    t->answerer = answerer;
    t->amp = (float) (sqrt(2.0) * rms * (answerer ? pow(10.0, -1.0 / 20.0) : 1.0));
    t->guard_amp = answerer ? (float) (sqrt(2.0) * rms * pow(10.0, -7.0 / 20.0)) : 0.0f;
    t->cden = answerer ? 10 : 20;
    for (int i = 0; i < 40; i++)
        t->cosv[i] = (float) cos(2.0 * PI * i / t->cden);
    for (int mu = 0; mu < 40; mu++)
        for (int m = 0; m < DPSK_TAPS; m++)
        {
            double g = pulse(mu / 40.0 + m - DPSK_L, DPSK_ALPHA, DPSK_L);

            t->g[mu][m] = (float) g;
            sum += g * g;
        }
    k = sqrt(40.0 / sum);
    for (int mu = 0; mu < 40; mu++)
        for (int m = 0; m < DPSK_TAPS; m++)
            t->g[mu][m] = (float) (t->g[mu][m] * k);
    t->phase = 1.0f;
    t->sign = 1.0f;
    t->flip_at = -1;
}

void v34_p2tx_bits(v34_p2tx_t *t, const uint8_t *bits, int n)
{
    for (int i = 0; i < n; i++)
    {
        int nt = (t->qt + 1) % V34_P2_QUEUE;

        if (nt == t->qh)
            break;
        t->q[t->qt] = bits[i];
        t->qt = nt;
    }
}

int v34_p2tx_pending(const v34_p2tx_t *t)
{
    return (t->qt - t->qh + V34_P2_QUEUE) % V34_P2_QUEUE;
}

void v34_p2tx_carrier(v34_p2tx_t *t, bool on)
{
    if (on && !t->on)
    {
        /* Starting from silence: the filter's memory is of nothing, and the
         * first symbol out is the point at an arbitrary phase that comes
         * before an INFO sequence (10.1.2.3.1). */
        memset(t->hist, 0, sizeof(t->hist));
    }
    t->on = on;
    if (!on)
    {
        t->qh = t->qt = 0;
        t->flip_at = -1;
    }
    t->probe = false;
}

void v34_p2tx_reverse_at(v34_p2tx_t *t, long long at)
{
    t->flip_at = at;
}

void v34_p2tx_probe(v34_p2tx_t *t, bool on, bool l1, double power_dbm0)
{
    double rms = V34_DBM0_RMS * pow(10.0, power_dbm0 / 20.0);

    if (on && !t->probe)
        t->probe_n = 0;
    t->probe = on;
    if (on)
        t->on = false;
    /* Each of the 21 tones carries a 21st of the power; L1 is 6 dB up. */
    t->probe_amp = (float) (rms * sqrt(2.0 / V34_PROBE_TONES) * (l1 ? 2.0 : 1.0));
}

float v34_p2tx_sample(v34_p2tx_t *t, long long line_time)
{
    long long kk;
    int mu;
    float b = 0.0f;
    float x;

    if (t->probe)
        return probe_table[t->probe_n++ % V34_P2_BOX] * t->probe_amp;
    if (!t->on)
    {
        t->n++;
        return 0.0f;
    }
    if (t->flip_at >= 0 && line_time >= t->flip_at)
    {
        t->sign = -t->sign;
        t->flip_at = -1;
    }
    kk = (t->n * 3) / 40;
    mu = (int) ((t->n * 3) % 40);
    while (t->k <= kk)
    {
        if (t->qh != t->qt)
        {
            if (t->q[t->qh])
                t->phase = -t->phase;
            t->qh = (t->qh + 1) % V34_P2_QUEUE;
        }
        t->hist[t->k & 15] = t->phase;
        t->k++;
    }
    for (int m = 0; m < DPSK_TAPS; m++)
        b += t->hist[(kk - m) & 15] * t->g[mu][m];
    x = b * t->cosv[(t->n * 3) % t->cden] * t->amp * t->sign;
    if (t->answerer)
        x += t->guard_amp * cosf((float) (2.0 * PI * 1800.0 * (double) (t->n % 40) / V34_FS));
    t->n++;
    return x;
}

/* The receiving half. */

void v34_p2rx_init(v34_p2rx_t *r, bool far_is_answerer)
{
    double sum = 0.0;

    memset(r, 0, sizeof(*r));
    r->cden = far_is_answerer ? 10 : 20;
    for (int i = 0; i < 40; i++)
    {
        r->cosv[i] = (float) cos(2.0 * PI * i / r->cden);
        r->sinv[i] = (float) sin(2.0 * PI * i / r->cden);
    }
    /* Matched to the transmitter's pulse, with unit gain at DC so that a
     * tone of amplitude A comes out at A/2. */
    r->mf_half = (int) (DPSK_L * V34_FS / 600.0);
    for (int i = -r->mf_half; i <= r->mf_half; i++)
    {
        double g = pulse(i * 600.0 / V34_FS, DPSK_ALPHA, DPSK_L);

        r->mf[i + r->mf_half] = (float) g;
        sum += g;
    }
    for (int i = 0; i <= 2 * r->mf_half; i++)
        r->mf[i] = (float) (r->mf[i] / sum);
    for (int k = 0; k < V34_P2_PHASES; k++)
        r->tau[k] = 2.0 * r->mf_half + 2.0 + k * (V34_FS / 600.0) / V34_P2_PHASES;
    r->tone_since = -1;
    r->tone_last = -1;
    r->prev_since = r->prev_until = -1;
    r->rev_rearm = 0;
}

static v34_cf_t zf_at(const v34_p2rx_t *r, double t)
{
    long long i0 = (long long) floor(t);
    float f = (float) (t - (double) i0);

    return r->zf[i0 % V34_P2_HIST] * (1.0f - f) + r->zf[(i0 + 1) % V34_P2_HIST] * f;
}

bool v34_p2rx_sample(v34_p2rx_t *r, float x, long long n, double *when)
{
    int ph = (int) ((n * 3) % r->cden);
    long long i = r->n;
    v34_cf_t m = x * r->cosv[ph] - I * (x * r->sinv[ph]);
    v34_cf_t z = 0.0f;
    bool reversed = false;
    long long zi;

    r->mix[i % V34_P2_HIST] = m;
    /* Matched filter output for the sample mf_half ago. */
    if (i >= 2 * r->mf_half)
    {
        for (int k = 0; k <= 2 * r->mf_half; k++)
            z += r->mix[(i - k) % V34_P2_HIST] * r->mf[k];
    }
    zi = i - r->mf_half;
    if (zi >= 0)
        r->zf[zi % V34_P2_HIST] = z;

    /* Boxcars on the unfiltered mix: 20 ms for "is there a tone", 10 ms for
     * where it reverses. The in-band power is the filtered signal's. */
    r->sbox += m;
    r->srev += m;
    if (i >= V34_P2_BOX)
        r->sbox -= r->mix[(i - V34_P2_BOX) % V34_P2_HIST];
    if (i >= V34_P2_REV)
        r->srev -= r->mix[(i - V34_P2_REV) % V34_P2_HIST];
    r->box[i % V34_P2_HIST] = r->sbox;
    r->rev[i % V34_P2_HIST] = r->srev;
    r->pbox[i % V34_P2_HIST] = crealf(z) * crealf(z) + cimagf(z) * cimagf(z);
    r->spow += r->pbox[i % V34_P2_HIST];
    if (i >= V34_P2_BOX)
        r->spow -= r->pbox[(i - V34_P2_BOX) % V34_P2_HIST];
    if ((i & 1023) == 0)
    {
        /* Rebuild the running sums so that rounding cannot creep. */
        r->sbox = r->srev = 0.0f;
        r->spow = 0.0;
        for (long long k = (i >= V34_P2_BOX - 1) ? i - V34_P2_BOX + 1 : 0; k <= i; k++)
        {
            r->sbox += r->mix[k % V34_P2_HIST];
            r->spow += r->pbox[k % V34_P2_HIST];
        }
        for (long long k = (i >= V34_P2_REV - 1) ? i - V34_P2_REV + 1 : 0; k <= i; k++)
            r->srev += r->mix[k % V34_P2_HIST];
    }
    r->level = (float) (r->spow / V34_P2_BOX);
    {
        v34_cf_t b = r->sbox / (float) V34_P2_BOX;
        float bp = crealf(b) * crealf(b) + cimagf(b) * cimagf(b);

        r->tone = (r->level > 0.0f) ? bp / r->level : 0.0f;
        if (r->tone > 1.5f)
            r->tone = 1.5f;
    }
    if (r->tone > 0.6f)
    {
        if (r->tone_since < 0)
            r->tone_since = n;
        r->tone_last = n;
    }
    else if (r->tone < 0.3f && r->tone_since >= 0)
    {
        r->prev_since = r->tone_since;
        r->prev_until = n;
        r->tone_since = -1;
    }

    /* A reversal: two adjacent 10 ms windows of the tone, opposite in phase. */
    /* Only in a tone that was steady until the reversal began - which is
     * what keeps L2, whose 2250 and 2550 Hz tones beat inside the window,
     * from looking like one. */
    if (i >= V34_P2_HIST && n >= r->rev_rearm && r->tone_last > 0 && n - r->tone_last < V34_P2_BOX + 40)
    {
        v34_cf_t a = r->rev[i % V34_P2_HIST];
        v34_cf_t b = r->rev[(i - V34_P2_REV) % V34_P2_HIST];
        float dot = crealf(a * conjf(b));
        float mag = cabsf(a) * cabsf(b);
        float full = (float) (V34_P2_REV * V34_P2_REV) * r->level;

        /* Both windows near full strength: a reversal is steady tone on
         * either side, where the start of an INFO sequence - whose fill
         * bits reverse the carrier four times running - is not. */
        if (cabsf(a) * cabsf(a) > 0.6f * full && cabsf(b) * cabsf(b) > 0.6f * full && dot < -0.8f * mag)
        {
            /* The 10 ms window's output passes through zero when the
             * reversal is in its middle. */
            float best = 1e30f;
            long long bi = i;

            for (long long k = i - V34_P2_REV; k <= i; k++)
            {
                float v = cabsf(r->rev[k % V34_P2_HIST]);

                if (v < best)
                {
                    best = v;
                    bi = k;
                }
            }
            *when = (double) (n - (i - bi)) - (V34_P2_REV - 1) / 2.0;
            /* The tone must have been steady for 30 ms before it reversed:
             * a tone starting after something else is not a reversal. */
            {
                long long since = (r->tone_since >= 0 && r->tone_since < (long long) *when - 240)
                                      ? r->tone_since
                                      : (r->prev_until >= (long long) *when - 160 ? r->prev_since : -1);

                if (since >= 0 && (long long) *when - since > 240)
                {
                    r->rev_rearm = n + 2 * V34_P2_REV;
                    reversed = true;
                }
            }
        }
    }

    /* DPSK: differential detection at each of the four phases. */
    for (int k = 0; k < V34_P2_PHASES; k++)
        while (zi >= 1 && r->tau[k] <= (double) (zi - 1))
        {
            v34_cf_t y = zf_at(r, r->tau[k]);

            r->tau[k] += V34_FS / 600.0;
            if (r->nbits[k] < (int) sizeof(r->bits[k]))
                r->bits[k][r->nbits[k]++] = (uint8_t) (crealf(y * conjf(r->zprev[k])) < 0.0f);
            r->zprev[k] = y;
        }
    r->n++;
    return reversed;
}

/* ----------------------------------------------------------- line probing */

void v34_probe_init(v34_probe_t *p)
{
    memset(p, 0, sizeof(*p));
}

bool v34_probe_sample(v34_probe_t *p, float x)
{
    if (p->blocks >= V34_PROBE_BLOCKS)
        return true;
    p->buf[p->pos++] = x;
    if (p->pos == V34_P2_BOX)
    {
        p->pos = 0;
        for (int k = 0; k < 80; k++)
        {
            double re = 0.0, im = 0.0;

            for (int n = 0; n < V34_P2_BOX; n++)
            {
                double a = 2.0 * PI * k * n / V34_P2_BOX;

                re += p->buf[n] * cos(a);
                im -= p->buf[n] * sin(a);
            }
            p->x[p->blocks][k] = (float) (re / V34_P2_BOX) + I * (float) (im / V34_P2_BOX);
        }
        p->blocks++;
    }
    return p->blocks >= V34_PROBE_BLOCKS;
}

void v34_probe_analyse(v34_probe_t *p)
{
    if (p->blocks < 4)
    {
        p->valid = false;
        return;
    }
    for (int t = 0; t < V34_PROBE_TONES; t++)
    {
        int bin = V34_PROBE_HZ[t] / 50;
        double s = 0.0, nz = 0.0;
        double mmean = 0.0;
        double var = 0.0;

        for (int b = 0; b < p->blocks; b++)
        {
            v34_cf_t v = p->x[b][bin];

            s += crealf(v) * crealf(v) + cimagf(v) * cimagf(v);
            mmean += cabsf(v);
            for (int d = -1; d <= 1; d += 2)
            {
                v34_cf_t w = p->x[b][bin + d];

                nz += crealf(w) * crealf(w) + cimagf(w) * cimagf(w);
            }
        }
        s /= p->blocks;
        nz /= 2.0 * p->blocks;
        mmean /= p->blocks;
        for (int b = 0; b < p->blocks; b++)
        {
            double d = cabsf(p->x[b][bin]) - mmean;

            var += d * d;
        }
        /* Twice: the magnitude sees only half of a complex noise. */
        var = 2.0 * var / p->blocks;
        /* The noise in the tone's own bin shows up as its magnitude's wander
         * from block to block - not its phase's, which a far end whose clock
         * is some ppm out turns steadily - and the bins between the tones
         * see the rest. Whichever is worse, plus a floor for G.711 and the
         * arithmetic. */
        if (var > nz)
            nz = var;
        if (nz < s * 1e-6)
            nz = s * 1e-6;
        p->gain_db[t] = (float) (10.0 * log10(s + 1e-12));
        /* As derived in v34_dsp.h's terms: the SNR a QAM signal at the
         * probe's total power sees per hertz, before dividing by the
         * symbol rate - see v34_probe_snr(). */
        p->snr_db[t] = (float) (10.0 * log10(1050.0 * (s - nz > 0 ? s - nz : s * 1e-6) / nz));
    }
    p->valid = true;
}

static double probe_snr_at(const v34_probe_t *p, double f)
{
    int k;

    if (f <= V34_PROBE_HZ[0])
        return p->snr_db[0];
    if (f >= V34_PROBE_HZ[V34_PROBE_TONES - 1])
        return p->snr_db[V34_PROBE_TONES - 1];
    for (k = 0; k < V34_PROBE_TONES - 1 && V34_PROBE_HZ[k + 1] < f; k++)
        ;
    return p->snr_db[k] + (p->snr_db[k + 1] - p->snr_db[k]) * (f - V34_PROBE_HZ[k]) /
                              (V34_PROBE_HZ[k + 1] - V34_PROBE_HZ[k]);
}

float v34_probe_snr(const v34_probe_t *p, int sr, bool high, int pre_emphasis)
{
    double S = v34_symbol_rate(sr);
    double fc = v34_carrier(sr, high);
    double pe_mean = 0.0;
    double inv = 0.0;
    int n = 0;

    if (!p->valid)
        return 0.0f;
    for (double f = fc - 0.5 * S + 12.5; f < fc + 0.5 * S; f += 25.0)
    {
        pe_mean += pow(10.0, v34_pre_emphasis_db(pre_emphasis, sr, f) / 10.0);
        n++;
    }
    pe_mean /= n;
    for (double f = fc - 0.5 * S + 12.5; f < fc + 0.5 * S; f += 25.0)
    {
        double snr = pow(10.0, probe_snr_at(p, f) / 10.0) / S;

        snr *= pow(10.0, v34_pre_emphasis_db(pre_emphasis, sr, f) / 10.0) / pe_mean;
        inv += 1.0 / (1.0 + snr);
    }
    inv /= n;
    return (float) (10.0 * log10(1.0 / inv - 1.0 + 1e-9));
}

/* ---------------------------------------------------------- echo canceller */

#define EC_MU_FAST 0.5f
#define EC_MU_SETTLED 0.03f
#define EC_MU_SLOW 0.002f
#define EC_MU_FLOOR 0.0002f

bool v34_ec_init(v34_ec_t *ec, const char *tag)
{
    memset(ec, 0, sizeof(*ec));
    ec->corr = calloc(V34_EC_CORR_MAX, sizeof(double));
    ec->p0 = (float) (V34_DBM0_RMS * V34_DBM0_RMS);
    ec->tag = tag;
    return ec->corr != NULL;
}

void v34_ec_free(v34_ec_t *ec)
{
    free(ec->corr);
    ec->corr = NULL;
}

void v34_ec_tx(v34_ec_t *ec, long long n, float x, bool learn)
{
    ec->txh[n & V34_TXH_MASK] = x;
    ec->txflag[n & V34_TXH_MASK] = (uint8_t) learn;
}

void v34_ec_schedule(v34_ec_t *ec, long long t0, double rtd)
{
    if (rtd < 0.0)
        rtd = 4000.0;
    ec->state = V34_EC_WAIT;
    /* Our training's echo is back within a round trip; give whatever went
     * before it a little longer to die away. */
    ec->corr_start = t0 + (long long) rtd + 320;
    ec->corr_lmax = (int) rtd + 480;
    if (ec->corr_lmax > V34_EC_CORR_MAX)
        ec->corr_lmax = V34_EC_CORR_MAX;
}

static void ec_decide(v34_ec_t *ec, long long n)
{
    double best = 0.0;
    int p = 0;
    double etx = 0.0;
    double rho;

    ec->state = V34_EC_DONE;
    for (int lag = 0; lag < ec->corr_lmax; lag++)
        if (fabs(ec->corr[lag]) > best)
        {
            best = fabs(ec->corr[lag]);
            p = lag;
        }
    for (long long k = ec->corr_start; k < ec->corr_start + V34_EC_CORR_WIN; k++)
    {
        float x = ec->txh[(k - p) & V34_TXH_MASK];

        etx += (double) x * x;
    }
    rho = (ec->corr_erx > 0.0 && etx > 0.0) ? best / sqrt(ec->corr_erx * etx) : 0.0;
    ec->erl_db = (ec->corr_erx > 0.0) ? (float) (10.0 * log10(etx / ec->corr_erx)) : 99.0f;
    if (rho < 0.2 || ec->corr_erx / V34_EC_CORR_WIN < ec->p0 * 1e-6)
    {
        DM_DEBUG("v34", "no echo of our own signal came back (correlation %.2f, %.1f dBm0) (tag=%s)", rho,
                 10.0 * log10(ec->corr_erx / V34_EC_CORR_WIN / ec->p0 + 1e-12), ec->tag);
        ec->enabled = false;
        return;
    }
    ec->enabled = true;
    ec->delay = (p > V34_EC_PRE) ? p - V34_EC_PRE : 0;
    memset(ec->w, 0, sizeof(ec->w));
    ec->mu = EC_MU_FAST;
    ec->fast_from = n;
    ec->in_avg = ec->res_avg = 0.0f;
    ec->erle_db = 0.0f;
    DM_DEBUG("v34", "our own signal comes back %.1f ms later, %.1f dB down (correlation %.2f); cancelling it "
                    "(tag=%s)",
             p / 8.0, ec->erl_db, rho, ec->tag);
}

void v34_ec_slow(v34_ec_t *ec, long long n)
{
    if (!ec->enabled || ec->mu == EC_MU_SLOW)
        return;
    if (ec->res_avg > 0.0f && ec->in_avg > 0.0f)
        ec->erle_db = 10.0f * log10f(ec->in_avg / ec->res_avg);
    ec->mu = EC_MU_SLOW;
    ec->slow_from = n;
    DM_DEBUG("v34", "echo canceller trained: %.1f dB of cancellation (tag=%s)", ec->erle_db, ec->tag);
}

float v34_ec_run(v34_ec_t *ec, float x, long long n, float far_power, float pmin)
{
    long long i0;
    float est = 0.0f;
    float norm = 0.0f;
    float e;

    if (ec->state == V34_EC_WAIT && n >= ec->corr_start)
    {
        ec->state = V34_EC_CORR;
        memset(ec->corr, 0, sizeof(double) * V34_EC_CORR_MAX);
        ec->corr_erx = 0.0;
    }
    if (ec->state == V34_EC_CORR)
    {
        for (int lag = 0; lag < ec->corr_lmax; lag++)
            ec->corr[lag] += (double) x * ec->txh[(n - lag) & V34_TXH_MASK];
        ec->corr_erx += (double) x * x;
        if (n >= ec->corr_start + V34_EC_CORR_WIN - 1)
            ec_decide(ec, n);
    }
    if (!ec->enabled)
        return x;

    i0 = n - ec->delay;
    for (int k = 0; k < V34_EC_TAPS; k++)
    {
        float r = ec->txh[(i0 - k) & V34_TXH_MASK];

        est += ec->w[k] * r;
        norm += r * r;
    }
    e = x - est;
    if (ec->mu > 0.0f && n >= ec->freeze_until && ec->txflag[i0 & V34_TXH_MASK] && norm > 1.0f)
    {
        float mu = ec->mu;
        float g;

        if (mu == EC_MU_FAST)
        {
            mu = EC_MU_FAST * expf(-(float) (n - ec->fast_from) / 1000.0f);
            if (mu < EC_MU_SETTLED)
                mu = EC_MU_SETTLED;
        }
        else
        {
            mu = EC_MU_SLOW * expf(-(float) (n - ec->slow_from) / 16000.0f);
            if (mu < EC_MU_FLOOR)
                mu = EC_MU_FLOOR;
        }
        g = mu * e / (norm + 1000.0f);
        for (int k = 0; k < V34_EC_TAPS; k++)
            ec->w[k] += g * ec->txh[(i0 - k) & V34_TXH_MASK];
        if (ec->mu == EC_MU_FAST)
        {
            bool settled = (n - ec->fast_from) > V34_EC_CONVERGE;
            float e2 = e * e;

            if (settled && far_power > 8.0f * ec->res_avg && far_power > pmin)
            {
                v34_ec_slow(ec, n);
            }
            else
            {
                if (settled && e2 > 4.0f * ec->res_avg)
                    e2 = 4.0f * ec->res_avg;
                ec->in_avg += 0.002f * (x * x - ec->in_avg);
                ec->res_avg += 0.002f * (e2 - ec->res_avg);
            }
        }
    }
    return e;
}
