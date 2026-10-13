#include "datamodem/util.h"

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <time.h>

/* Simulated time, for the selftest. It runs two modems against each other
 * flat out - two minutes of audio in a few tens of milliseconds - so every
 * deadline in the modem is frozen as far as the wall clock is concerned, and
 * nothing that depends on one can be tested there. The harness advances this
 * in step with the audio it generates; it stays zero everywhere else. */
static int64_t dm_clock_offset_ms = 0;

void dm_clock_advance_ms(int64_t ms)
{
    if (ms > 0)
        dm_clock_offset_ms += ms;
}

int64_t dm_now_real_ms(void)
{
    struct timespec ts;

#if defined(CLOCK_MONOTONIC)
    clock_gettime(CLOCK_MONOTONIC, &ts);
#else
    clock_gettime(CLOCK_REALTIME, &ts);
#endif
    return (int64_t) ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

int64_t dm_now_ms(void)
{
    struct timespec ts;
#if defined(CLOCK_MONOTONIC)
    clock_gettime(CLOCK_MONOTONIC, &ts);
#else
    clock_gettime(CLOCK_REALTIME, &ts);
#endif
    return (int64_t) ts.tv_sec * 1000 + ts.tv_nsec / 1000000 + dm_clock_offset_ms;
}

bool dm_mkdir_parent(const char *path, char *err, size_t err_len)
{
    char buf[1024];
    char *slash;

    if (path == NULL || *path == '\0')
    {
        snprintf(err, err_len, "empty path");
        return false;
    }
    if (strlen(path) >= sizeof(buf))
    {
        snprintf(err, err_len, "path too long: %s", path);
        return false;
    }
    snprintf(buf, sizeof(buf), "%s", path);

    slash = strrchr(buf, '/');
    if (slash == NULL || slash == buf)
        return true; /* current directory, or the root: nothing to create */
    *slash = '\0';

    for (char *p = buf + 1; *p != '\0'; p++)
    {
        if (*p != '/')
            continue;
        *p = '\0';
        if (mkdir(buf, 0755) != 0 && errno != EEXIST)
        {
            snprintf(err, err_len, "mkdir %s: %s", buf, strerror(errno));
            return false;
        }
        *p = '/';
    }
    if (mkdir(buf, 0755) != 0 && errno != EEXIST)
    {
        snprintf(err, err_len, "mkdir %s: %s", buf, strerror(errno));
        return false;
    }
    return true;
}

void dm_format_duration(int64_t ms, char *out, size_t out_len)
{
    long total = (long) (ms / 1000);
    long h = total / 3600;
    long m = (total % 3600) / 60;
    long s = total % 60;

    if (h > 0)
        snprintf(out, out_len, "%ldh%02ldm%02lds", h, m, s);
    else if (m > 0)
        snprintf(out, out_len, "%ldm%02lds", m, s);
    else
        snprintf(out, out_len, "%ld.%01lds", s, (long) ((ms % 1000) / 100));
}

void dm_format_byte(int c, char *out, size_t out_len)
{
    if (c >= 0x20 && c < 0x7f)
        snprintf(out, out_len, "%c", (char) c);
    else
        snprintf(out, out_len, "<%02X>", (unsigned) (c & 0xff));
}

bool dm_parse_key_spec(const char *s, int *out)
{
    long n;
    char *end;

    if (s == NULL || *s == '\0' || strcasecmp(s, "none") == 0 || strcasecmp(s, "off") == 0)
    {
        *out = -1;
        return true;
    }
    /* "^]" and friends: the control character the caret names. */
    if (s[0] == '^' && s[1] != '\0' && s[2] == '\0')
    {
        int c = toupper((unsigned char) s[1]);
        if (c == '?')
        {
            *out = 0x7f;
            return true;
        }
        if (c < '@' || c > '_')
            return false;
        *out = c - '@';
        return true;
    }
    /* A single non-digit character stands for itself, so --escape-char '+'
     * reads the way it is meant to. A lone digit is still a number. */
    if (s[1] == '\0' && !isdigit((unsigned char) s[0]))
    {
        *out = (unsigned char) s[0];
        return true;
    }
    errno = 0;
    n = strtol(s, &end, 0);
    if (errno != 0 || end == s || *end != '\0' || n < 0 || n > 255)
        return false;
    *out = (int) n;
    return true;
}

/* Appends c to out, which has room for out_len bytes including the NUL,
 * unless it is a control character or there is no room left. */
static void party_put(char *out, size_t out_len, size_t *n, unsigned char c)
{
    if (c < 0x20 || c == 0x7f || *n + 1 >= out_len)
        return;
    out[(*n)++] = (char) c;
    out[*n] = '\0';
}

static int hex_value(int c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    c = tolower(c);
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

void dm_parse_sip_party(const char *s, char *name, size_t name_len, char *user, size_t user_len)
{
    const char *uri;
    const char *lt;
    size_t n = 0;

    if (name_len > 0)
        name[0] = '\0';
    if (user_len > 0)
        user[0] = '\0';
    if (s == NULL || name_len == 0 || user_len == 0)
        return;

    while (*s == ' ' || *s == '\t')
        s++;

    /* The display name: quoted, with backslash escapes, or a run of tokens
     * up to the '<' that opens the URI. */
    lt = strchr(s, '<');
    if (*s == '"')
    {
        for (s++; *s != '\0' && *s != '"'; s++)
        {
            if (*s == '\\' && s[1] != '\0')
                s++;
            party_put(name, name_len, &n, (unsigned char) *s);
        }
        lt = strchr(s, '<');
    }
    else if (lt != NULL)
    {
        const char *end = lt;

        while (end > s && (end[-1] == ' ' || end[-1] == '\t'))
            end--;
        for (; s < end; s++)
            party_put(name, name_len, &n, (unsigned char) *s);
    }
    uri = (lt != NULL) ? lt + 1 : s;

    /* The user part: after the scheme, up to the host (sip, sips) or the
     * first parameter (tel, which has no host). sip:host alone has no user
     * part at all. */
    {
        bool is_tel = false;
        const char *at;
        const char *end;

        n = 0;
        if (strncasecmp(uri, "sip:", 4) == 0)
            uri += 4;
        else if (strncasecmp(uri, "sips:", 5) == 0)
            uri += 5;
        else if (strncasecmp(uri, "tel:", 4) == 0)
        {
            uri += 4;
            is_tel = true;
        }
        else
            return;

        at = strpbrk(uri, is_tel ? ">;?" : "@>;?");
        if (!is_tel && (at == NULL || *at != '@'))
            return;
        end = (at != NULL) ? at : uri + strlen(uri);
        for (; uri < end; uri++)
        {
            if (*uri == '%' && hex_value(uri[1]) >= 0 && hex_value(uri[2]) >= 0)
            {
                party_put(user, user_len, &n,
                          (unsigned char) (hex_value(uri[1]) * 16 + hex_value(uri[2])));
                uri += 2;
            }
            else
            {
                party_put(user, user_len, &n, (unsigned char) *uri);
            }
        }
    }
}
