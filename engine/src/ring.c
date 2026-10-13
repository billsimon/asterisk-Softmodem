#include "datamodem/ring.h"

#include <stdlib.h>
#include <string.h>

bool dm_ring_init(dm_ring_t *r, size_t capacity)
{
    memset(r, 0, sizeof(*r));
    r->buf = malloc(capacity);
    if (r->buf == NULL)
        return false;
    r->cap = capacity;
    pthread_mutex_init(&r->lock, NULL);
    return true;
}

void dm_ring_destroy(dm_ring_t *r)
{
    if (r->buf == NULL)
        return;
    pthread_mutex_destroy(&r->lock);
    free(r->buf);
    r->buf = NULL;
    r->cap = 0;
}

/* Callers hold the lock. */
static void push(dm_ring_t *r, const unsigned char *data, size_t len)
{
    size_t tail = (r->head + r->len) % r->cap;
    size_t first = r->cap - tail;

    if (first > len)
        first = len;
    memcpy(r->buf + tail, data, first);
    if (len > first)
        memcpy(r->buf, data + first, len - first);
    r->len += len;
}

static void drop_oldest(dm_ring_t *r, size_t n)
{
    if (n > r->len)
        n = r->len;
    r->head = (r->head + n) % r->cap;
    r->len -= n;
    r->dropped += n;
}

size_t dm_ring_write(dm_ring_t *r, const void *data, size_t len)
{
    size_t space;

    pthread_mutex_lock(&r->lock);
    space = r->cap - r->len;
    if (len > space)
        len = space;
    if (len > 0)
        push(r, data, len);
    pthread_mutex_unlock(&r->lock);
    return len;
}

void dm_ring_write_overwrite(dm_ring_t *r, const void *data, size_t len)
{
    const unsigned char *p = data;

    pthread_mutex_lock(&r->lock);
    /* More than a whole buffer at once: only the tail could ever be read. */
    if (len >= r->cap)
    {
        r->dropped += r->len + (len - r->cap);
        r->head = 0;
        r->len = 0;
        p += len - r->cap;
        len = r->cap;
    }
    if (len > r->cap - r->len)
        drop_oldest(r, len - (r->cap - r->len));
    push(r, p, len);
    pthread_mutex_unlock(&r->lock);
}

size_t dm_ring_read(dm_ring_t *r, void *out, size_t max)
{
    unsigned char *dst = out;
    size_t n;
    size_t first;

    pthread_mutex_lock(&r->lock);
    n = (max < r->len) ? max : r->len;
    first = r->cap - r->head;
    if (first > n)
        first = n;
    memcpy(dst, r->buf + r->head, first);
    if (n > first)
        memcpy(dst + first, r->buf, n - first);
    r->head = (r->head + n) % r->cap;
    r->len -= n;
    pthread_mutex_unlock(&r->lock);
    return n;
}

int dm_ring_getc(dm_ring_t *r)
{
    int c = -1;

    pthread_mutex_lock(&r->lock);
    if (r->len > 0)
    {
        c = r->buf[r->head];
        r->head = (r->head + 1) % r->cap;
        r->len--;
    }
    pthread_mutex_unlock(&r->lock);
    return c;
}

size_t dm_ring_len(dm_ring_t *r)
{
    size_t n;

    pthread_mutex_lock(&r->lock);
    n = r->len;
    pthread_mutex_unlock(&r->lock);
    return n;
}

size_t dm_ring_space(dm_ring_t *r)
{
    size_t n;

    pthread_mutex_lock(&r->lock);
    n = r->cap - r->len;
    pthread_mutex_unlock(&r->lock);
    return n;
}

size_t dm_ring_dropped(dm_ring_t *r)
{
    size_t n;

    pthread_mutex_lock(&r->lock);
    n = r->dropped;
    pthread_mutex_unlock(&r->lock);
    return n;
}

void dm_ring_clear(dm_ring_t *r)
{
    pthread_mutex_lock(&r->lock);
    r->head = 0;
    r->len = 0;
    pthread_mutex_unlock(&r->lock);
}
