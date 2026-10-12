/* A byte FIFO shared between the terminal thread and the media thread.
 *
 * Both directions of a call cross a thread boundary: pjmedia pulls 20 ms of
 * modem output from its own thread every 20 ms, while the bytes being typed
 * (and the bytes to print) belong to the main loop. One small lock per queue
 * is cheaper and far easier to reason about than lock-free tricks at 2400
 * bits per second. */
#ifndef DATAMODEM_RING_H
#define DATAMODEM_RING_H

#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct
{
    unsigned char *buf;
    size_t cap;
    size_t head;         /* read position */
    size_t len;          /* bytes currently held */
    size_t dropped;      /* bytes lost to overrun since creation */
    pthread_mutex_t lock;
} dm_ring_t;

bool dm_ring_init(dm_ring_t *r, size_t capacity);
void dm_ring_destroy(dm_ring_t *r);

/* Appends what fits and returns how many bytes were taken. A short return is
 * backpressure, not an error: the caller should stop reading its source until
 * there is room again. */
size_t dm_ring_write(dm_ring_t *r, const void *data, size_t len);

/* Appends all of data, discarding the oldest bytes if it does not fit, and
 * counts what was lost. For the receive queue, where there is nothing to push
 * back against. */
void dm_ring_write_overwrite(dm_ring_t *r, const void *data, size_t len);

size_t dm_ring_read(dm_ring_t *r, void *out, size_t max);

/* Removes and returns one byte, or -1 when empty. The modem's per-character
 * fetch. */
int dm_ring_getc(dm_ring_t *r);

size_t dm_ring_len(dm_ring_t *r);
size_t dm_ring_space(dm_ring_t *r);
size_t dm_ring_dropped(dm_ring_t *r);
void dm_ring_clear(dm_ring_t *r);

#endif /* DATAMODEM_RING_H */
