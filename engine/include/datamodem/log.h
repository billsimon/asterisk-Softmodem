/* Logging.
 *
 * Everything datamodem says about itself goes to stderr, one line per event.
 * stdout belongs to the remote system: it carries the bytes that came off the
 * line and nothing else, so `datamodem 5551234 > session.txt` captures a clean
 * transcript. Use --log-file to get the diagnostics off the terminal
 * altogether while a session is running. */
#ifndef DATAMODEM_LOG_H
#define DATAMODEM_LOG_H

#include <stdbool.h>
#include <stddef.h>

typedef enum
{
    DM_LOG_ERROR = 0,
    DM_LOG_WARN,
    DM_LOG_INFO,
    DM_LOG_DEBUG,
    DM_LOG_TRACE
} dm_log_level_t;

/* Initialise the logger. Safe to call more than once. */
void dm_log_init(dm_log_level_t level, bool json);

/* Send log output to a file instead of stderr. Returns false (and keeps
 * stderr) if the file cannot be opened. */
bool dm_log_set_file(const char *path, char *err, size_t err_len);
void dm_log_close(void);

/* True when logs and the session share a terminal, so the session code knows
 * it has to keep its own output tidy. */
bool dm_log_shares_terminal(void);

/* For the terminal UI: a function that sees every line that is logged (any
 * thread; NULL to stop), and the least important level still written to the
 * terminal when there is no log file - DM_LOG_ERROR - 1 for none at all. */
void dm_log_set_tap(void (*tap)(dm_log_level_t level, const char *component, const char *msg));
void dm_log_set_terminal_floor(int level);

/* Parse "error"/"warn"/"info"/"debug"/"trace". Returns false on a bad name. */
bool dm_log_level_parse(const char *name, dm_log_level_t *out);
const char *dm_log_level_name(dm_log_level_t level);

dm_log_level_t dm_log_get_level(void);
bool dm_log_enabled(dm_log_level_t level);

void dm_logf(dm_log_level_t level, const char *component, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

/* Emits a line that carries structured key=value pairs after the message, e.g.
 * dm_log_event(DM_LOG_INFO, "modem", "connected", "rate=%d ...").
 * In JSON mode the message and the pairs become separate fields. */
void dm_log_event(dm_log_level_t level, const char *component, const char *event,
                  const char *fmt, ...) __attribute__((format(printf, 4, 5)));

#define DM_ERROR(comp, ...) dm_logf(DM_LOG_ERROR, (comp), __VA_ARGS__)
#define DM_WARN(comp, ...) dm_logf(DM_LOG_WARN, (comp), __VA_ARGS__)
#define DM_INFO(comp, ...) dm_logf(DM_LOG_INFO, (comp), __VA_ARGS__)
#define DM_DEBUG(comp, ...) dm_logf(DM_LOG_DEBUG, (comp), __VA_ARGS__)
#define DM_TRACE(comp, ...) dm_logf(DM_LOG_TRACE, (comp), __VA_ARGS__)

/* Sinks for the two libraries, so their output lands in our stream in our
 * format rather than straight on the user's terminal. */
void dm_log_pjsip_writer(int level, const char *data, int len);
void dm_log_spandsp_message(int level, const char *text);
void dm_log_spandsp_error(const char *text);

#endif /* DATAMODEM_LOG_H */
