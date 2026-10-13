/*
 * The engine's logging API (engine/include/datamodem/log.h), for a host that
 * is not datamodem.
 *
 * datamodem's own log.c owns stderr, a log file and a writer thread. Here the
 * host owns logging - Asterisk's logger inside the module, stderr in the
 * tests - so every line goes to a sink the host installs, and the host says
 * which levels are worth formatting at all. Only what the engine calls is
 * implemented.
 */
#include "softmodem/bridge.h"
#include "datamodem/log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static sm_log_sink_t g_sink;
static sm_log_enabled_t g_enabled;
static dm_log_level_t g_level = DM_LOG_INFO;

void sm_log_set_sink(sm_log_sink_t sink, sm_log_enabled_t enabled)
{
	g_sink = sink;
	g_enabled = enabled;
}

void sm_log_set_level(dm_log_level_t level)
{
	g_level = level;
}

static void rtrim(char *s)
{
	size_t n = strlen(s);

	while (n > 0 && (s[n - 1] == '\n' || s[n - 1] == '\r' || s[n - 1] == ' '))
		s[--n] = '\0';
}

static void emit(dm_log_level_t level, const char *component, const char *msg)
{
	if (g_sink != NULL)
		g_sink(level, component, msg);
	else
		fprintf(stderr, "%-5s [%s] %s\n", dm_log_level_name(level), component, msg);
}

const char *dm_log_level_name(dm_log_level_t level)
{
	switch (level) {
	case DM_LOG_ERROR:
		return "error";
	case DM_LOG_WARN:
		return "warn";
	case DM_LOG_INFO:
		return "info";
	case DM_LOG_DEBUG:
		return "debug";
	default:
		return "trace";
	}
}

dm_log_level_t dm_log_get_level(void)
{
	return g_level;
}

bool dm_log_enabled(dm_log_level_t level)
{
	if (g_enabled != NULL)
		return g_enabled(level);
	return level <= g_level;
}

void dm_logf(dm_log_level_t level, const char *component, const char *fmt, ...)
{
	char msg[4096];
	va_list ap;

	if (!dm_log_enabled(level))
		return;
	va_start(ap, fmt);
	vsnprintf(msg, sizeof(msg), fmt, ap);
	va_end(ap);
	rtrim(msg);
	emit(level, component, msg);
}

void dm_log_event(dm_log_level_t level, const char *component, const char *event, const char *fmt, ...)
{
	char detail[3072];
	char msg[4096];
	va_list ap;

	if (!dm_log_enabled(level))
		return;
	va_start(ap, fmt);
	vsnprintf(detail, sizeof(detail), fmt, ap);
	va_end(ap);
	rtrim(detail);
	snprintf(msg, sizeof(msg), "%s %s", event, detail);
	emit(level, component, msg);
}

void dm_log_spandsp_message(int level, const char *text)
{
	char buf[4096];
	dm_log_level_t mapped;

	/* spandsp's SPAN_LOG_* severities, as datamodem maps them. */
	if (level <= 1)
		mapped = DM_LOG_ERROR;
	else if (level <= 4)
		mapped = DM_LOG_WARN;
	else if (level <= 6)
		mapped = DM_LOG_DEBUG;
	else
		mapped = DM_LOG_TRACE;
	if (!dm_log_enabled(mapped))
		return;
	snprintf(buf, sizeof(buf), "%s", text);
	rtrim(buf);
	if (buf[0] != '\0')
		emit(mapped, "dsp", buf);
}

void dm_log_spandsp_error(const char *text)
{
	char buf[4096];

	snprintf(buf, sizeof(buf), "%s", text);
	rtrim(buf);
	if (buf[0] != '\0')
		emit(DM_LOG_ERROR, "dsp", buf);
}
