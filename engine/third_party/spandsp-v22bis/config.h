/* Minimal config.h for building spandsp's V.22bis sources inside datamodem.
 *
 * spandsp normally generates this with autotools. We only build two of its
 * files, and all they need from it is to be told that the platform has the
 * single-precision maths functions - otherwise floating_fudge.h defines its
 * own shims and they collide with <math.h> on any modern toolchain.
 *
 * Deliberately does NOT define SPANDSP_USE_FIXED_POINT. The fixed-point
 * variant of V.22bis has its equaliser update commented out in the upstream
 * source, so it has no adaptive equaliser at all. */
#ifndef DATAMODEM_SPANDSP_CONFIG_H
#define DATAMODEM_SPANDSP_CONFIG_H

#define HAVE_ACOSF 1
#define HAVE_ASINF 1
#define HAVE_ATANF 1
#define HAVE_ATAN2F 1
#define HAVE_CEILF 1
#define HAVE_COSF 1
#define HAVE_EXPF 1
#define HAVE_FLOORF 1
#define HAVE_LOGF 1
#define HAVE_LOG10F 1
#define HAVE_POWF 1
#define HAVE_SINF 1
#define HAVE_TANF 1
#define HAVE_MATH_H 1
#define HAVE_STDINT_H 1
#define HAVE_INTTYPES_H 1

#endif /* DATAMODEM_SPANDSP_CONFIG_H */
