/* Configuration: command line flags, environment variables and an optional
 * key=value config file, merged in that order of precedence. */
#ifndef DATAMODEM_CONFIG_H
#define DATAMODEM_CONFIG_H

#include <stdbool.h>
#include <stddef.h>

#include "datamodem/log.h"

#define DM_STR_MAX 512

/* Process exit codes. Stable: scripts depend on them. */
typedef enum
{
    DM_EXIT_OK = 0,          /* the call ran and was cleared normally */
    DM_EXIT_USAGE = 2,       /* bad flags */
    DM_EXIT_CONFIG = 3,      /* missing credentials, nonsense settings */
    DM_EXIT_SIP = 4,         /* transport or registration failure */
    DM_EXIT_CALL = 5,        /* call rejected, busy, or never answered */
    DM_EXIT_NO_CARRIER = 6,  /* call answered but the modems never trained */
    DM_EXIT_TIMEOUT = 7,     /* a deadline was hit */
    DM_EXIT_INTERNAL = 8     /* we broke */
} dm_exit_code_t;

typedef enum
{
    DM_CMD_NONE = 0,
    DM_CMD_DIAL,
    DM_CMD_ANSWER,
    DM_CMD_SELFTEST,
    DM_CMD_VERSION,
    DM_CMD_HELP
} dm_command_t;

typedef struct
{
    dm_command_t command;

    /* SIP account */
    char server[DM_STR_MAX];       /* registrar / domain, e.g. sip.example.com or host:5060 */
    char username[DM_STR_MAX];     /* SIP user part */
    char auth_user[DM_STR_MAX];    /* auth username, defaults to username */
    char password[DM_STR_MAX];
    char realm[DM_STR_MAX];        /* digest realm, "*" by default */
    char from_uri[DM_STR_MAX];     /* overrides the derived sip:user@server */
    char proxy[DM_STR_MAX];        /* outbound proxy URI */
    char caller_id[DM_STR_MAX];    /* display name / P-Asserted number */
    bool do_register;
    bool register_explicit;        /* --register/--no-register was given, so do not
                                    * override it with the per-command default */
    int reg_timeout_s;             /* how long to wait for a 200 to REGISTER */
    int reg_expires_s;

    /* SIP transport */
    char transport[16];            /* udp | tcp | tls */
    char bind_addr[DM_STR_MAX];
    int local_port;                /* 0 = ephemeral */
    char public_addr[DM_STR_MAX];  /* advertised address when behind NAT */
    char stun_server[DM_STR_MAX];
    char nameserver[DM_STR_MAX];
    int rtp_port;                  /* base RTP port, 0 = pjsip default */
    int rtp_port_range;            /* ports above rtp_port that media may use */

    /* Media */
    char codec[16];                /* pcmu | pcma */
    int jitter_buffer_ms;
    bool user_phone;               /* append ;user=phone to the request URI */
    bool speaker;                  /* play the line until carrier, like ATM1 */

    /* Modem */
    char modulation[16];           /* v34 | v32bis | v32 | v22bis | v22 | v21 | bell103 | v23: the
                                    * fastest to try, and with step_down only the first */
    bool step_down;                /* fall back to slower modulations the far end turns out to want */
    int bit_rate;                  /* V.34 2400 ... 33600, V.32bis 14400 ... 4800, V.32 9600 | 4800,
                                    * V.22bis 2400 | 1200, 0 = the most it can do */
    char guard_tone[8];            /* none | 550 | 1800 (V.22bis answerer only) */
    int data_bits;                 /* 5..8 */
    char parity[8];                /* none | even | odd */
    int stop_bits;                 /* 1 | 2 */
    bool v14;                      /* V.14 rate adaption on the receive side */
    int tx_power;                  /* transmit level, dBm0 (negative) */
    int answer_tone_ms;            /* ANS burst we send when answering; 0 = none */
    int answer_wait_s;             /* how long the caller listens for ANS before
                                    * training anyway; 0 = do not wait at all */
    int answer_tail_ms;            /* how long the caller waits after hearing ANS
                                    * for the tone to finish before it trains */
    bool calling_tone;             /* send the V.25 1300 Hz calling tone while waiting */
    int train_timeout_s;           /* give up if the modems have not trained by then */
    int max_retrains;              /* give up on a link that will not hold; 0 = never */

    /* Error correction and compression */
    char v42[16];                  /* off | detect | require */
    int v42_timeout_s;             /* detect: fall back to raw async after this */
    bool v42bis;                   /* compression; needs v42 */
    int v42bis_dict;               /* P1, dictionary size in codewords */
    int v42bis_max_string;         /* P2, longest string the dictionary may hold */

    /* Session */
    char to[DM_STR_MAX];           /* number to dial, or a full SIP URI */
    int escape_char;               /* Hayes S2: the character in "+++"; >127 disables */
    int escape_guard_ms;           /* Hayes S12: silence needed either side of it */
    int escape_key;                /* immediate local escape, -1 = disabled */
    bool local_echo;               /* echo typed characters locally (ATE1) */
    bool tui;                      /* full screen with a status line, when interactive */
    char charset[16];              /* the far end's 8-bit characters: cp437 | utf8 | ascii */
    int connect_timeout_s;         /* SIP: ringing until answered */
    int answer_timeout_s;          /* `answer`: how long to wait for a call, 0 = forever */
    int idle_timeout_s;            /* hang up after this long with no data either way; 0 = off */
    int media_timeout_s;           /* hang up after this long with no inbound RTP; 0 = off */
    int max_call_s;                /* hard ceiling on one call; 0 = off */
    char exec[DM_STR_MAX];         /* run this per call, with the line as its stdin/stdout */
    bool exec_tty;                 /* ...on a pseudo-terminal rather than a pipe */
    int calls;                     /* `answer`: calls to take before exiting, 0 = no limit */
    bool hangup_on_eof;            /* clear the call once input ends and is sent */
    /* Logging */
    dm_log_level_t log_level;
    bool log_json;
    char log_file[DM_STR_MAX];
    int pjsip_log_level;           /* -1 = derive from log_level */
    int spandsp_log_level;         /* -1 = derive from log_level */
} dm_config_t;

void dm_config_defaults(dm_config_t *cfg);

/* Applies DATAMODEM_* environment variables over the defaults. */
void dm_config_apply_env(dm_config_t *cfg);

/* Reads a key=value file (# comments, blank lines ignored). Returns false and
 * fills err on a syntax error or unknown key. */
bool dm_config_apply_file(dm_config_t *cfg, const char *path, char *err, size_t err_len);

/* Sets one option by its long-flag name, e.g. "bit-rate". Used by the file
 * parser and the env loader so every route accepts the same vocabulary.
 * Returns false for an unknown key or a bad value. */
bool dm_config_set(dm_config_t *cfg, const char *key, const char *value, char *err, size_t err_len);

/* Parses argv. Returns an exit code; DM_EXIT_OK means cfg is ready. */
int dm_config_parse_args(dm_config_t *cfg, int argc, char *const argv[]);

/* Validates the config for cfg->command. */
bool dm_config_validate(const dm_config_t *cfg, char *err, size_t err_len);

void dm_config_log(const dm_config_t *cfg);
void dm_usage(void);

#endif /* DATAMODEM_CONFIG_H */
