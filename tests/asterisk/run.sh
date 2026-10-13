#!/usr/bin/env bash
# End-to-end: a real Asterisk running app_softmodem.so, and datamodem
# (https://github.com/billsimon/datamodem) as the modem at the other end of a
# real SIP call over loopback - RTP, G.711, the lot.
#
#   inbound   datamodem dials Asterisk, as a user dials a BBS; the Softmodem
#             answers, and a test server behind it sends a banner and a block
#             of text, echoes a line back and hangs up.
#   outbound  Asterisk originates a call to an answering datamodem that runs
#             cat; the Softmodem calls, and the test server sends binary and
#             expects it back byte for byte, then hangs up.
#
# Each runs at every modulation, with the far end fixed to it and the
# Softmodem at its defaults - so it is the Softmodem that has to find out what
# answered or what called.
#
#   tests/asterisk/run.sh                  everything
#   tests/asterisk/run.sh inbound-v22bis   only the named tests (see TESTS below)
#
# Needs: Asterisk with chan_pjsip, python3, and datamodem. Set
#   ASTERISK              the asterisk binary            (default: from PATH)
#   ASTERISK_MODULES_DIR  its modules                    (default: beside it, or /usr/lib/asterisk/modules)
#   DATAMODEM             the datamodem binary           (default: ../datamodem/build/datamodem, then PATH)
#   MODULE                the app_softmodem.so to test   (default: ./app_softmodem.so)
#   KEEP=1                keep the work directory even when everything passes
set -uo pipefail
ARGS=("$@")

ROOT=$(cd "$(dirname "$0")/../.." && pwd)
HERE="$ROOT/tests/asterisk"

ASTERISK=${ASTERISK:-$(command -v asterisk || true)}
if [ -z "$ASTERISK" ] || [ ! -x "$ASTERISK" ]; then
    echo "no asterisk binary: set ASTERISK" >&2
    exit 2
fi
if [ -z "${ASTERISK_MODULES_DIR:-}" ]; then
    for d in "$(dirname "$ASTERISK")/../lib/asterisk/modules" "$(dirname "$ASTERISK")/../lib64/asterisk/modules" \
             /usr/lib/asterisk/modules /usr/lib64/asterisk/modules; do
        if [ -f "$d/chan_pjsip.so" ]; then
            ASTERISK_MODULES_DIR=$(cd "$d" && pwd)
            break
        fi
    done
fi
if [ ! -f "${ASTERISK_MODULES_DIR:-}/chan_pjsip.so" ]; then
    echo "no chan_pjsip.so in ${ASTERISK_MODULES_DIR:-(not found)}: set ASTERISK_MODULES_DIR" >&2
    exit 2
fi
DATAMODEM=${DATAMODEM:-}
if [ -z "$DATAMODEM" ]; then
    if [ -x "$ROOT/../datamodem/build/datamodem" ]; then
        DATAMODEM="$ROOT/../datamodem/build/datamodem"
    else
        DATAMODEM=$(command -v datamodem || true)
    fi
fi
if [ -z "$DATAMODEM" ] || [ ! -x "$DATAMODEM" ]; then
    echo "no datamodem binary: set DATAMODEM" >&2
    exit 2
fi
MODULE=${MODULE:-$ROOT/app_softmodem.so}
if [ ! -f "$MODULE" ]; then
    echo "no $MODULE: run make first" >&2
    exit 2
fi

# Asterisk's control socket has to fit in a sockaddr_un.
WORK=$(mktemp -d "${TMPDIR:-/tmp}/smtest.XXXXXX")
if [ ${#WORK} -gt 70 ]; then
    rmdir "$WORK"
    WORK=$(mktemp -d /tmp/smtest.XXXXXX)
fi

SIP_PORT=${SIP_PORT:-5160}
DIAL_PORT=${DIAL_PORT:-5170}
ANS_PORT=${ANS_PORT:-5180}

AST_PID=""
cleanup() {
    if [ -n "$AST_PID" ]; then
        "$ASTERISK" -C "$WORK/etc/asterisk.conf" -rx "core stop now" >/dev/null 2>&1
        for _ in $(seq 1 50); do kill -0 "$AST_PID" 2>/dev/null || break; sleep 0.1; done
        kill "$AST_PID" 2>/dev/null
    fi
    for p in $(jobs -p); do kill "$p" 2>/dev/null; done
    if [ "${failed:-1}" -ne 0 ] || [ -n "${KEEP:-}" ]; then
        echo "logs in $WORK"
    else
        rm -rf "${WORK:?}"
    fi
}
trap cleanup EXIT

# ------------------------------------------------------------ the Asterisk

mkdir -p "$WORK/etc" "$WORK/modules" "$WORK/var/lib" "$WORK/var/spool" "$WORK/run" "$WORK/log"
for m in "$ASTERISK_MODULES_DIR"/*.so; do
    ln -sf "$m" "$WORK/modules/"
done
ln -sf "$MODULE" "$WORK/modules/app_softmodem.so"

cat >"$WORK/etc/asterisk.conf" <<EOF
[directories]
astetcdir => $WORK/etc
astmoddir => $WORK/modules
astvarlibdir => $WORK/var/lib
astdbdir => $WORK/var/lib
astkeydir => $WORK/var/lib
astdatadir => $WORK/var/lib
astagidir => $WORK/var/lib
astspooldir => $WORK/var/spool
astrundir => $WORK/run
astlogdir => $WORK/log
astsbindir => $(dirname "$ASTERISK")

[options]
verbose = 3
nocolor = yes
EOF

cat >"$WORK/etc/logger.conf" <<EOF
[general]
[logfiles]
messages.log => notice,warning,error,verbose(3)
EOF

cat >"$WORK/etc/modules.conf" <<EOF
[modules]
autoload = no
load => res_sorcery_config.so
load => res_sorcery_memory.so
load => res_sorcery_astdb.so
load => res_pjproject.so
load => res_rtp_asterisk.so
load => res_pjsip.so
load => res_pjsip_session.so
load => res_pjsip_pubsub.so
load => res_pjsip_sdp_rtp.so
load => res_pjsip_endpoint_identifier_ip.so
load => chan_pjsip.so
load => codec_ulaw.so
load => codec_alaw.so
load => pbx_config.so
load => app_system.so
load => res_clioriginate.so
load => app_softmodem.so
EOF

cat >"$WORK/etc/rtp.conf" <<EOF
[general]
rtpstart = 20000
rtpend = 20400
EOF

# One endpoint for datamodem in both directions: calls from it land in
# modem-in, and calls to it go to the answering datamodem's port. G.711 only,
# DTMF out of band, no fax detection: nothing that listens to the audio.
cat >"$WORK/etc/pjsip.conf" <<EOF
[transport-udp]
type = transport
protocol = udp
bind = 127.0.0.1:$SIP_PORT

[dm]
type = endpoint
context = modem-in
disallow = all
allow = ulaw
aors = dm
dtmf_mode = rfc4733
direct_media = no
fax_detect = no
; datamodem's SDP may name another of this machine's addresses, which a
; socket on loopback cannot send to: answer RTP where it comes from.
rtp_symmetric = yes
force_rport = yes

[dm]
type = aor
contact = sip:dm@127.0.0.1:$ANS_PORT

[dm]
type = identify
endpoint = dm
match = 127.0.0.1
EOF

# What each call ended with, for the checks below: the h extension runs
# after the Softmodem whichever end hung up.
RESULT_LINE='${SOFTMODEM_RESULT} ${SOFTMODEM_MODULATION} ${SOFTMODEM_RATE} ${SOFTMODEM_TX_RATE} ${SOFTMODEM_PROTOCOL} ${SOFTMODEM_BYTES_SENT} ${SOFTMODEM_BYTES_RECEIVED}'
cat >"$WORK/etc/extensions.conf" <<EOF
; Set per call from the CLI, which will not set an empty value: "l", the
; default bit order, stands in for no options.
[globals]
IN_PORT = 1
IN_OPTS = l
IN_RESULTS = /dev/null
OUT_PORT = 1
OUT_OPTS = l
OUT_RESULTS = /dev/null

[modem-in]
exten => modem,1,Softmodem(127.0.0.1,\${IN_PORT},\${IN_OPTS})
 same => n,Hangup()
exten => h,1,System(echo "$RESULT_LINE" >> \${IN_RESULTS})

[modem-out]
exten => go,1,Softmodem(127.0.0.1,\${OUT_PORT},\${OUT_OPTS})
 same => n,Hangup()
exten => h,1,System(echo "$RESULT_LINE" >> \${OUT_RESULTS})
EOF

ast() { "$ASTERISK" -C "$WORK/etc/asterisk.conf" -rx "$*" 2>/dev/null; }

"$ASTERISK" -C "$WORK/etc/asterisk.conf" -f -n -q >"$WORK/log/asterisk.out" 2>&1 &
AST_PID=$!
for _ in $(seq 1 100); do
    ast "core waitfullybooted" >/dev/null 2>&1 && break
    sleep 0.2
done
# Fully booted can be reported a moment before the module list answers for
# it, so a single look sometimes missed a module that was loading fine.
loaded=0
for _ in $(seq 1 50); do
    ast "module show like softmodem" 2>/dev/null | grep -q app_softmodem && { loaded=1; break; }
    sleep 0.2
done
if [ $loaded -eq 0 ]; then
    echo "Asterisk did not load app_softmodem.so:" >&2
    tail -30 "$WORK/log/messages.log" "$WORK/log/asterisk.out" >&2
    exit 1
fi
echo "Asterisk $("$ASTERISK" -V | sed 's/Asterisk //'), $(basename "$DATAMODEM") $("$DATAMODEM" --version 2>&1 | head -1 | awk '{print $2}'), work in $WORK"

# ---------------------------------------------------------------- helpers

# Waits for a process, for at most $2 seconds; kills it after that.
wait_for() {
    local pid=$1 secs=$2
    for _ in $(seq 1 $((secs * 5))); do
        kill -0 "$pid" 2>/dev/null || { wait "$pid" 2>/dev/null; return 0; }
        sleep 0.2
    done
    kill "$pid" 2>/dev/null
    wait "$pid" 2>/dev/null
    return 1
}

# Sets SERVER_PID and SERVER_PORT.
start_server() { # mode bytes name
    rm -f "${WORK:?}/${3:?}.port"
    python3 "$HERE/tcpserver.py" "$1" "$WORK/$3.port" "$WORK/$3.server.json" "$2" 170 \
        >"$WORK/$3.server.out" 2>&1 &
    SERVER_PID=$!
    for _ in $(seq 1 50); do [ -s "$WORK/$3.port" ] && break; sleep 0.1; done
    SERVER_PORT=$(cat "$WORK/$3.port")
}

# Checks what Asterisk recorded: the call ended because the server hung up,
# and ran one of the expected modulations.
check_result() { # name expected-modulations
    local line mod
    line=$(tail -1 "$WORK/$1.result" 2>/dev/null)
    if [ -z "$line" ]; then
        echo "no SOFTMODEM_RESULT recorded"
        return 1
    fi
    set -- "$1" "$2" $line
    mod=$4
    if [ "$3" != "SERVERCLOSED" ]; then
        echo "ended $3, not SERVERCLOSED ($line)"
        return 1
    fi
    if ! printf '%s\n' "$2" | tr '|' '\n' | grep -qx "$mod"; then
        echo "ran $mod, expected $2 ($line)"
        return 1
    fi
    echo "$mod $5/$6 $7, $8 bytes sent and $9 received"
}

passed=0
failed=0
report() { # name ok detail
    if [ "$2" -eq 0 ]; then
        passed=$((passed + 1))
        echo "ok - $1: $3"
    else
        failed=$((failed + 1))
        echo "not ok - $1: $3"
        echo "  (logs: $WORK/$1.*, $WORK/log/messages.log)"
    fi
}

# --------------------------------------------------------------- the calls
#
# Each kind of call is started and then finished, so that one of each can be
# in progress at once: the state of each is in IN_* and OUT_* globals.

# start_inbound NAME DATAMODEM-ARGS SOFTMODEM-OPTS BYTES
start_inbound() {
    local name=$1 dmargs=$2 opts=$3 bytes=$4
    start_server banner "$bytes" "$name"
    IN_SERVER=$SERVER_PID
    ast "dialplan set global IN_PORT $SERVER_PORT" >/dev/null
    ast "dialplan set global IN_OPTS ${opts:-l}" >/dev/null
    ast "dialplan set global IN_RESULTS $WORK/$name.result" >/dev/null
    IN_TOKEN="hello-from-datamodem-$name"

    mkfifo "$WORK/$name.stdin"
    # shellcheck disable=SC2086
    "$DATAMODEM" "sip:modem@127.0.0.1:$SIP_PORT" --server 127.0.0.1 --username dm --no-register \
        --bind-addr 127.0.0.1 --public-addr 127.0.0.1 \
        --local-port "$DIAL_PORT" --rtp-port 4400 $dmargs --max-call 160 \
        --log-file "$WORK/$name.dial.log" <"$WORK/$name.stdin" >"$WORK/$name.dial.out" 2>&1 &
    IN_DIAL=$!
    { printf '%s\r\n' "$IN_TOKEN"; sleep 170; } >"$WORK/$name.stdin" &
    IN_WRITER=$!
}

# finish_inbound NAME BYTES EXPECTED-MODULATIONS
finish_inbound() {
    local name=$1 bytes=$2 expect=$3 detail ok=0
    wait_for "$IN_DIAL" 170 || { ok=1; detail="datamodem was still connected after 170s"; }
    kill "$IN_WRITER" 2>/dev/null
    wait "$IN_WRITER" 2>/dev/null
    wait_for "$IN_SERVER" 10

    if [ $ok -eq 0 ]; then
        detail=$(python3 - "$WORK/$name.server.json" "$WORK/$name.dial.out" "$bytes" "$IN_TOKEN" <<'EOF'
import json, sys
try:
    res = json.load(open(sys.argv[1]))
except (OSError, ValueError):
    res = {"detail": "the server recorded nothing"}
out = open(sys.argv[2], "rb").read()
n, token = int(sys.argv[3]), sys.argv[4].encode()
block = bytearray()
i = 0
while len(block) < n:
    block += b"line %05d the quick brown fox jumps over the lazy dog\r\n" % i
    i += 1
block = bytes(block[:n])
problems = []
if not res.get("ok"):
    problems.append("server: " + res.get("detail", "?"))
elif res.get("line") != token.decode():
    problems.append("the server heard %r" % res.get("line"))
for what, want in (("the banner", b"SOFTMODEM TEST SERVER\r\n" + block + b"END OF BLOCK\r\n"),
                   ("the echo", b"ECHO " + token + b"\r\n"), ("GOODBYE", b"GOODBYE\r\n")):
    if want not in out:
        problems.append("datamodem did not receive %s intact" % what)
print("; ".join(problems) if problems else "")
EOF
)
        [ -n "$detail" ] && ok=1
    fi
    if [ $ok -eq 0 ]; then
        detail=$(check_result "$name" "$expect") || ok=1
    fi
    rm -f "${WORK:?}/${name:?}.stdin"
    report "$name" $ok "$detail"
}

# start_outbound NAME DATAMODEM-ARGS SOFTMODEM-OPTS BYTES
start_outbound() {
    local name=$1 dmargs=$2 opts=$3 bytes=$4
    start_server echo "$bytes" "$name"
    OUT_SERVER=$SERVER_PID
    ast "dialplan set global OUT_PORT $SERVER_PORT" >/dev/null
    ast "dialplan set global OUT_OPTS ${opts:-l}" >/dev/null
    ast "dialplan set global OUT_RESULTS $WORK/$name.result" >/dev/null

    # shellcheck disable=SC2086
    "$DATAMODEM" answer --server 127.0.0.1 --username dm --no-register \
        --bind-addr 127.0.0.1 --public-addr 127.0.0.1 \
        --local-port "$ANS_PORT" --rtp-port 4300 $dmargs --calls 1 --exec cat --max-call 160 \
        --log-file "$WORK/$name.answer.log" </dev/null >"$WORK/$name.answer.out" 2>&1 &
    OUT_ANSWER=$!
    for _ in $(seq 1 100); do
        grep -q "waiting for an inbound call" "$WORK/$name.answer.log" 2>/dev/null && break
        sleep 0.1
    done
    ast "channel originate PJSIP/dm extension go@modem-out" >/dev/null
}

# finish_outbound NAME BYTES EXPECTED-MODULATIONS
finish_outbound() {
    local name=$1 bytes=$2 expect=$3 detail ok=0
    wait_for "$OUT_ANSWER" 170 || { ok=1; detail="datamodem was still connected after 170s"; }
    wait_for "$OUT_SERVER" 10

    if [ $ok -eq 0 ]; then
        detail=$(python3 -c 'import json,sys; r=json.load(open(sys.argv[1])); print("" if r.get("ok") else "server: " + r.get("detail","?"))' \
            "$WORK/$name.server.json" 2>/dev/null || echo "the server recorded nothing")
        [ -n "$detail" ] && ok=1
    fi
    if [ $ok -eq 0 ]; then
        detail="$(check_result "$name" "$expect")" || ok=1
        [ $ok -eq 0 ] && detail="$detail; $(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["detail"])' "$WORK/$name.server.json")"
    fi
    report "$name" $ok "$detail"
}

# inbound|outbound NAME DATAMODEM-ARGS SOFTMODEM-OPTS BYTES EXPECTED-MODULATIONS
inbound() {
    start_inbound "$1" "$2" "$3" "$4"
    finish_inbound "$1" "$4" "$5"
}

outbound() {
    start_outbound "$1" "$2" "$3" "$4"
    finish_outbound "$1" "$4" "$5"
}

# Two calls at once, one each way: two channels, two modems, two threads.
concurrent() { # NAME IN-DMARGS IN-BYTES IN-EXPECT OUT-DMARGS OUT-BYTES OUT-EXPECT
    start_outbound "$1-out" "$5" "" "$6"
    start_inbound "$1-in" "$2" "" "$3"
    finish_outbound "$1-out" "$6" "$7"
    finish_inbound "$1-in" "$3" "$4"
}

# Every modulation each way, the far end fixed to it. The amounts are about
# ten seconds of line time each.
TESTS=(
    "inbound  inbound-v34      --modulation=v34_--no-step-down      -        20000 v34"
    "inbound  inbound-v32bis   --modulation=v32bis_--no-step-down   -        10000 v32bis"
    "inbound  inbound-v32      --modulation=v32_--no-step-down      -        6000  v32|v32bis"
    "inbound  inbound-v22bis   --modulation=v22bis_--no-step-down   -        2000  v22bis"
    "inbound  inbound-v22      --modulation=v22_--no-step-down      -        1000  v22|v22bis"
    "inbound  inbound-v21      --modulation=v21_--no-step-down      -        300   v21"
    "inbound  inbound-bell103  --modulation=bell103_--no-step-down  -        300   bell103"
    "inbound  inbound-v23      --modulation=v23_--no-step-down      v(V23)   1000  v23"
    "outbound outbound-v34     --modulation=v34_--no-step-down      -        20000 v34"
    "outbound outbound-v32bis  --modulation=v32bis_--no-step-down   -        10000 v32bis"
    "outbound outbound-v32     --modulation=v32_--no-step-down      -        6000  v32|v32bis"
    "outbound outbound-v22bis  --modulation=v22bis_--no-step-down   -        2000  v22bis"
    "outbound outbound-v22     --modulation=v22_--no-step-down      -        1000  v22|v22bis"
    "outbound outbound-v21     --modulation=v21_--no-step-down      -        200   v21"
    "outbound outbound-bell103 --modulation=bell103_--no-step-down  -        200   bell103"
    "outbound outbound-v23     --modulation=v23_--no-step-down      v(V23)   40    v23"
    # Both ends stepping down, from different places; and no V.42 at the far end.
    "inbound  inbound-v34-to-softmodem-v22bis  -                    v(V22bis) 2000 v22bis"
    "outbound outbound-softmodem-v32bis-to-v22bis --modulation=v22bis_--no-step-down v(V32bis) 2000 v22bis"
    "inbound  inbound-v32bis-no-v42 --modulation=v32bis_--no-step-down_--v42=off - 10000 v32bis"
)

# Two calls at once.
CONCURRENT=(
    "concurrent-v32bis-in-v22bis-out --modulation=v32bis_--no-step-down 10000 v32bis --modulation=v22bis_--no-step-down 2000 v22bis"
)

selected() { # name [filters...]
    [ $# -le 1 ] && return 0
    local t
    for t in "${@:2}"; do [ "$t" = "$1" ] && return 0; done
    return 1
}

for t in "${TESTS[@]}"; do
    # shellcheck disable=SC2086
    set -- $t
    kind=$1 name=$2 dmargs=${3//_/ } opts=$4 bytes=$5 expect=$6
    [ "$dmargs" = "-" ] && dmargs=""
    [ "$opts" = "-" ] && opts=""
    dmargs=${dmargs//=/ }
    selected "$name" "${ARGS[@]+"${ARGS[@]}"}" || continue
    "$kind" "$name" "$dmargs" "$opts" "$bytes" "$expect"
done

for t in "${CONCURRENT[@]}"; do
    # shellcheck disable=SC2086
    set -- $t
    name=$1
    selected "$name" "${ARGS[@]+"${ARGS[@]}"}" || continue
    in_args=${2//_/ } out_args=${5//_/ }
    concurrent "$name" "${in_args//=/ }" "$3" "$4" "${out_args//=/ }" "$6" "$7"
done

echo "# $passed passed, $failed failed"
[ $((passed + failed)) -gt 0 ] && [ "$failed" -eq 0 ]
