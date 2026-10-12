#!/usr/bin/env python3
"""The server a Softmodem connects to in tests/asterisk/run.sh.

Takes one connection, plays its part and writes a verdict as JSON.

  banner   (an inbound call: the far end dialled in, as to a BBS)
           sends a banner and BYTES of text, waits for a line from the far
           end, answers "ECHO <line>" and "GOODBYE", and closes.

  echo     (an outbound call: the far end runs cat)
           waits for the Softmodem's "CONNECTED", sends BYTES of binary,
           expects them back byte for byte, and closes.

  tcpserver.py MODE PORTFILE RESULTFILE BYTES [TIMEOUT]

PORTFILE is where the port it listens on is written, once it is listening.
"""
import json
import random
import socket
import sys
import time


def text_block(n):
    """n bytes of numbered lines, so a gap or a repeat shows where it is."""
    out = bytearray()
    i = 0
    while len(out) < n:
        out += b"line %05d the quick brown fox jumps over the lazy dog\r\n" % i
        i += 1
    return bytes(out[:n])


def recv_until(conn, pred, deadline):
    got = bytearray()
    while not pred(got):
        left = deadline - time.monotonic()
        if left <= 0:
            return bytes(got), False
        conn.settimeout(left)
        try:
            chunk = conn.recv(65536)
        except socket.timeout:
            return bytes(got), False
        if not chunk:
            return bytes(got), False
        got += chunk
    return bytes(got), True


def main():
    mode, portfile, resultfile, nbytes = sys.argv[1], sys.argv[2], sys.argv[3], int(sys.argv[4])
    timeout = float(sys.argv[5]) if len(sys.argv) > 5 else 150.0

    lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    lsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    lsock.bind(("127.0.0.1", 0))
    lsock.listen(1)
    with open(portfile, "w") as f:
        f.write("%d\n" % lsock.getsockname()[1])

    result = {"mode": mode, "ok": False, "detail": ""}
    lsock.settimeout(timeout)
    try:
        conn, _ = lsock.accept()
    except socket.timeout:
        result["detail"] = "the Softmodem never connected"
        json.dump(result, open(resultfile, "w"))
        return
    started = time.monotonic()
    deadline = started + timeout
    result["connected_after"] = 0.0

    try:
        if mode == "banner":
            conn.sendall(b"SOFTMODEM TEST SERVER\r\n" + text_block(nbytes) + b"END OF BLOCK\r\n")
            got, ok = recv_until(conn, lambda b: b"\r" in b or b"\n" in b, deadline)
            line = got.splitlines()[0].decode("latin-1") if got.splitlines() else ""
            result["line"] = line
            if not ok:
                result["detail"] = "no line from the far end (got %r)" % got[:80]
            else:
                conn.sendall(b"ECHO " + line.encode("latin-1") + b"\r\nGOODBYE\r\n")
                result["ok"] = True
                result["detail"] = "the far end said %r" % line
        elif mode == "echo":
            got, ok = recv_until(conn, lambda b: b"CONNECTED\r\n" in b, deadline)
            if not ok:
                result["detail"] = "no CONNECTED from a calling Softmodem (got %r)" % got[:80]
            else:
                rnd = random.Random(1234)
                payload = bytes(rnd.randrange(256) for _ in range(nbytes))
                sent_at = time.monotonic()
                conn.sendall(payload)
                back, ok = recv_until(conn, lambda b: len(b) >= nbytes, deadline)
                result["seconds"] = round(time.monotonic() - sent_at, 1)
                if back[:nbytes] == payload:
                    result["ok"] = True
                    result["detail"] = "%d bytes there and back in %.1fs" % (nbytes, result["seconds"])
                else:
                    diff = next((i for i in range(min(len(back), nbytes)) if back[i] != payload[i]),
                                min(len(back), nbytes))
                    result["detail"] = "%d of %d bytes came back, the first difference at %d" % (
                        len(back), nbytes, diff)
        else:
            result["detail"] = "unknown mode " + mode
    except OSError as e:
        result["detail"] = "connection failed: %s" % e
    finally:
        # Closing is what tells the Softmodem to hang up, once the line has
        # drained. Half-close and wait for it to go: closing with anything of
        # its unread would be a reset rather than a goodbye.
        json.dump(result, open(resultfile, "w"))
        try:
            conn.shutdown(socket.SHUT_WR)
            recv_until(conn, lambda b: False, deadline)
        except OSError:
            pass
        conn.close()


if __name__ == "__main__":
    main()
