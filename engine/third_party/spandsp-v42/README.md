# spandsp V.42, vendored and patched

`v42.c` from spandsp, compiled into datamodem so that it **overrides the one
in the installed libspandsp**. Unlike `../spandsp-v22bis`, which is simply a
newer copy of working code, this one carries a real patch. `v42.c.orig` is
the pristine file; `diff -u v42.c.orig v42.c` is the whole change.

## The patches

V.42's detection phase has the answering modem send an ADP: the character
`E`, then 8 to 16 one-bits, then `C`, then 8 to 16 one-bits, repeated.
spandsp counts those ones like this:

```c
case 2:
    s->neg.rxbits++;          /* counts the terminating zero as well */
    if (new_bit)
        break;
    if (s->neg.rxbits >= 8  &&  s->neg.rxbits <= 16)
```

The increment runs before the test, so the zero that *ends* the run of ones
is counted as one of them. The window is shifted by one: spandsp accepts 7
to 15 ones where V.42 specifies 8 to 16. A far end using 16 — the top of the
permitted range — is rejected, detection fails, and the call falls back to a
link with no error correction while the far end has already committed to
LAPM and is sending HDLC that we read as line noise.

Measured against spandsp's own documented pattern before the patch:

```
    6 ones between E and C: no
    7 ones between E and C: DETECTED
   ...
   15 ones between E and C: DETECTED
   16 ones between E and C: no        <- legal, and refused
```

### 2. Real equipment puts the whole idle after the pair

Instrumenting the detector against a real modem produced this:

```
detect gave up: 5324 bits seen, 0 good cycles;
                first char wrong 207 times, gap1 wrong 1 times (0..0),
                second char wrong 0 times, gap2 wrong 0 times
```

5324 bits carrying 208 characters is 25.6 bits each, so `E(10) + g1 +
C(10) + g2` comes to 51 bits a pair and the two gaps sum to 31. The one
time the detector did align on `E`, it measured the following gap as
**zero**. That far end sends `E` and `C` back to back and puts the whole
idle after the pair — where V.42, and spandsp, want 8 to 16 ones in each.
Rejected at the first gap, every single time.

So the accepted range is now simply "up to 64", with no lower bound. The
pattern is still `E` then `C` then idle, twice over, which is specific
enough; and a wrong guess is cheap, because LAPM establishment then fails
and datamodem falls back.

### 3. Resynchronisation latched onto data bits

The same log showed the first character failing 207 times — about once per
character received — and succeeding once. `case 0` is commented "look for
some ones" but accepted *any* zero, including the zeros inside a
character's data. Having misaligned once, it would misalign again on the
next character, and again, and never recover.

It now requires a real idle run before treating a zero as a start bit. The
second character is located by `case 3` at a known offset, so it does not
need an idle in front of it and is unaffected.

Together these make the detector accept both shapes from every starting
offset in the cycle:

```
textbook: E +8 ones+ C +8 ones          DETECTED from every offset
textbook: E +16 ones+ C +16 ones        DETECTED from every offset
the real far end: E C then 31 ones      DETECTED from every offset
```

### 4. Establishment sent XID and never SABME

The one that actually stopped real calls connecting. After detection
succeeds, `initiate_negotiation_expired()` called `lapm_config()`, which
transmits an **XID** — parameter negotiation — and nothing else. V.42 8.3
is explicit that an error-corrected connection is established by sending
SABME; 8.10 has XID initiated by the control function, not as a
precondition for connecting. A far end waiting for SABME never sees one,
so it never answers.

It then fails silently. While `configuring` is set, `t401_expired()` only
ever retransmits the XID, and the LAPM state is still `LAPM_IDLE` — which
matches no case in the retry-exhausted switch:

```c
switch (s->state) {
case LAPM_ESTABLISH:
case LAPM_RELEASE:   ... report_rx_status_change(SIG_STATUS_LINK_DISCONNECTED);
case LAPM_DATA:      ... }
```

so nothing is reported at all. From a real call it looks like this: four
`T.401 expired` a second apart, then silence, then the far end clears the
call.

```
29.330  v42 status 1 (LAPM_IDLE)      <- detection succeeded
29.491  Start negotiation
30.491  T.401 expired
31.490  T.401 expired
32.491  T.401 expired
33.490  T.401 expired
34.330  carrier dropped
35.461  BYE from the far end
```

Two spandsp ends never notice, because spandsp answers its own XID.

`lapm_connect()` is what was wanted — it sends the SABME, starts T401, and
sets the state to `LAPM_ESTABLISH` so a failure is at least reported. The
author had already commented the XID out of it.

**Revised by patch 8:** skipping XID entirely turned out to be wrong too —
see below. Establishment is now XID, then SABME; and SABME anyway if two
XIDs go unanswered, which still covers the far end that prompted this.

### 5. A failed establishment said nothing about why

`SIG_STATUS_LINK_DISCONNECTED` arrives with no reason attached, and there are
six quite different ways to reach it — from "we ran out of SABME retries"
through "the far end sent DISC" to "the far end answered our SABME with a
point-blank DM refusal". Everything spandsp knows about which it was is at
`SPAN_LOG_FLOW`. They are not the same problem and they do not call for the
same response, so `dm_v42_disconnect_cause` now carries a short description
out to whoever handles the status change.

The one that turns up in the field is **DM in answer to SABME**: the far end
completes detection, receives the SABME, and refuses. V.42 8.3.2.1 is clear
that this is legitimate and that the originator must report failure — so
spandsp is right to stop. But a far end entitled to say "not yet" is equally
entitled to be ready a moment later, and spandsp has no way to try again:
`lapm_connect()` is static, and `v42_restart()` goes all the way back to the
detection phase, which a far end that has already finished detection will not
run a second time. `dm_v42_reconnect()` repeats just the establishment step,
so `--v42-timeout` can mean what it says. See `v42_retry_establishment()` in
`src/modem.c` for the retry policy.

### 6. Both answerer detection patterns were treated as "yes"

Table 3/V.42 defines two Answerer Detection Patterns, and they mean opposite
things:

| Pattern | Meaning |
|---|---|
| `(E)` `(C)` | V.42 supported |
| `(E)` `(Null)` | No error-correcting protocol desired |

and 7.2.1.2 has the originator "take the appropriate action based upon the
ADP received (e.g. if EC is received, initiate LAPM)". Upstream matches both
in `negotiation_rx_bit()` case 3, advances the state machine identically for
each, and then initiates LAPM regardless — its own comment at the hit calls
the result "the V.42 supported pattern":

```c
if (s->calling_party  &&  s->neg.rxstream == 0x185)        /* (C)    */
    s->neg.rx_negotiation_step++;
else if (s->calling_party  &&  s->neg.rxstream == 0x001)   /* (Null) */
    s->neg.rx_negotiation_step++;
```

So a far end that politely declines error correction during detection is sent
a SABME anyway, and answers it with DM — the correct response to a request it
has already refused. From the outside detection appears to succeed,
establishment is refused every single time, and nothing says why. Which ADP
arrived is now recorded in `dm_v42_adp_no_ec`, and the `(Null)` form goes
straight to `LAPM_V42_UNSUPPORTED` without sending a SABME.

Two spandsp ends never send the `(Null)` form, so `dm_v42_answer_no_ec` (set
from `DATAMODEM_V42_ANSWER_NO_EC`) makes the answering end send it.

### 7. The wire was invisible

spandsp logs no LAPM frames at all, and `lapm_receive()` drops every frame
whose FCS fails without a word. Those two together make the most important
distinction in a failed negotiation impossible to see from the outside: a far
end that is answering correctly over a path that is mangling its answers
looks exactly like a far end that is saying nothing. Every frame is now
logged in both directions with its address, control field, P/F bit and first
octets, and frames that fail FCS are counted into `dm_v42_fcs_errors` and
logged as such rather than vanishing.

`lapm_receive()` also indexed `frame[1]` before checking that two octets had
arrived; a zero- or one-octet frame with a passing FCS read off the end of
the buffer. It now returns early.

### 8. XID is where V.42bis is negotiated, and spandsp ignored it

V.42bis is agreed in the XID exchange and nowhere else; without one,
compression is off. Upstream's `receive_xid()` parsed the far end's values
and then discarded them (the call to apply them is commented out), answered
with its own fixed values — compression in one direction, a 512 codeword
dictionary, 6-character strings — and compressed both ways regardless. With
patch 4 no XID was sent at all. Against a real modem that means either a DM
in answer to a SABME it was not expecting yet, or a link it believes is
uncompressed carrying compressed data.

Now:

- establishment sends XID (P bit set) carrying what `s->config` holds, which
  `src/modem.c` loads from `--v42bis`, `--v42bis-dict` and
  `--v42bis-max-string`; SABME follows the XID response, or follows two
  unanswered XIDs, in which case compression is off - unless the answer
  arrives late, before the UA, in which case it is taken: the far end has
  agreed and will compress whatever we decided in the meantime;
- T401 is per context, set by `dm_v42_set_t401()` from the round trip
  `src/modem.c` knows about (and, since patch 9, the line rate), never below
  V.42's 1 s. A fixed 1 s is shorter
  than some RTP round trips, and an XID answered at 1.1 s over a 772 ms path
  was given up on and then discarded;
- `receive_xid()` narrows `s->config` to the agreement — the directions both
  ends want, the smaller dictionary and string length — so a responder
  answers with the agreed values and an initiator records them;
  `dm_v42_xid_done()` says whether an exchange happened, and `src/modem.c`
  sets V.42bis up from the result when the link comes up;
- an XID response mirrors the command's P bit in F;
- the group length in the parser was a `uint16_t` checked with `< 0`, so a
  malformed parameter length walked off the end of the frame. It is an
  `int`.

`dm_v42_no_xid` (from `DATAMODEM_V42_NO_XID`) makes an end behave like the
far ends that know nothing of XID — never sending one, ignoring any received
— so the other end's fallback can be tested.

### 9. A one-way transfer was never acknowledged

Three faults in the data phase, which traffic in both directions hides
completely - acknowledgements ride on I-frames then - and which a download
exposes at once:

- `tx_information_rr_rnr_response()` answered every I-frame with an RR (or
  RNR) carrying F=1, polled or not. F now mirrors the command's P bit.
- `rx_supervisory_rsp_frame()` discarded, whole, any response with F=1 that
  arrived outside timer recovery - and since upstream's receiver sent nothing
  else, that was every acknowledgement it ever got from another spandsp, and
  possibly from far ends built on it. The F bit is now ignored there, as it
  means nothing, and the N(R) acknowledgement is taken.
- `lapm_hdlc_underflow()` started T401 for an I-frame only if `bit_timer`
  was zero. But T401 and T403 share that one timer, and T403 is running from
  the moment the link is up whenever T401 is not, so T401 never ran in the
  data phase at all. It now starts if what is running is anything other than
  T401 (8.4.8: T401 runs while an I-frame is unacknowledged).

Together they left the sender's window full of frames it would never hear
acknowledged, until T403 fired ten seconds later, polled, and sent everything
again - which the receiver, having had it all, rejected. A 100 KB one-way
transfer at 14400 had moved 59 KB in five minutes.

T401 actually running has a consequence for `src/modem.c`: a 128-octet
frame takes 3.5 s to send at 300 bps, so a T401 of 1 s expired before the
frame it was timing had left. It is now sized there to cover the round
trip, two frames at the line rate and some thinking time, and resized when
the rate changes.

The frame log also showed P/F from the wrong bit for I and S frames - bit 4
of the control octet, which is the U frame's P/F but part of N(S) or the
frame type in the others. It now reads the second control octet for those.

### 10. Being busy was a secret

`v42_set_local_busy_status()` only set a flag. The far end, told nothing,
went on sending; every I-frame that arrived was thrown away unanswered, and
it found out only by polling after T401 - and when the condition cleared,
nothing said so until its next poll, and the frames thrown away were
recovered only by timing out again. Now it sends an RNR on entering the busy
condition (8.4.7) and, on leaving it, an RR - or a REJ if anything was
discarded meanwhile (remembered per context, alongside the XID result), so
that it is sent again at once. `src/modem.c` sets the condition when its
receive queue is nearly full and clears it once it has drained.

### 11. Going back for a REJ forgot what had been sent

To answer a REJ, `reject_info()` winds V(S) back to V(A) and sends again from
there. But the far end can still acknowledge frames from beyond that point -
ones it did receive, the REJ having been for an earlier frame, or ones it
acknowledged before our going back reached it - and `ack_info()` checked
N(R) against the wound-back V(S), called that a protocol error, and sent
DISC. It takes a lot of frames in flight for the window to matter, which is
why V.32 never showed it: a V.34 retrain at 33 600 bit/s, with a round trip
of a couple of hundred milliseconds, did it every time - the data link came
back up after the modems had retrained and then threw itself away.

So the highest N(S) actually sent before going back is remembered (per
context, alongside the XID result), N(R) up to it is valid, and frames the
far end turns out to have had are released rather than sent a second time.
The remembered value is dropped once V(A) has passed it, or once sequence
numbers could have wrapped round to it.

### 12. A SABME during detection was not heard

While in detection the receiver listens for ODP or ADP and nothing else, so
a far end that sends SABME (or XID) instead is ignored until detection times
out and calls V.42 unsupported. V.42 allows an originator to skip detection,
and more to the point a far end that has already been through it will not
go through it again: when a retrain lands in the middle of the handshake the
two ends can come out of it at different stages, one still detecting, the
other establishing. So bits now go to the HDLC receiver during detection
too, and a good SABME or XID command ends detection and is answered as it
would have been after it. Anything else that arrives then - damaged frames
especially, since this is the receiver framing ODP and line noise - is
dropped without being counted as an FCS error.

### 13. The per-context state was never given back

What patches 8, 10 and 11 remember per context lives in a table of eight
slots beside spandsp's state, keyed by the context's address, and a slot was
taken on a context's first lookup and never released. After eight V.42 links
in one process every slot was taken for good: each later link was put in
slot 0, reset whenever a different link looked itself up, so two links at
once wiped each other's XID outcome - V.42bis went unused and T401 fell back
to 1 s. A context allocated where a freed one had been found that one's slot
as it stood, with its highest N(S) and its discarded-while-busy flag.

datamodem itself runs one link at a time, so it met only the second of
those; a program running the engine for many lines at once - an Asterisk
module does - met the first after its eighth call. `v42_free()` now gives the
slot back and `v42_init()` clears any left at its address.
`dm_v42_xid_slots_in_use()` reports how many are taken, for
`tests/v42_test.c`.

### The test hook

`dm_v42_refuse_sabme` makes the answering end refuse that many SABMEs with
DM, and `src/modem.c` sets it from `DATAMODEM_V42_REFUSE_SABME`. It exists
because two spandsp ends accept each other's SABME on the first attempt, so
without it the retry path above is unreachable from the test suite — and a
path that only ever runs against real equipment is a path that only ever
fails in the field. It is zero in any normal run.

```
DATAMODEM_V42_REFUSE_SABME=3 datamodem selftest --modulation v22bis \
    --v42 require --v42bis --v42-timeout 20
```

## The other V.42 defect is fixed from outside

spandsp hardcodes `tx_bit_rate = 28800` and derives every timer from it, so
at 2400 bps they run twelve times too long and at 300 bps ninety-six times.
That is corrected in `src/modem.c` rather than here, because the rate is not
known until the modem has trained and can change when V.22bis negotiates
down. See `v42_arm()`, which is called twice: once when the modem is built,
with the rate being offered, so the context is valid through training, and
again the moment the carrier comes up, with the rate that was actually
negotiated. The second call is the one that matters. A link that offered 2400
and settled at 1200 would otherwise run every LAPM timer at twice the speed
V.42 allows, and T401 would exhaust its retries while the far end was still
well within its rights to be thinking.

## Why overriding the library like this is safe

Nothing else in libspandsp references `v42_*` — checked across every `.c` in
the tree — so no library code can hold a `v42_state_t` that this file
allocated, or the reverse. Every header it needs is installed and unchanged.

Drop this directory and `DATAMODEM_VENDOR_V42` when a libspandsp ships with
the counting fixed.

## Licence

spandsp is LGPL-2.1-only. See `COPYING.LGPL`, and the note in
`../spandsp-v22bis/README.md` about static linking.
