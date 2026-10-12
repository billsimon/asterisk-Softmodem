# spandsp V.22bis, vendored

These are spandsp's two V.22bis source files, compiled directly into
datamodem so that they **override the ones in the installed libspandsp**.
Everything else — the FSK modems, V.42, V.42bis, the DSP helpers these two
files call — still comes from the shared library.

## Why

The V.22bis in the libspandsp that Homebrew installs does not carry data. It
completes the training handshake, reports a connection, and then delivers a
constant `0x55` for the rest of the call, because its receive equaliser
diverges to NaN about a second *before* training claims success. That is
measurable in a few lines of code against the stock library, and it matches
the symptoms users of other spandsp-based softmodems have reported for years
(a fixed repeating byte, forever).

It turns out not to be a defect in V.22bis as written. Homebrew builds the
`spandsp-0.0.6` release tarball from 2012. The sources here come from a later
upstream snapshot — still calling itself 0.0.6, but with files dated into
mid-2014 — and its V.22bis works: both directions, byte for byte, at 1200 and
at 2400, with the rate negotiation reaching 2400. No patching was needed. The
fix was using the newer code.

So most of this directory is not a patch set. It is the smallest possible
slice of a newer spandsp, taken because the version the platform ships
predates the work that made V.22bis usable.

## The first patch: come back at the rate we settled on

`v22bis_rx.c` carries three local changes, marked `datamodem:` in the
source; this is the first. On carrier loss upstream does:

```c
v22bis_restart(s, s->bit_rate);
```

`s->bit_rate` is the rate we *offered*, not the one the two ends agreed. A
call that offers 2400 and settles at 1200 therefore comes back from every
dropout offering 2400 again, sending the S1 rate-negotiation pattern at a far
end that is sitting in 1200 data mode and has no intention of renegotiating.
The two then disagree about how to demodulate each other, which reads as
noise, which triggers a retrain, which drops the carrier — and round it goes
until one end clears the call.

On a phone line this is invisible, because carrier drops essentially do not
happen. Over RTP a jitter buffer underrun is routine, so the loop runs on
every call that negotiates down. The patch restarts at
`s->negotiated_bit_rate` instead — but only when the receiver had reached
`NORMAL_OPERATION`, so that a dropout *during* training cannot silently
demote a link that never got to try 2400.

This one cannot be fixed by taking a newer spandsp: the line is unchanged
upstream. Note also that a spandsp-to-spandsp test will not show it, because
both ends restart together and renegotiate in step. It only appears against
equipment that holds its rate — which is to say, against real modems.

## Two more: an answerer that hears AA

V.32bis Annex A has a V.32 automode caller send AA, a pure 1800 Hz tone,
through the answering modem's answer tone and after it, and it relies on a
V.22bis answerer ignoring it: AA is out of the answerer's band, and real
V.22bis modems do. This one did not, in two ways, both marked `datamodem:`
in `v22bis_rx.c`. Enough AA leaks through the 1200 Hz receive filter to sit
right at the -43 dBm0 carrier threshold.

- **Each flicker of that carrier restarted the transmitter too.** On carrier
  loss upstream calls `v22bis_restart()`, which puts an answerer back to its
  75 ms of silence before USB1 - so with the carrier coming and going, USB1
  never went out at all, and the caller, which says nothing in V.22bis terms
  until it hears USB1, waited for ever. Before the handshake has begun (the
  transmitter still silent or still sending USB1) only the receiver is
  restarted now.
- **Any 270 ms of signal passed for SB1.** Upstream's answerer counts 270 ms
  in the low channel and commits to 1200 bps without looking at what it
  decoded. It now counts the symbols that do not descramble to ones - spare
  in `rx.training_error`, otherwise unused - and if more than a quarter of
  them did not, it was not SB1: the receiver starts again and USB1 carries
  on.

datamodem's own answering modem steps away from V.22bis when it hears AA, so
it is mostly the `--no-step-down` V.22bis answerer that needed these - and
the selftest's stand-in for an older V.22bis modem, which has to behave like
one.

## What is here

| | |
|---|---|
| `v22bis_tx.c` | verbatim from the later 0.0.6 snapshot |
| `v22bis_rx.c` | the same, plus the three patches above |
| `generated/*_rrc.h` | root-raised-cosine filter tables |
| `floating_fudge.h` | verbatim; needed by the two sources |
| `config.h` | ours — see the comment in it |
| `COPYING.LGPL` | spandsp's licence |

The filter tables are normally generated during spandsp's own build by
`make_modem_filter`. They are checked in here because we do not run that
build. To regenerate them from a spandsp source tree:

```
cc -O2 -DHAVE_CONFIG_H -I<dir with config.h> -Isrc \
   -o make_modem_filter src/make_modem_filter.c src/filter_tools.c -lm
./make_modem_filter -m V.22bis1200 -r    > v22bis_rx_1200_floating_rrc.h
./make_modem_filter -m V.22bis2400 -r    > v22bis_rx_2400_floating_rrc.h
./make_modem_filter -m V.22bis1200 -i -r > v22bis_rx_1200_fixed_rrc.h
./make_modem_filter -m V.22bis2400 -i -r > v22bis_rx_2400_fixed_rrc.h
./make_modem_filter -m V.22bis -t        > v22bis_tx_floating_rrc.h
./make_modem_filter -m V.22bis -i -t     > v22bis_tx_fixed_rrc.h
```

## Why overriding the library like this is safe

- Nothing else in libspandsp references `v22bis_*`. Checked across every `.c`
  in the tree: the fax modems use V.17/V.27ter/V.29, and no other module
  touches a `v22bis_state_t`. So no library code can end up holding a
  structure that these files allocated, or the reverse.
- Every spandsp header these two files depend on — `private/v22bis.h`,
  `logging.h`, `power_meter.h`, `complex.h`, `async.h`, `dds.h`,
  `vector_float.h` and the rest — is byte-identical between the installed
  headers and the snapshot these came from. The embedded structures and the
  helper functions they call have the same layout and the same ABI.
- The link order puts these objects in the executable, so they resolve ahead
  of the shared library.

If a future libspandsp fixes V.22bis, this directory still cannot go away
entirely: none of the patches above are upstream. Rebase it onto the
newer `v22bis_rx.c` and keep `DATAMODEM_VENDOR_V22BIS` on, or drop the whole
directory and accept that calls which negotiate down to 1200 will retrain
until they die.

## The test hook

`dm_v22bis_deaf_to_s1` (set from `DATAMODEM_V22BIS_DEAF_TO_S1`) makes the
calling modem ignore the answerer's S1, the way losing 100 ms of RTP does.
The answerer still hears our S1 and commits to 2400 while we commit to 1200,
which is the one way these two ends can end up demodulating each other at
different rates — and the thing that actually goes wrong on real calls. Two
spandsp ends in a loopback always agree, so without the hook the recovery
path in `check_rate_agreement()` is unreachable from the test suite.

```
DATAMODEM_V22BIS_DEAF_TO_S1=1 datamodem selftest --modulation v22bis --v42 off
```

## Licence

spandsp is LGPL-2.1-only, and these files are compiled into the datamodem
executable rather than dynamically linked. That is fine for in-house use, and
if datamodem is ever distributed as a binary the LGPL's relinking obligation
has to be honoured — which the presence of this directory, unmodified and
with its licence alongside, is most of the way towards satisfying.
