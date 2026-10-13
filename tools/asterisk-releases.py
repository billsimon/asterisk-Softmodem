#!/usr/bin/env python3
"""The newest released versions of each Asterisk major version, as JSON.

  tools/asterisk-releases.py [--count N] MAJOR...

Reads the release directory on downloads.asterisk.org and prints a JSON list
of version strings - the N newest final releases (no -rc, no -patch) of each
MAJOR, newest first within each - for a CI build matrix. For example:

  tools/asterisk-releases.py --count 3 20 22 23
  ["20.21.0", "20.20.0", "20.19.0", "22.11.0", ...]
"""
import argparse
import json
import re
import sys
import urllib.request

RELEASES = "https://downloads.asterisk.org/pub/telephony/asterisk/releases/"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=3)
    ap.add_argument("majors", nargs="+", type=int)
    args = ap.parse_args()

    with urllib.request.urlopen(RELEASES, timeout=60) as r:
        listing = r.read().decode("utf-8", "replace")
    found = {tuple(int(p) for p in m.groups())
             for m in re.finditer(r'href="asterisk-(\d+)\.(\d+)\.(\d+)\.tar\.gz"', listing)}

    out = []
    for major in args.majors:
        mine = sorted((v for v in found if v[0] == major), reverse=True)[:args.count]
        if len(mine) < args.count:
            sys.exit("only %d releases of Asterisk %d found at %s" % (len(mine), major, RELEASES))
        out += [".".join(str(p) for p in v) for v in mine]
    print(json.dumps(out))


if __name__ == "__main__":
    main()
