#!/bin/sh
# Copies datamodem's modem engine into engine/, applies patches/*.patch and
# records which datamodem commit it came from.
#
#   tools/sync-datamodem.sh [path to a datamodem checkout] [git ref]
#
# The checkout defaults to ../datamodem. Given a ref - origin/main, a tag -
# the engine is taken from that commit rather than from whatever the
# checkout's working tree has in it.
#
# The engine is datamodem's data pump - its modulations, V.42/V.42bis and the
# patched spandsp sources they need - and nothing else: no SIP, no terminal.
# Everything here is taken from datamodem unchanged except for what patches/
# says, so that bringing in a newer datamodem is this script and a rebuild.
set -eu

here=$(cd "$(dirname "$0")/.." && pwd)
src=${1:-"$here/../datamodem"}
src=$(cd "$src" && pwd)
ref=${2:-}
dst="$here/engine"
repo=$src

if [ -n "$ref" ]; then
    commit=$(git -C "$repo" rev-parse --verify "$ref^{commit}")
    tmp=$(mktemp -d)
    trap 'rm -rf "$tmp"' EXIT
    git -C "$repo" archive "$commit" | tar -x -C "$tmp"
    src=$tmp
fi

if [ ! -f "$src/src/modem.c" ]; then
    echo "$src does not look like a datamodem checkout" >&2
    exit 1
fi

rm -rf "$dst"
mkdir -p "$dst/include/datamodem" "$dst/src" \
         "$dst/third_party/spandsp-v22bis/generated" "$dst/third_party/spandsp-v42"

for h in config.h log.h modem.h ring.h util.h v32.h v34.h; do
    cp "$src/include/datamodem/$h" "$dst/include/datamodem/"
done
for f in modem.c ring.c util.c v32.c v34.c v34_codec.c v34_codec.h v34_dsp.c v34_dsp.h \
         v34_info.c v34_info.h; do
    cp "$src/src/$f" "$dst/src/"
done
for f in COPYING.LGPL README.md config.h floating_fudge.h v22bis_rx.c v22bis_tx.c; do
    cp "$src/third_party/spandsp-v22bis/$f" "$dst/third_party/spandsp-v22bis/"
done
cp "$src"/third_party/spandsp-v22bis/generated/*.h "$dst/third_party/spandsp-v22bis/generated/"
for f in COPYING.LGPL README.md config.h v42.c; do
    cp "$src/third_party/spandsp-v42/$f" "$dst/third_party/spandsp-v42/"
done
cp "$src/LICENSE" "$dst/LICENSE"

for p in "$here"/patches/*.patch; do
    [ -f "$p" ] || continue
    echo "applying $(basename "$p")"
    patch -s -p1 -d "$dst" < "$p"
done

{
    if [ -n "$ref" ]; then
        echo "datamodem $commit ($ref)"
    else
        echo "datamodem $(git -C "$repo" rev-parse HEAD)"
        if [ -n "$(git -C "$repo" status --porcelain -- src include third_party)" ]; then
            echo "with uncommitted changes"
        fi
    fi
    echo "synced $(date -u +%Y-%m-%dT%H:%M:%SZ) by tools/sync-datamodem.sh; patches: $(cd "$here/patches" 2>/dev/null && ls ./*.patch 2>/dev/null | tr '\n' ' ')"
} > "$dst/UPSTREAM"
echo "engine/ is now $(head -1 "$dst/UPSTREAM")"
