#!/bin/sh
# Headers to build app_softmodem against a given Asterisk release, without
# building Asterisk: downloads the release, checks it against its published
# SHA-256, configures it and generates the two headers configure and
# menuselect leave behind (autoconfig.h, buildopts.h).
#
#   tools/asterisk-headers.sh VERSION DIR
#
# DIR/asterisk-VERSION/include is then what ASTERISK_INCLUDE wants:
#
#   make ASTERISK_INCLUDE=DIR/asterisk-VERSION/include
#
# configure needs what Asterisk always does - libedit, jansson, libxml2,
# sqlite3, libuuid. Menuselect's choices are its defaults, so the module
# loads into an Asterisk built with default compile-time options; one built
# with others (DEBUG_THREADS, say) declines it, and wants it built against
# its own headers instead.
set -eu

version=$1
dir=$2
url=https://downloads.asterisk.org/pub/telephony/asterisk/releases
tarball=asterisk-$version.tar.gz

mkdir -p "$dir"
cd "$dir"
curl -fsSL --retry 3 -o "$tarball" "$url/$tarball"
curl -fsSL --retry 3 -o "asterisk-$version.sha256" "$url/asterisk-$version.sha256"
if command -v sha256sum >/dev/null; then
    sha256sum -c "asterisk-$version.sha256"
else
    shasum -a 256 -c "asterisk-$version.sha256"
fi
rm -rf "asterisk-$version"
tar -xzf "$tarball"
cd "asterisk-$version"

# pjproject only matters to the modules that use it; leaving it out spares
# configure downloading and building it.
./configure --without-pjproject-bundled --without-pjproject ${ASTERISK_CONFIGURE_FLAGS:-} \
    >configure.out 2>&1 || { tail -40 configure.out >&2; exit 1; }
make include/asterisk/buildopts.h >make-headers.out 2>&1 || { tail -40 make-headers.out >&2; exit 1; }
test -f include/asterisk/autoconfig.h
test -f include/asterisk/buildopts.h
echo "$PWD/include"
