#!/bin/sh
# app_softmodem.so for older Linux - Sangoma Linux 7 under FreePBX 16, and
# the Debian 10 and 11 it also installs on - built in a CentOS 7 container,
# so that it needs nothing newer than glibc 2.17. spandsp is linked into the
# module and kept out of its symbol table, so it does not matter which
# spandsp, if any, the system has, nor which one res_fax_spandsp loaded.
#
#   docker run --rm -v "$PWD:/src" -w /src quay.io/pypa/manylinux2014_x86_64 \
#       tools/build-el7.sh ASTERISK-VERSION [OUTDIR]
#
# manylinux2014 is CentOS 7 with a current gcc and its yum repositories
# pointed at the CentOS vault. Runs make test first, with the same spandsp.
set -eu

version=$1
out=${2:-dist}
work=${WORK:-/tmp/el7}
spandsp_url=https://deb.debian.org/debian/pool/main/s/spandsp/spandsp_0.0.6+dfsg.orig.tar.xz
spandsp_sha256=3dcdc611b8a119f1f26540d05e6279c4c1e5cd576271f6d45df431359fc190f9
prefix=$work/spandsp

mkdir -p "$work"
echo "==> build dependencies"
yum install -y -q libtiff-devel libedit-devel libxml2-devel sqlite-devel libuuid-devel \
    ncurses-devel bzip2 patch wget file xz >/dev/null

echo "==> spandsp, static and position-independent"
curl -fsSL --retry 3 -o "$work/spandsp.tar.xz" "$spandsp_url"
echo "$spandsp_sha256  $work/spandsp.tar.xz" | sha256sum -c -
rm -rf "$work/spandsp-src"
mkdir -p "$work/spandsp-src"
tar -xJf "$work/spandsp.tar.xz" -C "$work/spandsp-src" --strip-components=1
(
    cd "$work/spandsp-src"
    ./configure --prefix="$prefix" --disable-shared --enable-static --with-pic CFLAGS="-O2 -fPIC" \
        >"$work/spandsp.log" 2>&1 || { tail -50 "$work/spandsp.log"; exit 1; }
    make >>"$work/spandsp.log" 2>&1 || { tail -50 "$work/spandsp.log"; exit 1; }
    make install >>"$work/spandsp.log" 2>&1
)

echo "==> Asterisk $version headers"
# CentOS 7's jansson is older than Asterisk wants.
ASTERISK_CONFIGURE_FLAGS=--with-jansson-bundled \
    tools/asterisk-headers.sh "$version" "$work/asterisk"

spandsp_cflags="-I$prefix/include"
spandsp_libs="$prefix/lib/libspandsp.a -Wl,--exclude-libs,ALL"

echo "==> make test"
make clean
make test SPANDSP_CFLAGS="$spandsp_cflags" SPANDSP_LIBS="$spandsp_libs"

echo "==> app_softmodem.so"
make ASTERISK_INCLUDE="$work/asterisk/asterisk-$version/include" \
    SPANDSP_CFLAGS="$spandsp_cflags" SPANDSP_LIBS="$spandsp_libs"

echo "==> checks"
file app_softmodem.so
newest=$(objdump -T app_softmodem.so | grep -oE 'GLIBC_[0-9.]+' | sed 's/GLIBC_//' | sort -t. -k1,1n -k2,2n -k3,3n | tail -1)
echo "newest glibc symbol version: $newest"
[ "$(printf '%s\n2.17\n' "$newest" | sort -t. -k1,1n -k2,2n -k3,3n | tail -1)" = 2.17 ] ||
    { echo "needs glibc $newest, newer than 2.17" >&2; exit 1; }
if readelf -d app_softmodem.so | grep NEEDED | grep -q spandsp; then
    echo "linked to a shared libspandsp" >&2; exit 1
fi
if nm -D --undefined-only app_softmodem.so | grep -q ' TIFF'; then
    echo "libtiff symbols left undefined" >&2; exit 1
fi
if nm -D --defined-only app_softmodem.so | grep -qE ' (fsk_|v42bis_|async_|span_)'; then
    echo "spandsp symbols exported" >&2; exit 1
fi
readelf -d app_softmodem.so | grep NEEDED

mkdir -p "$out"
cp app_softmodem.so README "$out/"
