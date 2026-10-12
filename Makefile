# app_softmodem, built outside the Asterisk source tree against the headers
# an Asterisk install puts in place (make install-headers, or your
# distribution's asterisk-dev package).
#
#   make                         app_softmodem.so
#   make install                 into ASTERISK_MODULES_DIR
#   make test                    the test suite that needs no Asterisk
#   make test-asterisk           a real Asterisk, dialled by datamodem over SIP
#
# Variables:
#   ASTERISK_INCLUDE      where asterisk.h is            (default /usr/include)
#   ASTERISK_MODULES_DIR  where modules are installed    (default /usr/lib/asterisk/modules)
#   SPANDSP_CFLAGS / SPANDSP_LIBS                        (default from pkg-config)

ASTERISK_INCLUDE ?= /usr/include
ASTERISK_MODULES_DIR ?= /usr/lib/asterisk/modules
DESTDIR ?=

SPANDSP_CFLAGS ?= $(shell pkg-config --cflags spandsp libtiff-4 2>/dev/null || pkg-config --cflags spandsp 2>/dev/null)
SPANDSP_LIBS ?= $(shell pkg-config --libs spandsp 2>/dev/null || echo -lspandsp)

UNAME_S := $(shell uname -s)

CC ?= cc
OPT ?= -O2 -g
WARN = -Wall -Wextra -Wno-unused-parameter -Wshadow
BASE_CFLAGS = -std=gnu11 $(OPT) -fPIC -pthread -fno-strict-aliasing \
	-Iengine/include -I. $(SPANDSP_CFLAGS) \
	-DDATAMODEM_VENDORED_V22BIS=1 -DDATAMODEM_VENDORED_V42=1
ifneq ($(UNAME_S),Darwin)
BASE_CFLAGS += -D_GNU_SOURCE
endif

ifeq ($(UNAME_S),Darwin)
MODULE_LDFLAGS = -bundle -undefined dynamic_lookup
# Asterisk's compat.h redefines strlcpy and strlcat, which the macOS SDK has
# already made macros.
AST_CFLAGS = -Wno-macro-redefined
else
# -Bsymbolic: our V.22bis and V.42, not any libspandsp another module has
# already loaded, are the ones our own code calls.
MODULE_LDFLAGS = -shared -Wl,-Bsymbolic -Wl,--as-needed
endif
LIBS = $(SPANDSP_LIBS) -lm -pthread

BUILD = build

ENGINE_SRCS = engine/src/modem.c engine/src/ring.c engine/src/util.c engine/src/v32.c \
	engine/src/v34.c engine/src/v34_codec.c engine/src/v34_dsp.c engine/src/v34_info.c
V22BIS_SRCS = engine/third_party/spandsp-v22bis/v22bis_rx.c engine/third_party/spandsp-v22bis/v22bis_tx.c
V42_SRCS = engine/third_party/spandsp-v42/v42.c
CORE_SRCS = softmodem/bridge.c softmodem/log.c

ENGINE_OBJS = $(ENGINE_SRCS:%.c=$(BUILD)/%.o)
V22BIS_OBJS = $(V22BIS_SRCS:%.c=$(BUILD)/%.o)
V42_OBJS = $(V42_SRCS:%.c=$(BUILD)/%.o)
CORE_OBJS = $(CORE_SRCS:%.c=$(BUILD)/%.o)
LIB_OBJS = $(CORE_OBJS) $(ENGINE_OBJS) $(V22BIS_OBJS) $(V42_OBJS)

HEADERS = $(wildcard engine/include/datamodem/*.h engine/src/*.h softmodem/*.h)

.PHONY: all install clean test test-asterisk test-all

all: app_softmodem.so

# The engine is datamodem's, warnings and all; it is checked there.
$(ENGINE_OBJS): $(BUILD)/%.o: %.c $(HEADERS)
	@mkdir -p $(dir $@)
	$(CC) $(BASE_CFLAGS) -fvisibility=hidden -w -c $< -o $@

$(V22BIS_OBJS): $(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(BASE_CFLAGS) -fvisibility=hidden -w -DHAVE_CONFIG_H \
		-Iengine/third_party/spandsp-v22bis -Iengine/third_party/spandsp-v22bis/generated -c $< -o $@

$(V42_OBJS): $(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(BASE_CFLAGS) -fvisibility=hidden -w -DHAVE_CONFIG_H -Iengine/third_party/spandsp-v42 -c $< -o $@

$(CORE_OBJS): $(BUILD)/%.o: %.c $(HEADERS)
	@mkdir -p $(dir $@)
	$(CC) $(BASE_CFLAGS) -fvisibility=hidden $(WARN) -c $< -o $@

$(BUILD)/app_softmodem.o: app_softmodem.c $(HEADERS)
	@mkdir -p $(dir $@)
	@test -f $(ASTERISK_INCLUDE)/asterisk.h || \
		{ echo "asterisk.h is not in $(ASTERISK_INCLUDE): set ASTERISK_INCLUDE" >&2; exit 1; }
	$(CC) $(BASE_CFLAGS) $(WARN) $(AST_CFLAGS) -I$(ASTERISK_INCLUDE) -D_REENTRANT \
		-DAST_MODULE=\"app_softmodem\" -DAST_MODULE_SELF_SYM=__internal_app_softmodem_self \
		-c $< -o $@

app_softmodem.so: $(BUILD)/app_softmodem.o $(LIB_OBJS)
	$(CC) $(MODULE_LDFLAGS) -o $@ $^ $(LIBS)

install: app_softmodem.so
	install -d $(DESTDIR)$(ASTERISK_MODULES_DIR)
	install -m 755 app_softmodem.so $(DESTDIR)$(ASTERISK_MODULES_DIR)/

# ------------------------------------------------------------------- tests

$(BUILD)/tests/%.o: tests/%.c $(HEADERS)
	@mkdir -p $(dir $@)
	$(CC) $(BASE_CFLAGS) $(WARN) -c $< -o $@

$(BUILD)/softmodem-test: $(BUILD)/tests/softmodem_test.o $(LIB_OBJS)
	$(CC) -o $@ $^ $(LIBS)

test: $(BUILD)/softmodem-test
	$(BUILD)/softmodem-test $(TESTS)

test-asterisk: app_softmodem.so
	tests/asterisk/run.sh

test-all: test test-asterisk

clean:
	rm -rf $(BUILD) app_softmodem.so
