# Host build of the hardware independent layers.
# No cross toolchain and no board required.

CC     = gcc
CFLAGS = -std=c11 -Wall -Wextra -O1 -g
CFLAGS += -I../lib/control -I../config
LDLIBS = -lm

all: run_drive run_framing

run_drive: build/test_drive
	@./build/test_drive

run_framing: build/test_framing
	@./build/test_framing

build/test_drive: test_drive.c ../lib/control/drive.c
	@mkdir -p build
	$(CC) $(CFLAGS) $^ $(LDLIBS) -o $@

build/test_framing: test_framing.c ../lib/control/framing.c ../lib/control/protocol.c
	@mkdir -p build
	$(CC) $(CFLAGS) $^ $(LDLIBS) -o $@

clean:
	rm -rf build

.PHONY: all run_drive run_framing clean
