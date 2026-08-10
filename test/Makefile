# Host build of the hardware independent control layer
# No cross toolchain and no board required

CC     = gcc
CFLAGS = -std=c11 -Wall -Wextra -O1 -g
CFLAGS += -I../lib/control -I../config
LDLIBS = -lm

SRCS = test_drive.c ../lib/control/drive.c

all: run

run: build/test_drive
	@./build/test_drive

build/test_drive: $(SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(SRCS) $(LDLIBS) -o $@

clean:
	rm -rf build

.PHONY: all run clean
