CFLAGS ?= -O2 -std=gnu11 -Wall -Wno-unused-function -I. -I../../firmware/rp2350/src
all: sst_run
# z80.c is copied next to the host z80_mem.h: a quoted include is resolved relative to the including file first
sst_run: sst_run.c ../../firmware/rp2350/src/z80.c z80_mem.h ../../firmware/rp2350/src/z80.h
	cp ../../firmware/rp2350/src/z80.c z80_core.c
	$(CC) $(CFLAGS) -o $@ sst_run.c z80_core.c
	rm -f z80_core.c
clean:
	rm -f sst_run
