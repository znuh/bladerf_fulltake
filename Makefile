CC = gcc
CFLAGS = -Wall -Wextra -O2
LDFLAGS = -lbladeRF

all: bladerf_rx

bladerf_rx: bladerf_rx.c
	$(CC) $(CFLAGS) -o bladerf_rx bladerf_rx.c $(LDFLAGS)

clean:
	rm -f bladerf_rx
