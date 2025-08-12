/*
 * Copyright (C) 2025 Benedikt Heinz <Zn000h AT gmail.com>
 *
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This code is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this code.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libbladeRF.h>
#include <sys/mman.h>
#include <linux/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <stdint.h>
#include <limits.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>

#ifndef MIN
#define MIN(a,b)    ((a)<=(b)?(a):(b))
#endif

#define DEFAULT_FREQ         866450000  // 866.45 MHz
#define DEFAULT_SAMPLERATE   8000000    // 8 MS/s
#define DEFAULT_BANDWIDTH   (7000000)
#define NUM_BUFFERS         64
#define NUM_SAMPLES			(127*2048)
#define BUFFER_SIZE  		(NUM_SAMPLES * sizeof(uint16_t) * 2)
#define NUM_TRANSFERS       16
#define TIMEOUT_MS			3500

static volatile sig_atomic_t stop_flag = 0;

static void handle_signal(int sig) {
    (void)sig;
    stop_flag = 1;
}

struct mf {
	int fd;
	void *map_base;
	size_t map_size;
};

static void close_file(struct mf *mf, size_t written) {
	assert(mf->fd >= 0);
	assert(mf->map_base && (mf->map_base != MAP_FAILED));
    // Truncate file to actual written data
    if (written > 0) {
        if (msync(mf->map_base, written, MS_SYNC) != 0)
            perror("msync");
    }
	if (ftruncate(mf->fd, written) != 0)
		perror("ftruncate (final)");
	munmap(mf->map_base, mf->map_size);
	close(mf->fd);
}

static int create_file(struct mf *mf, const char *fn, size_t max_size) {
	memset(mf, 0, sizeof(struct mf));
	mf->fd = -1;

	// File create + mmap
    int fd = open(fn, O_CREAT | O_EXCL | O_RDWR, S_IRUSR | S_IWUSR | S_IRGRP);
    if (fd < 0) {
        perror("open");
        return fd;
    }
    if (ftruncate(fd, max_size) != 0) {
        perror("ftruncate");
        close(fd);
        return -1;
    }
    void *map_base = mmap(NULL, max_size, PROT_WRITE, MAP_SHARED | MAP_NORESERVE, fd, 0);
    if (map_base == MAP_FAILED) {
        perror("mmap");
        close(fd);
        return -1;
    }
    mf->fd = fd;
    mf->map_base = map_base;
    mf->map_size = max_size;
	return 0;
}

static void usage(const char *argv0) {
    fprintf(stderr, "Usage: %s -f <filename> -s <max_filesize>M/G/T [-g <manual_gain>] [-l <logfile>]\n", argv0);
    fputs("          (filesize multiplier: M, G or T for Mega-/Giga-/Terabytes)\n", stderr);
}

static float autoscale_float(float val, char *suffix) {
	static const char mult[]=" kMG";
	const char *p=mult;
	for(;(val>=1000.0) && p[1];p++)
		val/=1000.0;
	*suffix = *p;
	return val;
}

static size_t parse_fsize(const char *arg) {
	int len = strlen(arg);
	int suffix = arg[len-1];
	size_t mult = 0;
	switch(suffix) {
		case 'M' : mult=1000UL * 1000UL; break;
		case 'G' : mult=1000UL * 1000UL * 1000UL; break;
		case 'T' : mult=1000UL * 1000UL * 1000UL * 1000UL; break;
	}
	return strtoull(arg, NULL, 10) * mult;
}

int main(int argc, char **argv) {
	struct bladerf_metadata meta = {.flags = BLADERF_META_FLAG_RX_NOW};
	struct bladerf *dev = NULL;
	struct mf mf;
	const char *fname = NULL, *log_fname = NULL;
	size_t written = 0, max_size = 0, written_last = 0;
	int manual_gain = INT_MIN, res, opt;
	struct timeval tv_now, tv_last = {0}, tv_next = {0}, tv_sec = {.tv_sec=1};
	FILE *logfile = NULL;
	char suffix;
	float fv;

    // Parse args
    while ((opt = getopt(argc, argv, "f:s:g:l:")) != -1) {
        switch(opt) {
            case 'f': fname = optarg; break;
            case 's': max_size = parse_fsize(optarg); break;
            case 'g': manual_gain = atoi(optarg); break;
            case 'l': log_fname = optarg; break;
            default: usage(argv[0]); return 1;
        }
    }
    if (!fname || !max_size) {
        usage(argv[0]);
        return 1;
    }

	if(log_fname) {
		logfile = fopen(log_fname, "wx");
		if(!logfile) {
			perror("logfile fopen");
			return -1;
		}
	}

	if(create_file(&mf, fname, max_size))
		return -1;

	// Setup signal handlers
	struct sigaction sa;
	sa.sa_handler = handle_signal;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = 0;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

    // Open bladeRF
    if ((res = bladerf_open(&dev, NULL)) != 0) {
        fprintf(stderr, "Failed to open bladeRF: %s\n", bladerf_strerror(res));
        return -1;
    }

    // Configure RX channel 0
    if ((res = bladerf_set_frequency(dev, BLADERF_CHANNEL_RX(0), DEFAULT_FREQ)) != 0 ||
        (res = bladerf_set_sample_rate(dev, BLADERF_CHANNEL_RX(0), DEFAULT_SAMPLERATE, NULL)) != 0 ||
        (res = bladerf_set_bandwidth(dev, BLADERF_CHANNEL_RX(0), DEFAULT_BANDWIDTH, NULL)) != 0) {
        fprintf(stderr, "Failed to configure bladeRF: %s\n", bladerf_strerror(res));
        goto cleanup;
    }

    // AGC or manual gain?
    res = bladerf_set_gain_mode(dev, BLADERF_CHANNEL_RX(0), (manual_gain == INT_MIN) ? BLADERF_GAIN_AUTOMATIC : BLADERF_GAIN_MGC);
    if (res) {
		fprintf(stderr, "Failed to set AGC: %s\n", bladerf_strerror(res));
        goto cleanup;
     }

	// set manual gain value
    if (manual_gain != INT_MIN) {
		res = bladerf_set_gain(dev, BLADERF_CHANNEL_RX(0), manual_gain);
		if(res) {
			fprintf(stderr, "Failed to set manual gain: %s\n", bladerf_strerror(res));
            goto cleanup;
		}
	}

	res = bladerf_sync_config(dev, BLADERF_RX_X1, BLADERF_FORMAT_SC16_Q11_META,
                                 NUM_BUFFERS, NUM_SAMPLES, NUM_TRANSFERS,
                                 TIMEOUT_MS);
    if (res != 0) {
        fprintf(stderr, "Failed to configure RX sync interface: %s\n",
                bladerf_strerror(res));
        return res;
    }

    // Enable RX
    if ((res = bladerf_enable_module(dev, BLADERF_CHANNEL_RX(0), true)) != 0) {
        fprintf(stderr, "Failed to enable RX: %s\n", bladerf_strerror(res));
        goto cleanup;
    }

    fprintf(stderr, "Receiving... Press Ctrl+C to abort.\n");

	size_t remaining = mf.map_size / 4;	// convert bytes to samples
	uint32_t *dst    = mf.map_base;		// 1 sample == 2 * 16 Bits
	int overrun = 0;

	while(!stop_flag && !overrun && remaining && !(res = bladerf_sync_rx(dev, dst, MIN(remaining,NUM_SAMPLES), &meta, TIMEOUT_MS))) {
		gettimeofday(&tv_now, NULL);

		remaining -= meta.actual_count;
		dst       += meta.actual_count;
		written   += meta.actual_count * 4;
		overrun    = meta.status & BLADERF_META_STATUS_OVERRUN;

		/* show stats */
		if(timercmp(&tv_now, &tv_next, >=)) {
			timeradd(&tv_now, &tv_sec, &tv_next);
			if(tv_last.tv_sec) {
				struct timeval tmp;
				timersub(&tv_now, &tv_last, &tmp);
				float delta_t = (float)tmp.tv_usec / 1000000.0;
				delta_t += tmp.tv_sec;
				float datarate = written - written_last;
				datarate /= delta_t;
				datarate = autoscale_float(datarate, &suffix);
				char suffix2;
				fv = autoscale_float(written, &suffix2);
				printf("\r~%5.1f %cB/s, total: %6.2f %cB", datarate, suffix, fv, suffix2);
				fflush(stdout);
				if(logfile) {
					fprintf(logfile, "%ld.%ld %zu\n", tv_now.tv_sec, tv_now.tv_usec, written>>2);
					fflush(logfile);
				}
			}
			tv_last = tv_now;
			written_last = written;
		} // show stats

		//printf("%x %d %d\n",meta.res,meta.actual_count,overrun);
	} // rx loop

    bladerf_enable_module(dev, BLADERF_CHANNEL_RX(0), false);

	printf("\r%40s\r","");

	if(overrun)
		fputs("OVERRUN OCCURRED!\n", stderr);

cleanup:
	close_file(&mf, written);
	if(logfile)
		fclose(logfile);
	bladerf_close(dev);

	fv = autoscale_float(written, &suffix);
	printf("wrote %.2f %cBytes (%zu Bytes)\n",fv,suffix,written);

    return (res == 0 || stop_flag) ? 0 : 1;
}
