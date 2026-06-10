/*
 * Standalone C profiler for the devmem (mmap) access pattern used by the
 * fast readout path. This is the apples-to-apples partner of
 * devmem_profile.py.
 *
 * A "sweep step" is:
 *   - control-buffer WRITE: copy n_tones*CONTROL_N_WORDS 32-bit words into the
 *     mapped AXI-lite region (the C equivalent of
 *         axil_mm[start:start+length] = v.tobytes()
 *     in firmware_lib.write_control_buffer_data_fast).
 *   - accumulator READ: read n_read_chans*2 32-bit words back out (the C
 *     equivalent of
 *         raw = axil_mm[base:base+nbytes]; np.frombuffer(raw, '<i4')
 *     in firmware_lib.read_accumulated_data_fast).
 *
 * The mapping mirrors casperfpga LocalMemTransport:
 *     AXIL_OFFSET = 0xA0000000, MAP_SIZE = 32 MiB, /dev/mem, MAP_SHARED.
 * By default it maps an ordinary backing file (offset 0) so it runs anywhere;
 * pass --dev /dev/mem (root + real board) for actual hardware.
 *
 * The register window is treated as volatile uint32_t* so reads/writes are
 * real MMIO-style accesses and not elided by the optimiser.
 *
 * Build:   cc -O2 -o devmem_profile devmem_profile.c
 *   (or use the Makefile in this directory)
 */

#define _GNU_SOURCE
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define AXIL_OFFSET 0xA0000000UL
#define MAP_SIZE    (32UL * 1024 * 1024)   /* 32 MiB */
#define CONTROL_N_WORDS 4                  /* words per tone */

/* byte offsets within the mapped window for the two buffers */
#define WRITE_ADDR 0x000000UL
#define READ_ADDR  0x100000UL

static inline double now_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static int cmp_double(const void *a, const void *b) {
    double da = *(const double *)a, db = *(const double *)b;
    return (da > db) - (da < db);
}

typedef struct {
    const char *name;
    double mean_us, median_us, min_us, p99_us, max_us, mb_per_s;
    long n;
    size_t payload_bytes;
} summary_t;

static summary_t summarize(const char *name, double *times, long n,
                           size_t payload_bytes) {
    /* times are in seconds; copy+sort for percentiles */
    double *s = malloc(n * sizeof(double));
    memcpy(s, times, n * sizeof(double));
    qsort(s, n, sizeof(double), cmp_double);

    double sum = 0.0;
    for (long i = 0; i < n; i++) sum += s[i];
    double mean = sum / n;
    double median = s[n / 2];
    long p99i = (long)(0.99 * n);
    if (p99i >= n) p99i = n - 1;
    double p99 = s[p99i];
    double mn = s[0], mx = s[n - 1];
    double thru = mean > 0 ? (double)payload_bytes / mean / 1e6 : 0.0;

    printf("  %-14s mean=%8.2fus  median=%8.2fus  min=%7.2fus  "
           "p99=%8.2fus  max=%8.2fus  (%zuB, %7.1f MB/s)\n",
           name, mean * 1e6, median * 1e6, mn * 1e6, p99 * 1e6, mx * 1e6,
           payload_bytes, thru);

    summary_t out = { name, mean * 1e6, median * 1e6, mn * 1e6,
                      p99 * 1e6, mx * 1e6, thru, n, payload_bytes };
    free(s);
    return out;
}

int main(int argc, char **argv) {
    const char *dev = "./devmem_backing.bin";
    const char *out = "devmem_profile_c.txt";
    long n_steps = 10000;
    long n_tones = 1000;
    long n_read_chans = 1000;
    long warmup = 1000;
    unsigned long write_addr = WRITE_ADDR;
    unsigned long read_addr = READ_ADDR;
    int bulk = 0;   /* 0 = per-word volatile (default); 1 = single memcpy */

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--dev") && i + 1 < argc)        dev = argv[++i];
        else if (!strcmp(argv[i], "--out") && i + 1 < argc)   out = argv[++i];
        else if (!strcmp(argv[i], "--steps") && i + 1 < argc) n_steps = atol(argv[++i]);
        else if (!strcmp(argv[i], "--tones") && i + 1 < argc) n_tones = atol(argv[++i]);
        else if (!strcmp(argv[i], "--read-chans") && i + 1 < argc) n_read_chans = atol(argv[++i]);
        else if (!strcmp(argv[i], "--warmup") && i + 1 < argc) warmup = atol(argv[++i]);
        else if (!strcmp(argv[i], "--write-addr") && i + 1 < argc) write_addr = strtoul(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--read-addr") && i + 1 < argc)  read_addr = strtoul(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--bulk"))                  bulk = 1;
        else {
            fprintf(stderr, "usage: %s [--dev path] [--steps N] [--tones N] "
                            "[--read-chans N] [--warmup N] [--out file] "
                            "[--write-addr 0x..] [--read-addr 0x..] [--bulk]\n", argv[0]);
            return 1;
        }
    }
    if (write_addr == read_addr) {
        fprintf(stderr, "error: --write-addr and --read-addr must differ\n");
        return 1;
    }

    size_t write_words = (size_t)n_tones * CONTROL_N_WORDS;
    size_t write_bytes = write_words * 4;
    size_t read_words  = (size_t)n_read_chans * 2;   /* real+imag */
    size_t read_bytes  = read_words * 4;

    int is_devmem = !strcmp(dev, "/dev/mem");
    int fd;
    off_t map_offset = 0;

    if (is_devmem) {
        fd = open(dev, O_RDWR | O_SYNC);
        if (fd < 0) { perror("open /dev/mem"); return 1; }
        map_offset = (off_t)AXIL_OFFSET;
    } else {
        fd = open(dev, O_RDWR | O_CREAT, 0644);
        if (fd < 0) { perror("open backing file"); return 1; }
        if (ftruncate(fd, MAP_SIZE) != 0) { perror("ftruncate"); return 1; }
    }

    volatile uint8_t *base = mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE,
                                  MAP_SHARED, fd, map_offset);
    if (base == MAP_FAILED) { perror("mmap"); return 1; }

    volatile uint32_t *wbuf = (volatile uint32_t *)(base + write_addr);
    volatile uint32_t *rbuf = (volatile uint32_t *)(base + read_addr);
    /* plain (non-volatile) views for the bulk memcpy path */
    void *wdst = (void *)(base + write_addr);
    void *rsrc = (void *)(base + read_addr);

    /* payload to write each step */
    uint32_t *payload = malloc(write_bytes);
    for (size_t i = 0; i < write_words; i++) payload[i] = (uint32_t)i;
    /* destination buffer for the bulk read (mirrors Python copying mm[a:b]
     * out and then np.frombuffer-ing it) */
    uint32_t *rdst = malloc(read_bytes);

    double *write_times = malloc(n_steps * sizeof(double));
    double *read_times  = malloc(n_steps * sizeof(double));
    double *step_times  = malloc(n_steps * sizeof(double));

    volatile uint32_t sink = 0;  /* prevents the read loop being optimised out */

    /* one step = control write + accumulator read.
     * per-word: individual volatile MMIO accesses (default).
     * bulk    : a single memcpy across the region, like Python's mm[a:b]. */
    #define DO_WRITE() do { \
        if (bulk) { memcpy(wdst, payload, write_bytes); } \
        else { for (size_t k = 0; k < write_words; k++) wbuf[k] = payload[k]; } \
    } while (0)
    #define DO_READ() do { \
        if (bulk) { memcpy(rdst, rsrc, read_bytes); sink ^= rdst[0]; } \
        else { for (size_t k = 0; k < read_words; k++) sink ^= rbuf[k]; } \
    } while (0)

    for (long s = 0; s < warmup; s++) { DO_WRITE(); DO_READ(); }

    double t_start = now_s();
    for (long s = 0; s < n_steps; s++) {
        double t0 = now_s();
        DO_WRITE();
        double t1 = now_s();
        DO_READ();
        double t2 = now_s();
        write_times[s] = t1 - t0;
        read_times[s]  = t2 - t1;
        step_times[s]  = t2 - t0;
    }
    double t_total = now_s() - t_start;

    printf("\n=== C devmem profile ===\n");
    printf("device           : %s%s\n", dev, is_devmem ? " (AXI-lite @0xA0000000)" : "");
    printf("access mode      : %s\n", bulk ? "bulk (single memcpy)" : "per-word (volatile)");
    printf("steps            : %ld (warmup %ld)\n", n_steps, warmup);
    printf("tones/write      : %ld  -> %zu bytes\n", n_tones, write_bytes);
    printf("read chans       : %ld -> %zu bytes\n", n_read_chans, read_bytes);
    printf("total wall time  : %.2f ms  (%.2f us/step)\n",
           t_total * 1e3, t_total / n_steps * 1e6);
    printf("per-op timings:\n");
    summary_t sw = summarize("write", write_times, n_steps, write_bytes);
    summary_t sr = summarize("read",  read_times,  n_steps, read_bytes);
    summary_t ss = summarize("step(w+r)", step_times, n_steps, write_bytes + read_bytes);

    /* keep sink observable */
    if (sink == 0xDEADBEEF) fprintf(stderr, "unreachable %u\n", sink);

    FILE *f = fopen(out, "w");
    if (!f) { perror("fopen out"); return 1; }
    fprintf(f, "# C devmem profile\n");
    fprintf(f, "# device=%s steps=%ld warmup=%ld mode=%s\n",
            dev, n_steps, warmup, bulk ? "bulk" : "per-word");
    fprintf(f, "# tones_per_write=%ld write_bytes=%zu read_chans=%ld read_bytes=%zu\n",
            n_tones, write_bytes, n_read_chans, read_bytes);
    fprintf(f, "# total_wall_ms=%.4f\n", t_total * 1e3);
    summary_t arr[3] = { sw, sr, ss };
    for (int i = 0; i < 3; i++) {
        fprintf(f, "# summary %s: mean_us=%.3f median_us=%.3f min_us=%.3f "
                   "p99_us=%.3f max_us=%.3f MB_s=%.2f\n",
                arr[i].name, arr[i].mean_us, arr[i].median_us, arr[i].min_us,
                arr[i].p99_us, arr[i].max_us, arr[i].mb_per_s);
    }
    fprintf(f, "# columns: step_index  write_us  read_us  step_us\n");
    for (long s = 0; s < n_steps; s++) {
        fprintf(f, "%ld\t%.4f\t%.4f\t%.4f\n", s,
                write_times[s] * 1e6, read_times[s] * 1e6, step_times[s] * 1e6);
    }
    fclose(f);
    printf("\nwrote per-step samples -> %s\n\n", out);

    munmap((void *)base, MAP_SIZE);
    close(fd);
    free(payload); free(rdst); free(write_times); free(read_times); free(step_times);
    return 0;
}
