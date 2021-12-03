/*
 * Copyright (c) 2017, The Linux Foundation. All rights reserved.
 *
 * SPDX-License-Identifier:    BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define _GNU_SOURCE
#include <sched.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <sys/mman.h>
#include <linux/mman.h>
#include <pthread.h>
#include <sys/types.h>
#include <time.h>
#include <string.h>
#include <fcntl.h>
#include <limits.h>
#include <sys/time.h>
#include <math.h>
#include <locale.h>
#include <signal.h>


// PROGRESS_TICK_PROFILE - prints each thread's timer value at nlocks milestones to show thread concurrency
#define PROGRESS_TICK_PROFILE


#define TEST_HNF_MMAP

#include "lockhammer.h"
#include "perf_timer.h"
#include "alloc.h"

// function prototypes used by osq_lock
uintptr_t get_phys_addr(uintptr_t vaddr);
unsigned long delay_cycles_for_cpu_relax;
void * mmap_any_page(void);
//unsigned long calculate_select6(unsigned long x);
uint64_t * mmap_a_fixed_page(void * req_addr);

uint64_t * get_hnf_targeted_lock_memory(size_t erg_bytes, size_t num_lock_memory_tries,
	int cmn_hash_select_mode, int target_hnf, int use_mmap, int hugepagesz);

// cmn-phys-mem.c
int parse_cmn_hash_select_mode(const char * optarg);
int mem_is_on_target(int select_mode, unsigned long target_select, void * p);
unsigned long calculate_select6(unsigned long x);
void test_hnf_mmap (int cmn_hash_select_mode);
unsigned long (*return_select_function(int select_mode))(unsigned long);

int cmn_hash_select_mode = 0;   // default do not target an hnf
size_t page_size = 0;
size_t erg_bytes = 0;



#include ATOMIC_TEST

struct {
	uint64_t * p_test_lock;
	uint64_t * p_sync_lock;
	uint64_t * p_calibrate_lock;
	uint64_t * p_ready_lock;
} locks;

#define handle_error_en(en, msg) \
    do { errno = en; perror(msg); exit(EXIT_FAILURE); } while (0)

void* hmr(void *);

void print_usage (char *invoc) {
    fprintf(stderr,
            "Usage: %s\n\t[-t <#> threads]\n\t[-a <#> acquires per thread]\n\t"
            "[-c <#>[ns | in] critical iterations measured in ns or (in)structions, "
            "if no suffix, assumes instructions]\n\t"
            "[-p <#>[ns | in] parallelizable iterations measured in ns or (in)structions, "
            "if no suffix, assumes (in)structions]\n\t"
            "[-s safe-mode operation for running as non-root by reducing priority]\n\t"
            "[-i <#> interleave value for SMT pinning, e.g. 1: core pinning / no SMT, "
            "2: 2-way SMT pinning, 4: 4-way SMT pinning, may not work for multisocket]\n\t"
            "[-o <#:#:#:#> arbitrary pinning order separated by colon without space, "
            "command lstopo can be used to deduce the correct order]\n\t"
            "[-f select<N>  for CMN HN-F select hash algorithm, 1-6 ]\n\t"
            "[-H <#>  for which CMN HN-F to allocate lock variables]\n\t"
            "[-F <#>  for how many candidate lock variable locations to try]\n\t"
            "[-E <#>  Exclusive Reservation Granularity log_2 bytes; 0 means to use CTR.ERG\n\t"
            "[-O <#> Run limit timer ticks\n\t"
            "[-I <#> Run limit inner loop iterations\n\t"
	    "[-D delay  for osq_lock with RELAX_IS_UDELAY or RELAX_IS_NDELAY, the delay\n\t"
            "[-M [hugepagesize] use mmap for alloc; if hugepagesize given, use that size; use \"help\" to list sizes; default hugepagesize is \"default\"\n\t"
            "[-- <more workload specific arguments>]\n", invoc);
}

#define MAXTHREADS 513

pthread_t hmr_threads[MAXTHREADS];
unsigned long hmrs[MAXTHREADS];
unsigned long hmrtime[MAXTHREADS]; /* can't touch this */	// CPU time ns
unsigned long hmrrealtime[MAXTHREADS];	// wall clock time ns
unsigned long hmrdepth[MAXTHREADS];

unsigned long cntvct_start[MAXTHREADS];
unsigned long cntvct_end[MAXTHREADS];

#ifdef PROGRESS_TICK_PROFILE
unsigned long cntvct_10p[MAXTHREADS];
unsigned long cntvct_25p[MAXTHREADS];
unsigned long cntvct_50p[MAXTHREADS];
unsigned long cntvct_75p[MAXTHREADS];
unsigned long cntvct_90p[MAXTHREADS];
#endif

#ifdef OSQ_LOCK_COUNT_LOOPS
unsigned long osq_lock_wait_next_spins[MAXTHREADS];
unsigned long osq_unlock_wait_next_spins[MAXTHREADS];
unsigned long osq_lock_locked_spins[MAXTHREADS];
unsigned long osq_lock_unqueue_spins[MAXTHREADS];
unsigned long osq_lock_acquire_backoffs[MAXTHREADS];
#endif

unsigned long cntvct_diff[MAXTHREADS];

thread_args t_args[MAXTHREADS];

#define PAGESIZE_BYTES 4096


size_t nthrds;

// not really alarm, but itimer
void main_alarm_handler (int x) {
    printf("main_alarm_handler called.  terminating %zu threads.\n", nthrds);

    for (size_t i = 0; i < nthrds; i++) {
        int s = pthread_cancel(hmr_threads[i]);
        if (s) handle_error_en(x, "pthread_cancel");
    }
}

struct sigaction main_alarm_sa = {
    .sa_handler = main_alarm_handler,
    .sa_mask = {{0}},
    .sa_flags = 0
};


void * mmap_any_page(void) {
    void * mmap_ret = mmap(NULL, PAGESIZE_BYTES, PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_POPULATE|MAP_SHARED, -1, 0); // use shared instead of private so that we can remap vaddr

    if (mmap_ret == MAP_FAILED) {
        printf("mmap returned %p (MAP_FAILED). Exiting.\n", mmap_ret);
        exit(-1);
    }

    return mmap_ret;
}

uint64_t * mmap_a_fixed_page(void * req_addr) {
    void * mmap_ret = mmap(req_addr, PAGESIZE_BYTES,
            PROT_READ|PROT_WRITE,
            MAP_ANONYMOUS|MAP_POPULATE|MAP_PRIVATE|MAP_FIXED,
            -1, 0);

    if (mmap_ret == MAP_FAILED) {
        printf("mmap returned %p (MAP_FAILED). Exiting.\n", mmap_ret);
        exit(-1);
    }

    memset(mmap_ret, 0, 8);  // MAP_POPULATE causes prefault, but not allocation of memory, right? write to it to ensure allocation?

    return mmap_ret;
}




uint64_t * get_hnf_targeted_lock_memory(size_t erg_bytes, size_t num_lock_memory_tries,
	int cmn_hash_select_mode, int target_hnf, int use_mmap, int hugepagesz)
{
    size_t candidate_buffer_size = erg_bytes * num_lock_memory_tries;
    uint64_t * buf;

    if (use_mmap) {  // use mmap to get anonymous memory, optionally using MAP_HUGETLB
        buf = do_alloc(candidate_buffer_size, hugepagesz, erg_bytes);
    } else {
        buf = aligned_alloc(erg_bytes, candidate_buffer_size);
    }

    memset(buf, 0, candidate_buffer_size);	// actually allocate real memory

    uint64_t * p_lock = buf;
    int found_target = 0;

    if (cmn_hash_select_mode != 0) {
        for (size_t i = 0; i < num_lock_memory_tries; i++) {
            uint64_t * p = &(buf[i * erg_bytes / sizeof(uint64_t)]);	// advance by cacheline length
            if (mem_is_on_target(cmn_hash_select_mode, target_hnf, p)) {
                p_lock = p;
                found_target = 1;
                printf("found CMN HN-F target at offset i=%zu\n", i);
                break;
            }
        }
        if (! found_target) {
            fprintf(stderr, "did not find an HNF target after %zu tries\n",
                 num_lock_memory_tries);
            exit(-1);
        }
    }

    return p_lock;
}


// exclusive reservation granule ranges from 4 to 512 words.  read from CTR_EL0
size_t get_ctr_erg_bytes(void) {
#if defined(__aarch64__)
    size_t CTR, ERG, ERG_words;
    asm volatile ("mrs %0, CTR_EL0" : "=r" (CTR));
    ERG = (CTR >> 20) & 0xF;
    if (ERG == 0) {
       // According to Arm ARM, if CTR[ERG] is not specified, assume 512 words
       ERG_words = 512;
    }
    ERG_words = 1 << ERG;
//    printf("CTR = %lX, ERG = %lx (%zu words)\n", CTR, ERG, ERG_words);
    return ERG_words * 4;
#endif
#if defined(__x86_64__)
    // Intel architecture manual "11.4.5 Prevent Sharing of Modified Data and
    // False-Sharing" describes a way to determine this using CPUID
    // but the safe value is 64 bytes.
    return 64;
#endif
// intentionally blank here to cause compiler to error if neither x86_64 nor
// aarch64
}

char is_earlier_than_range(unsigned long v, unsigned long * haystack, size_t nthrds) {
    for (size_t i = 0; i < nthrds; i++) {
	if (v < haystack[i]) { return '<'; }
    }
    return ' ';
}

char is_later_than_range(unsigned long v, unsigned long * haystack, size_t nthrds) {
    for (size_t i = 0; i < nthrds; i++) {
	if (v > haystack[i]) { return '<'; }
    }
    return ' ';
}



int main(int argc, char** argv)
{
    struct sched_param sparam;

    unsigned long opt;
    unsigned long num_cores;    // number of processors online
    unsigned long result;
    unsigned long sched_elapsed = 0, real_elapsed = 0, realcpu_elapsed = 0;
    unsigned long start_ns = 0;
    double avg_lock_depth = 0.0;
    size_t num_lock_memory_tries = 64 * 2;  // this needs to be changed if select mode changes I guess?
    int target_hnf = 0;                 // this is the target HN-F number
    size_t erg_bytes_log_2 = 0;
    unsigned long run_limit_ticks = 0;
    unsigned long run_limit_inner_loop_iters = 10;
    int use_mmap = 0;
    int hugepagesz = HUGEPAGES_DEFAULT;
    unsigned long pinorder_count = 0;
    int do_hnf_map_test = 0;

    unsigned long timeout_usec = 0;

    // init some globals
    page_size = sysconf(_SC_PAGE_SIZE);
    printf("page_size = %zu\n", page_size);

    // get the set of online cores
    cpu_set_t online_cores;
    CPU_ZERO(&online_cores);

    sched_getaffinity(0, sizeof(cpu_set_t), &online_cores);
    num_cores = sysconf(_SC_NPROCESSORS_ONLN);

    /* Set defaults for all command line options */
    test_args args = { .nthrds = num_cores, // -t
                       .nacqrs = 50000,     // -a
                       .ncrit = 0,          // -c
                       .nparallel = 0,      // -p
                       .ileave = 1,         // -i
                       .safemode = 0,       // -s
                       .pinorder = NULL};   // -o

    opterr = 0;

    while ((opt = getopt(argc, argv, "t:a:c:p:i:o:sf:H:F:E:D:I:O:M::TA:")) != -1)
    {
        long optval = 0;
        int len = 0;
        char *csv = NULL;
        switch (opt) {
          case 'A':
            timeout_usec = strtoul(optarg, NULL, 0);
            break;
          case 'T':
            do_hnf_map_test = 1;
            break;
          case 't': // number of threads
            optval = strtol(optarg, (char **) NULL, 10);
            /* Do not allow number of threads to exceed online cores
               in order to prevent deadlock ... */
            if (optval < 0) {
                fprintf(stderr, "ERROR: thread count must be positive.\n");
                return 1;
            }
            else if (optval == 0) {
                optval = num_cores;
            }
            else if (optval <= num_cores) {
                args.nthrds = optval;
            }
            else {
		        // XXX: the behavior described by this message is not implemented.
                fprintf(stderr, "WARNING: limiting thread count to online cores (%ld).\n", num_cores);
            }
            break;
          case 'a':	// iter
            optval = strtol(optarg, (char **) NULL, 10);
            if (optval < 0) {
                fprintf(stderr, "ERROR: acquire count must be positive.\n");
                return 1;
            }
            else {
                args.nacqrs = optval;
            }
            break;
          case 'c':	// hold_unit
            // Set the units for loops
            len = strlen(optarg);
            if (optarg[len - 1] == 's') {
                args.ncrit_units = NS;
            } else {
                args.ncrit_units = INSTS;
            }

            optval = strtol(optarg, (char **) NULL, 10);
            if (optval < 0) {
                fprintf(stderr, "ERROR: critical iteration count must be positive.\n");
                return 1;
            }
            else {
                args.ncrit = optval;
            }
            break;
          case 'p':	// .post, .post_unit ns or insts
            // Set the units for loops
            len = strlen(optarg);
            if (optarg[len - 1] == 's') {
                args.nparallel_units = NS;
            } else {
                args.nparallel_units = INSTS;
            }

            optval = strtol(optarg, (char **) NULL, 10);
            if (optval < 0) {
                fprintf(stderr, "ERROR: parallel iteration count must be positive.\n");
                return 1;
            }
            else {
                args.nparallel = optval;
            }
            break;
          case 'i':	// .ileave
            optval = strtol(optarg, (char **) NULL, 10);
            if (optval < 0) {
                fprintf(stderr, "ERROR: core interleave must be positive.\n");
                return 1;
            }
            else {
                args.ileave = optval;
            }
            break;
          case 'o':	// pinorder - list the core numbers on which to run
            if (args.pinorder) {
                fprintf(stderr, "ERROR: do not specify -o pinorder more than once.\n"
                        "Use , or : to list multiple CPUs in pinorder\n");
                exit(-1);
            }
            args.pinorder = calloc(num_cores, sizeof(int));
            if (args.pinorder == NULL) {
                fprintf(stderr, "ERROR: cannot allocate enough memory for pinorder structure.\n");
                return 1;
            }
            {
                cpu_set_t pinorder_specified_cores;
                CPU_ZERO(&pinorder_specified_cores);

                /* support both comma and colon as delimiter */
                csv = strtok(optarg, ",:");
                for (size_t i = 0; i < num_cores && csv != NULL; ++i)
                {
                    optval = strtol(csv, (char **) NULL, 0);

                    if (CPU_ISSET(optval, &pinorder_specified_cores)) {
                        fprintf(stderr, "ERROR: core number %ld was previously specified in -o pinorder list.\n", optval);
                        exit(-1);
                    } else {
                        CPU_SET(optval, &pinorder_specified_cores);
                    }

                    if (! CPU_ISSET(optval, &online_cores)) {
                        fprintf(stderr, "ERROR: core number %ld specified in -o is not online.\n", optval);
                        exit(-1);
                    }

                    args.pinorder[i] = optval;
                    pinorder_count++;

                    csv = strtok(NULL, ",:");
                }

            }
            args.nthrds = pinorder_count;
            break;
          case 's':
            args.safemode = 1;
            break;
          case 'F': // number of lock structures to try to match HN-F
            num_lock_memory_tries = strtol(optarg, (char **) NULL, 0);
            break;
          case 'f': // CMN Hash select mode
            cmn_hash_select_mode = parse_cmn_hash_select_mode(optarg);
            break;
          case 'H':	// target HN-F number
            target_hnf = strtol(optarg, (char **) NULL, 0);
            break;
          case 'E': // log_2 of number of bytes for the exclusive reservation granule
            erg_bytes_log_2 = strtol(optarg, (char **) NULL, 0);
            if (erg_bytes_log_2 < 3) {
                fprintf(stderr, "can't specify an ERG log_2 value less than 3 because a 64-bit pointer needs at least 8 bytes\n");
                return -1;
            }
            break;
	  case 'D':  // delay for __nd
#if defined(__LINUX_OSQ_LOCK_H) && (defined(RELAX_IS_NDELAY) || defined(RELAX_IS_UDELAY))
	    {
		unsigned long cntfrq;
		asm ("mrs %0, cntfrq_el0" : "=r" (cntfrq));     // cycles per second
		printf("cntfrq = %lu\n", cntfrq);
#ifdef RELAX_IS_NDELAY
		unsigned long delay_time_ns = strtoul(optarg, NULL, 0);
		printf("delay_time_ns = %lu\n", delay_time_ns);
		delay_cycles_for_cpu_relax = delay_time_ns * cntfrq / 1e9;
#elif defined(RELAX_IS_UDELAY)
		unsigned long delay_time_us = strtoul(optarg, NULL, 0);
		printf("delay_time_us = %lu\n", delay_time_us);
		delay_cycles_for_cpu_relax = delay_time_us * cntfrq / 1e6;
#endif
	    }
	    printf("delay_cycles_for_cpu_relax = %lu\n", delay_cycles_for_cpu_relax);
#else
            printf("ignoring -D flag because it is only for modified lh_osq_lock for ndelay() or udelay()\n");
#endif
	    break;
          case 'O':
            run_limit_ticks = strtoul(optarg, NULL, 0);
            break;
          case 'I':
            run_limit_inner_loop_iters = strtoul(optarg, NULL, 0);
            printf("run_limit_inner_loop_iters = %lu\n", run_limit_inner_loop_iters);
            break;
          case 'M':
            use_mmap = 1;
	    //printf("optarg = %s, argv[optind] = %s\n", optarg, argv[optind]);
            if (optarg) {   // e.g. -Mhelp
                hugepagesz = parse_hugepagesz(optarg);
            } else if (argv[optind]) {
		if (argv[optind][0] == '-') {	// e.g. -M -nextflag => default
		    // argv[optind] is next flag
		    hugepagesz = HUGEPAGES_DEFAULT;
		} else if (argv[optind][0] != '\0') { // e.g. -M 2m => 2M
		    hugepagesz = parse_hugepagesz(argv[optind]);
            optind++;
		}
	    }
	    printf("using mmap; hugepagesz = %s\n", get_hugepagesz_by_enum(hugepagesz));
            break;
          case '?':
          default:
            print_usage(argv[0]);
            return 1;
        }
    }

    parse_test_args(args, argc, argv);


    // (re)compute exclusive reservation granule size for load/store exclusives

    if (erg_bytes_log_2 == 0) {
        erg_bytes = get_ctr_erg_bytes();
    } else {
        erg_bytes = 1 << erg_bytes_log_2;
    }

//    printf("Exclusive Reservation Granule size in bytes = %zu\n", erg_bytes);

    if (do_hnf_map_test) {
        test_hnf_mmap(cmn_hash_select_mode); // does not return
    }

    if (target_hnf >= (1 << cmn_hash_select_mode)) {
	printf("invalid target_hnf=%d, it is >= to select%d\n", target_hnf, cmn_hash_select_mode);
	exit(-1);
    }

    printf("Allocating the basic lockhammer locks using cmn_hash_select_mode = %d, target_hnf = %d\n", cmn_hash_select_mode, target_hnf);

    // allocate memory for each lock that maps into target_hnf
    // NOTE: this is not used for lh_osq_lock, which does its own lock allocation

    locks.p_test_lock      = get_hnf_targeted_lock_memory(erg_bytes, num_lock_memory_tries, cmn_hash_select_mode, target_hnf, use_mmap, hugepagesz);
    locks.p_sync_lock      = get_hnf_targeted_lock_memory(erg_bytes, num_lock_memory_tries, cmn_hash_select_mode, target_hnf, use_mmap, hugepagesz);
    locks.p_calibrate_lock = get_hnf_targeted_lock_memory(erg_bytes, num_lock_memory_tries, cmn_hash_select_mode, target_hnf, use_mmap, hugepagesz);
    locks.p_ready_lock     = get_hnf_targeted_lock_memory(erg_bytes, num_lock_memory_tries, cmn_hash_select_mode, target_hnf, use_mmap, hugepagesz);

#if 0
    printf("locks.p_test_lock      = %p\n", locks.p_test_lock      );
    printf("locks.p_sync_lock      = %p\n", locks.p_sync_lock      );
    printf("locks.p_calibrate_lock = %p\n", locks.p_calibrate_lock );
    printf("locks.p_ready_lock     = %p\n", locks.p_ready_lock     );
#endif

    pthread_attr_t hmr_attr;
    struct timespec tv_time;

    /* Select the FIFO scheduler.  This prevents interruption of the
       lockhammer test threads allowing for more precise measuremnet of
       lock acquisition rate, especially for mutex type locks where
       a lock-holding or queued thread might significantly delay forward
       progress if it is rescheduled.  Additionally the FIFO scheduler allows
       for a better guarantee of the requested contention level by ensuring
       that a fixed number of threads are executing simultaneously for
       the duration of the test.  This comes at the significant cost of
       reduced responsiveness of the system under test and the possibility
       for system instability if the FIFO scheduled threads remain runnable
       for too long, starving other processes.  Care should be taken in
       invocation to ensure that a given instance of lockhammer runs for
       no more than a few milliseconds and lockhammer should never be run
       on an already-deplayed system. */

    int s = pthread_attr_init(&hmr_attr);
    if (s) handle_error_en(s, "pthread_attr_init");
    if (!args.safemode) {
        s = pthread_attr_setinheritsched(&hmr_attr, PTHREAD_EXPLICIT_SCHED);
        if (s) handle_error_en(s, "pthread_attr_setinheritsched");
        s = pthread_attr_setschedpolicy(&hmr_attr, SCHED_FIFO);
        if (s) handle_error_en(s, "pthread_attr_setschedpolicy");
        sparam.sched_priority = 1;
        s = pthread_attr_setschedparam(&hmr_attr, &sparam);
        if (s) handle_error_en(s, "pthread_attr_setschedparam");
    }

    nthrds = args.nthrds;

    long thread_return_code[args.nthrds];
    for (size_t i = 0; i < args.nthrds; i++) {
        thread_return_code[i] = 0;
    }

#if defined(__LINUX_OSQ_LOCK_H) && defined(USE_HNF_TARGETED_ALLOC)
    initialize_lock(args.nthrds, args.pinorder);
#else
    initialize_lock(locks.p_test_lock, num_cores);
#endif

    // Get frequency of clock, and divide by 1B to get # of ticks per ns
    double tickspns = (double)timer_get_cnt_freq() / 1000000000.0;

    struct itimerval deadline = {
        .it_interval = { .tv_sec = 0, .tv_usec = 0 },
        .it_value = { .tv_sec = 0, .tv_usec = 0 }       // time until next expiration
    };

    struct itimerval disable_timer = {
        .it_interval = { .tv_sec = 0, .tv_usec = 0 },
        .it_value = { .tv_sec = 0, .tv_usec = 0 }
    };

    double run_limit_seconds = 0;

    if (run_limit_ticks) {
        uint32_t cntfrq = timer_get_cnt_freq();
        run_limit_seconds = run_limit_ticks / (double) cntfrq;
        printf("run_limit_ticks = %lu, at %u Hz, should take %f seconds\n",
                run_limit_ticks, cntfrq, run_limit_seconds);
    }

    if (timeout_usec) {
        deadline.it_value.tv_sec = timeout_usec / 1000000;
        deadline.it_value.tv_usec = timeout_usec % 1000000;
        printf("setitimer timeout = -A %lu.%06lu seconds\n",
            deadline.it_value.tv_sec,
            deadline.it_value.tv_usec);
    }

    if (run_limit_ticks && timeout_usec) {
        double deadline_it_value_sec = deadline.it_value.tv_sec + deadline.it_value.tv_usec * 1e-6;
        if (deadline_it_value_sec < run_limit_seconds) {
            printf("WARNING: setitimer timeout is less than run_limit_ticks\n");
        }
    }

    if (run_limit_ticks || timeout_usec) {
        setitimer(ITIMER_REAL, &deadline, NULL);
        sigaction(SIGALRM, &main_alarm_sa, NULL);
    }

    for (int i = 0; i < args.nthrds; ++i) {
        hmrs[i] = 0;
        t_args[i].ncores = num_cores;
        t_args[i].nthrds = args.nthrds;
        t_args[i].ileave = args.ileave;
        t_args[i].iter = args.nacqrs;
        t_args[i].lock = locks.p_test_lock;	// this is the main lock.
        t_args[i].rst = &hmrs[i];	// nlocks, should be .iter
        t_args[i].nsec = &hmrtime[i];	// CPU time
        t_args[i].real_nsec = &hmrrealtime[i];	// wallclock time
        t_args[i].depth = &hmrdepth[i];	// total_depth from lock implementation
        t_args[i].nstart = &start_ns;	// only marshal thread does this.
        t_args[i].hold = args.ncrit;
        t_args[i].hold_unit = args.ncrit_units;
        t_args[i].post = args.nparallel;
        t_args[i].post_unit = args.nparallel_units;
        t_args[i].tickspns = tickspns;
        t_args[i].pinorder = args.pinorder;
        t_args[i].cntvct_start = &cntvct_start[i];
        t_args[i].cntvct_end = &cntvct_end[i];
#ifdef PROGRESS_TICK_PROFILE
        t_args[i].cntvct_10p = &cntvct_10p[i];
        t_args[i].cntvct_25p = &cntvct_25p[i];
        t_args[i].cntvct_50p = &cntvct_50p[i];
        t_args[i].cntvct_75p = &cntvct_75p[i];
        t_args[i].cntvct_90p = &cntvct_90p[i];
#endif

#ifdef OSQ_LOCK_COUNT_LOOPS
        t_args[i].posq_lock_wait_next_spins   = &osq_lock_wait_next_spins  [i];
        t_args[i].posq_unlock_wait_next_spins = &osq_unlock_wait_next_spins[i];
        t_args[i].posq_lock_locked_spins      = &osq_lock_locked_spins     [i];
        t_args[i].posq_lock_unqueue_spins     = &osq_lock_unqueue_spins    [i];
        t_args[i].posq_lock_acquire_backoffs  = &osq_lock_acquire_backoffs [i];
#endif
        t_args[i].run_limit_ticks = run_limit_ticks;
        t_args[i].run_limit_inner_loop_iters = run_limit_inner_loop_iters;

        s = pthread_create(&hmr_threads[i], &hmr_attr, hmr, (void*)(&t_args[i]));
        if (s)
             handle_error_en(s, "pthread_create");
    }


    for (size_t i = 0; i < args.nthrds; ++i) {
        void * pthread_return_code = (void *) &(thread_return_code[i]);
        result = pthread_join(hmr_threads[i], &pthread_return_code);	// XXX: pthread_join returns error number, 0 on success
        if (pthread_return_code == PTHREAD_CANCELED) {
            printf("thread %zu was cancelled, hmrs[i] = %lu\n", i, hmrs[i]);
            thread_return_code[i] = 0;
        }
    }
    /* "Marshal" thread will collect start time once all threads have
        reported ready so we only need to collect the end time here */
    clock_gettime(CLOCK_MONOTONIC, &tv_time);

    setitimer(ITIMER_REAL, &disable_timer, NULL);    // XXX: check return value

    for (size_t i = 0; i < args.nthrds; i++) {
        printf("thread_return_code[%zu] = %ld\n", i, thread_return_code[i]);
    }

    real_elapsed = (1000000000ul * tv_time.tv_sec + tv_time.tv_nsec) - start_ns;

    pthread_attr_destroy(&hmr_attr);

    result = 0;
    for (size_t i = 0; i < args.nthrds; ++i) {
        result += hmrs[i];	// number of lockings, by thread
        sched_elapsed += hmrtime[i];	// CPU time spent by thread
        realcpu_elapsed += hmrrealtime[i];	// wallclock time spent by thread
        /* Average lock "depth" is an algorithm-specific auxiliary metric
           whereby each algorithm can report an approximation of the level
           of contention it observes.  This estimate is returned from each
           call to lock_acquire and accumulated per-thread.  These results
           are then aggregated and averaged here so that an overall view
           of the run's contention level can be determined. */
        avg_lock_depth += ((double) hmrdepth[i] / (double) hmrs[i]) / (double) args.nthrds;
    }

    fprintf(stderr, "%ld lock loops\n", result);
    fprintf(stderr, "%ld ns scheduled (sum of CPU time)\n", sched_elapsed);
    fprintf(stderr, "%ld ns real elapsed (~%f cores)\n", real_elapsed, ((float) sched_elapsed / (float) real_elapsed));
    fprintf(stderr, "%lf ns per access (scheduled)\n", ((double) sched_elapsed)/ ((double) result));
    fprintf(stderr, "%lf ns per access (real)\n", ((double) realcpu_elapsed)/ ((double) result));
    fprintf(stderr, "%lf ns access rate (wall clock time divided by total number of all lock attempts)\n", ((double) real_elapsed) / ((double) result));
    fprintf(stderr, "%lf average depth\n", avg_lock_depth);

    printf("nthrds, sched_elapsed/real_elapsed, sched_elapsed/result, realcpu/result, real/result, avg_lock_depth\n");
    printf("%ld, %f, %lf, %lf, %lf, %lf\n",
           args.nthrds,
           ((float) sched_elapsed / (float) real_elapsed),
           ((double) sched_elapsed)/ ((double) result),
           ((double) realcpu_elapsed)/ ((double) result),
           ((double) real_elapsed) / ((double) result),
           avg_lock_depth);


#if defined(__aarch64__)
    char cntvct_len_testbuf[100];
    unsigned long cntvct_now;

    asm ("mrs %0, CNTVCT_EL0" : "=r" (cntvct_now));

    int cntvct_len = snprintf(cntvct_len_testbuf, sizeof(cntvct_len_testbuf), "%lu", cntvct_now);


    size_t cntvct_start_earliest_thread = -1;
    size_t cntvct_start_latest_thread = -1;
    size_t cntvct_end_earliest_thread = -1;
    size_t cntvct_end_latest_thread = -1;
    unsigned long cntvct_start_earliest = -1;
    unsigned long cntvct_start_latest = 0;
    unsigned long cntvct_end_earliest = -1;
    unsigned long cntvct_end_latest = 0;

    unsigned long cntvct_diff_shortest = -1;
    unsigned long cntvct_diff_longest = 0;

    size_t cntvct_diff_shortest_thread = -1;
    size_t cntvct_diff_longest_thread = -1;

    for (size_t i = 0; i < args.nthrds; i++) {
	cntvct_diff[i] = cntvct_end[i] - cntvct_start[i];

        if (cntvct_diff[i] > cntvct_diff_longest) {
            cntvct_diff_longest = cntvct_diff[i];
            cntvct_diff_longest_thread = i;
        }

        if (cntvct_diff[i] < cntvct_diff_shortest) {
            cntvct_diff_shortest = cntvct_diff[i];
            cntvct_diff_shortest_thread = i;
        }

        if (cntvct_end[i] > cntvct_end_latest) {
            cntvct_end_latest = cntvct_end[i];
            cntvct_end_latest_thread = i;
        }

        if (cntvct_start[i] > cntvct_start_latest) {
            cntvct_start_latest = cntvct_start[i];
            cntvct_start_latest_thread = i;
        }

        if (cntvct_end[i] < cntvct_end_earliest) {
            cntvct_end_earliest = cntvct_end[i];
            cntvct_end_earliest_thread = i;
        }

        if (cntvct_start[i] < cntvct_start_earliest) {
            cntvct_start_earliest = cntvct_start[i];
            cntvct_start_earliest_thread = i;
        }
    }

    setlocale(LC_NUMERIC, "en_US.UTF-8");

    unsigned long cntfrq;
    asm ("mrs %0, CNTFRQ_EL0" : "=r" (cntfrq));
    printf("cntfrq = %'lu (%.3f ns per tick)\n", cntfrq, 1e9/cntfrq);

    printf("cntvct_start:  0x%lx .. 0x%lx (thread %zu .. %zu), spread = %'lu (%'.2f ns)\n",
		cntvct_start_earliest, cntvct_start_latest,
		cntvct_start_earliest_thread, cntvct_start_latest_thread,
		cntvct_start_latest - cntvct_start_earliest,
		(cntvct_start_latest - cntvct_start_earliest) * 1e9 / cntfrq
		);

    printf("cntvct_end:    0x%lx .. 0x%lx (thread %zu .. %zu), spread = %'lu (%'.2f ns)\n",
		cntvct_end_earliest, cntvct_end_latest,
		cntvct_end_earliest_thread, cntvct_end_latest_thread,
		cntvct_end_latest - cntvct_end_earliest,
		(cntvct_end_latest - cntvct_end_earliest) * 1e9 / cntfrq
		);

    printf("cntvct_diff: %lu (%'.3f ns) .. %lu (%'.3f ns) (thread %zu .. %zu), spread = %'lu (%'.3f ns), long/short = %0.3f\n",
		cntvct_diff_shortest, cntvct_diff_shortest * 1e9 / cntfrq,
		cntvct_diff_longest, cntvct_diff_longest * 1e9 / cntfrq,
		cntvct_diff_shortest_thread, cntvct_diff_longest_thread,
		cntvct_diff_longest - cntvct_diff_shortest,
		(cntvct_diff_longest - cntvct_diff_shortest) * 1e9 / cntfrq,
		cntvct_diff_longest / (double) cntvct_diff_shortest);


    double mean_cntvct_diff = 0;
    unsigned long cntvct_diff_sum = 0;

    for (size_t i = 0; i < args.nthrds; i++) {
        cntvct_diff_sum += cntvct_diff[i];
    }

    mean_cntvct_diff = ((double) cntvct_diff_sum) / args.nthrds;

    double sum_diff_squared = 0;
    double mean_abs_diff = 0;

    for (size_t i = 0; i < args.nthrds; i++) {
        double diff = (cntvct_diff[i] - mean_cntvct_diff);
        sum_diff_squared += diff * diff;

        mean_abs_diff += abs(diff);
    }

    sum_diff_squared /= args.nthrds;
    mean_abs_diff /= args.nthrds;

    double std_dev = sqrt(sum_diff_squared);

    printf("cntvct_diff_stats:  mean = %.f (%'0.3f ns), mean_abs_diff = %0.3f, std_dev = %0.3f\n",
	mean_cntvct_diff, mean_cntvct_diff * 1e9 / cntfrq, mean_abs_diff, std_dev);

    // compute buckets against normal distribution

    size_t neg_buckets[4] = {0};
    size_t pos_buckets[4] = {0};

    for (size_t i = 0; i < args.nthrds; i++) {
        double diff = (cntvct_diff[i] - mean_cntvct_diff);
        double sigmas = diff / std_dev;

        long bucket = (long) sigmas;
        if (bucket > 3)  { bucket = 3; }
        if (bucket < -3) { bucket = -3; }

        if (sigmas >= 0) {
            pos_buckets[bucket]++;
        } else {
            neg_buckets[-bucket]++;
        }

        // printf("sigmas = %f, bucket = %ld\n", sigmas, bucket);
    }

    printf("cntvct_diff distribution:\n");
    printf( "<-3 sigma: %zu\n",         neg_buckets[    4 - 1]);
    for (long sigma = 3; sigma >= 1; sigma--) {
        printf(" %2ld sigma: %zu\n", -sigma, neg_buckets[sigma - 1]);
    }
    for (long sigma = 1; sigma <  4; sigma++) {
        printf(" %2ld sigma: %zu\n", sigma, pos_buckets[sigma - 1]);
    }
    printf( " >3 sigma: %zu\n",        pos_buckets[    4 - 1]);

#ifdef PROGRESS_TICK_PROFILE
    // print cntvct tick on each thread at quarter intervals

    if (run_limit_ticks == 0) { // only do this for -a num_acquires mode
        printf("thread\t%*s %*s %*s %*s %*s %*s %*s 75%% rate (ns/acq)\n",
            -cntvct_len,  "0%",
            -cntvct_len-1,"10%",
            -cntvct_len,  "25%",
            -cntvct_len,  "50%",
            -cntvct_len-1,"75%",
            -cntvct_len-1,"90%",
            -cntvct_len,  "100%");

        for (size_t i = 0; i < args.nthrds; i++) {
            unsigned long ticks_10p = cntvct_10p[i] - cntvct_start[i];
            unsigned long ticks_25p = cntvct_25p[i] - cntvct_start[i];
            unsigned long ticks_50p = cntvct_50p[i] - cntvct_start[i];
            unsigned long ticks_75p = cntvct_75p[i] - cntvct_start[i];
            unsigned long ticks_90p = cntvct_90p[i] - cntvct_start[i];
            unsigned long ticks_100p= cntvct_end[i] - cntvct_start[i];

            unsigned long ticks_10p_90p = cntvct_90p[i] - cntvct_10p[i];
            double lock_acquire_rate = ticks_10p_90p * 1e9 / (0.8 * args.nacqrs * cntfrq);

            double lock_acquire_rate_75p = (cntvct_75p[i] - cntvct_start[i]) * 1e9 / (0.75 * args.nacqrs * cntfrq);
            // "lock acquire rate" is not acquires per second, it is ns per acquire.

            printf("%zu\t", i);

            if (0)	// don't want to remove it just yet, maybe useful in the future
            printf("%lu\t%lu\t%lu\t%lu\t%lu\t%lu\t%.f\t\t",
                 ticks_10p, ticks_25p, ticks_50p, ticks_75p, ticks_90p, ticks_100p, lock_acquire_rate);

            printf("%*lu %*lu%c %*lu %*lu %*lu%c %*lu%c %*lu %.f\n",
                cntvct_len, cntvct_start[i],
                cntvct_len, cntvct_10p[i], is_earlier_than_range(cntvct_10p[i], cntvct_start, args.nthrds),
                cntvct_len, cntvct_25p[i],
                cntvct_len, cntvct_50p[i],
                cntvct_len, cntvct_75p[i], is_later_than_range(cntvct_75p[i], cntvct_end, args.nthrds),
                cntvct_len, cntvct_90p[i], is_later_than_range(cntvct_90p[i], cntvct_end, args.nthrds),
                cntvct_len, cntvct_end[i],
                lock_acquire_rate_75p);
        }
    }
#endif
#endif

#ifdef OSQ_LOCK_COUNT_LOOPS
    printf("thread cpu  %20s %22s %17s %18s %22s\n", "lock_wait_next_spins", "unlock_wait_next_spins", "lock_locked_spins", "lock_unqueue_spins", "lock_acquire_backoffs");
    for (size_t i = 0; i < args.nthrds; i++) {
        printf("%6zu %3d  %20lu %22lu %17lu %18lu %22lu\n",
                i, args.pinorder ? args.pinorder[i] : (int) i,
                osq_lock_wait_next_spins[i],
                osq_unlock_wait_next_spins[i],
                osq_lock_locked_spins[i],
                osq_lock_unqueue_spins[i],
                osq_lock_acquire_backoffs[i]);
    }
#endif

    // print it again, but to stdout
    fprintf(stdout, "%lf ns access rate (wall clock time divided by total number of all lock attempts)\n", ((double) real_elapsed) / ((double) result));
    return 0;
}



/* Calculate timer spin-times where we do not access the clock.
 * First calibrate the wait loop by doing a binary search around
 * an estimated number of ticks. All threads participate to take
 * into account pipeline effects of threading.
 */
static void calibrate_timer(thread_args *x, unsigned long thread)
{
    if (x->hold_unit == NS) {
        /* Determine how many timer ticks would happen for this wait time */
        unsigned long hold = (unsigned long)((double)x->hold * x->tickspns);
        /* Calibrate the number of loops we have to do */
        x->hold = calibrate_blackhole(hold, 0, TOKENS_MAX_HIGH, thread);
    } else {
        x->hold = x->hold / 2;
    }

    // Make sure to re-sync any stragglers
    synchronize_threads(locks.p_calibrate_lock, x->nthrds);

    if (x->post_unit == NS) {
        unsigned long post = (unsigned long)((double)x->post * x->tickspns);
        x->post = calibrate_blackhole(post, 0, TOKENS_MAX_HIGH, thread);
    } else {
        x->post = x->post / 2;
    }
#ifdef DEBUG
    printf("Calibrated (%lu) with hold=%ld post=%ld\n", thread, x->hold, x->post);
#endif
}

static void update_timer_tick_progress(unsigned long nlocks,
        unsigned long target_10p, unsigned long * __restrict__ cntvct_10p,
        unsigned long target_25p, unsigned long * __restrict__ cntvct_25p,
        unsigned long target_50p, unsigned long * __restrict__ cntvct_50p,
        unsigned long target_75p, unsigned long * __restrict__ cntvct_75p,
        unsigned long target_90p, unsigned long * __restrict__ cntvct_90p) __attribute__((noinline));


#if defined(__LINUX_OSQ_LOCK_H) && defined(OSQ_LOCK_COUNT_LOOPS)

#undef lock_acquire
#define lock_acquire(lock, thread) osq_lock_acquire(lock, thread,  &osq_lock_wait_next_spins, &osq_lock_locked_spins, &osq_lock_unqueue_spins, &osq_lock_acquire_backoffs)

#undef lock_release
#define lock_release(lock, thread)  osq_lock_release(lock, thread, &osq_unlock_wait_next_spins)

#elif defined(__LINUX_OSQ_LOCK_H) && !defined(OSQ_LOCK_COUNT_LOOPS)

#undef lock_acquire
#define lock_acquire(lock, thread) osq_lock_acquire(lock, thread)

#undef lock_release
#define lock_release(lock, thread)  osq_lock_release(lock, thread)

#endif


typedef struct {
    thread_args * x;
    volatile unsigned long nlocks;
    volatile unsigned long total_depth;
    unsigned long thread;
    struct timespec * ptv_monot_start;
    struct timespec * ptv_start;
    unsigned long ticks_start;
    unsigned long ticks_end;

#ifdef OSQ_LOCK_COUNT_LOOPS
    unsigned long * posq_lock_wait_next_spins;
    unsigned long * posq_unlock_wait_next_spins;
    unsigned long * posq_lock_locked_spins;
    unsigned long * posq_lock_unqueue_spins;
    unsigned long * posq_lock_acquire_backoffs;
#endif

} cleanup_struct_t;

void thread_cleanup_routine(void * p) {
    printf("thread_cleanup_routine called\n");

    cleanup_struct_t * pcs = (cleanup_struct_t *) p;

    thread_args *x = pcs->x;
    struct timespec tv_monot_end, tv_end;

    unsigned long ticks_end = get_raw_counter();

    clock_gettime(CLOCK_MONOTONIC, &tv_monot_end);
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &tv_end);

    if (pcs->thread == 0)
        *(x->nstart) = (1000000000ul * pcs->ptv_monot_start->tv_sec + pcs->ptv_monot_start->tv_nsec);

    unsigned long ns_elap = (1000000000ul * tv_end.tv_sec + tv_end.tv_nsec) - (1000000000ul * pcs->ptv_start->tv_sec + pcs->ptv_start->tv_nsec);
    unsigned long real_ns_elap = (1000000000ul * tv_monot_end.tv_sec + tv_monot_end.tv_nsec) - (1000000000ul * pcs->ptv_monot_start->tv_sec + pcs->ptv_monot_start->tv_nsec);

    *(x->rst) = pcs->nlocks;

    *(x->nsec) = ns_elap;
    *(x->real_nsec) = real_ns_elap;

    *(x->depth) = pcs->total_depth;

    *(x->cntvct_start) = pcs->ticks_start;
    *(x->cntvct_end) = ticks_end;

#ifdef OSQ_LOCK_COUNT_LOOPS
    *(x->posq_lock_wait_next_spins)   =  *(pcs->posq_lock_wait_next_spins);
    *(x->posq_unlock_wait_next_spins) =  *(pcs->posq_unlock_wait_next_spins);
    *(x->posq_lock_locked_spins)      =  *(pcs->posq_lock_locked_spins);
    *(x->posq_lock_unqueue_spins)     =  *(pcs->posq_lock_unqueue_spins);
    *(x->posq_lock_acquire_backoffs)  =  *(pcs->posq_lock_acquire_backoffs);
#endif
}

void* hmr(void *ptr)
{
    unsigned long nlocks = 0;
    thread_args *x = (thread_args*)ptr;

#ifdef OSQ_LOCK_COUNT_LOOPS
    unsigned long osq_lock_wait_next_spins = 0;
    unsigned long osq_unlock_wait_next_spins = 0;
    unsigned long osq_lock_locked_spins = 0;
    unsigned long osq_lock_unqueue_spins = 0;
    unsigned long osq_lock_acquire_backoffs = 0;
#endif

    unsigned long *lock = x->lock;
    unsigned long target_locks = x->iter;
    unsigned long ncores = x->ncores;
    unsigned long ileave = x->ileave;
    unsigned long nthrds = x->nthrds;
    unsigned long hold_count = x->hold;
    unsigned long post_count = x->post;
    int *pinorder = x->pinorder;     // pinorder[] is the list of CPU numbers on which to run locking threads

    unsigned long thread;   // thread is hmr thread number, starting with 0.  Not a core or CPU.  Super confusing.

    struct timespec tv_monot_start, tv_monot_end, tv_start, tv_end;
    unsigned long ns_elap, real_ns_elap;
    unsigned long total_depth = 0;
    unsigned long run_limit_ticks = x->run_limit_ticks;
    unsigned long run_limit_inner_loop_iters = x->run_limit_inner_loop_iters;
    unsigned long ticks_start;
    unsigned long ticks_end;


    cpu_set_t affin_mask;

    CPU_ZERO(&affin_mask);

    /* Coordinate synchronized start of all lock threads to maximize
       time under which locks are stressed to the requested contention
       level.  thread is the order of the threads winning p_sync_lock */
    thread = fetchadd64_acquire(locks.p_sync_lock, 2) >> 1;

    cleanup_struct_t cs = {
        .x = x,
        .nlocks = 0,
        .ptv_monot_start = &tv_monot_start,
        .ptv_start = &tv_start,
        .thread = thread,
#ifdef OSQ_LOCK_COUNT_LOOPS
        .posq_lock_wait_next_spins   = & osq_lock_wait_next_spins,
        .posq_unlock_wait_next_spins = & osq_unlock_wait_next_spins,
        .posq_lock_locked_spins      = & osq_lock_locked_spins,
        .posq_lock_unqueue_spins     = & osq_lock_unqueue_spins,
        .posq_lock_acquire_backoffs  = & osq_lock_acquire_backoffs,
#endif
    };

    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    pthread_cleanup_push(thread_cleanup_routine, &cs);

    if (thread == 0) {
        /* First thread to register is a "marshal" who waits for subsequent
           threads to become ready and starts all threads with a write to the
           shared memory location. */
        if (pinorder) {
#ifdef DEBUG
            printf("pinorder[%lu] = %d, locks->sync_lock = %lu\n", thread, pinorder[thread], locks->sync_lock);
#endif
            CPU_SET(pinorder[thread], &affin_mask);
        } else {
            /* Set affinity to core 0 for first core in -i mode */
            CPU_SET(0, &affin_mask);
        }
        sched_setaffinity(0, sizeof(cpu_set_t), &affin_mask);

        /* Spin until the appropriate numer of threads have become ready */
        wait64(locks.p_ready_lock, nthrds - 1);
        fetchadd64_release(locks.p_sync_lock, 1);

        calibrate_timer(x, thread);
        hold_count = x->hold;
        post_count = x->post;
#ifdef __LINUX_OSQ_LOCK_H
        synchronize_threads(locks.p_calibrate_lock, nthrds);
        osq_lock_compute_blackhole_interval(thread, x->tickspns, pinorder);
#endif

        /* Wait for all threads to arrive from calibrating. */
        synchronize_threads(locks.p_calibrate_lock, nthrds);
        clock_gettime(CLOCK_MONOTONIC, &tv_monot_start);
    } else {
        /*
         * Non-zero core value indicates next core to pin, zero value means
         * fallback to default interleave mode. Note: -o and -i may have
         * conflicting pinning order that causes two or more threads to pin
         * on the same core. This feature interaction is intended by design
         * which allows 0 to serve as don't care mask and only changing the
         * pinning order we want to change for specific -i interleave mode.
         */
        if (pinorder) {
#ifdef DEBUG
            printf("pinorder[%lu] = %d, locks->sync_lock = %lu\n", thread, pinorder[thread], locks->sync_lock);
#endif
            CPU_SET(pinorder[thread], &affin_mask);
            sched_setaffinity(0, sizeof(cpu_set_t), &affin_mask);
        } else { /* Calculate affinity mask for my core and set affinity */
            /*
             * The concept of "interleave" is used here to allow for specifying
             * whether increasing cores counts first populate physical cores or
             * hardware threads within the same physical core. This assumes the
             * following relationship between logical core numbers (N), hardware
             * threads per core (K), and physical cores (N/K):
             *
             *  physical core |___core_0__|___core_1__|_core_N/K-1|
             *         thread |0|1|...|K-1|0|1|...|K-1|0|1|...|K-1|
             *  --------------|-|-|---|---|-|-|---|---|-|-|---|---|
             *   logical core | | |   |   | | |   |   | | |   |   |
             *              0 |*| |   |   | | |   |   | | |   |   |
             *              1 | | |   |   |*| |   |   | | |   |   |
             *            ... |...................................|
             *          N/K-1 | | |   |   | | |   |   |*| |   |   |
             *            N/K | |*|   |   | | |   |   | | |   |   |
             *          N/K+1 | | |   |   | |*|   |   | | |   |   |
             *            ... |...................................|
             *            N-K | | |   | * | | |   |   | | |   |   |
             *          N-K+1 | | |   |   | | |   | * | | |   |   |
             *            ... |...................................|
             *            N-1 | | |   |   | | |   |   | | |   | * |
             *
             * Thus by setting the interleave value to 1 physical cores are filled
             * first with subsequent cores past N/K adding subsequent threads
             * on already populated physical cores.  On the other hand, setting
             * interleave to K causes the algorithm to populate 0, N/K, 2N/K and
             * so on filling all hardware threads in the first physical core prior
             * to populating any threads on the second physical core.
             */
            CPU_SET(((thread * ncores / ileave) % ncores + (thread / ileave)), &affin_mask);
            sched_setaffinity(0, sizeof(cpu_set_t), &affin_mask);
        }

        fetchadd64_release(locks.p_ready_lock, 1);

        /* Spin until the "marshal" sets the appropriate bit */
        wait64(locks.p_sync_lock, (nthrds * 2) | 1);

        /* All threads participate in calibration */
        calibrate_timer(x, thread);
        hold_count = x->hold;
        post_count = x->post;

#ifdef __LINUX_OSQ_LOCK_H
        synchronize_threads(locks.p_calibrate_lock, nthrds);
        osq_lock_compute_blackhole_interval(thread, x->tickspns, pinorder);
#endif

        /* Wait for all threads to arrive from calibrating */
        synchronize_threads(locks.p_calibrate_lock, nthrds);
    }

    thread_local_init(thread);

#ifdef DDEBUG
    printf("thread = %lu\n", thread);
    printf("%ld %ld\n", hold_count, post_count);
#endif


    if (run_limit_ticks) {

        // TODO: for run_limit_ticks, count nlocks completed at tick milestones;  instead of ticks at nlocks milestones

        clock_gettime(CLOCK_MONOTONIC, &tv_monot_start);
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &tv_start);

        cs.ticks_start = ticks_start = get_raw_counter();

        do {

            for (size_t i = 0; i < run_limit_inner_loop_iters; i++) {
                /* Do a lock thing */
#ifndef __LINUX_OSQ_LOCK_H      // osq_lock does not use the lock pointer, and these prefetches of it are redundant
                prefetch64(lock);
#endif
                total_depth += lock_acquire(lock, thread);
                blackhole(hold_count);
                lock_release(lock, thread);
                blackhole(post_count);

                nlocks++;
            }

            cs.ticks_end = ticks_end = get_raw_counter();
            cs.total_depth = total_depth;
            cs.nlocks = nlocks;         // XXX: the problem with this is that lock_acquire() / lock_release() could livelock
                // before the for loop finishes, so cs.nlocks never gets updated.
//            printf("cs.nlocks = %lu\n", cs.nlocks);

        } while (ticks_end - ticks_start < run_limit_ticks);

        clock_gettime(CLOCK_MONOTONIC, &tv_monot_end);
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &tv_end);

    } else {

#ifdef PROGRESS_TICK_PROFILE
        unsigned long target_25p = target_locks / 4;
        unsigned long target_50p = target_25p * 2;
        unsigned long target_75p = target_25p * 3;

        unsigned long target_10p = target_locks * 0.1;
        unsigned long target_90p = target_locks * 0.9;

        unsigned long cntvct_10p = 0;
        unsigned long cntvct_25p = 0;
        unsigned long cntvct_50p = 0;
        unsigned long cntvct_75p = 0;
        unsigned long cntvct_90p = 0;
#endif

        clock_gettime(CLOCK_MONOTONIC, &tv_monot_start);
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &tv_start);

        cs.ticks_start = ticks_start = get_raw_counter();

        while (!target_locks || nlocks < target_locks) {
            /* Do a lock thing */
#ifndef __LINUX_OSQ_LOCK_H      // osq_lock does not use the lock pointer, and these prefetches of it are redundant
            prefetch64(lock);
#endif
            total_depth += lock_acquire(lock, thread);  // thread is just a logical thread number, it's not a core number
            blackhole(hold_count);
            lock_release(lock, thread);
            blackhole(post_count);

#ifdef PROGRESS_TICK_PROFILE
            // records ticks at nlocks milestones
            update_timer_tick_progress(nlocks,
                    target_10p, &cntvct_10p,
                    target_25p, &cntvct_25p,
                    target_50p, &cntvct_50p,
                    target_75p, &cntvct_75p,
                    target_90p, &cntvct_90p);
#endif

            nlocks++;
            cs.nlocks = nlocks; // XXX: will doing this be too much?
            cs.total_depth = total_depth;
        }

        cs.ticks_end = ticks_end = get_raw_counter();

        clock_gettime(CLOCK_MONOTONIC, &tv_monot_end);
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &tv_end);

#ifdef PROGRESS_TICK_PROFILE
        *(x->cntvct_10p) = cntvct_10p;
        *(x->cntvct_25p) = cntvct_25p;
        *(x->cntvct_50p) = cntvct_50p;
        *(x->cntvct_75p) = cntvct_75p;
        *(x->cntvct_90p) = cntvct_90p;
#endif
    }

    if (thread == 0)
        *(x->nstart) = (1000000000ul * tv_monot_start.tv_sec + tv_monot_start.tv_nsec);

    ns_elap = (1000000000ul * tv_end.tv_sec + tv_end.tv_nsec) - (1000000000ul * tv_start.tv_sec + tv_start.tv_nsec);
    real_ns_elap = (1000000000ul * tv_monot_end.tv_sec + tv_monot_end.tv_nsec) - (1000000000ul * tv_monot_start.tv_sec + tv_monot_start.tv_nsec);

    *(x->rst) = nlocks;
    *(x->nsec) = ns_elap;
    *(x->real_nsec) = real_ns_elap;
    *(x->depth) = total_depth;

    *(x->cntvct_start) = ticks_start;
    *(x->cntvct_end) = ticks_end;

#ifdef OSQ_LOCK_COUNT_LOOPS
    *(x->posq_lock_wait_next_spins)   =  osq_lock_wait_next_spins;
    *(x->posq_unlock_wait_next_spins) =  osq_unlock_wait_next_spins;
    *(x->posq_lock_locked_spins)      =  osq_lock_locked_spins;
    *(x->posq_lock_unqueue_spins)     =  osq_lock_unqueue_spins;
    *(x->posq_lock_acquire_backoffs)  =  osq_lock_acquire_backoffs;
#endif


    pthread_cleanup_pop(0);

    return NULL;
}



#ifdef PROGRESS_TICK_PROFILE
static void update_timer_tick_progress(unsigned long nlocks,
        unsigned long target_10p, unsigned long * __restrict__ cntvct_10p,
        unsigned long target_25p, unsigned long * __restrict__ cntvct_25p,
        unsigned long target_50p, unsigned long * __restrict__ cntvct_50p,
        unsigned long target_75p, unsigned long * __restrict__ cntvct_75p,
        unsigned long target_90p, unsigned long * __restrict__ cntvct_90p) {

    if (nlocks > target_10p && *cntvct_10p == 0) {
        *cntvct_10p = get_raw_counter();
        return;
    }

    if (nlocks > target_25p && *cntvct_25p == 0) {
        *cntvct_25p = get_raw_counter();
        return;
    }

    if (nlocks > target_50p && *cntvct_50p == 0) {
        *cntvct_50p = get_raw_counter();
        return;
    }

    if (nlocks > target_75p && *cntvct_75p == 0) {
        *cntvct_75p = get_raw_counter();
        return;
    }

    if (nlocks > target_90p && *cntvct_90p == 0) {
        *cntvct_90p = get_raw_counter();
        return;
    }
}
#endif


#ifdef TEST_HNF_MMAP
void test_hnf_mmap (int cmn_hash_select_mode) {
    const size_t tries = 50000;   // 11+x more than needed

    if (cmn_hash_select_mode == 0) {
        fprintf(stderr, "skipping test_hnf_mmap because -f selectn was not specified\n");
        exit(0);
    }

    size_t num_hnf = 1 << cmn_hash_select_mode;

    size_t cache_line_size = erg_bytes;

    if (cache_line_size != 64) {
        fprintf(stderr, "test_hnf_mmap() does not work yet for cache_line_size != 64\n");
        exit(-1);
    }

    // The expected max number of tries is (page_size/cache_line_size)*num_hnf.
    // For 4K page, 64 HN-Fs, this is tries = 4096 / 64 * 64 = 4096 tries, but
    // see below for weird problems if too much is allocated without being
    // explicitly freed.

    // This is how it works.  Allocate "tries" number of pages using mmap().
    // Then, for each page, compute a) the virtual address of the cache line
    // (CL) in the page that would have the spin_node struct's important
    // elements, and b) the physical address of the CL.  Then, compute the SCG
    // HN-F hash value of that CL's physical address.  If that CL's hash value
    // matches that of the target HN-F's hash value (its position in the RN-F
    // SAM), try to remap the virtual address of the page to be at the virtual
    // address of where it would be in the array of nodes.

    struct { void * v; int used; } v_ret[tries];
    struct { void * page_vaddr; size_t target_cl_vaddr; } v_remapped[num_hnf];

    for (size_t i = 0; i < tries; i++) {
        v_ret[i].v = mmap_any_page();
        v_ret[i].used = 0;
    }

    // printf("try: %zu\tv_ret[%zu].v = %p, .p = %zu, .hnf: %zu\n", i, i, v_ret[i].v, v_ret[i].p, v_ret[i].hnf);

    const size_t base_vaddr = 0xc00000000000UL;

    for (size_t hnf = 0; hnf < num_hnf; hnf++) {
        int found = 0;
        for (size_t i = 0; i < tries; i++) {

            // check if the page in v_ret[i] has a CL at the right offset whose paddr maps to the target HNF

            if (v_ret[i].used) { continue; }

            size_t try_cl_vaddr = ((size_t) v_ret[i].v) + hnf * cache_line_size;
            if (mem_is_on_target(cmn_hash_select_mode, hnf, (void *) try_cl_vaddr)) {

                size_t try_cl_paddr = get_phys_addr(try_cl_vaddr);

                printf("try %zu has target hnf %zu;  try_cl_vaddr = %lx, try_cl_paddr = %lx\n",
                        i, hnf, try_cl_vaddr, try_cl_paddr);

                v_ret[i].used = 1;

                size_t target_page_vaddr = base_vaddr + hnf * page_size;

                printf("trying to remap v_ret[%zu].v = %p to target_page_vaddr = %lx ... ", i, v_ret[i].v, target_page_vaddr);

                void * remapped_vaddr = mremap(v_ret[i].v, 0, page_size, MREMAP_FIXED|MREMAP_MAYMOVE, target_page_vaddr);

                if (remapped_vaddr == MAP_FAILED) {
                    printf("mremap failed\n");
                    exit(-1);
                }

                int munmap_ret = munmap(v_ret[i].v, page_size);   // munmap old mapping
                if (munmap_ret == -1) {
                    printf("munmap failed, errno = %d\n", errno);
                    exit(-1);
                }

                printf("ok\n");

                memset(remapped_vaddr, 0, 1);  // dummy write to cause copy-on-write
                // printf("after memset to remapped_vaddr\n");   // this may or may not be necessary

                v_remapped[hnf].page_vaddr = remapped_vaddr;
                v_remapped[hnf].target_cl_vaddr = (((size_t) remapped_vaddr) & (~((page_size-1)))) | (hnf * cache_line_size);

                found = 1;

                break;
            }
        }

        if (! found) {
            printf("couldn't find a match for hnf %zu\n", hnf);
            exit (-1);
        }
    }

    // print list of VA and PA of the cache lines allocated above, and the HNF that it maps to.

    for (size_t hnf = 0; hnf < num_hnf; hnf++) {
        size_t target_cl_vaddr = v_remapped[hnf].target_cl_vaddr;
        size_t target_cl_paddr = get_phys_addr(target_cl_vaddr);
        size_t target_cl_hnf = calculate_select6(target_cl_paddr);
        printf ("v_remapped[%2zu] cl_vaddr = %lx   cl_paddr = %lx  target_cl_hnf = %zu (calculated)\n",
                hnf, target_cl_vaddr, target_cl_paddr, target_cl_hnf);
    }

    // XXX: if you don't unmap the memory, Linux starts avoiding mapping to
    // paddrs that will match the desired hnf, and more and more and more tries
    // are needed to find a conforming page. maybe page coloring effect.

    for (size_t i = 0; i < tries; i++) {
        if (v_ret[i].used == 0) {
            munmap(v_ret[i].v, page_size);
        }
    }

    for (size_t hnf = 0; hnf < num_hnf; hnf++) {
        munmap(v_remapped[hnf].page_vaddr, page_size);
    }

    exit(0);
}
#else
void test_hnf_mmap(void) { printf("TEST_HNF_MMAP is not enabled.\n");  exit(-1); }
#endif

/* vim: set tabstop=4 shiftwidth=4 softtabstop=4 expandtab: */
