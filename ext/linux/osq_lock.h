/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Based on Linux kernel 4.16.10
 * https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux-stable.git
 * /commit/?h=v4.16.10&id=b3fdf8284efbc5020dfbd0a28150637189076115
 *
 * Description:
 *
 *      This workload implements kernel 'optimistic spin queue' derived from mcs
 *      lock. Tunable unqueue_retry times and max_backoff_sleep duration have
 *      also been added to simulate need_resched() condition and unqueue current
 *      cpu node from spinning queue and put to sleep.
 *
 * Changes from Linux kernel osq_lock.c
 *
 *      The original DEFINE_PER_CPU_SHARED_ALIGNED(struct optimistic_spin_node,
 *      osq_node) is modified to a cache-line aligned array allocated on the
 *      heap during osq_lock_init(), pointed to by global_osq_nodepool_ptr.
 *      The osq lock queue struct itself is declared as a global variable too,
 *      which substitutes for the upper level mutex lock struct indicated by
 *      lock pointer.  Therefore, we don't need to get the lock pointer from
 *      lock_acquire() and lock_release() interface.  The spinning node
 *      structure can be linearly located by osq_nodepool_ptr with
 *      threadnum/coreid as offset. The tail of osq_lock can be accessed by
 *      global_osq directly.
 *
 *      We haven't changed the algorithm except adding unqueue_retry and max_
 *      sleep_us as optional backoff sleep to mimic kernel rescheduling events.
 *      By default we essentially disable unqueue_retry and backoff sleep so
 *      that osq_lock performance is more stable and similar to mcs queue spin
 *      lock.
 *
 * Internals:
 *
 *      In order to port osq_lock from kernel space to user space, we added
 *      lk_barrier.h and lk_cmpxchg.h to synchronization-benchmarks/ext/linux/
 *      include. Because there are some special gcc options to restrict compiler
 *      from allocating x16/x17 registers in arch/arm64/lib/Makefile for
 *      atomic_ll_sc.o, and our osq_lock.h included from lockhammer.c will not
 *      generate any other separate object file, we have to modify cmpxchg.h
 *      and change cmpxchg LLSC/LSE implementation for aarch64.
 *
 *      Kernel arm64 cmpxchg.h supports both LLSC (load-link/store-conditional)
 *      and LSE (Armv8.1 large system extension) via dynamic binary patching.
 *      If CONFIG_AS_LSE and CONFIG_ARM64_LSE_ATOMICS have been enabled, kernel
 *      will use Armv8.1 new atomic instructions CAS to implement the compare
 *      and swap function. This inline function has 3 instructions mov/cas/mov,
 *      which will be overwritten during system boot up if the CPU doesn't
 *      support Armv8.1 LSE. The 3 new instructions are bl/nop/nop. The branch
 *      and link instruction will redirect program flow to Armv8.0 LLSC function
 *      without saving any of the caller's local registers. These registers are
 *      guaranteed to be safe because LLSC function in atomic_ll_sc.o only uses
 *      x16/x17 and LSE caller doesn't use x16/x17.
 *
 *      Since lockhammer doesn't have runtime cpu detection, whether to use LLSC
 *      or LSE is manually defined in the lockhammer Makefile. Therefore our new
 *      cmpxchg is also statically defined without branch and link or binary
 *      patching. LLSC and LSE cmpxchg will share the same interface but use
 *      different assembly code and intrinsic function calls.
 *
 * Workings:
 *
 *      osq_lock works similar to mcs spinlock except the optional unqueue path.
 *      Linux kernel qspinlock is slightly different than original mcs spinlock.
 *
 * Tuning Parameters:
 *
 *      osq_lock tuning parameters are optional.  They are specified on the
 *      lockhammer command line after lockhammer's own flags and separated from
 *      them by "--":
 *
 *          lh_osq_lock  lockhammer_flags ...  --  osq_lock_flags ...
 *
 *
 *      -u unqueue_retries                 Default: 2 billion  (2000000000)
 *          Max spin retries before going to unqueue path.
 *
 *      -S cpu:backoff_wait_us
 *          For the thread that runs on the specified cpu, wait for the
 *          specified backoff_wait_us microseconds on osq_lock unqueue.
 *          Overrides the value computed from -s max_backoff_wait_us * rand().
 *
 *      -s max_backoff_wait_us             Default: 0 us
 *          For each thread on cpus that do not have -S cpu:backoff_wait_us
 *          specified, the maximum backoff wait time in microseconds after an
 *          unqueue before another osq_lock() acquisition attempt.  The actual
 *          wait time is determined at init by choosing a random duration on
 *          the interval [0, max_backoff_wait_us), and waits for that same
 *          amount of time after each unqueue.  The wait is now implemented
 *          using the blackhole function to avoid making a nanosleep syscall.
 *
 *      -R random_seed                     Default: 0 (i.e., use seconds since epoch)
 *          Specify a random seed to use.  If not specified or if 0 is
 *          specified, then the number of seconds since the epoch is used.
 *          Affects the computed backoff_wait_us = rand() * max_backoff_wait_us.
 *
 *      -T
 *          Enable HN-F targeting for per-cpu spinnode allocation.
 *
 *      -H cpu:hnf
 *          For the thread that is to run on the specified cpu, allocate its
 *          spinnode structure's leading cache line to map to the specified hnf
 *          selection value.  This allows controlling the latency/distance
 *          between the CPU and the home node of the spinnode of that CPU.
 *
 *      -Q osq_hnf                         Default: 3
 *          Specifies the selection value of the HN-F on which to allocate the
 *          tail of the optimistic_spin_queue.
 *
 *      -D cpu:delay0:delay1:delay2:delay3:delay4
 *          Specifies delay parameter for each of the 4 calls to CPU_RELAX()
 *          in osq_lock.  See comment at at the bottom of this file for
 *          where the calls are in function outline.  Note: CPU_RELAX is
 *          re-implemented in osq_lock.h, so cpu_relax from
 *          ext/linux/include/lk_atomics.h is not being used.
 */

#ifndef __LINUX_OSQ_LOCK_H
#define __LINUX_OSQ_LOCK_H


#define USE_HNF_TARGETED_ALLOC  // enable to target placement of lockvars
#define OSQ_LOCK_COUNT_LOOPS    // enable to count loop iterations


/* redefine initialize_lock and parse_test_args with local functions */
#ifdef initialize_lock
#undef initialize_lock
#endif

#ifdef parse_test_args
#undef parse_test_args
#endif

#ifdef USE_HNF_TARGETED_ALLOC
#define initialize_lock(num_threads, pinorder) osq_lock_init(num_threads, pinorder)
#else
#define initialize_lock(lock, num_threads) osq_lock_init(lock, num_threads, NULL)
#endif


#define parse_test_args(args, argc, argv) osq_parse_args(args, argc, argv)

#include <inttypes.h>
#include <stdbool.h>
#include "atomics.h"
#include "lk_atomics.h"
#include "lk_cmpxchg.h"
#include "lk_barrier.h"

#define ATOMIC_INIT(i)    { (i) }


#ifdef OSQ_LOCK_COUNT_LOOPS
#define INCREMENT_COUNTER(x) ((*(x))++)
#else
#define INCREMENT_COUNTER(x)
#endif

#define MAX_NUM_CPUS    128

// cpu_to_hnf_map[] maps CPU number to the HN-F's hashed index of the RN-F SAM
// table.  The mapping is system dependent, possibly proprietary, which is why
// this version of the code does not specify an initialization of the table.
// ***** The implicit initial value of 0 is NOT optimal for any system. *****
// The mapping can be changed using one or more "-H cpu:hnf" flags.

unsigned char cpu_to_hnf_map[MAX_NUM_CPUS];

long cpu_to_sleep_time_us[MAX_NUM_CPUS] = { [ 0 ... (MAX_NUM_CPUS-1) ] = -1 };    // change using -S cpu:sleep_us



#define CPU_RELAX_PARAMETERIZED_DELAY

#ifdef CPU_RELAX_PARAMETERIZED_DELAY
long cpu_to_relax[MAX_NUM_CPUS][4] = {
    [ 0 ... (MAX_NUM_CPUS-1) ] = { 1, 1, 1, 1 },
     // [0] = osq_lock   - fast-path loop
     // [1] = osq_lock   - unqueue step A loop
     // [2] = osq_lock   - unqueue step B osq_wait_next loop
     // [3] = osq_unlock - osq_wait_next loop
};

static inline void CPU_RELAX (long relax) {
#if defined(__x86_64__)
        while (relax--)
            asm volatile ("pause" : : : "memory" );
#elif defined (__aarch64__) && defined(RELAX_IS_UDELAY)
        __udelay(relax);
#elif defined (__aarch64__) && defined(RELAX_IS_NDELAY)
        __ndelay(relax);
#elif defined (__aarch64__) && defined(RELAX_IS_ISBYIELD)
        while (relax--)
            asm volatile ("isb ; yield" : : : "memory" );
#elif defined (__aarch64__) && defined(RELAX_IS_ISB14)
        while (relax--)
            asm volatile ("isb #14" : : : "memory" );
#elif defined (__aarch64__) && defined(RELAX_IS_ISB)
        while (relax--)
            asm volatile ("isb" : : : "memory" );
#elif defined (__aarch64__) && defined(RELAX_IS_YIELD)
        while (relax--)
            asm volatile ("yield" : : : "memory" );
#elif defined (__aarch64__) && defined(RELAX_IS_NOP)
        while (relax--)
            asm volatile ("nop" : : : "memory");
#elif defined (__aarch64__) && defined(RELAX_IS_EMPTY)
        while (relax--)
            asm volatile ("" : : : "memory");
#endif
}
#else
#define CPU_RELAX(D) cpu_relax()
#endif


/*
 * An MCS like lock especially tailored for optimistic spinning for sleeping
 * lock implementations (mutex, rwsem, etc).
 *
 * Using a single mcs node per CPU is safe because sleeping locks should not be
 * called from interrupt context and we have preemption disabled while
 * spinning.
 *
 * The original version of this code used 128-bytes alignment to eliminate
 * false sharing for some earlier Armv8 CPUs that had 128-byte cache line
 * lengths.  The HN-F targeting code has not been tested on those CPUs,
 * because they do not use a CMN interconnect.
 */

//#define SPIN_NODE_ALIGNMENT 128UL
#define SPIN_NODE_ALIGNMENT 64UL

struct optimistic_spin_node {
    struct optimistic_spin_node *next;
    struct optimistic_spin_node *prev;
    long locked; /* 1 if lock acquired */
    long cpu; /* encoded CPU # + 1 value */
    unsigned long sleep_us; /* random sleep in us */
    unsigned long sleep_blackhole;
#ifdef USE_HNF_TARGETED_ALLOC
    long pad[2];
    // ----- 64 byte offset should be at this position -----
    long padmore[4096/sizeof(long)];        // XXX: need the 4096 to actually be page size.
}; // HNF targeting doesn't need to use struct alignment attribute
#else
} __attribute__ ((aligned (SPIN_NODE_ALIGNMENT)));
#endif

struct optimistic_spin_queue {
    /*
     * Stores an encoded value of the CPU # of the tail node in the queue.
     * If the queue is empty, then it's set to OSQ_UNLOCKED_VAL.
     */
    atomic_t tail;
} __attribute__ ((aligned (SPIN_NODE_ALIGNMENT)));

/* 0 means thread unlocked, 1~N represents each individual thread on core 1~N */
#define OSQ_UNLOCKED_VAL (0)

/*
 * maximum backoff sleep time in microseconds (default 0us, no sleep)
 * linux kernel scheduling intrinsic delay is less than 7us, however
 * we need to tune this parameter for different machines.
 * http://www.brendangregg.com/blog/2017-03-16/perf-sched.html
 */
#define MAX_BACKOFF_SLEEP_US 0

/*
 * Default unqueue retry times, most system spins at least 500~1000 times
 * before unqueue from optimistic_spin_queue. Default large value simply
 * disables unqueue path and make osq_lock more like mcs_queue_spinlock.
 */
#define DEFAULT_UNQUEUE_RETRY 2000000000

/* Init macro and function. */
#define OSQ_LOCK_UNLOCKED { ATOMIC_INIT(OSQ_UNLOCKED_VAL) }

#define GLOBAL_OSQ_TARGETED_HNF


/* Newly added global variables used by osq_lock algorithm */
static long long unqueue_retry;
static long long max_sleep_us;
#ifdef GLOBAL_OSQ_TARGETED_HNF
static unsigned long global_osq_hnf = 3;
static struct optimistic_spin_queue * pglobal_osq;
#else
static struct optimistic_spin_queue global_osq;
#endif
static struct optimistic_spin_node *global_osq_nodepool_ptr;
static unsigned int srand_seed = 0;
static unsigned int use_hnf_targeting = 0;

/* Newly added additional tuning parameters for optional backoff sleep */
static void osq_parse_args(test_args unused, int argc, char** argv) {
    int i = 0;
    char *endptr;
    unqueue_retry = DEFAULT_UNQUEUE_RETRY;
    max_sleep_us = MAX_BACKOFF_SLEEP_US;

    /* extended options retrieved after '--' operator */
    while ((i = getopt(argc, argv, "u:s:S:R:H:TQ:D:")) != -1)
    {
        switch (i) {
          case 'u':         // maximum number of spin retries before unqueue path
            errno = 0;
            unqueue_retry = strtoll(optarg, &endptr, 10);
            if ((errno == ERANGE && (unqueue_retry == LONG_LONG_MAX))
                    || (errno != 0 && unqueue_retry == 0) || endptr == optarg) {
                fprintf(stderr, "unqueue_retry: value unsuitable "
                                "for 'long long int'\n");
                exit(1);
            }
            break;

          case 's':         // maximum interval between lock_acquire retries
            errno = 0;
            max_sleep_us = strtoll(optarg, &endptr, 10);
            if ((errno == ERANGE && (max_sleep_us == LONG_LONG_MAX))
                    || (errno != 0 && max_sleep_us == 0) || endptr == optarg) {
                fprintf(stderr, "max_sleep_us: value unsuitable "
                                "for 'long long int'\n");
                exit(1);
            } else if (max_sleep_us < 0) {
                fprintf(stderr, "max_sleep_us must be a positive integer.\n");
                exit(1);
            }
            break;

          case 'D':         // per-cpu cpu_relax delay/iterations
            {
                char * pc = NULL;
                char * saveptr;
                size_t cpu;
                long relax;
//                printf("-D optarg = %s\n", optarg);

                pc = strtok_r(optarg, ",:", &saveptr);

                if (pc == NULL) {
                    fprintf(stderr, "expected -D %s to be in the form cpu:relax\n", optarg);
                    exit(-1);
                }

                cpu = strtol(pc, NULL, 0);

                if (cpu > (MAX_NUM_CPUS-1)) {
                    fprintf(stderr, "-D %s cpu is greater than %u, more than supported\n", optarg, MAX_NUM_CPUS-1);
                    exit(-1);
                }

                for (size_t i = 0; i < 4; i++) {
                    pc = strtok_r(NULL, ",:", &saveptr);

                    if (pc == NULL && i == 0) {
                        fprintf(stderr, "expected -D %s to be in the form cpu:delay0[,delay1[,...]], did not find the delay parts\n", optarg);
                        exit(-1);
                    }

                    if (pc == NULL && i >= 0) {
                        break;;
                    }

                    relax = strtol(pc, NULL, 0);

//                    printf("setting cpu_to_relax[cpu=%zu][%zu] => %zu us\n", cpu, i, relax);

                    cpu_to_relax[cpu][i] = relax;
                }
            }
            break;

          case 'S':         // per-cpu unqueue-to-acquire interval wait-time in microseconds
            {
                char * pc = NULL;
                char * saveptr;
                size_t cpu;
                unsigned long sleep_time_us;
//                printf("-S optarg = %s\n", optarg);

                pc = strtok_r(optarg, ",:", &saveptr);

                if (pc == NULL) {
                    fprintf(stderr, "expected -S %s to be in the form cpu:sleep_time\n", optarg);
                    exit(-1);
                }

                cpu = strtoul(pc, NULL, 0);

                if (cpu > (MAX_NUM_CPUS-1)) {
                    fprintf(stderr, "-S %s cpu is greater than %u, more than supported\n", optarg, MAX_NUM_CPUS-1);
                    exit(-1);
                }

                pc = strtok_r(NULL, ",:", &saveptr);

                if (pc == NULL) {
                    fprintf(stderr, "expected -S %s to be in the form cpu:sleep_time_us, did not find the sleep_time_us part\n", optarg);
                    exit(-1);
                }

                sleep_time_us = strtoul(pc, NULL, 0);

//                printf("setting cpu_to_sleep_time_us[cpu=%zu] => %zu us\n", cpu, sleep_time_us);

                cpu_to_sleep_time_us[cpu] = sleep_time_us;
            }
            break;

          case 'R':         // specify random seed for rand()
            srand_seed = strtoumax(optarg, NULL, 0);
            break;

	      case 'T':         // use HN-F targeting
            // Need to be root so that pagemaps can be read to determine physical address
            if (geteuid() != 0) {
                fprintf(stderr, "lh_osq_lock: -T was specified but geteuid was not 0 (root).  Use sudo?\n");
                exit(-1);
            }

            use_hnf_targeting = 1;
            break;

          case 'Q':         // HN-F selection value to use for global_osq
            global_osq_hnf = strtoul(optarg, NULL, 0);
            break;

          case 'H':         // per-cpu HN-F selection value to use for spinnode structure
            {
                char * pc = NULL;
                char * saveptr;
                size_t cpu, hnf;

                pc = strtok_r(optarg, ",:", &saveptr);

                if (pc == NULL) {
                    fprintf(stderr, "expected -H %s to be in the form cpu:hnf\n", optarg);
                    exit(-1);
                }

                cpu = strtoul(pc, NULL, 0);

                if (cpu > (MAX_NUM_CPUS-1)) {
                    fprintf(stderr, "-S %s cpu is greater than %u, more than supported\n", optarg, MAX_NUM_CPUS-1);
                    exit(-1);
                }

                pc = strtok_r(NULL, ",:", &saveptr);

                if (pc == NULL) {
                    fprintf(stderr, "expected -H %s to be in the form cpu:hnf, did not find the hnf part\n", optarg);
                    exit(-1);
                }

                hnf = strtoul(pc, NULL, 0);

                if (hnf > (MAX_NUM_CPUS-1)) {
                    fprintf(stderr, "-H %s hnf is greater than %u, more than support\n", optarg, MAX_NUM_CPUS-1);
                    exit(-1);
                }

//                printf("setting cpu_to_hnf_map[cpu=%zu] => hnf %zu\n", cpu, hnf);

                cpu_to_hnf_map[cpu] = hnf;

            }
            break;

          default:
            fprintf(stderr,
                    "osq_lock additional options after --:\n"
                    "\t[-h print this msg]\n"
                    "\t[-u max spin retries before unqueue, default 2 billion]\n"
                    "\t[-s max unqueue sleep in microseconds, default is 0]\n"
                    "\t[-R specify random seed]\n"
                    "\t[-T use HN-F targeting]\n"
                    "\t[-Q hnf  :  allocate global_osq on specified hnf]\n"
                    "\t[-H cpu:hnf [-H cpu:hnf] ...] change default CPU to HN-F mapping\n"
                    "\t[-D cpu:A,B,C,D ...] change the default cpu_relax() delay parameter\n"
                    "\t[-S cpu:A,B,C,D ...] change the default cpu_relax() delay parameter\n");

            exit(2);
        }
    }
}



/*
    allocate_hnf_targeted_spin_nodes - construct the spin_nodes in linear vaddr space
    such that the paddr of the cache line containing the locking info maps to the HNF
    closest to the cpu.
*/
static void * allocate_hnf_targeted_spin_nodes(unsigned long num_threads, int * pinorder) {

    const size_t tries = 50000;   // XXX: this is oversized
    // the expected maximum number of tries should be maximum of (page_size / cache_line) ** num_hnf

//    printf("pinorder_count = %zu, num_cores = %zu\n", pinorder_count, num_cores);
//    size_t num_threads = (pinorder && pinorder_count) ? pinorder_count : num_cores;

    if (num_threads > MAX_NUM_CPUS) {
        printf("WARNING: this code has not been tested for support of num_threads > 64 (with 4K pages).  64K pages is completely untested as well.\n");
    }

    struct { void * v; int used; } v_ret[tries];
    struct { void * page_vaddr; size_t target_cl_vaddr; } v_remapped[num_threads];

    for (size_t i = 0; i < tries; i++) {
        v_ret[i].v = mmap_any_page();
        v_ret[i].used = 0;
    }

    const size_t base_vaddr = 0xc00000000000UL;     // this is an arbitrarily picked vaddr

    for (size_t thread_number = 0; thread_number < num_threads; thread_number++) {

        // pinorder is an array of CPU numbers listed in the order
        // specified on the command line using -o

        size_t cpu = thread_number; // if pinorder is not specified, use cpu by the implicit thread_number

        if (pinorder) {             // if pinorder IS specified, use the cpus in the order specified
            cpu = pinorder[thread_number];
        }

        size_t hnf = cpu_to_hnf_map[cpu];

        int found = 0;

        for (size_t i = 0; i < tries; i++) {

            // check if the page in v_ret[i] has a CL at the right offset whose paddr maps to the target HNF

            if (v_ret[i].used) { continue; }

            size_t try_cl_vaddr     = ((size_t) v_ret[i].v) + thread_number * erg_bytes;

            if (mem_is_on_target(cmn_hash_select_mode, hnf, (void *) try_cl_vaddr)) {

//                printf("for thread_number = %zu, want cpu = %zu, try %zu has target hnf %zu;  try_cl_vaddr = %lx\n",
//                       thread_number, cpu, i, hnf, try_cl_vaddr);

                v_ret[i].used = 1;

                size_t target_page_vaddr = (base_vaddr + thread_number * sizeof(struct optimistic_spin_node)) & ~((page_size-1));

                printf("trying to remap v_ret[%zu].v = %p to target_page_vaddr = %lx\n", i, v_ret[i].v, target_page_vaddr);

                void * remapped_vaddr = mremap(v_ret[i].v, 0, page_size, MREMAP_FIXED|MREMAP_MAYMOVE, target_page_vaddr);

                if (remapped_vaddr == MAP_FAILED) {
                    printf("mremap failed\n");
                    exit(-1);
                }

                munmap(v_ret[i].v, page_size);  // unmap old mapping

                memset(remapped_vaddr, 0, 1);  // dummy write to force? allocation. maybe this is unnecesssary???? XXX: totally not sure, without it, cl_paddr is 0 sometimes.

                v_remapped[thread_number].page_vaddr = remapped_vaddr;
                v_remapped[thread_number].target_cl_vaddr = (((size_t) remapped_vaddr) & (~((page_size-1)))) | ((thread_number * erg_bytes) % page_size);

                found = 1;

                break;
            }
        }

        if (! found) {
            printf("couldn't find a match for hnf %zu\n", hnf);
            exit (-1);
        }
    }

    // mmap a page at the end for the padding part of the optimistic_spin_node.
    // paddr/hn-f placement does not matter because this memory will not be accessed other than in initialization,
    // which shouldn't be accessing it anyway besides laziness in using memset to do the initialization.

    mmap_a_fixed_page((void *) (((size_t) v_remapped[num_threads-1].page_vaddr) + page_size));

    // unmap unused mappings
    for (size_t i = 0; i < tries; i++) {
        if (v_ret[i].used == 0) {
            munmap(v_ret[i].v, page_size);
        }
    }

    // compute and report the hn-f mapping for each cl node

    struct optimistic_spin_node * _global_osq_nodepool_ptr = (struct optimistic_spin_node *) base_vaddr;

    for (size_t thread_num = 0; thread_num < num_threads; thread_num++) {

        struct optimistic_spin_node * p_ref = _global_osq_nodepool_ptr + thread_num;
        size_t target_cl_vaddr = v_remapped[thread_num].target_cl_vaddr;

//      this is a useful printf, don't remove.
//        printf("&_global_osq_nodepool_ptr[%zu] = %p; target_cl_vaddr = %lx\n",
//                cpu, &_global_osq_nodepool_ptr[cpu], target_cl_vaddr);

        if (target_cl_vaddr != (size_t) p_ref) {
            printf("target_cl_vaddr did not match &_global_osq_nodepool_ptr[cpu]!\n");
            exit (-1);
        }

        size_t cpu = thread_num;  //cpu_to_hnf_map[thread_num].cpu;
        if (pinorder) { cpu = pinorder[thread_num]; }

        size_t target_cl_paddr = get_phys_addr(target_cl_vaddr);
        size_t target_cl_hnf   = return_select_function(cmn_hash_select_mode)(target_cl_paddr);
        printf ("v_remapped[%2zu] (for cpu %2zu) cl_vaddr = %lx  cl_paddr = %lx  target_cl_hnf = %2zu (calculated) expected hnf = %2u %s\n",
                thread_num, cpu, target_cl_vaddr, target_cl_paddr, target_cl_hnf, cpu_to_hnf_map[cpu],
                (target_cl_hnf != cpu_to_hnf_map[cpu]) ? "did not match" : "OK"
            );
    }

    return (void *) _global_osq_nodepool_ptr;
}


/*
 * An MCS like lock especially tailored for optimistic spinning for sleeping
 * lock implementations (mutex, rwsem, etc).
 *
 * Using a single mcs node per CPU is safe because sleeping locks should not be
 * called from interrupt context and we have preemption disabled while
 * spinning.
 */
static inline void osq_lock_init(unsigned long num_threads, int * pinorder)
{
    printf("lh_osq_lock: sizeof(struct optimistic_spin_node) = %zu\n", sizeof(struct optimistic_spin_node));

    for (size_t i = 0; i < MAX_NUM_CPUS; i++) {// should this be num_threads?  probably not because this is before pinorder.
        printf("cpu_to_relax[cpu=>%zu]: %lu %lu %lu %lu\n", i, cpu_to_relax[i][0], cpu_to_relax[i][1], cpu_to_relax[i][2], cpu_to_relax[i][3]);
    }

    size_t size;
#ifdef USE_HNF_TARGETED_ALLOC
    if (use_hnf_targeting) {
        if (cmn_hash_select_mode == 0) {
            printf("osq_lock_init: use_targeted_hnf=1, but cmn_hash_select_mode=0. Use -f selectN (to the left of the --) to choose a mode. Exiting!\n");
            exit(-1);
        }
        size = num_threads * sizeof(struct optimistic_spin_node);
        printf("osq_lock_init: using hnf targeted spinnode allocation for num_threads = %lu, nodepool size bytes = %zu\n", num_threads, size);
        global_osq_nodepool_ptr = allocate_hnf_targeted_spin_nodes(num_threads, pinorder);
    } else  // XXX: this is tricky.  want to be able to gracefully continue witout root
            // if geteuid() returns nonzero (i.e. this thread is not root)
#endif
    {
        printf("osq_lock_init: using aligned_alloc() to allocate global_osq_nodepool\n");

        /*
         * Allocate optimistic_spin_node from heap during main thread initialization.
         * Each cpu core will have its own spinning node, aligned to SPIN_NODE_ALIGNMENT.
         */

        size = (num_threads + 1) * sizeof(struct optimistic_spin_node);    // why is there +1?

        if (size % SPIN_NODE_ALIGNMENT) {
            fprintf(stderr, "lh_osq_lock: ERROR: size = %zu, is not a multiple of %zu\n", size, SPIN_NODE_ALIGNMENT);
            exit(-1);
        }

        global_osq_nodepool_ptr = aligned_alloc(SPIN_NODE_ALIGNMENT, size);
    }

    printf("global_osq_nodepool_ptr = %p\n", global_osq_nodepool_ptr);

    if (global_osq_nodepool_ptr == NULL) exit(errno);

    memset(global_osq_nodepool_ptr, 0, size);

    /*
     * If osq spins more than unqueue_retry times, the spinning cpu may backoff
     * and sleep for 1 ~ 10 microseconds (on average 5 microseconds). Each spinning
     * thread uses a different backoff sleep time, and we can adjust the maximum
		     ^--- where in this code ensures the backoff is different for each thread, just the probability of rand()?
     * sleep time by redefine MAX_BACKOFF_SLEEP_US or tuning via parameter '-s'
     * By default, we disable this sleep (MAX_BACKOFF_SLEEP_US = 0)
     *
     * Note: Avoid assigning random_sleep a negative value, otherwise usleep would
     * have a very large sleep time after implicit casting negative to uint32_t.
		^-- for certain values of max_sleep_us, the +1 causes this to happen on overflow
     */
    if (srand_seed == 0) {
	srand_seed = time(0);
    }

    printf("lh_osq_lock: srand_seed = %u\n", srand_seed);
    srand(srand_seed);

#if 0
    printf("cpu: sleep time (us)\n");
    for (size_t i = 0; i < 64; i++) {
        printf("%zu: %lu\n", i, cpu_to_sleep_time_us[i]);
    }
#endif

    // print the per-thread post-unqueue sleep time
    for (size_t i = 0; i < num_threads; i++) {
        size_t cpu = i;
        if (pinorder) {
            cpu = pinorder[i];
        }
        long sleep_time_us = cpu_to_sleep_time_us[cpu];
        char * random_string = "";
        if (sleep_time_us >= 0) {
            global_osq_nodepool_ptr[i].sleep_us = sleep_time_us;
        } else if (max_sleep_us > 0) {
            global_osq_nodepool_ptr[i].sleep_us = rand() % max_sleep_us + 1;
            random_string = " (randomly selected)";
	}

        printf("thread %zu cpu_to_sleep_time_us[cpu=>%zu] = %lu%s\n",
            i, cpu, global_osq_nodepool_ptr[i].sleep_us, random_string);
    }

    /* Initialize global osq tail indicater to OSQ_UNLOCKED_VAL (0: unlocked) */
#ifdef GLOBAL_OSQ_TARGETED_HNF
    printf("allocating global_osq on global_osq_hnf = %lu\n", global_osq_hnf);
    // XXX: need to get some of these parameters from rest of program instead of
    // tries=512, hard-coded use_mmap=0, hugepagesz=0
    size_t max_num_tries_to_allocate_global_osq = 512;
    pglobal_osq = (struct optimistic_spin_queue *) get_hnf_targeted_lock_memory(erg_bytes, max_num_tries_to_allocate_global_osq, cmn_hash_select_mode, global_osq_hnf, 0, 0);
    atomic_set(&(pglobal_osq->tail), OSQ_UNLOCKED_VAL);
#else
    atomic_set(&global_osq.tail, OSQ_UNLOCKED_VAL);
#endif
}

static void osq_lock_compute_blackhole_interval(unsigned long thread_number, double tickspns, int * pinorder) {
    unsigned long sleep_us = global_osq_nodepool_ptr[thread_number].sleep_us;
    unsigned long sleep_blackhole = sleep_us ? calibrate_blackhole(tickspns * sleep_us * 1000, 0, TOKENS_MAX_HIGH, thread_number) : 0;

    global_osq_nodepool_ptr[thread_number].sleep_blackhole = sleep_blackhole;

    unsigned long cpu = thread_number;
    if (pinorder) { cpu = pinorder[thread_number]; }

    printf("thread %lu cpu %lu: tickspns = %f, sleep_us = %lu, sleep_blackhole = %lu\n",
        thread_number, cpu, tickspns, sleep_us, sleep_blackhole);
}

static inline bool osq_is_locked(struct optimistic_spin_queue *lock)
{
    return atomic_read(&lock->tail) != OSQ_UNLOCKED_VAL;
}

/*
 * Value 0 represents "no CPU" or "unlocked", thus the encoded value will be
 * the CPU number incremented by 1.
 */
static inline int encode_cpu(int cpu_nr)
{
    return cpu_nr + 1;
}

static inline int node_to_cpu(struct optimistic_spin_node *node)
{
    return node->cpu - 1;
}

/*
 * optimistic_spin_node for each cpu is stored linearly in main heap starting
 * from global_osq_nodepool_ptr
 */
static inline struct optimistic_spin_node * cpu_to_node(int encoded_cpu_val)
{
    int cpu_nr = encoded_cpu_val - 1;
    return global_osq_nodepool_ptr + cpu_nr;
}

/*        v--- what does stable mean?
 * Get a stable @node->next pointer, either for unlock() or unqueue() purposes.
 * Can return NULL in case we were the last queued and we updated @lock instead.
 */
#ifdef OSQ_LOCK_COUNT_LOOPS
static inline struct optimistic_spin_node *
osq_wait_next(struct optimistic_spin_queue *lock,		// aka global_osq
          struct optimistic_spin_node *node,			// aka us
          struct optimistic_spin_node *prev,			// aka prev, the previous tail of the queue
          unsigned long thread_number,                          // our thread number
          unsigned long * counter,		                // pointer to loop counter
          long relax_delay)
#else
static inline struct optimistic_spin_node *
osq_wait_next(struct optimistic_spin_queue *lock,		// aka global_osq
          struct optimistic_spin_node *node,			// aka us
          struct optimistic_spin_node *prev,			// aka prev, the previous tail of the queue
          unsigned long thread_number,                          // our thread number
          long relax_delay)
#endif
{
    struct optimistic_spin_node *next = NULL;
    int curr = encode_cpu(thread_number);
    int old;	// (encoded) cpu number of the node that is ahead of us in the queue

    /*
     * If there is a prev node in queue, then the 'old' value will be
     * the prev node's CPU #, else it's set to OSQ_UNLOCKED_VAL since if
     * we're currently last in queue, then the queue will then become empty.
     */
    old = prev ? prev->cpu : OSQ_UNLOCKED_VAL;			// prev is the node ahead of us in queue

    for (;;) {

        if (atomic_read(&lock->tail) == curr &&				// if the tail is us,
            atomic_cmpxchg_acquire(&lock->tail, curr, old) == curr) {	// try to change the tail to 'old'
            /*
             * We were the last queued, we moved @lock back. @prev
             * will now observe @lock and will complete its
             * unlock()/unqueue().
             */
            break;
        }

	// if we are here, then we were not at the tail of the queue (and
	// we failed to update the tail of the queue to the 'old' cpu number),
	// so that means there may be someone after us (i.e. ->next might not be
	// NULL).

        /*
         * We must xchg() the @node->next value, because if we were to
         * leave it in, a concurrent unlock()/unqueue() from
         * @node->next might complete Step-A and think its @prev is
         * still valid.
         *
         * If the concurrent unlock()/unqueue() wins the race, we'll
         * wait for either @lock to point to us, through its Step-B, or
         * wait for a new @node->next from its Step-C.
         */
        if (node->next) {		// this is a poll of our next pointer.
                                // if we have a value in ->next, clear it to NULL and return it.
            next = xchg(&node->next, NULL);  // our ->next is written by osq_lock (lock acquire) or
            if (next)			// unlock/unqueue
                break;
        }

        // if we had no next or our ->next got overwritten by someone else doing
        // unlock/unqueue, run this loop again.

        CPU_RELAX(relax_delay);

        INCREMENT_COUNTER(counter);
    }

    // so what osq_wait_next does is:
    // 1. take ourselves off of the queue tail iff we are the queue tail
    // 2. return our ->next if we have a ->next, and change our ->next to NULL.
    // we don't use node->prev.

    return next;
}

// osq_lock is called by lock_acquire.  returns true if we have acquired the lock.
/* uint64_t *osq is ignored because we use &global_osq instead */
#ifdef OSQ_LOCK_COUNT_LOOPS
static bool osq_lock(uint64_t *osq, unsigned long thread_number,
         unsigned long * osq_lock_wait_next_spins, unsigned long * osq_lock_locked_spins, unsigned long * osq_lock_unqueue_spins)
#else
static bool osq_lock(uint64_t *osq, unsigned long thread_number)
#endif
{
    /* each cpu core has only one thread spinning on one optimistic_spin_node */
    struct optimistic_spin_node *node = global_osq_nodepool_ptr + thread_number;

    /* optimistic_spin_queue points to the current osq tail */
#ifdef GLOBAL_OSQ_TARGETED_HNF
    struct optimistic_spin_queue *lock = pglobal_osq;
#else
    struct optimistic_spin_queue *lock = &global_osq;
#endif
    struct optimistic_spin_node *prev, *next;
    int curr = encode_cpu(thread_number);
    int old;
    long long back_off = 0;

    // one optimistic_spin_node
    node->locked = 0;
    node->next = NULL;
    node->cpu = curr;

    /*
     * We need both ACQUIRE (pairs with corresponding RELEASE in
     * unlock() uncontended, or fastpath) and RELEASE (to publish
     * the node fields we just initialised) semantics when updating
     * the lock tail.
     */
    old = atomic_xchg(&lock->tail, curr);   // _unconditionally_ put our cpu into tail.
    if (old == OSQ_UNLOCKED_VAL)			// if tail _was_ empty, then we have acquired the lock.
        return true;

    // if we are here, then there is already someone else ahead of us in the queue.

    // prev is a pointer to the node of the cpu 'old' that obtained the lock before us

    prev = cpu_to_node(old);		// compute node of the previous tail of queue, store as node->prev
    node->prev = prev;				// node -> prev points to node of the cpu that is ahead of us in the queue

    /*
     * osq_lock()            unqueue
     *
     * node->prev = prev        osq_wait_next()
     * WMB                      MB
     * prev->next = node        next->prev = prev // unqueue-C
     *
     * Here 'node->prev' and 'next->prev' are the same variable and we need
     * to ensure these stores happen in-order to avoid corrupting the list.
     */
    smp_wmb();

    // Now we make them point to us.  The atomic_xchg above for the tail
    // will return a unique cpu number 'old'.

    WRITE_ONCE(prev->next, node);		// in a previous tail node of queue, set its ->next to us. (we set our prev to it above)

    // so now the node that was ahead of us in the queue is doubly-linked to us

    /*
     * Normally @prev is untouchable after the above store; because at that
     * moment unlock can proceed and wipe the node element from stack.
     *
     * However, since our nodes are static per-cpu storage, we're
     * guaranteed their existence -- this allows us to apply
     * cmpxchg in an attempt to undo our queueing.
     */

#if defined(USE_SMP_COND_LOAD_RELAXED)
        /*
         * Wait to acquire the lock or cancellation. Note that need_resched()
         * will come with an IPI, which will wake smp_cond_load_relaxed() if it
         * is implemented with a monitor-wait. vcpu_is_preempted() relies on
         * polling, be careful.
         */

    // INCREMENT_COUNTER(osq_lock_locked_spins) is embedded in smp_cond_load_relaxed macro. see include/lk_atomics.h


    if (smp_cond_load_relaxed(&node->locked, VAL || (++back_off > unqueue_retry)))
	return true;
#else
    while (!READ_ONCE(node->locked)) {		// poll our node's locked variable until it is set.

        // XXX  Q: how does our node->locked get set?
        //      A: when someone else unlocks themselves and we are their ->next
        //         (or we BECOME their ->next through their waiting in
        //         osq_wait_next), // <-- how exactly? not through osq_unlock because that writes NULL into ->next
        //         they set ->next->locked=1 atomically upon return to osq_unlock()

        /*
         * TODO: Need to better emulate kernel rescheduling in user space.
         * Because we cannot use need_resched() in user space, we simply
         * add a upper limit named unqueue_retry (-s max_backoff_wait_us)
         * to mimic need_resched() or the per-cpu fixed backoff_wait_us (-S
         * cpu:backoff_wait_us).  If this limit has been exceeded by
         * back_off times, we will jump to unqueue path and remove the
         * spinning node from global osq.
         */

        /*
         * If we need to reschedule bail... so we can block.
         * Use vcpu_is_preempted() to avoid waiting for a preempted
         * lock holder.
         */
        //if (need_resched() || vcpu_is_preempted(node_to_cpu(node->prev)))

        if (++back_off > unqueue_retry) /* DEFAULT_UNQUEUE_RETRY is 2 billion */
            goto unqueue;

        CPU_RELAX(cpu_to_relax[thread_number][0]);

        INCREMENT_COUNTER(osq_lock_locked_spins);
    }
    return true;		// we got the lock

unqueue:
#endif

    //
    // If we are here, we polled node->locked for unqueue_retry number of times
    // and found it was always 0, i.e. we never won the lock. so we are
    // attempting to give up.
    //
    // step A = remove us from prev's ->next if it points to us.  if we somehow
    // get granted ->locked=1, then abort at this step (because we obtained the
    // lock).  for loop continues if prev's -> next is not us, or we are not
    // serendiptiously locked.
    //
    // step B = our prev's ->next WAS pointed to us and we cleared it to NULL
    // successfully in step A.  Now, using osq_wait_next, we repeatedly try to
    // clear us from the tail of the lock qeueue if we are the tail of the queue
    // (in which case osq_wait_next returns NULL and we return false from
    // osq_lock()), or we clear our ->next and osq_wait_next returns that
    // pointer that was in our ->next for step C.
    //
    // step C = assign  next->prev <---  prev, and assign prev->next to next,
    // basically removing us from the link.  finally return false.
    //




    /*
     * Step - A  -- stabilize @prev
     *
     * Undo our @prev->next assignment; this will make @prev's
     * unlock()/unqueue() wait for a next pointer since @lock points to us
     * (or later).
     */

    for (;;) {					// make ourselves not in the queue (prev does not point to us)
        if (prev->next == node &&				// if prev->next is us (node)
            cmpxchg(&prev->next, node, NULL) == node)		// then try to clear it out
            break;

        /*
         * We can only fail the cmpxchg() racing against an unlock(),
         * in which case we should observe @node->locked becoming
         * true.
         */
        if (smp_load_acquire(&node->locked))	// but if we failed to write NULL into prev->next because prev->next did not point to us, see if we got locked, and if so, return true,
            return true;			// we got the lock because someone unlocked and made us locked by writing 1 to our ->locked.

        CPU_RELAX(cpu_to_relax[thread_number][1]);			// otherwise wait a bit

        INCREMENT_COUNTER(osq_lock_unqueue_spins);

        /*
         * Or we race against a concurrent unqueue()'s step-B, in which
         * case its step-C will write us a new @node->prev pointer.
         */
        prev = READ_ONCE(node->prev);
    }

    /*
     * Step - B -- stabilize @next
     *
     * Similar to unlock(), wait for @node->next or move @lock from @node
     * back to @prev.
     */

    // if we are here, then we already cleared ourselves from the prev->next.
    // osq_wait_next will take us off queue tail and return our ->next.

#ifdef OSQ_LOCK_COUNT_LOOPS
    next = osq_wait_next(lock, node, prev, thread_number,
                         osq_lock_wait_next_spins,
                         cpu_to_relax[thread_number][2]);
#else
    next = osq_wait_next(lock, node, prev, thread_number,
                         cpu_to_relax[thread_number][2]);
#endif
    if (!next)			// if there is nothing pointed to by our next pointer, we removed from tail?
        return false;

    /*
     * Step - C -- unlink
     *
     * @prev is stable because its still waiting for a new @prev->next
     * pointer, @next is stable because our @node->next pointer is NULL and
     * it will wait in Step-A.
     */

    WRITE_ONCE(next->prev, prev);		// we did not get the lock, and we were not at end of queue
    WRITE_ONCE(prev->next, next);		// so take us out of the queue

    return false;
}


#define PRINT_HNF_TAG(x)  \
    printf("osq_unlock: " __stringify(x) " = %p, select6(" __stringify(x)  ") = %lu\n", (x), calculate_select6(get_phys_addr((uintptr_t) (x))));

/* uint64_t *osq is ignored because we use &global_osq instead */
//static void osq_unlock(uint64_t *osq, unsigned long thread_number, unsigned long * osq_unlock_wait_next_spins)

#ifdef OSQ_LOCK_COUNT_LOOPS
void osq_unlock(uint64_t *osq, unsigned long thread_number, unsigned long * osq_unlock_wait_next_spins)
#else
void osq_unlock(uint64_t *osq, unsigned long thread_number)
#endif
{
    /* optimistic_spin_queue stores the current osq tail globally */
#ifdef GLOBAL_OSQ_TARGETED_HNF
    struct optimistic_spin_queue *lock = pglobal_osq;
#else
    struct optimistic_spin_queue *lock = &global_osq;
#endif
    struct optimistic_spin_node *node, *next;
    int curr = encode_cpu(thread_number);

    /*
     * Fast path for the uncontended case.
     */

    // pointer, old (expected value), new (value to write if *pointer == old), and returns old if it is a match matches
    // basically this clears the tail if this node is the tail.  If this node isn't the tail, then there is more processing.

    if (atomic_cmpxchg_release(&lock->tail, curr,
                      OSQ_UNLOCKED_VAL) == curr) {
        return;
    }

    /*
     * Second most likely case.
     * If there is a next node, notify it.
     */
    node = global_osq_nodepool_ptr + thread_number;     // node = us
    next = xchg(&node->next, NULL);	    // unconditionally clear our node->next.
    if (next) {                         // if there was a node->next, set its locked to 1.
        WRITE_ONCE(next->locked, 1);    //     (i.e. the next node now has the lock.)
        return;
    }

    /*
     * Wait for another stable next, or get NULL if the queue is empty.
     */

    // if we are here, then both:
    //
    // 1. (fast path) we were not at the tail of the queue (if we were the tail
    // of the queue, we would have emptied the queue to do this osq_unlock),
    // which means our node->next might point to someone else behind us in the
    // queue.
    //
    //   - AND -
    //
    // 2. (second path) we did not have a next node pointer (our ->next was
    // NULL). (If we did have a next pointer, we would have cleared our ->next
    // to it and tell ->next it has the lock by writing next->locked=1.) but if
    // we did not have a next pointer and we were not at the tail of the queue,
    // that means our ->next is volatile and someone may soon write to it (i.e.
    // set our ->next to another node).
    //
    // XXX:  so how does the node that was node->next know its prev is no longer us?
    // XXX:  what sets this node's ->next to be non-NULL?
    //
    // so we now call osq_wait_next to (repeatedly) wait either of we are the
    // tail (and then remove ourselves from the tail of the queue) OR if at some
    // point our ->next comes valid (inside osq_wait_next), we set our struct's
    // ->next pointer to NULL and make 'next' to be locked (below).  Basically
    // the same as above, but in a loop with a cpu_relax() in between
    // iterations.

#ifdef OSQ_LOCK_COUNT_LOOPS
    next = osq_wait_next(lock, node, NULL, thread_number,
                         osq_unlock_wait_next_spins,
                         cpu_to_relax[thread_number][3]);
#else
    next = osq_wait_next(lock, node, NULL, thread_number,
                         cpu_to_relax[thread_number][3]);
#endif
    if (next)
        WRITE_ONCE(next->locked, 1);
}


/* standard lockhammer lock_acquire and lock_release interfaces */
//static unsigned long lock_acquire (uint64_t *lock, unsigned long threadnum) __attribute__((noinline));

//__attribute__((noinline))
//static // disable static to prevent inlining.


#ifdef OSQ_LOCK_COUNT_LOOPS
unsigned long osq_lock_acquire (uint64_t *lock, unsigned long threadnum, unsigned long * osq_lock_wait_next_spins, unsigned long * osq_lock_locked_spins, unsigned long * osq_lock_unqueue_spins, unsigned long * osq_lock_acquire_backoffs)
#else
unsigned long osq_lock_acquire (uint64_t *lock, unsigned long threadnum)
#endif
{
    /*
     * Note: The linux kernel implements additional mutex slow path in mutex.c
     * __mutex_lock_common() function. We will create another workload which
     * combines osq_lock and mutex_lock_common. This workload only benchmarks
     * osq_lock itself. The osq_lock is different from mcs_queue_spinlock
     * because of tunable unqueue path and backoff sleep time.
     */
    unsigned long sleep_blackhole = global_osq_nodepool_ptr[threadnum].sleep_blackhole;

//    PRINT_HNF_TAG(&((global_osq_nodepool_ptr + threadnum)->random_sleep));
#ifdef OSQ_LOCK_COUNT_LOOPS
    while (!osq_lock(lock, threadnum, osq_lock_wait_next_spins, osq_lock_locked_spins, osq_lock_unqueue_spins)) {
#else
    while (!osq_lock(lock, threadnum)) {
#endif
        /*
         * If still cannot acquire the lock after spinning for unqueue_retry
         * times, try to backoff for a predetermined number of microseconds
         * specified by parameter '-s'.  The default maximum sleep time is 0us.
         * Then attempt to reacquire the lock again infinitely until success.
         *
         * This behaves similar to kernel mutex with fine tuning sleep time.
         */

        blackhole(sleep_blackhole);

        INCREMENT_COUNTER(osq_lock_acquire_backoffs);
    }
    return 1;
}


#ifdef OSQ_LOCK_COUNT_LOOPS
static inline void osq_lock_release (uint64_t *lock, unsigned long threadnum, unsigned long * osq_unlock_wait_next_spins)
{
    osq_unlock(lock, threadnum, osq_unlock_wait_next_spins);
}
#else
static inline void osq_lock_release (uint64_t *lock, unsigned long threadnum)
{
    osq_unlock(lock, threadnum);
}
#endif

#endif /* __LINUX_OSQ_LOCK_H */



/////////////////////////////////////////////////////////
/*  outline of osq_lock and where the instrumented counters reside cpu_to_relax[]

	osq_lock_acquire
		osq_lock
            fast-path loop:
			    while ! node->locked        [[[ spins on our own node ]]]
                    osq_lock_locked_spins++     // [0] = osq_lock - fast-path loop
			unqueue path:
                    step A loop:
                        looping on prev->next to be us to clear it AND we are ! node->locked
                            [[[ spins on prev->next and our own node->locked ]]]]
                            osq_lock_unqueue_spins++    // [1] = osq_lock - unqueue step A loop
                        go to step B iff we were able to clear prev->next
                    step B loop:
                        inside osq_wait_next    [[[ spins on global tail and our own node->next ]]]
                            if we are the (tail) end of the queue, remove us from the queue. return false.
                            if we are not the end of the queue, try to clear our -> next pointer.
                            if our -> next pointer was clear:
                                osq_lock_wait_next_spins++   // [2] = osq_lock - unqueue step B osq_wait_next loop


	osq_lock_release
		osq_unlock
			osq_wait_next
				inside osq_wait_next    [[[ spins on global tail and our own node->next ]]]
                    if we are the (tail) end of the queue, remove us from the queue.
                    if we are not the end of the queue, try to clear our ->next pointer.
                    if our -> next pointer was clear:
                        osq_unlock_wait_next_spins++    // [3] = osq_unlock - osq_wait_next loop

   */


/* vim: set tabstop=4 shiftwidth=4 softtabstop=4 expandtab: */
