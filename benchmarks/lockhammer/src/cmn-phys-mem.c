// ------------------------------------------------------------------------
// CHI target select hash calculations

#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "pagemap.h"

#define PHYS_ADDR_BITS 48

#define CLEAR_CACHELINE_BITS (~((1<<6) -1))

const unsigned long PHYS_ADDR_MASK =
	((1ULL << PHYS_ADDR_BITS) - 1) & CLEAR_CACHELINE_BITS;


// 6-bit hash for 64x HN-F system
unsigned long calculate_select6(unsigned long x) {

	const unsigned long BITMASK = 0x1041041041041041UL;

	x &= PHYS_ADDR_MASK;

	unsigned long select =
		(__builtin_parityl(x & (BITMASK << 6)) << 0)|
		(__builtin_parityl(x & (BITMASK << 7)) << 1)|
		(__builtin_parityl(x & (BITMASK << 8)) << 2)|
		(__builtin_parityl(x & (BITMASK << 9)) << 3)|
		(__builtin_parityl(x & (BITMASK <<10)) << 4)|
		(__builtin_parityl(x & (BITMASK <<11)) << 5);

	return select;
}

// 5-bit hash for 32x HN-F system
unsigned long calculate_select5(unsigned long x) {
	const unsigned long BITMASK = 0x1084210842108421UL;

	x &= PHYS_ADDR_MASK;

	unsigned long select =
		(__builtin_parityl(x & (BITMASK << 6)) << 0)|
		(__builtin_parityl(x & (BITMASK << 7)) << 1)|
		(__builtin_parityl(x & (BITMASK << 8)) << 2)|
		(__builtin_parityl(x & (BITMASK << 9)) << 3)|
		(__builtin_parityl(x & (BITMASK <<10)) << 4);

	return select;
}


// 4-bit hash for 16x HN-F system
unsigned long calculate_select4(unsigned long x) {
	const unsigned long BITMASK = 0x1111111111111111UL;

	x &= PHYS_ADDR_MASK;

	unsigned long select =
		(__builtin_parityl(x & (BITMASK << 6)) << 0)|
		(__builtin_parityl(x & (BITMASK << 7)) << 1)|
		(__builtin_parityl(x & (BITMASK << 8)) << 2)|
		(__builtin_parityl(x & (BITMASK << 9)) << 3);

	return select;
}

// 3-bit hash for 8x HN-F system
unsigned long calculate_select3(unsigned long x) {
	const unsigned long BITMASK = 0x9249249249249249UL;

	x &= PHYS_ADDR_MASK;

	unsigned long select =
		(__builtin_parityl(x & (BITMASK << 6)) << 0)|
		(__builtin_parityl(x & (BITMASK << 7)) << 1)|
		(__builtin_parityl(x & (BITMASK << 8)) << 2);

	return select;
}


// 2-bit hash for 4x HN-F system
unsigned long calculate_select2(unsigned long x) {
	const unsigned long BITMASK = 0x5555555555555555UL;

	x &= PHYS_ADDR_MASK;

	unsigned long select =
		(__builtin_parityl(x & (BITMASK << 6)) << 0)|
		(__builtin_parityl(x & (BITMASK << 7)) << 1);

	return select;
}

// 1-bit hash for 2x HN-F system
unsigned long calculate_select1(unsigned long x) {

	x &= PHYS_ADDR_MASK;

	return __builtin_parityl(x);
}


int parse_cmn_hash_select_mode(const char * optarg) {

        if (0 == strcmp(optarg, "select6")) {
                return 6;
        }
        if (0 == strcmp(optarg, "select5")) {
                return 5;
        }
        if (0 == strcmp(optarg, "select4")) {
                return 4;
        }
        if (0 == strcmp(optarg, "select3")) {
                return 3;
        }
        if (0 == strcmp(optarg, "select2")) {
                return 2;
        }
        if (0 == strcmp(optarg, "select1")) {
                return 1;
        }
        printf("unknown select function %s\n", optarg);
        exit(-1);
        // shouldn't get here.
        return 0;
}


uintptr_t get_phys_addr(uintptr_t vaddr) {

     uintptr_t paddr = 0;

     static unsigned long PAGESIZE = 1;
     static unsigned long PAGE_MASK = 1;

     static unsigned long last_vpage = 1;	// ensure mismatch
     static unsigned long last_ppage = 1;

     if (PAGESIZE == 1) {
          PAGESIZE = sysconf(_SC_PAGESIZE);
          PAGE_MASK = ~(PAGESIZE - 1);
     }

     //	printf("pid = %d, vaddr = %zx\n", pid, vaddr);
     //	printf("vaddr & PAGE_MASK = %zx\n", vaddr & PAGE_MASK);
     //	printf("last_vpage        = %zx\n", last_vpage);

     if ((vaddr & PAGE_MASK) == last_vpage) {
          return last_ppage | (vaddr & ~PAGE_MASK);
     }

    if (geteuid() == 0) {
         pid_t pid = getpid();	// this process ID
         if (lkmc_pagemap_virt_to_phys_user(&paddr, pid, vaddr)) {
              fprintf(stderr, "error: virt_to_phys_user\n");
              return EXIT_FAILURE;
         }
         last_vpage = vaddr & PAGE_MASK;
         last_ppage = paddr & PAGE_MASK;

         return paddr;
    }

    fprintf(stderr, "didn't expect to get here in get_phys_addr, not running as root?\n");
    exit(-1);

    return -1;
}


// return 1 if p points to a physaddr that would map to HN-F specified by target_select for select_mode
int mem_is_on_target(int select_mode, unsigned long target_select, void * p) {
        unsigned long vaddr = (uintptr_t) p;
        size_t paddr = get_phys_addr(vaddr);
        unsigned long select;
        switch (select_mode) {
               case 6: select = calculate_select6(paddr); break;
               case 5: select = calculate_select5(paddr); break;
               case 4: select = calculate_select4(paddr); break;
               case 3: select = calculate_select3(paddr); break;
               case 2: select = calculate_select2(paddr); break;
               case 1: select = calculate_select1(paddr); break;
               default:  fprintf(stderr, "unknown select mode %d\n", select_mode); exit(-1);
        }
        return select == target_select;
}
