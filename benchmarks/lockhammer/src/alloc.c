#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <linux/mman.h>
#include <string.h>

#include "alloc.h"

void * do_alloc(size_t length, int use_hugepages, size_t nonhuge_alignment) {

	if (use_hugepages != HUGEPAGES_NONE) {
		int hugepage_size_flag = HUGEPAGES_DEFAULT;
		switch (use_hugepages) {
			case HUGEPAGES_64K:
				hugepage_size_flag = MAP_HUGE_64KB;
				break;
			case HUGEPAGES_2M:
				hugepage_size_flag = MAP_HUGE_2MB;
				break;
			case HUGEPAGES_32M:
				hugepage_size_flag = MAP_HUGE_32MB;
				break;
			case HUGEPAGES_512M:
				hugepage_size_flag = MAP_HUGE_512MB;
				break;
			case HUGEPAGES_1G:
				hugepage_size_flag = MAP_HUGE_1GB;
				break;
			case HUGEPAGES_16G:
				hugepage_size_flag = MAP_HUGE_16GB;
				break;
		}
		void * mmap_ret = mmap(NULL, length,
				       PROT_READ|PROT_WRITE,
				       MAP_PRIVATE|MAP_ANONYMOUS|MAP_HUGETLB|MAP_POPULATE|hugepage_size_flag,
				       -1, 0);

		if (mmap_ret == MAP_FAILED) {
			printf("mmap returned %p (MAP_FAILED). Exiting!\n"
			       "You probably need to allocate hugepages. Try:\n"
			       " sudo apt-get install hugepages\n"
			       " sudo hugeadm --create-global-mounts\n"
			       " sudo hugeadm --pool-pages-max DEFAULT:+1000\n"
			       "(Only the last line is needed after a reboot.)\n"
			       "Or, no pages of the requested hugepage size are available.\n",
			       mmap_ret);
			exit(-1);
		}

		return mmap_ret;

	}

	void * p;

	int ret = posix_memalign((void **) &p, nonhuge_alignment, length);

	if (ret) {
		printf("posix_memalign returned %d, exiting\n", ret);
		exit(-1);
	}

	// prefault
	memset(p, 1, length);

	return p;
}

const char * hugepagesz_map[] = {
	[HUGEPAGES_NONE]    = "none",
	[HUGEPAGES_DEFAULT] = "default",
	[HUGEPAGES_64K]     = "64k",
	[HUGEPAGES_2M]      = "2m",
	[HUGEPAGES_32M]     = "32m",
	[HUGEPAGES_512M]    = "512m",
	[HUGEPAGES_1G]      = "1g",
	[HUGEPAGES_16G]     = "16g"
};


int parse_hugepagesz(const char * hugepagesz_str) {

	for (size_t i = 0; i < HUGEPAGES_MAX_ENUM; i++) {
		if (0 == strcasecmp(hugepagesz_str, hugepagesz_map[i])) {
			return i;
		}
	}

	if (0 == strcasecmp(hugepagesz_str, "help")) {
		printf("supported values for -M hugepagesz  (does not check if system supports it or not):\n");
		for (size_t i = 0; i < HUGEPAGES_MAX_ENUM; i++) {
			printf("%s\n", hugepagesz_map[i]);
		}
		exit(-1);
	}

	fprintf(stderr, "could not parse hugepagesz_str %s, assuming HUGEPAGES_DEFAULT\n", hugepagesz_str);

	return HUGEPAGES_DEFAULT;
}

const char * get_hugepagesz_by_enum(size_t v) {
	if (v >= HUGEPAGES_MAX_ENUM) {
		return "";
	}
	return hugepagesz_map[v];
}
