# override keyword overwrites make command-line option LSE_ENABLE=y, therefore it has been removed
#CFLAGS += -g -O3 -fno-lto -Wall -I. -I./include -I../../ext/mysql/include -I../../ext/linux/include -I../../ext/tbb/include -I../../ext/sms/base
CFLAGS += -g -O2 -fno-lto -Wall -I. -I./include -I../../ext/mysql/include -I../../ext/linux/include -I../../ext/tbb/include -I../../ext/sms/base -fno-stack-protector -fstack-protector-explicit
# stack protector stuff causes some extra memory accesses????
#CFLAGS += -g -O2 -fno-lto -Wall -I. -I./include -I../../ext/mysql/include -I../../ext/linux/include -I../../ext/tbb/include -I../../ext/sms/base


CC = gcc

# Debug level
ifneq ($(DEBUG_LEVEL),)
ifeq ($(shell test $(DEBUG_LEVEL) -gt 0; echo $$?),0)
CFLAGS+=-DDEBUG
endif

ifeq ($(shell test $(DEBUG_LEVEL) -gt 1; echo $$?),0)
CFLAGS+=-DDDEBUG
endif
endif


# LSE support is experimental, please enable the below CFLAGS with caution
ifeq ($(LSE_ENABLE),1)
CFLAGS+=-march=armv8-a+lse -DUSE_LSE
endif


# Use builtin atomics instead of arch specific, if available
ifeq ($(USE_BUILTIN),1)
CFLAGS+=-DUSE_BUILTIN
endif



# cpu_relax variants for lh_osq_lock
# ----------------------------------

# Use YIELD for cpu_relax
ifeq ($(RELAX),YIELD)
CFLAGS+=-DRELAX_IS_YIELD

# Use ISB for cpu_relax
else ifeq ($(RELAX),ISB)
CFLAGS+=-DRELAX_IS_ISB

# Use ISB for cpu_relax
else ifeq ($(RELAX),ISB14)
CFLAGS+=-DRELAX_IS_ISB14

# Use ISB+YIELD for cpu_relax
else ifeq ($(RELAX),ISBYIELD)
CFLAGS+=-DRELAX_IS_ISBYIELD

# Use ndelay for cpu_relax
else ifeq ($(RELAX),NDELAY)
CFLAGS+=-DRELAX_IS_NDELAY

# Use udelay for cpu_relax
else ifeq ($(RELAX),UDELAY)
CFLAGS+=-DRELAX_IS_UDELAY

# Use nop for cpu_relax
else ifeq ($(RELAX),NOP)
CFLAGS+=-DRELAX_IS_NOP

# Use empty for cpu_relax
else ifeq ($(RELAX),EMPTY)
CFLAGS+=-DRELAX_IS_EMPTY


endif


# osq_lock - USE_SMP_COND_LOAD_RELAXED for primary loop
# -----------------------------------------------------

ifeq ($(USE_SMP_COND_LOAD_RELAXED),1)
CFLAGS+=-DUSE_SMP_COND_LOAD_RELAXED
endif


LDFLAGS=-pthread -lm -g -static

TARGET_ARCH:=$(shell ${CC} -dumpmachine | cut -d '-' -f 1)

TEST_TARGETS=lh_swap_mutex \
	lh_event_mutex \
	lh_cas_event_mutex \
	lh_cas_lockref \
	lh_cas_rw_lock \
	lh_incdec_refcount \
	lh_ticket_spinlock \
	lh_queued_spinlock \
	lh_empty \
	lh_jvm_objectmonitor \
	lh_tbb_spin_rw_mutex \
	lh_osq_lock \
	lh_clh_spinlock

ifeq ($(TARGET_ARCH),aarch64)
	TEST_TARGETS+=lh_hybrid_spinlock \
		lh_hybrid_spinlock_fastdequeue
endif

# uncomment to override the list of benchmarks to just one
TEST_TARGETS=lh_osq_lock

BUILD=build

ifeq ($(LSE_ENABLE),1)
BUILD:=$(BUILD).lse
endif

ifeq ($(USE_BUILTIN),1)
BUILD:=$(BUILD).builtin
endif

ifeq ($(USE_SMP_COND_LOAD_RELAXED),1)
BUILD:=$(BUILD).cond_load
endif

ifeq ($(RELAX),YIELD)
BUILD:=$(BUILD).relax_is_yield
else ifeq ($(RELAX),ISB)
BUILD:=$(BUILD).relax_is_isb
else ifeq ($(RELAX),ISB14)
BUILD:=$(BUILD).relax_is_isb14
else ifeq ($(RELAX),ISBYIELD)
BUILD:=$(BUILD).relax_is_isbyield
else ifeq ($(RELAX),NDELAY)
BUILD:=$(BUILD).relax_is_ndelay
else ifeq ($(RELAX),UDELAY)
BUILD:=$(BUILD).relax_is_udelay
else ifeq ($(RELAX),NOP)
BUILD:=$(BUILD).relax_is_nop
else ifeq ($(RELAX),EMPTY)
BUILD:=$(BUILD).relax_is_empty
endif


ALL_VARIANTS = \
builtin_0.lse_0 \
builtin_1.lse_0 \
builtin_0.lse_0.cond_load \
builtin_1.lse_0 \
builtin_1.lse_0.cond_load

ifeq ($(TARGET_ARCH),aarch64)
ALL_VARIANTS += \
builtin_0.lse_1 \
builtin_1.lse_1 \
builtin_1.lse_1.relax_yield \
builtin_1.lse_1.relax_isb \
builtin_1.lse_1.relax_nop \
builtin_1.lse_1.relax_empty \
builtin_1.lse_1.relax_isb14 \
builtin_1.lse_1.relax_isbyield \
builtin_1.lse_1.relax_ndelay \
builtin_1.lse_1.relax_udelay \
builtin_0.lse_1.cond_load \
builtin_1.lse_1.cond_load \
builtin_1.lse_1.cond_load.relax_yield \
builtin_1.lse_1.cond_load.relax_isb \
builtin_1.lse_1.cond_load.relax_nop \
builtin_1.lse_1.cond_load.relax_empty \
builtin_1.lse_1.cond_load.relax_isb14 \
builtin_1.lse_1.cond_load.relax_isbyield \
builtin_1.lse_1.cond_load.relax_ndelay \
builtin_1.lse_1.cond_load.relax_udelay
endif

CLEAN_ALL_VARIANTS = $(addprefix clean.,$(ALL_VARIANTS))


$(info BUILD=$(BUILD))

ALL_BINARIES = $(addprefix $(BUILD)/,$(TEST_TARGETS))

C_SRC = src/lockhammer.c src/cmn-phys-mem.c src/alloc.c

all: $(BUILD) $(ALL_BINARIES)

allvariants: $(ALL_VARIANTS)

cleanallvariants: $(CLEAN_ALL_VARIANTS)


builtin_0.lse_0:
	$(MAKE) USE_BUILTIN=0 LSE_ENABLE=0

builtin_0.lse_0.cond_load:
	$(MAKE) USE_BUILTIN=0 LSE_ENABLE=0 USE_SMP_COND_LOAD_RELAXED=1

builtin_1.lse_0:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=0

builtin_1.lse_0.cond_load:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=0 USE_SMP_COND_LOAD_RELAXED=1

ifeq ($(TARGET_ARCH),aarch64)
builtin_0.lse_1:
	$(MAKE) USE_BUILTIN=0 LSE_ENABLE=1

builtin_1.lse_1:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1

builtin_1.lse_1.relax_yield:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=YIELD

builtin_1.lse_1.relax_isb:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=ISB

builtin_1.lse_1.relax_nop:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=NOP

builtin_1.lse_1.relax_empty:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=EMPTY

builtin_1.lse_1.relax_isb14:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=ISB14

builtin_1.lse_1.relax_isbyield:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=ISBYIELD

builtin_1.lse_1.relax_ndelay:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=NDELAY

builtin_1.lse_1.relax_udelay:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=UDELAY


builtin_0.lse_1.cond_load:
	$(MAKE) USE_BUILTIN=0 LSE_ENABLE=1 USE_SMP_COND_LOAD_RELAXED=1

builtin_1.lse_1.cond_load:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 USE_SMP_COND_LOAD_RELAXED=1

builtin_1.lse_1.cond_load.relax_yield:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=YIELD USE_SMP_COND_LOAD_RELAXED=1

builtin_1.lse_1.cond_load.relax_isb:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=ISB USE_SMP_COND_LOAD_RELAXED=1

builtin_1.lse_1.cond_load.relax_nop:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=NOP USE_SMP_COND_LOAD_RELAXED=1

builtin_1.lse_1.cond_load.relax_empty:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=EMPTY USE_SMP_COND_LOAD_RELAXED=1

builtin_1.lse_1.cond_load.relax_isb14:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=ISB14 USE_SMP_COND_LOAD_RELAXED=1

builtin_1.lse_1.cond_load.relax_isbyield:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=ISBYIELD USE_SMP_COND_LOAD_RELAXED=1

builtin_1.lse_1.cond_load.relax_ndelay:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=NDELAY USE_SMP_COND_LOAD_RELAXED=1

builtin_1.lse_1.cond_load.relax_udelay:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=UDELAY USE_SMP_COND_LOAD_RELAXED=1
endif

$(BUILD):
	mkdir -p $@

$(BUILD)/lh_event_mutex: ../../ext/mysql/event_mutex.h include/atomics.h ../../ext/mysql/include/ut_atomics.h $(C_SRC) | $(BUILD)
	${CC} ${CFLAGS} -DATOMIC_TEST=\"$<\" $(C_SRC) -o $@ ${LDFLAGS}

$(BUILD)/lh_cas_event_mutex: ../../ext/mysql/cas_event_mutex.h include/atomics.h ../../ext/mysql/include/ut_atomics.h $(C_SRC) | $(BUILD)
	${CC} ${CFLAGS} -DATOMIC_TEST=\"$<\" $(C_SRC) -o $@ ${LDFLAGS}

$(BUILD)/lh_ticket_spinlock: ../../ext/linux/ticket_spinlock.h include/atomics.h ../../ext/linux/include/lk_atomics.h $(C_SRC) | $(BUILD)
	${CC} ${CFLAGS} -DATOMIC_TEST=\"$<\" $(C_SRC) -o $@ ${LDFLAGS}

$(BUILD)/lh_hybrid_spinlock: ../../ext/linux/hybrid_spinlock.h include/atomics.h ../../ext/linux/include/lk_atomics.h $(C_SRC) | $(BUILD)
	${CC} ${CFLAGS} -DATOMIC_TEST=\"$<\" $(C_SRC) -o $@ ${LDFLAGS}

$(BUILD)/lh_hybrid_spinlock_fastdequeue: ../../ext/linux/hybrid_spinlock_fastdequeue.h include/atomics.h ../../ext/linux/include/lk_atomics.h $(C_SRC) | $(BUILD)
	${CC} ${CFLAGS} -DATOMIC_TEST=\"$<\" $(C_SRC) -o $@ ${LDFLAGS}

$(BUILD)/lh_osq_lock: ../../ext/linux/osq_lock.h ../../ext/linux/include/lk_atomics.h ../../ext/linux/include/lk_barrier.h ../../ext/linux/include/lk_cmpxchg.h include/atomics.h $(C_SRC) | $(BUILD)
	${CC} ${CFLAGS} -DATOMIC_TEST=\"$<\" $(C_SRC) -o $@ ${LDFLAGS} -DOSQ_LOCK

$(BUILD)/lh_clh_spinlock: ../../ext/sms/clh_spinlock.h ../../ext/sms/base/build_config.h ../../ext/sms/base/cpu.h ../../ext/sms/base/llsc.h $(C_SRC) | $(BUILD)
	${CC} ${CFLAGS} -DATOMIC_TEST=\"$<\" $(C_SRC) -o $@ ${LDFLAGS}

$(BUILD)/lh_queued_spinlock: ../../ext/linux/queued_spinlock.h include/atomics.h ../../ext/linux/include/lk_atomics.h $(C_SRC) | $(BUILD)
	${CC} ${CFLAGS} -DATOMIC_TEST=\"$<\" $(C_SRC) -o $@ ${LDFLAGS}

$(BUILD)/lh_jvm_objectmonitor: ../../ext/jvm/jvm_objectmonitor.h include/atomics.h $(C_SRC) | $(BUILD)
	${CC} ${CFLAGS} -DATOMIC_TEST=\"$<\" $(C_SRC) -o $@ ${LDFLAGS}

$(BUILD)/lh_tbb_spin_rw_mutex: ../../ext/tbb/tbb_spin_rw_mutex.h ../../ext/tbb/include/tbb.h include/atomics.h $(C_SRC) | $(BUILD)
	${CC} ${CFLAGS} -DNDEBUG -DATOMIC_TEST=\"$<\" $(C_SRC) -o $@ ${LDFLAGS}

$(BUILD)/lh_%: tests/%.h include/atomics.h $(C_SRC) | $(BUILD)
	@echo using generic rule for $@
	${CC} ${CFLAGS} -DATOMIC_TEST=\"$<\" $(C_SRC) -o $@ ${LDFLAGS}


clean.builtin_0.lse_0:
	$(MAKE) USE_BUILTIN=0 LSE_ENABLE=0 clean

clean.builtin_0.lse_0.cond_load:
	$(MAKE) USE_BUILTIN=0 LSE_ENABLE=0 USE_SMP_COND_LOAD_RELAXED=1 clean

clean.builtin_1.lse_0:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=0 clean

clean.builtin_1.lse_0.cond_load:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=0 USE_SMP_COND_LOAD_RELAXED=1 clean


ifeq ($(TARGET_ARCH),aarch64)
clean.builtin_0.lse_1:
	$(MAKE) USE_BUILTIN=0 LSE_ENABLE=1 clean

clean.builtin_1.lse_1:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 clean

clean.builtin_1.lse_1.relax_yield:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=YIELD clean

clean.builtin_1.lse_1.relax_isb:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=ISB clean

clean.builtin_1.lse_1.relax_nop:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=NOP clean

clean.builtin_1.lse_1.relax_empty:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=EMPTY clean

clean.builtin_1.lse_1.relax_isb14:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=ISB14 clean

clean.builtin_1.lse_1.relax_isbyield:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=ISBYIELD clean

clean.builtin_1.lse_1.relax_ndelay:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=NDELAY clean

clean.builtin_1.lse_1.relax_udelay:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=UDELAY clean


clean.builtin_0.lse_1.cond_load:
	$(MAKE) USE_BUILTIN=0 LSE_ENABLE=1 USE_SMP_COND_LOAD_RELAXED=1 clean

clean.builtin_1.lse_1.cond_load:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 USE_SMP_COND_LOAD_RELAXED=1 clean

clean.builtin_1.lse_1.cond_load.relax_yield:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=YIELD USE_SMP_COND_LOAD_RELAXED=1 clean

clean.builtin_1.lse_1.cond_load.relax_isb:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=ISB USE_SMP_COND_LOAD_RELAXED=1 clean

clean.builtin_1.lse_1.cond_load.relax_nop:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=NOP USE_SMP_COND_LOAD_RELAXED=1 clean

clean.builtin_1.lse_1.cond_load.relax_empty:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=EMPTY USE_SMP_COND_LOAD_RELAXED=1 clean

clean.builtin_1.lse_1.cond_load.relax_isb14:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=ISB14 USE_SMP_COND_LOAD_RELAXED=1 clean

clean.builtin_1.lse_1.cond_load.relax_isbyield:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=ISBYIELD USE_SMP_COND_LOAD_RELAXED=1 clean

clean.builtin_1.lse_1.cond_load.relax_ndelay:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=NDELAY USE_SMP_COND_LOAD_RELAXED=1 clean

clean.builtin_1.lse_1.cond_load.relax_udelay:
	$(MAKE) USE_BUILTIN=1 LSE_ENABLE=1 RELAX=UDELAY USE_SMP_COND_LOAD_RELAXED=1 clean
endif


clean:
	rm -f $(ALL_BINARIES)
