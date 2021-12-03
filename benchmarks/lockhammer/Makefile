# override keyword overwrites make command-line option LSE_ENABLE=y, therefore it has been removed
CFLAGS += -g -O3 -Wall -I. -I./include -I../../ext/mysql/include -I../../ext/linux/include -I../../ext/tbb/include -I../../ext/sms/base
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


LDFLAGS=-lpthread -lm

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

ifeq ($(USE_BUILTIN),1)

ifeq ($(LSE_ENABLE),1)
BUILD=build.lse.builtin
else
BUILD=build.builtin
endif

else

ifeq ($(LSE_ENABLE),1)
BUILD=build.lse
else
BUILD=build
endif

endif

ALL_BINARIES = $(addprefix $(BUILD)/,$(TEST_TARGETS))

C_SRC = src/lockhammer.c

all: $(BUILD) $(ALL_BINARIES)

allvariants:
	make USE_BUILTIN=0 LSE_ENABLE=0
	make USE_BUILTIN=1 LSE_ENABLE=0
ifeq ($(TARGET_ARCH),aarch64)
	make USE_BUILTIN=0 LSE_ENABLE=1
	make USE_BUILTIN=1 LSE_ENABLE=1
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
	${CC} ${CFLAGS} -DATOMIC_TEST=\"$<\" $(C_SRC) -o $@ ${LDFLAGS}

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


clean:
	rm -f $(ALL_BINARIES)
