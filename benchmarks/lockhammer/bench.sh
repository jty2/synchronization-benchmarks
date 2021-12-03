#!/bin/bash


#BULIDS="build \
#build.builtin \
#build.lse \
#build.lse.builtin \
#build.lse.builtin.relax_is_isb \
#build.lse.builtin.relax_is_isbyield \
#build.lse.builtin.relax_is_ndelay \
#build.lse.builtin.relax_is_udelay \
#build.lse.builtin.relax_is_yield


set -xe

BUILDS=" \
build.lse.builtin.relax_is_isbyield \
build.lse.builtin.relax_is_isb \
build.lse.builtin.relax_is_yield \
build.lse.builtin.relax_is_ndelay \
build.lse.builtin.relax_is_udelay"


BUILDS=" \
build.lse.builtin.relax_is_isbyield \
build.lse.builtin.relax_is_isb \
build.lse.builtin.relax_is_yield"

# the *delay routines seem to hang/livelock too often.

#BUILDS=" build.lse.builtin.relax_is_ndelay"
#BUILDS=" build.lse.builtin.relax_is_udelay"

# as does isbyield???


#BUILDS=" \
#build.lse.builtin.relax_is_isb \
#build.lse.builtin.relax_is_yield"


#BUILDS=build.lse.builtin.relax_is_ndelay
#BUILDS=build.lse.builtin.relax_is_udelay


BUILDS=build.lse.builtin.relax_is_isb14

for build in $BUILDS
do

# missing from example:
# -f select6 -H-F 1000 -E 100


#for ndelay in {0..200..20}; do
#for ndelay in {40..200..20}; do		# hangs often, why?
#for udelay in {0..10}; do

for tries in {1..5}; do
#sudo setarch -R $build/lh_osq_lock -t 64 -a 500 -c 1000ns -p 0ns -D 0 -- -u 500 -s 5 | tee $build.$tries.log

# for ndelay
#sudo setarch -R $build/lh_osq_lock -t 64 -a 500 -c 1000ns -p 0ns -D 60 -- -u 500 -s 5 | tee $build.$tries.log

# for udelay
sudo setarch -R $build/lh_osq_lock -t 64 -a 500 -c 1000ns -p 0ns -D 1 -- -u 500 -s 5 | tee $build.$tries.log

#script -c "sudo setarch -R $build/lh_osq_lock -t 64 -a 500 -c 1000ns -p 0ns -D 0 -- -u 500 -s 5"  $build.$tries.log
#sudo setarch -R $build/lh_osq_lock -t 64 -a 500 -c 1000ns -p 0ns -D $ndelay -- -u 500 -s 5
#sudo setarch -R $build/lh_osq_lock -t 64 -a 500 -c 1000ns -p 0ns -D $udelay -- -u 500 -s 5
#done
done

done
