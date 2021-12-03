
#  run-osq-64.sh [threads] [randseed]

THREADS=$1
if [ -z "$THREADS" ]; then
	THREADS=64
	shift
fi

RANDSEED=$1
if [ -z "$RANDSEED" ]; then
	RANDSEED=11223344
#	RANDSEED=21342349
	shift
fi

ACQUIRES=1000
#./lh_osq_lock -t 64 -a 5000 -c 1000ns -p 0ns -s -i 1 -- -u 500 -s 5 -R 112233
#./lh_osq_lock -t 64 -a 5000 -c 1000ns -p 0ns -s -i 1 -- -u 500 -s 5
#sudo setarch -R ./lh_osq_lock -t $THREADS -a $ACQUIRES -c 1000ns -p 0ns -i 1 -- -u 500 -s 5 -R $RANDSEED

# without -i1 to do the thread per-cpu affinity; how is it that it does it automatically without -i1 ???
#sudo setarch -R ./lh_osq_lock -t $THREADS -a $ACQUIRES -c 1000ns -p 0ns -i 1 -- -u 500 -s 5 -R $RANDSEED
#sudo setarch -R ./lh_osq_lock -t $THREADS -a $ACQUIRES -c 1000ns -p 0ns -s $* -- -u 500 -s 5 -R $RANDSEED
#sudo setarch -R ./lh_osq_lock -t $THREADS -O 100000000 -c 1000ns -p 0ns $* -- -u 500 -s 5 -R $RANDSEED
sudo setarch -R ./lh_osq_lock -t $THREADS -O 1000000000 -c 1000ns -p 0ns $* -s -- -u 500 -s 5 -R $RANDSEED
#sudo ./lh_osq_lock -t $THREADS -a $ACQUIRES -c 1000ns -p 0ns -- -u 20000 -s 5 -R $RANDSEED
