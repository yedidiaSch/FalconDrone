# Host tests for portable flight-controller logic:  make -C Core/test
CFLAGS = -std=c11 -Wall -Wextra -Werror -Wconversion -g -fsanitize=address,undefined \
         -I../Inc -I../../shared/rc_protocol/include

CXXFLAGS = -std=c++17 -Wall -Wextra -Werror -Wconversion -g -fsanitize=address,undefined -I../Inc

all: test_rc_link test_tune_seq test_arming test_pid test_attitude_filter

test_rc_link: test_rc_link.c ../Src/rc_link.c ../Inc/rc_link.h ../../shared/rc_protocol/include/rc_protocol.h
	$(CC) $(CFLAGS) test_rc_link.c ../Src/rc_link.c -o $@
	./$@

test_tune_seq: test_tune_seq.c ../Src/tune_seq.c ../Inc/tune_seq.h
	$(CC) $(CFLAGS) test_tune_seq.c ../Src/tune_seq.c -o $@
	./$@

# arming.c includes imu_service.h → the HAL; stubs/ stands in for it.
test_arming: test_arming.c ../Src/arming.c ../Inc/arming.h
	$(CC) $(CFLAGS) -Istubs test_arming.c ../Src/arming.c -lm -o $@
	./$@

test_pid: test_pid.c ../Src/pid.c ../Inc/pid.h
	$(CC) $(CFLAGS) test_pid.c ../Src/pid.c -lm -o $@
	./$@

test_attitude_filter: test_attitude_filter.cpp ../Src/attitude_filter.cpp ../Inc/attitude_filter.h ../Inc/imu_types.h
	$(CXX) $(CXXFLAGS) test_attitude_filter.cpp ../Src/attitude_filter.cpp -o $@
	./$@

clean:
	rm -f test_rc_link test_tune_seq test_arming test_pid test_attitude_filter

.PHONY: all clean
