# Host tests for portable flight-controller logic:  make -C Core/test
CFLAGS = -std=c11 -Wall -Wextra -Werror -Wconversion -g -fsanitize=address,undefined \
         -I../Inc -I../../shared/rc_protocol/include

all: test_rc_link test_tune_seq

test_rc_link: test_rc_link.c ../Src/rc_link.c ../Inc/rc_link.h ../../shared/rc_protocol/include/rc_protocol.h
	$(CC) $(CFLAGS) test_rc_link.c ../Src/rc_link.c -o $@
	./$@

test_tune_seq: test_tune_seq.c ../Src/tune_seq.c ../Inc/tune_seq.h
	$(CC) $(CFLAGS) test_tune_seq.c ../Src/tune_seq.c -o $@
	./$@

clean:
	rm -f test_rc_link test_tune_seq

.PHONY: all clean
