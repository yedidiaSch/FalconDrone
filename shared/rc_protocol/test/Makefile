# Host tests for the frame codec:  make -C shared/rc_protocol/test
CFLAGS = -std=c11 -Wall -Wextra -Werror -Wconversion -g -fsanitize=address,undefined -I../include

test_rc_frame: test_rc_frame.c ../src/rc_frame.c ../include/rc_frame.h ../include/rc_protocol.h
	$(CC) $(CFLAGS) test_rc_frame.c ../src/rc_frame.c -o $@
	./$@

clean:
	rm -f test_rc_frame

.PHONY: clean
