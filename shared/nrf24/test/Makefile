# Host tests for the nRF24 driver:  make -C shared/nrf24/test
CFLAGS = -std=c11 -Wall -Wextra -Werror -Wconversion -g -fsanitize=address,undefined -I../include

test_nrf24: test_nrf24.c ../src/nrf24.c ../src/nrf24_regs.h ../include/nrf24.h
	$(CC) $(CFLAGS) test_nrf24.c ../src/nrf24.c -o $@
	./$@

clean:
	rm -f test_nrf24

.PHONY: clean
