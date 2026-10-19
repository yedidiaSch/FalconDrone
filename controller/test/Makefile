# Host tests for the controller screen:  make -C controller/test
#   make -C controller/test screens   → screens/*.pgm previews
CFLAGS = -std=c11 -Wall -Wextra -Werror -Wconversion -g -fsanitize=address,undefined \
         -I../main -I../../shared/rc_protocol/include
SRCS = test_ui.c ../main/gfx.c ../main/ui_draw.c

test_ui: $(SRCS) ../main/gfx.h ../main/ui_draw.h ../main/font5x7.h
	$(CC) $(CFLAGS) $(SRCS) -o $@
	./$@

screens: test_ui
	mkdir -p screens
	./test_ui screens

clean:
	rm -rf test_ui screens

.PHONY: screens clean
