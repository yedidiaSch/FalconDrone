#ifndef UI_H
#define UI_H

/*
 * Screen task: redraws the OLED at UI_HZ from the latest UiState. The radio
 * task publishes a new state every packet; publishing never blocks on the
 * display.
 */

#include "ui_draw.h"

#define UI_HZ 10

/* Init the display and start the task. Without a display, logs a warning
 * and returns — ui_publish() still works (and does nothing visible). */
void ui_start(void);

void ui_publish(const UiState *s);

#endif /* UI_H */
