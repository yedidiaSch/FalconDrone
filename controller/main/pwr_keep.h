#ifndef PWR_KEEP_H
#define PWR_KEEP_H

/*
 * Keeps the RC power board (hardware/rc_power) switched on.
 *
 * Its IP5306 turns the 5 V boost off after 32 s below 45 mA. PIN_PWR_KEEP
 * drives a 2N7002 that pulls the IP5306's KEY pin low — a button press — so
 * a short press every PWR_KEEP_PERIOD_MS restarts that timer.
 *
 * Does nothing unless CONFIG_FALCON_PWR_KEEP is set (menuconfig).
 */

/* Start the keep-alive task. */
void pwr_keep_start(void);

/* Hold the button for 2.5 s: the IP5306 switches the RC off. Blocks; does
 * not return if the power board is attached. */
void pwr_keep_power_off(void);

#endif /* PWR_KEEP_H */
