#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <dk_buttons_and_leds.h>

#include "main.h"
#include "uart.h"
#include "ble.h"
#include "ui.h"

LOG_MODULE_REGISTER(main);

static void configure_gpio(void)
{
	int err;

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}
}

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true) {
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

int main(void)
{
	int blink_status = 0;
    int err = 0;

    configure_gpio();

	err = uart_init();
	if (err) {
		error();
	}
    
	err = ble_init();

	if (err) {
		error();
	}    

	ui_init();	

	while (1) {		
		lv_task_handler();
		k_sleep(K_MSEC(100));
        dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
	}
}