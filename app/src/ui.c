#include <zephyr/kernel.h>
#include <zephyr/drivers/display.h>
#include <zephyr/logging/log.h>

#include "ui.h"

const struct device *display_dev;
lv_obj_t *hello_world_label;
lv_obj_t *text_label;

LOG_MODULE_REGISTER(ui);

void display_init(void) {

    display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

	if (display_dev == NULL) {
        LOG_ERR("Display device not found. Aborting application.");
	}
}

void ui_init(void) {
    display_init();

    hello_world_label = lv_label_create(lv_scr_act());
	
	lv_label_set_text(hello_world_label, "Hello world!");
	lv_obj_align(hello_world_label, LV_ALIGN_CENTER, 0, 0);

	text_label = lv_label_create(lv_scr_act());
	lv_obj_align(text_label, LV_ALIGN_BOTTOM_MID, 0, 0);

	lv_task_handler();
	display_blanking_off(display_dev);
}

void update_text(char *text) {
    lv_label_set_text(text_label, text);    
}