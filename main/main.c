/* LVGL Example project
 *
 * Basic project to test LVGL on ESP32 based projects.
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_timer.h"

/* Littlevgl specific */
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#include "lvgl_helpers.h"

#define TAG "demo"
#define LV_TICK_PERIOD_MS 1

static void lv_tick_task(void *arg);
static void guiTask(void *pvParameter);
static void create_demo_application(void);

static lv_obj_t *btn1;
static lv_obj_t *btn2;

void app_main() {
    xTaskCreatePinnedToCore(guiTask, "gui", 4096*2, NULL, 0, NULL, 1);
}

SemaphoreHandle_t xGuiSemaphore;

static void guiTask(void *pvParameter) {
    (void) pvParameter;
    xGuiSemaphore = xSemaphoreCreateMutex();

    lv_init();
    lvgl_driver_init();

    lv_color_t* buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);

    #ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    lv_color_t* buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2 != NULL);
    #else
    static lv_color_t *buf2 = NULL;
    #endif

    static lv_disp_buf_t disp_buf;
    uint32_t size_in_px = DISP_BUF_SIZE;

    #if defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_IL3820 || \
        defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_JD79653A || \
        defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_UC8151D || \
        defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_SSD1306
    size_in_px *= 8;
    #endif

    lv_disp_buf_init(&disp_buf, buf1, buf2, size_in_px);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;

    #if defined CONFIG_DISPLAY_ORIENTATION_PORTRAIT || defined CONFIG_DISPLAY_ORIENTATION_PORTRAIT_INVERTED
    disp_drv.rotated = 1;
    #endif

    #ifdef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    disp_drv.rounder_cb = disp_driver_rounder;
    disp_drv.set_px_cb = disp_driver_set_px;
    #endif

    disp_drv.buffer = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    #if CONFIG_LV_TOUCH_CONTROLLER != TOUCH_CONTROLLER_NONE
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.read_cb = touch_driver_read;
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    lv_indev_drv_register(&indev_drv);
    #endif

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    create_demo_application();

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << 26) | (1ULL << 27);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            lv_task_handler();

            if (gpio_get_level(GPIO_NUM_26) == 1) {
                lv_obj_set_style_local_bg_color(btn1, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_GREEN);
            } else {
                lv_obj_set_style_local_bg_color(btn1, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_RED);
            }

            if (gpio_get_level(GPIO_NUM_27) == 1) {
                lv_obj_set_style_local_bg_color(btn2, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_GREEN);
            } else {
                lv_obj_set_style_local_bg_color(btn2, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_RED);
            }

            xSemaphoreGive(xGuiSemaphore);
        }
    }

    free(buf1);
    #ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    free(buf2);
    #endif
    vTaskDelete(NULL);
}

static void create_demo_application(void) {
    lv_obj_t *scr = lv_disp_get_scr_act(NULL);

    btn1 = lv_btn_create(scr, NULL);
    lv_obj_set_size(btn1, 120, 50);
    lv_obj_align(btn1, NULL, LV_ALIGN_CENTER, 0, -60);
    lv_obj_t *label1 = lv_label_create(btn1, NULL);
    lv_label_set_text(label1, "Bomba 1");

    btn2 = lv_btn_create(scr, NULL);
    lv_obj_set_size(btn2, 120, 50);
    lv_obj_align(btn2, NULL, LV_ALIGN_CENTER, 0, 60);
    lv_obj_t *label2 = lv_label_create(btn2, NULL);
    lv_label_set_text(label2, "Bomba 2");
}

static void lv_tick_task(void *arg) {
    (void) arg;
    lv_tick_inc(LV_TICK_PERIOD_MS);
}
