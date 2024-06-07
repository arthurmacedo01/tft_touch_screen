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
#include "esp_log.h"

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#include "lvgl_helpers.h"

#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
#if defined CONFIG_LV_USE_DEMO_WIDGETS
#include "lv_examples/src/lv_demo_widgets/lv_demo_widgets.h"
#elif defined CONFIG_LV_USE_DEMO_KEYPAD_AND_ENCODER
#include "lv_examples/src/lv_demo_keypad_encoder/lv_demo_keypad_encoder.h"
#elif defined CONFIG_LV_USE_DEMO_BENCHMARK
#include "lv_examples/src/lv_demo_benchmark/lv_demo_benchmark.h"
#elif defined CONFIG_LV_USE_DEMO_STRESS
#include "lv_examples/src/lv_demo_stress/lv_demo_stress.h"
#else
#error "No demo application selected."
#endif
#endif

#define TAG "demo"
#define LV_TICK_PERIOD_MS 1

static void lv_tick_task(void *arg);
static void guiTask(void *pvParameter);
static void create_demo_application(void);
static void btn1_event_cb(lv_obj_t *btn, lv_event_t event);
static void btn2_event_cb(lv_obj_t *btn, lv_event_t event);
static void update_button_state(void);

SemaphoreHandle_t xGuiSemaphore;
static int gpio2_state = 0;  // Variable to track GPIO2 state
static int gpio26_state = 0; // Variable to track GPIO26 state

void app_main()
{
    xTaskCreatePinnedToCore(guiTask, "gui", 4096 * 2, NULL, 0, NULL, 1);
}

static void guiTask(void *pvParameter)
{
    (void)pvParameter;
    xGuiSemaphore = xSemaphoreCreateMutex();

    lv_init();
    lvgl_driver_init();

    lv_color_t *buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);

#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    lv_color_t *buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2 != NULL);
#else
    static lv_color_t *buf2 = NULL;
#endif

    static lv_disp_buf_t disp_buf;
    uint32_t size_in_px = DISP_BUF_SIZE;

#if defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_IL3820 || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_JD79653A || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_UC8151D || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_SSD1306
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
        .name = "periodic_gui"};
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_NUM_2) | (1ULL << GPIO_NUM_26);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_NUM_13) | (1ULL << GPIO_NUM_12);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    create_demo_application();

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10));

        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
        {
            lv_task_handler();
            update_button_state();
            xSemaphoreGive(xGuiSemaphore);
        }
    }

    free(buf1);
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    free(buf2);
#endif
    vTaskDelete(NULL);
}

static void btn1_event_cb(lv_obj_t *btn, lv_event_t event)
{
    if (event == LV_EVENT_CLICKED)
    {
        lv_obj_t *label = lv_obj_get_child(btn, NULL);
        lv_label_set_text(label, "Acionando");

        gpio2_state = !gpio2_state;
        gpio_set_level(GPIO_NUM_2, gpio2_state);
        gpio26_state = 0;
        gpio_set_level(GPIO_NUM_26, 0);
    }
}

static void btn2_event_cb(lv_obj_t *btn, lv_event_t event)
{
    if (event == LV_EVENT_CLICKED)
    {
        lv_obj_t *label = lv_obj_get_child(btn, NULL);
        lv_label_set_text(label, "Acionando");

        gpio26_state = !gpio26_state;
        gpio_set_level(GPIO_NUM_26, gpio26_state);
        gpio2_state = 0;
        gpio_set_level(GPIO_NUM_2, 0);
    }
}

static void update_button_state(void)
{
    lv_obj_t *scr = lv_disp_get_scr_act(NULL);
    lv_obj_t *btn2 = lv_obj_get_child(scr, NULL);
    lv_obj_t *label2 = lv_obj_get_child(btn2, NULL);
    lv_obj_t *btn1 = lv_obj_get_child(scr, btn2);
    lv_obj_t *label1 = lv_obj_get_child(btn1, NULL);

    int gpio13_level = gpio_get_level(GPIO_NUM_13);
    int gpio12_level = gpio_get_level(GPIO_NUM_12);

    ESP_LOGI(TAG, "GPIO2 state: %d, GPIO13 level: %d", gpio2_state, gpio13_level);
    ESP_LOGI(TAG, "GPIO26 state: %d, GPIO12 level: %d", gpio26_state, gpio12_level);

    if (gpio2_state == 0 && gpio13_level == 0)
    {
        lv_label_set_text(label1, "Comp 1 Desligado");
        lv_obj_set_style_local_bg_color(btn1, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_RED);
    }
    else if (gpio2_state == 0 && gpio13_level == 1)
    {
        lv_label_set_text(label1, "Comp 1 Desligando");
        lv_obj_set_style_local_bg_color(btn1, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_YELLOW);
    }
    else if (gpio2_state == 1 && gpio13_level == 0)
    {
        lv_label_set_text(label1, "Comp 1 Ligando");
        lv_obj_set_style_local_bg_color(btn1, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_YELLOW);
    }
    else if (gpio2_state == 1 && gpio13_level == 1)
    {
        lv_label_set_text(label1, "Comp 1 Ligado");
        lv_obj_set_style_local_bg_color(btn1, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_GREEN);
    }

    if (gpio26_state == 0 && gpio12_level == 0)
    {
        lv_label_set_text(label2, "Comp 2 Desligado");
        lv_obj_set_style_local_bg_color(btn2, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_RED);
    }
    else if (gpio26_state == 0 && gpio12_level == 1)
    {
        lv_label_set_text(label2, "Comp 2 Desligando");
        lv_obj_set_style_local_bg_color(btn2, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_YELLOW);
    }
    else if (gpio26_state == 1 && gpio12_level == 0)
    {
        lv_label_set_text(label2, "Comp 2 Ligando");
        lv_obj_set_style_local_bg_color(btn2, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_YELLOW);
    }
    else if (gpio26_state == 1 && gpio12_level == 1)
    {
        lv_label_set_text(label2, "Comp 2 Ligado");
        lv_obj_set_style_local_bg_color(btn2, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_GREEN);
    }
}

static void create_demo_application(void)
{
    lv_obj_t *scr = lv_disp_get_scr_act(NULL);
    lv_coord_t screen_width = lv_disp_get_hor_res(NULL);
    lv_coord_t screen_height = lv_disp_get_ver_res(NULL);
    lv_coord_t btn_width = screen_width * 0.4;
    lv_coord_t btn_height = screen_height * 0.4;

    lv_obj_t *btn1 = lv_btn_create(scr, NULL);
    lv_obj_set_pos(btn1, (screen_width - btn_width) / 2 - btn_width / 2 - 10, (screen_height - btn_height) / 2);
    lv_obj_set_size(btn1, btn_width, btn_height);
    lv_obj_set_event_cb(btn1, btn1_event_cb);

    lv_obj_t *label1 = lv_label_create(btn1, NULL);
    lv_label_set_text(label1, "Desligado");
    lv_obj_align(label1, btn1, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *btn2 = lv_btn_create(scr, NULL);                                                               // Create btn2 independently
    lv_obj_set_pos(btn2, (screen_width - btn_width) / 2 + btn_width / 2 + 10, (screen_height - btn_height) / 2); // Position btn2 on the opposite side of the screen
    lv_obj_set_size(btn2, btn_width, btn_height);
    lv_obj_set_event_cb(btn2, btn2_event_cb);

    lv_obj_t *label2 = lv_label_create(btn2, NULL);
    lv_label_set_text(label2, "Desligado");
    lv_obj_align(label2, btn2, LV_ALIGN_CENTER, 0, 0);

    update_button_state();
}

static void lv_tick_task(void *arg)
{
    (void)arg;
    lv_tick_inc(LV_TICK_PERIOD_MS);
}
