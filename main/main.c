/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/touch_pad.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "mqtt_client.h"

#include "protocol_examples_common.h"
#include "esp_wifi.h"


#include "esp_lcd_gc9a01.h"
static const char *TAG = "MQTT WALL SWITCH";
static const char *TOUCH_TAG = "Touch";
#pragma region DisplayDefines


// Using SPI2 in the example
#define LCD_HOST  SPI2_HOST

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (20 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_SCLK           18
#define EXAMPLE_PIN_NUM_MOSI           23
#define EXAMPLE_PIN_NUM_MISO           21
#define EXAMPLE_PIN_NUM_LCD_DC         26
#define EXAMPLE_PIN_NUM_LCD_RST        25
#define EXAMPLE_PIN_NUM_LCD_CS         5
#define EXAMPLE_PIN_NUM_BK_LIGHT       22
#define EXAMPLE_PIN_NUM_TOUCH_CS       5


#define EXAMPLE_LCD_H_RES              240
#define EXAMPLE_LCD_V_RES              240

// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

#define EXAMPLE_LVGL_DRAW_BUF_LINES    20 // number of display lines in each draw buffer
#define EXAMPLE_LVGL_TICK_PERIOD_MS    2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2

// LVGL library is not thread-safe, this example will call LVGL APIs from different tasks, so use a mutex to protect it
static _lock_t lvgl_api_lock;

extern void main_menu_lvgl_ui(lv_disp_t *disp);
extern void list_sub_menu(lv_display_t *disp);
extern void ledWall_sub_menu(lv_display_t *disp);
extern void change_slider_value_CW(int32_t new_value);
extern void change_slider_value_WW(int32_t new_value);

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

/* Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. */
static void example_lvgl_port_update_callback(lv_display_t *disp)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    lv_display_rotation_t rotation = lv_display_get_rotation(disp);

    switch (rotation) {
    case LV_DISPLAY_ROTATION_0:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    case LV_DISPLAY_ROTATION_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISPLAY_ROTATION_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISPLAY_ROTATION_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    }
}

static void example_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    example_lvgl_port_update_callback(disp);
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // because SPI LCD is big-endian, we need to swap the RGB bytes order
    lv_draw_sw_rgb565_swap(px_map, (offsetx2 + 1 - offsetx1) * (offsety2 + 1 - offsety1));
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
}
static void mqtt_event_handler(esp_mqtt_event_handle_t event);
#pragma endregion

#pragma region TouchDefines

#define TOUCH_THRESH_NO_USE   (0)
#define TOUCH_THRESH_PERCENT  (80)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (10)
#define T1_CHANNEL TOUCH_PAD_NUM0
#define T2_CHANNEL TOUCH_PAD_NUM1
#define T3_CHANNEL TOUCH_PAD_NUM2
#define T4_CHANNEL TOUCH_PAD_NUM3
#define T5_CHANNEL TOUCH_PAD_NUM4
#define T6_CHANNEL TOUCH_PAD_NUM5
#define T7_CHANNEL TOUCH_PAD_NUM6
#define T8_CHANNEL TOUCH_PAD_NUM7
#define T9_CHANNEL TOUCH_PAD_NUM8
#define T10_CHANNEL TOUCH_PAD_NUM9

#define TOUCH_CHANNELS { T1_CHANNEL, T2_CHANNEL, T3_CHANNEL, T4_CHANNEL, \
                         T5_CHANNEL, T6_CHANNEL, T7_CHANNEL, T8_CHANNEL, \
                         T9_CHANNEL, T10_CHANNEL }

static int touch_channels[] = TOUCH_CHANNELS;
static bool touchPressed[10];
static void tp_init_set_thresholds(void);
static void tp_rtc_intr(void *arg);

#pragma endregion

#pragma region Gui
uint8_t currentScreen = 0;
enum Screen{
    MAIN_SCREEN = 1,
    LED_WALL_SCREEN,
    FCN_LIST_SCREEN
};

int32_t cw_val_gui = 100;
int32_t ww_val_gui = 100;

#pragma endregion

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t time_till_next_ms = 0;
    uint32_t time_threshold_ms = 1000 / CONFIG_FREERTOS_HZ;
    while (1) {
        _lock_acquire(&lvgl_api_lock);
        time_till_next_ms = lv_timer_handler();
        _lock_release(&lvgl_api_lock);
        // in case of triggering a task watch dog time out
        time_till_next_ms = MAX(time_till_next_ms, time_threshold_ms);
        usleep(1000 * time_till_next_ms);
    }
}

void app_main(void)
{
    #pragma region setupWIFI
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    
    ESP_ERROR_CHECK(example_connect());
    #pragma endregion
    #pragma region setupDisplay
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_PIN_NUM_SCLK,
        .mosi_io_num = EXAMPLE_PIN_NUM_MOSI,
        .miso_io_num = EXAMPLE_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_PIN_NUM_LCD_DC,
        .cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };

    ESP_LOGI(TAG, "Install GC9A01 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));

    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    // create a lvgl display
    lv_display_t *display = lv_display_create(EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);

    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    size_t draw_buffer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_DRAW_BUF_LINES * sizeof(lv_color16_t);

    void *buf1 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    assert(buf1);
    void *buf2 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_display_set_buffers(display, buf1, buf2, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);
    // associate the mipi panel handle to the display
    lv_display_set_user_data(display, panel_handle);
    // set color depth
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    // set the callback which can copy the rendered image to an area of the display
    lv_display_set_flush_cb(display, example_lvgl_flush_cb);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    ESP_LOGI(TAG, "Register io panel event callback for LVGL flush ready notification");
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = example_notify_lvgl_flush_ready,
    };
    /* Register done callback */
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, display));



    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE*2, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "Display LVGL Meter Widget");
    lv_obj_t *scr = lv_display_get_screen_active(display);
    #pragma endregion
    // Lock the mutex due to the LVGL APIs are not thread-safe
    #pragma region setupTouch
    tp_init_set_thresholds();
    

    #pragma endregion
    _lock_acquire(&lvgl_api_lock);
    main_menu_lvgl_ui(display);
    currentScreen = MAIN_SCREEN;
    _lock_release(&lvgl_api_lock);

    
    while(1){
        switch(currentScreen){
            case MAIN_SCREEN:
                if(touchPressed[0]) {
                    touchPressed[0] = false;
                    lv_obj_clean(scr);
                    _lock_acquire(&lvgl_api_lock);
                    ledWall_sub_menu(display);
                    currentScreen = LED_WALL_SCREEN;
                    _lock_release(&lvgl_api_lock);
                }
                if(touchPressed[2]){
                    touchPressed[2] = false;
                    lv_obj_clean(scr);
                    _lock_acquire(&lvgl_api_lock);
                    list_sub_menu(display);
                    currentScreen = FCN_LIST_SCREEN;
                    _lock_release(&lvgl_api_lock);
                }
            break;
            case LED_WALL_SCREEN:
                if(touchPressed[0]) {//CW up
                    touchPressed[0] = false;
                    if(ww_val_gui<100)ww_val_gui+=5;
                    change_slider_value_WW(ww_val_gui);
                }
                if(touchPressed[3]) {//CW Down
                    touchPressed[3] = false;
                    if(ww_val_gui>0)ww_val_gui-=5;
                    change_slider_value_WW(ww_val_gui);
                }
                if(touchPressed[7]) {//WW up
                    touchPressed[7] = false;
                    if(cw_val_gui<100)cw_val_gui+=5;
                    change_slider_value_CW(cw_val_gui);
                }
                if(touchPressed[5]) {//WW Down
                    touchPressed[5] = false;
                    if(cw_val_gui>0)cw_val_gui-=5;
                    change_slider_value_CW(cw_val_gui);
                }
                
                if(touchPressed[4]) {
                    touchPressed[4] = false;
                    lv_obj_clean(scr);
                    _lock_acquire(&lvgl_api_lock);
                    main_menu_lvgl_ui(display);
                    currentScreen = MAIN_SCREEN;
                    _lock_release(&lvgl_api_lock);
                }
            break;
            default:
                if(touchPressed[4]) {
                    touchPressed[4] = false;
                    lv_obj_clean(scr);
                    _lock_acquire(&lvgl_api_lock);
                    main_menu_lvgl_ui(display);
                    currentScreen = MAIN_SCREEN;
                    _lock_release(&lvgl_api_lock);
                }
            break;
        }



        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}


static void tp_rtc_intr(void *arg)
{
    uint32_t pad_intr = touch_pad_get_status();

    for (int i = 0; i < sizeof(touch_channels)/sizeof(touch_channels[0]); i++) {
        if ((pad_intr >> touch_channels[i]) & 0x01) {
            touchPressed[i] = true;
        }
    }

    touch_pad_clear_status();
}
static void tp_init_set_thresholds(void)
{

    
    touch_pad_init();
    
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    // Set reference voltage for charging/discharging
    // For most usage scenarios, we recommend using the following combination:
    // the high reference valtage will be 2.7V - 1V = 1.7V, The low reference voltage will be 0.5V.
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    // Init touch pad IO
     for (int i = 0; i < sizeof(touch_channels)/sizeof(touch_channels[0]); i++) {
        touch_pad_config(touch_channels[i], TOUCH_THRESH_NO_USE);
     }
    
    
    // Initialize and start a software filter to detect slight change of capacitance.
    touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);
    // Set thresh hold
   // tp_set_thresholds();
    // Register touch interrupt ISR
    
    touch_pad_isr_register(tp_rtc_intr, NULL);
    touch_pad_intr_enable();
    uint16_t touch_value;
    //Set threshold for T1
    for (int i = 0; i < sizeof(touch_channels)/sizeof(touch_channels[0]); i++) {
        touch_pad_read_filtered(touch_channels[i], &touch_value);
        ESP_LOGI(TOUCH_TAG, "test init: touch pad T%d val is %d",i, touch_value);
        ESP_ERROR_CHECK(touch_pad_set_thresh(touch_channels[i], touch_value * 2 / 3));
     }

    
}

// static void mqtt_event_handler(esp_mqtt_event_handle_t event){ //here esp_mqtt_event_handle_t is a struct which receieves struct event from mqtt app start funtion
// esp_mqtt_client_handle_t client = event->client //making obj client of struct esp_mqtt_client_handle_t and assigning it the receieved event client
//   if(event->event_id == MQTT_EVENT_CONNECTED){
//   ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
//   esp_mqtt_client_subscribe(client,"your topic",0); //in mqtt we require a topic to subscribe and client is from event client and 0 is quality of service it can be 1 or 2
//   ESP_LOGI(TAG3, "sent subscribe successful" );
//   }
//   else if(event->event_id == MQTT_EVENT_DISCONNECTED)
//   {
//     ESP_LOGI(TAG3, "MQTT_EVENT_DISCONNECTED"); //if disconnected
//   }
//   else if(event->event_id == MQTT_EVENT_SUBSCRIBED)
//   {
//      ESP_LOGI(TAG3, "MQTT_EVENT_SUBSCRIBED");
//   }
//   else if(event->event_id == MQTT_EVENT_UNSUBSCRIBED) //when subscribed
//   {
//      ESP_LOGI(TAG3, "MQTT_EVENT_UNSUBSCRIBED");
//   }
//   else if(event->event_id == MQTT_EVENT_DATA)//when unsubscribed
//   {
//      ESP_LOGI(TAG3, "MQTT_EVENT_DATA");
//   }
//   else if(event->event_id == MQTT_EVENT_ERROR)//when any error
//   {
//      ESP_LOGI(TAG3, "MQTT_EVENT_ERROR");
//   }
// }

// static void mqtt_initialize(void)
// {
//   const esp_mqtt_client_config_t mqtt_cfg={
//     .uri="mqtt://io.adafruit.com", 
//     .event_handle=mqtt_event_handler, 
//   };
//   esp_mqtt_client_handle_t client=esp_mqtt_client_init(&mqtt_cfg); //sending struct as a parameter in init client function
//   esp_mqtt_client_start(client); //starting the process
// }