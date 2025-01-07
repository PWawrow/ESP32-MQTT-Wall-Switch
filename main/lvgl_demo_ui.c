/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// This demo UI is adapted from LVGL official example: https://docs.lvgl.io/master/examples.html#loader-with-arc

#include "lvgl.h"
#include <math.h>
#include <stdio.h>

#define LV_DEG_TO_RAD(deg) ((deg) * 3.14159f / 180.0f)


static lv_obj_t * btn;
static lv_display_rotation_t rotation = LV_DISP_ROTATION_0;
static lv_obj_t *sliderCW;
static lv_obj_t *sliderWW;

static void btn_cb(lv_event_t * e)
{
    lv_display_t *disp = lv_event_get_user_data(e);
    rotation++;
    if (rotation > LV_DISP_ROTATION_270) {
        rotation = LV_DISP_ROTATION_0;
    }
    lv_disp_set_rotation(disp, rotation);
}
static void set_angle(void * obj, int32_t v)
{
    lv_arc_set_value(obj, v);
}

#include "lvgl.h"
#include <math.h> // For sinf() and cosf()

#define LV_DEG_TO_RAD(deg) ((deg) * 3.14159f / 180.0f) // Convert degrees to radians

void main_menu_lvgl_ui(lv_display_t *disp)
{
    lv_obj_t *scr = lv_display_get_screen_active(disp);

    /* Apply dark theme to the screen */
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

    /* Central MQTT Data Label */
     lv_obj_t *circle_main = lv_obj_create(scr);
    lv_obj_align(circle_main, LV_ALIGN_CENTER, 0, 0); 
    lv_obj_set_size(circle_main, 235,235);
    lv_obj_set_style_radius(circle_main, LV_RADIUS_CIRCLE, 0);          // Make it a perfect circle
    lv_obj_set_style_bg_opa(circle_main, LV_OPA_TRANSP, 0);             // Make the background transparent
    lv_obj_set_style_border_width(circle_main, 5, 0);                  // Set the border thickness
    lv_obj_set_style_border_color(circle_main, lv_color_white(), 0); // Set the border color
    lv_obj_set_style_border_opa(circle_main, LV_OPA_COVER, 0);  


    //create individal menu items;
    const char *menuItems[8];
    menuItems[0] = LV_SYMBOL_POWER;
    menuItems[1] = LV_SYMBOL_SETTINGS;
    menuItems[2] = LV_SYMBOL_WIFI;
    menuItems[3] = LV_SYMBOL_LIST;
    menuItems[4] = "1";
    menuItems[5] = "2";
    menuItems[6] = "3";
    menuItems[7] = "4";
    /* Touchpad Indicators Around Circle */
    for (int i = 0; i < 8; i++) {
        int angle = (i * 45)+22; // Evenly spaced around the circle
        int radius = 70;    // Distance from the center
        int x = radius * cosf(LV_DEG_TO_RAD(angle+22));
        int y = radius * sinf(LV_DEG_TO_RAD(angle+22));

        lv_obj_t *indicator = lv_btn_create(scr);
        lv_obj_set_size(indicator, 1, 240);
        lv_obj_set_style_bg_color(indicator, lv_color_hex(0x444444), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(indicator, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_align(indicator, LV_ALIGN_CENTER, 0, 121);
        lv_obj_set_style_transform_angle(indicator, angle*10, 0);

        lv_obj_t * icon = lv_label_create(scr);
        lv_label_set_text(icon, menuItems[i]);
        
        lv_obj_align(icon, LV_ALIGN_CENTER, x, y);
        static lv_style_t style;
        lv_style_init(&style);
        // Set the font size (use a built-in font)
        lv_style_set_text_font(&style, &lv_font_montserrat_36); // Example: Montserrat font, size 20
        lv_style_set_text_color(&style, lv_color_white());  // Set text color to white
        // Apply the style to the label
        lv_obj_add_style(icon, &style, 0);
    }

    //icons
    
}

void list_sub_menu(lv_display_t *disp) {
    // Create a list object
    lv_obj_t *scr = lv_display_get_screen_active(disp);
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
    
     lv_obj_t *circle_main = lv_obj_create(scr);
    lv_obj_align(circle_main, LV_ALIGN_CENTER, 0, 0); 
    lv_obj_set_size(circle_main, 235,235);
    lv_obj_set_style_radius(circle_main, LV_RADIUS_CIRCLE, 0);          // Make it a perfect circle
    lv_obj_set_style_bg_opa(circle_main, LV_OPA_TRANSP, 0);             // Make the background transparent
    lv_obj_set_style_border_width(circle_main, 5, 0);                  // Set the border thickness
    lv_obj_set_style_border_color(circle_main, lv_color_white(), 0); // Set the border color
    lv_obj_set_style_border_opa(circle_main, LV_OPA_COVER, 0);  
    
    static lv_style_t dark_style;
    lv_style_init(&dark_style);

    // Set the background color to dark and text color to light
    lv_style_set_bg_color(&dark_style, lv_color_hex(0x2E2E2E));  // Dark gray background
    lv_style_set_text_color(&dark_style, lv_color_white());      // White text color
    lv_style_set_pad_all(&dark_style, 10);                       // Padding for better spacing



    const char *labels[4];
    labels[0] = "1 - LED WALL";
    labels[1] = "2 - BROKER WEATHER";
    labels[2] = "3 - BLINDS";
    labels[3] = "4 - SMTH";

    for(int i = 0; i<4; i++){
        // Add labels with the dark theme style
        lv_obj_t *label = lv_label_create(scr);
        lv_label_set_text(label, labels[i]);
        lv_obj_add_style(label, &dark_style, 0);
        lv_obj_align(label, LV_ALIGN_TOP_LEFT, 40, (30*i)+40);  // Position label at the top left
    }
    
    lv_obj_t * iconBack = lv_label_create(scr);
        lv_label_set_text(iconBack, LV_SYMBOL_LEFT);
        
        lv_obj_align(iconBack, LV_ALIGN_CENTER, 0, 80);
        static lv_style_t style;
        lv_style_init(&style);
        // Set the font size (use a built-in font)
        lv_style_set_text_font(&style, &lv_font_montserrat_36); // Example: Montserrat font, size 20
        lv_style_set_text_color(&style, lv_color_white());  // Set text color to white
        // Apply the style to the label
        lv_obj_add_style(iconBack, &style, 0);
    
    
}


void change_slider_value_CW(int32_t new_value) {
    lv_slider_set_value(sliderCW, new_value, LV_ANIM_ON);  // Set the value with animation
    printf("Slider value changed to: %ld\n", new_value);  // Print the new value
}
void change_slider_value_WW(int32_t new_value) {
    lv_slider_set_value(sliderWW, new_value, LV_ANIM_ON);  // Set the value with animation
    printf("Slider value changed to: %ld\n", new_value);  // Print the new value
}
void ledWall_sub_menu(lv_display_t *disp) {
    // Create a list object
    lv_obj_t *scr = lv_display_get_screen_active(disp);
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
    
     lv_obj_t *circle_main = lv_obj_create(scr);
    lv_obj_align(circle_main, LV_ALIGN_CENTER, 0, 0); 
    lv_obj_set_size(circle_main, 235,235);
    lv_obj_set_style_radius(circle_main, LV_RADIUS_CIRCLE, 0);          // Make it a perfect circle
    lv_obj_set_style_bg_opa(circle_main, LV_OPA_TRANSP, 0);             // Make the background transparent
    lv_obj_set_style_border_width(circle_main, 5, 0);                  // Set the border thickness
    lv_obj_set_style_border_color(circle_main, lv_color_white(), 0); // Set the border color
    lv_obj_set_style_border_opa(circle_main, LV_OPA_COVER, 0);  
    
    static lv_style_t dark_style;
    lv_style_init(&dark_style);

    // Set the background color to dark and text color to light
    lv_style_set_bg_color(&dark_style, lv_color_hex(0x2E2E2E));  // Dark gray background
    lv_style_set_text_font(&dark_style, &lv_font_montserrat_20);
    lv_style_set_text_color(&dark_style, lv_color_white());      // White text color
    lv_style_set_pad_all(&dark_style, 10);                       // Padding for better spacing


    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text(label, "LED WALL");
    lv_obj_add_style(label, &dark_style, 0);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, -86);  // Position label at the top left
    
    sliderCW = lv_slider_create(scr);

    lv_obj_set_size(sliderCW, 20, 120);  // Width 50, height 200 (vertical slider)
    lv_obj_align(sliderCW, LV_ALIGN_CENTER, 0, 0);  // Align it in the center of the screen
    lv_slider_set_range(sliderCW, 0, 100);  // Min: 0, Max: 100
    lv_slider_set_value(sliderCW, 100, LV_ANIM_ON);  // Start at 50 (mid-range)
    lv_obj_set_style_bg_color(sliderCW, lv_color_hex(0x0a8ffc), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(sliderCW, lv_color_hex(0x0a8ffc), LV_PART_KNOB);
    lv_obj_align(sliderCW, LV_ALIGN_CENTER, 40, 0);

    sliderWW = lv_slider_create(scr);

    lv_obj_set_size(sliderWW, 20, 120);  // Width 50, height 200 (vertical slider)
    lv_obj_align(sliderWW, LV_ALIGN_CENTER, 0, 0);  // Align it in the center of the screen
    lv_slider_set_range(sliderWW, 0, 100);  // Min: 0, Max: 100
    lv_slider_set_value(sliderWW, 0, LV_ANIM_ON);  // Start at 50 (mid-range)
    lv_slider_set_value(sliderWW, 100, LV_ANIM_ON);  // Start at 50 (mid-range)
    lv_obj_set_style_bg_color(sliderWW, lv_color_hex(0xfcb00a), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(sliderWW, lv_color_hex(0xfcb00a), LV_PART_KNOB);
    
    lv_obj_align(sliderWW, LV_ALIGN_CENTER, -40, 0);

    static lv_style_t style;
    lv_style_init(&style);
    // Set the font size (use a built-in font)
    lv_style_set_text_font(&style, &lv_font_montserrat_36); // Example: Montserrat font, size 20
    lv_style_set_text_color(&style, lv_color_white());  // Set text color to white

    lv_obj_t * iconUpWW = lv_label_create(scr);

    lv_label_set_text(iconUpWW, LV_SYMBOL_UP);
    lv_obj_align(iconUpWW, LV_ALIGN_CENTER, -80, -45);
    lv_obj_add_style(iconUpWW, &style, 0);

    lv_obj_t * iconDownWW = lv_label_create(scr);

    lv_label_set_text(iconDownWW, LV_SYMBOL_DOWN);
    lv_obj_align(iconDownWW, LV_ALIGN_CENTER, -80, 45);
    lv_obj_add_style(iconDownWW, &style, 0);

    lv_obj_t * iconUpCW = lv_label_create(scr);

    lv_label_set_text(iconUpCW, LV_SYMBOL_UP);
    lv_obj_align(iconUpCW, LV_ALIGN_CENTER, 80, -45);
    lv_obj_add_style(iconUpCW, &style, 0);

    lv_obj_t * iconDownCW = lv_label_create(scr);

    lv_label_set_text(iconDownCW, LV_SYMBOL_DOWN);
    lv_obj_align(iconDownCW, LV_ALIGN_CENTER, 80, 45);
    lv_obj_add_style(iconDownCW, &style, 0);


    lv_obj_t * iconBack = lv_label_create(scr);

    lv_label_set_text(iconBack, LV_SYMBOL_LEFT);
    lv_obj_align(iconBack, LV_ALIGN_CENTER, 0, 80);
    lv_obj_add_style(iconBack, &style, 0);
    
    
}