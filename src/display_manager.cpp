//
// Created by koorj on 21.01.2022.
//

#include <Arduino_GFX_Library.h>
#include <TFT_eSPI.h>
#include <Free_Fonts.h>

#include "data/bitmap.h"
#include "display_manager.h"
#include "device_manager.h"
#include "module_memory.h"
#include "tools/loop_timer.h"

TFT_eSPI tft = TFT_eSPI();  // Invoke custom library
TFT_eSprite sprite_total = TFT_eSprite(&tft); // Sprite object for needle

Arduino_DataBus *bus = new Arduino_ESP32PAR8(15, 33, 4, 2, 12, 13, 26, 25, 17, 16, 27, 14);
Arduino_GFX *gfx = new Arduino_ILI9331(bus, 32);




void display_manager_init() {
    gfx->begin();
    gfx->fillScreen(BLACK);

    sprite_total.getPointer();
    sprite_total.bitmap_bg;
}

int center(int data) {
    return DISPLAY_MANAGER_DISPLAY_SIZE / 2 + data;
}

void display_manager_2dframe() {
    // get max thresholds in x and y aka two half axies of an ellipsoid
    float threshold_y_max = abs(config_data.threshold_x[0]) > abs(config_data.threshold_x[1]) ? abs(config_data.threshold_x[0]) : abs(config_data.threshold_x[1]);
    float threshold_x_max = abs(config_data.threshold_y[0]) > abs(config_data.threshold_y[1]) ? abs(config_data.threshold_y[0]) : abs(config_data.threshold_y[1]);

    // check whether sensor data is outside of ellipsoid
    float scaled_max_radius = sqrt(pow(angles_x_y[1] / threshold_x_max, 2) + pow(angles_x_y[0] / threshold_y_max, 2));
    if(scaled_max_radius > 1)
        scaled_max_radius = ART_2DFRAME_MAX_RAD / scaled_max_radius;
    else
        scaled_max_radius = ART_2DFRAME_MAX_RAD;

    int pos_thr_x[2] = {(int) (config_data.threshold_y[0] / threshold_x_max * scaled_max_radius), (int) (config_data.threshold_y[1] / threshold_x_max * scaled_max_radius)};
    int pos_thr_y[2] = {(int) (config_data.threshold_x[0] / threshold_y_max * scaled_max_radius), (int) (config_data.threshold_x[1] / threshold_y_max * scaled_max_radius)};

    int pos_dot_x = map((long) (angles_x_y[1] * 100), 0, (long) (threshold_x_max * 100), 0, (long) scaled_max_radius);
    int pos_dot_y = map((long) (angles_x_y[0] * 100), 0, (long) (threshold_y_max * 100), 0, (long) scaled_max_radius);

    gfx->fillScreen(BLACK);

    // draw frame axies
    gfx->drawFastHLine(0, DISPLAY_MANAGER_CENTER_Y, DISPLAY_MANAGER_DISPLAY_SIZE, WHITE);
    gfx->drawFastVLine(DISPLAY_MANAGER_CENTER_X, 0, DISPLAY_MANAGER_DISPLAY_SIZE, WHITE);

    // draw dot description
    // sprite_total.setFreeFont(FF6);
    gfx->drawFastHLine(center(0), center(pos_dot_y), pos_dot_x, WHITE);
    gfx->drawFastVLine(center(pos_dot_x), center(0), pos_dot_y, WHITE);
    gfx->drawFastHLine(center(pos_dot_x), center(pos_dot_y), -pos_dot_x, WHITE);
    gfx->drawFastVLine(center(pos_dot_x), center(pos_dot_y), -pos_dot_y, WHITE);

    int y_desc_start_x = pos_dot_x > 0 ? -ART_2DFRAME_DESC_DISTANCE_X : 14;
    int y_desc_start_y = pos_dot_y;
    int x_desc_start_x = pos_dot_x - 20;
    int x_desc_start_y = pos_dot_y > 0 ? -14 : ART_2DFRAME_DESC_DISTANCE_y;

    if (y_desc_start_y > ART_2DFRAME_DESC_Y_LIMIT)
        y_desc_start_y = ART_2DFRAME_DESC_Y_LIMIT;
    else if (y_desc_start_y < -ART_2DFRAME_DESC_Y_LIMIT)
        y_desc_start_y = -ART_2DFRAME_DESC_Y_LIMIT ;

    if (x_desc_start_x > ART_2DFRAME_DESC_X_LIMIT - ART_2DFRAME_DESC_DISTANCE_X +15)
        x_desc_start_x = ART_2DFRAME_DESC_X_LIMIT - ART_2DFRAME_DESC_DISTANCE_X +15;
    else if (x_desc_start_x < -ART_2DFRAME_DESC_X_LIMIT)
        x_desc_start_x = -ART_2DFRAME_DESC_X_LIMIT;

    gfx->setCursor(center(x_desc_start_x), center(x_desc_start_y));
    gfx->print(angles_x_y[1], 1);
    gfx->setCursor(gfx->getCursorX(), center(x_desc_start_y - 18));
    // gfx->setTextFont(1);
    gfx->print(" o");

    // gfx->setFreeFont(FF6);
    gfx->setCursor(center(y_desc_start_x), center(y_desc_start_y));
    gfx->print(angles_x_y[0], 1);
    gfx->setCursor(gfx->getCursorX(), center(y_desc_start_y - 18));
    // gfx->setTextFont(1);
    gfx->print(" o");

    // draw thresholds TODO: use draw_rect to draw
    gfx->drawFastHLine(center(-10), center(pos_thr_x[0]), 20, RED);
    gfx->drawFastHLine(center(-10), center(pos_thr_x[0]) +1, 20, RED);
    gfx->drawFastHLine(center(-10), center(pos_thr_x[1]), 20, RED);
    gfx->drawFastHLine(center(-10), center(pos_thr_x[1]) +1, 20, RED);
    gfx->drawFastVLine(center(pos_thr_y[0]), center(-10), 20, RED);
    gfx->drawFastVLine(center(pos_thr_y[0]) +1, center(-10), 20, RED);
    gfx->drawFastVLine(center(pos_thr_y[1]), center(-10), 20, RED);
    gfx->drawFastVLine(center(pos_thr_y[1]) +1, center(-10), 20, RED);

    // draw dot
    gfx->fillCircle(center(pos_dot_x), center(pos_dot_y), 10, RED);
}

void display_manager_update() {

    display_manager_2dframe();
    /*
    sprite_total.pushImage(0, 0, 240, 240, mercy);
    sprite_total.setCursor(20, 100, 4);
    sprite_total.setTextColor(TFT_WHITE);
    sprite_total.print("X: ");
    sprite_total.print(angles_x_y[0], 1);
    sprite_total.print("°");
    sprite_total.setCursor(120, 100, 4);
    sprite_total.print("Y: ");
    sprite_total.print(angles_x_y[1], 1);
    sprite_total.print("°");*/

    // sprite_total.setCursor(20, 150, 4);
    // sprite_total.print(loop_timer_get_loop_freq());

    //sprite_total.pushSprite(0, 0);
}