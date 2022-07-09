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
TFT_eSprite sprite_monitor = TFT_eSprite(&tft);

Arduino_DataBus *bus = new Arduino_ESP32PAR8(15, 33, 4, 2, 12, 13, 26, 25, 17, 16, 27, 14);
Arduino_GFX *gfx = new Arduino_ILI9331(bus, 32);

uint32_t color_angle_x = WHITE;
uint32_t color_angle_y = WHITE;


void display_manager_init() {
    gfx->begin();
    gfx->fillScreen(BLACK);

    sprite_total.setColorDepth(8);
    sprite_total.createSprite(DISPLAY_MANAGER_DISPLAY_SIZE, DISPLAY_MANAGER_DISPLAY_SIZE);
    sprite_total.setSwapBytes(true); // Swap the byte order for pushImage() - corrects endianness

    sprite_monitor.setColorDepth(8);
    sprite_monitor.createSprite(ART_PRINT_TEXTFIELD_WIDTH, ART_PRINT_TEXTFIELD_HEIGHT);
    sprite_monitor.setSwapBytes(true); // Swap the byte order for pushImage() - corrects endianness
    sprite_monitor.fillScreen(BLUE);
}

int16_t _center(int16_t data) {
    return DISPLAY_MANAGER_DISPLAY_SIZE / 2 + data;
}

void _draw_ring(int32_t x_center, int32_t y_center, int32_t r_i, int32_t r_o, uint32_t color) {
    int32_t x_o, x_i;
    // iterate through y
    for (int i = -r_o; i < r_o; i++) {
        // calculate from where to where to fill pixels for each y row
        x_o = (int32_t) sqrt(pow(r_o, 2) - pow(i, 2));

        // if r is between r_o and r_i, no need sto skip pixel in the middle
        if (abs(i) > r_i)
            x_i = 0;

            // if r < r_i, pixels in the middle need to be skipped
        else x_i = (int32_t) sqrt(pow(r_i, 2) - pow(i, 2));

        // now iterate through each column in row y
        for (int k = -x_o; k < -x_i; k++)
            sprite_total.drawPixel(x_center + k, y_center + i, color);

        // and ignore pixel in the middle
        for (int k = x_i; k <= x_o; k++)
            sprite_total.drawPixel(x_center + k, y_center + i, color);
    }
}

void _draw_2dframe() {
    // get max thresholds in x and y aka two half axies of an ellipsoid
    float threshold_y_max = abs(config_data.threshold_angle_x[0]) > abs(config_data.threshold_angle_x[1]) ? abs(
            config_data.threshold_angle_x[0]) : abs(config_data.threshold_angle_x[1]);
    float threshold_x_max = abs(config_data.threshold_angle_y[0]) > abs(config_data.threshold_angle_y[1]) ? abs(
            config_data.threshold_angle_y[0]) : abs(config_data.threshold_angle_y[1]);

    // check whether sensor data is outside of ellipsoid
    auto scaled_max_radius = sqrt(pow(angles_x_y[1] / threshold_x_max, 2) + pow(angles_x_y[0] / threshold_y_max, 2));
    if (scaled_max_radius > 1.0)
        scaled_max_radius = ART_2DFRAME_MAX_RAD / scaled_max_radius;
    else
        scaled_max_radius = ART_2DFRAME_MAX_RAD;

    // calculate threshold marker and dot position
    int16_t pos_thr_x[2] = {(int16_t) (config_data.threshold_angle_y[0] / threshold_x_max * scaled_max_radius),
                            (int16_t) (config_data.threshold_angle_y[1] / threshold_x_max * scaled_max_radius)};
    int16_t pos_thr_y[2] = {(int16_t) (config_data.threshold_angle_x[0] / threshold_y_max * scaled_max_radius),
                            (int16_t) (config_data.threshold_angle_x[1] / threshold_y_max * scaled_max_radius)};

    auto pos_dot_x = (int16_t) map((long) (angles_x_y[1] * 1000), 0, (long) (threshold_x_max * 1000), 0,
                                   (long) scaled_max_radius);
    auto pos_dot_y = (int16_t) map((long) (angles_x_y[0] * 1000), 0, (long) (threshold_y_max * 1000), 0,
                                   (long) scaled_max_radius);


    // draw frame axies
    sprite_total.drawFastHLine(0, DISPLAY_MANAGER_CENTER_Y, DISPLAY_MANAGER_DISPLAY_SIZE, TFT_WHITE);
    sprite_total.drawFastVLine(DISPLAY_MANAGER_CENTER_X, 0, DISPLAY_MANAGER_DISPLAY_SIZE, TFT_WHITE);

    // draw dot description
    sprite_total.drawFastHLine(_center(0), _center(pos_dot_y), pos_dot_x, TFT_SILVER);
    sprite_total.drawFastVLine(_center(pos_dot_x), _center(0), pos_dot_y, TFT_SILVER);
    sprite_total.drawFastHLine(_center(pos_dot_x), _center(pos_dot_y), -pos_dot_x, TFT_SILVER);
    sprite_total.drawFastVLine(_center(pos_dot_x), _center(pos_dot_y), -pos_dot_y, TFT_SILVER);

    int y_desc_start_x = pos_dot_x > 0 ? -ART_2DFRAME_DESC_DISTANCE_X : 10;
    int y_desc_start_y;

    if (pos_dot_y <= (sprite_total.fontHeight(4) / 2 + 7) && pos_dot_y > 0)
        y_desc_start_y = sprite_total.fontHeight(4) / 2 - 3;
    else if (pos_dot_y > -(sprite_total.fontHeight(4) / 2 + 5) && pos_dot_y <= 0)
        y_desc_start_y = -(sprite_total.fontHeight(4) + 1);
    else y_desc_start_y = pos_dot_y - (sprite_total.fontHeight(4) / 2 - 2);

    int x_desc_start_x;
    if (pos_dot_x <= 25 && pos_dot_x > 0)
        x_desc_start_x = 6;
    else if (pos_dot_x > -40 && pos_dot_x <= 0)
        x_desc_start_x = -60;
    else x_desc_start_x = pos_dot_x - 20;

    int x_desc_start_y = pos_dot_y > 0 ? -28 : ART_2DFRAME_DESC_DISTANCE_y;

    if (y_desc_start_y > ART_2DFRAME_DESC_Y_LIMIT)
        y_desc_start_y = ART_2DFRAME_DESC_Y_LIMIT;
    else if (y_desc_start_y < -ART_2DFRAME_DESC_Y_LIMIT - sprite_total.fontHeight(GFXFF))
        y_desc_start_y = -ART_2DFRAME_DESC_Y_LIMIT - sprite_total.fontHeight(GFXFF);

    if (x_desc_start_x > ART_2DFRAME_DESC_X_LIMIT - ART_2DFRAME_DESC_DISTANCE_X + 10)
        x_desc_start_x = ART_2DFRAME_DESC_X_LIMIT - ART_2DFRAME_DESC_DISTANCE_X + 10;
    else if (x_desc_start_x < -ART_2DFRAME_DESC_X_LIMIT)
        x_desc_start_x = -ART_2DFRAME_DESC_X_LIMIT;


    sprite_total.setTextColor(color_angle_y);
    sprite_total.setCursor(_center(x_desc_start_x), _center(x_desc_start_y), 4);
    sprite_total.print(angles_x_y[1], 1);
    sprite_total.setCursor(sprite_total.getCursorX(), _center(x_desc_start_y + 4 - sprite_total.fontHeight(4) / 2), 2);
    sprite_total.print(" o");

    sprite_total.setTextColor(color_angle_x);
    sprite_total.setCursor(_center(y_desc_start_x), _center(y_desc_start_y), 4);
    sprite_total.print(angles_x_y[0], 1);
    sprite_total.setCursor(sprite_total.getCursorX(), _center(y_desc_start_y + 4 - sprite_total.fontHeight(4) / 2), 2);
    sprite_total.print(" o");

    // draw threshold markers
    sprite_total.drawRect(_center(-10), _center(pos_thr_y[0]), 20, 2, RED);
    sprite_total.drawRect(_center(-10), _center(pos_thr_y[1]), 20, 2, RED);
    sprite_total.drawRect(_center(pos_thr_x[0]), _center(-10), 2, 20, RED);
    sprite_total.drawRect(_center(pos_thr_x[1]), _center(-10), 2, 20, RED);

    // draw dot
    sprite_total.fillCircle(_center(pos_dot_x), _center(pos_dot_y), 10, RED);
}

uint32_t counter_warning_blink = 0;

void _draw_alarm() {
    if (flag_threshold_violation_angle_x || flag_threshold_violation_angle_y) {
        if (counter_warning_blink <= FREQ_LOOP_CYCLE_HZ / DISPLAY_MANAGER_WARING_BLINK_FREQ) {
            _draw_ring(_center(0), _center(0), 110, 120, RED);
            if (flag_threshold_violation_angle_x)
                color_angle_x = RED;
            if (flag_threshold_violation_angle_y)
                color_angle_y = RED;
        } else color_angle_x = color_angle_y = WHITE;

        if (counter_warning_blink > 2 * FREQ_LOOP_CYCLE_HZ / DISPLAY_MANAGER_WARING_BLINK_FREQ)
            counter_warning_blink = 0;

        counter_warning_blink++;
    } else {
        counter_warning_blink = 0;
        color_angle_x = color_angle_y = WHITE;
    }
}

uint64_t time_last_print = 0;

void display_manager_print(const char c[]) {
    // first reset print hold timer
    time_last_print = millis();

    char* input = strdup(c);                    // make copy of string to loose const pointer attribute
    String line[ART_PRINT_NUM_LINES];           // prepare one string buffer for each line
    uint8_t index_line = 0;                     // to keep track how many line are needed
    char* word = strtok(input, " \n");          // extract first word
    uint8_t pixels_avail = ART_PRINT_TEXTFIELD_WIDTH;
    while(word != nullptr && index_line < ART_PRINT_NUM_LINES) {
        // we iterate through the input string until no words left or all lines full
        if (sprite_monitor.textWidth(word, 4) > pixels_avail) {
            // the word does not fit in the remaining space any more
            index_line++;                               // therefore switch to new line
            pixels_avail = ART_PRINT_TEXTFIELD_WIDTH;   // reset available space
        }
        pixels_avail -= sprite_monitor.textWidth(word, 4);          // keep track on how much space is left
        line[index_line].concat(word);                                  // add word to line buffer
        line[index_line].concat(" ");                               // also a line space please
        word = strtok(nullptr, " \n");                                  // extract next word
    }
    // clear sprite
    sprite_monitor.fillScreen(BLACK);
    for (uint8_t i = 0; i <= index_line && i < ART_PRINT_NUM_LINES; i++) {
        // iterate through all filled line buffer and distribute content evenly in text field
        sprite_monitor.drawCentreString(line[i], ART_PRINT_TEXTFIELD_WIDTH / 2,
                                        (i+1) * ART_PRINT_TEXTFIELD_HEIGHT / (2+index_line) - sprite_monitor.fontHeight(4) / 2,
                                        4);
    }

    sprite_monitor.drawRoundRect(0, 0, ART_PRINT_TEXTFIELD_WIDTH, ART_PRINT_TEXTFIELD_HEIGHT, 5, WHITE);

    // update display at least once to display print
    display_manager_update();
}

void display_manager_update() {
    // first set the sprite to black to reset all the pixels
    sprite_total.fillScreen(TFT_BLACK);
    // now draw the coordinate system and the angle data
    _draw_2dframe();
    // draw the alarm
    _draw_alarm();

    // draw the monitor window and hold for predefined time
    if (millis() < time_last_print + (ART_PRINT_HOLD_MSEC)) {
        sprite_monitor.pushToSprite(&sprite_total, _center(-ART_PRINT_TEXTFIELD_WIDTH / 2),
                                    _center(-ART_PRINT_TEXTFIELD_HEIGHT / 2));
    }

    // execute and display the sprite on screen my calling the driver and passing the sprite data pointer
    gfx->draw8bitRGBBitmap(0, 0, (uint8_t*) sprite_total.getPointer(), 240, 320);
}

