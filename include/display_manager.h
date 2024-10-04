//
// Created by Cameloah on 21.01.2022.
//

#pragma once


#define DISPLAY_MANAGER_CENTER_X                120
#define DISPLAY_MANAGER_CENTER_Y                120

#define DISPLAY_MANAGER_DISPLAY_SIZE            240

#define ART_2DFRAME_MAX_RAD                     (DISPLAY_MANAGER_DISPLAY_SIZE / 2.0 - 10)
#define ART_2DFRAME_DESC_DISTANCE_X             60
#define ART_2DFRAME_DESC_DISTANCE_y             10

#define ART_2DFRAME_DESC_X_LIMIT                105
#define ART_2DFRAME_DESC_Y_LIMIT                75

#define ART_PRINT_TEXTFIELD_WIDTH               210
#define ART_PRINT_TEXTFIELD_HEIGHT              100
#define ART_PRINT_HOLD_MSEC                     5000
#define ART_PRINT_NUM_LINES                     3

#define DISPLAY_MANAGER_WARING_BLINK_FREQ       1

void display_manager_init();
void display_manager_update();
void display_manager_print(const char c[]);