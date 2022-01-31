//
// Created by koorj on 21.01.2022.
//

#pragma once

#define DISPLAY_MANAGER_CENTER_X                120
#define DISPLAY_MANAGER_CENTER_Y                120

#define DISPLAY_MANAGER_DISPLAY_SIZE            240

#define ART_2DFRAME_MAX_RAD                     (DISPLAY_MANAGER_DISPLAY_SIZE / 2.0 - 10)
#define ART_2DFRAME_DESC_DISTANCE_X             70
#define ART_2DFRAME_DESC_DISTANCE_y             30

#define ART_2DFRAME_DESC_X_LIMIT                105
#define ART_2DFRAME_DESC_Y_LIMIT                100


void display_manager_init();
void display_manager_update();