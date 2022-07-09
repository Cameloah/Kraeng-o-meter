//
// Created by koorj on 09.01.2022.
//

#pragma once

#include "BasicLinearAlgebra.h"

using namespace BLA;

#define MODULE_MEMORY_INIT_TIMER_MAX            100

typedef enum{
    MODULE_MEMORY_ERROR_NO_ERROR        = 0x00,
    MODULE_MEMORY_ERROR_READ_R_0_1      = 0x01,
    MODULE_MEMORY_ERROR_READ_R_1_2      = 0x02,
    MODULE_MEMORY_ERROR_INIT            = 0x03,
    MODULE_MEMORY_ERROR_READ            = 0x04,
    MODULE_MEMORY_ERROR_WRITE           = 0x05,
    MODULE_MEMORY_ERROR_BAD_DATA        = 0x06,
    MODULE_MEMORY_ERROR_UNKNOWN         = 0xFF
} MODULE_MEMORY_ERROR_t;

enum module_memory_index {
    ROT_MAT_0_1 = 0,
    ROT_MAT_1_2 = 1
};

typedef struct {
    uint8_t* data;
    uint8_t length;
} MODULE_MEMORY_SEG_DATA_t;

typedef struct {
    char wifi_ssid[50];
    char wifi_pw[50];
    bool flag_auto_update;
    bool flag_check_update;
    bool flag_external_warning;
    bool flag_device_calibration_state;
    bool flag_ship_calibration_state;
    uint8_t state_mode;
    float threshold_angle_x[2];
    float threshold_angle_y[2];
    float filter_mavg_factor;
    Matrix<3, 3> rot_mat_1_0;
    Matrix<3, 3> rot_mat_2_1;
} MODULE_MEMORY_CONFIG_t;

typedef struct {
    bool flag_external_warning;
    bool flag_device_calibration_state;
    bool flag_ship_calibration_state;
    uint8_t state_mode;
    float threshold_angle_x[2];
    float threshold_angle_y[2];
    float filter_mavg_factor;
    Matrix<3, 3> rot_mat_1_0;
    Matrix<3, 3> rot_mat_2_1;
} MODULE_MEMORY_CONFIG_LEGACY_t;

extern MODULE_MEMORY_CONFIG_t config_data;

/// \brief tries to open flash storage for a predefined amount of times before reporting an init error
/// \return memory module standard error
MODULE_MEMORY_ERROR_t module_memory_init();

/// \brief uses the externally available typedef config-data buffer and stores it as one
///        key in the flash storage
/// \return error if no such key could be created
MODULE_MEMORY_ERROR_t module_memory_save_config();

/// \brief loads saved config data in externally available typedef config-data buffer
/// \return error if no such key could be found
MODULE_MEMORY_ERROR_t module_memory_load_config();

/// \brief erases entire nvs flash and reinitializing it
/// \return memory module error
MODULE_MEMORY_ERROR_t module_memory_erase_namespace();

MODULE_MEMORY_ERROR_t module_memory_saveData(MODULE_MEMORY_SEG_DATA_t* user_buffer, uint8_t number_entries);
MODULE_MEMORY_ERROR_t module_memory_loadData(MODULE_MEMORY_SEG_DATA_t* user_buffer, uint8_t number_entries);