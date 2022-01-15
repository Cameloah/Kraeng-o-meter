//
// Created by koorj on 09.01.2022.
//

#pragma once

#define MODULE_MEMORY_INIT_TIMER_MAX            100
typedef enum{
    MODULE_MEMORY_ERROR_NO_ERROR        = 0x00,
    MODULE_MEMORY_ERROR_READ_R_0_1      = 0x01,
    MODULE_MEMORY_ERROR_READ_R_1_2      = 0x02,
    MODULE_MEMORY_ERROR_INIT            = 0x03,
    MODULE_MEMORY_ERROR_WRITE           = 0x04,
    MODULE_MEMORY_ERROR_UNKNOWN         = 0xFF
} MODULE_MEMORY_ERROR_t;

enum module_memory_index {
    ROT_MAT_0_1 = 0,
    ROT_MAT_1_2 = 1
};

/// \brief tries to open flash storage for a predefined amount of times before reporting an init error
/// \return memory module standard error
MODULE_MEMORY_ERROR_t module_memory_init();

/// \brief Saves calibration data in form of rotation matrices.
/// \param user_buffer byte vector of data
/// \param rot_mat_index enum of which rotation matrix to save to.
/// \param user_size size of data vector in bytes
/// \return Returns init-error if storage cannot be opened. returns write error if at least one
///         matrix cannot be read.
MODULE_MEMORY_ERROR_t module_memory_set_calibration(uint8_t* user_buffer, uint8_t rot_mat_index, uint8_t user_size);

/// \brief Reads calibration data in form of rotation matrices from esp32 storage.
/// \param user_buffer_R_0_1 data vector of rotation matrix from device to sensor frame
/// \param user_buffer_R_1_2 data vector of rotation matrix from ship to device frame
/// \param user_size size of data vectors in bytes
/// \return Returns specific error if one of the rotation matrices cannot be read. Returns
///         specific R_0_1 matrix error if both matrices cannot be read, since then R_1_2 is required
///         to be re-calibrated anyway.
MODULE_MEMORY_ERROR_t module_memory_get_calibration(uint8_t* user_buffer_R_0_1, uint8_t* rot_mat_index_R_1_2, uint8_t user_size);

/// \brief erases entire nvs flash and reintializing it
/// \return memory module error
MODULE_MEMORY_ERROR_t module_memory_erase_namespace();