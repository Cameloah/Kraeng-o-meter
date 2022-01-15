//
// Created by koorj on 09.01.2022.
//

#include "Arduino.h"
#include "Preferences.h"
#include <nvs_flash.h>

#include "module_memory.h"

Preferences mem_handler;

MODULE_MEMORY_ERROR_t module_memory_init() {
    // move into directory, create if not existent
    uint8_t timer_init = 0;
    while (!mem_handler.begin("Calibration", false)) {
        mem_handler.end();
        timer_init++;
        if (timer_init > MODULE_MEMORY_INIT_TIMER_MAX)
            return MODULE_MEMORY_ERROR_INIT;
    }

    return MODULE_MEMORY_ERROR_NO_ERROR;
}

MODULE_MEMORY_ERROR_t module_memory_set_calibration(uint8_t* user_buffer, uint8_t rot_mat_index, uint8_t user_size) {
    MODULE_MEMORY_ERROR_t error = MODULE_MEMORY_ERROR_NO_ERROR;

    // write variable with name key
    switch (rot_mat_index) {
        case ROT_MAT_0_1:
            if (!mem_handler.putBytes("R_0_1", user_buffer, user_size))
                error = MODULE_MEMORY_ERROR_WRITE; // something went wrong during write
            break;

        case ROT_MAT_1_2:
            if (!mem_handler.putBytes("R_1_2", user_buffer, user_size))
                error = MODULE_MEMORY_ERROR_WRITE; // something went wrong during write
    }

    return error;
}

MODULE_MEMORY_ERROR_t module_memory_get_calibration(uint8_t* user_buffer_R_0_1, uint8_t* user_buffer_R_1_2, uint8_t user_size) {
    if(!mem_handler.getBytes("R_0_1", user_buffer_R_0_1, user_size)) {
        return MODULE_MEMORY_ERROR_READ_R_0_1;
    }

    if(!mem_handler.getBytes("R_1_2", user_buffer_R_1_2, user_size)) {
        return MODULE_MEMORY_ERROR_READ_R_1_2;
    }

    return MODULE_MEMORY_ERROR_NO_ERROR;
}

MODULE_MEMORY_ERROR_t module_memory_erase_namespace() {
    if(nvs_flash_erase() != ESP_OK)
        return MODULE_MEMORY_ERROR_WRITE;
    if(nvs_flash_init() != ESP_OK)
        return MODULE_MEMORY_ERROR_INIT;

    Serial.println("Erase complete. Restarting ESP...");
    delay(2000);
    esp_restart();
}