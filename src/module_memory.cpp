//
// Created by koorj on 09.01.2022.
//

#include "Arduino.h"
#include "Preferences.h"
#include <ArduinoJson.h>
#include <nvs_flash.h>

#include "module_memory.h"

Preferences mem_handler;

MODULE_MEMORY_CONFIG_t config_data;

MODULE_MEMORY_SEG_DATA_t config_data_array[] = {
        {(uint8_t*) &config_data.flag_external_warning, sizeof (config_data.flag_external_warning)},
        {(uint8_t*) &config_data.mode,                  sizeof (config_data.mode)},
        {(uint8_t*) &config_data.rot_mat_1_0,           sizeof (config_data.rot_mat_1_0)},
        {(uint8_t*) &config_data.rot_mat_1_0,           sizeof (config_data.rot_mat_1_0)},
};

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

MODULE_MEMORY_ERROR_t module_memory_saveData(MODULE_MEMORY_SEG_DATA_t* user_buffer, uint8_t number_entries) {
    if (user_buffer == nullptr)
        return MODULE_MEMORY_ERROR_BAD_DATA;

    String Output;
    DynamicJsonDocument json(1024);

    // loop though all user data entries and create json arrays
    for (uint8_t entry = 0; entry < number_entries; ++entry) {

        JsonArray byte_array;                               // create json array
        char data_key[sizeof (number_entries)];             // create char buffer for key
        sprintf(data_key, "%ld", entry);                    // convert loop index to key
        byte_array = json.createNestedArray(data_key);      // attach key to current json array

        // fill current json array with user data
        for (int byte = 0; byte < user_buffer[entry].length; ++byte) {
            byte_array.add(user_buffer[entry].data[byte]);
        }
    }

    serializeJson(json, Output);
    mem_handler.putString("DataJson", Output);

    return MODULE_MEMORY_ERROR_NO_ERROR;
}

MODULE_MEMORY_ERROR_t module_memory_loadData(MODULE_MEMORY_SEG_DATA_t* user_buffer, uint8_t number_entries) {
    if (user_buffer == nullptr)
        return MODULE_MEMORY_ERROR_BAD_DATA;

    String Input;
    DynamicJsonDocument json(1024);
    DeserializationError error;

    Input = mem_handler.getString("ConfJson");

    error = deserializeJson(json, Input);
    if (error)
        return MODULE_MEMORY_ERROR_READ;

    for (uint8_t entry = 0; entry < number_entries; ++entry) {

        char data_key[sizeof (number_entries)];             // create char buffer for key
        sprintf(data_key, "%ld", entry);                    // convert loop index to key

        // fill user data with current json array
        for (int byte = 0; byte < user_buffer[entry].length; ++byte) {
            user_buffer[entry].data[byte] = json[data_key][byte];
        }
    }
    
    return MODULE_MEMORY_ERROR_NO_ERROR;
}