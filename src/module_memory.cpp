//
// Created by koorj on 09.01.2022.
//

#include "Arduino.h"
#include "Preferences.h"
#include <ArduinoJson.h>
#include <nvs_flash.h>

#include "module_memory.h"

Preferences mem_handler;

// TODO: add legacy config data format and read fct to transition from old set to new set
// TODO: change to SPIFFS and jsons and make module generalized. json file name as input from user --> new json instance
//       for every new file name, user inputs json keys, module loads json and returns data per key

MODULE_MEMORY_CONFIG_t config_data;

MODULE_MEMORY_ERROR_t module_memory_init() {
    // fill buffer with default values
    strcpy(config_data.wifi_ssid, "");
    strcpy(config_data.wifi_pw, "");
    config_data.flag_auto_update = false;
    config_data.flag_check_update = false;
    config_data.flag_external_warning = false;
    config_data.flag_device_calibration_state = false;
    config_data.flag_ship_calibration_state = false;
    config_data.state_mode = 2;
    config_data.threshold_angle_x[0] = -2;
    config_data.threshold_angle_x[1] = 2;
    config_data.threshold_angle_y[0] = -2;
    config_data.threshold_angle_y[1] = 2;
    config_data.filter_mavg_factor = 1.0;

    // move into directory, create if not existent
    uint8_t timer_init = 0;
    while (!mem_handler.begin("Config", false)) {
        mem_handler.end();
        timer_init++;
        if (timer_init > MODULE_MEMORY_INIT_TIMER_MAX)
            return MODULE_MEMORY_ERROR_INIT;
    }

    return MODULE_MEMORY_ERROR_NO_ERROR;
}

MODULE_MEMORY_ERROR_t module_memory_save_config() {
    if (!mem_handler.putBytes("config_data", (uint8_t*) &config_data, sizeof (MODULE_MEMORY_CONFIG_t)))
        return MODULE_MEMORY_ERROR_WRITE; // something went wrong during write

    return MODULE_MEMORY_ERROR_NO_ERROR;
}

MODULE_MEMORY_ERROR_t module_memory_load_config() {
    if (!mem_handler.getBytes("config_data", (uint8_t*) &config_data, sizeof (MODULE_MEMORY_CONFIG_t)))
        return MODULE_MEMORY_ERROR_READ;

    return MODULE_MEMORY_ERROR_NO_ERROR;
}

MODULE_MEMORY_ERROR_t module_memory_erase_namespace() {
    if (nvs_flash_erase() != ESP_OK)
        return MODULE_MEMORY_ERROR_WRITE;
    if (nvs_flash_init() != ESP_OK)
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