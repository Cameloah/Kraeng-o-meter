//
// Created by koorj on 19.01.2022.
//

#include "user_interface.h"
#include "Arduino.h"

#include "module_memory.h"
#include "device_manager.h"
#include "linalg_core.h"

bool enable_serial_stream = false;

void ui_config() {
    // extract next word
    char* sub_key = strtok(nullptr, " \n");

    // calibration of coordinate frames
    if (!strcmp(sub_key, "-r")) {
        // convert to int
        sub_key = strtok(nullptr, " \n");
        uint8_t calib_mode = *sub_key - '0';
        switch (calib_mode) {
            case 1:
                calibrate_device();
                break;
            case 2:
                calibrate_ship();
                break;
            default:
                Serial.println("Unknown mode. Calibration modes are '1' for device frame and '2' for ship frame.");
        }
    }

    // set thresholds
    else if (!strcmp(sub_key, "-t")) {
        float user_input;
        char* axis_key = strtok(nullptr, " \n");
        sub_key = strtok(nullptr, " \n");

        // import float
        user_input = atof(sub_key);

        if (!strcmp(sub_key, "--auto")) {
            // if option auto is provided copy data from main angles array
            if (*axis_key == 'f' || *axis_key == 'b')
                user_input = angles_x_y[0];
            else user_input = angles_x_y[1];
        }

        switch (*axis_key) {
            case 'f':
                config_data.threshold_x[0] = user_input;
                break;
            case 'b':
                config_data.threshold_x[1] = user_input;
                break;
            case 'l':
                config_data.threshold_y[0] = user_input;
                break;
            case 'r':
                config_data.threshold_y[1] = user_input;
                break;
            default:
                Serial.println("Invalid Parameters. At leas one of the following is missing.");
                Serial.println("config -t ['f' 'b' 'l' or 'r'] [value]  - set warning threshold for front, back, left or right of the ship");
                Serial.println("                               --auto   - acquire current inclination and set as warning threshold for");
                Serial.println("                                          front, back, left or right of the ship");
                return;
        }

        Serial << "Threshold for '" << axis_key << "' set to: " << user_input << "\n";
        module_memory_save_config();
    }

    else {
        Serial.println("Invalid Parameters. At leas one of the following is missing.");
        Serial.println("config -r [frame]                       - calibrate Kräng-o-meter. '1' for device frame, '2' for ship frame");
        Serial.println("       -t ['f' 'b' 'l' or 'r'] [value]  - set warning threshold for front, back, left or right of the ship");
        Serial.println("                               --auto   - acquire current inclination and set as warning threshold for");
        Serial.println("                                          front, back, left or right of the ship");
    }
}

void ui_mode() {
    // extract next word
    char* sub_key = strtok(nullptr, " \n");
    // convert to int
    uint8_t new_mode = *sub_key - '0';
    // check if within boundaries
    if (new_mode >= 0 && new_mode <= 2) {
        put_state_mode(new_mode);
        Serial.print("Change mode to: ");
        Serial.println(config_data.state_mode);
    }
    else Serial.println("Unknown mode. Modes are '0' for sensor, '1' for device and '2' for ship frame.");
}

void ui_stream() {
    // extract next word
    char* sub_key = strtok(nullptr, " \n");

    if (!strcmp(sub_key, "start")) {
        enable_serial_stream = true;
        return;
    }

    else if (!strcmp(sub_key, "stop"))
        enable_serial_stream = false;

    else
        Serial.println("Options are 'start', 'stop'.");
}

void ui_memory() {
    char* sub_key = strtok(nullptr, " \n");
    if (!strcmp(sub_key, "data")) {
        switch(module_memory_load_config()) {
            case MODULE_MEMORY_ERROR_READ:
                Serial.println("Keine Daten gefunden.");
                break;

            case MODULE_MEMORY_ERROR_NO_ERROR:
                Serial.println("Konfigurationsdaten aus Speicher: ");
                Serial << "external alarm: " << config_data.flag_external_warning << '\n';
                Serial << "device frame calibrated: " << config_data.flag_device_calibration_state << '\n';
                Serial << "ship frame calibrated: " << config_data.flag_ship_calibration_state << '\n';
                Serial << "warning threshold for x: " << config_data.threshold_x[0] << ", " << config_data.threshold_x[1] << '\n';
                Serial << "warning threshold for x: " << config_data.threshold_y[0] << ", " << config_data.threshold_y[1] << '\n';
                Serial << "R_1_0: " << config_data.rot_mat_1_0 << '\n';
                Serial << "R_2_1: " << config_data.rot_mat_2_1 << '\n';
                Serial << "mode: " << config_data.state_mode << '\n';
                break;

            default:
                Serial.println("Unbekannter Speicherfehler.");
        }
    }

    else if (!strcmp(sub_key, "erase")) {
        Serial.println("Erasing flash...");
        module_memory_erase_namespace();
        // print error if function returns
        Serial.println("Error erasing flash.");
    }

    else Serial.println("Unknown command.");
}

void ui_serial_comm_handler() {
    // listen for user input
    uint8_t rx_available_bytes = Serial.available();
    if (rx_available_bytes > 0) {
        // import entire string until "\n"
        char rx_user_input[rx_available_bytes];
        Serial.readBytes(rx_user_input, rx_available_bytes);

        //extract first word as command key
        char* rx_command_key = strtok(rx_user_input, " \n");

        if (!strcmp(rx_command_key, "mode"))
            ui_mode();

        else if (!strcmp(rx_command_key, "config"))
            ui_config();

        else if (!strcmp(rx_command_key, "stream"))
            ui_stream();

        else if (!strcmp(rx_command_key, "memory"))
            ui_memory();

        else if (!strcmp(rx_command_key, "help")) {
            Serial.println(" ");
            Serial.println("List of available commands:");
            Serial.println("config -r [frame]                       - calibrate Kräng-o-meter. '1' for device frame, '2' for ship frame");
            Serial.println("       -t ['f' 'b' 'l' or 'r'] [value]  - set warning threshold for front, back, left or right of the ship");
            Serial.println("                               --auto   - acquire current inclination and set as warning threshold for");
            Serial.println("                                          front, back, left or right of the ship");
            Serial.println(" ");
            Serial.println("stream [command]                        - toggle sensor data stream via serial port by adding 'start' or 'stop'");
            Serial.println(" ");
            Serial.println("mode [frame]                            - change streamed data frame. '0' for raw, '1' for device and '2' for ship frame");
            Serial.println(" ");
            Serial.println("memory [command]                        - access saved calibration data with 'data', erase all data with 'erase'");
            Serial.println(" ");
        }

        else {
            // unknown command
            Serial.println("Unknown command. To see list of available commands, type 'help'");
        }

        if (enable_serial_stream)
            delay(2000); // for readability when data stream is active
    }
}