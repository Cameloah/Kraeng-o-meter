#include <Arduino.h>
#include <Wire.h>
#include "BasicLinearAlgebra.h"

#include "tools/loop_timer.h"
#include "linalg_core.h"
#include "device_manager.h"
#include "module_memory.h"

#define FW_VERSION_MAJOR                    1
#define FW_VERSION_MINOR                    0
#define FW_VERSION_PATCH                    0

/* Changelog:
- 1.0.0 basic readout adapted from adafruit mpu6050 example
        display data and interface via serial comm
        calculates rot matrices from sensor to device housing and from device to ship during calibration
*/

// debug and system control options
#define SYSCTRL_LOOPTIMER               // enable loop frequency control, remember to also set the loop freq in the loop_timer.h

// state variables
uint8_t state_mode = 2;
bool enable_serial_stream = false;
float angles_x_y[] = {0, 0};

void setup() {
    // Setup serial communication, when pc is connected
    Serial.begin(115200);

    // esp_log_level_set("*", ESP_LOG_NONE); doesnt help yet

    delay(5000);
    Serial.print("Kräng-o-meter Version ");
    Serial.print(FW_VERSION_MAJOR);
    Serial.print(".");
    Serial.print(FW_VERSION_MINOR);
    Serial.print(".");
    Serial.println(FW_VERSION_PATCH);

    // initialize modules
    if (module_memory_init() != MODULE_MEMORY_ERROR_NO_ERROR)
        Serial.println("Error initializing memory module.");
    device_manager_imu_init();
    linalg_core_init();

    delay(100);
}

void serial_comm_handler() {
    // listen for user input
    uint8_t rx_available_bytes = Serial.available();
    if (rx_available_bytes > 0) {
        // import entire string until "\n"
        char rx_user_input[rx_available_bytes];
        Serial.readBytes(rx_user_input, rx_available_bytes);

        //extract first word as command key
        char* rx_command_key = strtok(rx_user_input, " \n");

        if (!strcmp(rx_command_key, "mode")) {
            // extract next word
            rx_command_key = strtok(nullptr, " \n");
            // convert to int
            uint8_t new_mode = *rx_command_key - '0';
            // check if within boundaries
            if (new_mode >= 0 && new_mode <= 2) {
                state_mode = new_mode;
                Serial.print("Change mode to: ");
                Serial.println(state_mode);
            }
            else Serial.println("Unknown mode. Modes are '0' for sensor, '1' for device and '2' for ship frame.");
        }

        else if (!strcmp(rx_command_key, "calibrate")) {
            // extract next word
            rx_command_key = strtok(nullptr, " \n");
            // convert to int
            uint8_t calib_mode = *rx_command_key - '0';
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

        else if (!strcmp(rx_command_key, "stream")) {
            // extract next word
            rx_command_key = strtok(nullptr, " \n");

            if (!strcmp(rx_command_key, "start")) {
                enable_serial_stream = true;
                return;
            }

            else if (!strcmp(rx_command_key, "stop"))
                enable_serial_stream = false;

            else
                Serial.println("Options are 'start', 'stop'.");
        }

        else if (!strcmp(rx_command_key, "memory")) {
            rx_command_key = strtok(nullptr, " \n");
            if (!strcmp(rx_command_key, "data")) {
                Matrix<3, 3> r_0_1;
                Matrix<3, 3> r_1_2;
                switch(module_memory_get_calibration((uint8_t*) &r_0_1, (uint8_t*) &r_1_2, sizeof(r_0_1))) {
                    case MODULE_MEMORY_ERROR_READ_R_0_1:
                        Serial.println("Keine Daten gefunden.");

                    case MODULE_MEMORY_ERROR_READ_R_1_2:
                        Serial << "R_0_1: " << r_0_1 << '\n';
                        Serial.println("Keine Schiffskalibrierung gefunden.");
                        break;

                    case MODULE_MEMORY_ERROR_NO_ERROR:
                        Serial.println("Kalibrierungsdaten aus Speicher: ");
                        Serial << "R_0_1: " << r_0_1 << '\n';
                        Serial << "R_1_2: " << r_1_2 << '\n';
                        break;

                    default:
                        Serial.println("Unbekannter Speicherfehler.");
                }
            }

            else if (!strcmp(rx_command_key, "erase")) {
                Serial.println("Erasing flash...");
                if (module_memory_erase_namespace() == MODULE_MEMORY_ERROR_NO_ERROR)
                    Serial.println("Flash erased.");
                else Serial.println("Error erasing flash.");
            }

            else Serial.println("Unknown command.");
        }

        else if (!strcmp(rx_command_key, "help")) {
            Serial.println(" ");
            Serial.println("List of available commands:");
            Serial.println("calibrate [frame] - calibrate Kräng-o-meter. '1' for device frame, '2' for ship frame");
            Serial.println("stream [command]  - toggle sensor data stream via serial port by adding 'start' or 'stop'");
            Serial.println("mode [frame]      - change streamed data frame. '0' for raw, '1' for device and '2' for ship frame");
            Serial.println("memory [command]  - access saved calibration data with 'data', erase all data with 'erase'");
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

void loop() {
    // save t_0 time stamp in loop_timer
    t_0 = micros();

    serial_comm_handler();

    // calculate angles
    calculate_tiltangle_x_y(device_manager_get_accel_raw(), angles_x_y, state_mode);

    if (enable_serial_stream) {
         switch (state_mode) {
             case 0:
                 Serial.print("Sensorkoordinaten, ");
                 break;

             case 1:
                 if (get_calibration_state() == 2) {
                     Serial.println("Nicht möglich. Gehäusekoordinatensystem nicht kalibriert.");
                     state_mode = 0;
                     delay(2000);
                     break;
                 }
                 Serial.print("Gehäusekoordinaten, ");
                 break;

             case 2:
                 if (get_calibration_state() == 2) {
                     Serial.println("Nicht möglich. Schiffskoordinatensystem nicht kalibriert.");
                     state_mode = 1;
                     delay(2000);
                     break;
                 }
                 else if (get_calibration_state() == 1){
                     Serial.println("Nicht möglich. Gehäusekoordinatensystem nicht kalibriert.");
                     state_mode = 0;
                     delay(2000);
                     break;
                 }
                 Serial.print("Schiffskoordinaten, ");
                 break;

             default:
                 break;
        }

        Serial.print("Neigung um Achse X: ");
        Serial.print(angles_x_y[0]);
        Serial.print("°, Y: ");
        Serial.print(angles_x_y[1]);
        Serial.println("°");
    }

    loop_timer++;   // iterate loop timer to track loop frequency

#ifdef SYSCTRL_LOOPTIMER
    // keep loop at constant cycle frequency
    loop_timer_check_cycle_freq();
#endif
}