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
bool enable_serial_stream = false;
float angles_x_y[] = {0, 0};

void setup() {
    // Setup serial communication, when pc is connected
    Serial.begin(115200);

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
                put_state_mode(new_mode);
                Serial.print("Change mode to: ");
                Serial.println(config_data.state_mode);
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
                switch(module_memory_load_config()) {
                    case MODULE_MEMORY_ERROR_READ:
                        Serial.println("Keine Daten gefunden.");
                        break;

                    case MODULE_MEMORY_ERROR_NO_ERROR:
                        Serial.println("Kalibrierungsdaten aus Speicher: ");
                        Serial << "R_1_0: " << config_data.rot_mat_1_0 << '\n';
                        Serial << "R_2_1: " << config_data.rot_mat_2_1 << '\n';
                        Serial << "mode: " << config_data.state_mode << '\n';
                        break;

                    default:
                        Serial.println("Unbekannter Speicherfehler.");
                }
            }

            else if (!strcmp(rx_command_key, "erase")) {
                Serial.println("Erasing flash...");
                module_memory_erase_namespace();
                // print error if function returns
                Serial.println("Error erasing flash.");
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
    calculate_tiltangle_x_y(device_manager_get_accel_raw(), angles_x_y);

    if (enable_serial_stream) {
         switch (config_data.state_mode) {
             case 0:
                 Serial.print("Sensorkoordinaten, ");
                 break;

             case 1:
                 if (get_calibration_state() == 2) {
                     Serial.println("Nicht möglich. Gehäusekoordinatensystem nicht kalibriert.");
                     put_state_mode(0);
                     delay(2000);
                     break;
                 }
                 Serial.print("Gehäusekoordinaten, ");
                 break;

             case 2:
                 if (get_calibration_state() == 2) {
                     Serial.println("Nicht möglich. Schiffskoordinatensystem nicht kalibriert.");
                     put_state_mode(1);
                     delay(2000);
                     break;
                 }
                 else if (get_calibration_state() == 1){
                     Serial.println("Nicht möglich. Gehäusekoordinatensystem nicht kalibriert.");
                     put_state_mode(0);
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