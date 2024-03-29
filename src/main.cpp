#include <Arduino.h>
#include <Wire.h>
#include "BasicLinearAlgebra.h"

#include "tools/loop_timer.h"
#include "linalg_core.h"
#include "device_manager.h"
#include "display_manager.h"
#include "module_memory.h"
#include "user_interface.h"
#include "wifi_debugger.h"
#include "version.h"



/* Changelog:
 * - 1.1.0 ability to update firmware automatically from github
 * - 1.0.1 minor improvements such as ability to cancel calibration procedure
 *      and several minor bugfixes
 * - 1.0.0 basic readout adapted from adafruit mpu6050 example
 *      display data and interface via serial comm
 *      calculates rot matrices from sensor to device housing and from device to ship during calibration
*/

// debug and system control options
#define SYSCTRL_LOOPTIMER               // enable loop frequency control, remember to also set the loop freq in the loop_timer.h

void setup() {
    delay(1000);
    // Setup serial communication
    Serial.begin(115200);

    // initialize modules
    if (module_memory_init() != MODULE_MEMORY_ERROR_NO_ERROR)
        Serial.println("Error initializing memory module.");
    module_memory_load_config();
    display_manager_init();

    display_manager_print(ui_info().c_str());

    // only enable wifi when necessary
    if (config_data.flag_check_update) {
        uint8_t retval = WIFI_DEBUGGER_ERROR_UNKNOWN;
        //update routine
        Serial.println("In Update-Modus gestartet.");
        display_manager_print("Suche nach Updates...");
        // reset flag
        config_data.flag_check_update = false;
        module_memory_save_config();

        retval = wifi_debugger_init(config_data.wifi_ssid, config_data.wifi_pw, URL_FW_VERSION, URL_FW_BIN);
        if(retval == WIFI_DEBUGGER_ERROR_NO_ERROR) {
            retval = wifi_debugger_fwVersionCheck(FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH);
            if (retval == WIFI_DEBUGGER_ERROR_NO_ERROR)
                wifi_debugger_firmwareUpdate();
            else if (retval == WIFI_DEBUGGER_ERROR_NO_UPDATE)
                display_manager_print("FW ist aktuell!");
            else display_manager_print("Fehler.");
        }
        else if (retval == WIFI_DEBUGGER_ERROR_WIFI)
            display_manager_print("WLAN nicht gefunden.");
        else display_manager_print("Fehler.");

        // restarting esp
        delay(3000);
        Serial.println("ESP32 wird neu gestartet.");
        display_manager_print("Neustart...");
        esp_restart();
    }

    else {
        // normal startup
        Serial.println("In Normal-Modus gestartet.");
        device_manager_init();
        linalg_core_init();
        // if enabled, set flag to automatically check for update upon next restart
        if (config_data.flag_auto_update) {
            config_data.flag_check_update = true;
            module_memory_save_config();
        }
    }


    delay(100);
}

void loop() {
    // save t_0 time stamp in loop_timer
    t_0 = micros();

    ui_serial_comm_handler();

    // calculate angles
    if (enable_measurements)
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

        Serial << "Neigung um Achse X: " << angles_x_y[0] << "°, Y: " << angles_x_y[1] << "° ";

        if (enable_serial_verbose) {
#ifdef SYSCTRL_LOOPTIMER
            Serial << "Main loop freq: " << loop_timer_get_loop_freq() << "Hz ";
#endif
        }

        Serial << "\n";
    }

    device_manager_check_warning();
    display_manager_update();
    loop_timer++;   // iterate loop timer to track loop frequency

#ifdef SYSCTRL_LOOPTIMER
    // keep loop at constant cycle frequency
    loop_timer_check_cycle_freq();
#endif
}