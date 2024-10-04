#include <Arduino.h>
#include <Wire.h>
#include "BasicLinearAlgebra.h"

#include "tools/loop_timer.h"
#include "linalg_core.h"
#include "device_manager.h"
#include "display_manager.h"
#include "module_memory.h"
#include "user_interface.h"
#include "version.h"

#include "main_project_utils.h"
#include "ram_log.h"
#include "webserial_monitor.h"



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

    // Initialize modules
    display_manager_init();
    display_manager_print(ui_info().c_str());

    project_utils_init("Kraeng-o-Meter");
    device_manager_init();
    linalg_core_init();

    delay(100);
}

void loop() {
    // save t_0 time stamp in loop_timer
    t_0 = micros();

    ui_serial_comm_handler();
    project_utils_update();

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