#include <Arduino.h>
#include <Wire.h>
#include "BasicLinearAlgebra.h"

#include "tools/loop_timer.h"
#include "linalg_core.h"
#include "device_manager.h"
#include "module_memory.h"
#include "user_interface.h"

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

void loop() {
    // save t_0 time stamp in loop_timer
    t_0 = micros();

    ui_serial_comm_handler();

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