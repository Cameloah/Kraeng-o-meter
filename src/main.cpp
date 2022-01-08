#include <Arduino.h>
#include <Wire.h>

#include "tools/loop_timer.h"
#include "linalg_core.h"
#include "device_manager.h"

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
uint8_t state_mode = 0;

void setup(void) {
    // Setup serial communication, when pc is connected
    Serial.begin(115200);

    delay(3000);
    Serial.print("Kraeng-o-meter Version ");
    Serial.print(FW_VERSION_MAJOR);
    Serial.print(".");
    Serial.print(FW_VERSION_MINOR);
    Serial.print(".");
    Serial.println(FW_VERSION_PATCH);

    // initialize modules
    device_manager_imu_init();
    linalg_core_init();

    calibrate_device();
    calibrate_ship();

    delay(100);
}

void loop() {
    // save t_0 time stamp in loop_timer
    t_0 = micros();

    // listen for user input
    String readString;
    if(Serial.available()) {
        char c = Serial.read();
        if (c == 'm') {
            readString = Serial.readString();
            state_mode = readString.toInt();
            Serial.print("Change mode to: ");
            Serial.println(state_mode);
            delay(2000);
        }

        if (c == 'c') {
            readString = Serial.readString();
            uint8_t calib_mode = readString.toInt();
            switch (calib_mode) {
                case 1:
                    calibrate_device();
                    break;
                case 2:
                    calibrate_ship();
                    break;
            }
        }
    }

    Serial.print("Mode: ");
    Serial.print(state_mode);
    Serial.print(". ");

    // calculate angles
    float angles_x_y[] = {0, 0};
    calculate_tiltangle_x_y(device_manager_get_accel_mean(), angles_x_y, state_mode);

    switch (get_calibration_state()) {
        case 2:
            Serial.print("Rohdaten! Gehäusekoordinatensystem nicht kalibriert. ");
            break;

        case 1:
            Serial.print("Gehäusekoordinaten! Schiffskoordinatensystem nicht kalibriert. ");
            break;

        default:
            Serial.print("Kalibriert. ");
    }

    Serial.print("Neigung um Achse X: ");
    Serial.print(angles_x_y[0]);
    Serial.print("°, Y: ");
    Serial.print(angles_x_y[1]);
    Serial.println("°");

    loop_timer++;   // iterate loop timer to track loop frequency

#ifdef SYSCTRL_LOOPTIMER
    // keep loop at constant cycle frequency
    loop_timer_check_cycle_freq();
#endif
}