#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>

#include "tools/loop_timer.h"
#include "linalg_core.h"

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

//______IMU_______________________________________
#define MPU6050_ACCEL_RANGE_G                   MPU6050_RANGE_2_G
#define MPU6050_GYRO_RANGE_DEG                  MPU6050_RANGE_500_DEG
#define MPU6050_FILTER_BW_HZ                    MPU6050_BAND_5_HZ

void setup(void) {
    // Setup serial communication, when pc is connected
    Serial.begin(115200);
    Serial.print("Kraeng-o-meter Version ");
    Serial.print(FW_VERSION_MAJOR);
    Serial.print(".");
    Serial.print(FW_VERSION_MINOR);
    Serial.print(".");
    Serial.println(FW_VERSION_PATCH);

    // Try to initialize
    Serial.println("Searching for MPU6050 chip...");
    while (!sensor_imu.begin()) {
        delay(10);
    }
    Serial.println("MPU6050 Found!");

    // Set sensor ranges
    sensor_imu.setAccelerometerRange(MPU6050_ACCEL_RANGE_G);
    Serial.print("Accelerometer range set to: ");
    switch (sensor_imu.getAccelerometerRange()) {
        case MPU6050_RANGE_2_G:
            Serial.println("+-2G");
            break;
        case MPU6050_RANGE_4_G:
            Serial.println("+-4G");
            break;
        case MPU6050_RANGE_8_G:
            Serial.println("+-8G");
            break;
        case MPU6050_RANGE_16_G:
            Serial.println("+-16G");
            break;
    }

    sensor_imu.setGyroRange(MPU6050_GYRO_RANGE_DEG);
    Serial.print("Gyro range set to: ");
    switch (sensor_imu.getGyroRange()) {
        case MPU6050_RANGE_250_DEG:
            Serial.println("+- 250 deg/s");
            break;
        case MPU6050_RANGE_500_DEG:
            Serial.println("+- 500 deg/s");
            break;
        case MPU6050_RANGE_1000_DEG:
            Serial.println("+- 1000 deg/s");
            break;
        case MPU6050_RANGE_2000_DEG:
            Serial.println("+- 2000 deg/s");
            break;
    }

    sensor_imu.setFilterBandwidth(MPU6050_FILTER_BW_HZ);
    Serial.print("Filter bandwidth set to: ");
    switch (sensor_imu.getFilterBandwidth()) {
        case MPU6050_BAND_260_HZ:
            Serial.println("260 Hz");
            break;
        case MPU6050_BAND_184_HZ:
            Serial.println("184 Hz");
            break;
        case MPU6050_BAND_94_HZ:
            Serial.println("94 Hz");
            break;
        case MPU6050_BAND_44_HZ:
            Serial.println("44 Hz");
            break;
        case MPU6050_BAND_21_HZ:
            Serial.println("21 Hz");
            break;
        case MPU6050_BAND_10_HZ:
            Serial.println("10 Hz");
            break;
        case MPU6050_BAND_5_HZ:
            Serial.println("5 Hz");
            break;
    }

    calibrate_device();
    calibrate_ship();

    Serial.println("");
    delay(100);
}

void loop() {
    // save t_0 time stamp in loop_timer
    t_0 = micros();

    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    sensor_imu.getEvent(&a, &g, &temp);

    // calculate angles
    float angles_x_y[] = {0, 0};
    float sensor_data[] = {a.acceleration.x, a.acceleration.y, a.acceleration.z};
    calculate_tiltangle_x_y(sensor_data, angles_x_y);

    switch (get_calibration_state()) {
        case 2:
            Serial.print("Rohdaten! Gehäusekoordinatensystem nicht kalibriert.");
            break;

        case 1:
            Serial.print("Gehäusekoordinaten! Schiffskoordinatensystem nicht kalibriert.");
            break;

        default:
            Serial.print("Calibrated.");
    }

    /* Print out the values */
    Serial.print("Neigung um Achse X: ");
    Serial.print(angles_x_y[0]);
    Serial.print(", Y: ");
    Serial.print(angles_x_y[1]);
    Serial.print(" in Grad.   ");


    loop_timer++;   // iterate loop timer to track loop frequency

#ifdef SYSCTRL_LOOPTIMER
    // keep loop at constant cycle frequency
    loop_timer_check_cycle_freq();
#endif
}