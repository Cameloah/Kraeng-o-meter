//
// Created by koorj on 07.01.2022.
//

#include "device_manager.h"

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "tools/loop_timer.h"

Adafruit_MPU6050 sensor_imu;

void device_manager_imu_init() {
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
}



void device_manager_filter_mavg(float* new_angles, float* angles) {
    for (int i = 0; i < 2; ++i) {
        angles[i] = (new_angles[i] * FILTER_MAVG_FACTOR + (1 - FILTER_MAVG_FACTOR) * angles[i]);
    }
}

Matrix<3> device_manager_get_accel_raw() {
    Matrix<3> return_buffer{};
    sensors_event_t a, g, temp;

    sensor_imu.getEvent(&a, &g, &temp); // pull sensor data
    return_buffer(0) = a.acceleration.x;
    return_buffer(1) = a.acceleration.y;
    return_buffer(2) = -1 * a.acceleration.z; // z axis of sensor is inverted

    return return_buffer;
}

Matrix<3> device_manager_get_accel_median() {
    sensors_event_t a, g, temp;
    float sensor_data[3][FILTER_MEDIAN_SIZE];

    for (int samples = 0; samples < FILTER_MEDIAN_SIZE; ++samples) {
        sensor_imu.getEvent(&a, &g, &temp); // pull sensor data
        sensor_data[0][samples] = a.acceleration.x;
        sensor_data[1][samples] = a.acceleration.y;
        sensor_data[2][samples] = -1 * a.acceleration.z; // z axis of sensor is inverted
        delay(FILTER_MEDIAN_INTERVAL); // wait
    }

    // sort elements
    float temporary;
    for (auto & k : sensor_data) {
        for (int i = 0; i < FILTER_MEDIAN_SIZE; i++) {
            for (int j = i+1; j < FILTER_MEDIAN_SIZE; j++) {
                if(k[i] > k[j]) {
                    temporary = k[i];
                    k[i] = k[j];
                    k[j] = temporary;
                }
            }
        }
    }

    Matrix<3> return_buffer{};
    for (int i = 0; i < 3; ++i) {
        return_buffer(i) = sensor_data[i][FILTER_MEDIAN_SIZE / 2];
    }

    return return_buffer;
}

Matrix<3> device_manager_get_accel_mean() {
    sensors_event_t a, g, temp;
    float sensor_data[3] = {0, 0, 0};

    for (int samples = 0; samples < FILTER_MEAN_SIZE; ++samples) {
        sensor_imu.getEvent(&a, &g, &temp); // pull sensor data
        sensor_data[0] += a.acceleration.x;
        sensor_data[1] += a.acceleration.y;
        sensor_data[2] += -1 * a.acceleration.z; // z axis of sensor is inverted
        delay(FILTER_MEAN_INTERVAL); // wait
    }

    Matrix<3> return_buffer{};
    for (int i = 0; i < 3; ++i) {
        return_buffer(i) = sensor_data[i] / FILTER_MEAN_SIZE;
    }

    return return_buffer;
}