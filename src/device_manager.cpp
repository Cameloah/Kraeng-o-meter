//
// Created by koorj on 07.01.2022.
//
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "device_manager.h"
#include "module_memory.h"
#include "tools/loop_timer.h"

Adafruit_MPU6050 sensor_imu;
// state variables
float angles_x_y[] = {0, 0};

bool flag_threshold_violation_angle_x = false;
bool flag_threshold_violation_angle_y = false;



void device_manager_init_imu() {
    // Try to initialize
    Serial.println("Searching for MPU6050 chip...");

    double t_0 = millis();
    while (!sensor_imu.begin()) {
        if (5000 < (millis() - t_0)) {
            Serial.println("No IMU found!");
            return;
        }
        delay(10);
    }
    Serial.println("MPU6050 Found!");

    // Set sensor ranges
    sensor_imu.setAccelerometerRange(MPU6050_ACCEL_RANGE_G);
    delay(10);
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

    delay(10);
    sensor_imu.setGyroRange(MPU6050_GYRO_RANGE_DEG);
    delay(10);
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

    delay(10);
    sensor_imu.setFilterBandwidth(MPU6050_FILTER_BW_HZ);
    delay(10);
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

void device_manager_init() {
    // imu
    device_manager_init_imu();

    // external GPIOs
    pinMode(PIN_EXTERNAL_WARNING_RELAY, OUTPUT);
    digitalWrite(PIN_EXTERNAL_WARNING_RELAY, HIGH);
}

void device_manager_check_warning() {
    // check if angles are outside of threshold
    if (angles_x_y[0] < config_data.threshold_angle_x[0] || angles_x_y[0] > config_data.threshold_angle_x[1])
        flag_threshold_violation_angle_x = true;

    if (angles_x_y[1] < config_data.threshold_angle_y[0] || angles_x_y[1] > config_data.threshold_angle_y[1])
        flag_threshold_violation_angle_y = true;

    // reset flags only if thresholds are sub-passed by a certain percent
    if (angles_x_y[0] > config_data.threshold_angle_x[0] * WARNING_FLAG_CLEAR_AT && angles_x_y[0] < config_data.threshold_angle_x[1] * WARNING_FLAG_CLEAR_AT)
        flag_threshold_violation_angle_x = false;

    if (angles_x_y[1] > config_data.threshold_angle_y[0] * WARNING_FLAG_CLEAR_AT && angles_x_y[1] < config_data.threshold_angle_y[1] * WARNING_FLAG_CLEAR_AT)
        flag_threshold_violation_angle_y = false;

    if (flag_threshold_violation_angle_x || flag_threshold_violation_angle_y)
    {
        // activate relay if config is activated
        if (config_data.flag_external_warning)
            digitalWrite(PIN_EXTERNAL_WARNING_RELAY, LOW);

        else digitalWrite(PIN_EXTERNAL_WARNING_RELAY, HIGH);
    }

    else digitalWrite(PIN_EXTERNAL_WARNING_RELAY, HIGH);
}

void device_manager_filter_mavg(const float* new_angles, float* angles) {
    for (int i = 0; i < 2; ++i) {
        angles[i] = (float) (new_angles[i] * (config_data.filter_mavg_factor / FREQ_LOOP_CYCLE_HZ)
                    + (1 - (config_data.filter_mavg_factor / FREQ_LOOP_CYCLE_HZ)) * angles[i]);
    }
}

Matrix<3> device_manager_get_accel_raw() {
    Matrix<3> return_buffer{0, 0, 0};
    sensors_event_t a, g, temp;
    memset(&a, 0x00, sizeof(sensors_event_t));

    if(!sensor_imu.getEvent(&a, &g, &temp)) // pull sensor data
        return return_buffer;

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