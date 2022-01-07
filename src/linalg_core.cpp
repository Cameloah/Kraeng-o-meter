//
// Created by Jo Uni on 02/01/2022.
//

#include "BasicLinearAlgebra.h"
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "../include/linalg_core.h"

using namespace BLA;

Adafruit_MPU6050 sensor_imu;

bool flag_device_calibration_state = false;
bool flag_ship_calibration_state = false;

Matrix<3, 3> rot_mat_0_1;   // define rot mat to device coordinates
Matrix<3, 3> rot_mat_1_2;   // define rot mat to ship coordinates

void init_linalg_core() {
    // fill rot matrices
    rot_mat_0_1.Fill(0);
    rot_mat_1_2.Fill(0);

    // create identity matrices
    for (int i = 0; i < 3; i++) {
        rot_mat_0_1(i, i) = 1;
        rot_mat_1_2(i, i) = 1;
    }
}

void calibrate_device() {
    /*
     * 1. wait for enter
     * 2. measure g vector, normalize, flip and save as e_z_device
     * 3. wait for enter
     * 4. measure g vector, normalize, flip and temp save as e_y_device
     * 5. calculate y cross z and save as e_x_device
     * 6. calculate z cross x and save as e_y_device
     * 7. save R_0_1 to later rotate sensor data to device frame
     */
    sensors_event_t a, g, temp;

    Serial.println("Gehäuse-Kalibrierung");
    Serial.println("Richten Sie das Gehäuse hochkant aus, die Anzeige zeigt nach oben.");
    Serial.println("Bestätigen Sie mit beliebiger Konsoleneingabe.");

    while(!Serial.available()) {}   // wait for any user input
    sensor_imu.getEvent(&a, &g, &temp); // measure g-vector
    float length = sqrt((a.acceleration.x * a.acceleration.x) +
                        (a.acceleration.y * a.acceleration.y) +
                        (a.acceleration.z * a.acceleration.z)); // calculate length
    Matrix<3> e_z_device = {a.acceleration.x / length, a.acceleration.y / length, a.acceleration.z / length};
    e_z_device *= -1;   // flip

    while(!Serial.available()) {}   // wait for any user input
    sensor_imu.getEvent(&a, &g, &temp); // measure g-vector
    length = sqrt((a.acceleration.x * a.acceleration.x) +
                  (a.acceleration.y * a.acceleration.y) +
                  (a.acceleration.z * a.acceleration.z)); // calculate length
    Matrix<3> e_y_device = {a.acceleration.x / length, a.acceleration.y / length, a.acceleration.z / length};
    e_y_device *= -1;   // flip

    Matrix<3> e_x_device = {(e_y_device(1) * e_z_device(2)) - (e_y_device(2) * e_z_device(1)),
                            (e_y_device(2) * e_z_device(0)) - (e_y_device(0) * e_z_device(2)),
                            (e_y_device(0) * e_z_device(1)) - (e_y_device(1) * e_z_device(0))}; // calculate cross product

    e_y_device(0) = (e_z_device(1) * e_x_device(2)) - (e_z_device(2) * e_x_device(1));
    e_y_device(1) = (e_z_device(2) * e_x_device(0)) - (e_z_device(0) * e_x_device(2));
    e_y_device(2) = (e_z_device(0) * e_x_device(1)) - (e_z_device(1) * e_x_device(0));  // calculate cross product

    Matrix<3, 3> rot_mat_1_0 = e_x_device || e_y_device || e_z_device;
    rot_mat_0_1 = ~rot_mat_1_0; // transpose mat

    // all good so set flag high
    flag_device_calibration_state = true;
}

void calibrate_ship() {
    /*
     * 1. wait for enter
     * 2. measure g vector, normalize, flip and save as e_z_ship
     * 3. draw orthnorm vec between e_z_ship and e_x_device, normalize and save as e_y_ship
     * 4. draw orthnorm vec between e_y_ship and e_z_ship and save as e_x_ship
     * 5. save R_1_2 to later rotate device data to ship frame
     */

    sensors_event_t a, g, temp;
    float e_x[] = {1, 0, 0};

    Serial.println("Schiffs-Kalibrierung");
    Serial.println("Achten Sie beim Einbau auf eine möglichst genaue Ausrichtung Richtung Bug.");
    Serial.println("Bestätigen Sie mit beliebiger Konsoleneingabe.");

    while(!Serial.available()) {}   // wait for any user input
    sensor_imu.getEvent(&a, &g, &temp); // measure g-vector
    float length = sqrt((a.acceleration.x * a.acceleration.x) +
                        (a.acceleration.y * a.acceleration.y) +
                        (a.acceleration.z * a.acceleration.z)); // calculate length
    Matrix<3> e_z_ship = {a.acceleration.x / length, a.acceleration.y / length, a.acceleration.z / length}; // in sensor frame
    e_z_ship *= -1;   // flip
    e_z_ship = rot_mat_0_1 * e_z_ship; // rotate to device frame

    Matrix<3> e_y_ship = {(e_z_ship(1) * e_x[2]) - (e_z_ship(2) * e_x[1]),
                          (e_z_ship(2) * e_x[0]) - (e_z_ship(0) * e_x[2]),
                          (e_z_ship(0) * e_x[1]) - (e_z_ship(1) * e_x[0])}; // calculate cross product

    Matrix<3> e_x_ship = {(e_y_ship(1) * e_z_ship(2)) - (e_y_ship(2) * e_z_ship(1)),
                          (e_y_ship(2) * e_z_ship(0)) - (e_y_ship(0) * e_z_ship(2)),
                          (e_y_ship(0) * e_z_ship(1)) - (e_y_ship(1) * e_z_ship(0))}; // calculate cross product

    Matrix<3, 3> rot_mat_2_1 = e_x_ship || e_y_ship || e_z_ship;
    rot_mat_1_2 = ~rot_mat_2_1; // transpose mat

    // all good so set flag high
    flag_ship_calibration_state = true;
}

void calculate_tiltangle_x_y(float* sensor_data, float* return_buffer) {
    // create sensor data vector
    Matrix<3> data_vector = {sensor_data[0], sensor_data[1], sensor_data[2]};

    // rotate vector
    data_vector = rot_mat_0_1 * rot_mat_1_2 * data_vector;

    // calculate angle using arctan2 and save in return_buffer
    return_buffer[0] = atan2(data_vector(1), data_vector(2)) * 180 / PI;
    return_buffer[1] = atan2(data_vector(0), data_vector(2)) * 180 / PI;
}

int get_calibration_state() {
    if(flag_device_calibration_state) {
        if(flag_ship_calibration_state)
            return 0;
        return 1;
    }
    return 2;
}