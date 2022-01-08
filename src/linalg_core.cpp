//
// Created by Jo Uni on 02/01/2022.
//

#include <Arduino.h>
#include <Wire.h>

#include "../include/linalg_core.h"
#include "../include/device_manager.h"


bool flag_device_calibration_state = false;
bool flag_ship_calibration_state = false;

Matrix<3, 3> rot_mat_0_1;   // define rot mat to device coordinates
Matrix<3, 3> rot_mat_1_2;   // define rot mat to ship coordinates

void _serial_flush() {
    byte w = 0;

    for (int i = 0; i < 10; i++)
    {
        while (Serial.available() > 0)
        {
            char k = Serial.read();
            w++;
            delay(1);
        }
        delay(1);
    }
}

float _length(Matrix<3> vector) {
    return sqrt(vector(0) * vector(0) + vector(1) * vector(1) + vector (2) * vector(2));
}

Matrix<3> _normalize(Matrix<3> vector) {
    float length = _length(vector);
    Matrix<3> unit_vector = {vector(0) / length, vector(1) / length, vector(2) / length};
    return unit_vector;
}

Matrix<3> _cross(Matrix<3> a, Matrix<3> b) {
    Matrix<3> result = {(a(1) * b(2)) - (a(2) * b(1)),
                            (a(2) * b(0)) - (a(0) * b(2)),
                            (a(0) * b(1)) - (a(1) * b(0))}; // calculate cross product
    return result;
}

void linalg_core_init() {
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

    Serial.println("Gehäuse-Kalibrierung");
    Serial.println("Stellen sie das Gehaeuse hochkant auf den Tisch, die Anzeige zeigt nach oben.");
    Serial.println("Bestätigen Sie mit beliebiger Konsoleneingabe.");

    while(!Serial.available()) {}   // wait for any user input
    _serial_flush();

    Matrix<3> e_z_device = device_manager_get_accel_median(); // measure g-vector
    e_z_device = _normalize(e_z_device); // normalize
    e_z_device *= -1;   // flip
    Serial << "e_z_device: " << e_z_device << '\n';

    Serial.println("Kippen Sie das Gehaeuse 90° zu sich, die Anzeige zeigt nach vorn und liegt gerade.");
    Serial.println("Bestätigen Sie mit beliebiger Konsoleneingabe.");

    while(!Serial.available()) {}   // wait for any user input
    _serial_flush();

    Matrix<3> e_y_device =  device_manager_get_accel_median(); // measure g-vector
    e_y_device = _normalize(e_y_device); // normalize
    e_y_device *= -1;   // flip

    Matrix<3> e_x_device = _cross(e_y_device, e_z_device); // calculate cross product
    e_x_device = _normalize(e_x_device); // normalize

    e_y_device = _cross(e_z_device, e_x_device); // calculate cross product

    Matrix<3, 3> rot_mat_1_0 = e_x_device || e_y_device || e_z_device;
    rot_mat_0_1 = ~rot_mat_1_0; // transpose mat
    Serial << "rot_mat_1_0: " << rot_mat_1_0 << '\n';
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

    Matrix<3> e_x = {1, 0, 0};

    Serial.println("Schiffs-Kalibrierung");
    Serial.println("Achten Sie beim Einbau auf eine moeglichst genaue Ausrichtung Richtung Bug. Gehaeuseschraeglage wird korrigiert. ");
    Serial.println("Bestaetigen Sie mit beliebiger Konsoleneingabe.");

    while(!Serial.available()) {}   // wait for any user input
    _serial_flush();

    Matrix<3> e_z_ship = device_manager_get_accel_median();// measure g-vector
    e_z_ship = _normalize(e_z_ship); // normalize
    e_z_ship *= -1;   // flip
    e_z_ship = rot_mat_0_1 * e_z_ship; // rotate to device frame

    Matrix<3> e_y_ship = _cross(e_z_ship, e_x); // calculate cross product
              e_y_ship = _normalize(e_y_ship); // normalize

    Matrix<3> e_x_ship = _cross(e_y_ship, e_z_ship); // calculate cross product

    Matrix<3, 3> rot_mat_2_1 = e_x_ship || e_y_ship || e_z_ship;
    rot_mat_1_2 = ~rot_mat_2_1; // transpose mat

    Serial << "rot_mat_2_1: " << rot_mat_2_1 << '\n';
    // all good so set flag high
    flag_ship_calibration_state = true;
}

void calculate_tiltangle_x_y(Matrix<3> data_vector, float* return_buffer, int mode) {
    // rotate vector
    switch (mode) {
        case 1:
            data_vector = rot_mat_0_1 * data_vector;
            break;

        case 2:
            data_vector = rot_mat_0_1 * data_vector;
            data_vector = rot_mat_1_2 * data_vector;
    }

    // Serial << "data_vector: " << data_vector << '\n';

    data_vector *= -1; // invert vector
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