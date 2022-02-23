//
// Created by Jo Uni on 02/01/2022.
//

#include <Arduino.h>
#include <Wire.h>
#include "BasicLinearAlgebra.h"
#include <cmath>

#include "../include/linalg_core.h"
#include "../include/device_manager.h"
#include "../include/module_memory.h"

void serial_flush_() {
    String k = Serial.readString();
}

float length_(Matrix<3> &vector) {
    return std::sqrt(vector(0) * vector(0)
                     + vector(1) * vector(1)
                     + vector(2) * vector(2));
}

Matrix<3> normalize_(Matrix<3> &vector) {
    float length = length_(vector);
    Matrix<3> unit_vector = {vector(0) / length, vector(1) / length, vector(2) / length};
    return unit_vector;
}

Matrix<3> cross_(Matrix<3> &a, Matrix<3> &b) {
    Matrix<3> result = {(a(1) * b(2)) - (a(2) * b(1)),
                            (a(2) * b(0)) - (a(0) * b(2)),
                            (a(0) * b(1)) - (a(1) * b(0))}; // calculate cross product
    return result;
}

void linalg_core_init() {
    // check for existing calibration data
    if (module_memory_load_config() != MODULE_MEMORY_ERROR_NO_ERROR) {
        Serial.println("Keine Konfigurationsdaten gefunden.");

        // fill rot matrices
        config_data.rot_mat_1_0.Fill(0);
        config_data.rot_mat_2_1.Fill(0);

        // create identity matrices
        for (int i = 0; i < 3; i++) {
            config_data.rot_mat_1_0(i, i) = 1;
            config_data.rot_mat_2_1(i, i) = 1;
        }
        return;
    }

    else Serial.println("Konfigurationsdaten aus Speicher geladen");
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
    Serial.println("Stellen sie das Gehäuse hochkant auf den Tisch, die Anzeige zeigt nach oben.");
    Serial.println("Bestätigen Sie mit beliebiger Konsoleneingabe.");
    while(!Serial.available()) {}   // wait for any user input
    serial_flush_();

    Matrix<3> e_z_device = device_manager_get_accel_mean(); // measure g-vector
    e_z_device = normalize_(e_z_device); // normalize
    e_z_device *= -1;   // flip

    Serial.println("Kippen Sie das Gehäuse 90° zu sich, die Anzeige zeigt nach vorn und liegt gerade.");
    Serial.println("Bestätigen Sie mit beliebiger Konsoleneingabe.");
    while(!Serial.available()) {}   // wait for any user input
    serial_flush_();

    Matrix<3> e_y_device =  device_manager_get_accel_mean(); // measure g-vector
    e_y_device = normalize_(e_y_device); // normalize
    e_y_device *= -1;   // flip

    Matrix<3> e_x_device = cross_(e_y_device, e_z_device); // calculate cross product
    e_x_device = normalize_(e_x_device); // normalize

    e_y_device = cross_(e_z_device, e_x_device); // calculate cross product

    Matrix<3, 3> rot_mat_0_1 = e_x_device || e_y_device || e_z_device;
    config_data.rot_mat_1_0 = ~rot_mat_0_1; // transpose mat

    // all good so set flag high
    config_data.flag_device_calibration_state = true;

    // save rot mat to filesystem
    if(module_memory_save_config() == MODULE_MEMORY_ERROR_NO_ERROR)
        Serial.println("Kalibrierung wurde gespeichert.");
    else
        Serial.println("Fehler beim Speichern. Bei Neustart ist Re-Kalibrierung nötig!");
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
    Serial.println("Achten Sie beim Einbau auf eine moeglichst genaue Ausrichtung Richtung Bug. Gehäuseschräglage wird korrigiert. ");
    Serial.println("Bestätigen Sie mit beliebiger Konsoleneingabe.");

    while(!Serial.available()) {}   // wait for any user input
    serial_flush_();

    Matrix<3> e_z_ship = device_manager_get_accel_mean();// measure g-vector
    e_z_ship = normalize_(e_z_ship); // normalize
    e_z_ship *= -1;   // flip
    e_z_ship = config_data.rot_mat_1_0 * e_z_ship; // rotate to device frame

    Matrix<3> e_y_ship = cross_(e_z_ship, e_x); // calculate cross product
              e_y_ship = normalize_(e_y_ship); // normalize

    Matrix<3> e_x_ship = cross_(e_y_ship, e_z_ship); // calculate cross product

    Matrix<3, 3> rot_mat_1_2 = e_x_ship || e_y_ship || e_z_ship;
    config_data.rot_mat_2_1 = ~rot_mat_1_2; // transpose mat

    // all good so set flag high
    config_data.flag_ship_calibration_state = true;

    // save rot mat to filesystem
    if(module_memory_save_config() == MODULE_MEMORY_ERROR_NO_ERROR)
        Serial.println("Kalibrierung wurde gespeichert.");
    else
        Serial.println("Fehler beim Speichern. Bei Neustart ist Re-Kalibrierung nötig!");

}

void calculate_tiltangle_x_y(Matrix<3> data_vector, float* return_buffer) {
    float new_angles[2];
    // rotate vector
    switch (config_data.state_mode) {
        case 1:
            data_vector = config_data.rot_mat_1_0 * data_vector;
            break;

        case 2:
            data_vector = config_data.rot_mat_1_0 * data_vector;
            data_vector = config_data.rot_mat_2_1 * data_vector;
    }

    data_vector *= -1; // invert vector

    // calculate angle using atan2 and save in return_buffer
    /*
    new_angles[0] = atan2(data_vector(1), data_vector(2)) * 180 / PI;
    new_angles[1] = atan2(data_vector(0), data_vector(2)) * 180 / PI;*/


    // use zyx euler rotation angles
    data_vector = normalize_(data_vector);
    float c_1_3 = data_vector(0);
    float c_2_3 = data_vector(1);
    float c_3_3 = data_vector(2);
    float c_3_1 = -1 * c_1_3 / sqrt(c_1_3 * c_1_3 + c_3_3 * c_3_3);
    float c_3_2 = -1 * c_2_3 * c_3_3 / sqrt(c_1_3 * c_1_3 + c_3_3 * c_3_3);

    new_angles[0] = -atan2(c_3_2, c_3_3) * 180 / PI;
    new_angles[1] = -atan2( -1 * c_3_1, sqrt(c_3_2 * c_3_2 + c_3_3 * c_3_3)) * 180 / PI;

    /*
    // use Jones rotation angles
    float c_1_3 = data_vector(0);
    float c_2_3 = data_vector(1);
    float c_3_3 = data_vector(2);
    new_angles[0] = atan2( c_2_3, sqrt(c_1_3 * c_1_3 + c_3_3 * c_3_3)) * 180 / PI;
    new_angles[1] = atan2( c_1_3, sqrt(c_2_3 * c_2_3 + c_3_3 * c_3_3)) * 180 / PI;*/

    device_manager_filter_mavg(new_angles, return_buffer);
}

int get_calibration_state() {
    if(config_data.flag_device_calibration_state) {
        if(config_data.flag_ship_calibration_state)
            return 0;
        return 1;
    }
    return 2;
}

void put_state_mode(uint8_t mode) {
    config_data.state_mode = mode;
    module_memory_save_config();
}