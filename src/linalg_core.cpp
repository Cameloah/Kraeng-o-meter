//
// Created by Jo Uni on 02/01/2022.
//

#include <Arduino.h>
#include <Wire.h>
#include "BasicLinearAlgebra.h"
#include <cmath>

#include "linalg_core.h"
#include "device_manager.h"
#include "display_manager.h"

#include "memory_module.h"
#include "ram_log.h"
#include "webserial_monitor.h"


MemoryModule config_data("kraengometer");


void serial_flush_() {
    String k = DualSerial.readString();
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
    // set up memory
    config_data.addParameter("ext_warning", false);
    config_data.addParameter("calib_device", false);
    config_data.addParameter("calib_ship", false);
    config_data.addParameter("state_mode", 2);
    config_data.addParameter("filter_factor", (float) 1.0);

    float threshold_angle_x[2] = {-1, 1};
    float threshold_angle_y[2] = {-1, 1};
    config_data.addParameter("th_angle_x", (uint8_t*) threshold_angle_x, sizeof(threshold_angle_x));
    config_data.addParameter("th_angle_y", (uint8_t*) threshold_angle_y, sizeof(threshold_angle_y));

    Matrix<3, 3> rot_mat_1_0;
    Matrix<3, 3> rot_mat_2_1;
    rot_mat_1_0.Fill(0);
    rot_mat_2_1.Fill(0);
    // create identity matrices
    for (int i = 0; i < 3; i++) {
        rot_mat_1_0(i, i) = 1;
        rot_mat_2_1(i, i) = 1;
    }
    config_data.addParameter("rot_mat_1_0", (uint8_t*) &rot_mat_1_0, sizeof(Matrix<3, 3>));
    config_data.addParameter("rot_mat_2_1", (uint8_t*) &rot_mat_2_1, sizeof(Matrix<3, 3>));

    esp_err_t retval = config_data.loadAllStrict();
    // try to load config data
    if (retval != ESP_OK) { //}== ESP_ERR_NVS_NOT_FOUND || retval == ESP_ERR_NOT_FOUND) {
        ram_log_notify(RAM_LOG_INFO, "Keine Konfigurationsdaten gefunden. Nutze Standardwerte.", true);
        if ((retval = config_data.saveAll()) != ESP_OK)
            DualSerial.println("Fehler beim Speichern der Standardwerte.");
    }

    else if (retval != ESP_OK) {
        DualSerial.println("Fehler beim Laden der Konfigurationsdaten.");
        return;
    }

    else {
        DualSerial.println("Konfigurationsdaten geladen.");
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

    DualSerial.println("Gehäuse-Kalibrierung");
    DualSerial.println("Stellen sie das Gehäuse hochkant auf den Tisch, die Anzeige zeigt nach oben.");
    DualSerial.println("Bestätigen Sie mit beliebiger Konsoleneingabe. Abbrechen mit 'abbruch'.");

    display_manager_print("Kalibrierung im Gehaeuse...");

    while(!DualSerial.available()) {}   // wait for any user input
    delay(50); // wait a bit for transfer of all serial data
    uint8_t rx_available_bytes = DualSerial.available();
    char rx_user_input[50];
    DualSerial.readBytes(rx_user_input, rx_available_bytes);
    serial_flush_();
    // extract first word
    char* rx_command_key = strtok(rx_user_input, " \n");
    //handle null pointer exception
    if (rx_command_key != nullptr) {
        if (!strcmp(rx_command_key, "abbruch")) {
            DualSerial.println("Kalibrierung abgebrochen.");
            return;
        }
    }

    Matrix<3> e_z_device = device_manager_get_accel_mean(); // measure g-vector
    e_z_device = normalize_(e_z_device); // normalize
    e_z_device *= -1;   // flip

    DualSerial.println("Kippen Sie das Gehäuse 90° zu sich, die Anzeige zeigt nach vorn und liegt gerade.");
    DualSerial.println("Bestätigen Sie mit beliebiger Konsoleneingabe. Abbrechen mit 'abbruch'.");

    while(!DualSerial.available()) {}   // wait for any user input
    delay(50); // wait a bit for transfer of all serial data
    rx_available_bytes = DualSerial.available();
    DualSerial.readBytes(rx_user_input, rx_available_bytes);
    serial_flush_();
    // extract first word
    rx_command_key = strtok(rx_user_input, " \n");
    //handle null pointer exception
    if (rx_command_key != nullptr) {
        if (!strcmp(rx_command_key, "abbruch")) {
            DualSerial.println("Kalibrierung abgebrochen.");
            return;
        }
    }

    Matrix<3> e_y_device =  device_manager_get_accel_mean(); // measure g-vector
    e_y_device = normalize_(e_y_device); // normalize
    e_y_device *= -1;   // flip

    Matrix<3> e_x_device = cross_(e_y_device, e_z_device); // calculate cross product
    e_x_device = normalize_(e_x_device); // normalize

    e_y_device = cross_(e_z_device, e_x_device); // calculate cross product

    Matrix<3, 3> rot_mat_0_1 = e_x_device || e_y_device || e_z_device;
    Matrix<3, 3> rot_mat_1_0 = ~rot_mat_0_1; // transpose mat

    config_data.set("rot_mat_1_0", (uint8_t*) &rot_mat_1_0, sizeof(Matrix<3, 3>));

    // all good so set flag high
    config_data.set("calib_device", true);
    DualSerial.println("Kalibrierung der Sensorlage im Gehäuse abgeschlossen. Soll die Kalibrierung gespeichert werden? 'ja', 'nein'");

    while (true) {
        while(!DualSerial.available()) {}   // wait for any user input
        delay(50); // wait a bit for transfer of all serial data
        rx_available_bytes = DualSerial.available();
        DualSerial.readBytes(rx_user_input, rx_available_bytes);
        serial_flush_();

        // extract first word
        char* rx_command_key = strtok(rx_user_input, " \n");
        //handle null pointer exception
        if (rx_command_key == nullptr) {
            DualSerial.println("Optionen sind 'ja' oder 'nein'.");
            continue;
        }

        if (!strcmp(rx_user_input, "nein")) {
            DualSerial.println("Abgeschlossen.");
            return;
        }

        else if (!strcmp(rx_user_input, "ja")) {
            // save rot mat to filesystem
            if (config_data.save("rot_mat_1_0") == ESP_OK && config_data.save("calib_device") == ESP_OK)
                DualSerial.println("Abgeschlossen. Kalibrierung wurde gespeichert.");
            else
                DualSerial.println("Fehler beim Speichern. Bei Neustart ist möglicherweise Re-Kalibrierung nötig!");
            return;
        }

        else DualSerial.println("Optionen sind 'ja' oder 'nein'.");
    }
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

    DualSerial.println("Schiffs-Kalibrierung");
    DualSerial.println("Achten Sie beim Einbau auf eine moeglichst genaue Ausrichtung Richtung Bug. Gehäuseschräglage wird korrigiert. ");
    DualSerial.println("Bestätigen Sie mit beliebiger Konsoleneingabe. Abbrechen mit 'abbruch'.");

    display_manager_print("Kalibrierung im Schiff...");

    while(!DualSerial.available()) {}   // wait for any user input
    delay(50); // wait a bit for transfer of all serial data
    uint8_t rx_available_bytes = DualSerial.available();
    char rx_user_input[50];
    DualSerial.readBytes(rx_user_input, rx_available_bytes);
    serial_flush_();
    // extract first word
    char* rx_command_key = strtok(rx_user_input, " \n");
    //handle null pointer exception
    if (rx_command_key != nullptr) {
        if (!strcmp(rx_command_key, "abbruch")) {
            DualSerial.println("Kalibrierung abgebrochen.");
            return;
        }
    }

    Matrix<3> e_z_ship = device_manager_get_accel_mean();// measure g-vector
    e_z_ship = normalize_(e_z_ship); // normalize
    e_z_ship *= -1;   // flip

    e_z_ship = *static_cast<Matrix<3, 3>*>(config_data.get("rot_mat_1_0")) * e_z_ship; // rotate to device frame

    Matrix<3> e_y_ship = cross_(e_z_ship, e_x); // calculate cross product
              e_y_ship = normalize_(e_y_ship); // normalize

    Matrix<3> e_x_ship = cross_(e_y_ship, e_z_ship); // calculate cross product

    Matrix<3, 3> rot_mat_1_2 = e_x_ship || e_y_ship || e_z_ship;
    Matrix<3, 3> result = ~rot_mat_1_2; // transpose mat

    config_data.set("rot_mat_2_1", (uint8_t*) &result, sizeof(Matrix<3, 3>));

    // all good so set flag high
    config_data.set("calib_ship", true);
    DualSerial.println("Kalibrierung der Gehäuselage im Schiff abgeschlossen. Soll die Kalibrierung gespeichert werden? 'ja', 'nein'");

    while (true) {
        while(!DualSerial.available()) {}   // wait for any user input
        delay(50); // wait a bit for transfer of all serial data
        rx_available_bytes = DualSerial.available();
        DualSerial.readBytes(rx_user_input, rx_available_bytes);
        serial_flush_();

        // extract first word
        char* rx_command_key = strtok(rx_user_input, " \n");
        //handle null pointer exception
        if (rx_command_key == nullptr) {
            DualSerial.println("Optionen sind 'ja' oder 'nein'.");
            continue;
        }

        if (!strcmp(rx_user_input, "nein")) {
            DualSerial.println("Abgeschlossen.");
            return;
        }

        else if (!strcmp(rx_user_input, "ja")) {
            // save rot mat to filesystem
            if (config_data.save("rot_mat_2_1") == ESP_OK && config_data.save("calib_ship") == ESP_OK)
                DualSerial.println("Abgeschlossen. Kalibrierung wurde gespeichert.");
            else
                DualSerial.println("Fehler beim Speichern. Bei Neustart ist möglicherweise Re-Kalibrierung nötig!");
            return;
        }

        else DualSerial.println("Optionen sind 'ja' oder 'nein'.");
    }
}

void calculate_tiltangle_x_y(Matrix<3> data_vector, float* return_buffer) {
    float new_angles[2];

    // rotate vector
    switch (*config_data.getInt("state_mode")) {
        case 1:
            data_vector = *static_cast<Matrix<3, 3>*>(config_data.get("rot_mat_1_0")) * data_vector;
            break;

        case 2:
            data_vector = *static_cast<Matrix<3, 3>*>(config_data.get("rot_mat_1_0")) * data_vector;
            data_vector = *static_cast<Matrix<3, 3>*>(config_data.get("rot_mat_2_1")) * data_vector;
    }

    data_vector *= -1; // invert vector

    // use zyx euler rotation angles
    data_vector = normalize_(data_vector);
    float c_1_3 = data_vector(0);
    float c_2_3 = data_vector(1);
    float c_3_3 = data_vector(2);
    float c_3_1 = -1 * c_1_3 / sqrt(c_1_3 * c_1_3 + c_3_3 * c_3_3);
    float c_3_2 = -1 * c_2_3 * c_3_3 / sqrt(c_1_3 * c_1_3 + c_3_3 * c_3_3);

    new_angles[0] = -atan2(c_3_2, c_3_3) * 180 / PI;
    new_angles[1] = -atan2( -1 * c_3_1, sqrt(c_3_2 * c_3_2 + c_3_3 * c_3_3)) * 180 / PI;


    device_manager_filter_mavg(new_angles, return_buffer);

    // little check for when there is nan reported. reinit IMU then
    if(isnan(return_buffer[0]) or isnan(return_buffer[1])) {
        DualSerial.println("NaN! Neustart..");
        esp_restart();
    }
}

int get_calibration_state() {
    if (*config_data.getBool("calib_device")) {
        if (*config_data.getBool("calib_ship"))
            return 2;
        return 1;
    }
    return 0;
}

void put_state_mode(uint8_t mode) {
    config_data.set("state_mode", mode, true);
}