//
// Created by Jo Uni on 02/01/2022.
//
#pragma once

extern Adafruit_MPU6050 sensor_imu;

/// \brief sets up rot matrices as identity matrices
///
void init_linalg_core();

/// \brief calibrates the device frame and creates rotation matrix to display sensor data
///        in device coordinates.
///
void calibrate_device();

/// \brief calibrates the ship frame and creates rotation matrix to display device data
///        in ship coordinates.
///
void calibrate_ship();

/// \brief calculates tilt angle in x and y axis by rotating first to device then to ship frame
/// \param sensor_data array with raw acceleration sensor data in m/s
/// \param return_buffer array with tilt angles in X and Y
///
void calculate_tiltangle_x_y(float* sensor_data, float* return_buffer);

/// \brief returns calibration state
/// \return 0 - calibrated, 1 - ship frame not calbrated, 2 - no calibration
///
int get_calibration_state();