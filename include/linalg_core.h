//
// Created by Jo Uni on 02/01/2022.
//
#pragma once

#include "BasicLinearAlgebra.h"
#include "memory_module.h"

using namespace BLA;

extern MemoryModule config_data;

/// \brief sets up rot matrices as identity matrices
void linalg_core_init();

/// \brief calibrates the device frame and creates rotation matrix to display sensor data
///        in device coordinates.
void calibrate_device();

/// \brief calibrates the ship frame and creates rotation matrix to display device data
///        in ship coordinates.
void calibrate_ship();

/// \brief calculates tilt angle in x and y axis by rotating first to device then to ship frame
/// \param sensor_data array with raw acceleration sensor data in m/s
/// \param return_buffer array with tilt angles in X and Y
///
void calculate_tiltangle_x_y(Matrix<3> data_vector, float* return_buffer);

/// \brief returns calibration state
/// \return 0 - calibrated, 1 - ship frame not calbrated, 2 - no calibration
int get_calibration_state();

/// \brief sets the mode in the config data and saves it
/// \param mode
void put_state_mode(uint8_t mode);