//
// Created by koorj on 07.01.2022.
//

#pragma once

#include <BasicLinearAlgebra.h>
#include "tools/loop_timer.h"

using namespace BLA;

//______IMU_______________________________________
#define MPU6050_ACCEL_RANGE_G                   MPU6050_RANGE_2_G
#define MPU6050_GYRO_RANGE_DEG                  MPU6050_RANGE_500_DEG
#define MPU6050_FILTER_BW_HZ                    MPU6050_BAND_5_HZ

//______Filter____________________________________
#define FILTER_MEDIAN_SIZE                      20
#define FILTER_MEDIAN_INTERVAL                  1
#define FILTER_MEAN_SIZE                        100
#define FILTER_MEAN_INTERVAL                    1
#define FILTER_MAVG_FACTOR                      1.0 / FREQ_LOOP_CYCLE_HZ

extern float angles_x_y[];

/// \brief initializes the imu communication, sets its accel, gyro and filter parameters
void device_manager_imu_init();

/// \brief applies a recursive moving average filter
/// \param new_angles pointer to new angle data in x and y
/// \param angles pointer to previous angle data
void device_manager_filter_mavg(float* new_angles, float* angles);

/// \brief returns the raw sensor data from the imu
/// \return buffer of sensor data in vector form
Matrix<3> device_manager_get_accel_raw();

/// \brief takes a predefined number of measurements from the imu and applies a median filter
/// \return buffer of sensor data in vector form
Matrix<3> device_manager_get_accel_median();

/// \brief takes a predefined number of measurements from the imu and applies a mean filter
/// \return buffer of sensor data in vector form
Matrix<3> device_manager_get_accel_mean();