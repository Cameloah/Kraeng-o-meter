//
// Created by koorj on 07.01.2022.
//

#pragma once

#include <BasicLinearAlgebra.h>

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

void device_manager_imu_init();

Matrix<3> device_manager_get_accel_median();
Matrix<3> device_manager_get_accel_mean();