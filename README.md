# Kraeng-o-meter
A device that measures the gravity vector using an inertia measurement unit and displays small tilt angles of a ship in two axies with high resolution.

![Kraeng-o-meter](https://user-images.githubusercontent.com/66171172/158064480-a04bc056-ee0c-41a2-8f8a-2e9fbea39458.jpg)

## Feature list
- measure tilt angles of +/- 5° with 0.05° precision
- display derivation from intial position on a 1.5 inch round display
- four configurable thresholds, two for each axis to trigger warning signal when exceeded
- display warning signal and trigger external warning signal using a relais
- configure options such as filter hardness, thresholds and usage of the external alarm
- USB serial communication handler to configure and calibrate the device
- save all configuration and calibration data to internal storage


## The Hardware
Like many of my projects this one is based on an ESP32 as the main micro-controller because of its relative high computational power, excellent periphery and affordability, all in a small package. The IMU is the famous MPU6050 and based on the assumptions and limitations of the entire project it was well capable of the job. A complete list of components is given below.

- *MCU:* ESP32 on a 38 pin Development Board
- *IMU:* MPU6050 on a GY-521 breakout board
- *Display:* 1.5 inch round LCD display with an ILI9331 driver
- *Relais:* TONGLING JQC-3FF-S-Z 5VDC relais on a no-brand breakout board
- *Power:* 5-30V to 3.3V 1A DC-DC no-brand converter

## Assumptions
The following assumptions were formulated, which significantly simplified the maths and eliminated several problems like e.g. gimbal-lock and drift.

- The measuring range should be around +/- 5° from the initial position, therefore simple pitch and roll angles are sufficient and gimbal lock is irrelevant.
- The resolution should be at least 0.1°.
- The rate of changes in tilt angles is comparably low, therefore the calculation is based on the accelleration measurements only. No gyroscope is taken into account.
- Reaction time can be low, therefore heavy filtering with introduced phaseshift can be used.

## The Math
To correctly display the tilt angles in two axies, the device needs to know the ships axies of pitch and roll rotation. To find the correct rotation-matrix, a series of two calibration procedures need to be perfomed once. the resulting rotation-matrices are multiplied to compute the final rotation-matrix to transform the accelleration measurements from the sensor frame to the ship frame.

At this point its worth noting that IMU-libraries that are based on the MPU6050 lib by Jeff Rowberg (e.g. https://github.com/electroniccats/mpu6050?utm_source=platformio&utm_medium=piohome) can pull the fully computed jaw-pitch-roll angles from the MPU6050's internal DMP but in this project I wanted to perfrom these calculations myself, giving me more control about the following two calibration algorithms.

### Calibration of the sensor-orientation in the device housing
To compute the rotation matrix from the potentially unknown sensor-orientation in the housing of the device to the device's internal frame, the following steps were implemented:

1. Request stable, upright orientation of the device by the user.
2. measure the g-vector, normalize, invert its direction and temporarily save it as the e_z_device vector representing the z-axis of the device frame in the sensor frame.
3. Request the user to tilt the device forwards by roughly 90°.
4. again measure the g-vector, normalize, invert and temporarily save it as the e_y_device vector representing the y-axis of the device frame in the sensor frame.
5. calculate the cross product of e_y_device and e_z_device to generate a vector perpendicular to the plane spanned by these two vectors. Normalize and save it as the final e_x_device vector representing the x-axis of the device frame in the sensor frame
6. calculate the cross product of e_z_device and e_x_device to ensure perpendicularity and update the e_y_device vector
7. The matrix R_0_1 consists of the 3 vertical vectors e_x_device, e_y_device and e_z_device and represents the device frame orientation in the sensor frame.
8. Since rotation-matrices are orthonormal, the desired rotation matrix R_1_0 to convert vectors in the sensor frame to the device frame is obtained by taking the transposed matrix of R_0_1.

The result is a simple, reliable, calibation procedure that can handle inperfect execution by the user and only relies on an accurate measurement in step 1.

### Calibration of the device-orientation in the ship
Since the Kräng-o-meter does not account for any yaw-rotation this axis is dependend on the mounting orientation in the ship. The device will always assume the ships roll-axis (y-axis of the ship frame) to be perpendicular to its device x-axis. Therefore, during the mounting process the 'front-side' of the device should point towards the front of the ship. 

To compute the rotation matrix from the previously calibrated device frame to the ship frame, the following steps were implemented:

1. Mount the device on the ship
2. measure the g-vector, normalize, invert its direction and save it as the e_z_ship vector representing the z-axis of the ship frame in device frame.
3. compute an orthonormal vector between e_z_ship and e_x_device using the cross-product, normalize and save it as the e_y_ship vector representing the y-axis of the ship frame in the device frame
4. compute an orthonormal vector between e_y_ship and e_z_ship and save it as the e_x_ship vector representing the x-axis of the ship frame in the device frame
5. compute the desired R_2_1 as the transposed of R_1_2 that consists of e_x_ship, e_y_ship and e_z_ship.

The final rotation matrix R_2_0 to transform sensor data to the ship frame is computed by matrix multiplication of R_1_0 and R_2_1.
