/*
 * Sensor_IMU_Autoload.h
 *
 *  Created on: May 20, 2015
 *      Author: adrien
 */

#ifndef PERIPHERALS_IMU_SENSOR_IMU_H_
#define PERIPHERALS_IMU_SENSOR_IMU_H_

// Include all type of IMU
//#include "peripherals/IMU/Sensor_Gyro_10DOF.h"
//#include "peripherals/IMU/Sensor_Gyro_MPU9150.h"
//

#include "IMUClass.h"
#include "IMUMPU9150.h"
#include "IMUITG3200.h"

#define T_GYRO_ITG3200 1
#define T_GYRO_MPU9150 2
#define GYRO_TYPE T_GYRO_ITG3200 // T_GYRO_MPU9150 for breeze board v1 | T_GYRO_ITG3200 for arduino mega board



#if GYRO_TYPE == T_GYRO_MPU9150
IMU_MPU9150 imu;
#else
IMU_ITG3200 imu;
#endif

void setupGyro()
{
	imu.setupGyro();
}

void updateGyroData()
{
	imu.updateGyroData();
}

#endif /* PERIPHERALS_IMU_SENSOR_IMU_H_ */
