/*
 * IMUClass.h
 *
 *  Created on: May 20, 2015
 *      Author: adrien
 */

#ifndef PERIPHERALS_IMU_IMUCLASS_H_
#define PERIPHERALS_IMU_IMUCLASS_H_


#include "arch/AVR/MCU/MCU.h"
#include "arch/AVR/wire/Wire.h"
#include "Common.h"
#include "arch/AVR/I2C/I2C.h"
#include "math/LowPassFilter.h"

// Low pass filters
//-------------------------------------------
LowPassFilter2pVector3f accel_filter;
LowPassFilter2pVector3f gyro_filter;
Vector3f accelFiltered;
Vector3f gyroFiltered;

// Parameter of the IMU
//-------------------------------------------
// Thoses values change when config sent to the IMU is changed
#define ACC_LSB_PER_G 16384.0f // 16384.0f/9.81f // 256.0f // 16384.0f
#define GYRO_LSB_PER_G 16.4f // FS_SEL 0 131.0

// Enable vibration measurement

double accNoise = 0.0; // Noise accelerometer measure in G (means output steady equals 1 due to gravity

// Variables
long lastUpdateAHRS_Us = 0;

double raw_gyro_xrate = 0.0, raw_gyro_yrate = 0.0, raw_gyro_zrate = 0.0;
double gyroXrate = 0.0, gyroYrate = 0.0, gyroZrate = 0.0;
double gyroZangle = 0.0, rel_accZ = 0.0;
double rel_accX = 0.0;
double rel_accY = 0.0;

int Gyro_output[3], Accel_output[3], Mag_output[3];

float dt_IMU = 0.01;

float Gyro_cal_x,Gyro_cal_y,Gyro_cal_z,Accel_cal_x,Accel_cal_y,Accel_cal_z;

// Acceleration CF filtered
//------------------------------
float Accel_pitch = 0;
float Accel_roll = 0;

class IMU_Class {
protected:
	byte chip_address;
	byte who_i_am_register;
	bool measure_vibration;
	bool enable_gyro_calibration;
	bool enable_accel_calibration;

	double accLsbPerG;
	double gyroLsbPerDegS;
	double initRoll;
	double initPitch;

	float accelScale[3];

public:
	// Default constructor
	IMU_Class();

	// Measure vibration
	bool measureVibration() {return measure_vibration;}

	// Initialize parameters
	void initParameters();

	// Get IMU readings data
	virtual void getIMUReadings(int Gyro_out[], int Accel_out[]);

	// Setyp gyroscope and accelerometer
	virtual void setupGyro() = 0;

	// Update euler angles from IMU data
	void updateGyroData();

	// Calibrate gyroscope
	void calibrateGyro();

	// Calibrate acceleromter
	void calibrateAccel();

	// Getter
	//----------------------------------------------
	double getInitRoll() {
		return initRoll;
	}

	double getInitPitch() {
		return initPitch;
	}

};


IMU_Class::IMU_Class()
{
	initParameters();
}

void IMU_Class::initParameters() {
	measure_vibration = false;
	enable_gyro_calibration = false;
	enable_accel_calibration = false;
	accLsbPerG = 1.0;
	gyroLsbPerDegS = 1.0;
	initRoll = 0.0;
	initPitch = 0.0;

	for (int i = 0; i < 3; i ++) {
		accelScale[i] = 1.0;
	}

	accelFiltered = vect3fInstance(0.0, 0.0, 1.0);
}

//-------------------------------------------------------------
// Update AHRS (Attitude and Heading Reference System)
// using Kalman filter
// To transform acceleromter data into roll, pitch :
// See http://stackoverflow.com/questions/3755059/3d-accelerometer-calculate-the-orientation
// or http://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/
void IMU_Class::updateGyroData() {
	long currentTimeUs = micros() ;
	dt_IMU = (currentTimeUs - lastUpdateAHRS_Us) / S_TO_US;
	if (lastUpdateAHRS_Us == 0) {
		dt_IMU = 0.01; // Initially, dt equals 10 ms
	}

	lastUpdateAHRS_Us = currentTimeUs;

	// IMU date retrieving
	//-----------------------------------------------
	getIMUReadings(Gyro_output, Accel_output);

	// Scale accelerometer data
	//	for (int k = 0; k < 3; k ++)
	//	{
	//		Accel_output[k] = Accel_output[k] * accelScale[k];
	//	}


	// Accelerometer data and filters
	//-----------------------------------------------
	rel_accX = (Accel_output[0] - Accel_cal_x) / accLsbPerG;
	rel_accY = (Accel_output[1] - Accel_cal_y) / accLsbPerG;
	rel_accZ = (Accel_output[2] - Accel_cal_z) / accLsbPerG;


	// Low pass filter accelerometer
//	accelFiltered = accel_filter.apply(vect3fInstance(rel_accX, rel_accY, rel_accZ));
	accelFiltered.x = accelFiltered.x * 0.95 + 0.05 * rel_accX;
	accelFiltered.y = accelFiltered.y * 0.95 + 0.05 * rel_accY;
	accelFiltered.z = accelFiltered.z * 0.95 + 0.05 * rel_accZ;

	Accel_pitch = vectAccelToPitch(accelFiltered) * RAD2DEG;
	Accel_roll = vectAccelToRoll(accelFiltered) * RAD2DEG;

	// Gyro data and filters
	//-----------------------------------------------
	raw_gyro_xrate = ((Gyro_output[0] - Gyro_cal_x)/ gyroLsbPerDegS) * dt_IMU;
	raw_gyro_yrate = -((Gyro_output[1] - Gyro_cal_y)/ gyroLsbPerDegS) * dt_IMU; // Tilt positive when going nose goes high
	raw_gyro_zrate = -((Gyro_output[2] - Gyro_cal_z)/ gyroLsbPerDegS) * dt_IMU;

	// Low pass filter on gyro (DESACTIVATED bad results)
	//	gyroFiltered = gyro_filter.apply(vect3fInstance(raw_gyro_xrate, raw_gyro_yrate, raw_gyro_zrate));

	//	gyroXrate = gyroFiltered.x;
	//	gyroYrate = gyroFiltered.y;
	//	gyroZrate = gyroFiltered.z;

	double alphaGyroRate = 0.4;
	gyroXrate = (1-alphaGyroRate)*gyroXrate + alphaGyroRate*raw_gyro_xrate;
	gyroYrate = (1-alphaGyroRate)*gyroYrate + alphaGyroRate*raw_gyro_yrate;
	gyroZrate = (1-alphaGyroRate)*gyroZrate + alphaGyroRate*raw_gyro_zrate;

	// Integrate gyro z rate to have approx yaw based on inertial data
	gyroZangle = 0.99 * gyroZangle + gyroZrate * dt_IMU;
	NormRadAngle(gyroZangle);

	// Update gyro filtered vector
	gyroFiltered = vect3fInstance(gyroXrate, gyroYrate, gyroZrate);
}


void IMU_Class::calibrateGyro()
{
	float Gyro_cal_x_sample = 0;
	float Gyro_cal_y_sample = 0;
	float Gyro_cal_z_sample = 0;

	int i;

	int nbSampleCalib = 50; // 100 original
	int sampleDurationMs = 20; // 50 original


	for (i = 0; i < nbSampleCalib; i ++)
	{
		getIMUReadings(Gyro_output, Accel_output);

		// Computes sum
		Gyro_cal_x_sample += Gyro_output[0];
		Gyro_cal_y_sample += Gyro_output[1];
		Gyro_cal_z_sample += Gyro_output[2];

		delay(sampleDurationMs);
	}

	Gyro_cal_x = Gyro_cal_x_sample / nbSampleCalib;
	Gyro_cal_y = Gyro_cal_y_sample / nbSampleCalib;
	Gyro_cal_z = Gyro_cal_z_sample / nbSampleCalib;

	Logger.print("Gyro cal x; y; z : ");
	Logger.print(Gyro_cal_x);
	Logger.print("; ");
	Logger.print(Gyro_cal_y);
	Logger.print("; ");
	Logger.print(Gyro_cal_z);
	Logger.println(" ");
	delay(500);
}

void IMU_Class::calibrateAccel()
{
	float Accel_cal_x_sample = 0;
	float Accel_cal_y_sample = 0;
	float Accel_cal_z_sample = 0;

	int i;

	int nbSampleCalib = 50; // 100 original
	int sampleDurationMs = 20; // 50 original

	Logger.println("Wait during accelerometer calibration");

	for (i = 0; i < nbSampleCalib; i ++)
	{
		getIMUReadings(Gyro_output, Accel_output);

		Accel_cal_x_sample += Accel_output[0];
		Accel_cal_y_sample += Accel_output[1];
		Accel_cal_z_sample += Accel_output[2];

		delay(sampleDurationMs);
	}

	Accel_cal_x = Accel_cal_x_sample / nbSampleCalib;
	Accel_cal_y = Accel_cal_y_sample / nbSampleCalib;
	Accel_cal_z = accLsbPerG - (Accel_cal_z_sample / nbSampleCalib) ; //sortie a accLsbPerG LSB/g (gravite terrestre) => offset a accLsbPerG pour mise a 0


	Logger.print("Acc cal x; y; z : ");
	Logger.print(Accel_cal_x);
	Logger.print("; ");
	Logger.print(Accel_cal_y);
	delay(500);
	Logger.print("; ");
	Logger.println(Accel_cal_z);
	delay(500);

	initRoll = (RAD2DEG * vectAccelToRoll(vect3fInstance(Accel_cal_x / accLsbPerG, Accel_cal_y / accLsbPerG, (accLsbPerG + Accel_cal_z) / accLsbPerG)));
	initPitch = (RAD2DEG * vectAccelToPitch(vect3fInstance(Accel_cal_x / accLsbPerG, Accel_cal_y / accLsbPerG, (accLsbPerG + Accel_cal_z) / accLsbPerG)));

	Logger.print("init roll = ");
	Logger.println(initRoll);
	delay(100);
	Logger.print("init pitch = ");
	Logger.println(initPitch);
	delay(100);

	Logger.println("Wait while finding max value");
	float  accelMax[3];
	for (int k = 0; k < 3; k++) {
		accelMax[k] = 0.0;
	}

	for (i = 0; i < 150; i ++)
	{
		getIMUReadings(Gyro_output, Accel_output);

		for (int j = 0; j < 3; j ++) {
			if (abs(Accel_output[j]) > abs(accelMax[j]))
			{
				accelMax[j] = abs(Accel_output[j]);
			}
		}
		Logger.println(i);
		delay(100);
	}


	for (int l = 0; l < 3; l ++)
	{
		accelScale[l] = accelMax[l] / accLsbPerG;

		Logger.print("accelScale[");
		Logger.print(l);
		Logger.print("] = ");
		Logger.println(accelScale[l]);
		delay(100);
	}

	/**
	 *
Gyro cal x; y; z : -10.10; 7.60; -2.20
Acc cal x; y; z : -29.68; -131.92; -486.96
init roll = -0.48
init pitch = -0.11

max_acc_0 = 20100.00
max_acc_1 = -18824.00
max_acc_2 = 19716.00
	 */
}



#endif /* PERIPHERALS_IMU_IMUCLASS_H_ */
