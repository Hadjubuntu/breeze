/*
 * Sensor_Gyro_10DOF.h
 * IMU 10-dof based on gyro, accelerometer, magnometer and barometer.
 *
 * Gyro : ITG3200D
 * Acc : ADXL345
 *
 *  Created on: 11 oct. 2014
 *      Author: Adrien Hadj-Salah
 */

#ifndef SENSOR_GYRO_10DOF_H_
#define SENSOR_GYRO_10DOF_H_


#include "arch/AVR/MCU/MCU.h"
#include "arch/AVR/wire/Wire.h"
#include "Common.h"
#include "arch/AVR/I2C/I2C.h"
#include "math/LowPassFilter.h"


#define ITG3200_CHIP_ADDRESS 0x68
#define ITG3200_WHO_IM_AM_REG 0x0
#define ADXL345_CHIP_ADDRESS 0x53


#define MEASURE_VIBRATION 0
#define ENABLE_IMU_CALIBRATION 1

#define ENABLE_COMPASS 0
void MPU9150_setupCompass();

// Low pass filters
//-------------------------------------------
LowPassFilter2pVector3f accel_filter;
LowPassFilter2pVector3f gyro_filter;
Vector3f accelFiltered;
Vector3f gyroFiltered;

// Parameter of the IMU
//-------------------------------------------
// Thoses values change when config sent to the IMU is changed
#define ACC_LSB_PER_G 256.0f // 16384.0f/9.81f // 256.0f // 16384.0f
#define GYRO_LSB_PER_G 14.375f // FS_SEL 0 131.0

// Enable vibration measurement

double accNoise = 0.0; // Noise accelerometer measure in G (means output steady equals 1 due to gravity

// Variables
long lastUpdateAHRS_Us = 0;

// Initial values from accelerometer
float init_roll = 0.0, init_pitch = 0.0;

double raw_gyro_xrate = 0.0, raw_gyro_yrate = 0.0, raw_gyro_zrate = 0.0;
double gyroXrate = 0.0, gyroYrate = 0.0, gyroZrate = 0.0;
double gyroZangle = 0.0, rel_accZ = 0.0;
double rel_accX = 0.0;
double rel_accY = 0.0;

int Gyro_output[3], Accel_output[3], Mag_output[3];

float dt_IMU = 0.01;

bool acc_z_initialized = false;
float initial_acc_z_bias = 0.0;
float acc_z_on_efz = 0.0;
float climb_rate = 0.0;

float Gyro_cal_x,Gyro_cal_y,Gyro_cal_z,Accel_cal_x,Accel_cal_y,Accel_cal_z;

// Acceleration CF filtered
//------------------------------
float Accel_pitch = 0;
float Accel_roll = 0;


//lecture gyroscope - datasheet ITG3200
//-------------------------------------
void getGyroscopeReadings(int Gyro_out[])
{
	byte buffer[6];
	readFrom(ITG3200_CHIP_ADDRESS,0x1D,6,buffer);

	Gyro_out[0]=(((int)buffer[0]) << 8 ) | buffer[1];
	Gyro_out[1]=(((int)buffer[2]) << 8 ) | buffer[3];
	Gyro_out[2]=(((int)buffer[4]) << 8 ) | buffer[5];
}

//lecture accelerometre - datasheet ADXL345
//-----------------------------------------
void getAccelerometerReadings(int Accel_out[])
{
	byte buffer[6];
	readFrom(ADXL345_CHIP_ADDRESS,0x32,6,buffer);

	Accel_out[0]=(((int)buffer[1]) << 8 ) | buffer[0];
	Accel_out[1]=(((int)buffer[3]) << 8 ) | buffer[2];
	Accel_out[2]=(((int)buffer[5]) << 8 ) | buffer[4];
}



// read IMU data - datasheet ITG3200
//-------------------------------------
void getIMUReadings(int Gyro_out[], int Accel_out[])
{
	getGyroscopeReadings(Gyro_out);
	getAccelerometerReadings(Accel_out);
}

// TODO Use fast_atan2 if not enough fast for CPU ?

// Fast vector acceleration to roll conversion
float vectAccelToRoll(Vector3f acc3f) {
	return atan2(acc3f.y, acc3f.z);
}

// Fast vector acceleration to pitch conversion
float vectAccelToPitch(Vector3f acc3f) {
	return atan2(acc3f.x, pythagorous2(acc3f.z, acc3f.y));
}

//-------------------------------------------
// Initialize IMU with calibration values
void setupGyroC() {

	// Prepare filters
	//---------------------------------------------
	accel_filter.set_cutoff_frequency(800, 20);
	gyro_filter.set_cutoff_frequency(800, 127);


	// Configure accelerometer and gyroscope
	//---------------------------------------------

	writeTo(ADXL345_CHIP_ADDRESS,0x2D,0x00); // power ctlr
	delay(5);
	writeTo(ADXL345_CHIP_ADDRESS,0x2D,0xff); // power ctlr
	delay(5);
	writeTo(ADXL345_CHIP_ADDRESS,0x2D,0x08); //accel en mode mesure
	delay(5);
	writeTo(ADXL345_CHIP_ADDRESS,0x31,0b00); //accel 11 bits - +/-2g
	delay(5);
	writeTo(ADXL345_CHIP_ADDRESS,0x2c,0x0d); // 800 hz output
	delay(5);


	writeTo(ITG3200_CHIP_ADDRESS,0x16,0x1A); //gyro +/-2000 deg/s + passe-bas a 100Hz
	delay(5);
	writeTo(ITG3200_CHIP_ADDRESS,0x15,0x09); //gyro echantillonage a 100Hz

	delay(200);

#if ENABLE_IMU_CALIBRATION == 1

	float Gyro_cal_x_sample = 0;
	float Gyro_cal_y_sample = 0;
	float Gyro_cal_z_sample = 0;

	float Accel_cal_x_sample = 0;
	float Accel_cal_y_sample = 0;
	float Accel_cal_z_sample = 0;

	int i;

	int nbSampleCalib = 100; // 100 original
	int sampleDurationMs = 20; // 50 original

	for(i = 0;i < nbSampleCalib;i += 1)
	{
		getIMUReadings(Gyro_output, Accel_output);

		Gyro_cal_x_sample += Gyro_output[0];
		Gyro_cal_y_sample += Gyro_output[1];
		Gyro_cal_z_sample += Gyro_output[2];

		Accel_cal_x_sample += Accel_output[0];
		Accel_cal_y_sample += Accel_output[1];
		Accel_cal_z_sample += Accel_output[2];

		delay(sampleDurationMs);
	}

	Gyro_cal_x = Gyro_cal_x_sample / nbSampleCalib;
	Gyro_cal_y = Gyro_cal_y_sample / nbSampleCalib;
	Gyro_cal_z = Gyro_cal_z_sample / nbSampleCalib;

	Accel_cal_x = Accel_cal_x_sample / nbSampleCalib;
	Accel_cal_y = Accel_cal_y_sample / nbSampleCalib;
	Accel_cal_z = (Accel_cal_z_sample / nbSampleCalib) ; //sortie a ACC_LSB_PER_G LSB/g (gravite terrestre) => offset a ACC_LSB_PER_G pour mise a 0


	Logger.println("------------------------------");
	Logger.println("IMU Calibration Output");
	Logger.println("------------------------------");
	Logger.print("Gyro cal x; y; z : ");
	Logger.print(Gyro_cal_x);
	Logger.print("; ");
	Logger.print(Gyro_cal_y);
	Logger.print("; ");
	Logger.print(Gyro_cal_z);
	Logger.println(" ");
	Logger.print("Acc cal x; y; z : ");
	Logger.print(Accel_cal_x);
	Logger.print("; ");
	Logger.print(Accel_cal_y);
	Logger.print("; ");
	Logger.println(Accel_cal_z);
	Logger.println("------------------------------");

	init_roll = (RAD2DEG * vectAccelToRoll(vect3fInstance(Accel_cal_x / ACC_LSB_PER_G, Accel_cal_y / ACC_LSB_PER_G, Accel_cal_z / ACC_LSB_PER_G)));
	init_pitch = (RAD2DEG * vectAccelToPitch(vect3fInstance(Accel_cal_x / ACC_LSB_PER_G, Accel_cal_y / ACC_LSB_PER_G, Accel_cal_z / ACC_LSB_PER_G)));


#else
	Logger.println("------------------------------");
	Logger.println("IMU Calibration Retrieve Saved Data");
	Logger.println("------------------------------");
	/*
------------------------------
IMU Calibration Output
------------------------------
Gyro cal x; y; z : -28.00; 58.00
Acc cal x; y; z : 195.00; 240.00; -201.00
m2 :
Gyro cal x; y; z : -20.00; 49.00; 33.00
Acc cal x; y; z : 34.00; 9.00; -243.00
	 */
	Gyro_cal_x = -20.00;
	Gyro_cal_y = 49.00;
	Gyro_cal_z = 33.0;
	Accel_cal_x = 34.00;
	Accel_cal_y = 9.00;
	Accel_cal_z = -243.00;
#endif
}

//-------------------------------------------------------------
// Update AHRS (Attitude and Heading Reference System)
// using Kalman filter
// To transform acceleromter data into roll, pitch :
// See http://stackoverflow.com/questions/3755059/3d-accelerometer-calculate-the-orientation
// or http://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/
void updateGyroDataC() {
	long currentTimeUs = micros() ;
	dt_IMU = (currentTimeUs - lastUpdateAHRS_Us) / S_TO_US;
	if (lastUpdateAHRS_Us == 0) {
		dt_IMU = 0.01; // Initially, dt equals 10 ms
	}

	lastUpdateAHRS_Us = currentTimeUs;

	// IMU date retrieving
	//-----------------------------------------------
	getIMUReadings(Gyro_output, Accel_output);


	// Accelerometer data and filters
	//-----------------------------------------------
	rel_accX = (Accel_output[0]) / ACC_LSB_PER_G;
	rel_accY = (Accel_output[1]) / ACC_LSB_PER_G;
	rel_accZ = (Accel_output[2]) / ACC_LSB_PER_G;


	// Low pass filter accelerometer
	accelFiltered = accel_filter.apply(vect3fInstance(rel_accX, rel_accY, rel_accZ));

	Accel_pitch = vectAccelToPitch(accelFiltered) * RAD2DEG;
	Accel_roll = vectAccelToRoll(accelFiltered) * RAD2DEG;

	// Gyro data and filters
	//-----------------------------------------------
	raw_gyro_xrate = ((Gyro_output[0] - Gyro_cal_x)/ GYRO_LSB_PER_G) * dt_IMU;
	raw_gyro_yrate = -((Gyro_output[1] - Gyro_cal_y)/ GYRO_LSB_PER_G) * dt_IMU; // Tilt positive when going nose goes high
	raw_gyro_zrate = -((Gyro_output[2] - Gyro_cal_z)/ GYRO_LSB_PER_G) * dt_IMU;

	// Low pass filter on gyro (DESACTIVATED bad results)
//	gyroFiltered = gyro_filter.apply(vect3fInstance(raw_gyro_xrate, raw_gyro_yrate, raw_gyro_zrate));

//	gyroXrate = gyroFiltered.x;
//	gyroYrate = gyroFiltered.y;
//	gyroZrate = gyroFiltered.z;

	double alphaGyroRate = 0.7;
	gyroXrate = (1-alphaGyroRate)*gyroXrate + alphaGyroRate*raw_gyro_xrate;
	gyroYrate = (1-alphaGyroRate)*gyroYrate + alphaGyroRate*raw_gyro_yrate;
	gyroZrate = (1-alphaGyroRate)*gyroZrate + alphaGyroRate*raw_gyro_zrate;

	// Integrate gyro z rate to have approx yaw based on inertial data
	gyroZangle = 0.99 * gyroZangle + gyroZrate * dt_IMU;
	NormRadAngle(gyroZangle);
}




#endif /* SENSOR_GYRO_10DOF_H_ */
