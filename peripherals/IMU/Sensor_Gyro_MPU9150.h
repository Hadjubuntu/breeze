/*
 * Sensor_Gyro_MPU9150.h
 *
 *  Created on: Feb 9, 2015
 *      Author: adrien
 */

#ifndef SENSOR_GYRO_MPU9150_H_
#define SENSOR_GYRO_MPU9150_H_

#include "arch/AVR/MCU/MCU.h"
#include "arch/AVR/wire/Wire.h"
#include "Common.h"
#include "arch/AVR/I2C/I2C.h"
#include "math/LowPassFilter.h"

#define MPU9150_CHIP_ADDRESS 0x68
#define AK8975_MAG_ADDRESS 0x0C
#define MEASURE_VIBRATION 1
#define ENABLE_IMU_CALIBRATION 1

#define ENABLE_COMPASS 0
void MPU9150_setupCompass();

// Low pass filters
//-------------------------------------------
LowPassFilter2pVector3f accel_filter;
LowPassFilter2pVector3f gyro_filter;
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



// read IMU data - datasheet ITG3200
//-------------------------------------
void getIMUReadings(int Gyro_out[], int Accel_out[])
{
	byte buffer[14];
	readFrom(MPU9150_CHIP_ADDRESS, 0x3B, 14,buffer);

	Gyro_out[0]=(((int)buffer[8]) << 8 ) | buffer[9];
	Gyro_out[1]=(((int)buffer[10]) << 8 ) | buffer[11];
	Gyro_out[2]=(((int)buffer[12]) << 8 ) | buffer[13];


	Accel_out[0]=(((int)buffer[0]) << 8 ) | buffer[1];
	Accel_out[1]=(((int)buffer[2]) << 8 ) | buffer[3];
	Accel_out[2]=(((int)buffer[4]) << 8 ) | buffer[5];

	// tempRaw = (i2cData[6] << 8) | i2cData[7];
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
void setupGyro() {

	// Prepare filters
	//-------------------------------------------------------
	accel_filter.set_cutoff_frequency(800, 10);
	gyro_filter.set_cutoff_frequency(800, 127);


	// Initialize IMU
	//-------------------------------------------------------
	delay(5);
	Wire.begin();

	uint8_t i2cData[14]; // Buffer for I2C data

	Logger.println("start configuration IMU");
	// Wake up and reset sensor
	while (i2cWrite(MPU9150_CHIP_ADDRESS, 0x6B, 0x80, true)); // PLL with X axis gyroscope reference and disable sleep mode
	delay(100);
	while (i2cWrite(MPU9150_CHIP_ADDRESS, 0x6B, 0x00, true)); // PLL with X axis gyroscope reference and disable sleep mode


	// Gyro and accelerometer configuration
	//----------------------------------------
	//	i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
	//	i2cData[1] = 0x00; // Disable FSYNC and set 1kHz Acc filtering, 1kHz Gyro filtering, 8 KHz sampling
	//	i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
	//	i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
	//	while (i2cWriteArray(MPU9150_CHIP_ADDRESS, 0x19, i2cData, 4, true)); // Write to all four registers at once

	// 2000° range gyro
	uint8_t data = 3 << 3;
	i2cWrite(MPU9150_CHIP_ADDRESS, 0x1B, data, true);
	delay(10);

	// Acc +/- 2g
	data = 0 << 3;
	i2cWrite(MPU9150_CHIP_ADDRESS, 0x1C, data, true);
	delay(10);

	// 800 Hz sample rate
	data = 1000 / 800 - 1;
	i2cWrite(MPU9150_CHIP_ADDRESS, 0x19, data, true);
	delay(10);

	// 256Hz filtering
	i2cWrite(MPU9150_CHIP_ADDRESS, 0x1A, 0x00, true);
	delay(100);

	Logger.println("IMU configured");

	// Read and print config
	byte buffer[2];
	readFrom(MPU9150_CHIP_ADDRESS, 0x1B, 1, buffer);

	Logger.print("IMU gyro_scale = ");
	Logger.println(buffer[0]);

#if ENABLE_COMPASS == 1
	MPU9150_setupCompass();
#endif

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
void updateGyroData() {
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
	Vector3f accelFiltered = accel_filter.apply(vect3fInstance(rel_accX, rel_accY, rel_accZ));

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





//MPU9150 Compass
//------------------------------------------------------
#if ENABLE_COMPASS == 1
#define MPU9150_CMPS_XOUT_L        0x4A   // R
#define MPU9150_CMPS_XOUT_H        0x4B   // R
#define MPU9150_CMPS_YOUT_L        0x4C   // R
#define MPU9150_CMPS_YOUT_H        0x4D   // R
#define MPU9150_CMPS_ZOUT_L        0x4E   // R
#define MPU9150_CMPS_ZOUT_H        0x4F   // R


void updateCompassData() {
#if ENABLE_COMPASS == 1
	byte buffer[6];

	readFrom(MPU9150_CHIP_ADDRESS, MPU9150_CMPS_XOUT_L, 6, buffer);
	Mag_output[0] = (((int)buffer[0]) << 8 ) | buffer[1];
	Mag_output[1] = (((int)buffer[2]) << 8 ) | buffer[3];
	Mag_output[2] = (((int)buffer[4]) << 8 ) | buffer[5];

	writeTo(AK8975_MAG_ADDRESS, 0x0a, 0x01);
#endif
}

double compassHeading = 0.0;

double getCompassHeading(Attitude *curAtt) {

	double rawFieldX = (double) Mag_output[0] * 0.3;
	double rawFieldY = (double) Mag_output[1] * 0.3;
	double rawFieldZ = (double) Mag_output[2] * 0.3;

	Vector3f mag_bf;
	mag_bf.x = rawFieldX;
	mag_bf.y = rawFieldY;
	mag_bf.z = rawFieldZ;


	Vector3f mag_ef = rot_bf_ef(mag_bf, curAtt);

	compassHeading = fast_atan2(-mag_ef.y, mag_ef.x) * RAD2DEG;
	while (compassHeading < 0.0) compassHeading += 360;
	while (compassHeading > 360) compassHeading -= 360;


	return compassHeading;
}



void MPU9150_setupCompass(){

	writeTo(AK8975_MAG_ADDRESS, 0x0A, 0x00); //PowerDownMode
	writeTo(AK8975_MAG_ADDRESS, 0x0A, 0x0F); //SelfTest
	writeTo(AK8975_MAG_ADDRESS, 0x0A, 0x00); //PowerDownMode

	writeTo(MPU9150_CHIP_ADDRESS, 0x24, 0x40); //Wait for Data at Slave0
	writeTo(MPU9150_CHIP_ADDRESS, 0x25, 0x8C); //Set i2c address at slave0 at 0x0C
	writeTo(MPU9150_CHIP_ADDRESS, 0x26, 0x02); //Set where reading at slave 0 starts
	writeTo(MPU9150_CHIP_ADDRESS, 0x27, 0x88); //set offset at start reading and enable
	writeTo(MPU9150_CHIP_ADDRESS, 0x28, 0x0C); //set i2c address at slv1 at 0x0C
	writeTo(MPU9150_CHIP_ADDRESS, 0x29, 0x0A); //Set where reading at slave 1 starts
	writeTo(MPU9150_CHIP_ADDRESS, 0x2A, 0x81); //Enable at set length to 1
	writeTo(MPU9150_CHIP_ADDRESS, 0x64, 0x01); //overvride register
	writeTo(MPU9150_CHIP_ADDRESS, 0x67, 0x03); //set delay rate
	writeTo(MPU9150_CHIP_ADDRESS, 0x01, 0x80);

	writeTo(MPU9150_CHIP_ADDRESS, 0x34, 0x04); //set i2c slv4 delay
	writeTo(MPU9150_CHIP_ADDRESS, 0x64, 0x00); //override register
	writeTo(MPU9150_CHIP_ADDRESS, 0x6A, 0x00); //clear usr setting
	writeTo(MPU9150_CHIP_ADDRESS, 0x64, 0x01); //override register
	writeTo(MPU9150_CHIP_ADDRESS, 0x6A, 0x20); //enable master i2c mode
	writeTo(MPU9150_CHIP_ADDRESS, 0x34, 0x13); //disable slv4

	// Throw some data
	int throwDataCpt = 0;
	while (throwDataCpt < 5) {
		updateCompassData();
		throwDataCpt ++;
		delay(20);
	}
}
#endif



#endif /* SENSOR_GYRO_MPU9150_H_ */
