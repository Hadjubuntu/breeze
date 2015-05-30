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

class IMU_Class {
protected:
	byte chip_address;
	byte who_i_am_register;
	bool measure_vibration;
	bool enable_imu_calibration;

	double accLsbPerG;
	double gyroLsbPerDegS;
public:
	IMU_Class();

	bool measureVibration() {return measure_vibration;}

	void initParameters();

	virtual void getIMUReadings(int Gyro_out[], int Accel_out[]);

	virtual void setupGyro() = 0;

	virtual void updateGyroData() = 0;

	void calibrate();

};


IMU_Class::IMU_Class()
{
}

void IMU_Class::initParameters() {
	measure_vibration = false;
	enable_imu_calibration = true;
	accLsbPerG = 1.0;
	gyroLsbPerDegS = 1.0;
}


void IMU_Class::calibrate()
{
	float Gyro_cal_x_sample = 0;
	float Gyro_cal_y_sample = 0;
	float Gyro_cal_z_sample = 0;

	float Accel_cal_x_sample = 0;
	float Accel_cal_y_sample = 0;
	float Accel_cal_z_sample = 0;


	int iterAverage = 1;
	float gyroAvgX[iterAverage];
	float gyroAvgY[iterAverage];
	float gyroAvgZ[iterAverage];

	int i;

	int nbSampleCalib = 100; // 100 original
	int sampleDurationMs = 5; // 50 original


	for (i = 0; i < nbSampleCalib; i ++)
	{
		getIMUReadings(Gyro_output, Accel_output);

		// Computes sum
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
	Accel_cal_z = accLsbPerG - (Accel_cal_z_sample / nbSampleCalib) ; //sortie a accLsbPerG LSB/g (gravite terrestre) => offset a accLsbPerG pour mise a 0


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
	delay(100);
	Logger.print("Acc cal x; y; z : ");
	Logger.print(Accel_cal_x);
	Logger.print("; ");
	Logger.print(Accel_cal_y);
	Logger.print("; ");
	Logger.println(Accel_cal_z);
	Logger.println("------------------------------");

	init_roll = (RAD2DEG * vectAccelToRoll(vect3fInstance(Accel_cal_x / accLsbPerG, Accel_cal_y / accLsbPerG, Accel_cal_z / accLsbPerG)));
	init_pitch = (RAD2DEG * vectAccelToPitch(vect3fInstance(Accel_cal_x / accLsbPerG, Accel_cal_y / accLsbPerG, Accel_cal_z / accLsbPerG)));

}



#endif /* PERIPHERALS_IMU_IMUCLASS_H_ */
