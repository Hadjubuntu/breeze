/*
 * Sensor_Gyro_MPU9150.h
 *
 *  Created on: Feb 9, 2015
 *      Author: adrien
 */

#ifndef SENSOR_GYRO_MPU9150_H_
#define SENSOR_GYRO_MPU9150_H_



// TODO adapt settings for MPU9150
// TODO find LSB data for accelerometer and gyro !

#include "arch/AVR/wire/Wire.h"
#include "Common.h"
#include "arch/AVR/I2C/I2C.h"

#define MPU9150_CHIP_ADDRESS 0x69
#define MEASURE_VIBRATION 0
#define ENABLE_IMU_CALIBRATION 0

#if MEASURE_VIBRATION
#include "../../math/Math.h"
double accNoise = 0.0; // Noise accelerometer measure in G (means output steady equals 1 due to gravity)
#endif

long lastUpdateAHRS_Us = 0;
// TODO simplifier les variables utilisés et de sortie
// Output variables
double gyroXrate = 0.0, gyroYrate = 0.0, gyroZangle = 0.0, kalAngleX = 0.0, kalAngleY = 0.0;

int Gyro_output[3],Accel_output[3];

float dt = 0.01;

float Gyro_cal_x,Gyro_cal_y,Gyro_cal_z,Accel_cal_x,Accel_cal_y,Accel_cal_z;
float raw_accel_roll, raw_accel_pitch;

//valeur initiales axe X (pitch)
//------------------------------
float Gyro_pitch = 0;
float Accel_pitch = 0;
float Predicted_pitch = 0;

//valeur initiales axe Y (roll)
//-----------------------------
float Gyro_roll = 0;
float Accel_roll = 0;
float Predicted_roll = 0;

//definition des bruits
//---------------------
float kalmanQ = 0.1; // 0.06 bruit de processus de covariance (default : 0.1)
float kalmanR = 5; // 15 bruit de mesure (default: 5)

//erreur de covariance
//--------------------
float P00 = 0.1;
float P11 = 0.1;
float P01 = 0.1;

//gains de Kalman
//---------------
float Kk0, Kk1;


//lecture gyroscope - datasheet ITG3200
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


//-------------------------------------------
// Initialize IMU with calibration values
void setupGyro() {

	delay(5);
	Wire.begin();

	//configuration gyroscope et accelerometre
	//----------------------------------------
	writeTo(0x53,0x31,0x09); //accel 11 bits - +/-4g
	writeTo(0x53,0x2D,0x08); //accel en mode mesure
	writeTo(0x68,0x16,0x1A); //gyro +/-2000 deg/s + passe-bas a 100Hz
	writeTo(0x68,0x15,0x09); //gyro echantillonage a 100Hz

	delay(200);

#if ENABLE_IMU_CALIBRATION == 1

	int Gyro_cal_x_sample = 0;
	int Gyro_cal_y_sample = 0;
	int Gyro_cal_z_sample = 0;

	int Accel_cal_x_sample = 0;
	int Accel_cal_y_sample = 0;
	int Accel_cal_z_sample = 0;

	int i;

	int nbSampleCalib = 100; // 100 original
	int sampleDurationMs = 50; // 50 original

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
	Accel_cal_z = (Accel_cal_z_sample / nbSampleCalib) - 256; //sortie a 256 LSB/g (gravite terrestre) => offset a 256 pour mise a 0

	Serial.print("Gyro cal x; y; z : ");
	Serial.print(Gyro_cal_x);
	Serial.print("; ");
	Serial.print(Gyro_cal_y);
	Serial.print("; ");
	Serial.print(Gyro_cal_z);
	Serial.println(" ");
	Serial.print("Acc cal x; y; z : ");
	Serial.print(Accel_cal_x);
	Serial.print("; ");
	Serial.print(Accel_cal_y);
	Serial.print("; ");
	Serial.print(Accel_cal_z);

#else
	Serial.println("IMU's calibration already done");
	/*
IMU only calibration
	Gyro cal x; y; z : 27.00; 11.00; 10.00
	Acc cal x; y; z : 0.00; 0.00; -33.00
Yak 54 calibration
    Gyro cal x; y; z : 34.00; 21.00; -16.00
    Acc cal x; y; z : 22.00; -3.00; -35.00
Pilatus calib :
  Gyro cal x; y; z : 29.00; 21.00; -9.00
  Acc cal x; y; z : -4.00; 10.00; -34.00
	 */
	Gyro_cal_x = 29.0;
	Gyro_cal_y = 21.0;
	Gyro_cal_z = -9.0;
	Accel_cal_x = -4.0;
	Accel_cal_y = 10.0;
	Accel_cal_z = -34.0;
#endif
}


//-------------------------------------------------------------
// Update AHRS (Attitude and Heading Reference System)
// using Kalman filter
void updateGyroData() {
	long currentTimeUs = micros() ;
	dt = (currentTimeUs - lastUpdateAHRS_Us) / S_TO_US;
	if (lastUpdateAHRS_Us == 0) {
		dt = 0.01; // Initially, dt equals 10 ms
	}

	lastUpdateAHRS_Us = currentTimeUs;

	// Retrieve IMU data
	getGyroscopeReadings(Gyro_output);
	getAccelerometerReadings(Accel_output);

	raw_accel_pitch = atan2((Accel_output[1] - Accel_cal_y) / 256,(Accel_output[2] - Accel_cal_z)/256) * 180 / PI;
	Accel_pitch = 0.3 * Accel_pitch + 0.7 * raw_accel_pitch;

	Gyro_pitch = Gyro_pitch + ((Gyro_output[0] - Gyro_cal_x)/ 14.375) * dt;

	//conserver l'echelle +/-180° pour l'axe X du gyroscope
	//-----------------------------------------------------
	if(Gyro_pitch < 180) Gyro_pitch += 360;
	if(Gyro_pitch >= 180) Gyro_pitch -= 360;

	//sortie du filtre de Kalman pour les X (pitch)
	//---------------------------------------------
	Predicted_pitch = Predicted_pitch + ((Gyro_output[0] - Gyro_cal_x)/14.375) * dt;

	raw_accel_roll = atan2((Accel_output[0] - Accel_cal_x) / 256,(Accel_output[2] - Accel_cal_z)/256) * 180 / PI;
	Accel_roll = 0.3 * Accel_roll + 0.7 * raw_accel_roll;

	Gyro_roll = Gyro_roll + ((Gyro_output[1] - Gyro_cal_y)/ 14.375) * dt;

	//conserver l'echelle +/-180° pour l'axe Y du gyroscope
	//-----------------------------------------------------
	if(Gyro_roll < 180) Gyro_roll += 360;
	if(Gyro_roll >= 180) Gyro_roll -= 360;

	//sortie du filtre de Kalman pour les Y (roll)
	//--------------------------------------------
	Predicted_roll = Predicted_roll - ((Gyro_output[1] - Gyro_cal_y)/14.375) * dt;

	P00 += dt * (2 * P01 + dt * P11);
	P01 += dt * P11;
	P00 += dt * kalmanQ;
	P11 += dt * kalmanQ;

	//mise a jour des gains de Kalman
	//-------------------------------
	Kk0 = P00 / (P00 + kalmanR);
	Kk1 = P01 / (P01 + kalmanR);

	//mise a jour de la mesure
	//------------------------
	Predicted_pitch += (Accel_pitch - Predicted_pitch) * Kk0;
	Predicted_roll += (Accel_roll - Predicted_roll) * Kk0;

	//mise a jour de l'erreur de covariance
	//-------------------------------------
	P00 *= (1 - Kk0);
	P01 *= (1 - Kk1);
	P11 -= Kk1 * P01;


	//-----------------------------------------------
	// Output update
	gyroXrate = ((Gyro_output[0] - Gyro_cal_x)/14.375) * dt;
	gyroYrate = ((Gyro_output[1] - Gyro_cal_y)/14.375) * dt;

	kalAngleX = Predicted_pitch; // ok this is shitty : pitch goes to roll, because variables definition problem TODO
	kalAngleY = Predicted_roll;
	gyroZangle += Gyro_output[2] / 14.375 * dt;



#if MEASURE_VIBRATION
	//-----------------------------------------------
	// If needed, measure vibration
	accNoise = sqrt(pow2(Accel_output[0] - Accel_cal_x) + pow2(Accel_output[1] - Accel_cal_y) + pow2(Accel_output[2] - Accel_cal_z)) / 256;
#endif
}



#endif /* SENSOR_GYRO_MPU9150_H_ */
