/*
 * Sensor_Gyro_MPU9150.h
 *
 *  Created on: Feb 9, 2015
 *      Author: adrien
 */

#ifndef SENSOR_GYRO_MPU9150_H_
#define SENSOR_GYRO_MPU9150_H_

// TODO verify ACC_LSB_PER_G accelerometer specs value


#include "arch/AVR/MCU/MCU.h"
#include "arch/AVR/wire/Wire.h"
#include "Common.h"
#include "arch/AVR/I2C/I2C.h"

#define MPU9150_CHIP_ADDRESS 0x68
#define AK8975_MAG_ADDRESS 0x0C
#define MEASURE_VIBRATION 0
#define ENABLE_IMU_CALIBRATION 0
#define ENABLE_COMPASS 1

//MPU9150 Compass
#define MPU9150_CMPS_XOUT_L        0x4A   // R
#define MPU9150_CMPS_XOUT_H        0x4B   // R
#define MPU9150_CMPS_YOUT_L        0x4C   // R
#define MPU9150_CMPS_YOUT_H        0x4D   // R
#define MPU9150_CMPS_ZOUT_L        0x4E   // R
#define MPU9150_CMPS_ZOUT_H        0x4F   // R

// Parameter of the IMU
// Thoses values change when config sent to the IMU is changed
#define ACC_LSB_PER_G 8192.0
#define GYRO_LSB_PER_G 131.0

// Enable vibration measurement
#if MEASURE_VIBRATION
#include "../../math/Math.h"
double accNoise = 0.0; // Noise accelerometer measure in G (means output steady equals 1 due to gravity)
#endif

// Variables
long lastUpdateAHRS_Us = 0;
// TODO simplifier les variables utilisés et de sortie
// Output variables
double gyroXrate = 0.0, gyroYrate = 0.0, gyroZangle = 0.0, kalAngleX = 0.0, kalAngleY = 0.0;

int Gyro_output[3], Accel_output[3], Mag_output[3];

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
float kalmanQ = 0.06; // 0.06 bruit de processus de covariance (default : 0.1)
float kalmanR = 0.5; // 15 bruit de mesure (default: 5)

//erreur de covariance
//--------------------
float P00 = 0.1;
float P11 = 0.1;
float P01 = 0.1;

//gains de Kalman
//---------------
float Kk0, Kk1;


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

void updateCompassData() {
#if ENABLE_COMPASS == 1
	byte buffer[6];

	readFrom(MPU9150_CHIP_ADDRESS, MPU9150_CMPS_XOUT_L, 6, buffer);
	Mag_output[0]=(((int)buffer[0]) << 8 ) | buffer[1];
	Mag_output[1]=(((int)buffer[2]) << 8 ) | buffer[3];
	Mag_output[2]=(((int)buffer[4]) << 8 ) | buffer[5];

	writeTo(AK8975_MAG_ADDRESS, 0x0a, 0x01);
#endif
}

double getCompassHeading() {
	double heading = fast_atan2((double)Mag_output[1], (double)Mag_output[0]) * RAD2DEG + 180;
	while (heading < 0) heading += 360;
	while (heading > 360) heading -= 360;
	return heading;
}


void MPU9150_setupCompass();

//-------------------------------------------
// Initialize IMU with calibration values
void setupGyro() {

	delay(5);
	Wire.begin();

	uint8_t i2cData[14]; // Buffer for I2C data

	Logger.println("start configuration IMU");

	//configuration gyroscope et accelerometre
	//----------------------------------------
	i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
	i2cData[1] = 0x00; // Disable FSYNC and set 1kHz Acc filtering, 1kHz Gyro filtering, 8 KHz sampling
	i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
	i2cData[3] = 0x01; // Set Accelerometer Full Scale Range to ±4g
	while (i2cWriteArray(MPU9150_CHIP_ADDRESS, 0x19, i2cData, 4, false)); // Write to all four registers at once
	while (i2cWrite(MPU9150_CHIP_ADDRESS, 0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

	Logger.println("IMU configured");

#if ENABLE_COMPASS == 1
	MPU9150_setupCompass();
#endif

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
	Accel_cal_z = (Accel_cal_z_sample / nbSampleCalib) + ACC_LSB_PER_G; //sortie a ACC_LSB_PER_G LSB/g (gravite terrestre) => offset a ACC_LSB_PER_G pour mise a 0


	Logger.println("------------------------------");
	Logger.println("IMU Calibration Output");
	Logger.println("------------------------------");
	Logger.print("Gyro cal x; y; z : ");
	Logger.print(Gyro_cal_x);
	Logger.print("; ");
	Logger.print(Gyro_cal_y);
	Logger.println(" ");
	Logger.print("Acc cal x; y; z : ");
	Logger.print(Accel_cal_x);
	Logger.print("; ");
	Logger.print(Accel_cal_y);
	Logger.print("; ");
	Logger.println(Accel_cal_z);
	Logger.println("------------------------------");

#else
	Logger.println("------------------------------");
	Logger.println("IMU Calibration Retrieve Saved Data");
	Logger.println("------------------------------");
	/*
------------------------------
IMU Calibration Output
------------------------------
Gyro cal x; y; z : -29.00; 59.00
Acc cal x; y; z : 55.00; 86.00; 7996.00
	 */
	Gyro_cal_x = -29.0;
	Gyro_cal_y = 59.0;
	Gyro_cal_z = 0.0;
	Accel_cal_x = 55.00;
	Accel_cal_y = 86.00;
	Accel_cal_z = 7996.00;
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
	getIMUReadings(Gyro_output, Accel_output);

	raw_accel_pitch = atan2((Accel_output[1] - Accel_cal_y) / ACC_LSB_PER_G,(Accel_output[2] - Accel_cal_z)/ACC_LSB_PER_G) * 180 / PI;
	Accel_pitch = 0.3 * Accel_pitch + 0.7 * raw_accel_pitch;

	Gyro_pitch = Gyro_pitch + ((Gyro_output[0] - Gyro_cal_x)/ GYRO_LSB_PER_G) * dt;

	//conserver l'echelle +/-180° pour l'axe X du gyroscope
	//-----------------------------------------------------
	if(Gyro_pitch < 180) Gyro_pitch += 360;
	if(Gyro_pitch >= 180) Gyro_pitch -= 360;

	//sortie du filtre de Kalman pour les X (pitch)
	//---------------------------------------------
	Predicted_pitch = Predicted_pitch + ((Gyro_output[0] - Gyro_cal_x)/GYRO_LSB_PER_G) * dt;

	raw_accel_roll = atan2((Accel_output[0] - Accel_cal_x) / ACC_LSB_PER_G,(Accel_output[2] - Accel_cal_z)/ACC_LSB_PER_G) * 180 / PI;
	Accel_roll = 0.3 * Accel_roll + 0.7 * raw_accel_roll;

	Gyro_roll = Gyro_roll + ((Gyro_output[1] - Gyro_cal_y)/ GYRO_LSB_PER_G) * dt;

	//conserver l'echelle +/-180° pour l'axe Y du gyroscope
	//-----------------------------------------------------
	if(Gyro_roll < 180) Gyro_roll += 360;
	if(Gyro_roll >= 180) Gyro_roll -= 360;

	//sortie du filtre de Kalman pour les Y (roll)
	//--------------------------------------------
	Predicted_roll = Predicted_roll - ((Gyro_output[1] - Gyro_cal_y)/GYRO_LSB_PER_G) * dt;

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
	gyroXrate = ((Gyro_output[0] - Gyro_cal_x)/GYRO_LSB_PER_G) * dt;
	gyroYrate = ((Gyro_output[1] - Gyro_cal_y)/GYRO_LSB_PER_G) * dt;

	kalAngleX = Predicted_pitch; // ok this is shitty : pitch goes to roll, because variables definition problem TODO
	kalAngleY = Predicted_roll;
	gyroZangle += Gyro_output[2] / GYRO_LSB_PER_G * dt;



#if MEASURE_VIBRATION
	//-----------------------------------------------
	// If needed, measure vibration
	accNoise = sqrt(pow2(Accel_output[0] - Accel_cal_x) + pow2(Accel_output[1] - Accel_cal_y) + pow2(Accel_output[2] - Accel_cal_z)) / ACC_LSB_PER_G;
#endif
}

void MPU9150_printMagField() {
#if ENABLE_COMPASS == 1
/**	Logger.println("Mag field");
	Logger.print("X: ");
	Logger.print(Mag_output[0]);
	Logger.print("Y: ");
	Logger.print(Mag_output[1]);
	Logger.print("Z: ");
	Logger.println(Mag_output[2]);*/
	Logger.print("Heading (deg): ");
	Logger.println(getCompassHeading());
#endif
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
}



#endif /* SENSOR_GYRO_MPU9150_H_ */
