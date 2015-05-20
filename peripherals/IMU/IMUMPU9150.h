/*
 * IMUMPU9150.h
 *
 *  Created on: May 20, 2015
 *      Author: adrien
 */

#ifndef PERIPHERALS_IMU_IMUMPU9150_H_
#define PERIPHERALS_IMU_IMUMPU9150_H_

#include "IMUClass.h"

class IMU_MPU9150: public IMU_Class {
public:
	IMU_MPU9150();
	void setupGyro();
	void getIMUReadings(int Gyro_out[], int Accel_out[]);
	void updateGyroData();
};

IMU_MPU9150::IMU_MPU9150()
{
	initParameters();

	// Init specific parameters
	accLsbPerG = 16384.0f;
	gyroLsbPerDegS = 16.4f;
	chip_address = 0x68;
	who_i_am_register = 0x0;
}

void IMU_MPU9150::setupGyro() {
	// Prepare filters
	//-------------------------------------------------------
	accel_filter.set_cutoff_frequency(800, 10);
	gyro_filter.set_cutoff_frequency(800, 127);


	// Initialize IMU
	//-------------------------------------------------------
	delay(5);
	Wire.begin();


	Logger.println("start configuration IMU");
	// Wake up and reset sensor
	while (i2cWrite(chip_address, 0x6B, 0x80, true)); // PLL with X axis gyroscope reference and disable sleep mode
	delay(100);
	while (i2cWrite(chip_address, 0x6B, 0x00, true)); // PLL with X axis gyroscope reference and disable sleep mode



	// 2000° range gyro
	uint8_t data = 3 << 3;
	i2cWrite(chip_address, 0x1B, data, true);
	delay(10);

	// Acc +/- 2g
	data = 0 << 3;
	i2cWrite(chip_address, 0x1C, data, true);
	delay(10);

	// 800 Hz sample rate
	data = 1000 / 800 - 1;
	i2cWrite(chip_address, 0x19, data, true);
	delay(10);

	// 256Hz filtering
	i2cWrite(chip_address, 0x1A, 0x00, true);
	delay(100);

	Logger.println("IMU configured");

	// Read and print config
	byte buffer[2];
	readFrom(chip_address, 0x1B, 1, buffer);

	Logger.print("IMU gyro_scale = ");
	Logger.println(buffer[0]);



	delay(200);

	if (enable_imu_calibration) {

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
		Accel_cal_z = (Accel_cal_z_sample / nbSampleCalib) ; //sortie a accLsbPerG LSB/g (gravite terrestre) => offset a accLsbPerG pour mise a 0


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

		init_roll = (RAD2DEG * vectAccelToRoll(vect3fInstance(Accel_cal_x / accLsbPerG, Accel_cal_y / accLsbPerG, Accel_cal_z / accLsbPerG)));
		init_pitch = (RAD2DEG * vectAccelToPitch(vect3fInstance(Accel_cal_x / accLsbPerG, Accel_cal_y / accLsbPerG, Accel_cal_z / accLsbPerG)));


	}
	else
	{
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
	}
}

void IMU_MPU9150::getIMUReadings(int Gyro_out[], int Accel_out[]) {
	byte buffer[14];
	readFrom(chip_address, 0x3B, 14,buffer);

	Gyro_out[0]=(((int)buffer[8]) << 8 ) | buffer[9];
	Gyro_out[1]=(((int)buffer[10]) << 8 ) | buffer[11];
	Gyro_out[2]=(((int)buffer[12]) << 8 ) | buffer[13];


	Accel_out[0]=(((int)buffer[0]) << 8 ) | buffer[1];
	Accel_out[1]=(((int)buffer[2]) << 8 ) | buffer[3];
	Accel_out[2]=(((int)buffer[4]) << 8 ) | buffer[5];

	// tempRaw = (i2cData[6] << 8) | i2cData[7];
}

//-------------------------------------------------------------
// Update AHRS (Attitude and Heading Reference System)
// using Kalman filter
// To transform acceleromter data into roll, pitch :
// See http://stackoverflow.com/questions/3755059/3d-accelerometer-calculate-the-orientation
// or http://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/
void IMU_MPU9150::updateGyroData() {
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
	rel_accX = (Accel_output[0]) / accLsbPerG;
	rel_accY = (Accel_output[1]) / accLsbPerG;
	rel_accZ = (Accel_output[2]) / accLsbPerG;


	// Low pass filter accelerometer
	accelFiltered = accel_filter.apply(vect3fInstance(rel_accX, rel_accY, rel_accZ));

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

	double alphaGyroRate = 0.7;
	gyroXrate = (1-alphaGyroRate)*gyroXrate + alphaGyroRate*raw_gyro_xrate;
	gyroYrate = (1-alphaGyroRate)*gyroYrate + alphaGyroRate*raw_gyro_yrate;
	gyroZrate = (1-alphaGyroRate)*gyroZrate + alphaGyroRate*raw_gyro_zrate;

	// Integrate gyro z rate to have approx yaw based on inertial data
	gyroZangle = 0.99 * gyroZangle + gyroZrate * dt_IMU;
	NormRadAngle(gyroZangle);
}



#endif /* PERIPHERALS_IMU_IMUMPU9150_H_ */
