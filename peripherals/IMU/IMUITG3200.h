/*
 * IMUITG3200.h
 *
 *  Created on: May 20, 2015
 *      Author: adrien
 */

#ifndef PERIPHERALS_IMU_IMUITG3200_H_
#define PERIPHERALS_IMU_IMUITG3200_H_

#include "IMUClass.h"

class IMU_ITG3200: public IMU_Class {
protected:
	byte chip_adxl345_address;
public:
	IMU_ITG3200();
	void setupGyro();
	void getIMUReadings(int Gyro_out[], int Accel_out[]);
	void updateGyroData();

	void getGyroscopeReadings(int Gyro_out[]);
	void getAccelerometerReadings(int Accel_out[]);
};


IMU_ITG3200::IMU_ITG3200()
{
	initParameters();

	// Init specific parameters
	accLsbPerG = 256.0f; // 16384.0f/9.81f // 256.0f // 16384.0f
	gyroLsbPerDegS = 14.375f; // FS_SEL 0 131.0
	chip_address = 0x68; // low : 0x68, high : 0X69
	who_i_am_register = 0x0;

	// Specific accelerometer combined with ITG3200 IMU
	chip_adxl345_address = 0x53; // low : 0x53; high : 0x1D
}

void IMU_ITG3200::setupGyro()
{
	// Prepare filters
	//---------------------------------------------
	accel_filter.set_cutoff_frequency(800, 20);
	gyro_filter.set_cutoff_frequency(800, 127);

	// Initialize IMU
	//-------------------------------------------------------
	delay(5);
	Wire.begin();


	Logger.println("start configuration IMU");

	// Configure accelerometer and gyroscope
	//---------------------------------------------

	writeTo(chip_adxl345_address,0x2D,0x00); // power ctlr
	delay(5);
	writeTo(chip_adxl345_address,0x2D,0xff); // power ctlr
	delay(5);
	writeTo(chip_adxl345_address,0x2D,0x08); //accel en mode mesure
	delay(5);
	writeTo(chip_adxl345_address,0x31,0b00); //accel 11 bits - +/-2g
	delay(5);
	writeTo(chip_adxl345_address,0x2c,0x0d); // 800 hz output
	delay(5);


	writeTo(chip_address,0x16,0x1A); //gyro +/-2000 deg/s + passe-bas a 100Hz
	delay(5);
	writeTo(chip_address,0x15,0x09); //gyro echantillonage a 100Hz

	delay(200);

	if (enable_imu_calibration)
	{
		calibrate();
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


//lecture gyroscope - datasheet ITG3200
//-------------------------------------
void IMU_ITG3200::getGyroscopeReadings(int Gyro_out[])
{
	byte buffer[6];
	readFrom(chip_address,0x1D,6,buffer);

	Gyro_out[0]=(((int)buffer[0]) << 8 ) | buffer[1];
	Gyro_out[1]=(((int)buffer[2]) << 8 ) | buffer[3];
	Gyro_out[2]=(((int)buffer[4]) << 8 ) | buffer[5];
}

//lecture accelerometre - datasheet ADXL345
//-----------------------------------------
void IMU_ITG3200::getAccelerometerReadings(int Accel_out[])
{
	byte buffer[6];
	readFrom(chip_adxl345_address,0x32,6,buffer);

	Accel_out[0]=(((int)buffer[1]) << 8 ) | buffer[0];
	Accel_out[1]=(((int)buffer[3]) << 8 ) | buffer[2];
	Accel_out[2]=(((int)buffer[5]) << 8 ) | buffer[4];
}



void IMU_ITG3200::getIMUReadings(int Gyro_out[], int Accel_out[])
{
	getGyroscopeReadings(Gyro_out);
	getAccelerometerReadings(Accel_out);
}

void IMU_ITG3200::updateGyroData()
{
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

#endif /* PERIPHERALS_IMU_IMUITG3200_H_ */
