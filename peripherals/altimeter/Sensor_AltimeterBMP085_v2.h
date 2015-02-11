/*
 * Sensor_AltimeterBMP085_v2.h
 *
 *  Created on: Feb 10, 2015
 *      Author: adrien
 */

#ifndef SENSOR_ALTIMETERBMP085_V2_H_
#define SENSOR_ALTIMETERBMP085_V2_H_

#include "arch/AVR/wire/Wire.h"

#define BMP085_ADDRESS 0x77  //(0xEE >> 1)
#define BMP085_EOC 30        // End of conversion pin PC7 on APM1


// No EOC connection from Baro
// Use times instead.
// Temp conversion time is 4.5ms
// Pressure conversion time is 25.5ms (for OVERSAMPLING=3)


// oversampling 3 gives 26ms conversion time. We then average
#define OVERSAMPLING 3

class AP_Baro_BMP085 {
public:
	AP_Baro_BMP085();
	bool            init();
	uint8_t         read();
	void 			accumulate(void);
	float           get_pressure();
	float			get_ground_pressure();
	long 			get_alt_cm();
	float           get_temperature() const;
	void writemem(uint8_t _addr, uint8_t _val);
	void readmem(uint8_t _addr, uint8_t _nbytes, uint8_t __buff[]);

	float get_altitude_difference(float, float) const;

private:
	uint8_t _dev_address;

	int32_t         RawPress;
	int32_t         RawTemp;
	float		    _temp_sum;
	float			_press_sum;
	uint8_t			_count;
	float           Temp;
	float           Press;
	float 			GroundPressure;
	float 			GroundTemp;
	// Flymaple has no EOC pin, so use times instead
	uint32_t        _last_press_read_command_time;
	uint32_t        _last_temp_read_command_time;


	uint32_t                            _last_update; // in us
	uint8_t                             _pressure_samples;
	// State machine
	uint8_t                         BMP085_State;
	// Internal calibration registers
	int16_t                         ac1, ac2, ac3, b1, b2, mb, mc, md;
	uint16_t                        ac4, ac5, ac6;


	uint32_t                        _retry_time;

	void                            Command_ReadPress();
	void                            Command_ReadTemp();
	void                            ReadPress();
	void                            ReadTemp();
	void                            Calculate();
	void							Calibrate();
	bool BMP_DATA_READY();
};

long AP_Baro_BMP085::get_alt_cm() {
	return 0;
}

bool AP_Baro_BMP085::BMP_DATA_READY() {
	long t_us = micros();
	int dtMin = 0;
	if (BMP085_State == 0) {
		dtMin = 5;
	}
	else {
		dtMin = 26;
	}
	if (t_us-_last_temp_read_command_time > dtMin) {
		return true;
	}
	else {
		return false;
	}
}

AP_Baro_BMP085::AP_Baro_BMP085() {
	_dev_address = BMP085_ADDRESS;
	_pressure_samples = 1;
	_retry_time = 0;
	_last_temp_read_command_time = 0;
	_last_press_read_command_time = 0;
}

void AP_Baro_BMP085::writemem(uint8_t _addr, uint8_t _val) {
	Wire.beginTransmission(_dev_address);   // start transmission to device
	Wire.write(_addr); // send register address
	Wire.write(_val); // send value to write
	Wire.endTransmission(); // end transmission
}

void AP_Baro_BMP085::readmem(uint8_t _addr, uint8_t _nbytes, uint8_t __buff[]) {
	Wire.beginTransmission(_dev_address); // start transmission to device
	Wire.write(_addr); // sends register address to read from
	Wire.endTransmission(); // end transmission

	Wire.beginTransmission(_dev_address); // start transmission to device
	Wire.requestFrom(_dev_address, _nbytes);// send data n-bytes read
	uint8_t i = 0;
	while (Wire.available()) {
		__buff[i] = Wire.read(); // receive DATA
		i++;
	}
	Wire.endTransmission(); // end transmission
}


// Public Methods //////////////////////////////////////////////////////////////
bool AP_Baro_BMP085::init()
{
	uint8_t buff[22];


	// We read the calibration data registers
	readmem(0xAA, 22, buff) ;


	ac1 = ((int16_t)buff[0] << 8) | buff[1];
	ac2 = ((int16_t)buff[2] << 8) | buff[3];
	ac3 = ((int16_t)buff[4] << 8) | buff[5];
	ac4 = ((int16_t)buff[6] << 8) | buff[7];
	ac5 = ((int16_t)buff[8] << 8) | buff[9];
	ac6 = ((int16_t)buff[10] << 8) | buff[11];
	b1 = ((int16_t)buff[12] << 8) | buff[13];
	b2 = ((int16_t)buff[14] << 8) | buff[15];
	mb = ((int16_t)buff[16] << 8) | buff[17];
	mc = ((int16_t)buff[18] << 8) | buff[19];
	md = ((int16_t)buff[20] << 8) | buff[21];

	_last_press_read_command_time = 0;
	_last_temp_read_command_time = 0;

	//Send a command to read Temp
	Command_ReadTemp();

	BMP085_State = 0;

	// init raw temo
	RawTemp = 0;

	GroundPressure = 0;
	GroundTemp = 0;
	Calibrate();

	return true;
}

void AP_Baro_BMP085::Calibrate() {
	int step = 0;

	while (step < 100) {
		accumulate();
		read();

		delay(20);
		step ++;
	}

	step = 0;
	while (step < 200) {
		accumulate();
		read();
		if (GroundPressure == 0) {
			GroundPressure = get_pressure();
		}
		else {
			GroundPressure = 0.8*GroundPressure + 0.2*get_pressure();
		}

		if (GroundTemp == 0) {
			GroundTemp = get_temperature();
		}
		else {
			GroundTemp = 0.8*GroundTemp + 0.2*get_temperature();
		}

		delay(20);
		step ++;
	}
}

// Read the sensor. This is a state machine
// acumulate a new sensor reading
void AP_Baro_BMP085::accumulate(void)
{
	if (!BMP_DATA_READY()) {
		return;
	}


	if (BMP085_State == 0) {
		ReadTemp();
	} else {
		ReadPress();
		Calculate();
	}
	BMP085_State++;
	if (BMP085_State == 5) {
		BMP085_State = 0;
		Command_ReadTemp();
	} else {
		Command_ReadPress();
	}

}

// return altitude difference in meters between current pressure and a
// given base_pressure in Pascal
float AP_Baro_BMP085::get_altitude_difference(float base_pressure, float pressure) const
{
	float ret;

	// on faster CPUs use a more exact calculation
	float scaling = pressure / base_pressure;
	float temp    = GroundTemp + 273.15f; // TODO _ground_temperature

	// This is an exact calculation that is within +-2.5m of the standard atmosphere tables
	// in the troposphere (up to 11,000 m amsl).
	ret = 153.8462f * temp * (1.0f - expf(0.190259f * logf(scaling)));

	return ret;
}


// Read the sensor using accumulated data
uint8_t AP_Baro_BMP085::read()
{
	if (_count == 0 && BMP_DATA_READY()) {
		accumulate();
	}
	if (_count == 0) {
		return 0;
	}
	_last_update = micros();

	Temp = 0.1f * _temp_sum / _count;
	Press = _press_sum / _count;

	_pressure_samples = _count;
	_count = 0;
	_temp_sum = 0;
	_press_sum = 0;

	return 1;
}

float AP_Baro_BMP085::get_pressure() {
	return Press;
}

float AP_Baro_BMP085::get_ground_pressure() {
	return GroundPressure;
}

float AP_Baro_BMP085::get_temperature() const {
	return Temp;
}

// Private functions: /////////////////////////////////////////////////////////

// Send command to Read Pressure
void AP_Baro_BMP085::Command_ReadPress()
{
	// Mode 0x34+(OVERSAMPLING << 6) is osrs=3 when OVERSAMPLING=3 => 25.5ms conversion time
	writemem(0xF4, 0x34+(OVERSAMPLING << 6));
	_last_press_read_command_time = micros();
}

// Read Raw Pressure values
void AP_Baro_BMP085::ReadPress()
{
	uint8_t buf[3];

	if (micros() < _retry_time) {
		return;
	}

	readmem(0xF6, 3, buf);
	_retry_time = micros() + 1000000;


	RawPress = (((uint32_t)buf[0] << 16)
			| ((uint32_t)buf[1] << 8)
			| ((uint32_t)buf[2])) >> (8 - OVERSAMPLING);
}

// Send Command to Read Temperature
void AP_Baro_BMP085::Command_ReadTemp()
{
	writemem(0xF4, 0x2E);
	_last_temp_read_command_time = micros();
}

// Read Raw Temperature values
void AP_Baro_BMP085::ReadTemp()
{
	uint8_t buf[2];
	int32_t _temp_sensor;

	if (micros() < _retry_time) {
		return;
	}

	readmem(0xF6, 2, buf);
	_retry_time = micros() + 1000;


	_temp_sensor = buf[0];
	_temp_sensor = (_temp_sensor << 8) | buf[1];

	RawTemp = 0.8*RawTemp + 0.2*_temp_sensor;
}


// Calculate Temperature and Pressure in real units.
void AP_Baro_BMP085::Calculate()
{
	int32_t x1, x2, x3, b3, b5, b6, p;
	uint32_t b4, b7;
	int32_t tmp;

	// See Datasheet page 13 for this formulas
	// Based also on Jee Labs BMP085 example code. Thanks for share.
	// Temperature calculations
	x1 = ((int32_t)RawTemp - ac6) * ac5 >> 15;
	x2 = ((int32_t) mc << 11) / (x1 + md);
	b5 = x1 + x2;
	_temp_sum += (b5 + 8) >> 4;

	// Pressure calculations
	b6 = b5 - 4000;
	x1 = (b2 * (b6 * b6 >> 12)) >> 11;
	x2 = ac2 * b6 >> 11;
	x3 = x1 + x2;
	//b3 = (((int32_t) ac1 * 4 + x3)<<OVERSAMPLING + 2) >> 2; // BAD
	//b3 = ((int32_t) ac1 * 4 + x3 + 2) >> 2;  //OK for OVERSAMPLING=0
	tmp = ac1;
	tmp = (tmp*4 + x3)<<OVERSAMPLING;
	b3 = (tmp+2)/4;
	x1 = ac3 * b6 >> 13;
	x2 = (b1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
	b7 = ((uint32_t) RawPress - b3) * (50000 >> OVERSAMPLING);
	p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	_press_sum += p + ((x1 + x2 + 3791) >> 4);

	_count++;
	if (_count == 254) {
		_temp_sum *= 0.5f;
		_press_sum *= 0.5f;
		_count /= 2;
	}
}



#endif /* SENSOR_ALTIMETERBMP085_V2_H_ */
