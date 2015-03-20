/****************************************************************************
 * Sensor_AltimeterBMP085.h - BMP085/I2C (Digital Pressure Sensor) library for Arduino       *
 * Rewritted for the Breeze project
 * Adrien Hadj-Salah
 ****************************************************************************/
/****************************************************************************
 * Tested on Arduino Mega with BMP085 Breakout                               *
 * SDA   -> pin 20   (no pull up resistors)                                  *
 * SCL   -> pin 21   (no pull up resistors)                                  *
 * XCLR  -> not connected                                                    *
 * EOC   -> not connected                                                    *
 * GND   -> pin GND                                                          *
 * VCC   -> pin 3.3V                                                         *
 * NOTE: SCL and SDA needs pull-up resistors for each I2C bus.               *
 *  2.2kOhm..10kOhm, typ. 4.7kOhm                                            *
 *****************************************************************************/
#ifndef BMP085_h
#define BMP085_h

#include "arch/AVR/MCU/MCU.h"
#include "arch/AVR/wire/Wire.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define BMP085_ADDR                 0x77     //0x77 default I2C address
#define BUFFER_SIZE                 3

#define AUTO_UPDATE_TEMPERATURE     true    //default is true
// when true, temperature is measured everytime pressure is measured (Auto).
// when false, user chooses when to measure temperature (just call calcTrueTemperature()).
// used for dynamic measurement to increase sample rate (see BMP085 modes below).

/* ---- Registers ---- */
#define CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define CAL_AC3           0xAE  // R   Calibration data (16 bits)    
#define CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define CAL_B1            0xB6  // R   Calibration data (16 bits)
#define CAL_B2            0xB8  // R   Calibration data (16 bits)
#define CAL_MB            0xBA  // R   Calibration data (16 bits)
#define CAL_MC            0xBC  // R   Calibration data (16 bits)
#define CAL_MD            0xBE  // R   Calibration data (16 bits)
#define CONTROL           0xF4  // W   Control register 
#define CONTROL_OUTPUT    0xF6  // R   Output registers 0xF6=MSB, 0xF7=LSB, 0xF8=XLSB

// unused registers
#define SOFTRESET         0xE0
#define VERSION           0xD1  // ML_VERSION  pos=0 len=4 msk=0F  AL_VERSION pos=4 len=4 msk=f0
#define CHIPID            0xD0  // pos=0 mask=FF len=8
// BMP085_CHIP_ID=0x55

/************************************/
/*    REGISTERS PARAMETERS          */
/************************************/
// BMP085 Modes
#define MODE_ULTRA_LOW_POWER    0 //oversampling=0, internalsamples=1, maxconvtimepressure=4.5ms, avgcurrent=3uA, RMSnoise_hPA=0.06, RMSnoise_m=0.5
#define MODE_STANDARD           1 //oversampling=1, internalsamples=2, maxconvtimepressure=7.5ms, avgcurrent=5uA, RMSnoise_hPA=0.05, RMSnoise_m=0.4
#define MODE_HIGHRES            2 //oversampling=2, internalsamples=4, maxconvtimepressure=13.5ms, avgcurrent=7uA, RMSnoise_hPA=0.04, RMSnoise_m=0.3
#define MODE_ULTRA_HIGHRES      3 //oversampling=3, internalsamples=8, maxconvtimepressure=25.5ms, avgcurrent=12uA, RMSnoise_hPA=0.03, RMSnoise_m=0.25
// "Sampling rate can be increased to 128 samples per second (standard mode) for
// dynamic measurement.In this case it is sufficient to measure temperature only 
// once per second and to use this value for all pressure measurements during period."
// (from BMP085 datasheet Rev1.2 page 10).
// To use dynamic measurement set AUTO_UPDATE_TEMPERATURE to false and
// call calcTrueTemperature() from your code. 
// Control register
#define READ_TEMPERATURE        0x2E 
#define READ_PRESSURE           0x34 
//Other
#define MSLP                    101325          // Mean Sea Level Pressure = 1013.25 hPA (1hPa = 100Pa = 1mbar)



class BMP085 {
public:  
	BMP085();

	// BMP initialization
	void init();                                              // sets current elevation above ground level to 0 meters
	void init(byte _BMPMode, int32_t _initVal, bool _centimeters);   // sets a reference datum
	// if _centimeters=false _initVal is Pa
	// Who Am I
	byte getDevAddr();

	// BMP mode  
	byte getMode();        
	void setMode(byte _BMPMode);                   // BMP085 mode 
	// initialization
	void setLocalPressure(int32_t _Pa);            // set known barometric pressure as reference Ex. QNH
	void setLocalAbsAlt(int32_t _centimeters);     // set known altitude as reference
	void setAltOffset(int32_t _centimeters);       // altitude offset
	void sethPaOffset(int32_t _Pa);                // pressure offset
	void zeroCal(int32_t _Pa, int32_t _centimeters);// zero Calibrate output to a specific Pa/altitude 
	// BMP Sensors
	void getPressure(int32_t *_Pa);                // pressure in Pa + offset
	long getAltitude();       // altitude in centimeters + offset
	long getTemperature();    // temperature in Celsius
	void calcTrueTemperature();                    // calc temperature data b5 (only needed if AUTO_UPDATE_TEMPERATURE is false)  
	void calcTruePressure();    // calc Pressure in Pa
	void calTruPressureState1();
	// dummy stuff
	void dumpCalData();                           // debug only

	void writemem(uint8_t _addr, uint8_t _val);
	void readmem(uint8_t _addr, uint8_t _nbytes, uint8_t __buff[]);

	void updateAltitudeCm();

	bool _pressure_updated;
private:

	int ac1,ac2,ac3,b1,b2,mb,mc,md;               // cal data  
	unsigned int ac4,ac5,ac6;                     // cal data
	long b5, oldEMA;                                      // temperature data

	uint8_t _dev_address;
	byte _buff[BUFFER_SIZE];                      // buffer  MSB LSB XLSB
	int _oss;                                     // OverSamplingSetting
	int _pressure_waittime[4];                    // Max. Conversion Time Pressure is ms for each mode

	int32_t _cm_Offset, _Pa_Offset;
	int32_t _param_datum, _param_centimeters;

	void getCalData();        

	//  Chip state
	// State 0 : Ask temperature
	// State 1 : Read temperature - update temp
	// State 2 : Ask pressure
	// State 3 : Read pressure - update altitude
	int _chip_state ;
	long _last_write_cmd_us;

	long _altitude_cm ;
	long _dt_us;

	long _true_pressure;

};


BMP085::BMP085() {
	_pressure_updated = false;
	_chip_state = 0;
	_last_write_cmd_us = 0;
	_altitude_cm = 0;
	_dt_us = 0;
	_true_pressure = 0;

	_dev_address = BMP085_ADDR;
	_pressure_waittime[0] = 5; // These are maximum convertion times.
	_pressure_waittime[1] = 8; // It is possible to use pin EOC (End Of Conversion)
	_pressure_waittime[2] = 14;// to check if conversion is finished (logic 1)
	_pressure_waittime[3] = 26;// or running (logic 0) insted of waiting for convertion times.
	_cm_Offset = 0;
	_Pa_Offset = 0;               // 1hPa = 100Pa = 1mbar

	oldEMA = 0;
}

void BMP085::init() {
	init(MODE_HIGHRES, 0, true);
}

void BMP085::init(byte _BMPMode, int32_t _initVal, bool _Unitmeters){
	getCalData();               // initialize cal data

	calcTrueTemperature();      // initialize b5

	setMode(_BMPMode);
	_Unitmeters ? setLocalAbsAlt(_initVal) : setLocalPressure(_initVal);
}

byte BMP085::getDevAddr() {
	return _dev_address;
}

byte BMP085::getMode(){
	return _oss;
}

void BMP085::setMode(byte _BMPMode){
	_oss = _BMPMode;
}

void BMP085::setLocalPressure(int32_t _Pa){
	int32_t tmp_alt;

	_param_datum = _Pa;
	tmp_alt = getAltitude();    // calc altitude based on current pressure
	_param_centimeters = tmp_alt;
}

void BMP085::setLocalAbsAlt(int32_t _centimeters){
	int32_t tmp_Pa;

	_param_centimeters = _centimeters;
	getPressure(&tmp_Pa);    // calc pressure based on current altitude
	_param_datum = tmp_Pa;
}

void BMP085::setAltOffset(int32_t _centimeters){
	_cm_Offset = _centimeters;
}

void BMP085::sethPaOffset(int32_t _Pa){
	_Pa_Offset = _Pa;
}

void BMP085::zeroCal(int32_t _Pa, int32_t _centimeters){
	setAltOffset(_centimeters - _param_centimeters);
	sethPaOffset(_Pa - _param_datum);
}

void BMP085::getPressure(int32_t *_Pa){

	calcTrueTemperature();
	while (_true_pressure == 0) {
		calcTruePressure();
		delay(26);
		calTruPressureState1();
	}

	*_Pa = _true_pressure / pow((1 - (float)_param_centimeters / 4433000), 5.255) + _Pa_Offset;
	// converting from float to int32_t truncates toward zero, 1010.999985 becomes 1010 resulting in 1 Pa error (max).
	// Note that BMP085 abs accuracy from 700...1100hPa and 0..+65�C is +-100Pa (typ.)
}

void BMP085::updateAltitudeCm() {

	_altitude_cm =  4433000 * (1 - pow((_true_pressure / (float)_param_datum), 0.1903)) + _cm_Offset;
	// converting from float to int32_t truncates toward zero, 100.999985 becomes 100 resulting in 1 cm error (max).
}

long BMP085::getAltitude(){
	return _altitude_cm;
}

long BMP085::getTemperature() {
	return  ((b5 + 8) >> 4) / 10;
}

void BMP085::calcTrueTemperature(){
	long ut,x1,x2;

	//read Raw Temperature
	writemem(CONTROL, READ_TEMPERATURE);

	delay(5);                                         // min. 4.5ms read Temp delay

	readmem(CONTROL_OUTPUT, 2, _buff);
	ut = ((long)_buff[0] << 8 | ((long)_buff[1]));    // uncompensated temperature value

	// calculate temperature
	x1 = ((long)ut - ac6) * ac5 >> 15;
	x2 = ((long)mc << 11) / (x1 + md);
	b5 = x1 + x2;
}

void BMP085::calTruPressureState1() {
	if (_chip_state == 1) {
		long c_dt_us = timeUs()-_last_write_cmd_us;

		 if (c_dt_us >= _pressure_waittime[_oss]*1000) { // && c_dt_us < _pressure_waittime[_oss]*1000+200
			_dt_us = c_dt_us;

			long up,x1,x2,x3,b3,b6,p;
			unsigned long b4,b7;
			int32_t tmp;


			readmem(CONTROL_OUTPUT, 3, _buff);
			up = ((((long)_buff[0] <<16) | ((long)_buff[1] <<8) | ((long)_buff[2])) >> (8-_oss)); // uncompensated pressure value

			// calculate true pressure
			b6 = b5 - 4000;             // b5 is updated by calcTrueTemperature().
			x1 = (b2* (b6 * b6 >> 12)) >> 11;
			x2 = ac2 * b6 >> 11;
			x3 = x1 + x2;
			tmp = ac1;
			tmp = (tmp * 4 + x3) << _oss;
			b3 = (tmp + 2) >> 2;
			x1 = ac3 * b6 >> 13;
			x2 = (b1 * (b6 * b6 >> 12)) >> 16;
			x3 = ((x1 + x2) + 2) >> 2;
			b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
			b7 = ((uint32_t)up - b3) * (50000 >> _oss);
			p = b7 < 0x80000000 ? (b7 << 1) / b4 : (b7 / b4) << 1;
			x1 = (p >> 8) * (p >> 8);
			x1 = (x1 * 3038) >> 16;
			x2 = (-7357 * p) >> 16;
			long tmpTruePressure = p + ((x1 + x2 + 3791) >> 4);

//			if (_pressure_updated) {
//				_true_pressure = 0.1*_true_pressure + 0.9*tmpTruePressure;
//			}
//			else {
				_true_pressure = tmpTruePressure;
				_pressure_updated = true;
//			}

			// Force update altitude cm
			updateAltitudeCm();

			_chip_state = 0;
		}
	}
}

void BMP085::calcTruePressure() {

	//read Raw Pressure
	if (_chip_state == 0) {
		writemem(CONTROL, READ_PRESSURE+(_oss << 6));
		_last_write_cmd_us = timeUs();
		_chip_state = 1;
	}
}

void BMP085::dumpCalData() {
	//	Logger.println("---cal data start---");
	//	Logger.print("ac1:");
	//	Logger.println(ac1);
	//	Logger.print("ac2:");
	//	Logger.println(ac2);
	//	Logger.print("ac3:");
	//	Logger.println(ac3);
	//	Logger.print("ac4:");
	//	Logger.println(ac4);
	//	Logger.print("ac5:");
	//	Logger.println(ac5);
	//	Logger.print("ac6:");
	//	Logger.println(ac6);
	//	Logger.print("b1:");
	//	Logger.println(b1);
	//	Logger.print("b2:");
	//	Logger.println(b2);
	//	Logger.print("mb:");
	//	Logger.println(mb);
	//	Logger.print("mc:");
	//	Logger.println(mc);
	//	Logger.print("md:");
	//	Logger.println(md);
	//	Logger.println("---cal data end---");
}

//PRIVATE methods

void BMP085::getCalData() {
	readmem(CAL_AC1, 2, _buff);
	ac1 = ((int)_buff[0] <<8 | ((int)_buff[1]));
	readmem(CAL_AC2, 2, _buff);
	ac2 = ((int)_buff[0] <<8 | ((int)_buff[1]));
	readmem(CAL_AC3, 2, _buff);
	ac3 = ((int)_buff[0] <<8 | ((int)_buff[1]));
	readmem(CAL_AC4, 2, _buff);
	ac4 = ((unsigned int)_buff[0] <<8 | ((unsigned int)_buff[1]));
	readmem(CAL_AC5, 2, _buff);
	ac5 = ((unsigned int)_buff[0] <<8 | ((unsigned int)_buff[1]));
	readmem(CAL_AC6, 2, _buff);
	ac6 = ((unsigned int)_buff[0] <<8 | ((unsigned int)_buff[1]));
	readmem(CAL_B1, 2, _buff);
	b1 = ((int)_buff[0] <<8 | ((int)_buff[1]));
	readmem(CAL_B2, 2, _buff);
	b2 = ((int)_buff[0] <<8 | ((int)_buff[1]));
	readmem(CAL_MB, 2, _buff);
	mb = ((int)_buff[0] <<8 | ((int)_buff[1]));
	readmem(CAL_MC, 2, _buff);
	mc = ((int)_buff[0] <<8 | ((int)_buff[1]));
	readmem(CAL_MD, 2, _buff);
	md = ((int)_buff[0] <<8 | ((int)_buff[1]));
}


void BMP085::writemem(uint8_t _addr, uint8_t _val) {
	Wire.beginTransmission(_dev_address);   // start transmission to device
	Wire.write(_addr); // send register address
	Wire.write(_val); // send value to write
	Wire.endTransmission(); // end transmission
}

void BMP085::readmem(uint8_t _addr, uint8_t _nbytes, uint8_t __buff[]) {
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

#endif
