/**
 * Breeze Project
 * ----------------------------------------
 * 2014 (c) Adrien HADJ-SALAH

 * PIN list :
 * - 6 x Servos
 * - ESC (Pin 6)
 * - optionnal [GPS Pin16,17 Serial2 (Actually just one pin needed)]
 * - Airspeed on analog pin 1
 * - Gyro on SDA/SCL pin mega 20 (yellow) 21 (orange) (A4 A5 on Uno)
 * - Serial1 for RF link on 18 (blue) 19 (green) Rx Tx
 * 
 ***********************************************************
 * Using Timer 4 for ESC, timer 3 for servo with lib,  Serial2 (Hardware Rx Pin 17 mega) for GPS
 * 
 * First succesful flight : 03/12/2014
 */
#include "Breeze.h"
#include "Scenario.h"
#include <Wire.h>
#include <BMP085.h>


long dt100HzSum = 0;
long iter100Hz = 0;

/*******************************************************************
 * Main function
 ******************************************************************/
void setup() {
	Serial.begin(115200) ;
	Serial.println("startup") ;
	previousTime = micros();

	setupMotors() ;
	Serial.println("Motors armed");

	// Create UAV structure and set variables to default values
	initializeUAVStructure(UAVCore) ;
	initializeUAVConfiguration();
	initFlightControl();
	Serial.println("UAV software initialized and flight control also");

	// Initialize components
	setupGyro() ;
	Serial.println("Gyro armed");

	setupAltimeter();
	Serial.println("Altimeter armed");

	setupServos() ;
	Serial.println("Servos armed");

	/*if (USE_RANGE_FINDER) {
		setupRangeDetector() ;
		Serial.println("Range detector armed");
	}*/


	if (USE_GPS_NAVIGUATION) {
		setupGPS();
		Serial.println("GPS armed");

		initNavig();
		Serial.println("Naviguation controller armed");
	}

	if (USE_AIRSPEED_SENSOR) {
		setupAirspeed();
		Serial.println("Airspeed sensor armed");
	}

	setupRFLink(&(UAVCore->autopilot), &(UAVCore->deciThrustPercent),
			&aileronCmd, &gouvernCmd, &rubberCmd, &flapsCmd,
			UAVCore->attitudeCommanded,
			&NAVIG_METHOD_ANGLE_DIFF,
			param,
			&AUTOSPEED_CONTROLLER,
			&(UAVCore->v_ms_goal)) ;
	Serial.println("RF link armed");


	Serial.println("Breeze armed");
	sendRFMessage("UAV_armed");

	// Initial state of flight : take-off after setup finished
	UAVCore->flightState = TAKEOFF;


	// Wait 2 seconds before starting
	delay(2000) ;
}



/*******************************************************************
 * Flight functions
 ******************************************************************/
// Read sensors and update data
void updateAttitude() {  
	// Update gyro data
	updateGyroData();

	if (GYRO_TYPE == T_GYRO_MPU6050) {
		UAVCore->currentAttitude->roll = kalAngleX;
		UAVCore->currentAttitude->pitch = -kalAngleY;
	}
	else {
		UAVCore->currentAttitude->roll = kalAngleX;
		UAVCore->currentAttitude->pitch = kalAngleY;
	}

	if (USE_GPS_NAVIGUATION) {
		UAVCore->headingCap = currentHeading;
	}

	UAVCore->currentAttitude->yaw = 0.0;

	UAVCore->gyroXrate = gyroXrate;
	UAVCore->gyroYrate = gyroYrate;
}


/*******************************************************************
 * Write command output to the servos
 ******************************************************************/
void processCommand() {
	writeRoll() ;
	writeGouvern();
	writeFlaps();
	writeRubber();
}


/*void updateRangeFinder() {
	UAVCore->altitudeSonar->addValue(distCm, rangeTimeMeasureUs);
}*/




void fullManual() {
	// Attitude commanded not written,
	// Stabilize function skipped only on no stabilized mode
	// Otherwise, user defined roll pitch to adopt and stabilized function do the job
	// Control by RF link
	if ((currentTime-lastTimeLinkWithGS) > S_TO_US*3) {
		// TODO return home procedure : switch to auto, UAV must return home and land
		UAVCore->deciThrustPercent = 0;	
		UAVCore->attitudeCommanded->roll = 0;		
		UAVCore->attitudeCommanded->pitch = 0;
		UAVCore->attitudeCommanded->yaw = 15;
	}
}


void reinitFlightModeParameters() {
	// Desactivate autospeed controller
	AUTOSPEED_CONTROLLER = 0;
	
	// Desactivate takeoff start time
	time_TakeOffStart = 0; 
	
	// Desactivate this function
	rf_automodeSwitchToken = false;
}

/*******************************************************************
 * 100Hz task (each 10 ms)
 ******************************************************************/
void process100HzTask() {
	G_Dt = (currentTime - hundredHZpreviousTime) / S_TO_US;
	

	// Update attitude from gyro
	updateAttitude() ;
	

	if (hundredHZpreviousTime > 0) {
		dt100HzSum += (currentTime - hundredHZpreviousTime);
		iter100Hz ++;
	}

	if (iter100Hz > 100000) {
		iter100Hz = 0;
		dt100HzSum = 0;
	}

	hundredHZpreviousTime = currentTime;
}


/*******************************************************************
 * 50Hz task (20 ms)
 ******************************************************************/
void process50HzTask() {

	// If UAV in auto mode
	// Define new command (roll, pitch, yaw, thrust) by using PID 
	// To reach roll pitch and yaw desired
	if (UAVCore->autopilot || (UAVCore->autopilot == false && rf_manual_StabilizedFlight == 1)) {
		stabilize2(UAVCore->attitudeCommanded->roll - UAVCore->currentAttitude->roll, 
				UAVCore->attitudeCommanded->pitch - UAVCore->currentAttitude->pitch,
				UAVCore->attitudeCommanded->yaw,
				&aileronCmd, &gouvernCmd, &rubberCmd,
				UAVCore->gyroXrate, UAVCore->gyroYrate,
				UAVCore->deciThrustPercent);

	}

	//-----------------------------------------------
	// Process and order all commands
	processCommand() ; 

	//-----------------------------------------------
	// Autopilot switch (On / Off)
	if (UAVCore->autopilot) {
		flightMode = flightModeAutopilot;
	}
	else {
		flightMode = FULL_MANUAL; // Better : flightModeManual (could be more than one version of manual control)
	}
	if (rf_automodeSwitchToken) {
		reinitFlightModeParameters();
	}


	//-----------------------------------------------
	// Define attitude desired to assure the mission
	switch (flightMode) {
	case FULL_MANUAL:
		fullManual();
		break;
	case FULL_FLIGHT_GPS:
		flightByGPS();
		break;
	case STABILIZE_ONLY:
		stabilizeOnly();
		break;
	case GROUND_NAV:
		groundNavDemo();
		break;
	case RTL:
		rtlNav();
		break;
	default:
		Serial.println("Unknow flight mode");
		break;		
	}


	// Study performance
	// long cDt = micros();

	// Update RF data down and up
	updateRFLink50Hz() ;

	/*
        Study performance (This function costs 1.2 ms max)
	double dt = (micros()-cDt)/1000.0;
	Serial.print("RF 50 Hz time elapsed in function =  ");
	Serial.print(dt, 2);
	Serial.println(" ms");
	 */

	/*if (USE_RANGE_FINDER) {
			// Update range finder data
			ultrasonicMakePulse();
			updateRangeFinder();
	}*/

	// Motor update
	// Command motor at % thrust
	motorUpdateCommandDeciPercent(UAVCore->deciThrustPercent);

}

/*******************************************************************
 * 20Hz task (50ms)
 ******************************************************************/
void process20HzTask() {
	// Update airspeed sensor
	if (USE_AIRSPEED_SENSOR) {
		double newAirspeedVms = updateAirspeed();
		airspeed_ms_mean->addValue(newAirspeedVms, currentTime);

		// Only if autospeed controller is activated and flight is in cruise state,
		// then adjust thrust regarding the current speed vms
		if (AUTOSPEED_CONTROLLER == 1) {
			// Auto-speed control with PID
			UAVCore->deciThrustPercent = controlSpeedWithThrustV2(currentTime, UAVCore->deciThrustPercent, UAVCore->v_ms_goal, UAVCore->currentAttitude);
		}
	}
}


/*******************************************************************
 * 10Hz task (100ms)
 ******************************************************************/
void process10HzTask() {
	// Update low priority rf com
	updateLowPriorityRFLink();

	// Update altimeter
	updateAltimeter();


#if MEASURE_VIBRATION	
	Serial.print("Acc noise vibration = ");
	Serial.print(accNoise);
	Serial.print(" | roll = ");
	Serial.print(UAVCore->currentAttitude->roll);
	Serial.print(" | pitch = ");
	Serial.print(UAVCore->currentAttitude->pitch);
	Serial.print(" | Acc X = ");
	Serial.print((Accel_output[0] - Accel_cal_x)/256);
	Serial.print(" | thrust = ");
	Serial.println(UAVCore->deciThrustPercent/10.0);
#endif
}

/*******************************************************************
 * 5Hz task (200ms)
 ******************************************************************/
void process5HzTask() {
	// Add speed calculated by the GPS
	if (USE_GPS_NAVIGUATION) {
		UAVCore->v_ms = lastVms;
		v_ms_mean->addValue(UAVCore->v_ms, currentTime);
	}
}



/*******************************************************************
 * 2Hz task (500ms)
 ******************************************************************/
void process2HzTask() {

	/*Serial.print("Airspeed : ");
	Serial.print(airspeed_ms_mean->getAverage());
	Serial.println(" m/s");

	Serial.print("Dt 100 Hz = ");
	Serial.print(dt100HzSum/iter100Hz);
	Serial.println(" us");
	 */

}




/*******************************************************************
 * 1Hz task 
 ******************************************************************/
void process1HzTask() {

	//---------------------------------------------------------
	// Send data to the ground station
	// Current position if GPS used : currentPosition
	updateRFLink1hz(toCenti(UAVCore->currentAttitude->roll), toCenti(UAVCore->currentAttitude->pitch), 
			(int)(UAVCore->headingCap),
			(int)(altitudeBarometer->getAverage()), toCenti(airspeed_ms_mean->getAverage()),
			currentPosition.lat, currentPosition.lon,
			(int)angleDiff, UAVCore->autopilot, currentWP);

	//---------------------------------------------------------
	// Print some data to the user
	/*Serial.print("V ms goal = ");
	Serial.print(UAVCore->v_ms_goal);
	Serial.print("\t | \t Thrust % = ") ;
	Serial.println(UAVCore->deciThrustPercent/10.0);
	Serial.print("Autopilot : ");
	Serial.println(UAVCore->autopilot ? "On" : "Off");


	Serial.print("    \tMesured\tCommanded\t\tCommand");
	Serial.println(" ");
	Serial.print("Roll\t\t");
	Serial.print(UAVCore->currentAttitude->roll) ;
	Serial.print("\t\t");
	Serial.print(UAVCore->attitudeCommanded->roll) ;
	Serial.print("\t\t\t");
	Serial.print(aileronCmd) ;

	Serial.println(" ");
	Serial.print("Pitch\t\t");
	Serial.print(UAVCore->currentAttitude->pitch) ;
	Serial.print("\t\t");
	Serial.print(UAVCore->attitudeCommanded->pitch) ;
	Serial.print("\t\t\t");
	Serial.print(gouvernCmd) ;

	Serial.println(" ");
	Serial.print("Yaw\t\t");
	Serial.print(UAVCore->currentAttitude->yaw) ;
	Serial.print("\t\t");
	Serial.print(UAVCore->attitudeCommanded->yaw) ;
	Serial.print("\t\t\t");
	Serial.print(rubberCmd) ;
	Serial.println(" ");
	 */
}


/*******************************************************************
 * Measure critical sensors task 
 * Must be runned as fast as possible
 ******************************************************************/
void measureCriticalSensors() {
	if (USE_GPS_NAVIGUATION) {
		updateGPS();
	}

	updateCriticalRFLink();
}

/*******************************************************************
 * Main loop funtions
 ******************************************************************/
void loop () {

	currentTime = micros();
	deltaTime = currentTime - previousTime;

	measureCriticalSensors();

	// ================================================================
	// 100Hz task loop
	// ================================================================
	if (deltaTime >= 10000) {
		frameCounter++;

		process100HzTask();

		// ================================================================
		// 50Hz task loop
		// ================================================================
		if (frameCounter % TASK_50HZ == 0) {  //  50 Hz tasks
			process50HzTask();
		}

		// ================================================================
		// 20Hz task loop
		// ================================================================
		if (frameCounter % TASK_20HZ == 0) {  //  20 Hz tasks
			process20HzTask();
		}
		// ================================================================
		// 10Hz task loop
		// ================================================================
		if (frameCounter % TASK_10HZ == 0) {  //   10 Hz tasks
			process10HzTask();
		}

		// ================================================================
		// 5Hz task loop
		// ================================================================
		if (frameCounter % TASK_5HZ == 0) {  //   5 Hz tasks
			process5HzTask();
		}

		// ================================================================
		// 2Hz task loop
		// ================================================================
		if (frameCounter % TASK_2HZ == 0) {  //   2 Hz tasks
			process2HzTask();
		}

		// ================================================================
		// 1Hz task loop
		// ================================================================
		if (frameCounter % TASK_1HZ == 0) {  //   1 Hz tasks
			process1HzTask();
		}

		previousTime = currentTime;
	}

	if (frameCounter >= 100) {
		frameCounter = 0;
	}
}
