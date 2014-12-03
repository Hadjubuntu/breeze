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
 */
#include "Breeze.h"
#include <Wire.h>
#include <L3G.h>
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
	initializeUAVParameters(UAVCore) ;
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
			UAVCore->attitudeCommanded) ;
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


void processCommand() {

	// Command aileron for roll
	writeRoll() ;
	writeGouvern();
	writeFlaps();
	writeRubber();
}


/*void updateRangeFinder() {
	UAVCore->altitudeSonar->addValue(distCm, rangeTimeMeasureUs);
}*/

void flightByGPS() {
	if (UAVCore->flightState == TAKEOFF) {
		if (time_TakeOffStart == 0) {
			time_TakeOffStart = currentTime;
		}

		// Full thrust
		UAVCore->deciThrustPercent = 700 ;

		// Give 30% thrust the 2 first seconds, then full thrust
		if ((currentTime-time_TakeOffStart) < 2 * S_TO_US) {
			UAVCore->deciThrustPercent = 300;
		}

		double pitchDesired = 30;

		// Don't try to takeoff before v_min or timeout of 7 seconds
		// But give a pitch offset, to prevent to put the nose down
		if ((USE_AIRSPEED_SENSOR && airspeed_ms_mean->getAverage() < V_MIN_TAKEOFF_MS)
				|| (currentTime-time_TakeOffStart) < 5 * S_TO_US) {
			pitchDesired = 5 ;
		}

		controlAttitudeCommand(UAVCore->currentAttitude, UAVCore->attitudeCommanded, 
				G_Dt, currentTime,  
				0, pitchDesired, 0);

		// Flaps down
		flapsCmd = 60;


		// After z higher than 3 meters, takeoff complete
		if (((currentTime-time_TakeOffStart) > MAX_DELAY_TAKEOFF_US) || (altitudeBarometer->areDataRelevant(currentTime)
				&& altitudeBarometer->getAverage() > 300.0)) {

			time_TakeOffDone = currentTime ;
			UAVCore->flightState = CLIMB ;
		}
	}
	else if (UAVCore->flightState == CLIMB) {
		if (time_ClimbStart == 0) {
			time_ClimbStart = currentTime;
		}

		flapsCmd = 30;
		UAVCore->deciThrustPercent = 700;
		controlAttitudeCommand(UAVCore->currentAttitude, UAVCore->attitudeCommanded, 
				G_Dt, currentTime,  
				0, 30, 0);

		if ((currentTime-time_ClimbStart) > CLIMB_DURATION_SECONDS * S_TO_US) {
			time_ClimbDone = currentTime;
			UAVCore->flightState = CRUISE ;
		}
	}
	else if (UAVCore->flightState == CRUISE) {

		if (AUTOSPEED_CONTROLLER == 0) {
			// cruise speed depends on current pitch (default thrust 50%)
			UAVCore->deciThrustPercent = 700 ;

			if (UAVCore->currentAttitude->pitch < 0) {
				UAVCore->deciThrustPercent = 500 ;
			}
			else if (UAVCore->currentAttitude->pitch < -10) {
				UAVCore->deciThrustPercent = 400 ;
			}
			else if (UAVCore->currentAttitude->pitch > 15) {
				UAVCore->deciThrustPercent = 900;
			}
		}
		else {
			// In auto speed control, we just define a speed to achieve, thrust is controlled by a PD controller
			// We choose 38 km/h to have enough lift to compensate the weight without trying to pitch-up
			UAVCore->v_ms_goal = 12; // ~40 km/h
		}

		// Update GPS heading
		updateHeading(altitudeBarometer);

		// Create related attitude command
		// If no signal GPS, then fly in circle low thrust, flaps 30% down
		if (GPSState == GPS_STATE_LOST) {
			// Desactivate autospeed controller and set low thrust
			AUTOSPEED_CONTROLLER = 0;
			UAVCore->deciThrustPercent = 400 ;

			// Circle zero pitch and flaps down
			controlAttitudeCommand(UAVCore->currentAttitude, UAVCore->attitudeCommanded, G_Dt, currentTime,  30, 0, 0);
			flapsCmd = 30;
		}
		else {
			controlAttitudeCommand(UAVCore->currentAttitude, UAVCore->attitudeCommanded, G_Dt, currentTime,  gpsRollDesired, gpsPitchDesired, 0);
		}


		// Retrieve flaps
		flapsCmd = 0;

		// If mission done or in go home mode and distance acceptable to land or after 6 minutes, start landing
		// Change state and stop autospeed controller
		if (MissionDone 
				|| (modeGoHome && altitudeBarometer->areDataRelevant(currentTime) && altitudeBarometer->getAverage() < 250.0)
				|| (currentTime - time_ClimbDone) > 6*60*S_TO_US) {

			UAVCore->flightState = LANDING ;
		}
	}
	else if (UAVCore->flightState == LANDING) {

		double yawDesired = 0.0;

		if (HOLD_HEADING_IN_TAKEOFF_AND_LANDING) {
			yawDesired = yawToHoldHeading(currentHeading, landingHoldHeading);
		}

		// If airplane 2m50 over the ground, descent 12 degrees down
		if (altitudeBarometer->getAverage() > 250) {
			if (AUTOSPEED_CONTROLLER == 0) {
				UAVCore->deciThrustPercent = 400 ;
			}
			else {
				UAVCore->v_ms_goal = 10;
			}

			flapsCmd = 50;
			controlAttitudeCommand(UAVCore->currentAttitude, UAVCore->attitudeCommanded, G_Dt, currentTime,  0, -12, yawDesired);
		}
		else {
			flapsCmd = 70;
			// Descent from (2m50 to 50 centimeters)
			if (altitudeBarometer->getAverage() > 50) {

				if (AUTOSPEED_CONTROLLER == 0) {
					UAVCore->deciThrustPercent = 350 ;
				}
				else {
					UAVCore->v_ms_goal = 8;
				}

				controlAttitudeCommand(UAVCore->currentAttitude, UAVCore->attitudeCommanded, G_Dt, currentTime,  0, 0, yawDesired);
			}
			// Arrondi
			else {
				// Flaps for flare
				AUTOSPEED_CONTROLLER = 0;
				flapsCmd = 90;
				UAVCore->deciThrustPercent = 0 ;
				controlAttitudeCommand(UAVCore->currentAttitude, UAVCore->attitudeCommanded, G_Dt, currentTime,  0, 5, 0);
			}
		}
	}

}



/**
 * The plane must try to keep the roll and pitch to 0
 */
void stabilizeOnly() {
	if (time_TakeOffStart == 0) {
		time_TakeOffStart = currentTime;
	}

	if (currentTime-time_TakeOffStart <= 3*S_TO_US) {
		UAVCore->deciThrustPercent = 0; // 15 is enough for the plane to ride !
	}
	else {
		UAVCore->deciThrustPercent = 0;
	}
	flapsCmd = 90;
	controlAttitudeCommand(UAVCore->currentAttitude, UAVCore->attitudeCommanded, G_Dt, currentTime,  0, 0, 0);
}

/**
 * Ground Nav aims to test the GPS naviguation by steering the plane on the ground
 */
void groundNavDemo() {
	if (time_TakeOffStart == 0) {
		time_TakeOffStart = currentTime;
	}

	//AUTOSPEED_CONTROLLER = 0;
	// Configuration autospeed controller
	// with 1.2 m/s and almost 50% thrust max to achieve that goal
	AUTOSPEED_CONTROLLER = 1;
	UAVCore->v_ms_goal = 3.6;
	UAVCore->thrustMax = 48;
	UAVCore->thrustBurst = 52;
	
	// Stop motor if wrong value of pitch (on the nose..)
	if ( abs(UAVCore->currentAttitude->pitch) > 30) {
		UAVCore->deciThrustPercent = 0;
	}

	if (GPSState == GPS_STATE_LOST) {
		UAVCore->deciThrustPercent = 0;
	}
	// If the mission is done, stop motor and do flaps full
	else if (MissionDone) {
		UAVCore->deciThrustPercent = 0;
		flapsCmd = 90;
	}
	else {
		double yawDesired = 0.0;
		updateHeading(altitudeBarometer);


		// After 2 seconds, start guidance, before just maintain steering
		if (currentTime - time_TakeOffStart > S_TO_US * 2) {
			yawDesired = gpsRollDesired;
		}

		if (AUTOSPEED_CONTROLLER == 0) {
			// Just enough thrust to move the plane (15% is enough)
			UAVCore->deciThrustPercent = 150; 
		}

		// Simple move the rubber depending on gps roll demand
		UAVCore->attitudeCommanded->roll = 0;		
		UAVCore->attitudeCommanded->pitch = 10;
		UAVCore->attitudeCommanded->yaw = yawDesired;
	}
}

void fullManual() {
	// Attitude commanded not written,
	// Stabilize function skipped only on no stabilized mode
	// Otherwise, user defined roll pitch to adopt and stabilized function do the job
	// Control by RF link
  if ((currentTime-lastTimeLinkWithGS) > S_TO_US*3) {
    UAVCore->deciThrustPercent = 0;
  }
}

/*******************************************************************
 * 100Hz task (each 10 ms)
 ******************************************************************/
void process100HzTask() {
	G_Dt = (currentTime - hundredHZpreviousTime) / S_TO_US;

	if (hundredHZpreviousTime > 0) {
		dt100HzSum += (currentTime - hundredHZpreviousTime);
		iter100Hz ++;
	}

	if (iter100Hz > 100000) {
		iter100Hz = 0;
		dt100HzSum = 0;
	}

	hundredHZpreviousTime = currentTime;


	// Run this block on 100Hz only with battery supply ..
	// Update attitude from gyro
	updateAttitude() ;

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
	default:
		Serial.println("Unknow flight mode");
		break;		
	}
}


/*******************************************************************
 * 50Hz task (20 ms)
 ******************************************************************/
void process50HzTask() {

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
 * 10Hz task (100ms)
 ******************************************************************/
void process10HzTask() {
	// Update altimeter
	updateAltimeter();

	// Update airspeed sensor
	if (USE_AIRSPEED_SENSOR) {
		double newAirspeedVms = updateAirspeed();
		airspeed_ms_mean->addValue(newAirspeedVms, currentTime);
	}

	// Only if autospeed controller is activated and flight is in cruise state,
	// then adjust thrust regarding the current speed vms
	if (USE_AIRSPEED_SENSOR && AUTOSPEED_CONTROLLER == 1 && UAVCore->v_ms_goal > 0.0) {
		// Auto-speed control with PID
		UAVCore->deciThrustPercent = controlSpeedWithThrust(currentTime, UAVCore->deciThrustPercent, UAVCore->v_ms_goal, UAVCore->currentAttitude, UAVCore->thrustMin, UAVCore->thrustMax, UAVCore->thrustBurst);
	}
}

/*******************************************************************
 * 5Hz task (200ms)
 ******************************************************************/
void process5HzTask() {
	// Update speed (m/s) using GPS data
	// Normally the GPS update each 500 ms but sometime its faster
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
	Serial.print("Acc noise vibration = ");
	Serial.println(accNoise);*/

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
	/*Serial.print("Autopilot : ");
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
