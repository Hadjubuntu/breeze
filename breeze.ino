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
#include "modules/scheduler/Scheduler.h"
#include "modules/scenario/Scenario.h"
#include "arch/AVR/wire/Wire.h"
#include "arch/AVR/MCU/MCU.h"
#include "peripherals/altimeter/Sensor_AltimeterBMP085.h"



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
	
	updateSurfaceControls();
}



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
		UAVCore->attitudeCommanded->yaw = 0;
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
		Logger.println("Unknow flight mode");
		break;		
	}



	// Update RF data down and up
	//------------------------------------------------------------
	updateRFLink50Hz() ;


	//------------------------------------------------------------
	/*if (USE_RANGE_FINDER) {
			// Update range finder data
			ultrasonicMakePulse();
			updateRangeFinder();
	}*/
	
	// Motor update
	// Command motor at % thrust
	//------------------------------------------------------------
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
	//------------------------------------------------------------
	updateLowPriorityRFLink();

	// Update altimeter
	//------------------------------------------------------------
	updateAltimeter();


#if MEASURE_VIBRATION	
	Logger.print("Acc noise vibration = ");
	Logger.print(accNoise);
	Logger.print(" | roll = ");
	Logger.print(UAVCore->currentAttitude->roll);
	Logger.print(" | pitch = ");
	Logger.print(UAVCore->currentAttitude->pitch);
	Logger.print(" | Acc X = ");
	Logger.print((Accel_output[0] - Accel_cal_x)/256);
	Logger.print(" | Acc X only = ");
	Logger.print((Accel_output[0] - Accel_cal_x)/256 - sin(toRad(abs(UAVCore->currentAttitude->pitch))));

	Logger.print(" | Gdt(0) (ms) = ");
	Logger.println(G_Dt*1000.0);
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

	/*Logger.print("Airspeed : ");
	Logger.print(airspeed_ms_mean->getAverage());
	Logger.println(" m/s");

	Logger.print("Dt 100 Hz = ");
	Logger.print(dt100HzSum/iter100Hz);
	Logger.println(" us");
	 */
	
	 schedulerStats();
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
	/*Logger.print("V ms goal = ");
	Logger.print(UAVCore->v_ms_goal);
	Logger.print("\t | \t Thrust % = ") ;
	Logger.println(UAVCore->deciThrustPercent/10.0);
	Logger.print("Autopilot : ");
	Logger.println(UAVCore->autopilot ? "On" : "Off");


	Logger.print("    \tMesured\tCommanded\t\tCommand");
	Logger.println(" ");
	Logger.print("Roll\t\t");
	Logger.print(UAVCore->currentAttitude->roll) ;
	Logger.print("\t\t");
	Logger.print(UAVCore->attitudeCommanded->roll) ;
	Logger.print("\t\t\t");
	Logger.print(aileronCmd) ;

	Logger.println(" ");
	Logger.print("Pitch\t\t");
	Logger.print(UAVCore->currentAttitude->pitch) ;
	Logger.print("\t\t");
	Logger.print(UAVCore->attitudeCommanded->pitch) ;
	Logger.print("\t\t\t");
	Logger.print(gouvernCmd) ;

	Logger.println(" ");
	Logger.print("Yaw\t\t");
	Logger.print(UAVCore->currentAttitude->yaw) ;
	Logger.print("\t\t");
	Logger.print(UAVCore->attitudeCommanded->yaw) ;
	Logger.print("\t\t\t");
	Logger.print(rubberCmd) ;
	Logger.println(" ");
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

	if (USE_RADIO_FUTABA) {
		sBus.FeedLine();
		if (sBus.toChannels == 1){
			sBus.UpdateChannels();
			sBus.toChannels = 0;
		}
	}

	updateCriticalRFLink();
}



Task uavTasks[] = {
		{&process100HzTask, 0, 10000, 5000, 0},
		{&process50HzTask, 0, 20000, 10000, 0},
		{&process20HzTask, 0, 50000, 25000, 0},
		{&process10HzTask, 0, 100000, 50000, 0},
		{&process5HzTask, 0, 200000, 50000, 0},
		{&process2HzTask, 0, 500000, 50000, 0},
		{&process1HzTask, 0, 1000000, 50000, 0}};

// Setup the UAV
//---------------------------------------------------------------
void setup() {
	Logger.begin(115200) ;
	Logger.println("startup") ;
	previousTime = timeUs();

	setupMotors() ;
	Logger.println("Motors armed");

	// Create UAV structure and set variables to default values
	initializeUAVStructure(UAVCore) ;
	initializeUAVConfiguration();
	initFlightControl();
	Logger.println("UAV software initialized and flight control also");

	// Initialize components
	setupGyro() ;
	Logger.println("Gyro armed");

	setupAltimeter();
	Logger.println("Altimeter armed");

	setupServos() ;
	Logger.println("Servos armed");

	/*if (USE_RANGE_FINDER) {
		setupRangeDetector() ;
		Logger.println("Range detector armed");
	}*/


	if (USE_GPS_NAVIGUATION) {
		setupGPS();
		Logger.println("GPS armed");

		initNavig();
		Logger.println("Naviguation controller armed");
	}

	if (USE_AIRSPEED_SENSOR) {
		setupAirspeed();
		Logger.println("Airspeed sensor armed");
	}

	setupRFLink(&(UAVCore->autopilot), &(UAVCore->deciThrustPercent),
			&aileronCmd, &gouvernCmd, &rubberCmd, &flapsCmd,
			UAVCore->attitudeCommanded,
			&NAVIG_METHOD_ANGLE_DIFF,
			param,
			&AUTOSPEED_CONTROLLER,
			&(UAVCore->v_ms_goal)) ;
	Logger.println("RF link armed");


	Logger.println("Breeze armed");

	// Initialize Scheduler
	int nbTasks = sizeof(uavTasks) / sizeof(uavTasks[0]);
	schedulerInit(uavTasks, nbTasks);

	// Initial state of flight : take-off after setup finished
	UAVCore->flightState = TAKEOFF;


	// Wait 2 seconds before starting
	delay(2000) ;
}



/*******************************************************************
 * Main loop funtions
 ******************************************************************/
void loop () {

	currentTime = timeUs();
	deltaTime = currentTime - previousTime;

	measureCriticalSensors();
	schedulerRun();
}
