/**
 * Breeze Project
 * ---------------------------------------------------------
 * Breeze is an open-source autopilot for UAV.
 * ---------------------------------------------------------
 * 2014 (c) Adrien HADJ-SALAH
 *
 * - Airspeed on analog pin 1
 * - Gyro on SDA/SCL pin mega 20 21
 * - Serial1 for RF link on 18 19 Rx Tx (Xbee)
 * 
 ***********************************************************
 * Serial3 (Rx pin 15 mega) for SBUS Futaba,
 * Serial2 (Hardware Rx Pin 17 mega) for GPS
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
 * Update IMU data
 ******************************************************************/
// Read sensors and update data
void updateAttitude() {  
	// Update gyro data
	updateGyroData();

	// Set new current attitude
	UAVCore->currentAttitude->roll = kalAngleX;
	UAVCore->currentAttitude->pitch = kalAngleY;

	// Update heading with GPS data
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
	switch (Firmware) {
	case FIXED_WING:
		updateSurfaceControls();
		break;
	case QUADCOPTER:
		updateMotorRepartition();
		break;
	default:
		Logger.println("Unknow firmware");
		break;
	}
}



void fullManual() {
	// Attitude commanded not written,
	// Stabilize function skipped only on no stabilized mode
	// Otherwise, user defined roll pitch to adopt and stabilized function do the job
	// Control by RF link
	if ((currentTime-lastTimeLinkWithGS) > S_TO_US*3) {
		// TODO return home procedure : switch to auto, UAV must return home and land
		UAVCore->deciThrustPercent = 0;	
		UAVCore->deciThrustCmd = 0;
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
 * Radio Futaba Controller
 ******************************************************************/
void updateRFRadioFutaba() {
	if (USE_RADIO_FUTABA == 1) {
		// Connexion lost with 
		if (timeUs() - sBus.lastUpdateUs > S_TO_US) {
			// failsafe
			UAVCore->deciThrustPercent = 0;
			UAVCore->deciThrustCmd = 0;
			UAVCore->autopilot = false;
		}
		else {
			UAVCore->attitudeCommanded->roll = (sBus.channels[0]-sBus.channelsCalib[0])*0.0682;
			UAVCore->attitudeCommanded->pitch = (sBus.channels[1]-sBus.channelsCalib[1])*0.0682;
			UAVCore->attitudeCommanded->yaw = (sBus.channels[3]-sBus.channelsCalib[3])*0.0682;
			UAVCore->deciThrustPercent = max((sBus.channels[2]-365)/1.38, 0);
			UAVCore->deciThrustCmd = UAVCore->deciThrustPercent;
		}
	}
}

void updateRFRadoFutabaLowFreq() {
	if (USE_RADIO_FUTABA == 1) {

		// LOW means manual
		if (sBus.channels[4] < 1000) {
			UAVCore->autopilot = false;
		}
		// HIGH means auto
		else {
			UAVCore->autopilot = true;
		}

		switch (sBus.channels[7]) {
		case 144:
			flapsCmd = 0;
			break;
		case 1024:
			flapsCmd = 50;
			break;
		case 1904:
			flapsCmd = 90;
			break;
		default:
			flapsCmd = 0;
			break;
		}
	}
}


/*******************************************************************
 * 100Hz task (each 10 ms)
 ******************************************************************/
void process100HzTask() {
	G_Dt = (currentTime - hundredHZpreviousTime) / S_TO_US;

	// Update attitude from gyro
	updateAttitude() ;


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
	updateRFRadioFutaba();



	// Motor update
	// Command motor at % thrust
	//------------------------------------------------------------
	motorUpdateCommandDeciPercent(UAVCore->deciThrustPercent);

}

/*******************************************************************
 * 20Hz task (50ms)
 ******************************************************************/
void process20HzTask() {
	if (USE_AIRSPEED_SENSOR) {
		double newAirspeedVms = updateAirspeed();
		airspeed_ms_mean->addValue(newAirspeedVms, currentTime);
	}

	if (AUTOSPEED_CONTROLLER == 1) {
		switch (Firmware) {
		case FIXED_WING:
			// Only if autospeed controller is activated and flight is in cruise state,
			// then adjust thrust regarding the current speed vms
			// Auto-speed control with PID
			if (USE_AIRSPEED_SENSOR) {
				UAVCore->deciThrustPercent = controlSpeedWithThrustV2(currentTime, UAVCore->deciThrustPercent, UAVCore->v_ms_goal, UAVCore->currentAttitude);
			}
			break;
		case QUADCOPTER:
			altitudeHoldController(altitudeBarometer->getAverage(), UAVCore->deciThrustCmd, &(UAVCore->deciThrustPercent));
			break;
		case ROCKET:
			break;
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


#if MEASURE_VIBRATION
	Logger.print("Acc noise vibration = ");
	Logger.print(accNoise);
	Logger.print(" | roll = ");
	Logger.print(UAVCore->currentAttitude->roll);
	Logger.print(" | pitch = ");
	Logger.print(UAVCore->currentAttitude->pitch);
	Logger.print(" | Acc X = ");
	Logger.print((Accel_output[0] - Accel_cal_x)/ACC_LSB_PER_G);

	Logger.print(" | Gdt(0) (ms) = ");
	Logger.println(G_Dt*1000.0);
	
	MPU9150_printMagField();
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

	// Update altimeter
	//------------------------------------------------------------
	updateAltimeter();

	// Update low frequency futaba RF
	//------------------------------------------------------------
	updateRFRadoFutabaLowFreq();
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

	//	schedulerStats();
	/**	Logger.println("--------------------------");
	Logger.print("X1 = ");
	Logger.println(thrustX1);
	Logger.print("X2 = ");
	Logger.println(thrustX2);
	Logger.print("X3 = ");
	Logger.println(thrustX3);
	Logger.print("X4 = ");
	Logger.println(thrustX4);
	Logger.print("sbus[4] = ");
	Logger.println(sBus.channels[4]); */
}



/*******************************************************************
 * 1Hz task 
 ******************************************************************/
void process1HzTask() {

	//---------------------------------------------------------
	// Update barometer temperature for altitude eval
	updateBaroTemperatureLowFreq();


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

	Logger.print("roll cmd = ");
Logger.println(UAVCore->attitudeCommanded->roll);
Logger.print("pitch cmd = ");
Logger.println(UAVCore->attitudeCommanded->pitch);
Logger.print("yaw cmd = ");
Logger.println(UAVCore->attitudeCommanded->yaw);
Logger.print("decithrust cmd = ");
Logger.println(UAVCore->deciThrustPercent);
Logger.print("rubber cmd =");
Logger.println(rubberCmd);
Logger.println("*****************************");


	Logger.print("alt (cm) = ");
	Logger.println(altitudeBarometer->getAverage());
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
		updateCriticalRadio();
	}

	// Update critical RF links
	updateCriticalRFLink();

	// Try to update barometer
	highFreqCheckUpdateAlt();
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
	switch (Firmware) {
	case FIXED_WING:
		Logger.println("Fixed wing firmware");
		break;
	case QUADCOPTER:
		Logger.println("Quadcopter firmware");
		break;
	case ROCKET:
		break;
	}
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

#if Firmware == FIXED_WING
	setupServos() ;
	Logger.println("Servos armed");
#endif


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

	if (USE_RADIO_FUTABA) {
		setupRadioFutaba();
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
