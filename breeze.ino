/**
 * Breeze Project
 * ---------------------------------------------------------
 * Breeze is an open-source autopilot for UAV.
 * ---------------------------------------------------------
 * 2014 (c) Adrien HADJ-SALAH
 * 
 * First succesful flight : 03/12/2014
 */


#include "Breeze.h"
#include "modules/scheduler/Scheduler.h"
#include "modules/scenario/Scenario.h"
#include "arch/AVR/wire/Wire.h"
#include "arch/AVR/MCU/MCU.h"
#include "peripherals/altimeter/Sensor_AltimeterBMP085.h"
#include "modules/AHRS/AHRS_Kalman.h"


// Skeleton functions
void measureCriticalSensors();
void updateClimbRate();

/*******************************************************************
 * Update attitude (angle and angle rate)
 ******************************************************************/
void updateAttitude() {  
	// Update IMU data
	updateGyroData();

	// Update AHRS
	updateAHRS(Accel_roll, gyroXrate, Accel_pitch, gyroYrate, dt_IMU);

	// Set new current attitude filtered by Kalman
	UAVCore->currentAttitude->roll = kalX.getOutput();
	UAVCore->currentAttitude->pitch = kalY.getOutput();

	// Update heading with GPS data
	if (USE_GPS_NAVIGUATION) {
		UAVCore->headingCap = currentHeading;
	}

	UAVCore->currentAttitude->yaw = 0.0;

	// Update angle rate
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
		if (timeUs() - sBus.lastUpdateUs > SBUS_SIGNAL_LOST_DELAY_US ||
				sBus.failsafe_status != SBUS_SIGNAL_OK) {
			// failsafe
			UAVCore->deciThrustPercent = 0;
			UAVCore->deciThrustCmd = 0;
			UAVCore->autopilot = false;
			AUTOSPEED_CONTROLLER = 0;
		}
		else {
			UAVCore->attitudeCommanded->roll = (sBus.channels[0]-sBus.channelsCalib[0])*0.0682;
			UAVCore->attitudeCommanded->pitch = (sBus.channels[1]-sBus.channelsCalib[1])*0.0682;
			UAVCore->attitudeCommanded->yaw = (sBus.channels[3]-sBus.channelsCalib[3])*0.0682;

			float sbus_throttle = max((sBus.channels[2]-365)/1.38, 0);
			if (AUTOSPEED_CONTROLLER == 0) {
				UAVCore->deciThrustPercent = sbus_throttle;
			}

			UAVCore->deciThrustCmd = sbus_throttle;

			// Decrease command angle in quadcopter mode
			if (Firmware == QUADCOPTER) {
				UAVCore->attitudeCommanded->roll = UAVCore->attitudeCommanded->roll * 0.43; // 0.45
				UAVCore->attitudeCommanded->pitch = UAVCore->attitudeCommanded->pitch * 0.43;
				UAVCore->attitudeCommanded->yaw = UAVCore->attitudeCommanded->yaw * 0.45;
			}
		}
	}
}

double Ki = 0.0;

void updateRFRadoFutabaLowFreq() {
	if (USE_RADIO_FUTABA == 1) {

		if (sBus.failsafe_status != SBUS_SIGNAL_OK) {
			AUTOSPEED_CONTROLLER = 0;
		}
		else {
			// Autopilot switch
			//------------------------------------------
			// LOW means manual
			UAVCore->autopilot = false;
			int old_state = AUTOSPEED_CONTROLLER;
			if (sBus.channels[SBUS_AUTO_CHANNEL] < 1000) {
				// TODO remettre autopilot, test
				// UAVCore->autopilot = false;
				AUTOSPEED_CONTROLLER = 0;
			}
			// HIGH means auto
			else {
				// UAVCore->autopilot = true;
				AUTOSPEED_CONTROLLER = 1;
			}

			if (AUTOSPEED_CONTROLLER != old_state) {
				output_alt_controller = 0.0;
				altSetPointCm = altCF + 80.0; // current altitude + 100 cm over
			}
		}

		// PID tuning
		//------------------------------------------

		double factor = (sBus.channels[5]-368.0) / (1984.0-368.0) * 3.0; // quad : 1.0
		param[ID_G_P_ROLL] =  factor;
		param[ID_G_D_ROLL] = factor * 0.01 * 0.1;
		param[ID_G_I_ROLL] = Ki;

		param[ID_G_P_PITCH] = factor;
		param[ID_G_D_PITCH] = factor * 0.01 * 0.1;
		param[ID_G_I_PITCH] = Ki;

		if (sBus.channels[7] > 1300) {
			Ki = 0.1; // 0.1
		}
		else if (sBus.channels[7] > 400) {
			Ki = 0.05; // 0.05
		}
		else {
			Ki = 0.0;
		}


		//		// Flaps
		//		//------------------------------------------
		//		switch (sBus.channels[7]) {
		//		case 144:
		//			flapsCmd = 0;
		//			break;
		//		case 1024:
		//			flapsCmd = 50;
		//			break;
		//		case 1904:
		//			flapsCmd = 90;
		//			break;
		//		default:
		//			flapsCmd = 0;
		//			break;
		//		}
	}
}


/*******************************************************************
 * 100Hz task (each 10 ms)
 ******************************************************************/
void process100HzTask() {
	G_Dt = (currentTime - hundredHZpreviousTime) / S_TO_US;

	// Update attitude from gyro
	updateAttitude() ;


	// If UAV in auto mode
	// Define new command (roll, pitch, yaw, thrust) by using PID 
	// To reach roll pitch and yaw desired
	if (UAVCore->autopilot || (UAVCore->autopilot == false && rf_manual_StabilizedFlight == 1)) {
		stabilize2(
				G_Dt, UAVCore->currentAttitude, 
				UAVCore->attitudeCommanded,
				&aileronCmd, &gouvernCmd, &rubberCmd,
				UAVCore->gyroXrate, UAVCore->gyroYrate,
				UAVCore->deciThrustPercent);
	}


	//-----------------------------------------------
	// Process and order all commands
	processCommand() ; 

	hundredHZpreviousTime = currentTime;
}


/*******************************************************************
 * 50Hz task (20 ms)
 ******************************************************************/
void process50HzTask() {

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

	// TODO remove flight mode, use mission instead

	// Update RF data down and up
	//------------------------------------------------------------
	updateRFLink50Hz() ;
	updateRFRadioFutaba();



	// Motor update
	// Command motor at % thrust
	//------------------------------------------------------------
	double boost = 1.0;

	if (Firmware == QUADCOPTER) {
		// If angle are not so aggressive and thrust over a minimum of 25%
		// then define boost to motors tilt compensation
		if (UAVCore->deciThrustPercent > 250
				&& abs(UAVCore->currentAttitude->roll) < 45.0
				&& abs(UAVCore->currentAttitude->pitch < 45.0)) {
			float cos_roll = fast_cos(toRad(UAVCore->currentAttitude->roll));
			float cos_pitch = fast_cos(toRad(UAVCore->currentAttitude->pitch));
			float cos_tilt = cos_roll * cos_pitch;

			boost = 1.0f / cos_tilt;
			Bound(boost, 1.0, 1.35);
		}
	}
	motorUpdateCommandDeciPercent(boost, UAVCore->deciThrustPercent);


	switch (Firmware) {
	case FIXED_WING:
		motorUpdateCommandFixedWing();
		break;
	case QUADCOPTER:
		motorUpdateCommandQuad();
		break;
	default:
		// Do nothing
		break;
	}


	updateClimbRate();
}


void updateClimbRate() {
	//----------------------------------------------
	// Update acceleration on z-axis in earth-frame
	Vector3f vect_acc_bf;
	vect_acc_bf.x = rel_accX;
	vect_acc_bf.y = rel_accY;
	vect_acc_bf.z = rel_accZ;
	Vector3f vect_acc_ef = rot_bf_ef(vect_acc_bf, UAVCore->currentAttitude);

	if (acc_z_initialized == false) {
		initial_acc_z_bias = (vect_acc_ef.z-1.0);
		acc_z_initialized = true;
	}

	acc_z_on_efz = vect_acc_ef.z - initial_acc_z_bias;

	// Force climb rate to be around 0.0 with 0.9 gain
	// climb rate = K ( previous + new_acc * dt) with dt = 0.02
	climb_rate = 0.9*(climb_rate + G_MASS *(acc_z_on_efz-1.0) * 0.02);

#if MEASURE_VIBRATION
	// Measure vibration
	accNoise = sqrt(pow2(vect_acc_ef.x) + pow2(vect_acc_ef.y) + pow2(vect_acc_ef.z));
#endif
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
			// Update climb rate
			// climb_rate = acc_filter->getFullstackIntegral(0.05);

			// Altitude controller
			altitudeHoldController(climb_rate, altCF, UAVCore->deciThrustCmd);
			UAVCore->deciThrustPercent = output_alt_controller; 
			break;
		case ROCKET:
			break;
		}
	}

	//-------------------------------------------
	// Update altimeter
	updateAltimeter(acc_z_on_efz);
}



/*******************************************************************
 * 10Hz task (100ms)
 ******************************************************************/
void process10HzTask() {
	// Update low priority rf com
	//------------------------------------------------------------
	updateLowPriorityRFLink();

	// Update compass data (doesn't work)
	// TODO find ASA calib by calling magnometer for ADC precision
	//------------------------------------------------------------
	//		updateCompassData();
	//		double heading = getCompassHeading(UAVCore->currentAttitude);
	//		Logger.print("heading (deg) = ");
	//		Logger.println(heading);

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

	// Update low frequency futaba RF
	//------------------------------------------------------------
	updateRFRadoFutabaLowFreq();


	//#if MEASURE_VIBRATION
	//	Logger.print("Acc_noise= ");
	//	Logger.print(accNoise);
	//	Logger.print(" | roll= ");
	//	Logger.print(UAVCore->currentAttitude->roll);
	//	Logger.print(" | accel_roll = ");
	//	Logger.print(Accel_roll);
	//	Logger.print(" | error = ");
	//	Logger.println(UAVCore->currentAttitude->roll - Accel_roll);
	//	Logger.print("P = ");
	//	Logger.print(kalX.getP00());
	//	Logger.print(" | ");
	//	Logger.println(kalX.getP11());
	//#endif	
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
	//		Logger.println("--------------------------");
	//			Logger.print("X1 = ");
	//			Logger.println(thrustX1);
	//			Logger.print("X2 = ");
	//			Logger.println(thrustX2);
	//			Logger.print("X3 = ");
	//			Logger.println(thrustX3);
	//	Logger.print("X4 = ");
	//	Logger.println(thrustX4); 

	/**
	Logger.print("sbus[4] = ");
	Logger.println(sBus.channels[4]); 

	Logger.print("sbus[5] = ");
		Logger.println(sBus.channels[5]); */
	//	Logger.print("Gyro pitch rate (deg) = ");
	//	Logger.println(- gyroYrate * ATTITUDE_CONTROL_DEG);
	//	Logger.print("setpoint pitch rate (deg) = ");
	//	Logger.println((UAVCore->attitudeCommanded->pitch - UAVCore->currentAttitude->pitch) * 1.5);
	//		Logger.print("Out roll (cmd) = ");
	//		Logger.println((UAVCore->attitudeCommanded->roll - UAVCore->currentAttitude->roll) * 4.5 - gyroXrate * ATTITUDE_CONTROL_DEG);

	//		Logger.print("sum error roll = ");
	//	Logger.println(sumErrorRoll);	
	//	Logger.print("sum error pitch = ");
	//	Logger.println(sumErrorPitch);

	//	Logger.println(Ki);

	//		Logger.print("x_rate = ");
	//		Logger.println(gyroXrate * ATTITUDE_CONTROL_DEG);
	//		Logger.print("roll = ");
	//		Logger.println(UAVCore->currentAttitude->roll);
	//	

	//			Logger.print("y_rate = ");
	//			Logger.println(gyroYrate * ATTITUDE_CONTROL_DEG);
	//				Logger.print("pitch = ");
	//				Logger.println(UAVCore->currentAttitude->pitch);


	//	Logger.print("acc_z hard = ");
	//	Logger.println(rel_accZ * (1.0 + abs(sin_pitch) * abs(sin_pitch)
	//			+ abs(sin_roll) * abs(sin_roll))); 

	//
	//		Logger.print("pitch = ");
	//		Logger.println(UAVCore->currentAttitude->pitch);


	//	Logger.print("roll cmd  = ");
	//	Logger.println(UAVCore->attitudeCommanded->roll);
	//
	//	Logger.print("pitch cmd  = ");
	//	Logger.println(UAVCore->attitudeCommanded->pitch);

	//			Logger.print("Gyro_z_rate = ");
	//			Logger.println(gyroZrate * ATTITUDE_CONTROL_DEG);
	//	Logger.print("Yaw desired = ");
	//	Logger.println(UAVCore->attitudeCommanded->yaw);

	//	Vector3f t;
	//	t.x = 1;
	//	t.y = 0;
	//	t.z = 1;
	//	Vector3f t_bf = rot_ef_bf(t, UAVCore->currentAttitude);
	//	Logger.println(t_bf.x);
	//		Logger.print("Climb_rate (cm/s) = ");
	//		Logger.println(climb_rate*100.0);
	
//	Logger.print("gyroZangle = ");
//	Logger.println(toDeg(gyroZangle));
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
			(int)angleDiff, UAVCore->autopilot, toCenti(accNoise));

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
	initAHRS(init_roll, init_pitch);
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
		radio_linked_checked = setupRadioFutaba();
	}
	else {
		radio_linked_checked = true;
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
	if (radio_linked_checked == false) {
		AUTOSPEED_CONTROLLER = 0;
		UAVCore->autopilot = false;
	}
	currentTime = timeUs();
	deltaTime = currentTime - previousTime;

	measureCriticalSensors();
	schedulerRun();

}
