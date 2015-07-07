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
#include "arch/AVR/wire/Wire.h"
#include "arch/AVR/MCU/MCU.h"
#include "peripherals/altimeter/Sensor_AltimeterBMP085.h"
#include "peripherals/range_detector/Sensor_AnalogSonar.h"
#include "modules/AHRS/AHRS_Kalman.h"
#include "peripherals/IMU/Sensor_IMU.h"
#include "math/IntegralSmooth.h"
#include "modules/AHRS/InertialNav.h"


// Inertial naviguation
InertialNav insNav;

// Skeleton functions
void measureCriticalSensors();

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

	UAVCore->currentAttitude->yaw = toDeg(gyroZangle);

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
				sBus.failsafe_status != SBUS_SIGNAL_OK) 
		{
			sBus.incrLostCom();

			if (sBus.isComLost()) 
			{
				// failsafe
				UAVCore->deciThrustPercent = 0;
				UAVCore->autopilot = false;
				AUTOSPEED_CONTROLLER = 0;
			}
		}
		else {
			// Reset communication losted
			sBus.resetLostCom();

			UAVCore->attitudeCommanded->roll = (sBus.channels[0]-sBus.channelsCalib[0])*0.0682;
			UAVCore->attitudeCommanded->pitch = (sBus.channels[1]-sBus.channelsCalib[1])*0.0682;
			double yawRate = (sBus.channels[3]-sBus.channelsCalib[3])*0.0682;

			float sbus_throttle = max((sBus.channels[2]-365)/1.38, 0);
			if (AUTOSPEED_CONTROLLER == 0) {
				UAVCore->deciThrustPercent = sbus_throttle;
			}

			// Decrease command angle in quadcopter mode
			if (Firmware == QUADCOPTER) {
				double factorSmooth = 0.35; // 0.43 is good
				UAVCore->attitudeCommanded->roll = UAVCore->attitudeCommanded->roll * factorSmooth; 
				UAVCore->attitudeCommanded->pitch = UAVCore->attitudeCommanded->pitch * factorSmooth;
				yawRate = yawRate * factorSmooth;

				if (abs(yawRate) < 1.0) {
					yawRate = 0.0;
				}
				UAVCore->attitudeCommanded->yaw = 0.99 * UAVCore->attitudeCommanded->yaw + yawRate * 0.07;
				double yawCmdRad = toRad(UAVCore->attitudeCommanded->yaw);
				NormRadAngle(yawCmdRad);
				UAVCore->attitudeCommanded->yaw = toDeg(yawCmdRad);

				BoundAbs(UAVCore->attitudeCommanded->roll, 15);
				BoundAbs(UAVCore->attitudeCommanded->pitch, 15);
			}
		}
	}
}

float altitudeSetPointCm = 0.0;

// Update values with optionnal channels
//------------------------------------------------
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
				altHoldCtrl.reset();
				altHoldCtrl.setAltSetPoint(altitudeSetPointCm); // current altitude + x cm above
			}
		}

		// PID tuning with potentiometer on the radio
		//------------------------------------------
		double factor = (sBus.channels[5]-368.0) / (1984.0-368.0) * 3.0; // New PID max 4.0
		param[ID_G_P_ROLL] =  factor;
		param[ID_G_D_ROLL] = factor * 0.01;

		param[ID_G_P_PITCH] = factor;
		param[ID_G_D_PITCH] = factor * 0.01;


		param[ID_G_I_PITCH] = 0.1; // Constante integral value
		param[ID_G_I_ROLL] = 0.1;

		PIDe *pidClimbRateMs = altHoldCtrl.getClimbRatePID();

		if (sBus.channels[7] > 1300) {
			pidClimbRateMs->setGainParameters(50.0, 0.002, 12.0);
			pidClimbRateMs->setMaxI(10.0);
		}
		else if (sBus.channels[7] > 400) {
			pidClimbRateMs->setGainParameters(25.0, 0.0002, 10.0);
			pidClimbRateMs->setMaxI(12.5);
		}
		else {
			pidClimbRateMs->setGainParameters(10.0, 0.0002, 6.0);
			pidClimbRateMs->setMaxI(20.0);
		}

		// Get yaw helper by default
		YAW_HELPER = 1;

		//		if (sBus.channels[6] > 1000) {
		//			YAW_HELPER = 1;
		//		}
		//		else {
		//			YAW_HELPER = 0;
		//		}

		if (sBus.channels[6] > 1000) {
			altitudeSetPointCm = 250.0;
			altHoldCtrl.setAltSetPoint(altitudeSetPointCm);
		}
		else {
			altitudeSetPointCm = 150.0;
			altHoldCtrl.setAltSetPoint(altitudeSetPointCm);
		}

		// Update PID parameters
		PID_roll.setGainParameters(param[ID_G_P_ROLL], param[ID_G_D_ROLL], param[ID_G_I_ROLL]);
		PID_pitch.setGainParameters(param[ID_G_P_PITCH], param[ID_G_D_PITCH], param[ID_G_I_PITCH]);

		if (Firmware == FIXED_WING) {
			// Flaps TODO find a channel for flaps
			//------------------------------------------
			switch (sBus.channels[8]) {
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
	if (UAVCore->autopilot || (UAVCore->autopilot == false && rf_manual_StabilizedFlight == 1)) 
	{
		stabilize2(G_Dt, UAVCore->currentAttitude, 
				UAVCore->attitudeCommanded,
				&aileronCmd, &gouvernCmd, &rubberCmd,
				UAVCore->gyroXrate, UAVCore->gyroYrate,
				UAVCore->deciThrustPercent);
	}


	//-----------------------------------------------
	// Process and order all commands
	processCommand() ; 

	// Update altitude hold controller for quadcopter
	if (Firmware == QUADCOPTER) {
		if (AUTOSPEED_CONTROLLER == 1) {
			altHoldCtrl.update100Hz(insNav.getAccZ_ef());
		}
	}

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
		// If angle are not so aggressive and thrust over a minimum of 10%
		// then define boost to motors tilt compensation
		if (UAVCore->deciThrustPercent > 100
				&& abs(UAVCore->currentAttitude->roll) < 45.0
				&& abs(UAVCore->currentAttitude->pitch < 45.0)) {
			float cos_roll = fast_cos(toRad(UAVCore->currentAttitude->roll));
			float cos_pitch = fast_cos(toRad(UAVCore->currentAttitude->pitch));
			float cos_tilt = cos_roll * cos_pitch;

			boost = 1.0f / cos_tilt;
			Bound(boost, 1.0, 1.5);
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


	// Update inertial naviguation
	//----------------------------------
	insNav.update50Hz(currentTime, sonarHealthy, sonarAltCm);

	// Update acceleration noise measurement
	//----------------------------------
	if (imu.measureVibration()) {
		// Measure vibration
		accNoise = sqrt(pow2(insNav.getAccX_ef()) + pow2(insNav.getAccY_ef()) + pow2(insNav.getAccZ_ef()));
	}
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
			// Altitude controller
			altHoldCtrl.update(insNav.getClimbRateMs(), altCF);
			UAVCore->deciThrustPercent = altHoldCtrl.getOutput(); 
			break;
		case ROCKET:
			break;
		}
	}

	//-------------------------------------------
	// Update altimeter
	updateAltimeter(sonarHealthy, sonarAltCm, insNav.getClimbRateMs(), insNav.getAccZ_ef());

	//------------------------------------------
	// Altitude controller learning parameters
	altHoldCtrl.learnFlyingParameters(sonarHealthy, sonarAltCm, insNav.getClimbRateMs(), UAVCore->deciThrustPercent);
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

	// Update sonar if used
	//------------------------------------------------------------
	if (USE_SONAR_ALT) {
		updateSonar(currentTime);
	}
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


	//if (imu.measureVibration()) {
	//	Logger.print("Acc_noise= ");
	//	Logger.print(accNoise);
	//	Logger.print(" | accel_roll = ");
	//	Logger.print(Accel_roll);
	//	Logger.print(" | error = ");
	//	Logger.println(UAVCore->currentAttitude->roll - Accel_roll);
	//	Logger.print("P = ");
	//	Logger.print(kalX.getP00());
	//	Logger.print(" | ");
	//	Logger.println(kalX.getP11());
	//}

//	char buf[60];
//	// Packet header
//	sprintf(buf, "%s", "accnoise");
//
//	// Packet payload	
//	sprintf(buf, "%s|%d",
//			buf,
//			toCenti(accNoise));
//	// Packet footer
//	sprintf(buf, "%s|\n", buf);
//	updateLearningData(buf);
}



/*******************************************************************
 * 2Hz task (500ms)
 ******************************************************************/
void process2HzTask() {
//	Logger.println(insNav.getClimbRateMs());
	/*Logger.print("Airspeed : ");
	Logger.print(airspeed_ms_mean->getAverage());
	Logger.println(" m/s");
	 */

	//	schedulerStats(); 
	//		Logger.println("--------------------------");
	//				Logger.print("X1 = ");
	//				Logger.println(thrustX1);
	//				Logger.print("X2 = ");
	//				Logger.println(thrustX2);
	//				Logger.print("X3 = ");
	//				Logger.println(thrustX3);
	//	Logger.print("X4 = ");
	//	Logger.println(thrustX4); 
}



/*******************************************************************
 * 1Hz task 
 ******************************************************************/
void process1HzTask() {

	// Send learning data to GCS
	//	updateLearningData(altHoldCtrl.learningToPacket());

	//---------------------------------------------------------
	// Send data to the ground station
	// Current position if GPS used : currentPosition

	//	updateRFLink1hz(toCenti(UAVCore->currentAttitude->roll), toCenti(UAVCore->currentAttitude->pitch), 
	//			(int)(UAVCore->currentAttitude->yaw),
	//			(int)(altCF), toCenti(airspeed_ms_mean->getAverage()),
	//			currentPosition.lat, currentPosition.lon,
	//			(int)angleDiff, UAVCore->autopilot, UAVCore->deciThrustPercent);


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
	initAHRS(imu.getInitRoll(), imu.getInitPitch());
	Logger.println("Gyro armed");

	setupAltimeter();
	Logger.println("Altimeter armed");

	if (Firmware == FIXED_WING) {
		setupServos() ;
		Logger.println("Servos armed");
	}


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

	if (USE_SONAR_ALT) {
		setupSonar();
		Logger.println("Ultrasonic sonar armed");
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

	// Wait 2 seconds before starting
	delay(2000) ;
}



/*******************************************************************
 * Main loop funtions
 ******************************************************************/
void loop () 
{
	// Force shutdown autopilot if radio doesn't work for safety
	if (radio_linked_checked == false) {
		AUTOSPEED_CONTROLLER = 0;
		UAVCore->autopilot = false;
	}

	// Get current time in microseconds 
	currentTime = timeUs();

	// Delta time since warmup
	deltaTime = currentTime - previousTime;

	// Compute critical functions
	measureCriticalSensors();

	// Run scheduler
	schedulerRun();
}
