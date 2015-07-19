#ifndef ALTITUDE_CONTROLLER_H_
#define ALTITUDE_CONTROLLER_H_

#include "arch/AVR/MCU/MCU.h"
#include "math/Math.h"
#include "math/PID.h"

#define LEARNING_NB_SAMPLES 10
#define USE_LEARNING_FLIGHT 1

class AltitudeHoldController
{
private:
	float altSetPointCm;
	float output_alt_controller;
	float maxAbsClimbRateMs;
	float maxAbsAccelTarget;
	int maxThrottleHover;
	int deciThrottleHover;
	int deciThrottleHoverSetpoint;
	int maxDeciThrottle;
	long prevTimeMainLoopMs;
	long prevTimeAccel;
	float K_AccelToThrottle;
	float K_ClimbRateToThrottle;
	float errorClimbRateMs;
	PIDe pidClimbRateMs;
	PIDe pidAccelZ;

	// Learning parameters
	int indexThrustHover;
	int deciThrustSamples[LEARNING_NB_SAMPLES];
	bool takeOffDetected;

public:
	AltitudeHoldController()
{
		altSetPointCm = 25.0;
		output_alt_controller = 0.0;
		maxAbsClimbRateMs = 0.6;
		maxAbsAccelTarget = 0.2;
		deciThrottleHover = 540;
		deciThrottleHoverSetpoint= 540;
		maxThrottleHover = 580;
		maxDeciThrottle = 740;
		prevTimeMainLoopMs = 0;
		prevTimeAccel = 0;
		K_AccelToThrottle = 1.0;
		K_ClimbRateToThrottle = 1.0;
		errorClimbRateMs = 0.0;

		pidClimbRateMs.init(30.0, 0.0002, 6.0, 5.0); // old : (6.0, 0.0002, 0.05, 2.0);
		pidClimbRateMs.setUseEnhancePID(false);

		pidAccelZ.init(6.0, 0.01, 3.0, 50); // old : 22, 0.01, 1.5, 60

		indexThrustHover = (int) (LEARNING_NB_SAMPLES / 2.0);
		takeOffDetected = false;

		initLearning();
}

	PIDe* getClimbRatePID() {
		return &pidClimbRateMs;
	}

	/**
	 * Converts altitude error in meters into climb rate error in meters
	 */
	float errorAltitudeToClimbRate(float errorAltitudeMeters)
	{
		// Input/output
		float errorClimbRate = 0.0;
		float kClimb = 0.35;
		float kDescent = 0.2;

		// Compute error climb rate
		if (errorAltitudeMeters >= 0.0) {
			errorClimbRate = kClimb * errorAltitudeMeters;
			BoundAbs(errorClimbRate, maxAbsClimbRateMs);
		}
		else {
			errorClimbRate = kDescent * errorAltitudeMeters;
			Bound(errorClimbRate, -0.3, 0.0);
		}

		return errorAltitudeMeters;
	}

	/**
	 * Update external loop with climb rate and current altitude
	 * Called at 20Hz
	 */
	void update(float climb_rate_ms,  int currentAltCm)
	{
		// Time
		long currentTimeMs = timeMs();
		float dt = 0.05;

		if (prevTimeMainLoopMs > 0) {
			dt = (currentTimeMs - prevTimeMainLoopMs) / 1000.0f;
		}

		//  Errors
		float errorAltitudeMeters = approx((altSetPointCm - currentAltCm)/100.0);
		float targetClimbRateMs = errorAltitudeToClimbRate(errorAltitudeMeters);
		errorClimbRateMs = 0.3 * errorClimbRateMs + 0.7 * (targetClimbRateMs - climb_rate_ms);

		// PID update
		pidClimbRateMs.update(errorClimbRateMs, dt);

		// Update motor output
		output_alt_controller = deciThrottleHover + K_ClimbRateToThrottle * pidClimbRateMs.getOutput();
		Bound(output_alt_controller, 0, maxDeciThrottle);

		// Time
		prevTimeMainLoopMs = currentTimeMs;
	}

	/**
	 * Update internal loop with acceleration in z-earth-frame-axis
	 */
	void update100Hz(float pAccZ_earthframe)
	{
		/**	// Time
		long cTime = timeUs();
		float dt = 0.01;
		if (prevTimeAccel > 0) {
			dt = (cTime - prevTimeAccel) / 1000000.0f;
		}

		// From climb rate PID output to acceleration error
		float accelTargetMs2 = pidClimbRateMs.getOutput();
		BoundAbs(accelTargetMs2, maxAbsAccelTarget * G_MASS);

		float accelError = accelTargetMs2 - (pAccZ_earthframe-1.0) * G_MASS;

		// PID accel update
		pidAccelZ.update(accelError, dt);
		output_alt_controller = deciThrottleHover + K_AccelToThrottle * pidAccelZ.getOutput();

		Bound(output_alt_controller, 0, maxDeciThrottle);

		// Time
		prevTimeAccel = cTime; */
	}

	void reset()
	{
		takeOffDetected = false;
		output_alt_controller = 0;
		pidClimbRateMs.reset();
		pidAccelZ.reset();
	}

	void setAltSetPoint(float newAltCm)
	{
		altSetPointCm = newAltCm;
	}

	float getOutput()
	{
		return output_alt_controller;
	}

	// Learning
	//-------------------------------------------------
	void initLearning()
	{
		for (int i = 0; i < LEARNING_NB_SAMPLES; i ++)
		{
			deciThrustSamples[i] = 0;
		}
	}

	/**
	 * TODO by looking on average value of climb/rate current deci throttle
	 * update throttle hover and PID gains during flight
	 */
	void learnFlyingParameters(bool sonarHealthy, float sonarAltCm, float climbRateMs, int deciThrustCmd)
	{
		// If we  use sonar, and data are healthy
		if (sonarHealthy)
		{
			// Store map climbRate / Throttle
			float dtSample = 0.2;
			float climbRateStart = -LEARNING_NB_SAMPLES * dtSample / 2.0;

			for (int j = 0; j < LEARNING_NB_SAMPLES; j ++)
			{
				float currentClimbRateStart = climbRateStart + j * dtSample;
				float currentClimbRateEnd = currentClimbRateStart + dtSample;

				if (climbRateMs > currentClimbRateStart && climbRateMs < currentClimbRateEnd)
				{
					float currentDeciThrustAverage = deciThrustSamples[j];
					deciThrustSamples[j] = (int) (0.8 * currentDeciThrustAverage + 0.2 * deciThrustCmd);
				}
			}

			// Detect takeoff and save throttle
//			if (takeOffDetected == false)
//			{
//				if (deciThrustCmd > 450 && sonarAltCm > 40.0)
//				{
//					deciThrottleHoverSetpoint = deciThrustCmd * 0.95;
//					Bound(deciThrottleHoverSetpoint, 500, maxThrottleHover)
//					takeOffDetected = true;
//				}
//			}
//			// Update current deci throttle hover
//			else {
//				if (deciThrottleHover < deciThrottleHoverSetpoint) {
//					deciThrottleHover ++;
//				}
//				else if (deciThrottleHover > deciThrottleHoverSetpoint) {
//					deciThrottleHover --;
//				}
//			}

			// Update current deci thrust hover
			//			int deciThrustSetpoint = deciThrustSamples[indexThrustHover];
			//			if (deciThrustSetpoint > deciThrottleHover) {
			//				deciThrottleHover ++;
			//			}
			//			else if (deciThrustSetpoint < deciThrottleHover) {
			//				deciThrottleHover --;
			//			}
			//
			//			Bound(deciThrottleHover, 500, maxThrottleHover);
		}
	}

	int getDeciThrottleHover() {
		return deciThrottleHover;
	}

	char* learningToPacket()
	{
		char buf[95];
		// Packet header
		sprintf(buf, "%s", "learn");

		// Packet payload
		for (int i = 0; i < LEARNING_NB_SAMPLES; i ++)
		{
			sprintf(buf, "%s|%d",
					buf,
					deciThrustSamples[i]);
		}

		// Packet footer
		sprintf(buf, "%s|\n", buf);

		return buf;
	}
};

AltitudeHoldController altHoldCtrl;

#endif
