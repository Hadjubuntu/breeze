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
	int deciThrottleHover;
	int maxDeciThrottle;
	long prevTimeClimbRate;
	long prevTimeAccel;
	int K_AccelToThrottle;
	float errorClimbRateMs;
	PIDe pidClimbRateMs;
	PIDe pidAccelZ;

	// Learning parameters
	int deciThrustSamples[LEARNING_NB_SAMPLES];

public:
	AltitudeHoldController()
{
		altSetPointCm = 25.0;
		output_alt_controller = 0.0;
		maxAbsClimbRateMs = 0.8;
		maxAbsAccelTarget = 0.25;
		deciThrottleHover = 550;
		maxDeciThrottle = 660;
		prevTimeClimbRate = 0;
		prevTimeAccel = 0;
		K_AccelToThrottle = 1.0;
		errorClimbRateMs = 0.0;

		pidClimbRateMs.init(6.0, 0.0002, 0.05, 2.0);
		pidAccelZ.init(24.0, 0.01, 1.0, 90); // old : 22, 0.01, 1.5, 60

		initLearning();
}

	float errorAltitudeToClimbRate(float errorAltitudeMeters)
	{
		// Input/output
		float errorClimbRate = 0.0;
		float k = 1.0;

		// Compute error climb rate
		errorClimbRate = k * errorAltitudeMeters;
		BoundAbs(errorClimbRate, maxAbsClimbRateMs);

		return errorAltitudeMeters;
	}

	/**
	 * Update external loop with climb rate and current altitude
	 */
	void update(float climb_rate_ms,  int currentAltCm)
	{
		// Time
		long cTime = timeUs();
		float dt = 0.05;
		if (prevTimeClimbRate > 0) {
			dt = (cTime - prevTimeClimbRate) / 1000000.0f;
		}

		//  Errors
		float errorAltitudeMeters = approx((altSetPointCm - currentAltCm)/100.0);
		float targetClimbRateMs = errorAltitudeToClimbRate(errorAltitudeMeters);
		errorClimbRateMs = 0.5 * errorClimbRateMs + 0.5 * (targetClimbRateMs - climb_rate_ms);

		// PID update
		pidClimbRateMs.update(errorClimbRateMs, dt);

		// Time
		prevTimeClimbRate = cTime;
	}

	/**
	 * Update internal loop with acceleration in z-earth-frame-axis
	 */
	void update100Hz(float pAccZ_earthframe)
	{
		// Time
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
		prevTimeAccel = cTime;
	}

	void reset()
	{
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
		}
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
