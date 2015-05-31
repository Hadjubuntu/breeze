#ifndef ALTITUDE_CONTROLLER_H_
#define ALTITUDE_CONTROLLER_H_

#include "arch/AVR/MCU/MCU.h"
#include "math/Math.h"
#include "math/PID.h"

class AltitudeHoldController
{
private:
	float altSetPointCm;
	float output_alt_controller;
	float maxAbsClimbRateMs;
	int deciThrottleHover;
	int maxDeciThrottle;
	long prevTimeUs;
	int K_climbRateToDeciThrottle;
	PIDe pidAltCtrl;

public:
	AltitudeHoldController()
	{
		altSetPointCm = 25.0;
		output_alt_controller = 0.0;
		maxAbsClimbRateMs = 0.6;
		deciThrottleHover = 600;
		maxDeciThrottle = 700;
		prevTimeUs = 0;
		K_climbRateToDeciThrottle = 50;

		pidAltCtrl.init(1.0, 0.1, 0.1, 200);
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
	 * Altitude Hold Controller aims to keep UAV to level
	 */
	void update(float climb_rate_ms, int currentAltCm, int deciThrustCmd)
	{
		// Time
		long cTime = timeUs();
		float dt = (cTime - prevTimeUs) / 1000000.0;

		// Errors
		float errorAltitudeMeters = approx((altSetPointCm - currentAltCm)/100.0);
		float errorClimbRateMs = errorAltitudeToClimbRate(errorAltitudeMeters);

		// PID update
		pidAltCtrl.update(errorClimbRateMs, dt);

		output_alt_controller =  deciThrottleHover + K_climbRateToDeciThrottle * pidAltCtrl.getOutput();
		Bound(output_alt_controller, 0, maxDeciThrottle);

		// Time
		prevTimeUs = cTime;
	}

	void reset()
	{
		output_alt_controller = 0;
	}

	void setAltSetPoint(float newAltCm)
	{
		altSetPointCm = newAltCm;
	}

	float getOutput()
	{
		return output_alt_controller;
	}

	/**
	 * TODO by looking on average value of climb/rate current deci throttle
	 * update throttle hover and PID gains during flight
	 */
	void learnFlyingParameters()
	{

	}
};

AltitudeHoldController altHoldCtrl;

#endif
