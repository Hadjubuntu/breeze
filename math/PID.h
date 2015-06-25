/*
 * PID.h
 *
 * Generic PID implementation simple non-linear model with a K(t) = Kp * Ke(t) depending on error
 *
 *  Created on: 9 august 2014
 *      Author: hadjmoody
 */

#ifndef PID_H_
#define PID_H_

#include "Common.h"

// TODO create Kp(t) = K0*Ke(t) with right parameters

class PIDe {
protected:
	float Kp;
	float Ki;
	float Kd;
	float maxI;
	float i;
	float d;
	float alpha_d;
	float error;
	float prevError;
	float output;

	float Ke;

	// See papers : http://downloads.deusm.com/designnews/1477-Elsevier06.pdf
	bool useEnhancePID;
	float Kboost;
	float Kboost_max;
public:
	PIDe();
	void init(float pKp, float pKi, float pKd, float pMaxI);
	void setGainParameters(float pKp, float pKi, float pKd);
	void update(float error, float dtSeconds);
	float getOutput();
	void reset();
	void printGains();
	float getI();
	float getError();
};

PIDe::PIDe()
{
	init(1.0, 0.1, 0.01, 10);
}

void PIDe::init(float pKp, float pKd, float pKi, float pMaxI) {
	Kp = pKp;
	Ki = pKi;
	Kd = pKd;
	maxI = pMaxI;
	i = 0.0;
	d = 0.0;
	alpha_d = 0.6;
	prevError = 0.0;
	output = 0.0;
	useEnhancePID = true;
	Kboost = 0.0125;
	Kboost_max = 2.0;
	Ke = 1.0;
}

void PIDe::setGainParameters(float pKp, float pKd, float pKi)
{
	Kp = pKp;
	Ki = pKi;
	Kd = pKd;
}


void PIDe::reset()
{
	i = 0.0;
	d = 0.0;
	prevError = 0.0;
}

void PIDe::update(float e, float dtSeconds)
{
	error = e;

	// Evaluate differential
	float dError = 0.0;
	if (dtSeconds > 0.0) {
		dError = (error - prevError) / dtSeconds;
	}
	prevError = error;

	// Evaluate integral
	i = i + error * dtSeconds;
	BoundAbs(i, maxI);

	d = (1.0 - alpha_d) * d + alpha_d * dError;


	if (useEnhancePID)
	{
		Ke = (exp(Kboost * error) + exp(-Kboost * error)) / 2.0;
		Bound(Ke, 1.0, Kboost_max);
	}

	// Computes output
	output = Ke * (Kp * error + Ki * i + Kd * d);
}

float PIDe::getOutput()
{
	return output;
}

float PIDe::getI() {
	return i;
}

float PIDe::getError() {
	return error;
}

void PIDe::printGains()
{
	Logger.print("Kp = ");
	Logger.println(Kp);
	delay(10);
	Logger.print("Kd = ");
	Logger.println(Kd);
	delay(10);
	Logger.print("Ki = ");
	Logger.println(Ki);
	delay(10);
	Logger.print("Ke(t) = ");
	Logger.println(Ke);
	delay(10);
}


#endif /* PID_H_ */
