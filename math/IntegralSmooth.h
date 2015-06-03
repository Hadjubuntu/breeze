/*
 * IntegralSmooth.h
 *
 *  Created on: Jun 3, 2015
 *      Author: adrien
 */

#ifndef MATH_INTEGRALSMOOTH_H_
#define MATH_INTEGRALSMOOTH_H_

#include "Common.h"

class IntegralSmooth
{
private:
	int updateFreqHz;
	float maxIter;
	int nbIterSameSign;
	float output;
	float alphaSmooth;
	bool lastValPositive;
public:
	IntegralSmooth(float pAlphaSmooth, int pUpdateFreqHz) {
		alphaSmooth = pAlphaSmooth;
		updateFreqHz = pUpdateFreqHz;
		maxIter = (float) pUpdateFreqHz;
		nbIterSameSign = 0;
		output = 0.0;
		lastValPositive = true;
	}

	void update(double e, double dt)
	{
		output = alphaSmooth * output + (nbIterSameSign / maxIter) * e * dt;

		if (output >= 0.0)
		{
			if (lastValPositive) {
				nbIterSameSign ++;
			}
			else {
				nbIterSameSign = 1;
			}
			lastValPositive = true;
		}
		else {
			if (lastValPositive == false) {
				nbIterSameSign ++;
			}
			else {
				nbIterSameSign = 1;
			}

			lastValPositive = false;
		}

		Bound(nbIterSameSign, 0, maxIter);
	}

	float getOutput()
	{
		return output;
	}
};


#endif /* MATH_INTEGRALSMOOTH_H_ */
