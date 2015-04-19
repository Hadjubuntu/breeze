/**
 * FilterAverage is a class which aims to filter the data to have a smooth output through times.
 * Main functions are :
 *
 * addValue(value, timeUs) Add a new value
 * areDataRevelant(time) Are the data up-to-date compare to a time in us
 * getAverage() Get the smooth output
 * getDerivative() Get the smooth derivate of values
 * reset() Restart values
 *
 * @author : Adrien Hadj-Salah
 */

#ifndef FILTERAVERAGE_H
#define FILTERAVERAGE_H


#include "Common.h"

/*
 * FilterAverage.h

 TODO do all times with unsigned long ...
 *
 *  Created on: 25 august 2014
 *      Author: Arien Hadj-Salah
 */
#define NB_SAMPLING 30
#define MAX_TIME_ELAPSED_RELEVANT_DATA 10*S_TO_US // 10 seconds

long absolute(long v) {
	if (v < 0) {
		v = -v;
	}
	return v;
}

class FilterAverage {
public:
	FilterAverage(int pFilterLength, int pMinFilterLowPass, int pMaxFilterHighPass, bool pWeightedMode) {
		filterLength = pFilterLength;
		minFilterLowPass = pMinFilterLowPass;
		maxFilterHighPass = pMaxFilterHighPass;
		weightedMode = pWeightedMode;

		reset();

		// Prevent from length too long
		if (filterLength > NB_SAMPLING) {
			filterLength = NB_SAMPLING;
		}
	};

	void addValue(double pValue, long pTimeUpdate) {
		// Pick up last index to check if the last insertion was before this proposed element
		int previousIndexValue = 0;
		if (index > 0) {
			previousIndexValue = index - 1;
		}
		else if (index == 0 && hasCycle) {
			previousIndexValue = NB_SAMPLING - 1;
		}

		if (pTimeUpdate > timeUpdate[previousIndexValue]) {
			Bound(pValue, minFilterLowPass, maxFilterHighPass);

			values[index] = pValue;
			timeUpdate[index] = pTimeUpdate;
			index++;

			if (index >= NB_SAMPLING) {
				index = 0;
				hasCycle = true;
			}

		}
	};

	void reset() {
		index = 0;
		hasCycle = false;
		for (int i=0; i < NB_SAMPLING; i++) {
			values[i] = 0.0;
			timeUpdate[i] = 0;
		}
	};

	/**
	 * Evaluate mean value of the filter
	 */
	double getAverage() {
		// If no values return zero
		if (index==0 && !hasCycle) {
			return 0.0;
		}

		// Otherwise prepare variables
		double average = 0.0;
		double nSum = 0.0;
		double cWeight = 1;
		int iStart = 0;
		int meanLength = filterLength;
		if (!hasCycle && meanLength > index) {
			meanLength = index;
		}

		// Then add data
		if (index > meanLength) {
			iStart = index-meanLength;
		}
		for (int j=iStart; j < index; j ++) {
			if (weightedMode) {
				cWeight = 1.0 / (index-j);
			}

			average += cWeight * values[j];
			nSum += cWeight;

			// DEBUG: printf("j=%d | index=%d | cWeight=%f | val=%f\n", j, index, cWeight, values[j]);
		}

		if (hasCycle && meanLength > index) {
			for (int k=NB_SAMPLING-1; k > (NB_SAMPLING-(meanLength-index+1)); k--) {

				if (weightedMode) {
					cWeight = 1.0/(index + NB_SAMPLING-k);
				}

				average += cWeight * values[k];
				nSum += cWeight;
			}
		}

		return average / nSum;
	}

	double getFullstackIntegral(double defaultDtSeconds) {
		double output = 0.0;
		double dt = defaultDtSeconds;
		long t_max = 0;
		long t_min = 0;

		for (int i=0; i < NB_SAMPLING; i ++) {
			if (i > 0) {
				dt = (timeUpdate[i]-timeUpdate[i-1]) / S_TO_US;
			}

			if (t_max == 0 || t_max < timeUpdate[i]) {
				t_max = timeUpdate[i];
			}
			if (t_min == 0 || t_min > timeUpdate[i]) {
				t_min = timeUpdate[i];
			}

			output = output + dt * values[i];
		}


		double duration_seconds = (t_max - t_min) / S_TO_US;
		if (duration_seconds > 0) {
			output = output / duration_seconds;
		}

		return output;
	}

	/**
	 * Get derivative value of the filter
	 */
	double getDerivative() {
		if (index < 1 && !hasCycle) {
			return 0.0;
		}

		double output = 0.0;
		int iPosCompareToIndex = 0;
		double cWeight = 1.0;
		double nSum = 0.0;

		for (int i=1; i < NB_SAMPLING; i ++) {

			if (index > 0 && i < index) {
				iPosCompareToIndex = index-i;
			}
			else if (index == 0 && hasCycle) {
				iPosCompareToIndex = NB_SAMPLING - i;
			}
			else if (i > index && hasCycle) {
				iPosCompareToIndex = NB_SAMPLING - i + index;
			}

			// There is no previous to index..s
			if (i != index) {
				cWeight = 1.0 / (iPosCompareToIndex+1.0);
				output += cWeight * (values[i]-values[i-1]) / ((timeUpdate[i]-timeUpdate[i-1])/S_TO_US);
				nSum += cWeight;

				// DEBUG: printf("i=%d | index=%d | cWeight=%f | val=%f | dt=%f\n", i, index, cWeight, values[i], ((timeUpdate[i]-timeUpdate[i-1])/S_TO_US));
			}
		}

		return output / nSum;
	};


	/**
	 * Tells whether if the data is relevant (not too old) or not
	 */
	bool areDataRelevant(long pTime) {
		if (index == 0 && !hasCycle) {
			return false;
		}

		int i = index-1;
		if (i < 0 && hasCycle) {
			i = NB_SAMPLING-1;
		}

		return (absolute(pTime-timeUpdate[i]) < MAX_TIME_ELAPSED_RELEVANT_DATA);
	};



private:
	int minFilterLowPass;
	int maxFilterHighPass;
	int filterLength;
	int index;
	bool hasCycle;
	long timeUpdate[NB_SAMPLING];
	double values[NB_SAMPLING];
	bool weightedMode;
};


#endif /* FILTERAVERAGE_H_ */

