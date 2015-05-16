/*
 * RFPlot.h
 *
 * This module aims to collect data before sending it the GCS in block in order
 * for the user to plot a graph between two variables
 *
 *  Created on: Apr 6, 2015
 *      Author: hadjsalah
 */

#ifndef MODULES_RFLINK_RFPLOT_H_
#define MODULES_RFLINK_RFPLOT_H_

#include "math/Math.h"

// Global variables
//---------------------------------------
#define DATA_BUFFER_SIZE 50
#define NB_BYTES_PER_PACKET 85

int dataIdx = 0;
int sendingIdx = 0;
Vector3f datas[DATA_BUFFER_SIZE];



/****************************************************
 * Add data to vector3f until the stack is full
 ****************************************************/
void addDataToVector3f(Vector3f vector[], int *i, double pX, double pY, double pZ) {
	if ((*i) < DATA_BUFFER_SIZE) {
		Vector3f el;
		el.x = (int) (pX * 10);
		el.y = (int) (pY * 10);
		el.z = (int) (pZ * 10);

		vector[(*i)] = el;
		(*i) ++;
	}
	// Otherwise throw error buffer overflow
}

void addDataPlot(double pX, double pY, double pZ) {
	addDataToVector3f(datas, &dataIdx, pX, pY, pZ);
}

/****************************************************
 * Send buffer of those data until packet is full
 ****************************************************/
void updateLowFreqRFPlot() {

	// Prepare variables
	//---------------------------------------
	int nbBytes = 0;
	char buf[NB_BYTES_PER_PACKET];

	// Packet header
	//---------------------------------------
	sprintf(buf, "%s", "plot");

	// While the packet is not too big add some data
	//---------------------------------------
	while (nbBytes < NB_BYTES_PER_PACKET
			&& sendingIdx < dataIdx) {

		sprintf(buf, "%s|%d|%d|%d",
				buf,
				toCenti(datas[sendingIdx].x),
				toCenti(datas[sendingIdx].y),
				toCenti(datas[sendingIdx].z));

		// TODO define cbyte with data inserted
		int cByte = 5*3 + 3;
		nbBytes += cByte;
		sendingIdx ++;
	}

	sprintf(buf, "%s|\n", buf);

	// Write on UART 1
	Serial1.write(buf);


	// TEST this function
	// SEND those data to GCS via RFLink2
	// UPDATE data FIFO updateFifo = sublist(sendingIdx, to the end)

	// Copy rest of the data at the beginning of the FIFO
	//---------------------------------------
	int j = 0;

	for (int i = sendingIdx; i < dataIdx; i ++) {
		datas[i] = vect3fInstance(datas[i].x, datas[i].y, datas[i].z);
		j ++;
	}
	dataIdx = j;

	if (j == 0) {
		sendingIdx = 0;
	}
}


#endif /* MODULES_RFLINK_RFPLOT_H_ */
