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

#define DATA_BUFFER_SIZE 50
#define NB_BYTES_PER_PACKET 85

int dataIdx = 0;

typedef struct T_VECTOR3D {
	int x;
	int y;
	int z;
} Vector3D;

Vector3D datas[DATA_BUFFER_SIZE];

void addData(double pX, double pY, double pZ) {
	if (dataIdx < DATA_BUFFER_SIZE) {
		Vector3D el;
		el.x = (int) (pX * 10);
		el.y = (int) (pY * 10);
		el.z = (int) (pZ * 10);

		datas[dataIdx] = el;
		dataIdx ++;
	}
	// Otherwise throw error buffer overflow
}

// Try to send data in buffer
void updateLowFreqRFPlot() {
	int nbBytes = 0;
	int sendingIdx = 0;
	char buf[NB_BYTES_PER_PACKET];

	while (nbBytes < NB_BYTES_PER_PACKET
			&& sendingIdx < dataIdx) {

		sprintf(buf, "%s|%d|%d|%d",
				buf,
				datas[sendingIdx].x,
				datas[sendingIdx].y,
				datas[sendingIdx].z);

		// TODO define cbyte with data inserted
		int cByte = 5*3+3;
		nbBytes += cByte;
		sendingIdx ++;
	}

	sprintf(buf, "%s|\n", buf);

	// TEST this function
	// SEND those data to GCS via RFLink2
	// UPDATE data FIFO updateFifo = sublist(sendingIdx, to the end)
}



#endif /* MODULES_RFLINK_RFPLOT_H_ */
