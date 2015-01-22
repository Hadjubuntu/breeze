/*
 * Scheduler.h
 *
 *  Created on: Jan 18, 2015
 *      Author: Adrien Hadj-Salah
 */

#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "arch/AVR/MCU/MCU.h"

#define MAX_TASKS 30

typedef void (*task_fn_t)(void);

typedef struct T_TASK {
	task_fn_t processTask;
	long lastStartTimeUs;
	long tickIntervalUs;
	long maxDurationUs;
	long averageDurationUs;
} Task ;


Task *tasks;
int nbTasks = 0;


// Init the scheduler
// ------------------------------------------------------
// sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0])
void schedulerInit(Task *pTaskList, int pNbTasks) {
	tasks = pTaskList;
	nbTasks = pNbTasks;
}

long getTaskAverageDurationUs(int pIndex) {
	if (tasks) {
		return (tasks[pIndex].averageDurationUs);
	}

	return 0;
}

/**
 * Example taken :
Task #0 duration 3475 us | slot time taken : 34% | 100 Hz
Task #1 duration 97 us | slot time taken : 0% | 50 Hz
Task #2 duration 200 us | slot time taken : 0% | 20 Hz
Task #3 duration 22769 us | slot time taken : 22% | 10 Hz
Task #4 duration 6 us | slot time taken : 0% | 5 Hz
Task #5 duration 25109 us | slot time taken : 5% | 2 Hz
Task #6 duration 2875 us | slot time taken : 0% | 1 Hz

 *
 */
void schedulerStats() {
	for (int i = 0; i < nbTasks; i ++) {
		long taskDurationUs = getTaskAverageDurationUs(i);
		long nextTickUs = tasks[i].tickIntervalUs;
		int percent = (int)(((double)(taskDurationUs) / nextTickUs) * 100.0);

		Logger.print("Task #");
		Logger.print(i);
		Logger.print(" duration ");
		Logger.print(taskDurationUs);
		Logger.print(" us | slot time taken : ");
		Logger.print(percent);
		Logger.println("%");
	}
}

// Run the scheduler
// Must be runned at 100Hz or higher frequency
// ------------------------------------------------------
void schedulerRun() {

	for (int i = 0; i < nbTasks; i ++) {
		Task *cTask = &tasks[i];
		long ctime = timeUs();

		if ((ctime - cTask->lastStartTimeUs) > cTask->tickIntervalUs) {
			cTask->lastStartTimeUs = ctime;
			cTask->processTask();
			long durationUs = timeUs() - cTask->lastStartTimeUs;
			cTask->averageDurationUs = 0.3 * durationUs + 0.7 * cTask->averageDurationUs;
//			if (cTask->averageDurationUs > cTask->maxDurationUs) {
//				Serial.print("Error task #");
//				Serial.print(i);
//				Serial.println(" too long");
//			}
		}
	}
}

#endif /* SCHEDULER_H_ */
