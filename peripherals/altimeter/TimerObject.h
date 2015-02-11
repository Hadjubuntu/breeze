/*
 * TimerObject.h
 *
 *  Created on: Feb 10, 2015
 *      Author: adrien
 */
#include "stdlib.h"
#include "Arduino.h"

#ifndef TIMER_OBJECT_H
#define TIMER_OBJECT_H
typedef void (*CallBackType)();


class TimerObject{
private:
	void Create(unsigned long int ms, CallBackType callback, bool isSingle);
	unsigned long int msInterval;
	bool blEnabled;
	bool blSingleShot;
	CallBackType onRun;
	bool Tick();
	unsigned long LastTime;
	unsigned long DiffTime;//used when I pause the Timer and need to resume

public:
	TimerObject(unsigned long int ms);
	TimerObject(unsigned long int ms, CallBackType callback);
	TimerObject(unsigned long int ms, CallBackType callback, bool isSingle);
	~TimerObject();

	void setInterval(unsigned long int ms);
	void setEnabled(bool Enabled);
	void setSingleShot(bool isSingle);
	void setOnTimer(CallBackType callback);
	void Start();
	void Resume();
	void Pause();
	void Stop();
	void Update();


	unsigned long int getInterval();
	unsigned long int getCurrentTime();
	CallBackType getOnTimerCallback();

	bool isEnabled();
	bool isSingleShot();

};


TimerObject::TimerObject(unsigned long int ms){
	Create(ms, NULL, false);
}

TimerObject::TimerObject(unsigned long int ms, CallBackType callback){
	Create(ms, callback, false);
}

TimerObject::TimerObject(unsigned long int ms, CallBackType callback, bool isSingle){
	Create(ms, callback, isSingle);
}

void TimerObject::Create(unsigned long int ms, CallBackType callback, bool isSingle){
	setInterval(ms);
	setEnabled(false);
	setSingleShot(isSingle);
	setOnTimer(callback);
	LastTime = 0;
}

void TimerObject::setInterval(unsigned long int ms){
	msInterval = (ms > 0) ? ms : 0;
}

void TimerObject::setEnabled(bool Enabled){
	blEnabled = Enabled;
}

void TimerObject::setSingleShot(bool isSingle){
	blSingleShot = isSingle;
}

void TimerObject::setOnTimer(CallBackType callback){
	onRun = callback;
}

void TimerObject::Start(){
	LastTime = millis();
	setEnabled(true);
}

void TimerObject::Resume(){
	LastTime = millis() - DiffTime;
	setEnabled(true);
}

void TimerObject::Stop(){
	setEnabled(false);

}

void TimerObject::Pause(){
	DiffTime = millis() - LastTime;
	setEnabled(false);

}

void TimerObject::Update(){
	if(Tick())
		onRun();
}

bool TimerObject::Tick(){
	if(!blEnabled)
		return false;
	if(LastTime > millis()*2)//millis restarted
		LastTime = 0;
	if ((unsigned long int)(millis() - LastTime) >= msInterval) {
		LastTime = millis();
		if(isSingleShot())
			setEnabled(false);
	    return true;
	}
	return false;
}


unsigned long int TimerObject::getInterval(){
	return msInterval;
}

unsigned long int TimerObject::getCurrentTime(){
	return (unsigned long int)(millis() - LastTime);
}
CallBackType TimerObject::getOnTimerCallback(){
	return onRun;
}

bool TimerObject::isEnabled(){
	return blEnabled;
}

bool TimerObject::isSingleShot(){
	return blSingleShot;
}

#endif // TIMER_OBJECT_H
