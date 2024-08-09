#include "Arduino.h"
#include "peristalticPump.h"

volatile bool peristalticPump::pumpDirection = true;
volatile bool peristalticPump::pinState = false;
volatile bool peristalticPump::isPumpOn = false;
volatile bool peristalticPump::isForcedStop = false;


peristalticPump::peristalticPump(){
    pumpDirection = true;//true is forward false is backwards
    pinState = false;
    isPumpOn = false;

    pinMode(pumpDirectionPin, OUTPUT);
    setPumpDirection(true);//true is forward

    //Initialize pump pin
    pinMode(pumpPin, OUTPUT);

    //note that this may be required to have in main not sure
    //Initialize internal timer interupt used to control pump rpm
    timer = timerBegin(0, 80, true);           	// timer 0, prescalar: 80, UP counting
    timerAlarmWrite(timer, 1000000, true);  		// Match value= 1000000 for 1 sec. delay.
    timerAlarmEnable(timer);           			// Enable Timer with interrupt (Alarm Enable)
}


  //true is forward, false is reverse
void peristalticPump::setPumpDirection(bool dir){
  pumpDirection = dir;
  digitalWrite(pumpDirectionPin, pumpDirection);
}

void peristalticPump::onTimer(){
  if(isPumpOn && !isForcedStop){
    pinState = !pinState;
    digitalWrite(pumpPin, -pinState);
  }
}

void peristalticPump::stopPump(){
    isPumpOn = false;
}

void peristalticPump::setPumpRPM(int rpm){
  //note this code assumes that the prescaler on the internal timer (timer)
  //is set such that each tick is a microsecond
  int standardSteps = 200;
  int stepsPerSecond = rpm*standardSteps*pumpMicrosteps/60;
  int pumpStepDelay_us = 1000000/stepsPerSecond;
  if(rpm == 0){
    isPumpOn = false;
    timerAlarmWrite(timer, 1000000, true);//make it trigger every second and ignore the trigger, in the future make it so the timer actually becomes dissabled
  }
  else{
    isPumpOn = true;
    timerAlarmWrite(timer, pumpStepDelay_us/2, true);  		// Match value= 1000000 for 1 sec. delay.
  }
}