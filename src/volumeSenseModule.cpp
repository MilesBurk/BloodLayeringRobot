#include "Arduino.h"
#include "volumeSenseModule.h"
#include "peristalticPump.h"


volatile short volumeSenseModule::volumeSensorPins[] = {volumeSensingPin1, volumeSensingPin2, volumeSensingPin3, volumeSensingPin4};

volatile short volumeSenseModule::currentTubeBeingFilled = 1;

volatile bool volumeSenseModule::performingFinalFill = false;

volumeSenseModule::volumeSenseModule(){
    //initialize ESP pins for input
    for(int i = 0; i < numberOfSensors; i++){
        pinMode(volumeSensorPins[i], INPUT);
    }

}

void volumeSenseModule::timeToStopPump(){
    //Ensure that it was the intended pin that has triggere the interupt and not one of the other pins
    if(performingFinalFill && !digitalRead(volumeSensorPins[currentTubeBeingFilled-1])){
        peristalticPump::stopPump();
        performingFinalFill = false;
    }
}
