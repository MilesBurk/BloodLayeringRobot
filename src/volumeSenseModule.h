#pragma once
#include "Arduino.h"

//must change these values once we know which pins will be used for certain.
#define volumeSensingPin1 23
#define volumeSensingPin2 23
#define volumeSensingPin3 23
#define volumeSensingPin4 23

#define numberOfSensors 4

class volumeSenseModule{
    public:
        volumeSenseModule();
        static void timeToStopPump();
        static volatile short volumeSensorPins[];
        static volatile short currentTubeBeingFilled;
};