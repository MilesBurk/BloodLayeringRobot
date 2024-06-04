#pragma once
#include "Arduino.h"

//must change these values once we know which pins will be used for certain.
#define volumeSensingPin1 32
#define volumeSensingPin2 33
#define volumeSensingPin3 25
#define volumeSensingPin4 26

#define numberOfSensors 4

class volumeSenseModule{
    public:
        volumeSenseModule();
        static bool timeToStopPump();
        static volatile short volumeSensorPins[];
        static volatile short currentTubeBeingFilled;
        static volatile bool performingFinalFill;
};