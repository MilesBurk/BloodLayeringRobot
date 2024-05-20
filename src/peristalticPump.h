#pragma once
#include "Arduino.h"

#define pumpPin 14
#define pumpMicrosteps 16
#define pumpDirectionPin 5 //NOTE THAT FORWARD DIRECTION FOR THE PUMP IS TRUE/HIGH

class peristalticPump{
    public:
        peristalticPump();
        static void onTimer();
        static void setPumpRPM(int rpm);
        void setPumpDirection(bool dir);//true is forward, false is reverse
        static void stopPump();
        static hw_timer_t * timer;      //H/W timer defining (Pointer to the Structure)

    private:
        static volatile bool pumpDirection;//true is forward false is backwards
        static volatile bool pinState;
        static volatile bool isPumpOn;
};


