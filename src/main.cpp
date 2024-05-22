#include "Arduino.h"
#include "gantry.h"
#include "ballScrew.h"
#include "servoTiltModule.h"

#define runButton 35 
#define homeButton 32
#define emergencyStopButton 33

//THIS roughly means the distance you want to nozzle to be at before entrance into the tube at 60 degrees
#define heightAbovePivot_um 55000
#define tubeWidth_mm 27

#define pumpPin 14
#define pumpMicrosteps 16

#define pumpDirectionPin 5 //NOTE THAT FORWARD DIRECTION FOR THE PUMP IS TRUE/HIGH

#define volumeSensingPin 23

#define startingX_mm 0
#define startingY_mm 67


bool pumpDirection = true;//true is forward false is backwards


hw_timer_t * timer = NULL;      //H/W timer defining (Pointer to the Structure)
volatile bool pinState = false;
volatile bool isPumpOn = false;

servoTiltModule TiltModule = servoTiltModule();
gantry Gantry = gantry();
int startingZ_mm  = Gantry.getMaxZDisplacement() - 50;

void stopAllMotors();
void stopPump();
void performFillingMotionforAll4();
void performFillingMotionFor1Tube(int tubeNumber);
void onTimer();
void setPumpRPM(int rpm, int pump_pin, int microstepsPerStep);
void setPumpDirection(bool dir);//true is forward, false is reverse

void setup() {
  //Initialize tilt module pwm
  TiltModule.pwm = Adafruit_PWMServoDriver();
  TiltModule.pwm.begin();
  TiltModule.pwm.setPWMFreq(freq);
  
  //Initialize buttons
  pinMode(runButton, INPUT);
  pinMode(homeButton, INPUT);
  pinMode(emergencyStopButton, INPUT);
  pinMode(pumpDirectionPin, OUTPUT);
  setPumpDirection(true);//true is forward


  //Initialize pump pin
  pinMode(pumpPin, OUTPUT);

  pinMode(volumeSensingPin, INPUT);

  //Add external interupt for emergency stop button  
  attachInterrupt(digitalPinToInterrupt(emergencyStopButton), stopAllMotors, RISING);

  attachInterrupt(digitalPinToInterrupt(volumeSensingPin), stopPump, FALLING);

  

  //Initialize internal timer interupt used to control pump rpm
  timer = timerBegin(0, 80, true);           	// timer 0, prescalar: 80, UP counting
  timerAttachInterrupt(timer, &onTimer, true); 	// Attach interrupt
  timerAlarmWrite(timer, 1000000, true);  		// Match value= 1000000 for 1 sec. delay.
  timerAlarmEnable(timer);           			// Enable Timer with interrupt (Alarm Enable)
}

void loop() {
  //Perform homing sequence and tilt tube holders to initial angles
  if(digitalRead(homeButton) == HIGH){
    //go to upright angle
    TiltModule.setAllTubesToAngle(0);
    delay(1000);
    Gantry.homeGantry();
    //go to loading angle
    int loadingAngle = -30;
    TiltModule.setAllTubesToAngle(loadingAngle);
  }

  //Perform one of the filling sequences
  if (digitalRead(runButton) == HIGH)
  {

    //performFillingMotionforAll4();  
    performFillingMotionFor1Tube(1);

  }
}





void performFillingMotionFor1Tube(int tubeNumber){
  //Make sure that the user is calling a valid tube 
  if (tubeNumber > 4 || tubeNumber < 1){
    return;
  }

  int startingXPosition_mm = TiltModule.getAbsoluteStartingXPositionOfTube(startingX_mm, tubeNumber);

  //only for testing get rid after
  delay(1000);
  setPumpDirection(false);
  setPumpRPM(300, pumpPin, pumpMicrosteps);
  delay(3000);
  setPumpRPM(0, pumpPin, pumpMicrosteps);
  setPumpDirection(true);


  setPumpDirection(true);
  
  TiltModule.goDirectlyToTubeAngle(0, tubeNumber);

  delay(1000);
  
  //go to center above the tube.
  Gantry.goToAbsPosition_mm(startingXPosition_mm, startingY_mm, startingZ_mm, 5);
   
  //move to startign position for angle 60 deg, split up in 2 motions to avoid collision
  int firstFillAngle = 60;
  

  //Offset in z direction so that nozzle travels along bottom side of tube as opposed to along the center of thubes axis
  int zOffsetForBottomOfTube = tubeWidth_mm*1000/(2*sin(firstFillAngle*PI/float(180)));
  
  //note that I 57000 ensures contact but maybe even  bit too much bending of the nozzle
  //note I found that 55000 also works with less bending, can likely still be improved. note that a lower number will make nozzle bend less, i.e lift it up
  int intialHeightAboveAxis = 55000;
  
  
  
  //try combining moves int o1 diagonal pass 
  Gantry.goToRelativePosition(0, heightAbovePivot_um*sin(PI*firstFillAngle/float(180)), heightAbovePivot_um*cos(PI*firstFillAngle/float(180)) - intialHeightAboveAxis - zOffsetForBottomOfTube, 5000);    

  
  
  delay(1000);

  int TUBE_ANGLE_OFFSET_FOR_INSERTION = 15;
  //FIRST MOVE TUBE 5 degs so there are no collisions then move back
  TiltModule.sweepTubeToAngle(firstFillAngle + TUBE_ANGLE_OFFSET_FOR_INSERTION, 1, tubeNumber);


  int entranceDistance_um = 64500;
  //slide into tube very slowly as deep as posssible
  Gantry.goToRelativePosition(0, -entranceDistance_um*sin(PI*(firstFillAngle)/float(180)), -entranceDistance_um*cos(PI*(firstFillAngle)/float(180)), 5000);
   
  TiltModule.sweepTubeToAngle(firstFillAngle, 1, tubeNumber);

  //initizl pump prime
  setPumpRPM(300, pumpPin, pumpMicrosteps);
  delay(1900);
  setPumpRPM(0, pumpPin, pumpMicrosteps);
  delay(1000);
  //ADD CODE HERE TO DO INITIAL FILLING WHILE THE TUBE IS FULLY IN.
  setPumpRPM(10, pumpPin, pumpMicrosteps);
  delay(30000);

  //ONCE THE BLOOD HAS REACHED WHERE THE NOZZLE IS THE CONTINUE TO NEXT SECTION.

  //find distance to move out of the tube
    //this is the diagonal distance out of the tube you with to travel, I assume it is just 1cm shy of where you started so as to ensure you are in the tube at the end
    int exitDistance_um = entranceDistance_um - 20000;
  //the line below pulls the tube out in 1 shot where as the loop lets you set different pump speeds as you pull it out.
  //Gantry.goToRelativePosition(0, exitDistance_um*sin(PI*firstFillAngle/float(180)), exitDistance_um*cos(PI*firstFillAngle/float(180)), 5000);
   
   
      //HERE I LET YOU DO DIFFERENT PUMP SEQUENCES AS YOU FILL IT UP
   int pumpRPMS[] = {10, 12, 14, 16, 18, 20, 0};
   //MAKE SURE BOTH THESE ARRAYS HAVE SAME NUMBER OF ELEMENTS!!
   int delays_ms_Per_pumpingInterval[] = {30000, 30000, 30000, 20000, 15000, 15000, 1000};

   int numberOfPumpingSequencesWhileExitingTube = sizeof(pumpRPMS)/sizeof(int);
   int exitDistancePerPumpSequence_um = exitDistance_um/numberOfPumpingSequencesWhileExitingTube;
   for(int i = 0; i < numberOfPumpingSequencesWhileExitingTube; i++){
    //set new pumping speed and slide gantry up tube, take sliding time as delay before new pumping speed
    setPumpRPM(pumpRPMS[i], pumpPin, pumpMicrosteps);
    Gantry.goToRelativePosition(0, exitDistancePerPumpSequence_um*sin(PI*firstFillAngle/float(180)), exitDistancePerPumpSequence_um*cos(PI*firstFillAngle/float(180)), delays_ms_Per_pumpingInterval[i]);
   }
  
  //take the nozzle out by traveling straight up.
  Gantry.goToRelativePosition(0, 0, 40000, 5000);

  // straighten out 
  int finalFillAngle = -10;
  TiltModule.sweepTubeToAngle(finalFillAngle, 1, tubeNumber);


  //go to center above the tube
  Gantry.goToAbsPosition_mm(startingXPosition_mm, startingY_mm, startingZ_mm, 5);

  int depthBelowTubeTop_um = 25000;//this is for the depth along the tube wall you want the final fill position to be at.


  //take the nozzle to tube wall, old logic
  Gantry.goToRelativePosition(0, -(tubeWidth_mm*1000/2 + sin(abs(finalFillAngle)*PI/180)*depthBelowTubeTop_um), -cos(abs(finalFillAngle)*PI/180)*depthBelowTubeTop_um, 5000);
  
  setPumpRPM(50, pumpPin, pumpMicrosteps);
  delay(10000);

  //go to center above the tube
  Gantry.goToAbsPosition_mm(startingXPosition_mm, startingY_mm, startingZ_mm, 5);

  //go back to center
  TiltModule.sweepTubeToAngle(0, 2, tubeNumber);

    //will need to be changed so it just runs once at the end of filling four tubes
  delay(1000);
  setPumpDirection(false);
  setPumpRPM(300, pumpPin, pumpMicrosteps);
  delay(3000);
  setPumpRPM(0, pumpPin, pumpMicrosteps);
  setPumpDirection(true);

}

//DO NOT CHANGE ANYTHING BELOW THIS LINE
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
.
.
.
.
.
.
.
.
.
.
.
.
.
.
.
.
.
.
.
.
.
.
.
*/

void performFillingMotionforAll4(){
  //cycle through all 4 motions.
  for(int i = 0; i < TiltModule.getNumberOfTubes(); i++){
    performFillingMotionFor1Tube(i+1);
  }

  //once it is finished then go to top left
  Gantry.goToAbsPosition_mm(0, Gantry.getMaxYDisplacement(), Gantry.getMaxZDisplacement(), 10);
}


//true is forward, false is reverse
void setPumpDirection(bool dir){
  pumpDirection = dir;
  digitalWrite(pumpDirectionPin, pumpDirection);
}

void stopAllMotors(){
  Gantry.emergencyStop();
  setPumpRPM(0, pumpPin, pumpMicrosteps);  
}

void onTimer(){
  if(isPumpOn){
    pinState = !pinState;
    digitalWrite(pumpPin, -pinState);
  }
}

void stopPump(){
  setPumpRPM(0, pumpPin, pumpMicrosteps);
}

void setPumpRPM(int rpm, int pump_pin, int microstepsPerStep){
  //note this code assumes that the prescaler on the internal timer (timer)
  //is set such that each tick is a microsecond
  int standardSteps = 200;
  int stepsPerSecond = rpm*standardSteps*microstepsPerStep/60;
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



