#include "Arduino.h"
#include "gantry.h"
#include "ballScrew.h"
#include "servoTiltModule.h"
#include "peristalticPump.h"
#include "volumeSenseModule.h"

#define runButton 35 
#define homeButton 32
#define emergencyStopButton 33

//THIS roughly means the distance you want to nozzle to be at before entrance into the tube at 60 degrees
#define heightAbovePivot_um 55000
#define tubeWidth_mm 27


#define startingX_mm 0
#define startingY_mm 63

volumeSenseModule VolumeSensors = volumeSenseModule();
peristalticPump Pump = peristalticPump();

servoTiltModule TiltModule = servoTiltModule();
gantry Gantry = gantry();
int startingZ_mm  = Gantry.getMaxZDisplacement() - 50;

void initializePushButtons();
void initializeInterupts();
void stopAllMotors();
void performFillingMotionforAll4();
void performFillingMotionFor1Tube(int tubeNumber);
void lockerStorageSequence();


void setup() {
  //Initialize tilt module pwm
  TiltModule.pwm = Adafruit_PWMServoDriver();
  TiltModule.pwm.begin();
  TiltModule.pwm.setPWMFreq(freq);
  
  initializePushButtons();
  initializeInterupts();
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
    //lockerStorageSequence(); 

    //ONLY TEST WITH FIRST TUBE, OTHER VOLUYME SENSORES ARE NOT YET CONNECTED
    performFillingMotionFor1Tube(3);
  }
}



void performFillingMotionFor1Tube(int tubeNumber){
  //Make sure that the user is calling a valid tube 
  if (tubeNumber > 4 || tubeNumber < 1){
    return;
  }

  VolumeSensors.currentTubeBeingFilled = tubeNumber;
  int startingXPosition_mm = TiltModule.getAbsoluteStartingXPositionOfTube(startingX_mm, tubeNumber);


  //only for testing get rid after
  delay(1000);
  Pump.setPumpDirection(false);
  Pump.setPumpRPM(300);
  delay(3000);
  Pump.setPumpRPM(0);
  Pump.setPumpDirection(true);

  
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
  Pump.setPumpRPM(300);
  delay(1900);
  Pump.setPumpRPM(0);
  delay(1000);
  //ADD CODE HERE TO DO INITIAL FILLING WHILE THE TUBE IS FULLY IN.
  Pump.setPumpRPM(10);
  delay(30000);

  //ONCE THE BLOOD HAS REACHED WHERE THE NOZZLE IS THE CONTINUE TO NEXT SECTION.

  ///find distance to move out of the tube
    //this is the diagonal distance out of the tube you with to travel, I assume it is just 1cm shy of where you started so as to ensure you are in the tube at the end
    int exitDistance_um = entranceDistance_um - 11000;
  //the line below pulls the tube out in 1 shot where as the loop lets you set different pump speeds as you pull it out.
  //Gantry.goToRelativePosition(0, exitDistance_um*sin(PI*firstFillAngle/float(180)), exitDistance_um*cos(PI*firstFillAngle/float(180)), 5000);
   
   
      //HERE I LET YOU DO DIFFERENT PUMP SEQUENCES AS YOU FILL IT UP
   int pumpRPMS[] = {10, 12, 14, 16, 22, 25, 30, 0};
   //MAKE SURE BOTH THESE ARRAYS HAVE SAME NUMBER OF ELEMENTS!!
   int delays_ms_Per_pumpingInterval[] = {3000, 3000, 3000, 2000, 1500, 2000, 2000, 1000};

   int numberOfPumpingSequencesWhileExitingTube = sizeof(pumpRPMS)/sizeof(int);
   int exitDistancePerPumpSequence_um = exitDistance_um/numberOfPumpingSequencesWhileExitingTube;
   for(int i = 0; i < numberOfPumpingSequencesWhileExitingTube; i++){
    //set new pumping speed and slide gantry up tube, take sliding time as delay before new pumping speed
    Pump.setPumpRPM(pumpRPMS[i]);
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
  
  Pump.setPumpRPM(50);
  //here you would fill until the volume sensors is triggered.
  while(peristalticPump::isPumpOn){};//basically wait until the pump turns itself off.

  //go to center above the tube
  Gantry.goToAbsPosition_mm(startingXPosition_mm, startingY_mm, startingZ_mm, 5);

  //go back to center
  TiltModule.sweepTubeToAngle(0, 2, tubeNumber);

    //will need to be changed so it just runs once at the end of filling four tubes
  delay(1000);
  Pump.setPumpDirection(false);
  Pump.setPumpRPM(300);
  delay(3000);
  Pump.setPumpRPM(0);
  Pump.setPumpDirection(true);

}








void performFillingMotionforAll4(){
  //cycle through all 4 motions.
  for(int i = 0; i < TiltModule.getNumberOfTubes(); i++){
    performFillingMotionFor1Tube(i+1);
  }

  //once it is finished then go to top left
  Gantry.goToAbsPosition_mm(0, Gantry.getMaxYDisplacement(), Gantry.getMaxZDisplacement(), 10);
}

void stopAllMotors(){
  Gantry.emergencyStop();
  Pump.stopPump();  
}

void initializePushButtons(){
  //Initialize buttons
  pinMode(runButton, INPUT);
  pinMode(homeButton, INPUT);
  pinMode(emergencyStopButton, INPUT);
}

  void initializeInterupts(){
    //Add external interupt for emergency stop button  
    attachInterrupt(digitalPinToInterrupt(emergencyStopButton), stopAllMotors, RISING);
    
    //Interupt for volume sensor pins.
    for(int i = 0; i < numberOfSensors; i++){
      attachInterrupt(digitalPinToInterrupt(volumeSenseModule::volumeSensorPins[i]), peristalticPump::stopPump, FALLING);
    }

    //Interupt for pump control
    timerAttachInterrupt(Pump.timer, peristalticPump::onTimer, true); 	// Attach interrupt for pump
  }

void lockerStorageSequence(){
  Gantry.goToAbsPosition_mm(Gantry.getMaxXDisplacement()/2, Gantry.getMaxYDisplacement()/2, Gantry.getMaxZDisplacement(), 10);
}


