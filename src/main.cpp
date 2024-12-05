#include <Arduino.h>
#include "Sensing.h"
#include "Driving.h"

/**
 * PINS:
 * 1,2,3,4,5,7 used by QuadDecoder.h
 * 8, 9, 10, 11 used by motor driver
 * 13, 14 used by Ultrasonic
 * 15, 16, 17 used by mics
 * 18, 19, 20 used by IR signal return
 * 
 * 
 * 21, 22, 23 analog still open
 * 6, 12 digital/PWM still open
 */

//declare subroutines
void continueRampage();
void initRampage(int offendingSensor);
void handleBlockade(double distanceVal);
void updateState(int NEW_STATE);
void revertState();
void pollIRValueRadially();
int getHeadingFromirMAP();
void idleWhileRotating();


const int NORMAL = 1;
const int SENSING = 2;
const int BLOCKED = 3;
int CURRENT_STATE, LAST_STATE;

uint32_t nextSoundPollTime;
#define BLOCKAGE_TOLERANCE 0.15

uint32_t offroadTimer;
bool offRoadTimerActive;
int irMAP[360];
#define IR_LOWER_THRESHOLD 1250 //threshold of 1000 for 

void setup(){
  Serial.begin(9600);
  initSensing();
  initDriving();
  
  nextSoundPollTime = millis();
  offroadTimer = millis();

  offRoadTimerActive = false;

  CURRENT_STATE = NORMAL;
  LAST_STATE = 0;//set to invalid state if unused
  enableMovement();
  // Serial.println("waiting for input to continue...");
  // while(!Serial.available()){
  //   Serial.println(getDistanceValue());
  //   delay(100);
  // }
  delay(2000);
  Serial.println("beginning program.");
}

void loop(){
  /**
   * state machine. may be in normal mode, blocked, or sensing. 
   */
  if(CURRENT_STATE == NORMAL){
    int linePosition = getLinePosition();
    std::array<int, 3> irValues = getIRValues();

    //determine if vehicle has lost sight of the line and make corrections accordingly

/*
    if(irValOffroad(irValues[0], irValues[1], irValues[2])){
      if(offRoadTimerActive){
        if(linePosition >850 && linePosition < 1150 ){//all three sensors read low, and sensor was not at an edge recently
          if(offroadTimer < millis()){//any buffer time from exit of last sensing mode has expired
            irValues = getIRValues();
            if(irValues[0] < IR_LOWER_THRESHOLD && irValues[1] < IR_LOWER_THRESHOLD && irValues[2] < IR_LOWER_THRESHOLD){
                Serial.print("have an expired timer... ");
                Serial.println(" and it expired recently with the low condition, indicating a current fault");
                Serial.printf("IR values: %i, %i, %i\n", irValues[0], irValues[1], irValues[2]);
                Serial.printf("readLine value: %i", linePosition);
                Serial.println("off course detected, initiating sensing mode");
                updateState(SENSING);
                disableMovement();
                resetTickCounts();
                rotateForCalibration();
            }
          }
        }
      }
      else{
        offRoadTimerActive = true;
        offroadTimer = millis()+100;
      }
    }
*/


    //update driving vars with IR readLine data and mic values for bump compensation
    std::array<int, 3> micValues = getMicValues();
    setDrivingVars(linePosition, micValues[MIC_FRONT_RIGHT],  micValues[MIC_REAR], micValues[MIC_FRONT_LEFT]);

    //every ~100 ms:
    //read ultrasonic range sensor. called less often that other inputs due to 10 ms delay on each ultrasonic reading
    // uint32_t current_time = millis();
    // if(nextSoundPollTime < current_time){
    //   nextSoundPollTime = current_time + 100;//update the time to be 100 ms later
      
    //   double distanceVal = getDistanceValue();//ultrasonic sensor and mics should be polled significantly less often than the IR reflectance sensor.
    //   // handleBlockade(distanceVal);
    // }
  }
  else if(CURRENT_STATE == SENSING){
    //Serial.println("currently sensing due to lack of line to follow");
    //360 degree rotation started in NORMAL state last iteration
    bool finishedRotating = continueRotating(getEncoderData(LEFT), getEncoderData(RIGHT));
    pollIRValueRadially();
    if(finishedRotating){
      int newHeading = getHeadingFromirMAP();
      // Serial.print("New calculated heading is: ");
      // Serial.println(newHeading);
      // Serial.println("sensed and turned toward new trajectory. exiting sensing state");

      //resetTickCounts();
      rotateToAngle(newHeading);
      idleWhileRotating();
      enableMovement();
      updateState(NORMAL);
    }
  }
  else if(CURRENT_STATE == BLOCKED){
    Serial.println("blockade detected. call offroad functions to handle this");
  }

}

/**
 * hold in a loop until rotation has completed.
 * prevents logic from executing while rotating
 */
void idleWhileRotating(){
  while(!continueRotating(getEncoderData(LEFT), getEncoderData(RIGHT))){
    delay(5);//hold in non-sensing state until rotation is complete
  }
}

/**
 * function to be called continuously and given the distance value from the ultrasonic sensor
 * handles locking and unlocking movement as needed given the input data
 * function may be called while in a state other than normal, and uses last_state to return to the correct state after blockade removed. this logic may turn out to be super shitty
 * maybe have a dynamic array and push/pop so that index 0 is the current state
 */
void handleBlockade(double distanceVal){
 if(CURRENT_STATE == BLOCKED){
    if(distanceVal > BLOCKAGE_TOLERANCE){
      Serial.println("Blockade removed, reverting to normal state");
      enableMovement();
      revertState();
    }
  }
  else{
    if(distanceVal < BLOCKAGE_TOLERANCE){
      Serial.println("Blockade detected, executing evasion maneuver");
      disableMovement();
      updateState(BLOCKED);
      // rotateByDegrees(95);
      // idleWhileRotating();
      // driveUnchecked(30);
      // rotateToAngle(-90);
      // idleWhileRotating();
      // driveUnchecked(60);
      // rotateByDegrees(-90);
      // idleWhileRotating();
      // driveUnchecked(30);
      // rotateByDegrees(90);
      // idleWhileRotating();
    }
  }
}

/**
 * function to store the value of the current state and update it to the new input value
 */
void updateState(int NEW_STATE){
  LAST_STATE = CURRENT_STATE;
  CURRENT_STATE = NEW_STATE;
}

/**
 * function to revert to last known state. in the edge case where two states took over and we lost touch on normal, just set to normal
 */
void revertState(){
  CURRENT_STATE = LAST_STATE;
  LAST_STATE = 0;
  if(CURRENT_STATE == 0){
    CURRENT_STATE = NORMAL;
  }
}

/**
 * function to record IR reflectance values for each degree of rotation and store in the irMAP array
 */
void pollIRValueRadially(){
  int encoderCountLeft = getEncoderData(LEFT);
  if(encoderCountLeft % 10 == 0){
    std::array<int, 3> irValues = getIRValues();
    irMAP[encoderCountLeft/10] = (irValues[0] +irValues[1] + irValues[2])/3; //averaging the IR values between all three sensors smooths out the plateau into a really nice peak
  }
}

/**
 * calculate and return the most likely direction of continued travel based on date in the irMAP
 */
int getHeadingFromirMAP(){
  int peaksCounter = 0;
  int highestIndex = 180;
  int highestValue = 0;

  std::array<int, 360> peakValues;//shouldn't ever have more than a couple peakValues, bumping this up a lot just to prevent a crash
  peakValues.fill(180);//fill with the backwards direction as this will be lowest priority but is logically a safe bet.

  // for(int i = 5; i<354; i++){
  //   Serial.printf("%i, %i\n", i, irMAP[i]);
  // }
  for(int i = 0; i<360; i++){
    //Serial.printf("for irMAP index %i\n", i);
    //generate map indicies
    int mapIndices[11];
    for(int j = -5; j < 6; j++){
      int absoluteIndex = i+j;
      if(absoluteIndex < 0){
        mapIndices[j+5] = 360+absoluteIndex;//360-negative = less than 360
      }
      else if(absoluteIndex > 359){
        mapIndices[j+5] = absoluteIndex - 360;//over 360 - 360 = low value
      }
      else{
        mapIndices[j+5] = i+j;//normal
      }
      //Serial.printf("mapIndices index %i gets value%i\n", i, mapIndices[j+5]);
    }

    int lowAve = (irMAP[mapIndices[0]]+irMAP[mapIndices[1]]+irMAP[mapIndices[2]]+irMAP[mapIndices[3]]+irMAP[mapIndices[4]])/5;
    int highAve = (irMAP[mapIndices[6]]+irMAP[mapIndices[7]]+irMAP[mapIndices[8]]+irMAP[mapIndices[9]]+irMAP[mapIndices[10]])/5;

    if(irMAP[i] > lowAve && irMAP[i] >highAve){//if current value is greater than the average of the 10 values around it then it should be considered a peak
      if(irMAP[i] >1500){//only count a potential peak if it is over 1500
        Serial.printf("During irMAP evaluation found a peak of %i at %i\n", irMAP[i], i);
        peakValues[peaksCounter] = i;
        peaksCounter++;
      }
    }
    Serial.printf("%i, %i\n", i, irMAP[i]);
  }
  //TODO: test and adjust. peak detection and filtering seems to be working pretty well now.

  for(int i = 0; i < 360; i++){
    if(peakValues[i] < 150 || peakValues[i] > 210){//only proceed if not roughly behind, default highestValue of 180 covers situation where there are no peak values outside of this range
      if(irMAP[peakValues[i]] > highestValue){
        highestValue = irMAP[peakValues[i]];//update the reflectance value
        highestIndex = peakValues[i];//update the index where the reflectance value was measured
      }
    }
  }

  offroadTimer = millis() + 1000;//give it  a second upon entering normal mode to get back on track
  offRoadTimerActive = true;

  return highestIndex;
}



