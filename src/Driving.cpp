#include <Arduino.h>
#include "Driving.h"
#include "Sensing.h"

//Enable Pin nD2 not nessecarry to connect if you use a jumper
#define MotorA_DIR_PIN 10 // Direction Pin
#define MotorA_PWM_PIN 11 // PWM Pin
#define MotorB_DIR_PIN 9 // Direction Pin
#define MotorB_PWM_PIN 8 // PWM Pin make sure the PWM pins actually support that signal

#define LINE_READING_TARGET 1000
#define P_WEIGHT 0.3

int BASE_PWM = 120;
int ROTATE_PWM = 90;
bool movementEnabled;
int currentTickTarget;

double micVal0MovingAverage;
double micVal1MovingAverage;
double micVal2MovingAverage;

double axleWidth = 160;
double wheelDiameter = 80;
double wheelCircumference = M_PI_2 * (wheelDiameter/2);
double vehicleCircumference = M_PI_2 * (axleWidth/2);
double wheelRotationsPerRevolution = vehicleCircumference/wheelCircumference;

#define TICKS_PER_ENCODER_REV 48
#define GEAR_RATIO 74.85//this should actually be 74.83 but it works as is so I don't want to fuck with it


/**
 * setup vars and objects related to driving functions
 */
void initDriving(){
    movementEnabled = false;
    pinMode(MotorA_PWM_PIN, OUTPUT);
    pinMode(MotorB_PWM_PIN, OUTPUT);
    pinMode(MotorA_DIR_PIN, OUTPUT);
    pinMode(MotorB_DIR_PIN, OUTPUT);
    currentTickTarget = 0;
    micVal0MovingAverage = 200;
    micVal1MovingAverage = 200;
    micVal2MovingAverage = 200;
}

/**
 * function to rotate a radial distance equal to the input value
 * function ignores movementEnabled
 * continueRotating() must be called continuously after initating a rotate to ensure it completes
 */
void rotateByDegrees(int degreesToRotate){
    resetTickCounts();
    int ticksToRotate = abs(((TICKS_PER_ENCODER_REV*GEAR_RATIO)/360)*degreesToRotate * wheelRotationsPerRevolution/2);

    currentTickTarget = ticksToRotate;

    int dirMotorA, dirMotorB;
    if(degreesToRotate<0){
        dirMotorA = HIGH;
        dirMotorB = LOW;
    }
    else{
        dirMotorA = LOW;
        dirMotorB = HIGH;
    }

    digitalWrite(MotorA_DIR_PIN, dirMotorA);
    digitalWrite(MotorB_DIR_PIN, dirMotorB);

    analogWrite(MotorA_PWM_PIN, ROTATE_PWM);
    analogWrite(MotorB_PWM_PIN, ROTATE_PWM);
}

/**
 * function to initiate a rotation approximately 360 degrees but with an even number of encoder ticks to ensure 360 even segments of 10 ticks are created
 * function ignore movementEnabled
 * continueRotating() must be called continuously after initating a rotate to ensure it completes
 */
void rotateForCalibration(){
    currentTickTarget = 3600;

    digitalWrite(MotorA_DIR_PIN, LOW);
    digitalWrite(MotorB_DIR_PIN, HIGH);

    analogWrite(MotorA_PWM_PIN, ROTATE_PWM);
    analogWrite(MotorB_PWM_PIN, ROTATE_PWM);
}

/**
 * function to be called after a rotateByDegrees operation
 */
bool continueRotating(int leftEncoderData, int rightEncoderData){
    int tickDifference = leftEncoderData - rightEncoderData;

    analogWrite(MotorA_PWM_PIN, ROTATE_PWM+tickDifference);
    analogWrite(MotorB_PWM_PIN, ROTATE_PWM-tickDifference);

    if(leftEncoderData>=currentTickTarget){
        analogWrite(MotorB_PWM_PIN, 0);
        digitalWrite(MotorB_DIR_PIN, HIGH);
    }

    if(rightEncoderData >= currentTickTarget){
        analogWrite(MotorA_PWM_PIN, 0);
        digitalWrite(MotorA_DIR_PIN, HIGH);

        //right counter has completed, check if left has also completed to exit
        if(leftEncoderData >=currentTickTarget){
            currentTickTarget = 0;
            return true;//rotation complete
        }
    }

    return false;//rotation has not completed
}

/**
 * given a target angle, calculate the relative and initiate a turn
 * continueRotating() needs to be called continuously after initiating a rotate
 */
void rotateToAngle(int degreesTarget){
    
    if(degreesTarget>180){
        rotateByDegrees(degreesTarget-360);
    }
    else{
        rotateByDegrees(degreesTarget);
    }
}


/**
 * function to drive a specified distance in millimeters at the current heading.
 * no other logic will be processed until function completes
 */
void driveUnchecked(int distance){
    resetTickCounts();
    //set forward or reverse dependent on sign of ditance parameter
    if(distance >0){
        digitalWrite(MotorA_DIR_PIN, HIGH);
        digitalWrite(MotorB_DIR_PIN, HIGH);
    }
    else{
        digitalWrite(MotorA_DIR_PIN, LOW);
        digitalWrite(MotorB_DIR_PIN, LOW);
    }
    
    analogWrite(MotorA_PWM_PIN, ROTATE_PWM);
    analogWrite(MotorB_PWM_PIN, ROTATE_PWM);
    
    int ticksToDrive = abs((TICKS_PER_ENCODER_REV*GEAR_RATIO)* distance/wheelCircumference);
    Serial.printf("entered an unchecked drive sequence of %i mm with %i ticks calculated", distance, ticksToDrive);

    while(getEncoderData(LEFT) < ticksToDrive){
        delay(5);//wait until  drive finished
    }

    analogWrite(MotorA_PWM_PIN, 0);
    analogWrite(MotorB_PWM_PIN, 0);
}

/**
 * normalize the range of an input for a PWM controlled pin. any value beyond the bounds of 0-255 will be pulled in to the end of the range
 */
int rangePWM(int input){
    if(input > 255){
        return 255;
    }
    else if(input < 0){
        return 0;
    }
    else{
        return input;
    }
}

/**
 * function to be called continuously and fed a value from the QTR sensor.
 * logic to update PWM and driver signals contained in this function
 * 
 */
void setDrivingVars(int lineReading, int micVal0, int micVal1, int micVal2){
    int difference = LINE_READING_TARGET - lineReading;

    int pwm_left = BASE_PWM - difference*P_WEIGHT;//when difference is positive this indicates the left side of the vehicle is over the line and need to steer left to correct.
    int pwm_right = BASE_PWM + difference*P_WEIGHT;

    pwm_left = 0;
    pwm_right = 0;

    updateMovingAverages(micVal0, micVal1, micVal2);
    if(micVal0 > (int)micVal0MovingAverage+30){
        pwm_left -= 150;
        pwm_right += 150;
        Serial.println("nudge detected on front right, steering to left");
    }
    else if(micVal1 > (int)micVal1MovingAverage+30){//detected on rear, bump both speeds up
        pwm_left += 200;
        pwm_right += 200;
        Serial.println("nudge detected on rear, increasing pwm of both motors");
    }
    else if(micVal2 > (int)micVal2MovingAverage+30){//detected front left, steer more to right
        pwm_left += 150;
        pwm_right -= 150;
        Serial.println("nudge detected on front left, steering to right");
    }

    //write values
    if(movementEnabled){
        analogWrite(MotorA_PWM_PIN, rangePWM(pwm_right));
        analogWrite(MotorB_PWM_PIN, rangePWM(pwm_left));
    }
}

/**
 * sets flag to enable movement high
 */
void enableMovement(){
    movementEnabled = true;
    digitalWrite(MotorA_DIR_PIN, HIGH);
    digitalWrite(MotorB_DIR_PIN, HIGH);
}

/**
 * sets flag to enable movement low. also sets PWM of both motors to zero
 */
void disableMovement(){
    movementEnabled = false;
    digitalWrite(MotorA_PWM_PIN, LOW);
    digitalWrite(MotorB_PWM_PIN, LOW);
}

/**
 * function to update the moving averages for each mic where the weight of the current value is 1/1000 per iteration logarithmically
 */
void updateMovingAverages(int micVal0, int micVal1, int micVal2){
    double tempMicval0 = micVal0MovingAverage*999 + micVal0;
    micVal0MovingAverage = tempMicval0 / 1000;

    double tempMicval1 = micVal1MovingAverage*999 + micVal1;
    micVal1MovingAverage = tempMicval1 / 1000;

    double tempMicval2 = micVal2MovingAverage*999 + micVal2;
    micVal2MovingAverage = tempMicval2 / 1000;
}