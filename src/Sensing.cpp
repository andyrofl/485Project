#include "Arduino.h"
#include "Sensing.h"
#include <QTRSensors.h> //for line following sensor

#define TRIGGER_PIN 13
#define ECHO_PIN 14

#define MIC_PIN_0 15
#define MIC_PIN_1 16
#define MIC_PIN_2 17

#define IR_PIN_1 18
#define IR_PIN_3 19
#define IR_PIN_5 20

//declare private/helper functions
void calcPos(void);

QTRSensors qtr;
uint16_t sensorValues[3];

volatile int countMotorLeft;
volatile int countMotorRight;


//number of highs and lows per rotation of the motor shaft (PRE GEARBOX)
#define TICKS_PER_REV 48 // Number of ticks/counts per wheel revolution (this will depend on your motor)
#define PERIOD 10000     // How fast velocity is calculated in microseconds
const double PERIOD_IN_MINUTES = PERIOD / 60000000;


const double rpmConvert =  74.83;  // --taken from pololu reference data. TA says to just use 75

double IRVal1MovingAverage;
double IRVal3MovingAverage;
double IRVal5MovingAverage;

/**
 * interrupt functions for tick increments
 */
void incrementLeftCount(){
    countMotorLeft++;
}
void incrementRightCount(){
    countMotorRight++;
}

void initSensing(){
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    pinMode(MIC_PIN_0, INPUT);
    pinMode(MIC_PIN_1, INPUT);
    pinMode(MIC_PIN_2, INPUT);

    pinMode(IR_PIN_1, INPUT);
    pinMode(IR_PIN_3, INPUT);
    pinMode(IR_PIN_5, INPUT);
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){IR_PIN_1, IR_PIN_3, IR_PIN_5}, 3);

    countMotorLeft = 0;
    countMotorRight = 0;

    pinMode(1, INPUT_PULLUP);
    pinMode(2, INPUT_PULLUP);
    pinMode(4, INPUT_PULLUP);
    pinMode(5, INPUT_PULLUP);


    attachInterrupt(digitalPinToInterrupt(1), incrementLeftCount, RISING);
    attachInterrupt(digitalPinToInterrupt(2), incrementLeftCount, RISING);

    attachInterrupt(digitalPinToInterrupt(4), incrementRightCount, RISING);
    attachInterrupt(digitalPinToInterrupt(5), incrementRightCount, RISING);

    IRVal1MovingAverage = 1500;
    IRVal3MovingAverage = 1500;
    IRVal5MovingAverage = 1500;
}

/**
 * reset tick counters for left and right motor encoders
 */
void resetTickCounts(){
    countMotorLeft = 0;
    countMotorRight = 0;
}

/**
 * triggers a pulse on the Ultrasonic sensor, reads and processes the input time to return an approximate distance in meters
 * this function shouldn't need to be public or specified in header?
 */
double getDistanceValue(){
    digitalWrite(TRIGGER_PIN, HIGH);
    delay(10);
    digitalWrite(TRIGGER_PIN, LOW);
    double the_time = pulseIn(ECHO_PIN, HIGH);
    return the_time*0.0002 + 0.0069;//calculate a distance in meters and return
}

std::array<int, 3> getMicValues(){
    std::array<int, 3> micValsArray = {analogRead(MIC_PIN_0), analogRead(MIC_PIN_1), analogRead(MIC_PIN_2)};
    return micValsArray;
}

int getLinePosition(){
    return (int) qtr.readLineBlack(sensorValues);;//1000 corresponds to middle sensor
}


/**
 * function to return an array pointer of the raw IR sensor values
 */
std::array<int, 3> getIRValues(){
    qtr.read(sensorValues);//TODO: check if calling this is strictly necessary, values should be updated by readLineBlack but do not appear to be
    std::array<int, 3> sensorValsArray = {sensorValues[0], sensorValues[1], sensorValues[2]};
    return sensorValsArray;
}

int getEncoderData(int encoderID){
    if(encoderID == LEFT){
        return countMotorLeft;
    }
    else if(encoderID == RIGHT){
        return countMotorRight;
    }
    return 0;
}


bool irValOffroad(int irVal1, int irVal3, int irVal5){
    double tempIRval1 = IRVal1MovingAverage*999 + irVal1;
    IRVal1MovingAverage = tempIRval1 / 1000;

    double tempIRval3 = IRVal3MovingAverage*999 + irVal3;
    IRVal3MovingAverage = tempIRval3 / 1000;

    double tempIRval5 = IRVal5MovingAverage*999 + irVal5;
    IRVal5MovingAverage = tempIRval5 / 1000;

    return (irVal1 + 100 < IRVal1MovingAverage && irVal3 + 100 < IRVal3MovingAverage && irVal5 + 100 < IRVal5MovingAverage);
}