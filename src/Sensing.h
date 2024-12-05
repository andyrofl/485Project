/**
 * Header file for Sensing related functions
 */
#include <array>//include array to return sensor values

//define constants for left and right encoders
const int LEFT = 20;
const int RIGHT = 30;
//define constants for audio sensors. starting from front right and moving clockwise
const int MIC_FRONT_RIGHT = 0;
const int MIC_REAR = 1;
const int MIC_FRONT_LEFT = 2;

/**
 * function definitions
 */
void initSensing();

void resetTickCounts();

double getDistanceValue();

std::array<int, 3> getMicValues();

int getLinePosition();

std::array<int, 3> getIRValues();

int getEncoderData(int encoderID);

bool irValOffroad(int irVal1, int irVal3, int irVal5);