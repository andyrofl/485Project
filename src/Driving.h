/**
 * Header file for Driving related functions
 */

/**
 * function definitions
 */
void initDriving();

void rotateByDegrees(int degreesToRotate);

void rotateToAngle(int degreesTarget);

void rotateForCalibration();

void driveUnchecked(int distance);

void setDrivingVars(int lineReading, int micVal0, int micVal1, int micVal2);

void enableMovement();

void disableMovement();

bool continueRotating(int leftEncoderData, int RightEncoderData);

void updateMovingAverages(int micVal0, int micVal1, int micVal2);