#include <stdbool.h>

void robotControllerInit();

double getRobotBearing();

double * robotControllerGetRobotCoordinate();

double robotControllerGetRobotHeading();

void motorStop();

void motorMoveForward();

void motorRotateLeft();

void motorRotateRight();

void motorTurnLeft();

void motorTurnRight();