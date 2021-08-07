#include <stdbool.h>

void positioningControllerInit();

double * positioningControllerGetRobotCoordinate();

double positioningControllerGetRobotHeading();

double positioningControllerCalcDistanceToDestination(const double destinationCoordinate[2]);

double positioningControllerCalcThetaDotToDestination(const double destinationCoordinate[2]);