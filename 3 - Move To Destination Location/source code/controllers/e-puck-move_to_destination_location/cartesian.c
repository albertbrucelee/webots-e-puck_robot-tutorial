
#include "cartesian.h"

#include <stdlib.h>
#include <math.h>

/* 
 * in real world, there must be gps noise so the coordinate is not accurate
 * so to check if two coordinate are equal, we cannot check with formula: coordinate1==coordinate2
 * we must use a threshold accuracy
 * */
#define COORDINATE_MATCHING_ACCURACY 0.01 //in meter

/* 
 * in real world, there must be compass noise so the heading is not accurate
 * so to check if two heading are equal, we cannot check with formula: heading1==heading2
 * we must use a threshold accuracy
 * */
#define THETA_MATCHING_ACCURACY 1 //in degrees


double * convertVec3fToCartesianVec2f(const double coordinate3f[3]) {
    double *coordinate2f = malloc(2);
    coordinate2f[0] = coordinate3f[0];
    coordinate2f[1] = -coordinate3f[2];
    return coordinate2f;
}

double * convertCartesianVec2fToVec3f(const double coordinate2f[2]) {
    double *coordinate3f = malloc(3);
    coordinate3f[0] = coordinate2f[0];
    coordinate3f[2] = -coordinate2f[1];
    return coordinate3f;
}

double convertCompassBearingToHeading(double heading) {
    /* 
	 * in webots, heading increasement is rotate in clockwise
	 * in cartesian, heading increasement is rotate in counterclockwise
	 * so headingInCartesian = 360-headingInWebots
	 * */
    heading = 360-heading;
    
    /* 
	 * in webots, heading is 0 if robot faced to y+ axis
	 * in cartesian, heading is 0 if robot face to x+ axis
	 * so headingInCartesian = headingInWebots+90
	 * */
    heading = heading + 90;
    if (heading > 360.0)
        heading = heading - 360.0;

    return heading;
}

bool isCoordinateEqual(const double coordinate1[2], const double coordinate2[2])
{
    if (fabs(coordinate1[0]-coordinate2[0]) < COORDINATE_MATCHING_ACCURACY &&
            fabs(coordinate1[1]-coordinate2[1]) < COORDINATE_MATCHING_ACCURACY) {
        return true;
    }
    else {
        return false;
    }
}

bool isCoordinateVectorEqual(const double coordinateVector1, const double coordinateVector2)
{
    if (fabs(coordinateVector1-coordinateVector2) < COORDINATE_MATCHING_ACCURACY) {
        return true;
    }
    else {
        return false;
    }
}

bool isThetaEqual(const double theta, const double theta2)
{
    if (fabs(theta - theta2) < THETA_MATCHING_ACCURACY)
    {
        return true;
    } else
    {
        return false;
    }
}

double calcDestinationThetaInDegrees(const double currentCoordinate[2], const double destinationCoordinate[2]) {
    return atan2(destinationCoordinate[1] - currentCoordinate[1], destinationCoordinate[0] - currentCoordinate[0]) * 180 / M_PI;
}

double calcThetaDot(double heading, double destinationTheta) {
    double theta_dot = destinationTheta - heading;

    if (theta_dot > 180)
        theta_dot = -(360-theta_dot);
    else if (theta_dot < -180)
        theta_dot = (360+theta_dot);

    return theta_dot;
}

double calcRotatedThetaByThetaDot(double theta, double theta_dot)
{
	if (theta_dot == 0)
	{
		return theta;
	}

	theta += theta_dot;

	/*
	 * if theta negative or more than 360, then convert it to normal degree
	 * */
	
	if (theta < 0)
	{
		theta = theta + 360;
	}
	else if (theta >= 360)
	{
		theta = theta - 360;
	}
	
	return theta;
}

double calcDistance(const double currentCoordinate[2], const double destinationCoordinate[2]) {
    return sqrt(pow(destinationCoordinate[0]-currentCoordinate[0], 2) + pow(destinationCoordinate[1]-currentCoordinate[1], 2));
}

