#include "robot_controller.h"
#include "cartesian.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/nodes.h>
#include <webots/gps.h>
#include <webots/compass.h>

#define MAX_SPEED 6.28 //angular speed in rad/s
static WbDeviceTag left_motor, right_motor;

#define GPS_SAMPLING_PERIOD 1 //in ms
WbDeviceTag gps;

#define COMPASS_SAMPLING_PERIOD 1 //in ms
WbDeviceTag compass;

void robotControllerInit(int time_step)
{
    // get a handler to the motors and set target position to infinity (speed control).
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);
    
    // get a handler to the gps
    gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, GPS_SAMPLING_PERIOD);
	
    // get a handler to the compass
	compass = wb_robot_get_device("compass");
    wb_compass_enable(compass, COMPASS_SAMPLING_PERIOD);
}

/*
 * copied from: https://cyberbotics.com/doc/reference/compass
 * */
double getRobotBearing()
{
    /* calculate bearing angle in degrees */
    const double *north = wb_compass_get_values(compass);
    double rad = atan2(north[0], north[2]);
    double bearing = (rad - 1.5708) / M_PI * 180.0;
    if (bearing < 0.0) {
        bearing = bearing + 360.0;
	}
	
    return bearing;
}

double * robotControllerGetRobotCoordinate()
{
	return convertVec3fToCartesianVec2f(wb_gps_get_values(gps));
}

double robotControllerGetRobotHeading()
{
	return convertCompassBearingToHeading(getRobotBearing());
}

void motorStop()
{
    wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 0);
}

void motorMoveForward()
{
    wb_motor_set_velocity(left_motor, MAX_SPEED);
    wb_motor_set_velocity(right_motor, MAX_SPEED);
}

void motorRotateLeft()
{
    wb_motor_set_velocity(left_motor, -MAX_SPEED);
    wb_motor_set_velocity(right_motor, MAX_SPEED);
}

void motorRotateRight()
{
    wb_motor_set_velocity(left_motor, MAX_SPEED);
    wb_motor_set_velocity(right_motor, -MAX_SPEED);
}