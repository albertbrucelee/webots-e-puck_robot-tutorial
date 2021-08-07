#include <stdio.h>
#include <stdlib.h>

#include "e-puck-move_to_destination_location.h"
#include "motor_controller.h"
#include "positioning_controller.h"
#include "cartesian.h"

#include <webots/robot.h>

// tangensial/linear speed in m/s. 
// Tangensial speed = angular speed * wheel radius 
// Tangensial speed = 6.28 rad * 2.05 cm = 0.12874 m/s
#define TANGENSIAL_SPEED 0.12874

// Speed of robot to spinning in place (in cycles per second)
// 1 cycle = 360 degrees. 
// Robot rotational speed = tangensial speed / (phi * axle length) 
// note: axle length is distance between wheels
// Robot rotational speed = 0.12874 / (phi*0.052) = 0.787744755
#define ROBOT_ROTATIONAL_SPEED 0.772881647

// Speed of robot to spinning in place (in degrees per second)
// Robot angular speed in degrees = robot rotational speed * 360 
// Robot angular speed in degrees = 0.787744755*360 = 283.588111888 
#define ROBOT_ANGULAR_SPEED_IN_DEGREES 278.237392796

int time_step;

int getTimeStep()
{
    static int time_step = -1;
    if (time_step == -1)
        time_step = (int)wb_robot_get_basic_time_step();
    return time_step;
}

void step()
{
    if (wb_robot_step(time_step) == -1)
    {
        wb_robot_cleanup();
        exit(EXIT_SUCCESS);
    }
}

static void init()
{
	time_step = getTimeStep();
	
	motorControllerInit(time_step);
	
	positioningControllerInit(time_step);
	
    step();
}

void rotateHeading(const double thetaDot)
{
	// if thetaDot is zero
	if (!cartesianIsThetaEqual(thetaDot, 0))
	{
		// the duration required for the robot to rotate the body by the specified thetaDot
		double duration = abs(thetaDot) / ROBOT_ANGULAR_SPEED_IN_DEGREES;
		printf("duration to face the destination: %.5f\n", duration);

		// if thetaDot > 0, robot will rotate to left
		if (thetaDot > 0)
		{
			// set robot motor to rotate left
			motorRotateLeft();
		}
		// if thetaDot < 0, robot will rotate to right
		else if (thetaDot < 0)
		{
			// set robot motor to rotate right
			motorRotateRight();
		}

		// run the simulator
		double start_time = wb_robot_get_time();
		do
		{
			step();
		}
		while (wb_robot_get_time() < start_time + duration);
	}
}

void moveForward(double distance) 
{
	// the duration required for the robot to move by the specified distance
	double duration = distance / TANGENSIAL_SPEED;
	printf("duration to reach target location: %.5f\n", duration);

	// set robot motor to move forward
	motorMoveForward();

	// run the simulator
	double start_time = wb_robot_get_time();
	do
	{
		step();	
	}
	while (wb_robot_get_time() < start_time + duration);
	
	// stop the motor
    motorStop();
	step();
}

void moveToDestination(const double destinationCoordinate[2])
{
	double * currentCoordinate = positioningControllerGetRobotCoordinate();
	printf("Initial Coordinate: %.5f %.5f\n", currentCoordinate[0], currentCoordinate[1]);

	printf("Destination Coordinate: %.5f %.5f\n", destinationCoordinate[0], destinationCoordinate[1]);
	
	// if the robot is already at the destination location
	if (cartesianIsCoordinateEqual(positioningControllerGetRobotCoordinate(), destinationCoordinate))
	{
		printf("Robot is already at the destination location\n");
		return;
	}

	// thetaDot is the degree of rotation needed by the robot to face the destination
	// thetaDot is zero if robot is facing the destination
	double thetaDotToDestination = positioningControllerCalcThetaDotToDestination(destinationCoordinate);
	printf("thetaDotToDestination: %.5f\n", thetaDotToDestination);

	rotateHeading(thetaDotToDestination);

	// the distance needed for the robot to reach its destination
	double distanceToDestination = positioningControllerCalcDistanceToDestination(destinationCoordinate);
	printf("distanceToDestination: %.5f\n", distanceToDestination);
	
	moveForward(distanceToDestination);

	currentCoordinate = positioningControllerGetRobotCoordinate();
	printf("Stop Coordinate: %.5f %.5f\n", currentCoordinate[0], currentCoordinate[1]);
}

int main(int argc, char **argv)
{
    wb_robot_init();

    init();
	
    const double destinationCoordinate[2] = {0.35, -0.35};
    
    moveToDestination(destinationCoordinate);
    
	wb_robot_cleanup();
    return EXIT_SUCCESS;
}