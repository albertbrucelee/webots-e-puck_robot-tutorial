/*
 * Copyright 2021 Albert Alfrianta
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * 
 * Created on: 2021-08, Bogor, Indonesia
 * 
 * Contact: albert.alfrianta@gmail.com or https://www.linkedin.com/in/albert-alfrianta/
 * 
 * Description:
 * 	Please read the header file for the method explanations.
 */


#include <stdio.h>
#include <stdlib.h>

#include "e-puck-adaptive_movement.h"
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

/*
Method to get simulator time step.
Webots have simulator time step. 
The basic time step is the time step increment used by Webots to advance the virtual time and perform physics simulation.
*/
int getTimeStep()
{
    static int time_step = -1;
    if (time_step == -1)
        time_step = (int)wb_robot_get_basic_time_step();
    return time_step;
}

/*
This command is to perform simulation steps. 
This needed for the controller time step. 
The controller time step is the time increment of time executed at each iteration of the control loop of a controller. 
We must call this to synchronize our program and the simulator condition. 
It will return -1 if the simulation is stopped. 
If we not call this command, the robot will do nothing. 
For example the wb_motor_set_velocity(left_motor, MAX_SPEED) only set the motor speed value. 
So we need to call and looping the wb_robot_step(time_step) command to make the robot move.
*/
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

void adaptiveMoveToDestination(const double destinationCoordinate[2])
{
	// if the robot is already at the destination location
	if (cartesianIsCoordinateEqual(positioningControllerGetRobotCoordinate(), destinationCoordinate)) {
		step();
		return;
	}

	printf("Moving robot\n");

	// while robot not arrived
	while (!cartesianIsCoordinateEqual(positioningControllerGetRobotCoordinate(), destinationCoordinate))
	{
		double thetaDotToDestination = positioningControllerCalcThetaDotToDestination(destinationCoordinate);

		if (cartesianIsThetaEqual(thetaDotToDestination, 0)) {
			motorMoveForward();
		} else if (thetaDotToDestination > 0) {
			motorRotateLeft();
		} else {
			motorRotateRight();
		}

		step();
	}

	motorStop();
	step();

	double * currentCoordinate = positioningControllerGetRobotCoordinate();
	printf("Stop coordinate: %.5f %.5f\n\n", currentCoordinate[0], currentCoordinate[1]);
}

int main(int argc, char **argv)
{
    wb_robot_init();

    init();
	
    const double destinationCoordinate[2] = {0, 0};
	printf("Destination coordinate: %.5f %.5f\n", destinationCoordinate[0], destinationCoordinate[1]);

	double * initialCoordinate = positioningControllerGetRobotCoordinate();
	printf("Initial coordinate: %.5f %.5f\n", initialCoordinate[0], initialCoordinate[1]);
	printf("\n");
	
	while (true) {
    	adaptiveMoveToDestination(destinationCoordinate);
	}
    
	wb_robot_cleanup();
    return EXIT_SUCCESS;
}