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


#include "positioning_controller.h"
#include "cartesian.h"

#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/compass.h>

#define GPS_SAMPLING_PERIOD 1 //in ms
WbDeviceTag gps;

#define COMPASS_SAMPLING_PERIOD 1 //in ms
WbDeviceTag compass;

void positioningControllerInit(int time_step)
{
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

double * positioningControllerGetRobotCoordinate()
{
	return cartesianConvertVec3fToCartesianVec2f(wb_gps_get_values(gps));
}

double positioningControllerGetRobotHeading()
{
	return cartesianConvertCompassBearingToHeading(getRobotBearing());
}

double positioningControllerCalcDistanceToDestination(const double destinationCoordinate[2])
{
	const double *currentCoordinate = positioningControllerGetRobotCoordinate();
	return cartesianCalcDistance(currentCoordinate, destinationCoordinate);
}

double positioningControllerCalcThetaDotToDestination(const double destinationCoordinate[2])
{
	const double *currentCoordinate = positioningControllerGetRobotCoordinate();
	double robotHeading = positioningControllerGetRobotHeading();
	double destinationTheta = cartesianCalcDestinationThetaInDegrees(currentCoordinate, destinationCoordinate);
	return cartesianCalcThetaDot(robotHeading, destinationTheta);
}