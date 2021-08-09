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
 * 	This program contain the controller to handle the robot positioning (by using gps and compass).
 */


/*
To initialize all stuff.
*/
void positioningControllerInit();

/*
To get robot coordinate by using gps.
*/
double * positioningControllerGetRobotCoordinate();

/*
To get robot heading by using compass.
0 degree = robot faced to x+ axis
90 degree = robot faced to y+ axis
180 degree = robot faced to x- axis
270 degree = robot faced to y- axis
*/
double positioningControllerGetRobotHeading();

/*
To calculate distance from robot coordinate to destination coordinate.
*/
double positioningControllerCalcDistanceToDestination(const double destinationCoordinate[2]);

/*
To calculate the angle required for the robot to rotate its body to face the destination.
*/
double positioningControllerCalcThetaDotToDestination(const double destinationCoordinate[2]);