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
 * 	This program contain helpers for cartesian calculation.
 */


#include <stdbool.h>

/*
 * webots use 3D coordinate
 * here, we only use 2D coordinate
 * x axis in cartesian = x axis in webots
 * y axis in cartesian = -z axis in webots
 * */
double * cartesianConvertVec3fToCartesianVec2f(const double coordinate[3]);

double * cartesianConvertCartesianVec2fToVec3f(const double coordinate[2]);

/* 
 * in webots, heading increasement is rotate in clockwise
 * in cartesian, heading increasement is rotate in counterclockwise
 * 
 * in webots, heading is 0 if robot faced to y+ axis
 * in cartesian, heading is 0 if robot face to x+ axis
 * */
double cartesianConvertCompassBearingToHeading(double bearing);

/* 
 * in real world, there must be gps noise so the coordinate is not accurate
 * so to check if two coordinate are equal, we cannot check with formula: coordinate1==coordinate2
 * we must use a threshold accuracy
 * */
bool cartesianIsCoordinateEqual(const double coordinate1[2], const double coordinate2[2]);

/*
 * same with isCoordinateEqual function
 * but this function only to check the single axis
 * */
bool cartesianIsCoordinateVectorEqual(const double coordinate1Vector, const double coordinate2Vector);

/* 
 * in real world, there must be compass noise so the heading is not accurate
 * so to check if two heading are equal, we cannot check with formula: heading1==heading2
 * we must use a threshold accuracy
 * */
bool cartesianIsThetaEqual(const double theta, const double theta2);

/*
 * calc destination theta from robot
 * */
double cartesianCalcDestinationThetaInDegrees(const double currentCoordinate[2], const double destinationCoordinate[2]);

/*
 * thetaDot is the degree of rotation needed by the robot to face the destination
 * thetaDot is zero if robot is facing the destination
 * */
double cartesianCalcThetaDot(double heading, double destinationTheta);

/*
 * robot heading after rotated by thetaDot
 * */
double cartesianCalcRotatedThetaByThetaDot(double theta, double theta_dot);

/*
 * calc distance between two coordinate
 * */
double cartesianCalcDistance(const double currentCoordinate[2], const double destinationCoordinate[2]);
