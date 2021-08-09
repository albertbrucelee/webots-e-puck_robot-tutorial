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
 * 	This program is the main controller, contain the robot logic. 
 */

/*
This is where the program begin.
*/
int main(int argc, char **argv);

/*
To move the robot to destination.
*/
void moveToDestination(const double destinationCoordinate[2]);

/*
To rotate robot heading by specified angle
*/
void rotateHeading(const double thetaDot);

/*
To move forward the robot by specified distance
*/
void moveForward(double distance);
