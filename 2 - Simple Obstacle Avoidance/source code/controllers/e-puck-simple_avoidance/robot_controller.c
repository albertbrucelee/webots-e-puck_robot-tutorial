/*
 * Copyright 2019 Albert Alfrianta
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
 * Created on: 2019-10-21, Bogor, Indonesia
 * 
 * Contact: albert.brucelee@gmail.com or https://www.linkedin.com/in/albert-alfrianta/
 * 
 * Description:
 * 	Controller program for e-puck robot, simulated in webots
 * 	Written in C programming language
 * 	Function: detect obstacle, and move the robot
 * 
 */


#include "robot_controller.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

/* Motor device */
static WbDeviceTag left_motor, right_motor;

/* E-puck angular speed in rad/s */
#define MAX_SPEED 6.28

/* distance sensor */
#define NUMBER_OF_DISTANCE_SENSORS 8
static WbDeviceTag distance_sensors[NUMBER_OF_DISTANCE_SENSORS];
static const char *distance_sensors_names[NUMBER_OF_DISTANCE_SENSORS] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};

/*
 * The sensor has noise
 * So even there is no obstacle, sensor values is not zero
 * So to detect obstacle, we must use this threshold
 * Based on my experiment, the good threshold value is 140
 * Obstacle detected condition is true if the sensor values is larger then this threshold value
 * */
#define SENSOR_VALUE_DETECTION_THRESHOLD 140

/* speed of robot to spinning in place (in degrees per second) */
#define ROBOT_ANGULAR_SPEED_IN_DEGREES 283.588111888

static int TIME_STEP;

/* function to init robot controller stuff */
void robot_controller_init(int time_step)
{
	TIME_STEP = time_step;
  /* get a handler to the motors and set target position to infinity (speed control) */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  /* get a handler to the sensors */
	for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++) {
			distance_sensors[i] = wb_robot_get_device(distance_sensors_names[i]);
			wb_distance_sensor_enable(distance_sensors[i], TIME_STEP);
	}
}

static float calculate_rotation_time(float degrees)
{
	return abs(degrees) / ROBOT_ANGULAR_SPEED_IN_DEGREES;
}

/* function to stop the motor (set motor velocity to zero) */
void motor_stop() {
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}

/* function to set motor velocity to move forward */
void motor_move_forward() {
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
}

/* function to set motor velocity to rotate right in place*/
void motor_rotate_right() {
  wb_motor_set_velocity(left_motor, MAX_SPEED);
  wb_motor_set_velocity(right_motor, -MAX_SPEED);
}

/* function to set motor velocity to rotate left in place*/
void motor_rotate_left() {
  wb_motor_set_velocity(left_motor, -MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
}

void motor_rotate_left_in_degrees(float degrees) {
	motor_rotate_left();
	
	float duration = calculate_rotation_time(degrees);
	float start_time = wb_robot_get_time();
	do
	{
		wb_robot_step(TIME_STEP);
	} while (wb_robot_get_time() < start_time + duration);
	
	motor_stop();
}

/* function to get sensors condition 
 * if sensor detect obstacle, then the condition is true
 * */
bool * get_sensors_condition()
{
	static bool sensors_condition[NUMBER_OF_DISTANCE_SENSORS] = {false};
	
	for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS ; i++) {
		/*
		 * Obstacle detected condition is true if the sensor values is larger then the threshold value
		 * */
		if (wb_distance_sensor_get_value(distance_sensors[i]) > SENSOR_VALUE_DETECTION_THRESHOLD) {
			sensors_condition[i] = true;
		} else {
			sensors_condition[i] = false;
		}
	}
	
	return sensors_condition;
}

/* function to print sensors values
 * */
void print_sensor_values() {
	printf("%s sensor values: ", wb_robot_get_name());
	
	for (int i = 0; i < NUMBER_OF_DISTANCE_SENSORS ; i++) {
		printf("%d:%.3f ", i, wb_distance_sensor_get_value(distance_sensors[i]));
	}
	
	printf("\n");
}
