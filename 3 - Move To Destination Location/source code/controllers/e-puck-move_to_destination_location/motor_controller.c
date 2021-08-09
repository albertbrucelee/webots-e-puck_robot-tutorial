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


#include "motor_controller.h"

#include <webots/robot.h>
#include <webots/motor.h>

#define MAX_SPEED 6.28 //angular speed in rad/s
static WbDeviceTag left_motor, right_motor;

void motorControllerInit(int time_step)
{
    // get a handler to the motors and set target position to infinity (speed control).
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);
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