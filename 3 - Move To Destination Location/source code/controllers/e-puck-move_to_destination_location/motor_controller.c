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