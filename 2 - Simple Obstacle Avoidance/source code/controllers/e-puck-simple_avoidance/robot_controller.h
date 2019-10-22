
#include <stdbool.h>

void robot_controller_init();

void motor_stop();

void motor_move_forward();

void motor_rotate_right();

void motor_rotate_left();

void motor_rotate_left_in_degrees(float degrees);

void update_sensor_values();

void print_sensor_values();

bool is_obstacle_front();

bool is_obstacle_front_right();

bool is_obstacle_front_left();

bool is_obstacle_right();

bool is_obstacle_left();
