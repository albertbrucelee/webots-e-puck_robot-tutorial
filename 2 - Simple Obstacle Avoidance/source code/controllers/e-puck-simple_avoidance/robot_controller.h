
#include <stdbool.h>

void robot_controller_init();

void motor_stop();

void motor_move_forward();

void motor_rotate_right();

void motor_rotate_left();

void motor_rotate_left_in_degrees(float degrees);

bool * get_sensors_condition();

void print_sensor_values();
