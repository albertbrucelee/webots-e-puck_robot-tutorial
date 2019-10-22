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
 * 	Simple avoidance program, implement in e-puck robot, simulated in Webots Simulator
 * 	Written in C programming language
 * 
 */


#include <stdio.h>

#include <webots/robot.h>

#include "robot_controller.h"

int TIME_STEP;

/* function to init robot stuff */
static void init_robot() {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  /* get simulator time step */
	TIME_STEP = (int)wb_robot_get_basic_time_step();
	
	/* init the controller */
	robot_controller_init(TIME_STEP);	
}

/* main function */
int main(int argc, char **argv) {
  init_robot();
  
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {		
		//print_sensor_values();
		
		bool *is_sensors_active = get_sensors_condition();
		
		if (is_sensors_active[1] && is_sensors_active[6]) {
			motor_rotate_left_in_degrees(180);
		} else if (is_sensors_active[0] || is_sensors_active[1]) {
			motor_rotate_left();
		} else if (is_sensors_active[7] || is_sensors_active[6]) {
			motor_rotate_right();
		} else {
			motor_move_forward();
		}
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
