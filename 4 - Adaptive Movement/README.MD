# Adaptive Movement

This repository contain the program to move robot to destination location adaptively, implemented on e-puck robot and simulated on webots robot simulator.
Additional modules added to robot: GPS, Compass
The gps coordinate system is using local coordinate system.

The source code is written in C programming language.

## Tutorial
For the tutorial about how the algorithm work, please read it in my Medium [here](https://medium.com/@albert.brucelee).

## Source Code
This is the program structure:
* e-puck-adaptive_movement.c : This program is the main controller, contain the robot logic.
* motor_controller.c : This program contain the controller to handle the robot motor.
* positioning_controller.c : This program contain the controller to handle the robot positioning (by using gps and compass).
* cartesian.c : This program contain helpers for cartesian calculation.

For more code explanations, please read the comments in the header files (.h).

## Demo
Watch the demo on my youtube [here](https://www.youtube.com/watch?v=KOuOE1j2tc4).
Or open the video inside the "documentation" folder.

## Other Tutorials
This tutorial is a part of Webots tutorial series. 
Visit my Medium [here](https://medium.com/@albert.brucelee) for other series. 
Go to this repository root for the source codes. 

## Question?

If you have any question, feel free to contact me by email or [Linkedin](https://www.linkedin.com/in/albert-alfrianta/)

## License

```
Copyright 2021 Albert Alfrianta

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```


