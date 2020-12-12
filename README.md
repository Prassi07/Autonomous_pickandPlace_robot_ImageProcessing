# eYantra17RobotCode
Code of E-Yantra robotics Competiton 2017

Atmega2560 is a C code to be uploaded in the FireBird Hardware. It receives strings from a remote computer and executes commands to navigate around the arena.

The python file is running on a remote computer which is connected to an overhead camera. The feed from the camera is used to detect the robot and objects on the arena.
Once all positions are known, the scripts uses A* algorithm to calculate that shortest path and guides the drop to pick up the object and then navigates it to the drop zone. 
