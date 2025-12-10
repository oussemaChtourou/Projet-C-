# Projet-C-
This project simulates an autonomous robot navigating a dynamically generated, complex environment using the A* search algorithm for optimal pathfinding and a Right-Hand Rule (Wall Following) strategy for real-time obstacle evasion.

Features:
 - Dynamic Map Generation: Creates a maze-like environment with 9-cell-wide "roads" and static obstacles, ensuring a solvable path from start (3) to goal (4).

 - A* Pathfinding: Implements the A* algorithm with a clearance heuristic to find the shortest path while respecting the robot's physical size.

 - Robot Body Simulation (5x5): The robot occupies a 5×5 cell area.

- Front Sensor Simulation (5x3): A sensor detects obstacles up to 3 cells in front of the robot to initiate evasion early.

- Obstacle Evasion: When an obstacle is detected, the robot switches to a wall-following mode (Right-Hand Rule) and is temporarily allowed to use the designated Unauthorized Zone to maneuver around the block.

- Command Queues: Uses separate Normal and Urgent command queues for movement instructions (AVANCER, TOURNER_DROITE, ARRET_URGENCE),

PPM Image Output: Generates three visual maps for analysis:

- map.ppm: The initial generated environment.

- map1.ppm: The environment with the calculated A* path.

- map2.ppm: The final map showing the robot's actual 5×5 path, including detours taken during evasion.

The project is divided into three main files:
- main.c: Controls the simulation flow
- map.h : Header file defining all data structures 
- map.c : Contains all the implementation logic

sources:
image processing : https://www.youtube.com/watch?v=LRajghjH5BI&t=1425s - https://www.youtube.com/watch?v=dCqIKDo_Hkc
random numbers : https://www.youtube.com/watch?v=_wWfHRbzdX4
timestamp : https://stackoverflow.com/questions/9596945/how-to-get-appropriate-timestamp-in-c-for-logs - https://www.geeksforgeeks.org/c/time-function-in-c/
A* Algorithm : https://www.codegenes.net/blog/a-star-heuristic-for-multiple-goals/#7-implementation-considerations-and-optimizations
