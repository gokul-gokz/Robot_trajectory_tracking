# Robot_trajectory_tracking
### Description: 
This repository contains the code for calculating the Forward Kinematics and Inverse Kinematics of a 3 DOF ROBOT.

Loads waypoints of trajectory from a text file and calculates the trajectory connecting the waypoints in joint space.

Sends the data through socket connection and receives data from the robot through the socket connection.

### DH parameters and Joint Limits
 ![Parameter](/image/DH_table.png)
 
### Inverse Kinematics equations
 ![IK](/image/IK.png)
 
 ## Code Organization
 
 ### Header Files
 
 #### Robot.h
 Contains the class definition of the robot class
 
 #### Connection.h
 Contains the class definition of the connection class
 
 #### Trajectory.h
 Contains a function to load trajectory from text file and return as a vector
 
 ### Source files
 #### Robot.cpp
 Contains the class declaration of the robot class
 
 #### Connection.cpp
 Contains the class declafration of the comnection class
 
 #### main.cpp
 Contains the code for creating the robot object and make the robot follow a trajectory in joint space.
 
 ### Steps to run the code.
 1. Clone the repository
 2. cd cmake-build-debug
 3. Run the executable Robot
 
 ### Bugs
 1. Reading the trajectory waypoints from the text file using fstream is buggier as it is not able parse the points consistently. Will be fixed soon.
 
 
 
 
 
 
 
 
