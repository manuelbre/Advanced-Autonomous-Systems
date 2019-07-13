# Advanced Autonomous Systems Term 1 2019
Assignments for the UNSW class [Advanced Autonomous Systems](https://www.handbook.unsw.edu.au/undergraduate/courses/2019/MTRN4010) Term 1 2019

## Assignments
[Assignments](./assignments) contains my solution to the class assignments.
The assignments are in the realm of sensor fusion, system modeling and genetic algorithms.

In the following I will give a short overview of the tasks and show some results.
Project 1 to 3 are based on the same project. The task was to process Lidar and IMU data of a wheeled mobile robot.
First the data is clustered in project 1 to detect landmarks.
In project 2 the IMU data is used to localize the robot by integration of the sensor data.
Finally in project 3 an Extended Kalman Filter (EKF) is used for sensor fusion.
Project 4 on the other hand is independent of the other projects and is about genetic algorithms.

### Project 1
[Project 1](./assignments/project1)

### Clustering of Lidar data
The task of this projects was to preprocess bearing and range measurements of the Lidar and cluster them based on some given criteria.

![Alt text](./assignments/project1/results/plot_clustering.gif?raw=true "Results Clustering.")


### Project 2
[Project 2](./assignments/project2)

The goal of this project was to use IMU data to calculate the position of a wheeled robot.

![Alt text](./assignments/project2/results/plot_robot_pos.gif?raw=true "Path Video.")

![Alt text](./assignments/project2/results/plot_robot_pos.png?raw=true "Resulting Path")

![Alt text](./assignments/project2/results/plot_w.png?raw=true "Integrated Yawrate.")


### Project 3
[Project 3](./assignments/project3)


#### Extended Kalman Filter with Simulated Data
A simple simulation of a wheeled mobile robot has been used to test the EKF.
The EKF estimates both the robot state and the bias of the yaw rate.
In the following image one can see the difference between the simple integration and the result with the EKF.

![Alt text](./assignments/project3/results/plot_sim_path.png?raw=true "EKF Simulation Data.")

#### Extended Kalman Filter with Real Data
In this part real IMU and Lidar data of a wheeled mobile has been used to estimate the state of the robot with help of the EKF. Further the EKF estimates the Yawrate bias. In addition we removed the velocity data of the robot and estimate also the velocity.

In the following we can see that the EKF performs very well and outperforms the simple IMU data integration clearly.

![Alt text](./assignments/project3/results/plot_ekf_pos.gif?raw=true "Real Data EKF Start Time.")

### Project 4

[Project 4](./assignments/project4)

#### Genetic Algorithm for Path planning.
Path planning of a simulated mobile robot with help of genetic algorithms.The goal is to visit every landmark in a certain order such that the path has minimial length.

![Alt text](./assignments/project4/results/plot_genetic_algo.png?raw=true "Results of Genetic Algorithm Path Planning.")


#### Fuzzy Control
Fuzzy of a mobile robot. The goal is to move the robot from a start position $S$ to a target position $T$. A moving virtual target with offset is used to improve the performance.

![Alt text](./assignments/project4/results/plot_fuzzy_control.png?raw=true "Frame of Mobile Robot Simulation.")


#### Swarm Optimization
Find best hyperparameter for fuzzy control task with help Particle Swarm Optimization.

![Alt text](./assignments/project4/results/plot_particle_swarm.png?raw=true "Results of Swarm Optimization for Fuzzy Control of Mobile Robot.")
