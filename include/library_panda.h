#pragma once

#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>

//Franka
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include "common.h"


//Set the collision behavior
void collisionBehavior(franka::Robot& robot);

//Gravity compensation 
franka::JointPositions ZeroTorque(franka::Robot& robot, MotionGenerator motion_generator);

//Determine forces
Eigen::VectorXd DetermineForcesEndEffector(franka::Robot& robot );

//Cartesian velocity control (sends joint velocities!)
franka::JointVelocities CartesianVelocity(franka::Robot& robot, std::array<double, 3> goal_pose, double time_max); 

//Predictor
double Predictor(std::array<double, 3> new_position); 

//Grasp object and go home 
void MovetoPick(franka::Robot& robot, std::array <double, 3> marker_position_pickandplace); 

//Pick object from pick pose
int PickObject(franka::Robot& robot, char** argv, std::array<double, 3> marker_position_pickandplace );

//Handover sub-task
int Handover(franka::Robot& robot, char** argv, bool detected, Eigen::VectorXd threshold ); 


