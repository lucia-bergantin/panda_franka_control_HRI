//Lucia Bergantin

#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <library_panda.h>
#include "common.h"
#include <tf/transform_listener.h>
//Franka
//Robot
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
//Gripper
#include <franka/gripper_state.h>
#include <franka_gripper/franka_gripper.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>


//Namespace
using namespace ros; 
using namespace std;

///////////////////////////////////////////GLOBAL VARIABLES/////////////////////////////////////////////////////////////////////////////////////////////////////////

double speed = 0.2;
//known positions
array<double, 7> home_joint_pose = {{0.0243574,-0.821853,-0.0464922,-2.08663,-0.0316816,1.27753,0.789403}}; 
//known transformations
array<double, 3> home_cart_pos = {{0.256695,-0.0179287,0.59324}}; 
//time movement 
double t_int = 4.5; 
double t_predicted; 
int check_pick; 
int check_hand; 

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {
  if (argc != 2) {
    cerr << "Usage: " << argv[0] << " <robot-ip> " << endl; 
    return -1;
  }
  
  // ROS Initialization
  ros::init(argc, argv, "contr_panda");
  ROS_INFO("Node contr_panda connected to roscore");
  ros::NodeHandle node_handle("~");//ROS Handler - local namespace.
  ros::NodeHandle nh_global;//ROS Handler - global.
  
  //Variables initialization
  array<double, 7> initial_pose;
  //Listener initialization (to the transform between base and marker)
  tf::TransformListener listener; 
  //marker position retrieved from the transformation
  array<double, 3> marker_position_handover; //position where the handover takes place (marker_8)
  array<double, 3> marker_position_pickandplace;  //position close to the pick&place position (marker_10)
  array<double, 3> pickandplace_position; //position of pick&place
  array<double, 3> marker_position_give_back; //position where the robot grasps the object (marker_8)
  array<double, 3> marker_position_place; //position where the robot releases the object (marker_9)
  //grasping variables
  double grasping_width = 0.02; //0.033;
  bool homing = true;
  bool detected; 
  //variables definition for handover
  Eigen::VectorXd forces_handover(7);
  Eigen::VectorXd forces_give_back(7);
  Eigen::VectorXd threshold(3);
  threshold << 0.8, 0.8, 0.3; //to be modified 
  
  try {
    ///////////////////////////////////////////////INITIALIZATION////////////////////////////////////////////////////////////
    
    //Declaration of  robot variables
    franka::Robot robot(argv[1]); 
    franka::RobotState robot_state;
    franka::Model model(robot.loadModel());
    //Definition of gripper variables
    franka::Gripper gripper(argv[1]);
    franka::GripperState gripper_state = gripper.readOnce();

    //Set collision behavior: this function sets the boundaries for torques and forces
    setDefaultBehavior(robot);
    collisionBehavior(robot);

    //Print that the motion is starting
    cout << "Joint motion: This node will move the robot! Press Enter to continue."<< endl;
    cin.ignore();
    cout << "Moving to home pose and checking the gripper."<< endl;
    
    //Read the state of the robot and initialize motion
    initial_pose = robot_state.q;
    MotionGenerator motion_generator(speed, initial_pose); 
     
    //PREPARATION FOR CONTROL
    
    //Move to home pose if not already
    /*if( initial_pose != home_joint_pose){
     CartesianVelocity(robot, home_cart_pos, t_int); 
    }
    // Do a homing in order to estimate the maximum grasping width with the current fingers
    if (homing) {      
      gripper.homing(); 
    }
    //Open gripper if it is not yet open
    if(gripper_state.width != gripper_state.max_width){
       gripper.move(gripper_state.max_width, 0.1);  
    }*/

   //////////////////////////////////////DETECT PRESENCE OF HUMAN OPERATOR/////////////////////////////////////////////////

    //Detect the marker and its position
    ros::Time start_time_waitforhuman = ros::Time::now();
    ros::Duration timeout_waitforhuman(20.0); 
    detected = false; 
    listener.clear(); 
    //Detect base-marker_8 transform
    while(ros::Time::now() - start_time_waitforhuman < timeout_waitforhuman && detected != true) {
      ros::spinOnce();
       tf::StampedTransform transform_waitforhuman;
       try{
         //Use of tf listener to read the transform between base and marker_8 (human operator)
         listener.waitForTransform("/panda_link0", "/ar_marker_8",
                              ros::Time::now(), ros::Duration(1.0));
         listener.lookupTransform("/panda_link0", "/ar_marker_8",
                                  ros::Time(0), transform_waitforhuman);
         detected = true;  
        }
       catch (tf::TransformException &ex) {
         ROS_ERROR("%s",ex.what());
         ros::Duration(1.0).sleep();
         continue;
       }
    } 
    //Check if a human operator is detected and act accordingly
    if(detected == false){
        cout << "No human operator detected. No work today! Bye bye" << endl; 
        return -1; 
    } 
    cout << "Human operator detected. Let's start working!" << endl; 

   ////////////////////////////////////////PICK OBJECT 1////////////////////////////////////////////////
    
    //Detect the marker and its position
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(2.0); 
    detected = false; 
    listener.clear(); 
    //Detect base-marker_9 transform
    while(ros::Time::now() - start_time < timeout) {
      ros::spinOnce();
       tf::StampedTransform transform;
       try{
         //Use of tf listener to read the transform between base and marker_9 (pick pose)
         listener.waitForTransform("/panda_link0", "/ar_marker_10",
                              ros::Time::now(), ros::Duration(1.0));
         listener.lookupTransform("/panda_link0", "/ar_marker_10",
                                  ros::Time(0), transform);
         detected = true; 
        //Set position of marker (15 cm above it on the z axis)
        marker_position_pickandplace[0] =  transform.getOrigin().x() + 0.01;
        marker_position_pickandplace[1] =  transform.getOrigin().y() - 0.08;
        marker_position_pickandplace[2] =  transform.getOrigin().z() + 0.106;
       }
       catch (tf::TransformException &ex) {
         ROS_ERROR("%s",ex.what());
         ros::Duration(1.0).sleep();
         continue;
       }
    }
    //Check if pick pose detected and act accordingly
    if(detected == false){
        cout << "No pick position detected. No work today! Bye bye" << endl; 
        return -1; 
    } 
    
    //Pick first bar
    check_pick = PickObject(robot, argv, marker_position_pickandplace);
    if (check_pick == -1){
      return -1;
    }
    
    //////////////////////////////////////HANDOVER 1////////////////////////////////////////////////
    
    check_hand = Handover(robot, argv, detected, threshold ); 
    if(check_hand == -1 ){
      return -1; 
    } 

    ///////////////////////////////////PICK OBJECT 2///////////////////////////////////////////////////////////// 

    //Define new pick pose
    marker_position_pickandplace[1] -=  0.11; //Da determinare: indicativamente 5 cm in meno
    //Pick second bar
    check_pick = PickObject(robot, argv, marker_position_pickandplace );
    if (check_pick == -1){
      return -1;
    }
    
    //////////////////////////////////////HANDOVER 2////////////////////////////////////////////////
    
    check_hand = Handover(robot, argv, detected, threshold ); 
    if(check_hand == -1 ){
      return -1; 
    } 

    ////////////////////////////////////////PICK OBJECT 3////////////////////////////////////////////////////////
     //Define new pick pose
    marker_position_pickandplace[1] -=  0.11; //Da determinare: indicativamente 5 cm in meno
    //Pick third bar
    check_pick = PickObject(robot, argv, marker_position_pickandplace);
    if (check_pick == -1){
      return -1;
    }
    
    //////////////////////////////////////HANDOVER 3////////////////////////////////////////////////
    
    check_hand = Handover(robot, argv, detected, threshold ); 
    if(check_hand == -1 ){
      return -1; 
    } 
    /////////////////////////////////////////END//////////////////////////////////////////////////////
    cout << endl << "Finished work for today, shutting down" << endl;
  
  } catch (const franka::Exception& e) {
    cout << e.what() << endl;
    return -1;
  }
  ROS_INFO("ROS-Node Terminated\n");
  return 0;
}

