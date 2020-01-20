#include <cmath>
#include <iostream>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iterator>
//#include <tf2_bullet/tf2_bullet.h>
//#include <tf/transform_listener.h>
#include "common.h"
#include <tf/transform_listener.h>
//Franka
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/duration.h>
//Gripper
#include <franka/gripper_state.h>
#include <franka_gripper/franka_gripper.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>

//Ros
#include <ros/ros.h>

//Namespace
using namespace ros;
using namespace std;

//known transformations
array<double, 3> home = {{0.256695,-0.0179287,0.59324}}; 
double grasp = 0.02;
double t_max_long = 5;
double t_max_short = 3.5; 
double t_max_int = 4; 


//Set the collision behavior
/* This function changes the collision behavior. Set separate torque and force boundaries for acceleration/deceleration and constant velocity
   movement phases. Forces or torques between lower and upper threshold are shown as contacts in the RobotState. Forces or torques above the upper 
   threshold are registered as collision and cause the robot to stop moving. */
void collisionBehavior(franka::Robot& robot){
    std::array<double, 7> lower_torque_thresholds_nominal{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.}};
    std::array<double, 7> upper_torque_thresholds_nominal{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 7> lower_torque_thresholds_acceleration{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
    std::array<double, 7> upper_torque_thresholds_acceleration{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 6> lower_force_thresholds_nominal{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_nominal{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    std::array<double, 6> lower_force_thresholds_acceleration{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_acceleration{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    robot.setCollisionBehavior(
        lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
        lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
        lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
        lower_force_thresholds_nominal, upper_force_thresholds_nominal);
}

//Determine torques
Eigen::VectorXd DetermineForcesEndEffector(franka::Robot& robot ){ 

   //Initialization
   Eigen::VectorXd torques(7);
   Eigen::VectorXd forces(7);
   array<double, 42> jacobian_array;   
   Eigen::MatrixXd jacobian_transpose(7,6);
   Eigen::MatrixXd pinv;
   array < double, 7 > gravity_initial; 
   franka::RobotState initial_state = robot.readOnce();
   franka::Model model(robot.loadModel());
   
   //Get Jacobian between base and end-effector, compute pseudoinverse and transpose to obtain J_{T}_{-1}
    //Define Jacobian and its transpose
    jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, initial_state);
    Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
     
    // Computing the pseudoinverse of the Jacobian 
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(jacobian.transpose());
    pinv.setIdentity(jacobian.cols(), jacobian.rows());
    pinv = qr.householderQ() * pinv;
    //pinv = qr.matrixQR().topLeftCorner(jacobian.rows(),jacobian.rows()).triangularView<Eigen::Upper>().transpose().solve<Eigen::OnTheRight>(pinv);
    pinv = qr.matrixQR().triangularView<Eigen::Upper>(); 
    
    //Compute the transpose
    jacobian_transpose = pinv.transpose(); 
 
    //Definition of gravity
    gravity_initial = model.gravity(initial_state); 
 
    //Get torques on end-effector with gravity compensation
    cout<< "Grab the object and apply a force on your left."<<endl; 
    ros::Time start_time_torques = ros::Time::now();
    ros::Duration timeout_torques(0.5); 
    int count = 0;
    while(ros::Time::now() - start_time_torques < timeout_torques) {
      ros::spinOnce();
       try{
        robot.read([&count, &gravity_initial, &torques](const franka::RobotState& robot_state) {
        for(int i = 0; i < 7; i++){
          torques(i) = robot_state.tau_J[i] - gravity_initial[i] ;
        }
        return count++ < 1;
       });
       }
       catch (const std::exception& ex) {
        cout << ex.what() << std::endl;
       }
    }
    
    //Compute force on the end-effector
    forces = jacobian_transpose*torques;
    return forces; 
}

franka::JointVelocities CartesianVelocity(franka::Robot& robot, array<double, 3> goal_pose, double t_max){
    
    //Load the kinematic and dynamic model, read the robot state, initialize time_max
    franka::Model model = robot.loadModel();
    franka::RobotState robot_state = robot.readOnce();
    
    //Define the base to end effector transformation
    std::array<double, 16> oTee_array = model.pose(franka::Frame::kEndEffector, robot_state);

    //Define distance between the final and the initial position
    Eigen::VectorXd u(3);
    //cambiare i segni
    u << goal_pose[0] - oTee_array[12],goal_pose[1] - oTee_array[13], goal_pose[2] - oTee_array[14];

    //Norm of distance 
    double incr = 0 ; 
    for(int k= 0; k<3; k++){
           incr += u(k)*u(k);
    } 
    double norm_u = sqrt(incr);

    //Define unit vector t: p_d=primitive*t
    Eigen::VectorXd t_vec(3);
    t_vec << u(0)/norm_u, u(1)/norm_u, u(2)/norm_u;
  
    //Calculate parameters for primitive
    Eigen::VectorXd parameters(4);
    Eigen::VectorXd conditions(4);
    conditions << 0.0, 0.0, norm_u, 0.0;
    std::array<double, 16> T = {{0.0              , 0.0        , 0.0,   1,
                                 0.0              , 0.0        , 1    , 0.0,
                                 t_max*t_max*t_max, t_max*t_max, t_max, 1,
                                 3*t_max*t_max    , 2*t_max    , 1    , 0.0}}; 
    Eigen::Map<const Eigen::Matrix<double, 4, 4> > Mat_T(T.data());
    parameters = Mat_T.inverse()*conditions; 
    
    //Initialize position and velocity desired, error
    Eigen::VectorXd p_vel_control(3);
    Eigen::VectorXd p_vel_control_full(6);
    Eigen::VectorXd p_vel_d(3);
    Eigen::VectorXd q_vel_control(7);
    Eigen::VectorXd error(3); 
    std::array<double, 16> oTpose_array; 
    std::array<double, 42> jacobian_array;
    Eigen::MatrixXd pinv;
    
    //Initialize time and gain
    double time = 0.0; 
    double gain = 1.1; 
   
    robot.control(
        [=, &time, &u, &model, &p_vel_control_full,  &q_vel_control, &pinv, &jacobian_array, 
                                &gain, &goal_pose, &oTpose_array, &error, &p_vel_d, &p_vel_control, 
                                &parameters, &t_max, &t_vec](const franka::RobotState& robot_state, 
                                   franka::Duration period) -> franka::JointVelocities {
          
          time += period.toSec(); 

          //Calculate linear error
          oTpose_array = model.pose(franka::Frame::kEndEffector, robot_state);
          error(0) = oTpose_array[12] - goal_pose[0]; //error on x
          error(1) = oTpose_array[13] - goal_pose[1]; //error on x
          error(2) = oTpose_array[14] - goal_pose[2]; //error on x

          //Calculate velocity desired (cartesian)
          p_vel_d = t_vec*(3*parameters(3)*time*time + 2*parameters(2)*time + parameters(1)); 
          
          //Control law (cartesian) 
          p_vel_control = - gain*error + p_vel_d*0;
          p_vel_control_full << p_vel_control(0), p_vel_control(1), p_vel_control(2), 0.0, 0.0, 0.0;  
          
          //Define Jacobian and its transpose
          jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
          Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
        
          // Computing the pseudoinverse of the Jacobian (only when full rank, another procedure needed when not considering ee)
            Eigen::HouseholderQR<Eigen::MatrixXd> qr(jacobian.transpose());
            pinv.setIdentity(jacobian.cols(), jacobian.rows());
            pinv = qr.householderQ() * pinv;
            pinv = qr.matrixQR().topLeftCorner(jacobian.rows(),jacobian.rows()).triangularView<Eigen::Upper>().transpose().solve<Eigen::OnTheRight>(pinv);

          
          //Transfer in joint space
          q_vel_control = pinv*p_vel_control_full; 
          
          //Send velocities to manipulator  
          franka::JointVelocities velocities = {{q_vel_control(0), q_vel_control(1), q_vel_control(2), q_vel_control(3), 
                                                 q_vel_control(4), q_vel_control(5), q_vel_control(6)}};                                                         
          
          if(time >= 1.2*t_max){
             velocities = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
              std::cout << std::endl << "Finished motion, reached pose" << std::endl;
            return franka::MotionFinished(velocities);
            
          }
          return velocities;  
          
    });    
}

double Predictor(array<double, 3> new_position){
  //Initialization 
  bool what_to_do = false;
  double n = 5; //Number of predictions per step
  double f = 10; //Downsampling
  Eigen::MatrixXd O(2,2);
  O << 0.0, 0.0, 
       0.0, 0.0; 
  Eigen::MatrixXd I(2,2); //Identical matrix
  I << 1.0, 0.0,
       0.0, 1.0; 
  array<double, 2> vel = {{5,5}}; //velocities (to be modified)
  Eigen::VectorXd noise(2);
  noise << 0.2, 0.2; //noise on x and y (to be modified)
  double noise_d = 0.2; 
  //Initialization System Model
  double T_e = 0.01; //Sampling period system
  Eigen::MatrixXd A(2,2);
  A << 1.0, T_e, 
       0.0, 1.0; 
  /*std::array<double, 4> A_vec = {{1.0, T_e, 0.0, 1.0}};
  Eigen::Map<const Eigen::Matrix<double, 2, 2> > A(A_vec.data());*/
  Eigen::VectorXd C(2); 
  C << 1.0, 0.0; 
  Eigen::MatrixXd D(5,10);
  D <<   1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
  //Initialization Observer Model
  Eigen::MatrixXd H(2,1);
  H << 0.5750, 1.3750; 
  //Initialization Predictor Model
  double T_p = 0.02; //Sampling period predictor
  Eigen::MatrixXd A_p(2,2);
  A_p << 1.0, T_p,
         0.0, 1.0; 
  //State space matrices 
  Eigen::MatrixXd L(10,2);
  L << 1.0, 0.02,
       0.0,  1.0,
       1.0, 0.04,
       0.0,  1.0,
       1.0, 0.06, 
       0.0,  1.0,
       1.0, 0.08,
       0.0,  1.0,
       1.0,  0.1,
       0.0,  1.0;
  //Initialization x
  Eigen::VectorXd x_curr(2,1);
  x_curr << new_position[1],
                     vel[1];
  Eigen::VectorXd x_est_curr (2,1);
  x_est_curr = x_curr + noise; 
  Eigen::MatrixXd X_p_curr(10,1);
  X_p_curr << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; 
  Eigen::MatrixXd x_next(2,1);
  double x_usc_curr; 
  Eigen::MatrixXd x_est_next(2,1);
  Eigen::MatrixXd X_p_next(10,1);
  //Initialization y
  Eigen::VectorXd y_curr(2,1);
  y_curr << new_position[2],
                     vel[2];
  Eigen::VectorXd y_est_curr (2,1);
  y_est_curr = y_curr + noise; 
  Eigen::MatrixXd Y_p_curr(10,1);
  Y_p_curr << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; 
  Eigen::MatrixXd y_next(2,1);
  double y_usc_curr;
  Eigen::MatrixXd y_est_next(2,1);
  Eigen::MatrixXd Y_p_next(10,1);
  //Initialization area of interest
  double x_lin; 
  double y_lin;
  double R = 0.2; //radius of the area of interest
  Eigen::VectorXd x12(2,1);
  Eigen::VectorXd N (2,1);
  Eigen::VectorXd x1C(2,1);
  double v; 
  double w; 
  double t = 0.0; 
  //Loop
  int k = 0; 
  while(k<100 && what_to_do == false){
    //x
    // System model
    x_next = A * x_curr;
    x_usc_curr = C.dot(x_curr) + 0.01*noise_d;  
    // Luenberger Observer Model
    x_est_next = A * x_est_curr + H * (x_usc_curr - C.dot(x_est_curr));  
    // Predictor Model
    X_p_next = L * x_est_curr;
    
    // y
    // System model
    y_next = A * y_curr;
    y_usc_curr = C.dot(y_curr) + 0.01*noise_d; 
    // Luenberger Observer Model
    y_est_next = A * y_est_curr + H * (y_usc_curr - C.dot(y_est_curr));    
    // Predictor Model
    Y_p_next = L * y_est_curr;

    // Update of status: system, observer and predictor
    // x
    x_curr = x_next;
    x_est_curr = x_est_next; 
    X_p_curr = X_p_next;
    // y
    y_curr = y_next;
    y_est_curr = y_est_next; 
    Y_p_curr = Y_p_next;


    //Find the direction of the movement
    x_lin = x_curr(0) + x_curr(1)*k*T_p;
    y_lin = y_curr(0) + y_curr(1)*k*T_p;   

    // Define the area of interest 
    x12 << x_lin - x_curr(0), y_lin - y_curr(0);
    N = x12/sqrt(x12(0)*x12(0) + x12(1)*x12(1));
    x1C << home[1] - x_lin, home[2] - y_lin;
    w = N(0)*x1C(1) - N(1)*x1C(0); 
    v = fabs(w); 
  
    //Detection of intersection
    if(v <= R){
      what_to_do = true;
      t =  k*T_p; 
    } 
      k++; 
}  
  //Pass the time according to the result obtained
  //double t_long = 6;
  //double t_int = 5; 
  
  if(t >= 0 && t<=t_max_int){
    t = t_max_int;
  } else if (t>=t_max_int){
    t = t_max_long; 
  }

  return t; 

}


void MovetoPick(franka::Robot& robot, array <double, 3> marker_position_pickandplace){
    
    marker_position_pickandplace[2] -= 0.06;
    //Move to pick and place position pose 
    CartesianVelocity(robot,marker_position_pickandplace, t_max_short-0.5); 

}


int PickObject(franka::Robot& robot, char** argv, array<double, 3> marker_position_pickandplace){
    franka::Gripper gripper(argv[1]);
    franka::GripperState gripper_state = gripper.readOnce();
    
    //Move close to the marker pose
    CartesianVelocity(robot, marker_position_pickandplace, t_max_long);
    
    //Move to pick pose
    MovetoPick(robot, marker_position_pickandplace); 


    //GRASP OBJECT
    
    //Check if grasp width is too big for the gripper
    if (gripper_state.max_width < grasp) {
      std::cout << "Object is too large for the current fingers on the gripper." << std::endl;
      return -1;
    }
    // Grasp the object
    if (!gripper.grasp(grasp, 0.1, 0.1)) {
      std::cout << "Failed to grasp object." << std::endl;
      return -1;
    }
     
    //MOVE TO HOME POSE
    
    //Move to intermediate pose (a little higher in regards of the pick pose)
    CartesianVelocity(robot, marker_position_pickandplace, t_max_short);
    //Move to home pose
    CartesianVelocity(robot, home, t_max_long); 
    return 0; 
}

int Handover(franka::Robot& robot, char** argv, bool detected, Eigen::VectorXd threshold ){
    franka::Gripper gripper(argv[1]);
    franka::GripperState gripper_state = gripper.readOnce();
    Eigen::VectorXd forces_handover(3); 
    array<double, 3> new_position = home; 
    array<double, 2> check_value = {{0.0, 0.0}}; 
    double t_predicted;
    tf::TransformListener listener;  

    //Detect the marker and its position
    ros::Time start_time_handover = ros::Time::now();
    ros::Duration timeout_handover(4.0);
    //detected = false;
    listener.clear();
    
    //Detect base-marker_8 transform
    while (ros::Time::now() - start_time_handover < timeout_handover)
    {
      ros::spinOnce();
      tf::StampedTransform transform_8;
      tf::StampedTransform transform_9;
      try
      { 
            listener.waitForTransform("/panda_link0", "/ar_marker_8",
                                    ros::Time::now(), ros::Duration(0.5));
            listener.lookupTransform("/panda_link0", "/ar_marker_8",
                                   ros::Time(0), transform_8); 
                                   
            check_value[0]=  transform_8.getOrigin().x();  
            
      } catch (tf::TransformException &ex){ ; } 
      
      try
      {
            listener.waitForTransform("/panda_link0", "/ar_marker_9",
                                    ros::Time::now(), ros::Duration(0.5));
            listener.lookupTransform("/panda_link0", "/ar_marker_9",
                                   ros::Time(0), transform_9); 
           
            check_value[1] =  transform_9.getOrigin().x();
                
         
        } catch (tf::TransformException &ex){ ; }
    }
    
     
    if(check_value[0] != 0.0){
          detected = true;  
    } else if(check_value[1] != 0.0){
        detected = false; 
    } 
    
     if(detected == true ){ 
        new_position[0] += 0.25; 
        t_predicted = Predictor(new_position); 
        
        if(t_predicted <= 0){
           return -1; 
        }
        cout << "Human operator detected. Let's keep working!" << endl;
    
        CartesianVelocity(robot, new_position, t_predicted + 1); 

        //HANDOVER

        //Detect forces
        forces_handover = DetermineForcesEndEffector( robot); 
 
        //Handover: check if the force thresholds are exceeded or not
        cout <<"Handing the object to human operator."<<endl; 
        if( fabs(forces_handover(0)) > threshold(0) || fabs(forces_handover(1)) > threshold(1) || 
                                                                 fabs(forces_handover(2)) > threshold(2)){ 
            gripper.stop();
            gripper.move(gripper_state.max_width, 0.1); 
            cout << "Release object now" << endl;
            } else {
                cout <<"No human operator grasping the object."<<endl; 
                return -1;
            }

        //Move back to home pose
        CartesianVelocity(robot, home, t_max_int+2);
        return 0; 
  } else {
            return -1; 
  }
}