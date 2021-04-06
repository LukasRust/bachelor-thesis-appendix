#include "./base.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "object_avoidance_catfish_example/collisionmsg.h"
#include "vector_thruster/tau.h"
#include "gazebo_msgs/ModelStates.h"


class Controller
{
public:
  Pose actual;
  Pose desired;
  ControlVector proportional_error;
  ControlVector integral_error;
  ControlVector proportional_orientation_error;
  ControlVector integral_orientation_error;
  ControlVector proportional_position_error;
  ControlVector integral_position_error;
  float integral_speed_error;
  float kp_forward_translational = 0.7f;
  float kp_side_translational = 0.0f;

  float kp_orientationSpinPitch = 17.f;
  float kp_orientationSpinYaw = 9.f;

  float kp_positionSpinPitch = 8.f;//70.0f;
  float kp_positionSpinYaw = 4.f;//70.0f;

  float ki_forward_translational = 0.f;
  float ki_side_translational = 0.f;
  float ki_orientationSpin = 0.f;
  float ki_positionSpin = 0.f;
  Eigen::Matrix3d robotInWorld;
  Eigen::Quaterniond orientation;
  Eigen::Quaterniond W_nI_R;
  Eigen::Quaterniond R_nI_W;
  Eigen::Vector3d startPosition;
  bool firstPositionFound = false;

  /*
  float kp_position;
  float kp_euler;
  float kp_x_velocity;
  float kp_yz_velocity;
  float kp_x_acceleration;
  float kp_yz_acceleration;
  float ki_position;
  float ki_euler;
  float ki_x_velocity;
  float ki_yz_velocity;
  float ki_x_acceleration;
  float ki_yz_acceleration;
  */
  ControlVector getOrientationError();
  ControlVector getPositionError();
  void calcErrors();
  Tau getCorrectionalTau();
public:
  Tau execute();
  void updateActualTau(const vector_thruster::pose::ConstPtr& msgPose);
  void updateDesiredTau(const vector_thruster::pose::ConstPtr& msgPose);
  void updateActualPose(const gazebo_msgs::ModelStates::ConstPtr& stateMsg);
};
