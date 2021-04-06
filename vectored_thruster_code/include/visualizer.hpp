#include "ros/ros.h"
#include "./base.hpp"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Path.h"
#include "object_avoidance_catfish_example/collisionmsg.h"
#include "vector_thruster/tau.h"
#include "vector_thruster/trajectory.h"
#include "vector_thruster/pose.h"
#include "gazebo_msgs/ModelStates.h"
#include <cmath>
#include <vector>

class Visualizer
{
public:
  int loopRate = 1;
  void responseDesired(const vector_thruster::pose::ConstPtr& msgPose);
  void responseActual(const gazebo_msgs::ModelStates::ConstPtr& stateMsg);
  nav_msgs::Path actualPath;
  nav_msgs::Path desiredPath;
  bool firstPositionFound = false;
  Eigen::Vector3d startPosition;
};
