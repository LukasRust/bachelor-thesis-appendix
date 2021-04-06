#include "ros/ros.h"
#include "./base.hpp"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "vector_thruster/tau.h"
#include "vector_thruster/trajectory.h"
#include "vector_thruster/pose.h"
#include "gazebo_msgs/ModelStates.h"
#include <cmath>
#include <vector>


class Saver
{
public:
  int loopRate = 1;
  void responseDesired(const vector_thruster::pose::ConstPtr& msgPose);
  void responseActual(const gazebo_msgs::ModelStates::ConstPtr& stateMsg);
  std::vector<vector_thruster::pose> actualPath;
  std::vector<vector_thruster::pose> desiredPath;
  bool firstPositionFound = false;
  Eigen::Vector3d startPosition;
  bool dataflowActual = false;
  bool dataflowActualLastFrame = false;
  int dataflowActualCounter = 0;
  bool dataflowDesired = false;
  bool dataflowDesiredLastFrame = false;
  int dataflowDesiredCounter = 0;
  bool datastartedActual = false;
  bool datastartedDesired = false;
  std::string name = "_s_curve_left_horizontal";
};
