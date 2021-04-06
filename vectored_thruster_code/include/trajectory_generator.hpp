#include "ros/ros.h"
#include "./base.hpp"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "object_avoidance_catfish_example/collisionmsg.h"
#include "vector_thruster/tau.h"
#include "vector_thruster/trajectory.h"
#include "vector_thruster/pose.h"
#include <cmath>

class TrajectoryGenerator
{
private:
  float minRangeToWaypoint = 0.2;
  float speed = 0.2;
public:
  const int loopRate = 3;
  int counter = 0; //Zeuigt auf den Anfangspunklt in all der aktuellen situation
  Trajectory trajectory;
  ControlVector futureVelocity;
  Pose actual;
  Pose desired;
  std::vector<Waypoint> all;
  ros::Time startTime;
  ros::Time passedTime;
  float maxTime = 10.f; //to be filled!!!!
  float time = 0.f; //to be filled!!!!
  float lastTimePoint = 0.f;
  float w_dot = 0.f;
  float w = 0.f;
  float future_w = 0.f;
  bool firstFrame = true;
  void updateTime();
  void calcDesiredPose();
  void execute();
  void calcPolynom();
  void checkIfReached();
  void calcRotation();
  void calcBendTime(bool present);
  vector_thruster::pose createMsg();
  void updateModelTau(const vector_thruster::tau::ConstPtr& msgTau);
  void updateTrajectory(const vector_thruster::trajectory::ConstPtr& msgTrajectory);
  //void setTrajectory(Trajectory new);
};
