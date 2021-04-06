#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "object_avoidance_catfish_example/collisionmsg.h"
#include "vector_thruster/tau.h"
#include "vector_thruster/trajectory.h"
#include "vector_thruster/pose.h"
#include <vector>
#include <Eigen/Dense>
#include <cmath>


struct ControlVector
{
  float x{0.f};
  float y{0.f};
  float z{0.f};
  ControlVector(){}
  ControlVector(float x_, float y_, float z_) : x(x_), y(y_), z(z_){}
  ControlVector operator+(ControlVector cv)
  {
    ControlVector result;
    result.x = this->x + cv.x;
    result.y = this->y + cv.y;
    result.z = this->z + cv.z;
    return result;
  }
  ControlVector operator-(ControlVector cv)
  {
    ControlVector result;
    result.x = this->x - cv.x;
    result.y = this->y - cv.y;
    result.z = this->z - cv.z;
    return result;
  }
  ControlVector operator*(float f)
  {
    ControlVector cv;
    cv.x = this->x * f;
    cv.y = this->y * f;
    cv.z = this->z * f;
    return cv;
  }
  void operator=(ControlVector cv)
  {
    this->x = cv.x;
    this->y = cv.y;
    this->z = cv.z;
  }
  void operator=(vector_thruster::control_vector msgCV)
  {
    this->x = msgCV.x;
    this->y = msgCV.y;
    this->z = msgCV.z;
  }
  void operator=(vector_thruster::control_vector::ConstPtr& msgCV)
  {
    this->x = msgCV->x;
    this->y = msgCV->y;
    this->z = msgCV->z;
  }
  void operator=(Eigen::Vector3f v)
  {
    this->x = v[0];
    this->y = v[1];
    this->z = v[2];
  }
  void operator=(Eigen::Vector3d v)
  {
    this->x = (float)v[0];
    this->y = (float)v[1];
    this->z = (float)v[2];
  }
  float length()
  {
    return std::sqrt((this->x*this->x) + (this->y*this->y) + (this->z*this->z));
  }
  Eigen::Vector3d toEigenVector()
  {
    Eigen::Vector3d output;
    output.x() = this->x;
    output.y() = this->y;
    output.z() = this->z;
    return output;
  }
};

struct Pose
{
  ControlVector position;
  ControlVector velocity;
  ControlVector rotation;
  ControlVector acceleration;
  double time;
  Pose(){}
  Pose(ControlVector p, ControlVector v, ControlVector r, ControlVector a) : position(p), velocity(v), rotation(r), acceleration(a){}
  Pose operator-(Pose p)
  {
    this->position = this->position - p.position;
    this->velocity = this->velocity - p.velocity;
    this->rotation = this->rotation - p.rotation;
    this->acceleration = this->acceleration - p.acceleration;
  }
  Pose operator+(Pose p)
  {
    this->position = this->position + p.position;
    this->velocity = this->velocity + p.velocity;
    this->rotation = this->rotation + p.rotation;
    this->acceleration = this->acceleration + p.acceleration;
  }
  Pose operator=(const vector_thruster::pose::ConstPtr& msgPose)
  {
    this->position = msgPose->position;
    this->velocity = msgPose->velocity;
    this->rotation = msgPose->rotation;
    this->acceleration = msgPose->acceleration;
  }
  Pose operator=(vector_thruster::pose msgPose)
  {
    this->position = msgPose.position;
    this->velocity = msgPose.velocity;
    this->rotation = msgPose.rotation;
    this->acceleration = msgPose.acceleration;
  }
};

struct Tau
{
  ControlVector translational;
  ControlVector rotational;

  Tau()
  {
    translational.x = 0.f;
    translational.y = 0.f;
    translational.z = 0.f;
    rotational.x = 0.f;
    rotational.y = 0.f;
    rotational.z = 0.f;
  }

  Tau operator+(Tau t)
  {
    Tau result;
    result.translational = this->translational + t.translational;
    result.rotational = this->rotational + t.rotational;
    return result;
  }
  Tau operator-(Tau t)
  {
    Tau result;
    result.translational = this->translational - t.translational;
    result.rotational = this->rotational - t.rotational;
    return result;
  }
  void operator=(Tau t)
  {
    this->translational = t.translational;
    this->rotational = t.rotational;
  }
  void operator=(Eigen::VectorXd v)
  {
    this->translational.x = v[0];
    this->translational.y = v[1];
    this->translational.z = v[2];
    this->rotational.x = v[3];
    this->rotational.y = v[4];
    this->rotational.z = v[5];
  }
  void operator=(const vector_thruster::tau::ConstPtr& msgTau)
  {
    this->translational = msgTau->translational;
    this->rotational = msgTau->rotational;
  }
  void operator*(float f)
  {
    this->translational = this->translational*f;
    this->rotational = this->rotational*f;
  }
};

struct Waypoint
{
  ControlVector position;
  ControlVector velocity;
  ControlVector acceleration;
  Waypoint(){}
  Waypoint(ControlVector p, ControlVector v, ControlVector a) : position(p), velocity(v), acceleration(a){}
  void operator=(Waypoint w)
  {
    this->position = w.position;
    this->velocity = w.velocity;
    this->acceleration = w.acceleration;
  }
  void operator=(Pose p)
  {
    this->position = p.position;
    this->velocity = p.velocity;
    this->acceleration = p.acceleration;
  }
};

struct Trajectory
{
  Waypoint start;
  std::vector<Waypoint> via;
  Waypoint end;
  Trajectory operator+(Waypoint w)
  {
    this->via.push_back(w);
  }
  /*Trajectory operator--()
  {
    this->via.pop_front();
  }
  Trajectory operator-(Waypoint w)
  {
    this->via.erase(w);
  }*/
};

float sign(float f)
{
  if(f > 0.0f)
    return 1.f;
  if(f < 0.0f)
    return -1.f;
  else
    return 0.0;
};

vector_thruster::tau fillTauMsg(vector_thruster::tau msg, Tau tau)
{
  msg.translational.x = tau.translational.x;
  msg.translational.y = tau.translational.y;
  msg.translational.z = tau.translational.z;
  msg.rotational.x = tau.rotational.x;
  msg.rotational.y = tau.rotational.y;
  msg.rotational.z = tau.rotational.z;
  return msg;
}
