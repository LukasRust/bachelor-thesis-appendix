#include "../include/plotter.hpp"
#include "../include/matplotlib-cpp/matplotlibcpp.h"
#include <cmath>
#include <iostream>

namespace plt = matplotlibcpp;
void Plotter::responseDesired(const vector_thruster::pose::ConstPtr& msgPose)
{
  vector_thruster::pose pose;
  pose.position.x = msgPose->position.x;
  pose.position.y = msgPose->position.y;
  pose.position.z = msgPose->position.z;
  //Eigen::Vector3d origin(1.f,0.f,0.f);
  //Eigen::Vector3d to(msgPose->velocity.x, msgPose->velocity.y, msgPose->velocity.z);
  //Eigen::Quaterniond q;
  //q = q.FromTwoVectors(origin,to); //TODO check if right
  //pose.orientation.w = q.w();
  //pose.orientation.x = q.x();
  //pose.orientation.y = q.y();
  //pose.orientation.z = q.z();
  pose.rotation.x = msgPose->rotation.x;
  pose.rotation.y = msgPose->rotation.y;
  pose.rotation.z = msgPose->rotation.z;
  pose.velocity.x = msgPose->velocity.x;
  pose.velocity.y = msgPose->velocity.y;
  pose.velocity.z = msgPose->velocity.z;
  this->desiredPath.push_back(pose);
  datastarted = true;
  dataflow = true;
}

void Plotter::responseActual(const gazebo_msgs::ModelStates::ConstPtr& stateMsg)
{
  int counter = 0;
  bool found = false;
  vector_thruster::pose pose;
  while(!found)
  {
    if(stateMsg->name[counter] == "deepleng")
    {
      if(!firstPositionFound)
      {
        startPosition << stateMsg->pose[counter].position.x, stateMsg->pose[counter].position.y, stateMsg->pose[counter].position.z;
        this->firstPositionFound = true;
      }
      pose.position.x = stateMsg->pose[counter].position.x - startPosition.x();
      pose.position.y = stateMsg->pose[counter].position.y - startPosition.y();
      pose.position.z = stateMsg->pose[counter].position.z - startPosition.z();
      //pose.orientation.x = stateMsg->pose[counter].orientation.x;
      //pose.orientation.y = stateMsg->pose[counter].orientation.y;
      //pose.orientation.z = stateMsg->pose[counter].orientation.z;
      //pose.orientation.w = stateMsg->pose[counter].orientation.w;
      pose.rotation.x = stateMsg->twist[counter].angular.x;
      pose.rotation.y = stateMsg->twist[counter].angular.y;
      pose.rotation.z = stateMsg->twist[counter].angular.z;
      pose.velocity.x = stateMsg->twist[counter].linear.x;
      pose.velocity.y = stateMsg->twist[counter].linear.y;
      pose.velocity.z = stateMsg->twist[counter].linear.z;
      found = true;
    }
    else
    {
      counter++;
    }
  }
  this->actualPath.push_back(pose);
  datastarted = true;
  dataflow = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plotter");
  ros::NodeHandle n;
  Plotter p;
  ros::Subscriber subDesired = n.subscribe("desired_pose", 1000, &Plotter::responseDesired, &p);
  ros::Subscriber subActual = n.subscribe("gazebo/model_states", 1000, &Plotter::responseActual, &p);
  ros::Rate loop_rate(p.loopRate);
  while(ros::ok() && (p.dataflow || !p.datastarted))
  {
    p.dataflow = false;
    ros::spinOnce();
    loop_rate.sleep();
  }
  //Put plots here:

  std::vector<float> actual_x;
  std::vector<float> actual_y;
  std::vector<float> actual_t;
  std::vector<float> actual_v;
  float t_counter = 0.f;
  for(vector_thruster::pose pose : p.actualPath)
  {
    actual_x.push_back(pose.position.x);
    actual_y.push_back(pose.position.y);
    t_counter += 1.f;
    actual_t.push_back(t_counter);//pose.header.stamp.toSec());
    actual_v.push_back(std::sqrt(pose.velocity.x*pose.velocity.x + pose.velocity.y*pose.velocity.y + pose.velocity.z*pose.velocity.z));
  }
  float actual_length = t_counter;

  std::vector<float> desired_x;
  std::vector<float> desired_y;
  std::vector<float> desired_t_acc;
  std::vector<float> desired_t;
  std::vector<float> desired_v;
  t_counter = 0.f;
  for(vector_thruster::pose pose : p.desiredPath)
  {
    desired_x.push_back(pose.position.x);
    desired_y.push_back(pose.position.y);
    t_counter += 1.f;
    desired_t_acc.push_back(t_counter);//pose.header.stamp.toSec());
    desired_v.push_back(std::sqrt(pose.velocity.x*pose.velocity.x + pose.velocity.y*pose.velocity.y + pose.velocity.z*pose.velocity.z));
  }
  float desired_length = t_counter;
  float time_factor = actual_length/desired_length;

  for(float t : desired_t_acc)
  {
    desired_t.push_back(t*time_factor);
  }

  plt::plot(actual_x,actual_y,"r-",desired_x,desired_y, "g-");
  plt::xlabel("t");
  plt::ylabel("Y");
  plt::show();
  plt::plot(actual_t,actual_v,"r-",desired_t,desired_v, "g-");
  plt::xlabel("t");
  plt::ylabel("Y");
  plt::show();
}
