#include "../include/saver.hpp"
//#include "../include/matplotlib-cpp/matplotlibcpp.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

void Saver::responseDesired(const vector_thruster::pose::ConstPtr& msgPose)
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
  pose.time = ros::Time::now().toSec();
  this->desiredPath.push_back(pose);
  datastartedDesired = true;
  dataflowDesiredLastFrame = true;
}

void Saver::responseActual(const gazebo_msgs::ModelStates::ConstPtr& stateMsg)
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
      pose.time = ros::Time::now().toSec();
      found = true;
    }
    else
    {
      counter++;
    }
  }
  this->actualPath.push_back(pose);
  datastartedActual = true;
  dataflowActualLastFrame = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "saver");
  ros::NodeHandle n;
  Saver s;
  ros::Subscriber subDesired = n.subscribe("desired_pose", 1000, &Saver::responseDesired, &s);
  ros::Subscriber subActual = n.subscribe("gazebo/model_states", 1000, &Saver::responseActual, &s);
  ros::Rate loop_rate(s.loopRate);
  while(ros::ok() && ((s.dataflowActual && s.dataflowDesired) || (!s.datastartedActual || !s.datastartedDesired)))
  {
    if(!s.dataflowActualLastFrame && s.datastartedActual)
      s.dataflowActualCounter++;
    else
      s.dataflowActualCounter = 0;

    if(!s.dataflowDesiredLastFrame && s.datastartedDesired)
      s.dataflowDesiredCounter++;
    else
      s.dataflowDesiredCounter = 0;

    if(s.dataflowActualCounter >= 2)
      s.dataflowActual = false;
    else
      s.dataflowActual = true;

    if(s.dataflowDesiredCounter >= 3)
      s.dataflowDesired = false;
    else
      s.dataflowDesired = true;

    s.dataflowActualLastFrame = false;
    s.dataflowDesiredLastFrame = false;
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("Start saving");
  //----------------actual trajectories------------------------------------------------------
  std::ofstream csvfileactual;
  csvfileactual.open("/home/lukas/catkin_ws/src/vector_thruster/files/thesis/actual_trajectories_"+s.name+".csv");
  if( !csvfileactual ) { // file couldn't be opened
      std::cerr << "Error: file could not be opened" << "/n";
      exit(1);
  }
  csvfileactual << "actual_trajectories in format position.x ; position.y ; position.z ; velocity.x ; velocity.y ; velocity.z ; angular.x ; angular.y ; angular.z";
  for(vector_thruster::pose pose : s.actualPath)
  {
    csvfileactual << std::to_string(pose.position.x) + ";" + std::to_string(pose.position.y) + ";" + std::to_string(pose.position.z) + ";" + std::to_string(pose.velocity.x) + ";" + std::to_string(pose.velocity.y) + ";" + std::to_string(pose.velocity.z) + ";" + std::to_string(pose.rotation.x) + ";" + std::to_string(pose.rotation.y) + ";" + std::to_string(pose.rotation.z) + ";" + std::to_string(pose.time) << std::endl;
  }
  csvfileactual.close();

  //-----------------desired trajectory-----------------------------------------------------
  std::ofstream csvfiledesired;
  csvfiledesired.open("/home/lukas/catkin_ws/src/vector_thruster/files/thesis/desired_trajectories_"+s.name+".csv");
  if( !csvfiledesired ) { // file couldn't be opened
      std::cerr << "Error: file could not be opened" << "/n";
      exit(1);
  }
  csvfiledesired << "desired_trajectories in format position.x ; position.y ; position.z ; velocity.x ; velocity.y ; velocity.z ; angular.x ; angular.y ; angular.z";
  for(vector_thruster::pose pose : s.desiredPath)
  {
    csvfiledesired << std::to_string(pose.position.x) + ";" + std::to_string(pose.position.y) + ";" + std::to_string(pose.position.z) + ";" + std::to_string(pose.velocity.x) + ";" + std::to_string(pose.velocity.y) + ";" + std::to_string(pose.velocity.z) + ";" + std::to_string(pose.rotation.x) + ";" + std::to_string(pose.rotation.y) + ";" + std::to_string(pose.rotation.z) + ";" + std::to_string(pose.time) << std::endl;
  }
  csvfiledesired.close();

  ROS_INFO("Saving finished");
  ROS_INFO("Saving finished");
  ROS_INFO("Saving finished");
  ROS_INFO("Saving finished");
  ROS_INFO("Saving finished");
  ROS_INFO("Saving finished");
  ROS_INFO("Saving finished");

  return 0;
}

  //Put plots here:
  /*
  std::vector<float> actual_x;
  std::vector<float> actual_y;
  std::vector<float> actual_t;
  std::vector<float> actual_v;
  float t_counter = 0.f;
  for(vector_thruster::pose pose : s.actualPath)
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
  for(vector_thruster::pose pose : s.desiredPath)
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
  */
