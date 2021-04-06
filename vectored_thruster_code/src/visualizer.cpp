#include "../include/visualizer.hpp"
#include "../include/matplotlib-cpp/matplotlibcpp.h"
namespace plt = matplotlibcpp;

void Visualizer::responseDesired(const vector_thruster::pose::ConstPtr& msgPose)
{
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = msgPose->position.x;
  pose.pose.position.y = msgPose->position.y;
  pose.pose.position.z = msgPose->position.z;
  Eigen::Vector3d origin(1.f,0.f,0.f);
  Eigen::Vector3d to(msgPose->velocity.x, msgPose->velocity.y, msgPose->velocity.z);
  Eigen::Quaterniond q;
  q = q.FromTwoVectors(origin,to); //TODO check if right
  pose.pose.orientation.w = q.w();
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  this->desiredPath.poses.push_back(pose);
}

void Visualizer::responseActual(const gazebo_msgs::ModelStates::ConstPtr& stateMsg)
{
  int counter = 0;
  bool found = false;
  geometry_msgs::PoseStamped pose;
  while(!found)
  {
    if(stateMsg->name[counter] == "deepleng")
    {
      if(!firstPositionFound)
      {
        startPosition << stateMsg->pose[counter].position.x, stateMsg->pose[counter].position.y, stateMsg->pose[counter].position.z;
        this->firstPositionFound = true;
      }
      pose.pose.position.x = stateMsg->pose[counter].position.x - startPosition.x();
      pose.pose.position.y = stateMsg->pose[counter].position.y - startPosition.y();
      pose.pose.position.z = stateMsg->pose[counter].position.z - startPosition.z();
      pose.pose.orientation.x = stateMsg->pose[counter].orientation.x;
      pose.pose.orientation.y = stateMsg->pose[counter].orientation.y;
      pose.pose.orientation.z = stateMsg->pose[counter].orientation.z;
      pose.pose.orientation.w = stateMsg->pose[counter].orientation.w;
      found = true;
    }
    else
    {
      counter++;
    }
  }
  pose.header.frame_id = "deepleng/base_link";
  this->actualPath.poses.push_back(pose);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visualizer");
  ros::NodeHandle n;
  Visualizer v;
  float m = 0.65f;
  float b = 200.f;
  std::vector<float> t(1000);
  std::vector<float> t_short(300);
  std::vector<float> ramp(1000);
  std::vector<float> constant(300);
  std::vector<float> circle(1000);
  std::vector<float> ground(1050);
  float radiusf = 300.f;
  int radius = 300;
  float t_c;
  for(int t_i = 0; t_i < t.size(); t_i++)
  {
    t[t_i] = (float)(t_i);
    if(t_i < t_short.size())
    {
      t_short[t_i] = (float)(t_i);
      constant[t_i] = b;
    }
    ramp[t_i] = b + t[t_i] * m;
    ground[t_i] = 0.f;

    float threshold = 30.f;
    if((t_i - radius) * -1.f <= threshold && (t_i - radius) * -1.f >= 0.f)
    {
      t_c = (t_i - radius) * -1.f;
      circle[t_i] = 0.f;
      int index = radiusf * std::cos((t_c/radius) * 2.f * 3.14f);
      circle[index] = radiusf * std::sin((t_c/radius) * 2.f * 3.14f) + b;
    }
    else
      circle[t_i] = 0.f;

  }
  plt::plot(t,ramp,"k-",t_short,constant, "r--",t,ground, "k-");//, t,circle,"r-");
  plt::xlabel("t");
  plt::ylabel("Y");
  plt::show();
}



















  /*ros::Subscriber subDesired = n.subscribe("desired_pose", 1000, &Visualizer::responseDesired, &v);
  ros::Subscriber subActual = n.subscribe("gazebo/model_states", 1000, &Visualizer::responseActual, &v);
  ros::Publisher pubDesired = n.advertise<nav_msgs::Path>("pathDesired",1000);
  ros::Publisher pubActual = n.advertise<nav_msgs::Path>("pathActual",1000);
  ros::Rate loop_rate(v.loopRate);*/
  /*while(ros::ok())
  {
    pubActual.publish(v.actualPath);
    pubDesired.publish(v.desiredPath);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 1;*/
