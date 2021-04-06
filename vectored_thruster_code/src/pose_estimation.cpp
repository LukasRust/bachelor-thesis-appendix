#include "../include/pose_estimation.hpp"

vector_thruster::pose poseMsg;

void update(const gazebo_msgs::ModelStates::ConstPtr& stateMsg)
{
  int counter = 0;
  bool found = false;
  while(!found)
  {
    if(stateMsg->name[counter] == "deepleng")
    {
      poseMsg.position.x = stateMsg->pose[counter].position.x;
      poseMsg.position.y = stateMsg->pose[counter].position.y;
      poseMsg.position.z = stateMsg->pose[counter].position.z;
      poseMsg.rotation.x = stateMsg->twist[counter].angular.x;
      poseMsg.rotation.y = stateMsg->twist[counter].angular.y;
      poseMsg.rotation.z = stateMsg->twist[counter].angular.z;
      poseMsg.velocity.x = stateMsg->twist[counter].linear.x;
      poseMsg.velocity.y = stateMsg->twist[counter].linear.y;
      poseMsg.velocity.z = stateMsg->twist[counter].linear.z;
      found = true;
    }
    else
    {
      counter++;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_estimation");
  ros::NodeHandle n;
  ros::Subscriber subDesired = n.subscribe("/gazebo/model_states", 1000, &update); //gazebo_msgs/ModelStates
  ros::Publisher pub = n.advertise<vector_thruster::pose>("pose_estimation",1000);
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    pub.publish(poseMsg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
