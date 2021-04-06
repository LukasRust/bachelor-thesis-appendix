#include "../include/actuator.hpp"

float pitch,yaw,force,rotorConstant;

void update(const vector_thruster::thrustersignal::ConstPtr& u)
{
  pitch = u->pitch;
  yaw = u->yaw;
  force = u->force;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "actuator");
  ros::NodeHandle n;
  ros::Subscriber subDesired = n.subscribe("actuator_signal", 1000, &update);
  ros::Publisher yawPub = n.advertise<std_msgs::Float64>("/deepleng/yaw_controller/command",1000);
  ros::Publisher pitchPub = n.advertise<std_msgs::Float64>("/deepleng/pitch_controller/command",1000);
  ros::Publisher forcePub = n.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/deepleng/thrusters/0/input",1000);
  ros::Rate loop_rate(10);
  rotorConstant = 0.01515f;
  while(ros::ok())
  {
    std_msgs::Float64 pitchMsg,yawMsg;
    pitchMsg.data = pitch;
    yawMsg.data = yaw;
    uuv_gazebo_ros_plugins_msgs::FloatStamped forceMsg;
    forceMsg.data = std::sqrt(std::abs(force/rotorConstant))* sign(force);
    //ROS_INFO("force: %f input: %f", force, forceMsg.data);
    pitchPub.publish(pitchMsg);
    yawPub.publish(yawMsg);
    forcePub.publish(forceMsg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
