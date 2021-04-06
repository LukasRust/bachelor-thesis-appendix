#include "../Dynamic_Model/uwv_dynamic_model/.git/src/DynamicModel.hpp"
#include "ros/ros.h"
#include "vector_thruster/tau.h"
#include "vector_thruster/trajectory.h"
#include "vector_thruster/pose.h"
#include "../include/base.hpp"
#include "vector_thruster/thrustersignal.h"
#include "uuv_gazebo_ros_plugins_msgs/FloatStamped.h"
#include "std_msgs/Float64.h"
#include "gazebo_msgs/ModelStates.h"

Eigen::VectorXd acceleration(6);
Eigen::VectorXd velocity(6);
Eigen::Quaterniond orientation;

void updateActual(const gazebo_msgs::ModelStates::ConstPtr& stateMsg)
{
  int counter = 0;
  bool found = false;
  while(!found)
  {
    if(stateMsg->name[counter] == "deepleng")
    {
      orientation.x() = stateMsg->pose[counter].orientation.x;
      orientation.y() = stateMsg->pose[counter].orientation.y;
      orientation.z() = stateMsg->pose[counter].orientation.z;
      orientation.w() = stateMsg->pose[counter].orientation.w;
      found = true;
    }
    else
    {
      counter++;
    }
  }
}

void updateDesired(const vector_thruster::pose::ConstPtr& msgDesired)
{
  Eigen::Vector3d translational;
  translational[0] = msgDesired->velocity.x;
  translational[1] = msgDesired->velocity.y;
  translational[2] = msgDesired->velocity.z;
  translational = orientation.inverse()._transformVector(translational);
  velocity[0] = translational.x();
  velocity[1] = 0.f;//translational.y(); //TODO consider if this is a good idea or not!
  velocity[2] = 0.f;//translational.z();
  Eigen::Vector3d rotation;//angular speed
  rotation.x() = msgDesired->rotation.x;
  rotation.y() = msgDesired->rotation.y;//TODO = null?
  rotation.z() = msgDesired->rotation.z;
  Eigen::MatrixXd T_inv(3,3);
  Eigen::Vector3d euler = orientation.toRotationMatrix().eulerAngles(2, 1, 0);
  double roll = euler(2), pitch = euler(1);
  /* from eq.(2.28) p.25 [Fossen, 2011]*/
  T_inv << 1., 0., -sin(pitch),
           0., cos(roll), cos(pitch)*sin(roll),
           0., -sin(roll), cos(pitch)*cos(roll);

           //the abovwe was used before but threw errors or unplausible values for the pitch angle when the x axis of the robot aligned to the y axis of the world frame
  /*
  Eigen::Vector3d out;
  out << std::atan2(2.*(orientation.w()*orientation.x() + orientation.y()*orientation.z()), 1. - 2.*(orientation.x()*orientation.x() + orientation.y()*orientation.y())),
         std::asin(2.*(orientation.w()*orientation.y() - orientation.z()*orientation.x())),
         std::atan2(2.*(orientation.w()*orientation.z() + orientation.x()*orientation.y()), 1. - 2.*(orientation.y()*orientation.y() + orientation.z()*orientation.z()));
         */
  rotation = T_inv*rotation;
  //rotation = out;
  velocity[3] = rotation.x();
  velocity[4] = rotation.y();
  velocity[5] = rotation.z();
  acceleration << 0.,0.,0.,0.,0.,0.;
  acceleration[0] = msgDesired->acceleration.x;
  acceleration[1] = msgDesired->acceleration.y;
  //acceleration[2] = msgDesired->acceleration.z; //TODO consider if this is helpful or not
  //ROS_INFO("velocity in dm. x: %f y: %f z: %f",velocity[0], velocity[1], velocity[2]);
}

int main(int argc, char **argv)
{
 ros::init(argc, argv, "dynamic_model");
 ros::NodeHandle n;
 ros::Subscriber subActual = n.subscribe("/gazebo/model_states", 1000, &updateActual);
 ros::Subscriber subDesired = n.subscribe("desired_pose", 1000, &updateDesired);
 ros::Publisher pub = n.advertise<vector_thruster::tau>("model_tau",1000);  // initialize the model parameters
 ros::Rate loop_rate = 10;
 // instance of DynamicModel
 uwv_dynamic_model::DynamicModel dynamic_model;

 velocity[0] = 0.0;
 while(ros::ok())
 {
 uwv_dynamic_model::ModelParameters model_parameters;
 model_parameters.model_type = uwv_dynamic_model::QUADRATIC;
 model_parameters.inertia_matrix << 100., 0., 0., 0., 0., 0.,
             0., 100., 0., 0., 0., 0.,
             0., 0., 100., 0., 0., 0.,
             0., 0., 0., 100., 0., 0.,
             0., 0., 0., 0., 100., 0.,
             0., 0., 0., 0., 0., 100.;
model_parameters.center_of_gravity = Eigen::Vector3d(0., 0., 0.0);
model_parameters.center_of_buoyancy = Eigen::Vector3d(0.,0.,0.005);
model_parameters.linear_damping_matrix << 30.,0., 0., 0., 0., 0.,
                                          0., 80.,0., 0., 0., 0.,
                                          0., 0., 80.,0., 0., 0.,
                                          0., 0., 0., 5.,0., 0.,
                                          0., 0., 0., 0., 200.,0.,
                                          0., 0., 0., 0., 0., 200.;
model_parameters.weight = 185.0;
model_parameters.buoyancy = 185.0;
model_parameters.quad_damping_matrices.resize(1);
model_parameters.quad_damping_matrices[0]  << 5.,0., 0., 0., 0., 0.,
                                          0., 300.,0., 0., 0., 0.,
                                          0., 0., 300.,0., 0., 0.,
                                          0., 0., 0., 15.,0., 0.,
                                          0., 0., 0., 0., 500.,0.,
                                          0., 0., 0., 0., 0., 500.;

//-5 -300 -300 -15 -500 -500

 // set the model_parameters in dynamic_model
 dynamic_model.setModelParameters(model_parameters);

 Eigen::VectorXd efforts = dynamic_model.calcEfforts(acceleration, velocity, orientation);
 //Tau tau = acc; fossen 277
 vector_thruster::tau msgTau;
 msgTau.translational.x = efforts[0];
 msgTau.translational.y = 0.f;//efforts[1];
 msgTau.translational.z = 0.f;//efforts[2];//TODO consider if this makes sense or not
 msgTau.rotational.x = 0.f;//efforts[3];
 msgTau.rotational.y = efforts[4];
 msgTau.rotational.z = efforts[5];
 pub.publish(msgTau);
 ros::spinOnce();
 loop_rate.sleep();
}
}
