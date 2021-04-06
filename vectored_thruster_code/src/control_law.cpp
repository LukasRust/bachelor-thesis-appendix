#include "../include/control_law.hpp"

void Controller::calcErrors()
{
  proportional_error = orientation.inverse()._transformVector((desired.position-actual.position).toEigenVector());
  integral_error = integral_error + proportional_error;
  proportional_orientation_error = getOrientationError();
  integral_orientation_error = integral_orientation_error + proportional_orientation_error;
  proportional_position_error = getPositionError();
  integral_position_error = integral_position_error + proportional_position_error;
}

ControlVector Controller::getOrientationError()
{
  Eigen::Vector3d desiredInWorld(desired.velocity.x, desired.velocity.y, desired.velocity.z);
  //xIn World muss noch gedreht werden, kann das sein? da fehlt doch was! trajectory in world wird ja auch gedreht zu trajectory in robot
  //Eigen::Vector3d xInWorld(actual.velocity.x, actual.velocity.y, actual.velocity.z);
  //Eigen::Vector3d trajectoryInRobot = R_nI_W._transformVector(trajectoryInWorld);
  //Eigen::Vector3d xInRobot = R_nI_W._transformVector(xInWorld);
  //hier muss ich das auch nochmal ünerprüfen
  Eigen::Vector3d actualInWorld = W_nI_R._transformVector(Eigen::Vector3d(1.f,0.f,0.f));
  ControlVector orientationSpin;
  orientationSpin = desiredInWorld.normalized().cross(actualInWorld);
  orientationSpin.x = 0.f;
  //orientationSpin.y = 0.f;
  return orientationSpin;
}

ControlVector Controller::getPositionError()
{
  Eigen::Vector3d xInWorld = W_nI_R._transformVector(Eigen::Vector3d(1.,0.,0.));
  //Eigen::Vector3d xInRobot = R_nI_W._transformVector(xInWorld);
  //dieses robot in world ist ja total weird, wo hab ich das her?
  //das hier muss gedreht werden oder? also to Position ist hier ja noch im world frame! wenn der roboter zum beispiel bsichen links daneben steht ist der vektor immer total schief

  Eigen::Vector3d toPosition(desired.position.x-actual.position.x, desired.position.y-actual.position.y, desired.position.z-actual.position.z);
  ControlVector positionSpin;
  positionSpin.y = 0.f;
  positionSpin.z = 0.f;
  if(toPosition.norm() > 0.5f)
    positionSpin = toPosition.normalized().cross(xInWorld);
  positionSpin.x = 0.f;
  //positionSpin.y = 0.f;
  return positionSpin;
}

Tau Controller::getCorrectionalTau()
{
  //Typen von proportional error wurden geändert, macht vielleicht keine sinn mehr von pose auf contzrol vector
  Tau correctional;
  calcErrors();
  correctional.translational.x = proportional_error.x * kp_forward_translational + integral_error.x * ki_forward_translational; //TODO berücksichtigt nicht die orientierung des auvs. vektor müsste um orientierung gedreht werden
  correctional.translational.y = proportional_error.y * kp_side_translational + integral_error.y * ki_side_translational;
  correctional.translational.z = proportional_error.z * kp_side_translational + integral_error.z * ki_side_translational;
  correctional.rotational.x = 0.f;//= proportional_position_error.x * kp_positionSpin + integral_position_error.x * ki_positionSpin + proportional_orientation_error.x * kp_orientationSpin + integral_orientation_error.x * ki_orientationSpin;
  correctional.rotational.y = -1. * proportional_position_error.y * kp_positionSpinPitch + integral_position_error.y * ki_positionSpin + -1. * proportional_orientation_error.y * kp_orientationSpinPitch + integral_orientation_error.y * ki_orientationSpin;
  correctional.rotational.z = -1. * proportional_position_error.z * kp_positionSpinYaw + integral_position_error.z * ki_positionSpin + -1. * proportional_orientation_error.z * kp_orientationSpinYaw + integral_orientation_error.z * ki_orientationSpin;
  return correctional;
}

Tau Controller::execute()
{
  return getCorrectionalTau();
}

/*
void Controller::updateActualTau(const vector_thruster::pose::ConstPtr& msgPose)
{
  this->actual = msgPose;
}
*/

void Controller::updateDesiredTau(const vector_thruster::pose::ConstPtr& msgPose)
{
  //TODO trajectory wird in robot frame angegeben, positionsfehler aber im world frame ermittelt!
  this->desired = msgPose;
}

void Controller::updateActualPose(const gazebo_msgs::ModelStates::ConstPtr& stateMsg)
{
  int counter = 0;
  bool found = false;
  while(!found)
  {
    if(stateMsg->name[counter] == "deepleng")
    {
      if(!firstPositionFound)
      {
        startPosition << stateMsg->pose[counter].position.x,stateMsg->pose[counter].position.y,stateMsg->pose[counter].position.z;//actual.position.x, actual.position.y, actual.position.z;
        firstPositionFound = true;
      }
      actual.position.x = stateMsg->pose[counter].position.x - startPosition.x();
      actual.position.y = stateMsg->pose[counter].position.y - startPosition.y();
      actual.position.z = stateMsg->pose[counter].position.z - startPosition.z();
      actual.rotation.x = stateMsg->twist[counter].angular.x;
      actual.rotation.y = stateMsg->twist[counter].angular.y;
      actual.rotation.z = stateMsg->twist[counter].angular.z;
      actual.velocity.x = stateMsg->twist[counter].linear.x;
      actual.velocity.y = stateMsg->twist[counter].linear.y;
      actual.velocity.z = stateMsg->twist[counter].linear.z;
      orientation.x() = stateMsg->pose[counter].orientation.x;
      orientation.y() = stateMsg->pose[counter].orientation.y;
      orientation.z() = stateMsg->pose[counter].orientation.z;
      orientation.w() = stateMsg->pose[counter].orientation.w;
      orientation.normalize();
      //muss das erst invertiert werden?
      W_nI_R = orientation.toRotationMatrix();
      R_nI_W = orientation.inverse().toRotationMatrix();
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
  ros::init(argc, argv, "control_law");
  ros::NodeHandle n;
  Controller c;
  //ros::Subscriber subActual = n.subscribe("pose_estimation", 1000, &Controller::updateActualTau, &c);
  ros::Subscriber subDesired = n.subscribe("desired_pose", 1000, &Controller::updateDesiredTau, &c);
  ros::Subscriber subPose = n.subscribe("/gazebo/model_states", 1000, &Controller::updateActualPose, &c);
  ros::Publisher pub = n.advertise<vector_thruster::tau>("controller_tau",1000);
  ros::Rate loop_rate(10);
  vector_thruster::tau tauMsg;
  c.actual.velocity.x = 1.f;
  while(ros::ok())
  {
    tauMsg = fillTauMsg(tauMsg,c.execute());
    pub.publish(tauMsg);
    ros::spinOnce();
    loop_rate.sleep();
    ROS_INFO("actual position: %f %f %f and desired position: %f %f %f", c.actual.position.x, c.actual.position.y, c.actual.position.z, c.desired.position.x, c.desired.position.y, c.desired.position.z);
    ROS_INFO("actual velocity: %f %f %f and desired velocity: %f %f %f", c.actual.velocity.x, c.actual.velocity.y, c.actual.velocity.z, c.desired.velocity.x, c.desired.velocity.y, c.desired.velocity.z);
    ROS_INFO("startPosition: %f %f %f",c.startPosition.x(),c.startPosition.y(),c.startPosition.z());
  }
}
