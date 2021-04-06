#include "ros/ros.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "object_avoidance_catfish_example/collisionmsg.h"
#include "vector_thruster/tau.h"
#include "vector_thruster/thrustersignal.h"
#include "./base.hpp"
#include "../src/qpSolver/cpp/src/optimization.h"
#include <fstream>
#include <iostream>
#include <string>


class ControlAllocator
{
public:
  Tau modelInput;
  Tau regulationInput;
  Tau input;
  Tau current;
  float currentForce = 0.f;
  float currentYaw = 0.f;
  float currentPitch = 0.f;
  float x = -1.325;
  float y = 0.;
  float z = 0.;
  Tau errorRadius; //macht eigentlich keinen sinn
  float yawRadius;
  float pitchRadius;
  float forceRadius;
  Tau yawDerivative;
  Tau pitchDerivative;
  Tau forceDerivative;
  std::vector<alglib::real_1d_array> xs;
  std::vector<float> times;
  std::vector<Tau> modelInputs;
  std::vector<Tau> regulationInputs;
  std::vector<Tau> inputs;
  std::vector<Tau> currents;
  std::vector<std::vector<float>> signals;
  void calcPitchDerivative();
  void calcYawDerivative();
  void calcForceDerivative();
  void calcCurrent();
  vector_thruster::thrustersignal calcAllocation();
  void updateModelTau(const vector_thruster::tau::ConstPtr& msgTau);
  void updateControlTau(const vector_thruster::tau::ConstPtr& msgTau);
  void saveLog(std::vector<alglib::real_1d_array> xs);
};
