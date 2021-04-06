#include "../include/control_allocation.hpp"

void ControlAllocator::calcPitchDerivative()
{
  pitchDerivative.translational.x = std::cos(currentYaw) * -std::sin(currentPitch);
  pitchDerivative.translational.y = std::sin(currentYaw) * -std::sin(currentPitch);
  pitchDerivative.translational.z = -std::cos(currentPitch);
  pitchDerivative.rotational.x = y*-std::cos(currentPitch)-z*std::sin(currentYaw)*-std::sin(currentPitch);
  pitchDerivative.rotational.y = z*std::cos(currentYaw)*-std::sin(currentPitch)-x*-std::cos(currentPitch);
  pitchDerivative.rotational.z = x*std::sin(currentYaw)*-std::sin(currentPitch)-y*std::cos(currentYaw)*-std::sin(currentPitch);
  pitchDerivative * currentForce;
}

void ControlAllocator::calcYawDerivative()
{
  yawDerivative.translational.x = -std::sin(currentYaw)*std::cos(currentPitch);
  yawDerivative.translational.y = std::cos(currentYaw)*std::cos(currentPitch);
  yawDerivative.translational.z = -std::sin(currentPitch);
  yawDerivative.rotational.x = y*-std::sin(currentPitch)-z*std::cos(currentYaw)*std::cos(currentPitch);
  yawDerivative.rotational.y = z*-std::sin(currentYaw)*std::cos(currentPitch)-x*-std::sin(currentPitch);
  yawDerivative.rotational.z = x*std::cos(currentYaw)*std::cos(currentPitch)-y*-std::sin(currentYaw)*cos(currentPitch);
  yawDerivative * currentForce;
}

void ControlAllocator::calcForceDerivative()
{
  forceDerivative.translational.x = std::cos(currentYaw)*std::cos(currentPitch);
  forceDerivative.translational.y = std::sin(currentYaw)*std::cos(currentPitch);
  forceDerivative.translational.z = -std::sin(currentPitch);
  forceDerivative.rotational.x = y*-std::sin(currentPitch)-z*std::sin(currentYaw)*std::cos(currentPitch);
  forceDerivative.rotational.y = z*std::cos(currentYaw)*std::cos(currentPitch)-x*-std::sin(currentPitch);
  forceDerivative.rotational.z = x*std::sin(currentYaw)*std::cos(currentPitch)-y*std::cos(currentYaw)*cos(currentPitch);
}

void ControlAllocator::calcCurrent()
{
  current.translational.x = std::cos(currentYaw)*std::cos(currentPitch);
  current.translational.y = std::sin(currentYaw)*std::cos(currentPitch);
  current.translational.z = -std::sin(currentPitch);
  current.rotational.x = y*-std::sin(currentPitch)-z*std::sin(currentYaw)*std::cos(currentPitch);
  current.rotational.y = z*std::cos(currentYaw)*std::cos(currentPitch)-x*-std::sin(currentPitch);
  current.rotational.z = x*std::sin(currentYaw)*std::cos(currentPitch)-y*std::cos(currentYaw)*cos(currentPitch);
  current * currentForce;
}

vector_thruster::thrustersignal ControlAllocator::calcAllocation()
{
  //if(std::abs(input.rotational.y > 1000.f))
    //input.rotational.y = 0.f;
  this->calcPitchDerivative();
  this->calcYawDerivative();
  this->calcForceDerivative();
  alglib::real_2d_array a;
  a.setlength(9,9);
  for(int i = 0; i < 9; i++)
  {
    for(int n = 0; n < 9; n++)
    {
      a(i,n) = 0.;
    }
  }
  a(0,0)=20.0;
  a(1,1)=0.2; //Müsste eigentlich ganz hoch sein um minimiert zu werden!
  a(2,2)=0.2;//Müsste eigentlich ganz hoch sein um minimiert zu werden!
  a(3,3)=0.2;//Müsste eigentlich ganz hoch sein um minimiert zu werden!
  a(4,4)=2.;
  a(5,5)=2.;
  a(6,6)=2.;
  a(7,7)=2.;
  a(8,8)=2.;
  //füllbar mit setcontent()dann array anfang und größe oder so
  alglib::real_1d_array x0 = "[0,0,0,0,0,0,-0.01,-0.01,0]";
  alglib::real_1d_array s = "[1,1,1,1,1,1,0.001,0.001,0.1]";
  alglib::real_1d_array bndl = "[-1000.0,-10000.0,-10000.0,-10000.0,-100.0,-100.0,-500,-500,-1000.0]";//"-pitchRadius,-yawRadius,-forceRadius]";
  alglib::real_1d_array bndu = "[1000.0,10000.0,10000.0,10000.0,100.0,100.0,500,500,1000.0]";//"pitchRadius,yawRadius,forceRadius]";
  alglib::real_2d_array c;
  c.setlength(6,10);
  c(0,0) = 1.0; c(0,1) = 0.0; c(0,2) = 0.0; c(0,3) = 0.0; c(0,4) = 0.0; c(0,5) = 0.0; c(0,6) = pitchDerivative.translational.x; c(0,7) = yawDerivative.translational.x; c(0,8) = forceDerivative.translational.x; c(0,9) = input.translational.x - current.translational.x;
  c(1,0) = 0.0; c(1,1) = 1.0; c(1,2) = 0.0; c(1,3) = 0.0; c(1,4) = 0.0; c(1,5) = 0.0; c(1,6) = pitchDerivative.translational.y; c(1,7) = yawDerivative.translational.y; c(1,8) = forceDerivative.translational.y; c(1,9) = input.translational.y - current.translational.y;
  c(2,0) = 0.0; c(2,1) = 0.0; c(2,2) = 1.0; c(2,3) = 0.0; c(2,4) = 0.0; c(2,5) = 0.0; c(2,6) = pitchDerivative.translational.z; c(2,7) = yawDerivative.translational.z; c(2,8) = forceDerivative.translational.z; c(2,9) = input.translational.z - current.translational.z;
  c(3,0) = 0.0; c(3,1) = 0.0; c(3,2) = 0.0; c(3,3) = 1.0; c(3,4) = 0.0; c(3,5) = 0.0; c(3,6) = pitchDerivative.rotational.x; c(3,7) = yawDerivative.rotational.x; c(3,8) = forceDerivative.rotational.x; c(3,9) = input.rotational.x - current.rotational.x;
  c(4,0) = 0.0; c(4,1) = 0.0; c(4,2) = 0.0; c(4,3) = 0.0; c(4,4) = 1.0; c(4,5) = 0.0; c(4,6) = pitchDerivative.rotational.y; c(4,7) = yawDerivative.rotational.y; c(4,8) = forceDerivative.rotational.y; c(4,9) = input.rotational.y - current.rotational.y;
  c(5,0) = 0.0; c(5,1) = 0.0; c(5,2) = 0.0; c(5,3) = 0.0; c(5,4) = 0.0; c(5,5) = 1.0; c(5,6) = pitchDerivative.rotational.z; c(5,7) = yawDerivative.rotational.z; c(5,8) = forceDerivative.rotational.z; c(5,9) = input.rotational.z - current.rotational.z;
  alglib::integer_1d_array ct = "[0,0,0,0,0,0]";
  alglib::real_1d_array x;
  alglib::minqpstate state;
  alglib::minqpreport rep;
  alglib::minqpcreate(9, state);
  alglib::minqpsetquadraticterm(state, a);
  //alglib::minqpsetlinearterm(state, b);
  alglib::minqpsetstartingpoint(state, x0);
  alglib::minqpsetbc(state, bndl, bndu);
  /*try
  {*/
    alglib::minqpsetlc(state, c, ct);
  /*}
  catch(...)
  {*/
  for(int i = 0;i < 6;i++)
  {
    for(int n = 6;n < 10;n++)
    {
      ROS_INFO("c %i %i: %f",i,n, c(i,n));
    }
  }
  //}
  //minqpsetalgoquickqp kann anscheinend nicht convexe proble nicht lösen?
  //minqpsetalgobleic
  //alglib::minqpsetscale(state, s);
  alglib::minqpsetalgobleic(state, 0.0, 0.0, 0.0, 0);//noch ein true als letzten paramter für quiksolver
  alglib::minqpoptimize(state);
  alglib::minqpresults(state, x, rep);
  //printf("tType: %d\n inneriterations: %d outeriterations: %d", int(rep.terminationtype), rep.inneriterationscount, rep.outeriterationscount); // EXPECTED: 4
  //printf("%s\n", x.tostring(5).c_str()); // EXPECTED: [2.5,2]
  vector_thruster::thrustersignal signal;
  signal.pitch = currentPitch + x[6];
  signal.yaw = currentYaw + x[7];
  signal.force = currentForce + x[8];
  currentPitch += x[6];
  currentYaw += x[7];
  currentForce += x[8];



  this->times.push_back(ros::Time::now().toSec());
  this->xs.push_back(x);
  this->modelInputs.push_back(this->modelInput);
  this->regulationInputs.push_back(this->regulationInput);
  this->inputs.push_back(this->input);
  this->currents.push_back(this->current);
  std::vector<float> acc;
  acc.push_back(currentPitch);acc.push_back(currentYaw);acc.push_back(currentForce);
  signals.push_back(acc);

  calcCurrent();

  return signal;
  //krass verzögert! und bleibt anscheinend immer negativ, wechselt die Seite nicht TODO
}

void ControlAllocator::updateModelTau(const vector_thruster::tau::ConstPtr& msgTau)
{
  modelInput = msgTau;
  input = modelInput + regulationInput;
}

void ControlAllocator::updateControlTau(const vector_thruster::tau::ConstPtr& msgTau)
{
  regulationInput = msgTau;
  input = modelInput + regulationInput;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_allocation");
  ros::NodeHandle n;
  ControlAllocator ca;
  ros::Subscriber subModelTau = n.subscribe("model_tau", 1000, &ControlAllocator::updateModelTau, &ca);
  ros::Subscriber subControllerTau = n.subscribe("controller_tau", 1000, &ControlAllocator::updateControlTau, &ca);
  ros::Publisher pub = n.advertise<vector_thruster::thrustersignal>("actuator_signal",1000);
  ros::Rate loop_rate(10);
  /*try
  {*/
    ca.input.translational.x = 0.0;
    ca.input.translational.y = 0.0;
    ca.input.translational.z = 0.0;
    ca.input.rotational.x = 0.0;
    ca.input.rotational.y = 0.0;
    ca.input.rotational.z = 0.0;
    ca.current.translational.x = 0.0;
    ca.current.translational.y = 0.0;
    ca.current.translational.z = 0.0;
    ca.current.rotational.x = 0.0;
    ca.current.rotational.y = 0.0;
    ca.current.rotational.z = 0.0;
    ca.currentForce = 0.0;
    ca.currentYaw = -0.01;
    ca.currentPitch = -0.01;
    //TODO funktioniert nicht da dsss pathplanning zwischendurch negative werte für die x geschwindigkeiten rauskommen

  while(ros::ok())
  {
    vector_thruster::thrustersignal tsMsg;
    try
    {
      tsMsg = ca.calcAllocation();
    }
    catch(alglib::ap_error e)
    {
      printf("error msg: %s\n", e.msg.c_str());
    }
    pub.publish(tsMsg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  ca.saveLog(ca.xs);
}

void ControlAllocator::saveLog(std::vector<alglib::real_1d_array> xs)
{
  ROS_INFO("Start saving control allocation log");
  //----------------actual trajectories------------------------------------------------------
  std::ofstream csvfileactual;
  csvfileactual.open("/home/lukas/catkin_ws/src/vector_thruster/files/thesis/calog_.csv");
  if( !csvfileactual ) { // file couldn't be opened
      std::cerr << "Error: file could not be opened" << "/n";
      exit(1);
  }
  csvfileactual << "control allocation log";

  int counter = 0;
  for(alglib::real_1d_array x : this->xs)
  {
    csvfileactual << std::to_string(x[0]) + ";" + std::to_string(x[1]) + ";" + std::to_string(x[2]) + ";" + std::to_string(x[3]) + ";" + std::to_string(x[4]) + ";" + std::to_string(x[5]) + ";" + std::to_string(x[6]) + ";" + std::to_string(x[7]) + ";" + std::to_string(x[8]) + ";" + std::to_string(times.at(counter)) << std::endl;
    counter++;
  }
  csvfileactual << std::endl;
  csvfileactual << std::endl;
  csvfileactual << std::endl;
  csvfileactual << std::endl;
  counter = 0;
  for(Tau t : modelInputs)
  {
    csvfileactual << std::to_string(t.translational.x) + ";" +  std::to_string(t.translational.y) + ";" +std::to_string(t.translational.z) + ";" +std::to_string(t.rotational.x) + ";" +std::to_string(t.rotational.y) + ";" +std::to_string(t.rotational.z)  + ";" + std::to_string(times.at(counter)) << std::endl;
    counter++;
  }
  csvfileactual << std::endl;
  csvfileactual << std::endl;
  csvfileactual << std::endl;
  csvfileactual << std::endl;
  counter = 0;
  for(Tau t : regulationInputs)
  {
    csvfileactual << std::to_string(t.translational.x) + ";" +  std::to_string(t.translational.y) + ";" +std::to_string(t.translational.z) + ";" +std::to_string(t.rotational.x) + ";" +std::to_string(t.rotational.y) + ";" +std::to_string(t.rotational.z)  + ";" + std::to_string(times.at(counter)) << std::endl;
    counter++;
  }
  csvfileactual << std::endl;
  csvfileactual << std::endl;
  csvfileactual << std::endl;
  csvfileactual << std::endl;
  counter = 0;
  for(Tau t : inputs)
  {
    csvfileactual << std::to_string(t.translational.x) + ";" +  std::to_string(t.translational.y) + ";" +std::to_string(t.translational.z) + ";" +std::to_string(t.rotational.x) + ";" +std::to_string(t.rotational.y) + ";" +std::to_string(t.rotational.z)  + ";" + std::to_string(times.at(counter)) << std::endl;
    counter++;
  }
  csvfileactual << std::endl;
  csvfileactual << std::endl;
  csvfileactual << std::endl;
  csvfileactual << std::endl;
  counter = 0;
  for(Tau t : currents)
  {
    csvfileactual << std::to_string(t.translational.x) + ";" +  std::to_string(t.translational.y) + ";" +std::to_string(t.translational.z) + ";" +std::to_string(t.rotational.x) + ";" +std::to_string(t.rotational.y) + ";" +std::to_string(t.rotational.z)  + ";" + std::to_string(times.at(counter)) << std::endl;
    counter++;
  }
  csvfileactual << std::endl;
  csvfileactual << std::endl;
  csvfileactual << std::endl;
  csvfileactual << std::endl;
  counter = 0;
  for(std::vector<float> v : signals)
  {
    csvfileactual << std::to_string(v.at(0)) + ";" +  std::to_string(v.at(1)) + ";" +std::to_string(v.at(2)) + ";" + std::to_string(times.at(counter)) << std::endl;
    counter++;
  }
  //csvfileactual
  csvfileactual.close();
}
