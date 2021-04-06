#include "../include/trajectory_generator.hpp"

vector_thruster::pose TrajectoryGenerator::createMsg()
{
  vector_thruster::pose pose;
  pose.position.x = desired.position.x;
  pose.position.y = desired.position.y;
  pose.position.z = desired.position.z;
  pose.velocity.x = desired.velocity.x * w_dot;
  pose.velocity.y = desired.velocity.y * w_dot;
  pose.velocity.z = desired.velocity.z * w_dot;
  pose.rotation.x = desired.rotation.x;
  pose.rotation.y = desired.rotation.y;
  pose.rotation.z = desired.rotation.z;
  pose.acceleration.x = desired.acceleration.x * w_dot;
  pose.acceleration.y = desired.acceleration.y * w_dot;
  pose.acceleration.z = desired.acceleration.z * w_dot;
  return pose;
}


void TrajectoryGenerator::updateTrajectory(const vector_thruster::trajectory::ConstPtr& msgTrajectory)
{
  startTime = ros::Time::now();
  Waypoint start;
  start.position = msgTrajectory->start.position;
  start.acceleration = msgTrajectory->start.acceleration;
  start.velocity = msgTrajectory->start.velocity;
  all.push_back(start);
  for(int i = 0; i < msgTrajectory->size/9; i++)
  {
    Waypoint w
    (
      ControlVector(msgTrajectory->via[i+0], msgTrajectory->via[i+1], msgTrajectory->via[i+2]),
      ControlVector(msgTrajectory->via[i+3], msgTrajectory->via[i+4], msgTrajectory->via[i+5]),
      ControlVector(msgTrajectory->via[i+6], msgTrajectory->via[i+7], msgTrajectory->via[i+8])
    );
    all.push_back(w);
  }
  Waypoint end;
  end.position = msgTrajectory->end.position;
  end.acceleration = msgTrajectory->end.acceleration;
  end.velocity = msgTrajectory->end.velocity;
  all.push_back(end);
}

void TrajectoryGenerator::checkIfReached()
{
  if(w > maxTime * 1.0 && w < maxTime * 1.2)// && (actual.position - desired.position).length() < minRangeToWaypoint)
  {
    all.erase(all.begin());
    startTime = ros::Time::now();
  }
}

void TrajectoryGenerator::calcBendTime(bool present)
{
  if(present)
  {
    if(firstFrame)
    {
      w_dot = (1.f-(std::exp(time*-0.5f)));
      firstFrame = false;
    }
    else{
    w_dot = (speed * (1.f-(std::exp(time*-0.5f))))
            /
            desired.velocity.length();
    }
    ROS_INFO("w_dot %f", w_dot);
    w = w + w_dot * (time - lastTimePoint);
    lastTimePoint = time;
  }
  else
  {
    future_w = w + w_dot * (time - lastTimePoint);
  }
}

void TrajectoryGenerator::calcPolynom()
{
  Waypoint first;
  Waypoint second;
  if(counter < all.size()-1)
  {
    first = all.at(counter);
    second = all.at(counter+1);
  }
  else
    return;

  maxTime = (first.position - second.position).length()/speed; //TODO wenn trajektorie mit entschleunigung beschleunigung die velocity mitteln sodass schätzung besser wird ansonsten steigt zwischenduch die geschwindigkeit deutloich über vorgegebenes maximum
  time = (float)(ros::Time::now() - startTime).toSec();

  calcBendTime(true);

  desired.position.x =
  first.position.x +
  first.velocity.x * w +
  ((3/(maxTime*maxTime))*(second.position.x-first.position.x)-(1.f/maxTime)*(2.f*first.velocity.x + second.velocity.x)) * (w*w) +
  (-1.f*(2.f/(maxTime*maxTime*maxTime))*(second.position.x-first.position.x)+(1.f/(maxTime*maxTime))*(first.velocity.x + second.velocity.x)) *(w*w*w);

  desired.velocity.x =
  first.velocity.x +
  ((3/(maxTime*maxTime))*(second.position.x-first.position.x)-(1.f/maxTime)*(2.f*first.velocity.x + second.velocity.x)) * w * 2.f +
  (-1.f*(2.f/(maxTime*maxTime*maxTime))*(second.position.x-first.position.x)+(1.f/(maxTime*maxTime))*(first.velocity.x + second.velocity.x)) * (w*w) * 3.f;

  desired.acceleration.x =
  ((3/(maxTime*maxTime))*(second.position.x-first.position.x)-(1.f/maxTime)*(2.f*first.velocity.x + second.velocity.x)) * 2.f +
  (-1.f*(2.f/(maxTime*maxTime*maxTime))*(second.position.x-first.position.x)+(1.f/(maxTime*maxTime))*(first.velocity.x + second.velocity.x)) * w * 6.f;



  desired.position.y =
  first.position.y +
  first.velocity.y * w +
  ((3/(maxTime*maxTime))*(second.position.y-first.position.y)-(1.f/maxTime)*(2.f*first.velocity.y + second.velocity.y)) * (w*w) +
  (-1.f*(2.f/(maxTime*maxTime*maxTime))*(second.position.y-first.position.y)+(1.f/(maxTime*maxTime))*(first.velocity.y + second.velocity.y)) *(w*w*w);

  desired.velocity.y =
  first.velocity.y +
  ((3/(maxTime*maxTime))*(second.position.y-first.position.y)-(1.f/maxTime)*(2.f*first.velocity.y + second.velocity.y)) * w * 2.f +
  (-1.f*(2.f/(maxTime*maxTime*maxTime))*(second.position.y-first.position.y)+(1.f/(maxTime*maxTime))*(first.velocity.y + second.velocity.y)) * (w*w) * 3.f;

  desired.acceleration.y =
  ((3/(maxTime*maxTime))*(second.position.y-first.position.y)-(1.f/maxTime)*(2.f*first.velocity.y + second.velocity.y)) * 2.f +
  (-1.f*(2.f/(maxTime*maxTime*maxTime))*(second.position.y-first.position.y)+(1.f/(maxTime*maxTime))*(first.velocity.y + second.velocity.y)) * w * 6.f;



  desired.position.z =
  first.position.z +
  first.velocity.z * w +
  ((3/(maxTime*maxTime))*(second.position.z-first.position.z)-(1.f/maxTime)*(2.f*first.velocity.z + second.velocity.z)) * (w*w) +
  (-1.f*(2.f/(maxTime*maxTime*maxTime))*(second.position.z-first.position.z)+(1.f/(maxTime*maxTime))*(first.velocity.z + second.velocity.z)) *(w*w*w);

  desired.velocity.z =
  first.velocity.z +
  ((3/(maxTime*maxTime))*(second.position.z-first.position.z)-(1.f/maxTime)*(2.f*first.velocity.z + second.velocity.z)) * w * 2.f +
  (-1.f*(2.f/(maxTime*maxTime*maxTime))*(second.position.z-first.position.z)+(1.f/(maxTime*maxTime))*(first.velocity.z + second.velocity.z)) * (w*w) * 3.f;

  desired.acceleration.z =
  ((3/(maxTime*maxTime))*(second.position.z-first.position.z)-(1.f/maxTime)*(2.f*first.velocity.z + second.velocity.z)) * 2.f +
  (-1.f*(2.f/(maxTime*maxTime*maxTime))*(second.position.z-first.position.z)+(1.f/(maxTime*maxTime))*(first.velocity.z + second.velocity.z)) * w * 6.f;

//--------------------------------------------------------------------------------------------------------------------------------------------------------//

  time += 1.f/((float)loopRate);

  calcBendTime(false);

  futureVelocity.x =
  first.velocity.x +
  ((3/(maxTime*maxTime))*(second.position.x-first.position.x)-(1.f/maxTime)*(2.f*first.velocity.x + second.velocity.x)) * future_w * 2.f +
  (-1.f*(2.f/(maxTime*maxTime*maxTime))*(second.position.x-first.position.x)+(1.f/(maxTime*maxTime))*(first.velocity.x + second.velocity.x)) * (future_w*future_w) * 3.f;

  futureVelocity.y =
  first.velocity.y +
  ((3/(maxTime*maxTime))*(second.position.y-first.position.y)-(1.f/maxTime)*(2.f*first.velocity.y + second.velocity.y)) * future_w * 2.f +
  (-1.f*(2.f/(maxTime*maxTime*maxTime))*(second.position.y-first.position.y)+(1.f/(maxTime*maxTime))*(first.velocity.y + second.velocity.y)) * (future_w*future_w) * 3.f;

  futureVelocity.z =
  first.velocity.z +
  ((3/(maxTime*maxTime))*(second.position.z-first.position.z)-(1.f/maxTime)*(2.f*first.velocity.z + second.velocity.z)) * future_w * 2.f +
  (-1.f*(2.f/(maxTime*maxTime*maxTime))*(second.position.z-first.position.z)+(1.f/(maxTime*maxTime))*(first.velocity.z + second.velocity.z)) * (future_w*future_w) * 3.f;
}

void TrajectoryGenerator::calcRotation()
{
  Eigen::Vector3d now;
  now.x() = desired.velocity.x;
  now.y() = desired.velocity.y;
  now.z() = desired.velocity.z;
  now.normalize();
  Eigen::Vector3d future;
  future.x() = futureVelocity.x;
  future.y() = futureVelocity.y;
  future.z() = futureVelocity.z;
  future.normalize();


  desired.rotation.y = std::atan2(futureVelocity.z,futureVelocity.x) - std::atan2(desired.velocity.z,desired.velocity.x);

//https://git.hb.dfki.de/ric-maritime/control/control-auv_model_based_control/-/blob/refactor/src/Common.cpp


  desired.rotation.z = std::atan2(futureVelocity.y,futureVelocity.x) - std::atan2(desired.velocity.y,desired.velocity.x);
  desired.rotation.y = desired.rotation.y * ((float)loopRate);
  desired.rotation.z = desired.rotation.z * ((float)loopRate);
  /*ROS_INFO("the velocity now and future normalized: x:%f y:%f z: %f and x:%f y:%f z: %f",
  desired.velocity.x,
  desired.velocity.y,
  desired.velocity.z,
  futureVelocity.x,
  futureVelocity.y,
  futureVelocity.z);*/
  //ROS_INFO("ratio x'/y': %f", desired.velocity.x/desired.velocity.y);
  //das ist irgendwie quatsch TODO
  //ich denke die rotation müsste noch invertiert werden. ist immer genau die umgekehrte drehung. ist jetzt grade invertiert
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_generator");
  ros::NodeHandle n;
  TrajectoryGenerator tg;
  ros::Subscriber subDesired = n.subscribe("trajectory", 1000, &TrajectoryGenerator::updateTrajectory, &tg);
  ros::Publisher pub = n.advertise<vector_thruster::pose>("desired_pose",1000);
  ros::Rate loop_rate(tg.loopRate);

  /*tg.desired.velocity.x = 1.;
  tg.desired.velocity.y = 0.;
  tg.desired.velocity.z = 0.;
  tg.futureVelocity.x = 1.;
  tg.futureVelocity.y = -1.;
  tg.futureVelocity.z = 0.;
  tg.calcRotation();
  ROS_INFO("yRotation: %f and zRotation: %f", tg.desired.rotation.y, tg.desired.rotation.z);*/
  /*Waypoint first,second;
  first.position.x = 3.f;
  first.position.y = 3.f;
  first.position.z = 3.f;
  first.velocity.x = 0.7f;
  first.velocity.y = 0.7f;
  first.velocity.z = 0.7f;
  second.position.x = 10.f;
  second.position.y = 10.f;
  second.position.z = 10.f;
  second.velocity.x = -0.4f;
  second.velocity.y = 0.4f;
  second.velocity.z = 0.4f;
  tg.all.push_back(first);
  tg.all.push_back(second);*/
  while(ros::ok())
  {
    ROS_INFO("time: %f , maxTime: %f and start time: %f and w: %f", tg.time, tg.maxTime, tg.startTime.toSec(),tg.w);
    if(!tg.all.empty())
    {
      tg.checkIfReached();
      tg.calcPolynom();
      tg.calcRotation();
      //tg.desired.velocity = tg.desired.velocity * tg.w_dot;
      //tg.desired.acceleration = tg.desired.acceleration * tg.w_dot;
      //desired.rotation = desired.rotation * w_dot;
      pub.publish(tg.createMsg());
    }
    //ROS_INFO("position x: %f and velocity x: %f at %f", tg.desired.position.x, tg.desired.velocity.x, tg.time);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

//t_actual = t_actual + 1-(e^(t*-0.5)) * time_shift
