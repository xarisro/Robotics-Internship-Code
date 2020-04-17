#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

#include "ros/ros.h"
#include "vicon_bridge/Markers.h"

#include "armadillo"

#include <thread>

using namespace std; 
using namespace arma;

using namespace std::chrono;
using namespace std::this_thread;

#include "/home/franka/libfranka/examples/examples_common.h"

int i;
int count3 = 0;

bool fist_time = true;

ros::Subscriber sub;

double x;
double y;
double z;

double x12;
double y12;
double z12;

double x13;
double y13;
double z13;

double refx = 0.00425;
double refy = 0.00405;

int sign2;
int sign3;
double dist;
double angle1;
double end_of_programm_dist;

bool first_time = true;

double prev_x13;
double prev_y13;

double Vax;
double Vay;
double Vbx;
double Vby;
double angle7;

double z_ref = 0;
double last_joint_ref = 0;

std::array<double, 2> starting_vec;
std::array<double, 2> vectorr;
std::array<double, 2> stick_vel = {0,0};
std::array<double, 2> ang_vel = {0,0};
std::array<double, 20> avg_vectorx = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
std::array<double, 20> avg_vectory = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
std::array<double, 2> pd;
std::array<double, 2> avg;


#define distance(x1, y1, z1, x2, y2, z2) sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) + (z2-z1)*(z2-z1));

inline mat createJac(std::array< double, 42 > Jackin, int length){
  mat Jackout(length,7);
  for(int j = 0; j < length; j++){
    for(int k = 0; k < 7; k++){
      Jackout(j,k) = Jackin[6*k+j];
    }
  }
  return Jackout;
}

inline float angle(double Max, double May, double Mbx, double Mby, double Mcx,double Mcy,double Mdx,double Mdy){
  Vax= Max-Mbx ;
  Vay= May-Mby ;
  Vbx= Mcx-Mdx ;
  Vby= Mcy-Mdy ;
  return atan2(Vay,Vax) - atan2(Vby,Vbx);
}

void MarkersCallback(const vicon_bridge::Markers::ConstPtr& msg)
{
  x = -msg->markers[2].translation.x;
  y = -msg->markers[2].translation.y;
  z = msg->markers[2].translation.z;

  x13 = -msg->markers[1].translation.x;
  y13 = -msg->markers[1].translation.y;
  z13 = msg->markers[1].translation.z;

  x12 = -msg->markers[0].translation.x;
  y12 = -msg->markers[0].translation.y;
  z12 = msg->markers[0].translation.z;

  if(msg->markers[4].translation.x!=0 || msg->markers[4].translation.y!=0){
    pd[0] = -msg->markers[4].translation.x/1000;
    pd[1] = -msg->markers[4].translation.y/1000;
  }

  if(msg->markers[6].translation.x!=0 && msg->markers[7].translation.x!=0 &&
     msg->markers[3].translation.x!=0 && msg->markers[4].translation.x!=0){
    
    angle7 = 1.3 * angle( -msg->markers[7].translation.x,
                          -msg->markers[7].translation.y,
                          -msg->markers[6].translation.x,
                          -msg->markers[6].translation.y,
                          -msg->markers[3].translation.x,
                          -msg->markers[3].translation.y,
                          -msg->markers[4].translation.x,
                          -msg->markers[4].translation.y);

    end_of_programm_dist = distance(  -msg->markers[7].translation.x,
                                      -msg->markers[7].translation.y,
                                      -msg->markers[7].translation.z,
                                      -msg->markers[6].translation.x,
                                      -msg->markers[6].translation.y,
                                      -msg->markers[6].translation.z);
  }

  if(x!=0 || y!=0 || z!=0) {

    stick_vel[0] = ((x13 - prev_x13) * 1000);
    stick_vel[1] = ((y13 - prev_y13) * 1000);

    avg_vectorx[count3 % 20] = stick_vel[0];
    avg_vectory[count3 % 20] = stick_vel[1];
    avg[0] = 0;
    avg[1] = 0;
    for(int j = 0; j < 20; j++){
      avg[0] += avg_vectorx[j];
      avg[1] += avg_vectory[j]; 
    }
    if(0.0105 * avg[0] > 800) avg[0] = 0;
    if(0.0105 * avg[1] > 800) avg[1] = 0;
    count3++;

    ang_vel[0] = ((x-x12)/1000 - vectorr[0]) * 1000;
    ang_vel[1] = ((y-y12)/1000 - vectorr[1]) * 1000;
    vectorr[0] = (x-x12)/1000;
    vectorr[1] = (y-y12)/1000;

    prev_x13 = x13;
    prev_y13 = y13;
  }
 }

void subscribe_target_motion() {
  ros::NodeHandle n ;
  
  sub = n.subscribe("vicon/markers",10,MarkersCallback);  // 1: buffer size of message queue

  ros::Rate loop_rate(2000);  // 2 kHz
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep(); 
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "vicon_subscriber");

  try {
    franka::Robot robot("192.168.2.105");

    setDefaultBehavior(robot);

robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});    

    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    double time = 0.0;

    static std::thread t1(subscribe_target_motion);

    static std::array<double, 2> ref = {refx, refy};
    static std::array<double, 7> Torque;
    static std::array<double, 7> current_q;
    static std::array<double, 7> current_qdot;
    static std::array<double, 2> acceleration;

    static colvec new_vel(6);
    static colvec err(6);
    static colvec qdot(7);
    static mat Jac(6,7);
    static mat JacPlus(7,6);

    sleep_for(milliseconds(600));

    err(3) = 0;
    err(4) = 0;
    err(5) = 0;

    new_vel(2) = 0;
    new_vel(3) = 0;
    new_vel(4) = 0;
    new_vel(5) = 0;

    static double Kd = 56; 
    static double Kang = 500;
    static double Kt = 0.03;
    static double Kvel = 0.0095;
    static double Kangvel = 1.6;
    static double Kmove = 40;

    static franka::Model model(robot.loadModel());

    while (abs(ref[0] - vectorr[0]) > 0.015 || abs(ref[1] - vectorr[1]) > 0.015) cout << abs(ref[1] - vectorr[1]) << endl;

    robot.control([=,&time](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::Torques {

      time += period.toSec();

      auto state_pose = robot_state.O_T_EE;
      std::array<double, 16> current_pose = state_pose;

      current_q = robot_state.q;
      current_qdot = robot_state.dq;

      if (first_time) {
        z_ref = current_pose[14];
        last_joint_ref = current_q[6] - angle7;
        first_time = false;
        starting_vec[0] = current_pose[12] - pd[0];
        starting_vec[1] = current_pose[13] - pd[1];
      }

      dist = distance(current_pose[12],current_pose[13],0,pd[0]+starting_vec[0],pd[1]+starting_vec[1],0);
      Kmove = max((60 - 190 * dist), 25.0);

      for(i = 0; i<2; i++) {
        acceleration[i] = -Kmove * (pd[i] + starting_vec[i] - current_pose[12+i]) + Kang * (ref[i] - vectorr[i]) + Kangvel * ang_vel[i] + Kvel * avg[i]/20 - Kd * new_vel(i);
        new_vel(i) = new_vel(i) + acceleration[i] * period.toSec();
      }
      new_vel(2) = 7.5 * (z_ref - current_pose[14]);

      Jac = createJac(model.zeroJacobian(franka::Frame::kEndEffector, robot_state), 6);  

      JacPlus = pinv(Jac); //pseudoinverse

      qdot = JacPlus * new_vel;

      for(i=0; i<6; i++) {
        Torque[i] =  Kt * (qdot(i)-current_qdot[i])/0.001 + robot_state.tau_ext_hat_filtered[i]; 
      }
      Torque[6] = 3 * (last_joint_ref + angle7 - current_q[6]) - 1 * current_qdot[6];
      if(abs(Torque[6]) > 2.5) Torque[6] = 0;

      franka::Torques tau = franka::Torques(Torque);

      if (!ros::ok() || end_of_programm_dist < 50) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        t1.join();
        return franka::MotionFinished(tau);
      }

      return tau;

    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}
