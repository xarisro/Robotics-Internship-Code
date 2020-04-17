#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/gripper.h>

#include "ros/ros.h"
#include "vicon_bridge/Markers.h"

#include <thread>

#include "armadillo"
#include <chrono>

using namespace arma;

using namespace std; 

using namespace std::chrono;
using namespace std::this_thread;

#include "/home/franka/libfranka/examples/examples_common.h"

colvec pd(3);

double x_ref;
double y_ref;
double z_ref;

int i;

bool fist_time = true;

double target_gripper_width = 1.0;  
double target_gripper_speed = 0.1; 
double target_gripper_force = 15;
bool grasped = false;
franka::Gripper* gripper = nullptr;

double angle7 = 0.0;
double angle7_ref;

double angle6 = 3.14/2;

ros::Subscriber sub;

double grip1x ;
double grip1y ;
double grip1z ;
double grip2x ;
double grip2y ;
double grip2z ;

double a;
double b;
double c;
double dist;

double a1;
double b1;
double c1;
double dist1;

double Vax;
double Vay;
double Vbx;
double Vby;

inline float angle(double Max, double May, double Mbx, double Mby, double Mcx,double Mcy,double Mdx,double Mdy){
  Vax= Max-Mbx ;
  Vay= May-Mby ;
  Vbx= Mcx-Mdx ;
  Vby= Mcy-Mdy ;
  //if ((Vax*Vax+Vay*Vay)*(Vbx*Vbx+Vby*Vby) < 0.001) return 0 ;
  //return acos((Vax*Vbx+Vay*Vby)/(sqrt((Vax*Vax+Vay*Vay)*(Vbx*Vbx+Vby*Vby))));
  return atan2(Vay,Vax) - atan2(Vby,Vbx); //atan2(Vay,Vax);
}

void MarkersCallback(const vicon_bridge::Markers::ConstPtr& msg)
{
  if (fist_time && msg->markers[3].translation.x!=0 && msg->markers[4].translation.x!=0 &&
         msg->markers[0].translation.x!=0 && msg->markers[1].translation.x!=0 && msg->markers[2].translation.x!=0){
    x_ref = -msg->markers[1].translation.x-pd(0)*1000;
    y_ref = -msg->markers[1].translation.y-pd(1)*1000;
    z_ref = msg->markers[1].translation.z-pd(2)*1000;
    //angle6_ref = angle();
    angle7_ref = 1.5 * angle( -msg->markers[4].translation.x,
                              -msg->markers[4].translation.y,
                              -msg->markers[3].translation.x,
                              -msg->markers[3].translation.y,
                              -msg->markers[0].translation.x,
                              -msg->markers[0].translation.y,
                              -msg->markers[1].translation.x,
                              -msg->markers[1].translation.y);
    fist_time = false;
  }
  else if (!fist_time) {
    if(msg->markers[1].translation.x!=0 && msg->markers[1].translation.y!=0 && msg->markers[1].translation.z!=0){
      pd(0) = (-msg->markers[1].translation.x-x_ref)/1000; //std::cout << x_d << std::endl;
      pd(1) = (-msg->markers[1].translation.y-y_ref)/1000; 
      pd(2) = (msg->markers[1].translation.z-z_ref)/1000; 
      //angle6 = angle();
      }
      if(msg->markers[3].translation.x!=0 && msg->markers[4].translation.x!=0 &&
         msg->markers[0].translation.x!=0 && msg->markers[1].translation.x!=0 && msg->markers[2].translation.x!=0){

        grip1x = -msg->markers[3].translation.x ;
        grip1y = -msg->markers[3].translation.y ;
        grip1z = msg->markers[3].translation.z ;
        grip2x = -msg->markers[4].translation.x ;
        grip2y = -msg->markers[4].translation.y ;
        grip2z = msg->markers[4].translation.z ;
      
        angle7 = 1.5 * angle( -msg->markers[4].translation.x,
                              -msg->markers[4].translation.y,
                              -msg->markers[3].translation.x,
                              -msg->markers[3].translation.y,
                              -msg->markers[0].translation.x,
                              -msg->markers[0].translation.y,
                              -msg->markers[1].translation.x,
                              -msg->markers[1].translation.y) - angle7_ref;
        // if (angle7 > 3.14) angle7 -= 3.14;
        // else if(angle7 < -3.14) angle7 += 3.14;
        cout << angle7 << endl;
      }
    }

  if((msg->markers[1].translation.x!=0) && (msg->markers[2].translation.x!=0)){
    a = -msg->markers[2].translation.x + msg->markers[1].translation.x;
    b = -msg->markers[2].translation.y + msg->markers[1].translation.y;
    c = msg->markers[2].translation.z - msg->markers[1].translation.z;
    dist = sqrt(a*a + b*b + c*c);
    angle6 = 0.065 * (90-dist) + 3.14/1.6;
  }
}

void graspp() {
  while (true) {
    a1 = grip1x - grip2x;
    b1 = grip1y - grip2y;
    c1 = grip1z - grip2z;
    dist1 = sqrt(a1*a1 + b1*b1 + c1*c1);

    if((grip2x!=0 && grip2y!=0 && grip2z!=0) && ( grip1x!=0 && grip1y!=0 && grip1z!=0)){
      if(dist1 < 50 && !grasped) {
        gripper->grasp(0.04, target_gripper_speed, target_gripper_force, 1, 1);
        grasped = true;
      }
      else if(grasped && dist1 >= 65) {
        gripper->move(target_gripper_width, target_gripper_speed);
        grasped = false;
      } 
    }

    sleep_for(milliseconds(1)); 
  }
}

void subscribe_target_motion() {
  // subscription stuff
  ros::NodeHandle n ;
  
  sub = n.subscribe("vicon/markers",10,MarkersCallback);  // 1: buffer size of message queue

  ros::Rate loop_rate(2000);  // 2 kHz
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep(); 
  }
}
  
inline mat createJac(std::array< double, 42 > Jackin){
  mat Jackout(3,7);
  for(int j = 0; j < 3; j++){
    for(int k = 0; k < 7; k++){
      Jackout(j,k) = Jackin[6*k+j];
    }
  }
  return Jackout;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "vicon_subscriber");

  try {
    franka::Robot robot("192.168.2.105");
    setDefaultBehavior(robot);

    std::array<double, 6> lower_force_thresholds_nominal{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_nominal{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    std::array<double, 6> lower_force_thresholds_acceleration{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_acceleration{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
        lower_force_thresholds_nominal, upper_force_thresholds_nominal);

    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    gripper = new franka::Gripper("192.168.2.105"); 
    
    double time = 0.0;

    static std::thread t1(subscribe_target_motion);
    static std::thread t2(graspp);

    pd(0) = 0.45;
    pd(1) = 0.07;
    pd(2) = 0.2;
          
    static double Kp = 8;
    static double Kd = 3;
    static double Kerr = 2.5;
    static double Kt = 0.02;
    static double Kp7 = 5;
    static double Kd7 = 2;

    static std::array<double, 7> Torque;
    static std::array<double, 7> current_qdot;
    static std::array<double, 7> current_q;

    static std::array<double, 3> acceleration;
    static colvec new_vel(3);
    static colvec prev_vel(3);
    static colvec err(3);
    static colvec qdot(7);
    static mat Jac(3,7);
    static mat JacPlus(7,3);

    for(i = 0; i<3; i++) prev_vel(i) = 0;

    static franka::Model model(robot.loadModel());

    robot.control([=,&time](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::Torques {

      time += period.toSec();

      auto state_pose = robot_state.O_T_EE;
      std::array<double, 16> current_pose = state_pose;

      for(i = 0; i<3; i++) err(i) = pd(i) - current_pose[12+i];
 
      for(i = 0; i<3; i++) acceleration[i] = Kp*err(i) - Kd*prev_vel(i);

      for(i = 0; i<3; i++) new_vel(i) = prev_vel(i) + acceleration[i] * period.toSec();

      Jac = createJac(model.zeroJacobian(franka::Frame::kEndEffector, robot_state));  

      JacPlus = pinv(Jac); //pseudoinverse

      qdot = JacPlus * (new_vel + Kerr * err);
      current_q = robot_state.q;
      current_qdot = robot_state.dq;

      for(i=0; i<5; i++) {
        Torque[i] =  Kt * (qdot(i)-current_qdot[i])/0.001 + robot_state.tau_ext_hat_filtered[i]; // speed * (Kp*(q_d[i] - current_q[i]) - Kd*current_dot_q[i] + robot_state.tau_ext_hat_filtered[i]);
      }
      Torque[5] = Kp7 * (angle6 - current_q[5]) - Kd7 * current_qdot[5];
      Torque[6] = Kp7 * (angle7 - current_q[6]) - Kd7 * current_qdot[6];
      //cout << Torque[6] << endl;
      if(abs(Torque[6]) > 2.5) Torque[6] = 0;

      //cout << Torque[6] << endl;

      for(i = 0; i<3; i++) prev_vel(i) = new_vel(i);

      franka::Torques tau = franka::Torques(Torque);

      if (!ros::ok()) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        t1.join();
        t2.join();
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
