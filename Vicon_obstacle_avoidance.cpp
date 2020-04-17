#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

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
colvec Vobst(6);
colvec qdotrep1(7), qdotrep2(7), qdotrep3(7), qdotrep4(7);
colvec qdotlimitrep(7);

mat obstacle(3,2);
mat Jac(6,7);
mat Jac2(6,7);
mat JacPlus(7,6);
mat JacPlusEnd(7,6);

double offsetx = 0.0;
double offsety = 0;
double offsetz = 0.55;

double x,y,z;

double obstacle_radius = 0.3;
double Krep = 0.15;
double Klimrep = 0.15;

double dist;
double total_dist;
double dist_from_marker = 0.15;

double angle1;
double vel_norm;
double z_ref_norm;

int sign3;

std::array<double, 3> x2,x3,z_ref;
std::array<double, 2> desired_angle;

franka::Robot robot("192.168.2.105");
franka::Model model(robot.loadModel());
franka::RobotState robot_state;
std::array<double, 16> current_pose, current_posee, initial_pose;
std::array<double, 7> current_q;

std::array<double, 7> Max_Angles = {{2.96, 1.83, 2.96, 0.087, 2.96, 3.82, 2.96}};   
std::array<double, 7> Min_Angles = {{-2.96, -1.83, -2.96, -3.14, -2.96, -0.087, -2.96}};

ros::Subscriber sub;

#define distance(x1, y1, z1, x2, y2, z2) sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) + (z2-z1)*(z2-z1));

inline double barrier(double eps, double d, double offset){
  return ((d < eps) ? (eps-d)/(d*d+offset) : 0.0);
} 

inline float angle(double Vax, double Vay, double Vbx, double Vby){
  if ((Vax*Vax+Vay*Vay)*(Vbx*Vbx+Vby*Vby) < 0.1) return 0 ;
  return acos((Vax*Vbx+Vay*Vby)/(sqrt((Vax*Vax+Vay*Vay)*(Vbx*Vbx+Vby*Vby))));
}

inline void crossProduct(std::array<double, 3> vect_A, std::array<double, 3> vect_B)
{ 
    z_ref[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1];
    z_ref[1] = vect_A[0] * vect_B[2] - vect_A[2] * vect_B[0];
    z_ref[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
}

void MarkersCallback(const vicon_bridge::Markers::ConstPtr& msg)
{
  x = -msg->markers[0].translation.x;
  y = -msg->markers[0].translation.y;
  z = msg->markers[0].translation.z;
  
  x2[0] = -msg->markers[1].translation.x - x;
  x2[1] = -msg->markers[1].translation.y - y;
  x2[2] = msg->markers[1].translation.z - z;

  x3[0] = -msg->markers[2].translation.x - x;
  x3[1] = -msg->markers[2].translation.y - y;
  x3[2] = msg->markers[2].translation.z - z;

  obstacle(0,0) = -msg->markers[3].translation.x/1000;
  obstacle(1,0) = -msg->markers[3].translation.y/1000;
  obstacle(2,0) = msg->markers[3].translation.z/1000 + 0.4+0.0;

  obstacle(0,1) = -msg->markers[6].translation.x/1000;
  obstacle(1,1) = -msg->markers[6].translation.y/1000;
  obstacle(2,1) = msg->markers[6].translation.z/1000 + 0.4+0.0;

  if (!(x==0 && y==0 && z==0)){
    crossProduct(x2,x3);

    z_ref_norm = distance(z_ref[0], z_ref[1], z_ref[2], 0, 0, 0);
    for(int k = 0; k<3; k++) z_ref[k] = z_ref[k] / z_ref_norm;
    
    int sign = (z_ref[1] > 0) ? 1 : -1;
    desired_angle[0] = - sign * angle(z_ref[1], z_ref[2], 0, -1);

    sign = (z_ref[0] > 0) ? 1 : -1;
    desired_angle[1] = - sign * angle(z_ref[0], z_ref[2], 0, -1);

    offsetx = -dist_from_marker * z_ref[0];
    offsety = dist_from_marker * z_ref[1];
    offsetz = - dist_from_marker * z_ref[2] + 0.4;

    pd(0) = x/1000 + offsetx; 
    pd(1) = y/1000 + offsety; 
    pd(2) = z/1000 + offsetz; 
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
  
inline mat createJac(std::array< double, 42 > Jackin, int length){
  int j,k;
  mat Jackout(length,7);
  for(j = 0; j < length; j++){
    for(k = 0; k < 7; k++){
      Jackout(j,k) = Jackin[6*k+j];
    }
  }
  return Jackout;
}

void UpdateQrep(){

      double norm;
      int i,j;
      std::array<double, 16> temp_pose;

      while(ros::ok()){
          //chrono::steady_clock::time_point beginn = chrono::steady_clock::now();
          
          //REPULSION FIELD FROM OBSTACLES

          //if(dist < 0.05) obstacle_radius = 0; //NOT NESSECARY

          current_posee = model.pose(franka::Frame::kJoint4, robot_state);
          
          Jac2 = createJac(model.zeroJacobian(franka::Frame::kJoint4, robot_state), 3);  
          JacPlus = pinv(Jac2);

          qdotrep1.fill(0);
          for(j = 0; j<2; j++){
              norm = distance(current_posee[12],current_posee[13],current_posee[14],obstacle(0,j),obstacle(1,j),obstacle(2,j));
              for(i = 0; i<3; i++) {
                Vobst(i) = (current_posee[12+i] - obstacle(i,j)) * barrier(obstacle_radius, norm, 0.05);
              }
              Vobst = Vobst/norm;
              qdotrep1 += Krep * JacPlus * Vobst.rows(0,2);
          }

          temp_pose = model.pose(franka::Frame::kJoint5, robot_state);
          for(i = 0; i<3; i++) temp_pose[12+i] = (temp_pose[12+i] + current_posee[12+i]) / 2;

          current_posee = temp_pose;  
          qdotrep2.fill(0);
          for(j = 0; j<2; j++){
              norm = distance(current_posee[12],current_posee[13],current_posee[14],obstacle(0,j),obstacle(1,j),obstacle(2,j));
              for(i = 0; i<3; i++) {
                Vobst(i) = (current_posee[12+i] - obstacle(i,j)) * barrier(obstacle_radius, norm, 0.05);
              }
              Vobst = Vobst/norm;
              qdotrep2 += Krep * JacPlus * Vobst.rows(0,2);
          }

          current_posee = model.pose(franka::Frame::kJoint5, robot_state);

          Jac2 = createJac(model.zeroJacobian(franka::Frame::kJoint6, robot_state), 3);  
          JacPlus = pinv(Jac2);

          qdotrep3.fill(0);
          for(j = 0; j<2; j++){
              norm = distance(current_posee[12],current_posee[13],current_posee[14],obstacle(0,j),obstacle(1,j),obstacle(2,j));
              for(i = 0; i<3; i++) {
                Vobst(i) = (current_posee[12+i] - obstacle(i,j)) * barrier(obstacle_radius, norm, 0.05);
              }
              Vobst = Vobst/norm;
              qdotrep3 += Krep * JacPlus * Vobst.rows(0,2);
          }

          current_posee = model.pose(franka::Frame::kEndEffector, robot_state);
          qdotrep4.fill(0);
          for(j = 0; j<2; j++){
              norm = distance(current_posee[12],current_posee[13],current_posee[14],obstacle(0,j),obstacle(1,j),obstacle(2,j));
              for(i = 0; i<3; i++) {
                Vobst(i) = (current_posee[12+i] - obstacle(i,j)) * barrier(obstacle_radius, norm, 0.05);
              }
              Vobst = Vobst/norm;
              qdotrep4 += Krep * JacPlusEnd * Vobst;
          }

          //chrono::steady_clock::time_point endd = chrono::steady_clock::now();
          //cout << chrono::duration_cast<chrono::nanoseconds> (endd - beginn).count() << endl;

          //JOINT ANGLE REPULSION FIELD

          for(j = 0; j<7; j++) qdotlimitrep(j) = -Klimrep * barrier(0.87, abs(Max_Angles[j] - current_q[j]), 0.0) + Klimrep * barrier(0.87, abs(Min_Angles[j] - current_q[j]), 0.0);


          sleep_for(milliseconds(1));

      }

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "vicon_subscriber");

  try {    
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

    //INITIALIZATION

    double time = 0.0;

    pd(0) = 0.45;
    pd(1) = 0.07;
    pd(2) = 0.45;

    desired_angle[0] = 0;
    desired_angle[1] = 0;

    z_ref[0] = 0;
    z_ref[1] = 0;
    z_ref[2] = -1;

    obstacle(0,0)=0.7;
    obstacle(1,0)=0;
    obstacle(2,0)=0.5;
    obstacle(0,1)=0.408676;
    obstacle(1,1)=-0.124560;
    obstacle(2,1)=0.315585;

    franka::RobotState state = robot.readOnce();
    initial_pose = state.O_T_EE;

    total_dist = distance(initial_pose[12],initial_pose[13],initial_pose[14],pd(0),pd(1),pd(2));
    if (total_dist < 0.3) total_dist = 0.3;

    //GAINS

    static std::thread t1(subscribe_target_motion);
    static std::thread t2(UpdateQrep);

    static double Kp = 10;
    static double Kd = 3;
    //static double KerrMax = 0.4;
    static double Kattr = 0.7;
    static double Kerr;

    static double KpOrr = 0.9;
    static double KdOrr = 3.5;

    sleep_for(milliseconds(600));

    static std::array<double, 7> Torque;
    static std::array<double, 7> current_qdot;

    //static std::array<double, 3> acceleration;
    static colvec new_vel(6);
    static colvec err(6);
    static colvec qdot(7);

    Vobst.fill(0.0);
    err.fill(0.0);
    new_vel.fill(0.0);

    int i;

    robot.control([=,&time,&i](const franka::RobotState& robot_statee,
                                         franka::Duration period) -> franka::Torques {

      //chrono::steady_clock::time_point beginn = chrono::steady_clock::now();
      current_pose = robot_statee.O_T_EE;

      robot_state = robot_statee;

      for(i = 0; i<3; i++) {
        err(i) = pd(i) - current_pose[12+i];
        new_vel(i) = Kattr * err(i); 
      }

      //LIMITING VELOCITY

      vel_norm = distance(new_vel(0), new_vel(1), new_vel(2), 0, 0, 0);
      if (vel_norm > 0.08){
        for(i = 0; i<3; i++) new_vel(i) = new_vel(i) * 0.08 / vel_norm;
      }

      //ORIENTATIONNNNNNNNN

      for(i = 3; i<5; i++){
        sign3 = (current_pose[12-i] > 0) ? 1 : -1;
        if(i == 4) sign3 = -sign3;
        angle1 = sign3 * angle(current_pose[12-i], current_pose[10], 0, -1);//z_ref[4-i], z_ref[2]);
        new_vel(i) = new_vel(i) + (KpOrr * (desired_angle[i-3] - angle1) - KdOrr * new_vel(i)) * period.toSec();
      }
      
      sign3 = ((x2[0] * current_pose[5] - x2[1] * current_pose[4] > 0) ? 1 : -1);
      new_vel(5) = new_vel(5) + (KpOrr * (0 - sign3 * angle(x2[0], x2[1], current_pose[4], current_pose[5])) - KdOrr * new_vel(5)) * period.toSec();
      if (new_vel(5) > 0.15) new_vel(5) = 0.15;

      //ATTRACTION FIELD

      Jac = createJac(model.zeroJacobian(franka::Frame::kEndEffector, robot_state), 6);  

      JacPlusEnd = pinv(Jac); //pseudoinverse

      dist = distance(current_pose[12],current_pose[13],current_pose[14],pd(0),pd(1),pd(2));
      
      Kp = 10 + (1 - dist/total_dist) * 40;
      Kd = 3 + (1 - dist/total_dist) * 7;

      if(dist > 0.005) Kerr = 0.005;
      else Kerr = 0;
      
      double normm = distance(0, 0, 0, err(0), err(1), err(2));
      qdot = JacPlusEnd * (new_vel + Kerr * err / normm);

      //ADDING OBSTACLE REPULTION FIELDS

      qdot += qdotrep1;
      qdot += qdotrep2;
      qdot += qdotrep3;
      qdot += qdotrep4;

      //ADDING JOINT LIMIT REPULTION FIELD

      qdot += qdotlimitrep;

      //CONVERTING TO TORQUES
      current_q = robot_state.q;
      current_qdot = robot_state.dq;

      for(i=0; i<7; i++) {
        Torque[i] =  Kp * qdot(i) - Kd * current_qdot[i] + robot_state.tau_ext_hat_filtered[i]; 
      }

      franka::Torques tau = franka::Torques(Torque);

      if (!ros::ok()) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        t1.join();
        t2.join();
        return franka::MotionFinished(tau);
      }

      //chrono::steady_clock::time_point endd = chrono::steady_clock::now();
      //cout << chrono::duration_cast<chrono::nanoseconds> (endd - beginn).count() << endl;

      return tau;

    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}
