#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/gripper.h>

#include "ros/ros.h"
#include "vicon_bridge/Markers.h"

#include <thread>
#include <chrono>

#include "armadillo"

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

ros::Subscriber sub;

double offsetx = 0.0;
double offsety = 0;
double offsetz = 0.55;

double x;
double y;
double z;
double dist;
double dist2;

double Kp = 0.6;
double Kd = 0.9;
double Kerr = 3;
double Kt = 0.001;

franka::Gripper* gripper = nullptr;
double target_gripper_width = 1.0;  
double target_gripper_speed = 0.1; 
double target_gripper_force = 15;
franka::GripperState grstate;
bool release = false;
bool stop = false;

int marker = 0;

double max_dist = sqrt(0.738607*0.738607+0.177282*0.177282+0.619284*0.619284);

inline double distance(double x1, double y1, double z1, double x2, double y2, double z2){
  return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) + (z2-z1)*(z2-z1));
}

void MarkersCallback(const vicon_bridge::Markers::ConstPtr& msg)
{
  x = -msg->markers[marker].translation.x;
  y = -msg->markers[marker].translation.y;
  z = msg->markers[marker].translation.z;

  dist = distance(x/1000,y/1000,z/1000,-offsetx,-offsety,-offsetz);

  if(x!=0 && y!=0 && z!=0 && dist < max_dist){
    pd(0) = x/1000 + offsetx; 
    pd(1) = y/1000 + offsety; 
    pd(2) = z/1000 + offsetz;   
  }

  if (stop)
    {
      if(!release){
          gripper->grasp(0.04, target_gripper_speed, target_gripper_force, 1, 1);
          stop = false;
        }
      else {
        gripper->move(target_gripper_width, target_gripper_speed);
        stop = false;
      }
      release = !release;
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

int main(int argc, char** argv) {
  ros::init(argc, argv, "vicon_subscriber");

  try {
    franka::Robot robot("192.168.2.105");
    setDefaultBehavior(robot);

    gripper = new franka::Gripper("192.168.2.105");

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

    pd(0) = 0.45;
    pd(1) = 0.07;
    pd(2) = 0.45;

    static std::thread t1(subscribe_target_motion);

    sleep_for(milliseconds(600)); 

    static std::array<double, 3> acceleration;
    static colvec new_vel(3);
    static colvec prev_vel(3);
    static colvec err(3);

    franka::RobotState robot_state = robot.readOnce();

    static std::array<double, 16> current_pose = robot_state.O_T_EE;
    static double initial_dist = distance(current_pose[12],current_pose[13],0,pd(0),pd(1),0);
    static double zmax = initial_dist/20;
    static bool changed = false;

    for(i = 0; i<3; i++) prev_vel(i) = 0;

    static franka::Model model(robot.loadModel());

    robot.control([=,&time](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianVelocities {

      time += period.toSec();

      auto state_pose = robot_state.O_T_EE;
      current_pose = state_pose;

      dist2 = distance(current_pose[12],current_pose[13],0,pd(0),pd(1),0);

      if(changed && dist2 > 0.02){
        initial_dist = dist2;
        zmax = initial_dist/20;
        changed = false;
      }

      for(i = 0; i<3; i++) {
        err(i) = pd(i) - current_pose[12+i];
        if(!stop) acceleration[i] = Kp*err(i) - Kd*prev_vel(i);
        else acceleration[i] = -Kd*prev_vel(i);

        new_vel(i) = prev_vel(i) + acceleration[i] * period.toSec(); 
      }

      if(!stop) if(dist2/initial_dist > 0.35) new_vel(2) -= 1.0*(zmax*4/initial_dist - zmax*8*dist2 / (initial_dist*initial_dist)) * period.toSec();

      for(i = 0; i<3; i++) prev_vel(i) = new_vel(i);

       franka::CartesianVelocities output = {{new_vel(0), new_vel(1), new_vel(2), 0.0, 0.0, 0.0}};

      if(!stop && dist2 < 0.02 && abs(current_pose[14]-pd(2)) < 0.02 && distance(prev_vel(0),prev_vel(1),prev_vel(2),0,0,0) < 0.02) {
          if (marker == 0) marker = 3;
          else if (marker == 3) marker = 0;  
          changed = true;
          stop = true ;
          // pd(0) = 0.55;
          // pd(1) = -0.3;
          // pd(2) = 0.45;
      }

      if (!ros::ok()) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        t1.join();
        return franka::MotionFinished(output);
      }

      return output;

    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}
