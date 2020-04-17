#include <cmath>
#include <iostream>
#include <stdio.h>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/gripper.h>

#include <thread>

#include "armadillo"
#include <chrono>

using namespace arma;

using namespace std; 

using namespace std::chrono;
using namespace std::this_thread;

#include "/home/franka/libfranka/examples/examples_common.h"

#include "/home/franka/libfranka/build/gamepad/gamepad.h"

#include <libudev.h>

int i;

bool fist_time = true;

double target_gripper_width = 1.0;  
double target_gripper_speed = 0.1; 
double target_gripper_force = 15;
bool grasped = false;
franka::Gripper* gripper = nullptr;

double angle7 = 0.0;

double angle6 = 3.14/2;

int lx,ly;
int rx,ry;
std::array<double, 16> initial_pose;

double speedconst = 0.25, speed7 = 1.5, speed6 = 1, speedZconst = 0.15;
double speed, speedZ, signZ;
double temp;

bool start = false;
bool firstt_time = true;
bool grasp_button = false;

GAMEPAD_DEVICE DEV = GAMEPAD_0;

void GamePadCallback()
{
  while(!start){
    GamepadUpdate();

    GamepadStickXY(DEV, STICK_LEFT, &lx, &ly);
    GamepadStickXY(DEV, STICK_RIGHT, &rx, &ry);

    start = GamepadButtonDown(DEV, BUTTON_LEFT_THUMB) == GAMEPAD_TRUE;
    grasp_button = GamepadButtonDown(DEV, BUTTON_A) == GAMEPAD_TRUE;

    if(GamepadButtonDown(DEV, BUTTON_RIGHT_SHOULDER) == GAMEPAD_TRUE) signZ = 1.0;
    else if(GamepadButtonDown(DEV, BUTTON_LEFT_SHOULDER) == GAMEPAD_TRUE) signZ = -1.0;
    else signZ = 0;

    if(GamepadButtonDown(DEV, BUTTON_START) == GAMEPAD_TRUE) {
      speed = speedconst / 4;
      speedZ = signZ * speedZconst / 3;
    } 
    else {
      speed = speedconst;
      speedZ = signZ * speedZconst;
    } 

    angle6 -= speed6 * 0.001 * rx / 32767;

    if(firstt_time && GamepadTriggerValue(DEV, TRIGGER_LEFT) !=0){
      firstt_time = false;
    }
    else if(!firstt_time){
      temp = GamepadTriggerValue(DEV, TRIGGER_LEFT) - 128;
      if(abs(temp) > 10){
        angle7 += speed7 * 0.001 * (temp) / 128;
        if ((angle7 > 2.85 && temp > 0) || (angle7 < -2.85 && temp < 0)) angle7 -= speed7 * 0.001 * (temp) / 128;
      }
    }

    sleep_for(milliseconds(1));
  }
}

void graspp() {
  while (!start) {
    if(grasp_button){
      if(!grasped) {
        gripper->grasp(0.04, target_gripper_speed, target_gripper_force, 1, 1);
        grasped = true;
      }
      else {
        gripper->move(target_gripper_width, target_gripper_speed);
        grasped = false;
      } 
    }

    sleep_for(milliseconds(1)); 
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

int main() {

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

    GamepadInit();

    franka::RobotState state = robot.readOnce();
    initial_pose = state.O_T_EE;

    angle6 = state.q[5];
    angle7 = state.q[6];

    static std::thread t1(GamePadCallback);
    static std::thread t2(graspp);
          
    static double Kerr = 2.5;
    static double Kt = 0.02;
    static double Kp7 = 8;
    static double Kd7 = 2;

    static std::array<double, 7> Torque;
    static std::array<double, 7> current_qdot;
    static std::array<double, 7> current_q;

    static colvec new_vel(3);
    static colvec err(3);
    static colvec qdot(7);
    static mat Jac(3,7);
    static mat JacPlus(7,3);

    static franka::Model model(robot.loadModel());

    robot.control([=,&time](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::Torques {

      time += period.toSec();
      
      new_vel(0) = speed * ly / 32767;
      new_vel(1) = -speed * lx / 32767;
      new_vel(2) = speedZ;

      Jac = createJac(model.zeroJacobian(franka::Frame::kEndEffector, robot_state));  

      JacPlus = pinv(Jac); //pseudoinverse

      qdot = JacPlus * (new_vel + Kerr * err);
      current_q = robot_state.q;
      current_qdot = robot_state.dq;

      for(i=0; i<6; i++) {
        Torque[i] =  Kt * (qdot(i)-current_qdot[i])/0.001 + robot_state.tau_ext_hat_filtered[i]; // speed * (Kp*(q_d[i] - current_q[i]) - Kd*current_dot_q[i] + robot_state.tau_ext_hat_filtered[i]);
      }
      Torque[5] = Kp7 * (angle6 - current_q[5]) - Kd7 * current_qdot[5];
      Torque[6] = Kp7 * (angle7 - current_q[6]) - Kd7 * current_qdot[6];

      franka::Torques tau = franka::Torques(Torque);

      if (start) {
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
