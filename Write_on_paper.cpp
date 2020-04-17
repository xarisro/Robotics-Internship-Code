#include <cmath>
#include <iostream>

#include <string>
#include <sstream>
#include <fstream>
#include <franka/exception.h>
#include <franka/robot.h>

#include "/home/franka/libfranka/examples/examples_common.h"

using namespace std; 

const int Time = 25000;

std::array<double, Time+1> x_points;
std::array<double, Time+1> y_points;
std::array<double, Time+1> z_points;

int main() {
  try {
    franka::Robot robot("192.168.2.105"); //Creating the robot object using the given robot IP
    setDefaultBehavior(robot);

    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Enter a phrase in uppercase and use _ for spaces" << std::endl;
    
    string word;
    std::cin >> word;

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    static double Kp = 20;
    static double Kd = 5;
    static double Ki = 15;

    std::ifstream ip;

    char letter[1] ;

    int count1 = 0;
    for(unsigned int j = 0; j < word.length(); j++){
      
      count1++;

      word.copy(letter, 1, j);
      //strcpy(letter, string(word[j])) ;

      if(count1 > 8) {
        count1 = 0;
        j--;
        //letter[0] = "q" ;
        string("q").copy(letter, 1, 0) ;
      } 
      
      std::ifstream ip(string("/home/franka/libfranka/examples/Harry_Controllers/Letters/") + letter + ".csv");
      
    

      if(!ip.is_open()) std::cout << "ERROR:File Open" << '\n';    

      franka::RobotState state = robot.readOnce();
      std::array<double, 16> initial_pose = state.O_T_EE_d; 

      x_points[0] = initial_pose[12];
      y_points[0] = initial_pose[13];
      z_points[0] = initial_pose[14];

      std::string longline;

      getline(ip, longline, '\n');
      std::stringstream ss(longline);
     
      std::string token0;

      getline(ss,token0, ',');
      double x1 = stod(token0);

      getline(ss,token0, ',');
      double y1 = stod(token0);

      getline(ss,token0, ',');
      double z1 = stod(token0);

      int c=1; 
      double scale = 0.6;
      if(letter[0] == 'q') scale = 1;

      for (std::string longline; std::getline(ip, longline, '\n'); ) {
          std::stringstream ss1(longline);

          getline(ss1,token0, ',');
          double x2 = stod(token0);

          getline(ss1,token0, ',');
          double y2 = stod(token0);

          getline(ss1,token0, ',');
          double z2 = stod(token0);

          x_points[c] = x_points[c-1] + scale * (x2 - x1);
          y_points[c] = y_points[c-1] + scale * (y2 - y1);
          z_points[c] = z_points[c-1] + (z2 - z1);

          x1 = x2;
          y1 = y2;
          z1 = z2;

          c++;
      }
      ip.close(); //closing the csv file

      double time = 0.0;
      int i = 0;

      double prev_vel_x = 0.0;
      double prev_vel_y = 0.0;
      double prev_vel_z = 0.0;

      double error_int_x = 0.0;
      double error_int_y = 0.0;
      double error_int_z = 0.0;

      robot.control([&time, &initial_pose, &i, &prev_vel_x, &prev_vel_y, &prev_vel_z,
        &error_int_x, &error_int_y, &error_int_z, &c](const franka::RobotState& robot_state,
                                           franka::Duration period) -> franka::CartesianVelocities {
        time += period.toSec();

        auto state_pose = robot_state.O_T_EE_d;
        std::array<double, 16> current_pose = state_pose; //Reading the current robot state

        error_int_x += (x_points[i] - current_pose[12]) * period.toSec();
        error_int_y += (y_points[i] - current_pose[13]) * period.toSec();
        error_int_z += (z_points[i] - current_pose[14]) * period.toSec();

        double acceleration_x = Kp*(x_points[i] - current_pose[12]) - Kd*prev_vel_x + Ki*error_int_x;
        double acceleration_y = Kp*(y_points[i] - current_pose[13]) - Kd*prev_vel_y + Ki*error_int_y;
        double acceleration_z = Kp*(z_points[i] - current_pose[14]) - Kd*prev_vel_z + Ki*error_int_z;

        double new_vel_x = prev_vel_x + acceleration_x * period.toSec();
        double new_vel_y = prev_vel_y + acceleration_y * period.toSec();
        double new_vel_z = prev_vel_z + acceleration_z * period.toSec();

        prev_vel_x = new_vel_x;
        prev_vel_y = new_vel_y;
        prev_vel_z = new_vel_z;

        franka::CartesianVelocities output = {{new_vel_x, new_vel_y, new_vel_z, 0.0, 0.0, 0.0}};

        if (i >= c - 20 && time > c/1000 + 1) {
          std::cout << std::endl << "Finished letter" << std::endl;
          return franka::MotionFinished(output); //newpose
        }
        else if(i<c - 20){
          i++;
        }
        return output;
      });
    }

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
