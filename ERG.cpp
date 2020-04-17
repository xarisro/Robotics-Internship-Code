#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

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
colvec qvdotrep1(7), qvdotrep2(7), qvdotrep3(7), qvdotrep4(7);
colvec qvdotlimitrep(7), qvattr(7);
colvec new_vel(6);
colvec err(6);
colvec qvdot(7),qv(7),ro(7);
colvec qsimdotdot(7);
colvec Dynamics(7);

double obstacles_number = 2;
mat obstacle(3,obstacles_number);
mat Jac(6,7);
mat JacPlus(7,6);

double Field_radius = 0.35, obstacle_radius = 0.13;
double Krep = 0.23;
double Klimrep = 0.15;

double Kattr = 0.7;
double KpOrr = 0.9;
double KdOrr = 3.5;
double Kerr = 0.005;

double Kp = 10;
double Kd = 3;

double Kq = 1, KT = 1.0/2.0, Kobst = 1.0/0.07;
double Kdelta = 9;
//double Kdelta = 1;

double dist, total_dist, distob;

double angle1;
double vel_norm;
double norm3;

double periodd;

int sign3;

std::array<double, 2> desired_angle;

franka::Robot robot("192.168.2.105");
franka::Model model(robot.loadModel());
franka::RobotState robot_state;
std::array<double, 16> current_pose, current_posee, initial_pose, sim_pose;
std::array<double, 7> current_q;
std::array<double, 7> qsim;
std::array<double, 7> qsimdot;

std::array<double, 7> Max_Angles = {{2.96, 1.83, 2.96, 0.087, 2.96, 3.82, 2.96}};   
std::array<double, 7> Min_Angles = {{-2.96, -1.83, -2.96, -3.14, -2.96, -0.087, -2.96}};

std::array<double, 7> Max_Torques = {{87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0}};   
std::array<double, 7> Min_Torques = {{-87.0, -87.0, -87.0, -87.0, -12.0, -12.0, -12.0}};

#define distance(x1, y1, z1, x2, y2, z2) sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) + (z2-z1)*(z2-z1));

inline double barrier(double eps, double d, double offset){
  return ((d < eps) ? (eps-d)/(d*d+offset) : 0.0);
} 

inline float angle(double Vax, double Vay, double Vbx, double Vby){
  if ((Vax*Vax+Vay*Vay)*(Vbx*Vbx+Vby*Vby) < 0.1) return 0 ;
  return acos((Vax*Vbx+Vay*Vby)/(sqrt((Vax*Vax+Vay*Vay)*(Vbx*Vbx+Vby*Vby))));
}

inline mat getJac(std::array< double, 42 > Jackin, int length){
  int j,k;
  mat Jackout(length,7);
  for(j = 0; j < length; j++){
    for(k = 0; k < 7; k++){
      Jackout(j,k) = Jackin[6*k+j];
    }
  }
  return Jackout;
}

inline mat getMass(std::array< double, 49 > Massin){
  int j,k;
  mat Massout(7,7);
  for(j = 0; j < 7; j++){
    for(k = 0; k < 7; k++){
      Massout(j,k) = Massin[7*k+j];
    }
  }
  return Massout;
}

inline colvec getCoriolis(std::array< double, 7 > Corin){
  int i;
  colvec Corout(7);
  for(i = 0; i < 7; i++) Corout(i) = Corin[i];
  return Corout;
}

inline colvec FindObstRepulsion(){
  int i,j;
  colvec result(7);
  result.fill(0);
  for(j = 0; j<obstacles_number; j++){
    norm3 = distance(current_posee[12],current_posee[13],current_posee[14],obstacle(0,j),obstacle(1,j),obstacle(2,j));
    for(i = 0; i<3; i++) {
      Vobst(i) = (current_posee[12+i] - obstacle(i,j)) * barrier(Field_radius, norm3, 0.05);
    }
    Vobst = Vobst/norm3;
    result += Krep * JacPlus * Vobst.rows(0,2);
  }
  return result;
}

inline double DynamicSafetyMargin(double TimeHorizon){
  int i;
  double timestep = 0.01;
  double Deltaq = 500000.0, deltaq = 0.05;
  double Deltaobst = 500000.0, deltaobst = 0.05;
  double DeltaT = 500000.0, deltaT = 0.05;
  double Delta;
  double Kp = 4, Kd = 0.25;

  const franka::RobotState robot_statee(robot_state);
  std::array<double, 16> temp_posee;
  mat Mass(7,7);
  colvec Coriolis(7), gravity(7), Torque_applied(7);

  for(i=0; i<7; i++){
    qsim[i] = robot_statee.q[i]; //Initializing joint angles
    qsimdot[i] = robot_statee.dq[i]; //Initializing joint velocities
  }  

  for(int k=0; k < TimeHorizon/timestep; k++){
    
    Mass = getMass(model.mass(qsim, robot_statee.I_total, robot_statee.m_total, robot_statee.F_x_Ctotal));
    Coriolis = getCoriolis(model.coriolis(qsim, qsimdot, robot_statee.I_total, robot_statee.m_total, robot_statee.F_x_Ctotal));

    for(i=0; i<7; i++) Dynamics(i) = -Coriolis(i) + Kp * (qv(i) - qsim[i]) - Kd * qsimdot[i];

    //Finding qdotdot by soving the linear system Mass * qdd = Dynamics

    qsimdotdot = solve(Mass, Dynamics);

    for(i=0; i<7; i++){
      
      qsimdot[i] = qsimdot[i] + qsimdotdot(i) * timestep;
      qsim[i] = qsim[i] + qsimdot[i] * timestep + qsimdotdot(i) * timestep * timestep / 2;
    }

    //For the Deltaq

    for(i=0; i<7; i++){
      Deltaq = min(Deltaq, qsim[i] - (1 + deltaq) * Min_Angles[i]);
      Deltaq = min(Deltaq, (1 - deltaq) * Max_Angles[i] - qsim[i]);
    }

    //For input saturation

    //getting gravity and coriolis is the same function
    gravity = getCoriolis(model.gravity(qsim, robot_statee.m_total, robot_statee.F_x_Ctotal, {{0., 0.,-9.81}})); 

    for(i=0; i<7; i++) Torque_applied(i) = Kp * (qv(i) - qsim[i]) - Kd * qsimdot[i] + gravity(i);
    
    for(i=0; i<7; i++){
      DeltaT = min(DeltaT, Torque_applied(i) - (1 + deltaT) * Min_Torques[i]);
      DeltaT = min(DeltaT, (1 - deltaT) * Max_Torques[i] - Torque_applied(i));
    }

    //For Minimum obstacle distance
    //position of the 4th joint
    sim_pose = model.pose(franka::Frame::kJoint4, qsim, robot_statee.F_T_EE, robot_statee.EE_T_K);          
    for(i = 0; i<obstacles_number; i++) distob =  distance(sim_pose[12],sim_pose[13],sim_pose[14],obstacle(0,i),obstacle(1,i),obstacle(2,i));
    Deltaobst = min(Deltaobst, distob - (1 + deltaobst) * obstacle_radius);

    //position of the 5th joint
    temp_posee = model.pose(franka::Frame::kJoint5, qsim, robot_statee.F_T_EE, robot_statee.EE_T_K); 
    for(i = 0; i<3; i++) temp_posee[12+i] = (temp_posee[12+i] + sim_pose[12+i]) / 2;
    
    sim_pose = temp_posee;  
    for(i = 0; i<obstacles_number; i++) distob =  distance(sim_pose[12],sim_pose[13],sim_pose[14],obstacle(0,i),obstacle(1,i),obstacle(2,i));
    Deltaobst = min(Deltaobst, distob - (1 + deltaobst) * obstacle_radius);

    //position of the 6th joint
    sim_pose = model.pose(franka::Frame::kJoint6, qsim, robot_statee.F_T_EE, robot_statee.EE_T_K);
    for(i = 0; i<obstacles_number; i++) distob =  distance(sim_pose[12],sim_pose[13],sim_pose[14],obstacle(0,i),obstacle(1,i),obstacle(2,i));
    Deltaobst = min(Deltaobst, distob - (1 + deltaobst) * obstacle_radius);

    //position of the end-effector
    sim_pose = model.pose(franka::Frame::kEndEffector, qsim, robot_statee.F_T_EE, robot_statee.EE_T_K);
    for(i = 0; i<obstacles_number; i++) distob =  distance(sim_pose[12],sim_pose[13],sim_pose[14],obstacle(0,i),obstacle(1,i),obstacle(2,i));
    Deltaobst = min(Deltaobst, distob - (1 + deltaobst) * obstacle_radius);
  
    //cout << qsim[1] << endl;
  }

  //cout << endl;
  Delta = min(Kq * Deltaq, KT * DeltaT);
  Delta = min(Delta, Kobst * Deltaobst);
  Delta = max(Delta, 0.0);

  return Kdelta * Delta;
  //return Delta;
}

void UpdateQrep(){

      double normm, Delta;
      int i,j;
      std::array<double, 16> temp_pose;

      while(true){
          //chrono::steady_clock::time_point beginn = chrono::steady_clock::now();
          
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
            new_vel(i) = new_vel(i) + (KpOrr * (desired_angle[i-3] - angle1) - KdOrr * new_vel(i)) * periodd;
          }

          //ATTRACTION FIELD

          Jac = getJac(model.zeroJacobian(franka::Frame::kEndEffector, robot_state), 6);  

          JacPlus = pinv(Jac); //pseudoinverse
          
          normm = distance(0, 0, 0, err(0), err(1), err(2));
          qvattr = JacPlus * (new_vel + Kerr * err / normm);




          //REPULSION FIELD FROM OBSTACLES

          //if(dist < 0.05) Field_radius = 0; //NOT NESSECARY
          //current_posee = current_pose;
          Jac = getJac(model.zeroJacobian(franka::Frame::kEndEffector, robot_state), 3);  
          JacPlus = pinv(Jac); //pseudoinverse
          qvdotrep1 = FindObstRepulsion();

          //position of the 4th joint
          current_posee = model.pose(franka::Frame::kJoint4, robot_state);          
          Jac = getJac(model.zeroJacobian(franka::Frame::kJoint4, robot_state), 3);  
          JacPlus = pinv(Jac);
          qvdotrep2 = FindObstRepulsion();

          //position of the 5th joint
          temp_pose = model.pose(franka::Frame::kJoint5, robot_state);
          for(i = 0; i<3; i++) temp_pose[12+i] = (temp_pose[12+i] + current_posee[12+i]) / 2;
          current_posee = temp_pose;  
          qvdotrep3 = FindObstRepulsion();

          //position of the 6th joint
          current_posee = model.pose(franka::Frame::kJoint6, robot_state);
          Jac = getJac(model.zeroJacobian(franka::Frame::kJoint6, robot_state), 3);  
          JacPlus = pinv(Jac);
          qvdotrep4 = FindObstRepulsion();

          //JOINT ANGLE REPULSION FIELD

          for(j = 0; j<7; j++) qvdotlimitrep(j) = -Klimrep * barrier(0.87, abs(Max_Angles[j] - current_q[j]), 0.0) 
                                                  +Klimrep * barrier(0.87, abs(Min_Angles[j] - current_q[j]), 0.0);
                                                 
          //ADDING OBSTACLE REPULTION FIELDS AND JOINT LIMIT REPULTION FIELD      

          ro = qvattr + qvdotrep1 + qvdotrep2 + qvdotrep3 + qvdotrep4 + qvdotlimitrep;                                             
          
          //Calculating Dynamic safety Margin

          Delta = DynamicSafetyMargin(1.5); 
          //cout << Delta << endl;

          //Calculating reference

          qvdot = ro * Delta;
          qv = qv + qvdot * periodd;

          //chrono::steady_clock::time_point endd = chrono::steady_clock::now();
          //cout << chrono::duration_cast<chrono::microseconds> (endd - beginn).count() << endl;

          //sleep_for(microseconds(800));

      }

}

int main() {

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

    pd(0) = 0.306201;
    pd(1) = 0.378428;
    pd(2) = 0.45;

    desired_angle[0] = 0;
    desired_angle[1] = 0;

    obstacle(0,0)=0.7;
    obstacle(1,0)=0;
    obstacle(2,0)=0.5;
    obstacle(0,1)=0.363247;
    obstacle(1,1)=-0.063366;
    obstacle(2,1)=0.267722;

    franka::RobotState state = robot.readOnce();
    initial_pose = state.O_T_EE;

    total_dist = distance(initial_pose[12],initial_pose[13],initial_pose[14],pd(0),pd(1),pd(2));
    if (total_dist < 0.3) total_dist = 0.3;

    for(int i=0; i<7; i++) qv(i) = state.q[i];


    static std::thread t2(UpdateQrep);

    static std::array<double, 7> Torque;
    static std::array<double, 7> current_qvdot;

    Vobst.fill(0.0);
    err.fill(0.0);
    new_vel.fill(0.0);

    int i;

    robot.control([=,&time,&i](const franka::RobotState& robot_statee,
                                         franka::Duration period) -> franka::Torques {

      //chrono::steady_clock::time_point beginn = chrono::steady_clock::now();
      current_pose = robot_statee.O_T_EE;
      robot_state = robot_statee;
      periodd = period.toSec();
      time += periodd;

      //CONVERTING TO TORQUES
      current_q = robot_state.q;
      current_qvdot = robot_state.dq;

      dist = distance(current_pose[12],current_pose[13],current_pose[14],pd(0),pd(1),pd(2));
      
      Kp = 40;
      Kd = 10;
      //Kp = 10 + (1 - dist/total_dist) * 40; //ALSO CHANGE Kdelta
      //Kd = 3 + (1 - dist/total_dist) * 7;

      if(dist > 0.005) Kerr = 0.005;
      else Kerr = 0;

      for(i=0; i<7; i++) {
        //Torque[i] =  Kp * qvdot(i) - Kd * current_qvdot[i] + robot_state.tau_ext_hat_filtered[i]; 
        Torque[i] =  Kp * (qv(i) - current_q[i]) - Kd * current_qvdot[i] + robot_state.tau_ext_hat_filtered[i]; 
      }

      franka::Torques tau = franka::Torques(Torque);

      if (time > 30) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
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
