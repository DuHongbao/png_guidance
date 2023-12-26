#ifndef  _VEHICLE_H_
#define _VEHICLE_H_

#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <Eigen/Eigen>


#define ST 0.1 //采样频率

typedef std::vector<Eigen::Vector3d> V3s;

class States{ 

        public:        
                Eigen::Vector3d pos;
                Eigen::Vector3d vel;
                Eigen::Vector3d acc;

        public:

                States(Eigen::Vector3d _pos, Eigen::Vector3d _vel, Eigen::Vector3d _acc){
                        pos = _pos;
                        vel = _vel;
                        acc = _acc;
                };

                States(const States& state){
                        pos = state.pos;
                        vel =    state.vel;
                        acc =  state.acc;
                };

                States(){
                        Eigen::Vector3d  vec = Eigen::Vector3d::Zero();
                        pos = vec;
                        vel = vec;
                        acc = vec;
                }
                ~States(){}      
};


class vehicle
{
public:
        std::string role;
        //Eigen::Vector3d pos, vel, acc;
        States veh_states;
        std::vector<Eigen::Vector3d> pos_hist, vel_hist, acc_hist;
        double N;
public:
        vehicle();
        vehicle(const vehicle& veh);
        vehicle( std::string _role,  States state);

        ~vehicle();
        States states();
        void init(std::string _role,  States state);
        void setVehicleID(std::string vehName);
        void  setState(States  state);

        void  getPos(Eigen::Vector3d &Pos);
        
        void  getVel(Eigen::Vector3d &Vel);
        


        Eigen::Vector3d pn_guidance(Eigen::Vector3d r, Eigen::Vector3d vr);
        Eigen::Vector3d opn_guidance(Eigen::Vector3d r, Eigen::Vector3d vr, V3s d, V3s vro);
        void update(double time_d);
        void update(Eigen::Vector3d targ_pos, Eigen::Vector3d targ_vel);
        void update(Eigen::Vector3d targ_pos, Eigen::Vector3d targ_vel,  V3s obs_pos, V3s obs_vel);
        std::vector<Eigen::Vector3d> veh_path();
        std::vector<Eigen::Vector3d> veh_vel_path();
        std::vector<Eigen::Vector3d> veh_acc_path();
};

#endif