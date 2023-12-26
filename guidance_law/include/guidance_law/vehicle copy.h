#ifndef  _VEHICLE_H_
#define _VEHICLE_H_

#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <Eigen/Eigen>

#define ST 0.1 //采样频率

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
                }
                States(){}
                ~States(){}      
};


class vehicle
{
private:
        char role;
        //Eigen::Vector3d pos, vel, acc;
        States veh_states;
        std::vector<Eigen::Vector3d> pos_hist, vel_hist, acc_hist;
        double N;
public:
        vehicle();
        vehicle(char _role,  States state);
        ~vehicle();
        States states();
        Eigen::Vector3d pn_guidance(Eigen::Vector3d r, Eigen::Vector3d vr);
        void update(double time_d);
        void update(Eigen::Vector3d targ_pos, Eigen::Vector3d targ_vel);
        std::vector<Eigen::Vector3d> veh_path();
        std::vector<Eigen::Vector3d> veh_vel_path();
        std::vector<Eigen::Vector3d> veh_acc_path();
};

vehicle::vehicle(){    }

vehicle::vehicle(char _role,  States state)
{
        role = _role;
        veh_states.pos = state.pos;
        veh_states.vel = state.vel;
        veh_states.acc = state.acc;
        N = 3.0;
}

vehicle::~vehicle(){}

Eigen::Vector3d vehicle::pn_guidance(Eigen::Vector3d r, Eigen::Vector3d vr){
        Eigen::Vector3d r_3d(0.0,  0.0,  0.0);
        Eigen::Vector3d vr_3d(0.0,  0.0,  0.0);
        Eigen::Vector3d rotation_vec(0.0,  0.0,  0.0);
        N = 3.0;
        r_3d = r;
        vr_3d = vr;
        rotation_vec = r_3d.cross(vr_3d)/(r_3d.dot(r_3d));
        Eigen::Vector3d a_cmd = N*vr_3d.cross(rotation_vec);
        return a_cmd;
}

void vehicle::update(double time_d){

                //veh_states.acc;可以设置机动目标 time_d
                veh_states.acc<<0.5,0.5,0.0;
                veh_states.acc(0) *= sin(0.05*time_d);
                veh_states.acc(1) *= cos(0.05*time_d);

                veh_states.vel += veh_states.acc * ST;
                veh_states.pos += veh_states.vel *ST;
}

void vehicle::update(Eigen::Vector3d targ_pos, Eigen::Vector3d targ_vel){
                Eigen::Vector3d rng;
                Eigen::Vector3d cv;
                Eigen::Vector3d pn_cmd;
                rng =  targ_pos - veh_states.pos;
                cv =   targ_vel - veh_states.vel;
                pn_cmd = pn_guidance(rng, cv);

                veh_states.acc = pn_cmd;

                veh_states.vel += veh_states.acc * ST;
                veh_states.pos += veh_states.vel *ST;

                pos_hist.push_back(veh_states.pos);
                vel_hist.push_back(veh_states.vel);
                acc_hist.push_back(veh_states.acc);        
}


States vehicle::states(){
        return veh_states;
}

std::vector<Eigen::Vector3d> vehicle::veh_path(){
        return pos_hist;
 }

std::vector<Eigen::Vector3d> vehicle::veh_vel_path(){
        return vel_hist;
 }

std::vector<Eigen::Vector3d> vehicle::veh_acc_path(){
        return acc_hist;
 }

#endif