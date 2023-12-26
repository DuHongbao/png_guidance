#include "guidance_law/vehicle.h"




vehicle::vehicle(){    }

vehicle::vehicle(const vehicle& veh){
        veh_states = veh.veh_states;
        role = veh.role;
}

vehicle::vehicle( std::string _role,  States state)
{
        role = _role;
        veh_states.pos = state.pos;
        veh_states.vel = state.vel;
        veh_states.acc = state.acc;
        N = 3.0;
}

vehicle::~vehicle(){}

void vehicle::init(std::string _role,  States state){
        role = _role;
        veh_states.pos = state.pos;
        veh_states.vel = state.vel;
        veh_states.acc = state.acc;
        N = 3.0;
}
void vehicle::setVehicleID(std::string vehName){
        role = vehName;
}


void  vehicle::setState(States  state){
        veh_states = state;
}

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

Eigen::Vector3d vehicle::opn_guidance(Eigen::Vector3d r, Eigen::Vector3d vr, V3s d, V3s vro){
        Eigen::Vector3d a_cmd(0.0 ,0.0 ,0.0);
        std::vector<double> BX;
        std::vector<double> dBX_dt;
        double ro = 0.5;//障碍物尺度
        /************* obstacle avoidance control law*************/
        /*              BX = 0.5*(dxvro)^2 - vro^2ro^2                             */
        /*       dBX_dt = [d^3 dot(theta)cos(sigma-theta) - voro^2sin(sigma - sigma_o)]a      */
        /*                             LfB + LgBa >= -beta B                             */
        /*                     a* = an - max(0, LgBan - betaB(x))/LgB                           */
        BX[0] = 0.5*(d[0].cross(vro[0]).squaredNorm() - vro[0].squaredNorm()*ro*ro);

       return a_cmd;
}


void vehicle::update(double time_d){

        if(this->role =="Targat" ){
                veh_states.acc<<1.0,0.0, 0.0;
        }
        else{
                veh_states.acc<<0.0,0.0, 0.0;
        }
        //veh_states.acc;可以设置机动目标 time_d
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

void vehicle::update(Eigen::Vector3d targ_pos, Eigen::Vector3d targ_vel,  V3s obs_pos, V3s obs_vel){
        Eigen::Vector3d r;                    //无人机与目标相对距离
        Eigen::Vector3d vr;                       //无人机与目标相对速度T-D
        V3s d;                      //无人机与障碍相对距离
        V3s vro;                    //无人机与障碍相对速度
        Eigen::Vector3d opn_cmd;        //避障制导律

        r =  targ_pos - veh_states.pos; 
        vr =   targ_vel - veh_states.vel;

        for(int i = 0; i < obs_pos.size(); i++){
                d.push_back( obs_pos[i] - veh_states.pos );
                vro.push_back(obs_vel[i] - veh_states.vel);
        }

        opn_cmd = opn_guidance(r, vr ,d, vro);
        veh_states.acc = opn_cmd;

        veh_states.vel += veh_states.acc * ST;
        veh_states.pos += veh_states.vel *ST;
        pos_hist.push_back(veh_states.pos);
        vel_hist.push_back(veh_states.vel);
        acc_hist.push_back(veh_states.acc);  
}

States vehicle::states(){
        return veh_states;
}

void  vehicle::getPos(Eigen::Vector3d &Pos){
        Pos = veh_states.pos;
}


void  vehicle::getVel(Eigen::Vector3d &Vel){
        Vel = veh_states.vel;
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
