#include "guidance_law/guidance_law.h"

namespace oaguider{


void GuidanceLaw::init(  ros::NodeHandle &nh  , std::string dname, std::string  tname, States D, States T){
        ROS_INFO("GuideLaw Init!");

        ROS_INFO("Set GuideLaw  Parameters!");
        nh.param("guide/guide_type", guide_type_, -1);
        nh.param("guide/maxVel",maxVel_, -1.0);
        nh.param("guide/maxAcc",maxAcc_, -1.0);
        nh.param("guide/maxJerk",maxJerk_, -1.0);
        nh.param("guide/N", N_, 3.0);
        nh.param("guide/sampleFinalTime", tf_, 100.0);

        Drone.init(dname,D);
        Target.init(tname, T);
}

void GuidanceLaw::setParam(double Vel, double Acc, double Jerk, double N){
        maxVel_ = Vel;
        maxAcc_ = Acc;
        maxJerk_ = Jerk;
        N_ = N;
}


void GuidanceLaw::setID(std::string dname, std::string  tname){
        Drone.setVehicleID(dname);
        Target.setVehicleID(tname);
}


void GuidanceLaw::setVeh(States drone, States target){
        Drone.setState(drone);
        Target.setState(target);

}

void  GuidanceLaw::setInterceptedPoint(Eigen::Vector3d  point){
        InterceptedPt_ =  point;
}

void  GuidanceLaw::getInterceptedPoint(Eigen::Vector3d  &point){
       point = InterceptedPt_;
}



void GuidanceLaw::setDroneANDEnvStates(vector<Eigen::Vector3d> &drone, vector<Eigen::Vector3d> &target, vector<Eigen::Vector3d> &obs){
        Drone.veh_states.pos = drone[0];
        Drone.veh_states.vel = drone[1];
        Drone.veh_states.acc = drone[2];
        Drone.N = N_;

        Target.veh_states.pos = target[0];
        Target.veh_states.vel = target[1];

        // Obstacles;
        obs_num  = obs.size()/2;//because the position and velocity.
        Obstacles.clear();
        for(int i = 0; i < obs_num; i++ ){
                Obstacles[i].veh_states.pos = obs[i];
                Obstacles[i].veh_states.vel = obs[i+1];
        }
}


void GuidanceLaw::plan(){
        droneTraj_.clear();
        targetTraj_.clear();
        double t(0.0), tf(100.0);
        double distance(0.0);
        Eigen::Vector3d D_temp_, T_temp_;
        Eigen::Vector3d O_temp_, vel_temp_, acc_temp_;

        if(guide_type_ == GuideType::PN)
        {
                calcPNGuideTraj(Target,Drone,droneTraj_,targetTraj_);
        }
        else if(guide_type_ == GuideType::CBF)
        {
                 calcCBFGuideTraj(Target,Drone,droneTraj_,targetTraj_);
        }
        else if(guide_type_ == GuideType::JPSCBF){

        }else{

        }

}


void GuidanceLaw::Eigen2Poly(vector<Eigen::Vector3d> &traj){
        Eigen::MatrixXd pos(3, traj.size());
        Eigen::VectorXd time(traj.size() - 1);
        Eigen::Vector3d end_vel = Eigen::Vector3d::Zero();
        Eigen::Vector3d end_acc = Eigen::Vector3d::Zero();

        for (int i = 0; i < traj.size(); i++)
                {pos.col(i) = traj[i];}
                        
        for (int i = 0; i < traj.size() - 1; i++){
                time(i) = (pos.col(i + 1) - pos.col(i)).norm() / maxVel_;
                //cout<<"maxVel_:"<<maxVel_<<endl;
        }
        


        time(0) *= 2.0;
        time(time.rows() - 1) *= 2.0;
        dronePolyTraj = PolynomialTraj::minSnapTraj(pos, Drone.veh_states.vel, end_vel, Drone.veh_states.acc, end_acc, time);
        //cout<<"dronePolyTraj.evaluateVel(0.5):"<<dronePolyTraj.evaluate(0.5)<<endl;
}


void GuidanceLaw::simplifyToSevenPoints(vector<Eigen::Vector3d> &traj){
        vector<Eigen::Vector3d> newTraj;
        int size = traj.size();
        for(int i = 0; i<size; ){
                newTraj.push_back(traj[i]);
                i +=( size/7 );
        }
        traj.clear();
        traj = newTraj;
}


bool  GuidanceLaw::calcPNGuideTraj(vehicle &D, vehicle &T, vector<Eigen::Vector3d> &DTraj, vector<Eigen::Vector3d> &TTraj){
        double t=0.0;
        Eigen::Vector3d  IntePt;
        DTraj.clear();
        droneTraj_.clear();
        TTraj.clear();
        Eigen::Vector3d D_temp_, T_temp_;
        double distance = (D.states().pos - T.states().pos).norm();

        while(t<100){
                T.targetUpdate(T.states().vel);
                //cout<<"T.states().vel:"<<T.states().vel(0)<<" "<<T.states().vel(1)<<" "<<T.states().vel(2)<<endl;
                //cout<<"T.states().pos:"<<T.states().pos(0)<<" "<<T.states().pos(1)<<" "<<T.states().pos(2)<<endl;
                D.update(T.states().pos, T.states().vel);
                distance = (D.states().pos - T.states().pos).norm();
                //cout<<"t:"<<t<<", distance:"<<distance<<endl;
                D.getPos(D_temp_);
                T.getPos(T_temp_);
                DTraj.push_back(D_temp_);
                droneTraj_.push_back(D_temp_);
                TTraj.push_back(T_temp_);

                if(distance<0.5){
                        //ROS_INFO("Target will intercepted after: %f seconds", t);

                        IntePt = D.states().pos;
                        //ROS_WARN("D.states().pos: %f, %f, %f", D.states().pos(0),D.states().pos(1),D.states().pos(2));
                        break;
                        
                }
                t+= 0.1;
        }
        if(distance<0.5){
                return true;
        }else{
                return false;
        }

}


// PolynomialTraj GuidanceLaw::guidePNTraj(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
//                                                     const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc, double trajtime){
//         States state_A(start_pt, start_vel, start_acc);
//         States state_B(end_pt, end_vel, end_acc);
//         vehicle  A("drone", state_A), B("target",state_B);
//         Eigen::Vector3d A_temp_;
//         vector<Eigen::Vector3d>  Eigentraj;
//         PolynomialTraj traj;
//         double t = 0.0, tf = trajtime, ts = 0.5;

//         ////////////calcPNGuideTraj(A,B,dtraj,ttraj);
//         double distance;
//         do{
//                 distance = (A.states().pos - B.states().pos).norm();
//                 while(t<tf){
//                         t = 0.0;
//                         B.update(t);
//                         A.update(B.states().pos, B.states().vel);
//                         distance = (A.states().pos - B.states().pos).norm();
//                         A.getPos(A_temp_);
//                         Eigentraj.push_back(A_temp_);
//                         if(distance< 0.5){
//                                 break;
//                         }
//                         t += ts;
//                 }
//                 A.setState(state_A);
//                 B.setState(state_B);
                
//                 ts *= 0.5;
//                 tf *= 1.2;
//         }while(distance>0.5);

//         ////////////simplify

//         vector<Eigen::Vector3d> newTraj;
//         int size = Eigentraj.size();
//         for(int i = 0; i<size; ){
//                 newTraj.push_back(Eigentraj[i]);
//                 i +=( size/8 );
//         }
//         Eigentraj.clear();
//         Eigentraj = newTraj;

//         ////////////////// to polytraj

//         Eigen::MatrixXd pos(3, Eigentraj.size());
//         Eigen::VectorXd time(Eigentraj.size() - 1);

//         for (int i = 0; i < Eigentraj.size(); i++)
//                 {pos.col(i) = Eigentraj[i];}
                        
//         for (int i = 0; i < Eigentraj.size() - 1; i++)
//                 {time(i) = (pos.col(i + 1) - pos.col(i)).norm() / 1.0;}  //1.0is the max velocity

//         time(0) *= 2.0;
//         time(time.rows() - 1) *= 2.0;

//         traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);

//         return traj;
// }


PolynomialTraj GuidanceLaw::guidePNTraj(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                    const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc, double trajtime){
        States state_A(start_pt, start_vel, start_acc);
        States state_B(end_pt, end_vel, end_acc);
        vehicle  A("drone", state_A), B("target",state_B);
        Eigen::Vector3d A_temp_;
        vector<Eigen::Vector3d>  Eigentraj;
        PolynomialTraj traj;
        double trajTime = 0.0;
        double t = 0.0, tf = trajtime, ts = 1.5;

        ////////////calcPNGuideTraj(A,B,dtraj,ttraj);
        double distance;
        do{
                distance = (A.states().pos - B.states().pos).norm();
                while(t<tf){
                        t = 0.0;
                        B.update(t);
                        A.update(B.states().pos, B.states().vel);
                        distance = (A.states().pos - B.states().pos).norm();
                        A.getPos(A_temp_);
                        Eigentraj.push_back(A_temp_);
                        if(distance< 0.5){
                                cout<<"Find!"<<endl;
                                break;
                        }
                        t += ts;
                }
                A.setState(state_A);
                B.setState(state_B);
                
                ts *= 0.5;
                tf *= 1.2;
                cout<<"Find!!!"<<endl;
        }while(distance>0.5);

        ////////////simplify


        vector<Eigen::Vector3d> newTraj;
        int size = Eigentraj.size();
        for(int i = 0; i<size; ){
                newTraj.push_back(Eigentraj[i]);
                i +=( size/8 );
        }
        Eigentraj.clear();
        Eigentraj = newTraj;

        for(int i = 0; i< Eigentraj.size();i++){
                ROS_INFO("Eigentraj[i]:%f, %f, %f", Eigentraj[i][0], Eigentraj[i][1], Eigentraj[i][2]);
                //cout<<Eigentraj[i]<<endl;
        }
        ////////////////// to polytraj

        Eigen::MatrixXd pos(3, Eigentraj.size());
        Eigen::VectorXd time(Eigentraj.size() - 1);

        for (int i = 0; i < Eigentraj.size(); i++)
                {pos.col(i) = Eigentraj[i];}
                        
        for (int i = 0; i < Eigentraj.size() - 1; i++)
                {
                        time(i) = (pos.col(i + 1) - pos.col(i)).norm() / 10.0;
                        trajTime += time(i);
                        ROS_INFO("Time[i]: %f", time(i));
                        //cout<<"Time:"<<time(i)<<endl;
                }  
        //time(0) *= 2.0;
        //time(time.rows() - 1) *= 2.0;
        cout<<"Here"<<pos<<endl;
        Eigen::Vector3d ev = Eigen::Vector3d::Zero();
        Eigen::Vector3d ea = Eigen::Vector3d::Zero();
        traj = PolynomialTraj::minSnapTraj(pos, start_vel, ev, start_acc, ea, time);
       // traj = PolynomialTraj::one_segment_traj_gen(Eigentraj[0], start_vel, start_acc, Eigentraj.back(), ev, ea, 19);
        
        return traj;
}

//于2023年12月28日晚验证guidePNTraj函数的正确性


bool GuidanceLaw::calcCBFGuideTraj(vehicle &D, vehicle &T, vector<Eigen::Vector3d> &DTraj, vector<Eigen::Vector3d> &TTraj){
        double t=0.0;
        Eigen::Vector3d  IntePt;
        DTraj.clear();
        TTraj.clear();
        Eigen::Vector3d D_temp_, T_temp_;
        while(t<tf_){
                T.update(t);
                D.update(T.states().pos, T.states().vel);
                double distance = (D.states().pos - T.states().pos).norm();
                D.getPos(D_temp_);
                T.getPos(T_temp_);
                DTraj.push_back(D_temp_);
                TTraj.push_back(T_temp_);

                if(distance<0.5){
                        std::cout<<"Intercepted at:"<<t<<std::endl;
                        IntePt = D.states().pos;
                        std::cout<<D.states().pos<<std::endl;
                        return true;
                }
                t+= ST;/*  */
        }
        return false;
}



}