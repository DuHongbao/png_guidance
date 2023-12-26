#include <nav_msgs/Path.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <random>

#include "drone_trajs/oag_visualization.h"
#include "drone_trajs/polynomial_traj.h"
#include "drone_trajs/uniform_bspline.h"
#include "guidance_law/guidance_law.h"
#include "guidance_law/vehicle.h"

int main(int argc, char** argv){
        ros::init(argc,argv,"png_node");
        ros::NodeHandle nh;
        ros::Rate loop_rate(10);
        int OBS_NUM(10);
        OAGVisualization::Ptr visualization_;
        visualization_.reset(new OAGVisualization(nh,OBS_NUM));
        std::vector<Eigen::Vector3d> droneTraj_;
        std::vector<Eigen::Vector3d> targetTraj_;
        std::vector<Eigen::Vector3d> velTraj_;
        std::vector<Eigen::Vector3d> accTraj_;
        std::vector<std::vector<Eigen::Vector3d>> obstaclesTraj_(OBS_NUM);
        ros::Time begin = ros::Time::now();
        double t(0.0), tf(100.0);
        double distance(0.0);
        
        
        std::default_random_engine generator;  
        std::uniform_real_distribution<double> distribution(0.0, 50.0);

        Eigen::Vector3d T_pos(50.0,50.0,50.0), T_vel(2.0, 1.50, 0.0), T_acc(0.0, 0.0, 0.0);
        Eigen::Vector3d D_pos(0.0, 0.0, 0.0), D_vel(10.0, 1.0, 0.0), D_acc(0.0, 0.0, 0.0);
        Eigen::Vector3d D_temp_,T_temp_,O_temp_, vel_temp_,acc_temp_;
        States T_init(T_pos, T_vel, T_acc);
        States D_init(D_pos, D_vel, D_acc);
        std::string Dstr = "Drone";
        std::string Tstr = "Targat";
        vehicle Drone(Dstr, D_init);
        vehicle Target(Tstr, T_init);
        vector<vehicle> Obstacles;
        
        for(int i = 0; i < OBS_NUM; i++){
                double rand_x = distribution(generator);
                double rand_y = distribution(generator);
                double rand_z = distribution(generator);
                double rand_vx = 0.01*distribution(generator);
                double rand_vy = 0.01*distribution(generator);
                double rand_vz = 0.01*distribution(generator);
                Eigen::Vector3d Obs_pos(rand_x,rand_y,rand_z), Obs_vel(rand_vx, rand_vy, rand_vz), Obs_acc(0.0, 0.0, 0.0);
                States Obs_init(Obs_pos, Obs_vel, Obs_acc);
                std::string Ostr = "obstacle";
                std::string numstr = std::to_string(i);
                Ostr += numstr;
                vehicle Obs(Ostr, Obs_init);
                Obstacles.push_back(Obs);
        }

        while(t<tf){
                Target.update(t);
                Drone.update(Target.states().pos, Target.states().vel);

                for(int i = 0; i < OBS_NUM; i++){
                        Obstacles[i].update(t);
                }

                distance = (Drone.states().pos - Target.states().pos).norm();
                D_temp_(0) = Drone.states().pos(0);
                D_temp_(1) = Drone.states().pos(1);
                D_temp_(2) = Drone.states().pos(2);
                T_temp_(0) = Target.states().pos(0);
                T_temp_(1) = Target.states().pos(1);
                T_temp_(2) = Target.states().pos(2);
                vel_temp_(0) = Drone.states().vel(0);
                vel_temp_(1) = Drone.states().vel(1);
                vel_temp_(2) = Drone.states().vel(2);
                acc_temp_(0) = Drone.states().acc(0);
                acc_temp_(1) = Drone.states().acc(1);
                acc_temp_(2) = Drone.states().acc(2);
                droneTraj_.push_back(D_temp_);
                targetTraj_.push_back(T_temp_);
                velTraj_.push_back(vel_temp_);
                accTraj_.push_back(acc_temp_);
                for(int i = 0; i < OBS_NUM; i++){
                        O_temp_(0) = Obstacles[i].states().pos(0);
                        O_temp_(1) = Obstacles[i].states().pos(1);
                        O_temp_(2) = Obstacles[i].states().pos(2);
                        obstaclesTraj_[i].push_back(O_temp_);
                }




                if(distance<0.5){
                        std::cout<<"Intercepted at:"<<t<<std::endl;
                        std::cout<<Target.states().pos<<std::endl;
                        std::cout<<Drone.states().pos<<std::endl;
                        break;
                }
                t+=ST;
                std::cout<<"Time:"<<t<<std::endl;

        }

        ros::Time end = ros::Time::now();
        std::cout<<"Cost Time:"<<(end-begin).toSec()<<std::endl;
                while(ros::ok()){
                        visualization_->displayDroneTraj(droneTraj_, 1, 0);
                        visualization_->displayTargetTraj(targetTraj_, 1, 0);
                        visualization_->displayInterceptPoint(targetTraj_.back(), Eigen::Vector4d(0.5,0,0.5,1),5.0, 0);
                        for(int i = 0; i < OBS_NUM; i++){
                                visualization_->displayObstacleTraj(i, obstaclesTraj_[i],5,0);
                        }
                        visualization_->displayVel(droneTraj_,velTraj_,1, 0);
                        visualization_->displayAcc(droneTraj_,accTraj_,2, 0);
                        ros::spinOnce(); 
                        loop_rate.sleep();
                }

        return 0;
}