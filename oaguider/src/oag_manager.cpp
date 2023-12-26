#include <oaguider/oag_manager.h>

#include <thread>
#include <visualization_msgs/Marker.h>

namespace oaguider{

        //1
        void OAGManager::initGuiderModules(ros::NodeHandle &nh, OAGVisualization::Ptr vis ){
                nh.param("manager/max_vel", gp_.maxVel_, -1.0);
                nh.param("manager/min_vel", gp_.maxAcc_, -1.0);
                nh.param("manager/max_jerk", gp_.maxJerk_, -1.0);
                nh.param("manager/feasibility_tolerance",gp_.feasibility_tolerance_, 0.0);
                nh.param("manager/control_points_distance", gp_.ctrl_pt_dist, -1.0);
                nh.param("manager/planning_horizon", gp_.guide_horizen_, 5.0);

                
                local_data_.traj_id_ = 0;
                visualization_ =vis;
                guide_law_.reset(new GuidanceLaw);
                guide_law_->setParam(nh);

        }

        //2
        bool OAGManager::reboundReguide(){
                vector<Eigen::Vector3d> drone;
                vector<Eigen::Vector3d> target;
                vector<Eigen::Vector3d> obstacles;

                guide_law_->setDroneANDEnvStates(drone, target, obstacles);
                guide_law_->plan();
                return true;
        }

        //3
        bool OAGManager::guideGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc){

                States DState(start_pos, start_vel, start_acc);
                States TState(end_pos, end_vel, end_acc);

                vehicle drone("temp_drone", DState);
                vehicle  target("temp_target", TState);

                GuidanceLaw temp_guider(drone, target);

                vector<Eigen::Vector3d>  DTraj, TTraj;
                //guide_law_->calcGuideTraj(guide_law_->Drone, guide_law_->Target, guide_law_->droneTraj_, guide_law_->targetTraj_);
                if (temp_guider.calcPNGuideTraj(drone, target, DTraj, TTraj))
                        return true;
                else{
                        return false;
                }
        }

        //4  可能改变
        void OAGManager::deliverTrajToOptimizer(){

        }

        //5  
        void OAGManager::calcInterceptPt(const Eigen::Vector3d startPt, const Eigen::Vector3d targetPt, Eigen::Vector3d &InterceptPoint){
                InterceptPoint<<0.0,0.0,0.0;
        }


}