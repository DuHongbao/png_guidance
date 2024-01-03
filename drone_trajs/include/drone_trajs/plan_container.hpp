#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include<Eigen/Eigen>
#include<vector>
#include<ros/ros.h>

#include<drone_trajs/polynomial_traj.h>
#include<drone_trajs/uniform_bspline.h>

using std::vector;

namespace oaguider{


struct GuideParammeters{
        double maxVel_, maxAcc_, maxJerk_;
        double ctrl_pt_dist;
        double feasibility_tolerance_; 
        double guide_horizen_;

        double time_search_ = 0.0;
        double time_oa_search_ = 0.0;
};

struct LocalTrajData{
        int traj_id_;
        double duration_;
        ros::Time start_time_;
        Eigen::Vector3d start_pos_;
        UniformBspline position_traj_, velocity_traj_, acceleration_traj_;
};

class GlobalTrajData{
        public:
                PolynomialTraj global_traj_;
                vector<UniformBspline> local_traj_;

                double global_duration_;
                ros::Time  global_start_time_;
                double  local_start_time_, local_end_time_;
                double time_increase_;
                double  last_time_inc_;
                double last_progress_time_;


                void setGlobalTraj(const PolynomialTraj &traj, const ros::Time &time)
                {
                        global_traj_ = traj;
                        global_traj_.init();
                        global_duration_ = global_traj_.getTimeSum();
                        global_start_time_ = time;

                        local_traj_.clear();
                        local_start_time_ = -1;
                        local_end_time_ = -1;
                        time_increase_ = 0.0;
                        last_time_inc_ = 0.0;
                        last_progress_time_ = 0.0;
                }

                Eigen::Vector3d getVelocity(double t){
                        if (t >= -1e-3 && t <= local_start_time_){
                                return global_traj_.evaluateVel(t);
                        }
                        else if (t >= local_end_time_ && t <= global_duration_ + 1e-3){
                                return global_traj_.evaluateVel(t - time_increase_);
                        }
                        else{
                                double tm, tmp;
                                local_traj_[0].getTimeSpan(tm, tmp);
                                return local_traj_[1].evaluateDeBoor(tm + t - local_start_time_);
                        }
                }

                Eigen::Vector3d getPosition(double t){

                        if (t >= -1e-3 && t <= local_start_time_){
                                return global_traj_.evaluate(t - time_increase_ + last_time_inc_);
                        }
                        else if (t >= local_end_time_ && t <= global_duration_ + 1e-3){
                                return global_traj_.evaluate(t - time_increase_);
                        }
                        else{
                                double tm, tmp;
                                local_traj_[0].getTimeSpan(tm, tmp);
                                return local_traj_[0].evaluateDeBoor(tm + t - local_start_time_);
                        }
                }

};





}//namespace 




#endif