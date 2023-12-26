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
                double global_duration_;

};





}//namespace 




#endif