#ifndef _OAG_MANAGER_H_
#define _OAG_MANAGER_H_
#include <stdlib.h>
#include<memory>
#include<ros/ros.h>
#include "drone_trajs/oag_visualization.h"
#include "drone_trajs/plan_container.hpp"
#include "guidance_law/guidance_law.h" 



using namespace std;
namespace oaguider{




class OAGManager{
        public:
                OAGManager(){};
                ~OAGManager(){};
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                typedef unique_ptr<OAGManager> Ptr;
                void initGuiderModules(ros::NodeHandle &nh, OAGVisualization::Ptr vis = NULL);
                bool reboundReguide(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel);
                bool guideGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
                void deliverTrajToOptimizer();

                void getInterceptPt(Eigen::Vector3d &InterceptPoint);

                 void updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now);

                  bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);
        public:
                GuideParammeters gp_;
                LocalTrajData local_data_;
                double local_esti_duration_;
                GlobalTrajData global_data_;
                GuidanceLaw::Ptr guide_law_;
                bool force_new_polynomial_;

        private:
                OAGVisualization::Ptr  visualization_;

                int continous_failures_count_{0};


};



}  // namespace oaguider

#endif