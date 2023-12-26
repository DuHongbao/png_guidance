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
                bool reboundReguide();
                bool guideGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
                void deliverTrajToOptimizer();

                void getInterceptPt(Eigen::Vector3d &InterceptPoint);
        public:
                GuideParammeters gp_;
                LocalTrajData local_data_;
                GlobalTrajData global_data_;
                GuidanceLaw::Ptr guide_law_;

        private:
                OAGVisualization::Ptr  visualization_;

                int continous_failures_count_{0};


};



}  // namespace oaguider

#endif