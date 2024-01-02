#ifndef _OAGFSM_H_
#define  _OAGFSM_H_
#include<ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>

#include <oaguider/oag_manager.h>
#include <drone_trajs/DataDisp.h>
#include <drone_trajs/Bspline.h>
#include "drone_trajs/oag_visualization.h"


namespace oaguider
{
        class OagFSM{
                private:

                        enum FSM_STATE{
                                INIT,
                                WAIT_TARGET,
                                GEN_NEW_TRAJ,
                                REPLAN_TRAJ,
                                EXEC_TRAJ,
                                EMERGENCY_STOP,
                        };

                        enum TARGET_STATE{
                                STATIC_TARGET = 1,
                                MOVING_TARGET = 2
                        };
                        OAGManager::Ptr guider_manager_;
                        OAGVisualization::Ptr visualization_;
                        drone_trajs::DataDisp data_disp_;

                        /*parameters*/
                        int target_type_;
                        double waypoints_[50][3];
                        int waypoint_num_, wp_id_;
                        double planning_horizon_, planning_horizon_time_;
                        double emergency_time_;
                        double no_replan_thresh_, replan_thresh_;
                        bool flag_realworld_experiment_;
                        bool enable_fail_safe_;
                        bool flag_escape_emergency_;
                        bool use_wpts_;

                        /*guider data*/
                        bool have_trigger_, have_target_, have_odom_,have_new_target_;
                        FSM_STATE  exec_state_;
                        int continously_called_times_{0};

                        Eigen::Vector3d odom_pos_,odom_vel_,odom_acc_;
                        Eigen::Quaterniond odom_orient_;

                        Eigen::Vector3d init_pt_, start_pt_, start_vel_,start_acc_, start_yaw_;
                        Eigen::Vector3d end_pt_, end_vel_;
                        Eigen::Vector3d intercept_pt_, intercept_yaw_,old_intercept_pt_;
                        Eigen::Vector3d local_target_pt_;
                        std::vector<Eigen::Vector3d> wps_;
                        int current_wp_;
                        int obstacle_num_;

                        /*ROS utils*/
                        ros::NodeHandle node_;
                        ros::Timer exec_timer_, safety_timer_;
                        ros::Subscriber intercept_point_sub_, odom_sub_, waypoint_sub_, trigger_sub_; 
                        ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_;

                        void execFsmCbk(const ros::TimerEvent &e);
                        void checkCollisionCbk(const ros::TimerEvent &e);
                        void odomCbk(const nav_msgs::OdometryConstPtr &msg);
                        //void interceptPointCbk(const nav_msgs::PathConstPtr& msg);
                        //void triggerCbk(const geometry_msgs::PoseStampedPtr &msg);

                        void changeFSMExecState(FSM_STATE new_state, string pos_call);
                        bool callOaguiderTRAJ(const int trial_times);
                        void planNextWaypoint(const Eigen::Vector3d next_wp);
                        void targetCallback(const geometry_msgs::PoseStampedPtr &msg);  //change from waypointCallback
                        void calculateInterceptPoint(Eigen::Vector3d &targetCurrPt, Eigen::Vector3d &InterceptPt);
                        bool GuideFromGlobalTraj(const int trial_times);
                        bool GuideFromCurrentTraj(const int trial_times);
                        void getLocalTarget();
                        bool callReboundReguide();
                        bool callEmergencyStop(Eigen::Vector3d stop_pos);

                public:
                        OagFSM(){}
                        ~OagFSM(){}
                        void init(ros::NodeHandle &nh);

                       



                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };


}//namespace oaguider



#endif