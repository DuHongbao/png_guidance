#ifndef _GUIDANCE_LAW_H_
#define _GUIDANCE_LAW_H_

#include <random>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include "drone_trajs/polynomial_traj.h"
#include "drone_trajs/uniform_bspline.h"
#include "drone_trajs/oag_visualization.h"
#include "guidance_law/vehicle.h"

const int RATE(10);
class GuidanceLaw{
        private:
                ros::NodeHandle nh;
                OAGVisualization::Ptr visualization_;
                int obs_num;
                vehicle Drone;
                vehicle Target;
                vector<vehicle> Obstacles;
                PolynomialTraj traj;
                ros::Subscriber Obs_sub, Tar_sub, odom_sub,imu_sub;
                double max_vel_;

        public:
                typedef unique_ptr<GuidanceLaw> Ptr;


                GuidanceLaw();
                GuidanceLaw( ros::NodeHandle n, int obsnum);
                ~GuidanceLaw(){};
                void obsCbk(const sensor_msgs::PointCloudPtr &obs_pos);
                void targetCbk(const nav_msgs::OdometryPtr &tar);
                void odomCbk(const nav_msgs::OdometryPtr &msg);
                void imuCbk(const sensor_msgs::ImuPtr &drone_imu);
                void init(ros::NodeHandle &nh);
                // void plan();
                void plan();
                void visilization(PolynomialTraj traj);
                PolynomialTraj  topolyTraj(std::vector<Eigen::Vector3d>  path);


};
#endif