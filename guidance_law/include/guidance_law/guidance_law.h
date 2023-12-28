#ifndef _GUIDE_LAW_H_
#define _GUIDE_LAW_H_

#include <random>
#include <ros/ros.h>
#include <vector>
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

using namespace std;
namespace oaguider{
        const int RATE(10);

        class GuidanceLaw{


                public:
                        enum GuideType{
                                PN = 0,
                                CBF = 1,
                                JPSCBF = 2
                        };
                        ros::NodeHandle nh;

                        OAGVisualization::Ptr visualization_;
                        int obs_num;
                        double N_;
                        double tf_;
                        vehicle Drone;
                        vehicle Target;
                        vector<vehicle> Obstacles;
                        PolynomialTraj traj;
                        ros::Subscriber Obs_sub, Tar_sub, odom_sub,imu_sub;
                        double maxVel_, maxAcc_, maxJerk_;
                        static double maxVel;

                        vector<Eigen::Vector3d> droneTraj_, droneVelTraj_;
                        vector<Eigen::Vector3d> targetTraj_, targetVelTraj_;
                        PolynomialTraj dronePolyTraj;

                        Eigen::Vector3d InterceptedPt_;

                public:
                        typedef unique_ptr<GuidanceLaw> Ptr;
                        int guide_type_;
                        GuidanceLaw(){ROS_INFO("Hello!");};
                        GuidanceLaw(vehicle D, vehicle T){ 
                                Drone.init(D.role, D.veh_states);
                                Target.init(T.role, T.veh_states);
                        };

                        ~GuidanceLaw(){};

                        void init(std::string dname, std::string  tname, States D, States T);
                        void setID(std::string dname, std::string  tname);
                        void setVeh(States drone, States target);
                        void  setInterceptedPoint(Eigen::Vector3d  point);
                        void  getInterceptedPoint(Eigen::Vector3d  &point);

                        void setParam(ros::NodeHandle &nh);
                        void setDroneANDEnvStates(vector<Eigen::Vector3d> &drone, vector<Eigen::Vector3d> &target, vector<Eigen::Vector3d> &obs);
                        void plan();
                        void Eigen2Poly(vector<Eigen::Vector3d> &traj);
                        void simplifyToSevenPoints(vector<Eigen::Vector3d> &traj);
                      
                        bool  calcPNGuideTraj(vehicle &D, vehicle &T, vector<Eigen::Vector3d> &DTraj, vector<Eigen::Vector3d> &TTraj);
                        static PolynomialTraj guidePNTraj(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                    const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc, double trajtime);
                        bool  calcCBFGuideTraj(vehicle &D, vehicle &T, vector<Eigen::Vector3d> &DTraj, vector<Eigen::Vector3d> &TTraj);
        };

}
#endif