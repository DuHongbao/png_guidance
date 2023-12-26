#ifndef _OAG_VISUALIZATION_H_
#define _OAG_VISUALIZATION_H_
#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdlib.h>
 using namespace std;

class OAGVisualization{
private:
        ros::NodeHandle node;
        ros::Publisher Drone_traj_pub;
        ros::Publisher Target_traj_pub;
        ros::Publisher MatrixXd_traj_pub;
        std::vector<ros::Publisher> Obstacles_traj_pub;
        ros::Publisher Acc_arrows_pub;
        ros::Publisher Vel_arrows_pub;
        ros::Publisher IntcPoint_pub;

public:
        OAGVisualization(/* args */) {}
        ~OAGVisualization() {}
        OAGVisualization(ros::NodeHandle &nh, int obs_num);

        typedef std::shared_ptr<OAGVisualization> Ptr;

        void displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id,  bool show_sphere = true);
        void displayDroneTraj(vector<Eigen::Vector3d> init_pts, const double scale, int id);
        void displayTargetTraj(vector<Eigen::Vector3d> init_pts, const double scale, int id);
        void displayMatrixXdTraj(Eigen::MatrixXd optimal_pts, int id);
        void displayObstacleTraj(int obs_num, vector<Eigen::Vector3d> init_pts, const double scale, int id);
        void displayAcc(vector<Eigen::Vector3d> pos_pts, vector<Eigen::Vector3d> acc_pts, const double scale, int id);
        void displayVel(vector<Eigen::Vector3d> pos_pts, vector<Eigen::Vector3d> vel_pts, const double scale, int id);
        void displayInterceptPoint(Eigen::Vector3d intc_point, Eigen::Vector4d color, const double scale, int id);
        void generateArrowDisplayArray(visualization_msgs::MarkerArray &array,const vector<Eigen::Vector3d> &pos_pts,
                                                        const vector<Eigen::Vector3d> &v_or_a, double scale, Eigen::Vector4d color, int id);
};


#endif