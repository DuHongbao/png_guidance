#include "drone_trajs/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "drone_trajs/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <Eigen/Core>
using namespace std;
        ros::Publisher pre_target_pub ;

        void targetCallback(const nav_msgs::OdometryConstPtr &msg){
            Eigen::Vector3d  tar_pt_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
            Eigen::Vector3d  tar_vel_ = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);

            nav_msgs::Odometry odom;
            odom.header.stamp    = ros::Time::now();
            odom.header.frame_id = "world";
                
            odom.pose.pose.position.x = msg->pose.pose.position.x;
            odom.pose.pose.position.y = msg->pose.pose.position.y;
            odom.pose.pose.position.z = msg->pose.pose.position.z;

            odom.pose.pose.orientation.w = 1;
            odom.pose.pose.orientation.x = 0;
            odom.pose.pose.orientation.y = 0;
            odom.pose.pose.orientation.z = 0;

            odom.twist.twist.linear.x = msg->twist.twist.linear.x;
            odom.twist.twist.linear.y = msg->twist.twist.linear.y;
            odom.twist.twist.linear.z = msg->twist.twist.linear.z;

            odom.twist.twist.angular.x = 0.0;
            odom.twist.twist.angular.y = 0.0;
            odom.twist.twist.angular.z = 0.0;
                
            pre_target_pub.publish(odom);


        }




int main(int argc, char **argv)
{
        ros::init(argc, argv, "targets_predict");
        // ros::NodeHandle node;
        ros::NodeHandle nh("~");
        ros::Rate loop_rate(1);


        pre_target_pub = nh.advertise<nav_msgs::Odometry>("/target2_pred", 1);      
        ros::Subscriber target_odom_sub;

        target_odom_sub = nh.subscribe("/target_odom_2", 1,  targetCallback);


        while(ros::ok()){
              loop_rate.sleep(); 
              ros::spinOnce();
        }



      return 0;
}