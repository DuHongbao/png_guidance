#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>

int main(int argc, char** argv){
        ros::init(argc,argv,"oagpredictor");
        ros::NodeHandle nh;
        ros::Rate loop_rate(10);
        ros::Time begin = ros::Time::now(); 
        ros::Publisher drone_odom_pub = nh.advertise<nav_msgs::Odometry>("odom",2);
        ros::Publisher target_predict_pub = nh.advertise<nav_msgs::Odometry>("tarpos",2);
        //ros::Publisher obstacle_predict_pub;
        nav_msgs::OdometryPtr odom_msg = boost::make_shared<nav_msgs::Odometry>(); 
        nav_msgs::OdometryPtr target_msg = boost::make_shared<nav_msgs::Odometry>();   
        odom_msg->header.stamp = ros::Time::now();  
        odom_msg->header.frame_id = "map";  
        //odom_msg->child_frame_id = "base_link";  
        odom_msg->pose.pose.position.x = 0.0;  
        odom_msg->pose.pose.position.y = 0.0;  
        odom_msg->pose.pose.position.z = 0.0;  
        odom_msg->pose.covariance[0] = 0.01;  
        odom_msg->pose.covariance[7] = 0.01;  
        odom_msg->twist.twist.linear.x = -10.;  
        odom_msg->twist.twist.linear.y = 1.0;  
        odom_msg->twist.twist.linear.z = 0;  
        odom_msg->twist.covariance[0] = 0.01;  
        odom_msg->twist.covariance[7] = 0.01;  
        
        target_msg->header.stamp = ros::Time::now();  
        target_msg->header.frame_id = "map";  
        //target_msg->child_frame_id = "base_link";  
        target_msg->pose.pose.position.x = 50.0;  
        target_msg->pose.pose.position.y = 20.0;  
        target_msg->pose.pose.position.z = 20.0;  
        target_msg->pose.covariance[0] = 0.01;  
        target_msg->pose.covariance[7] = 0.01;  
        target_msg->twist.twist.linear.x = 2.0;  
        target_msg->twist.twist.linear.y = 1.5;  
        target_msg->twist.twist.linear.z = 0.0;  
        target_msg->twist.covariance[0] = 0.01;  
        target_msg->twist.covariance[7] = 0.01;  

        ros::Time current = ros::Time::now();
        while (ros::ok()){
                drone_odom_pub.publish(odom_msg); 
                target_predict_pub.publish(target_msg); 
                ros::spinOnce(); 
                loop_rate.sleep();  
        }  

        return 0;
}

