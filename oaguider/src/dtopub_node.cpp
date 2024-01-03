#include <ros/ros.h>  
#include <nav_msgs/Odometry.h>  
  
int main(int argc, char** argv)  
{  
    // 初始化ROS节点  
    ros::init(argc, argv, "odom_publisher");  
      
    // 创建ROS节点句柄  
    ros::NodeHandle nh;  
      
    // 创建一个发布者对象，发布到"odom"话题，队列长度为10，使用无持久性的发布者  
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/predicted_target", 10);  
      
    // 创建一个Odometry消息对象  
    nav_msgs::Odometry odom;  
      
    // 设置时间戳和序列号（如果需要）  
    ros::Time current_time = ros::Time::now();  
    odom.header.stamp = current_time;  
    odom.header.seq = 0; // 序列号初始化为0或任何你想要的数字  
      
    // 设置其他Odometry消息字段（如位置、速度、方向等）  
    // ...  
      
    // 循环发布消息  
    ros::Rate loop_rate(1); // 设置发布频率为1Hz  
    while (ros::ok()) {  
        // 填充Odometry消息内容（如果需要更改）  
        // ...  
          
        // 发布消息  
        odom_pub.publish(odom);  
          
        // 等待下一个循环周期  
        loop_rate.sleep();  
    }  
    return 0;  
}