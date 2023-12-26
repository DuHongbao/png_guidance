#include<ros/ros.h>
#include<std_msgs/Empty.h>

int main(int argc, char** argv){
        ros::init(argc, argv, "traj_server");
        ros::NodeHandle nh("~");

        ROS_INFO("Helllo Traj_server");






        return 0;
}