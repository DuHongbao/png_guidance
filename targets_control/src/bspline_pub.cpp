#include "drone_trajs/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "drone_trajs/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <Eigen/Core>
using namespace std;




int main(int argc, char** argv){
        ros::init(argc, argv, "bspline_pub");

        ros::NodeHandle nh("~");

        ros::Publisher bspline_pub = nh.advertise<drone_trajs::Bspline>("/bspline", 10);


        while(ros::ok()){
                
                ROS_WARN("WARNNING!");
        }

        ros::spin();
        return 0;
}