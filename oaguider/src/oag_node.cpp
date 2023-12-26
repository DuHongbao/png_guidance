#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <oaguider/oag_manager.h>
#include <oaguider/oagfsm.h>

using namespace  oaguider;
int main(int argc, char **argv){
        ros::init(argc, argv, "oaguider_node");
        ros::NodeHandle nh("~");
        
        OagFSM oa_guider;
        oa_guider.init(nh);
        ros::spin();
        return 0;
}
