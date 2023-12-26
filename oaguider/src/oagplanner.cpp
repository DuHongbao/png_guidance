#include "guidance_law/guidance_law.h"
#include <signal.h>

int main(int argc, char** argv){
        ros::init(argc,argv,"oagplnnner");
        ros::NodeHandle nh;
        ros::Rate loop_rate(10);

        GuidanceLaw guiL(nh, 5);
        guiL.init(nh);

        //ros::spin();

        while(ros::ok()){

                guiL.plan();
                ros::spinOnce(); 
                loop_rate.sleep();
        }

        return 0;
}