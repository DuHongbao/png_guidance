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


double trajTime = 0.0;
PolynomialTraj guidePNTraj(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                    const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc, double trajtime){
        States state_A(start_pt, start_vel, start_acc);
        States state_B(end_pt, end_vel, end_acc);
        vehicle  A("drone", state_A), B("target",state_B);
        Eigen::Vector3d A_temp_;
        vector<Eigen::Vector3d>  Eigentraj;
        PolynomialTraj traj;
        double t = 0.0, tf = trajtime, ts = 1.5;

        ////////////calcPNGuideTraj(A,B,dtraj,ttraj);
        double distance;
        do{
                distance = (A.states().pos - B.states().pos).norm();
                while(t<tf){
                        t = 0.0;
                        B.update(t);
                        A.update(B.states().pos, B.states().vel);
                        distance = (A.states().pos - B.states().pos).norm();
                        A.getPos(A_temp_);
                        Eigentraj.push_back(A_temp_);
                        if(distance< 0.5){
                                cout<<"Find!"<<endl;
                                break;
                        }
                        t += ts;
                }
                A.setState(state_A);
                B.setState(state_B);
                
                ts *= 0.5;
                tf *= 1.2;
                cout<<"Find!!!"<<endl;
        }while(distance>0.5);

        ////////////simplify


        vector<Eigen::Vector3d> newTraj;
        int size = Eigentraj.size();
        for(int i = 0; i<size; ){
                newTraj.push_back(Eigentraj[i]);
                i +=( size/8 );
        }
        Eigentraj.clear();
        Eigentraj = newTraj;

        for(int i = 0; i< Eigentraj.size();i++){
                cout<<Eigentraj[i]<<endl;
        }
        ////////////////// to polytraj

        Eigen::MatrixXd pos(3, Eigentraj.size());
        Eigen::VectorXd time(Eigentraj.size() - 1);

        for (int i = 0; i < Eigentraj.size(); i++)
                {pos.col(i) = Eigentraj[i];}
                        
        for (int i = 0; i < Eigentraj.size() - 1; i++)
                {
                        time(i) = (pos.col(i + 1) - pos.col(i)).norm() / 10.0;
                        trajTime += time(i);
                        cout<<"Time:"<<time(i)<<endl;
                }  
        //time(0) *= 2.0;
        //time(time.rows() - 1) *= 2.0;
        cout<<"Here"<<pos<<endl;
        Eigen::Vector3d ev = Eigen::Vector3d::Zero();
        Eigen::Vector3d ea = Eigen::Vector3d::Zero();
        traj = PolynomialTraj::minSnapTraj(pos, start_vel, ev, start_acc, ea, time);
       // traj = PolynomialTraj::one_segment_traj_gen(Eigentraj[0], start_vel, start_acc, Eigentraj.back(), ev, ea, 19);
        
        return traj;
}


int main(int argc, char **argv)
{
        ros::init(argc, argv, "test_node");
        ros::NodeHandle nh("~");
        Eigen::Vector3d startPt, endPt, startVel, endVel, startAcc, endAcc;
        trajTime = 0.0;
        double  time(100.0);
        startPt<<0.0, 0.0, 0.0;
        startVel<<1.0, 1.0, 0.0;
        startAcc <<0.0,0.0,0.0;
        
        endPt << 1.0,5.0, 0.4;
        endVel<< 0.0, 1.0, 1.0;
        endAcc << 0.0, 0.0, 0.0;

        PolynomialTraj traj;

        traj = guidePNTraj(startPt, startVel, startAcc, endPt, endVel, endAcc, time);

        cout<<"Length:"<<traj.getTimeSum() <<endl;

        cout<<"Vel:"<<traj.getMeanVel()<<endl;


        for (double t = 0.0; t < trajTime; t += 0.1){
                cout<<traj.evaluate(t)[0]<<","<<traj.evaluate(t)[1] <<","<<traj.evaluate(t)[2]<<endl;;
        }
        // for(int i = 0; i< traj.getTimes().size(); i++){
        //         cout<<traj.getTimes()[i]<<endl;
        // }


        ros::spin();
        return 0;
}
