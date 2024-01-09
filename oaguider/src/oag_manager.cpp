#include <oaguider/oag_manager.h>

#include <thread>
#include <visualization_msgs/Marker.h>

namespace oaguider{

        //1
        void OAGManager::initGuiderModules(ros::NodeHandle &nh, OAGVisualization::Ptr vis ){
                nh.param("guide/max_vel", gp_.maxVel_, 2.0);
                nh.param("guide/max_acc", gp_.maxAcc_, 3.0);
                nh.param("guide/max_jerk", gp_.maxJerk_, 4.0);
                nh.param("guide/feasibility_tolerance", gp_.feasibility_tolerance_, 0.05);
                nh.param("guide/control_points_distance", gp_.ctrl_pt_dist, 0.4);
                nh.param("guide/guide_horizon", gp_.guide_horizen_, 5.0);

                
                local_data_.traj_id_ = 0;
                visualization_ =vis;

                States drone,target;
                std::string droneID = "drone509"; 
                std::string targetID = "paperTiger";

                guide_law_.reset(new GuidanceLaw);
                //guide_law_->setParam(nh);
                guide_law_->init(nh, droneID, targetID, drone, target);
                //guide_law_->setID(droneID, targetID );
                //guide_law_->setVeh(drone, target);
 

                visualization_ = vis;
        }

        //2
        bool OAGManager::reboundReguide(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,Eigen::Vector3d local_target_vel){

                ROS_WARN("reboundReguide()");
                static int count = 0;

                if((start_pt - local_target_pt).norm() < 0.2){
                        cout << "Close to goal" << endl;
                        continous_failures_count_++;
                        return false;
                }
                ROS_WARN("reboundReguide()__");
                ros::Time t_start = ros::Time::now();
                ros::Duration t_init, t_guide, t_refine;

                double ts = (start_pt - local_target_pt).norm() > 0.1 ? gp_.ctrl_pt_dist / gp_.maxVel_ * 1.5 : gp_.ctrl_pt_dist / gp_.maxVel_ * 5; 
                vector<Eigen::Vector3d> drone;
                vector<Eigen::Vector3d> target;
                vector<Eigen::Vector3d> obstacles;
                ROS_WARN("ts:%f",ts);
                //guide_law_->setDroneANDEnvStates(drone, target, obstacles);
                //guide_law_->plan();

                vector<Eigen::Vector3d> point_set, start_end_derivatives;
                static bool flag_first_call = true, flag_force_polynomial = false;
                bool flag_regenerate = false;

                do 
                {
                        point_set.clear();
                        start_end_derivatives.clear();
                        flag_regenerate = false;

                        // Intial path  generated from a min-snap traj by older.
                        if(flag_first_call || flag_force_polynomial){
                                flag_first_call = false;
                                flag_force_polynomial = false;

                                PolynomialTraj   gl_traj;
                                double dist = (start_pt - local_target_pt).norm();
                                double time = pow(gp_.maxVel_, 2) / gp_.maxAcc_ > dist ? 
                                sqrt(dist / gp_.maxAcc_)
                                 : (dist - pow(gp_.maxVel_, 2) / gp_.maxAcc_) / gp_.maxVel_ + 2 * gp_.maxVel_ / gp_.maxAcc_;
                                
                                
                                Eigen::MatrixXd pos(3, 3);
                                Eigen::VectorXd local_t;


                                ROS_WARN("local_esti_duration_ %f: ", local_esti_duration_);
                                ROS_WARN("global_duration_ %f: ", global_data_.global_duration_);

                                double t;
                                bool flag_too_far;
                                ts *= 1.5;
                                do{

                                        ts /= 1.5;
                                        point_set.clear();
                                        flag_too_far = false;  
                                        Eigen::Vector3d  last_pt = global_data_.global_traj_.evaluate(0);                                      
                                        for(t = 0; t < local_esti_duration_; t+=ts){
                                                Eigen::Vector3d pt = global_data_.global_traj_.evaluate(t);
                                                double points_dist = (last_pt - pt).norm();
                                                if(points_dist > gp_.ctrl_pt_dist * 1.5){
                                                        flag_too_far = true;
                                                        break;
                                                }
                                                last_pt = pt;
                                                point_set.push_back(pt);

                                        }
                                }while(flag_too_far);


                                cout<<"______________________"<<endl;
                                //gl_traj = GuidanceLaw::guidePNTraj(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
                                //gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), local_t);

                                
                                
                                // ts *= 1.5;
                                // do{
                                //         ts /= 1.5;
                                //         point_set.clear();
                                //         flag_too_far = false;
                                //         Eigen::Vector3d  last_pt = gl_traj.evaluate(0);
                                //         for(t = 0; t < time; t+=ts){
                                //                 Eigen::Vector3d pt = gl_traj.evaluate(t);
                                //                 double points_dist = (last_pt - pt).norm();
                                //                 if(points_dist > gp_.ctrl_pt_dist * 1.5){
                                //                         flag_too_far = true;
                                //                         break;
                                //                 }
                                //                 last_pt = pt;
                                //                 point_set.push_back(pt);
                                //         }
                                // }while(flag_too_far);

                                t -= ts;
                                start_end_derivatives.push_back(global_data_.global_traj_.evaluateVel(0));
                                start_end_derivatives.push_back(local_target_vel);
                                start_end_derivatives.push_back(global_data_.global_traj_.evaluateAcc(0));
                                start_end_derivatives.push_back(global_data_.global_traj_.evaluateAcc(t));
                                
                                cout<<"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$"<<endl;
                                for(int i = 0; i<point_set.size();i++){
                                        ROS_INFO("point_set[i]%f, %f, %f",  point_set[i](0), point_set[i](1), point_set[i](2));
                                }

                        
                        }else{
                                // Initial path generated from previous trajectory.
                                double t;
                                double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

                                vector<double> pseudo_arc_length;
                                vector<Eigen::Vector3d> segment_point;
                                pseudo_arc_length.push_back(0.0);
                                for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
                                {
                                        segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
                                        //cout<<"t:"<<t<<"  local_data_.duration_"<<local_data_.duration_<<endl;
                                        if (t > t_cur){
                                                pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
                                        }
                                }
                                cout<<"t:"<<t<<endl;
                                t -= ts;
                                double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / gp_.maxVel_ * 2.0;
                                if (poly_time > ts)
                                {
                                        PolynomialTraj gl_traj = GuidanceLaw::guidePNTraj(local_data_.position_traj_.evaluateDeBoorT(t), local_data_.velocity_traj_.evaluateDeBoorT(t), 
                                        local_data_.acceleration_traj_.evaluateDeBoorT(t), local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);
                                        for(t = ts; t < poly_time; t += ts){
                                                if(!pseudo_arc_length.empty())
                                                {
                                                        segment_point.push_back(gl_traj.evaluate(t));
                                                        pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
                                                }else{
                                                        ROS_ERROR("pseudo_arc_length is empty, return!");
                                                        continous_failures_count_++;
                                                        return false;
                                                }
                                        }
                                }

                                double sample_length = 0;
                                double cps_dist = gp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
                                size_t id = 0;
                                do
                                {
                                        cps_dist /= 1.5;
                                        point_set.clear();
                                        sample_length = 0;
                                        id = 0;
                                        while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
                                        {
                                                if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
                                                {
                                                        point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                                                                (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
                                                        sample_length += cps_dist;
                                                }
                                                else
                                                        id++;
                                        }
                                //point_set.push_back(local_target_pt);

                                } while (point_set.size() < 7); // If the start point is very close to end point, this will help


                                start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
                                start_end_derivatives.push_back(local_target_vel);              //
                                start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur)); 
                                start_end_derivatives.push_back(Eigen::Vector3d::Zero());

                                if (point_set.size() > gp_.guide_horizen_ / gp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
                                {
                                        flag_force_polynomial = true;
                                        flag_regenerate = true;
                                }

                        }
                }while(flag_regenerate);


                Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
                UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

                UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
                pos.setPhysicalLimits(gp_.maxVel_, gp_.maxAcc_, gp_.feasibility_tolerance_);

                updateTrajInfo(pos, ros::Time::now());    //更新局部轨迹 

                //time spend
                static double sum_time = 0;
                static int count_success = 0;
                sum_time += (t_init + t_guide + t_refine).toSec();
                count_success++;
                cout << "total time:\033[42m" << (t_init + t_guide + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_guide).toSec() << ",refine:" << t_refine.toSec() << ",avg_time=" << sum_time / count_success << endl;

                continous_failures_count_ = 0;
                return true;
        }

        //3
        bool OAGManager::guideGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc){
                vector<Eigen::Vector3d> points;
                points.push_back(start_pos);
                points.push_back(end_pos);

                States DState(start_pos, start_vel, start_acc);
                States TState(end_pos, end_vel, end_acc);

                vehicle drone("temp_drone", DState);
                vehicle  target("temp_target", TState);

                GuidanceLaw temp_guider(drone, target);
                //temp_guider.reset(new GuidanceLaw);
                //temp_guider->init(nh, droneID, targetID, drone, target);
                temp_guider.setParam(gp_.maxVel_, gp_.maxAcc_, gp_.maxJerk_, 3.0);

                vector<Eigen::Vector3d>  DTraj, TTraj;
                //guide_law_->calcGuideTraj(guide_law_->Drone, guide_law_->Target, guide_law_->droneTraj_, guide_law_->targetTraj_);
                if (temp_guider.calcPNGuideTraj(drone, target, DTraj, TTraj)){

                        guide_law_->setInterceptedPoint(DTraj.back());

                        // cout<<"DTraj.back():"<<DTraj.back()<<endl;

                        temp_guider.simplifyToSevenPoints(DTraj);
                        temp_guider.Eigen2Poly(DTraj);

                        auto time_now = ros::Time::now();
                        global_data_.setGlobalTraj(temp_guider.dronePolyTraj, time_now);

                        //cout<<"temp_guider.dronePolyTraj.evaluateVel(0.5):"<<temp_guider.dronePolyTraj.evaluateVel(0.5)<<endl;

                        return true;
                }
                else
                {
                        return false;
                }
        }

        //4  可能改变
        void OAGManager::deliverTrajToOptimizer(){

        }

        //5  
        void OAGManager::getInterceptPt(Eigen::Vector3d &InterceptPoint){
                guide_law_->getInterceptedPoint(InterceptPoint);
        }

        //6
        void OAGManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
        {
                local_data_.start_time_ = time_now;
                local_data_.position_traj_ = position_traj;
                local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
                local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
                local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
                local_data_.duration_ = local_data_.position_traj_.getTimeSum();
                local_data_.traj_id_ += 1;
        }




}