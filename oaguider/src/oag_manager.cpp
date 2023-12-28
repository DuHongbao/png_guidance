#include <oaguider/oag_manager.h>

#include <thread>
#include <visualization_msgs/Marker.h>

namespace oaguider{

        //1
        void OAGManager::initGuiderModules(ros::NodeHandle &nh, OAGVisualization::Ptr vis ){
                nh.param("manager/max_vel", gp_.maxVel_, -1.0);
                nh.param("manager/min_vel", gp_.maxAcc_, -1.0);
                nh.param("manager/max_jerk", gp_.maxJerk_, -1.0);
                nh.param("manager/feasibility_tolerance",gp_.feasibility_tolerance_, 0.0);
                nh.param("manager/control_points_distance", gp_.ctrl_pt_dist, -1.0);
                nh.param("manager/planning_horizon", gp_.guide_horizen_, 5.0);

                
                local_data_.traj_id_ = 0;
                visualization_ =vis;

                States drone,target;
                std::string droneID = "drone509"; 
                std::string targetID = "paperTiger";

                guide_law_.reset(new GuidanceLaw);
                guide_law_->init(droneID, targetID, drone, target);
                //guide_law_->setID(droneID, targetID );
                //guide_law_->setVeh(drone, target);
                guide_law_->setParam(nh);

                visualization_ = vis;
        }

        //2
        bool OAGManager::reboundReguide(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,Eigen::Vector3d local_target_vel){
                static int count = 0;

                if((start_pt - local_target_pt).norm() < 0.2){
                        cout << "Close to goal" << endl;
                        continous_failures_count_++;
                        return false;
                }

                ros::Time t_start = ros::Time::now();
                ros::Duration t_init, t_guide, t_refine;

                double ts = (start_pt - local_target_pt).norm() > 0.1 ? gp_.ctrl_pt_dist / gp_.maxVel_ * 1.5 : gp_.ctrl_pt_dist / gp_.maxVel_ * 5; 
                vector<Eigen::Vector3d> drone;
                vector<Eigen::Vector3d> target;
                vector<Eigen::Vector3d> obstacles;

                guide_law_->setDroneANDEnvStates(drone, target, obstacles);
                //guide_law_->plan();

                vector<Eigen::Vector3d> point_set, start_end_derivatives;
                static bool flag_first_call = true, flag_force_polynomial = false;
                bool flag_regenerate = false;

                do 
                {
                        point_set.clear();
                        start_end_derivatives.clear();
                        flag_regenerate = false;

                        // Intial path  generated from a min-snaptraj by older.
                        if(flag_first_call || flag_force_polynomial){
                                flag_first_call = false;
                                flag_force_polynomial = false;

                                PolynomialTraj   gl_traj;
                                double dist = (start_pt - local_target_pt).norm();
                                double time = pow(gp_.maxVel_, 2) / gp_.maxAcc_ > dist ? 
                                sqrt(dist / gp_.maxAcc_)
                                 : (dist - pow(gp_.maxVel_, 2) / gp_.maxAcc_) / gp_.maxVel_ + 2 * gp_.maxVel_ / gp_.maxAcc_;


                                gl_traj = GuidanceLaw::guidePNTraj(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);

                                double t;
                                bool flag_too_far;
                                ts *=1.5;
                                do{
                                        ts /= 1.5;
                                        point_set.clear();
                                        flag_too_far = false;
                                        Eigen::Vector3d  last_pt = gl_traj.evaluate(0);
                                        for(t = 0; t < time; t+=ts){
                                                Eigen::Vector3d pt = gl_traj.evaluate(t);
                                                double points_dist = (last_pt - pt).norm();
                                                if(points_dist > gp_.ctrl_pt_dist * 1.5){
                                                        flag_too_far = true;
                                                        break;
                                                }
                                                last_pt = pt;
                                                point_set.push_back(pt);
                                        }
                                }while(flag_too_far);

                                t -= ts;
                                start_end_derivatives.push_back(gl_traj.evaluateVel(0));
                                start_end_derivatives.push_back(local_target_vel);
                                start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
                                start_end_derivatives.push_back(gl_traj.evaluateAcc(t));

                        
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




                                start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
                                start_end_derivatives.push_back(local_target_vel);
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

                vector<Eigen::Vector3d>  DTraj, TTraj;
                //guide_law_->calcGuideTraj(guide_law_->Drone, guide_law_->Target, guide_law_->droneTraj_, guide_law_->targetTraj_);
                if (temp_guider.calcPNGuideTraj(drone, target, DTraj, TTraj)){
                        
                        guide_law_->setInterceptedPoint(DTraj.back());

                        temp_guider.simplifyToSevenPoints(DTraj);
                        temp_guider.Eigen2Poly(DTraj);
                        auto time_now = ros::Time::now();
                        global_data_.setGlobalTraj(temp_guider.dronePolyTraj, time_now);

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