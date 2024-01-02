#include <oaguider/oagfsm.h>

namespace oaguider{


        void OagFSM::odomCbk(const nav_msgs::OdometryConstPtr &msg){
                odom_pos_(0) = msg->pose.pose.position.x;
                odom_pos_(1) = msg->pose.pose.position.y;
                odom_pos_(2) = msg->pose.pose.position.z;

                odom_vel_(0) = msg->twist.twist.linear.x;
                odom_vel_(1) = msg->twist.twist.linear.y;
                odom_vel_(2) = msg->twist.twist.linear.z;

                 //odom_acc_ = estimateAcc( msg );

                odom_orient_.w() = msg->pose.pose.orientation.w;
                odom_orient_.x() = msg->pose.pose.orientation.x;
                odom_orient_.y() = msg->pose.pose.orientation.y;
                odom_orient_.z() = msg->pose.pose.orientation.z;

                have_odom_ = true;
        }

        void OagFSM::init(ros::NodeHandle &nh){
                current_wp_ = 0;
                exec_state_ = FSM_STATE::INIT;
                have_target_ = false; 
                have_odom_ = false;
                use_wpts_ = false;

                nh.param("fsm/flight_type", target_type_, -1);
                nh.param("fsm/emergency_time", emergency_time_, -1.0);
                nh.param("fsm/realworld_experimemt", flag_realworld_experiment_,false);
                nh.param("fsm/fail_safe", enable_fail_safe_, true);
                obstacle_num_ = 0;
                have_trigger_ = !flag_realworld_experiment_;

                nh.param("fsm/waypoint_num", waypoint_num_,-1);
                for(int i = 0; i < waypoint_num_; i++){
                        nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
                        nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
                        nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
                }

                visualization_.reset(new OAGVisualization(nh , obstacle_num_));
                guider_manager_.reset(new OAGManager);
                guider_manager_->initGuiderModules(nh, visualization_);
                guider_manager_->deliverTrajToOptimizer();

                /*callback*/
                exec_timer_ = nh.createTimer(ros::Duration(0.01), &OagFSM::execFsmCbk, this);
                safety_timer_ = nh.createTimer(ros::Duration(0.05), &OagFSM::checkCollisionCbk, this);

                odom_sub_ = nh.subscribe("odom_world", 1, &OagFSM::odomCbk, this);

                bspline_pub_ = nh.advertise<drone_trajs::Bspline>("guider/bspline", 10);
                data_disp_pub_ = nh.advertise<drone_trajs::DataDisp>("guider/data_display", 100);


                waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &OagFSM::targetCallback, this);
                ROS_INFO("Wait for 1 second.");
                int count = 0;



                while (ros::ok() && count++ < 1000)
                {
                        ros::spinOnce();
                        ros::Duration(0.001).sleep();
                }


                while(ros::ok() && (!have_odom_ || !have_trigger_)){
                        ros::spinOnce();
                        ros::Duration(0.001).sleep();
                }
                      
        }

        //0
        void OagFSM::targetCallback(const geometry_msgs::PoseStampedPtr &msg){
                if (msg->pose.position.z < -0.1)
                        return;
                cout << "Triggered!"<<endl;

                init_pt_ = odom_pos_;
                Eigen::Vector3d object_pt(msg->pose.position.x, msg->pose.position.y, 1.0);
                //calculateInterceptPoint(object_pt, intercept_pt_);
                planNextWaypoint(object_pt);
        }

        //1
        void OagFSM::planNextWaypoint(const Eigen::Vector3d next_wp){
                bool success = false;
                Eigen::Vector3d start_pos, start_vel, start_acc, end_pos, end_vel, end_acc;
                if(intercept_pt_ == old_intercept_pt_)// if the intercept point not changed;
                {
                        ;
                }
                else{
                        success = guider_manager_->guideGlobalTraj(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc);
                }

                if(success){
                        end_pt_ = next_wp;
                        constexpr double step_size_t = 0.1;
                        int i_end = floor(guider_manager_->global_data_.global_duration_ / step_size_t);
                        vector<Eigen::Vector3d> global_traj(i_end);
                        for (int i = 0; i < i_end; i++){
                                global_traj[i] = guider_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
                        }
                        end_vel_.setZero();
                        have_target_ = true;
                        have_new_target_ = true;

                        if (exec_state_ == WAIT_TARGET)
                                changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
                        else{
                                while (exec_state_ != EXEC_TRAJ){
                                        ros::spinOnce();
                                        ros::Duration(0.001).sleep();
                                }
                                changeFSMExecState(REPLAN_TRAJ, "TRIG");
                        }
                        visualization_->displayDroneTraj(global_traj, 0.1, 0);
                }else{
                        ROS_ERROR("Unable to generate global trajectory!");
                }
        }

        //2
        void OagFSM::execFsmCbk(const ros::TimerEvent &e){
                exec_timer_.stop(); // To avoid blockage
                switch (exec_state_)
                {
                        case INIT:{
                                if(!have_odom_){
                                        goto force_return;
                                }
                                if(!have_trigger_){
                                        goto force_return;
                                }
                                changeFSMExecState(WAIT_TARGET, "FSM");
                                break;
                        }
                        case WAIT_TARGET:{
                                if (!have_target_ || !have_trigger_)
                                        goto force_return;
                                else {
                                        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
                                }
                                break;
                        }
                        case GEN_NEW_TRAJ:{
                                start_pt_ = odom_pos_;
                                start_pt_ = odom_vel_;
                                start_acc_.setZero();
                                Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
                                start_yaw_(0) = atan2(rot_x(1), rot_x(0));
                                start_yaw_(1) = start_yaw_(2) = 0.0;


                                bool success = callOaguiderTRAJ(10);
                                if (success){
                                        changeFSMExecState(EXEC_TRAJ, "FSM");
                                }else{
                                        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
                                }
                                break;
                        }
                        case EXEC_TRAJ:{
                                /**/
                                LocalTrajData *info = &guider_manager_->local_data_;
                                ros::Time time_now = ros::Time::now();
                                double t_cur = (time_now - info->start_time_).toSec();
                                t_cur = min(info->duration_, t_cur);


                                Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

                                if( (local_target_pt_ - end_pt_).norm() < 1e-3  ){          // close to the global target        
                                        if (t_cur > info->duration_ - 1e-2){
                                                have_target_ = false;
                                                have_trigger_ = false;

                                                changeFSMExecState(WAIT_TARGET, "FSM");
                                                goto force_return;
                                                // return;
                                                }
                                                else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
                                                {
                                                        changeFSMExecState(REPLAN_TRAJ, "FSM");
                                                }
                                }else if( t_cur > replan_thresh_ ){      
                                        changeFSMExecState(REPLAN_TRAJ, "FSM");
                                }

                                break;
                        }
                        case REPLAN_TRAJ:{
                                if (GuideFromCurrentTraj(1))
                                {
                                        changeFSMExecState(EXEC_TRAJ, "FSM");
                                }
                                else
                                {
                                        changeFSMExecState(REPLAN_TRAJ, "FSM");
                                }
                                break;
                        }
                        case EMERGENCY_STOP:{
                                if (flag_escape_emergency_) // Avoiding repeated calls
                                {
                                        callEmergencyStop(odom_pos_);
                                }
                                else
                                {
                                        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
                                        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
                                }

                                flag_escape_emergency_ = false;
                                break;
                        }
                }

                data_disp_.header.stamp = ros::Time::now();
                data_disp_pub_.publish(data_disp_);
                force_return:;
                        exec_timer_.start();
        }

        //3
        bool OagFSM::GuideFromGlobalTraj(const int trial_times){

                start_pt_ = odom_pos_;
                start_pt_ = odom_vel_;
                start_acc_.setZero();

                for (int i = 0; i < trial_times; i++){
                if (callReboundReguide()){
                        return true;
                }
                }
                return false;
        }

        //4
        bool OagFSM::GuideFromCurrentTraj(const int trial_times){
                LocalTrajData *info = &guider_manager_->local_data_;
                ros::Time time_now = ros::Time::now();
                double t_cur = (time_now - info-> start_time_).toSec();

                start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
                start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
                start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

                bool success = callReboundReguide();

                if (!success){
                        success = callReboundReguide();
                        //changeFSMExecState(EXEC_TRAJ, "FSM");
                        if (!success){
                                for (int i = 0; i < trial_times; i++){
                                        success = callReboundReguide();
                                        if (success)
                                                break;
                                }
                                if (!success){
                                        return false;
                                }
                        }
                }

                return true;
        }

        //5
        void OagFSM::getLocalTarget(){

        }

        //6
        bool OagFSM::callReboundReguide(){
                getLocalTarget();
                //bool plan_and_refine_success =  guider_manager_->reboundReguide();
                auto info = &guider_manager_->local_data_;
                drone_trajs::Bspline bspline;
                /* 1. publish traj to traj_server */
                bspline_pub_.publish(bspline);
                /* 2. publish traj for visualization */
                visualization_->displayMatrixXdTraj(info->position_traj_.get_control_points(), 0);
                return true;
        }

        //7
        void OagFSM::calculateInterceptPoint(Eigen::Vector3d &targetCurrPt, Eigen::Vector3d &InterceptPt){
                States current_drone;
                States current_target;
                std::vector<Eigen::Vector3d>  d_traj,t_traj;
                
                 //通过回调函数获取
                vehicle temp_drone("temp_drone", current_drone);
                vehicle temp_target("temp_target", current_target);
                
                GuidanceLaw temp_guider(temp_drone, temp_target);

                temp_guider.calcPNGuideTraj(temp_drone, temp_target, d_traj, t_traj);

                InterceptPt = temp_drone.veh_states.pos;

                //guider_manager_->guide_law_->calcPNGuideTraj();
                //guider_manager_->getInterceptPt(InterceptPt);
        }


        bool OagFSM::callOaguiderTRAJ(const int trial_times){

                return true;
        }


        void OagFSM::checkCollisionCbk(const ros::TimerEvent &e){

        }


        void OagFSM::changeFSMExecState(FSM_STATE new_state, string pos_call){

                if (new_state == exec_state_)
                continously_called_times_++;
                else
                continously_called_times_ = 1;

                static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
                int pre_s = int(exec_state_);
                exec_state_ = new_state;
                cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
        }


        bool OagFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
        {
                return true;
        }


}