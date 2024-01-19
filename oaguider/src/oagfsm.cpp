#include <oaguider/oagfsm.h>

namespace oaguider{


        //    function 0:
        void OagFSM::init(ros::NodeHandle &nh){

                exec_state_ = FSM_STATE::INIT;
                have_target_ = false; 
                have_odom_ = false;
                use_wpts_ = false;

                nh.param("fsm/realworld_experimemt", flag_realworld_experiment_,false);
                nh.param("fsm/guide_horizon", guide_horizon_, 6.5);
                nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, 1.0);
                nh.param("fsm/thresh_replan_time", replan_thresh_, 1.0);
                nh.param("fsm/fail_safe", enable_fail_safe_, true);

                obstacle_num_ = 0;
                intercept_pt_ = 9999*Eigen::Vector3d::Ones();

                have_trigger_ = !flag_realworld_experiment_;

                visualization_.reset(new OAGVisualization(nh , obstacle_num_));
                guider_manager_.reset(new OAGManager);
                guider_manager_->initGuiderModules(nh, visualization_);
                guider_manager_->deliverTrajToOptimizer();

                /*callback*/
                exec_timer_ = nh.createTimer(ros::Duration(0.01), &OagFSM::execFsmCbk, this);
                odom_sub_ = nh.subscribe("/odom_world", 1, &OagFSM::odomCbk, this);
                waypoint_sub_ = nh.subscribe("/target2_pred", 1, &OagFSM::targetCallback, this);

                bspline_pub_ = nh.advertise<drone_trajs::Bspline>("/guider/bspline", 10);
                data_disp_pub_ = nh.advertise<drone_trajs::DataDisp>("/guider/data_display", 100);
                destroyed_pub_ = nh.advertise<std_msgs::Bool>("/destroy_state", 10);
                
                ROS_INFO("Wait for 1 second.");
                int count = 0;
                old_intercept_pt_ << 9999.0, 9999.0, 9999.0 ;

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

        //    function 1:
        void OagFSM::targetCallback(const nav_msgs::OdometryConstPtr &msg){
                ROS_INFO("targetCallback().");
                if (msg->pose.pose.position.z < -0.1)
                        return;

                init_pt_ = odom_pos_;
                end_pt_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
                end_vel_ = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);

                planNextWaypoint();
        }

        //    function 2:
        void OagFSM::odomCbk(const nav_msgs::OdometryConstPtr &msg){
                //ROS_INFO("odomCbk().");
                odom_pos_(0) = msg->pose.pose.position.x;
                odom_pos_(1) = msg->pose.pose.position.y;
                odom_pos_(2) = msg->pose.pose.position.z;

                odom_vel_(0) = msg->twist.twist.linear.x;
                odom_vel_(1) = msg->twist.twist.linear.y;
                odom_vel_(2) = msg->twist.twist.linear.z;

                odom_orient_.w() = msg->pose.pose.orientation.w;
                odom_orient_.x() = msg->pose.pose.orientation.x;
                odom_orient_.y() = msg->pose.pose.orientation.y;
                odom_orient_.z() = msg->pose.pose.orientation.z;

                have_odom_ = true;
        }


        //   function 3:
        void OagFSM::planNextWaypoint(){
                bool success = false;
                Eigen::Vector3d start_pos, start_vel, start_acc, end_acc;
                start_pos = odom_pos_;
                start_vel = odom_vel_;


                intercept_pt_ = calculateInterceptPoint(start_pos, start_vel, end_pt_, end_vel_);
                ROS_WARN("position: %f,%f, %f, || %f, %f, %f",start_pos[0], start_pos[1], start_pos[2], end_pt_[0], end_pt_[1], end_pt_[2]);
                ROS_WARN("velocity: %f,%f, %f, || %f, %f, %f",start_vel[0], start_vel[1], start_vel[2], end_vel_[0], end_vel_[1], end_vel_[2]);

                //ROS_INFO("end_pt_: %f, %f, %f", end_pt_[0],end_pt_(1), end_pt_(2));
                //ROS_INFO("end_vel_: %f, %f, %f", end_vel_(0),end_vel_(1), end_vel_(2));
                //ROS_WARN("intercept_pt_: %f, %f, %f", intercept_pt_(0),intercept_pt_(1), intercept_pt_(2));

                ROS_WARN("intercept_pt_: %f,%f, %f, || %f, %f, %f",intercept_pt_[0], intercept_pt_[1], intercept_pt_[2], old_intercept_pt_[0], old_intercept_pt_[1], old_intercept_pt_[2]);
                if((start_pos - intercept_pt_).norm()<1.0){
                        changeFSMExecState(WAIT_TARGET, "TRIG");
                        destroy_state.data = true; // 初始化为true 
                        destroyed_pub_.publish(destroy_state);
                        
                }
                
                if( (intercept_pt_ - old_intercept_pt_).norm()<1.5 )// if the intercept point not changed;
                {
                        cout<<"Tunnel1:"<<endl;
                        success = true;
                }
                else{
                        cout<<"Tunnel2:"<<endl;

                        start_acc = Eigen::Vector3d::Zero();

                        end_acc = Eigen::Vector3d::Zero();

                        success = guider_manager_->guideGlobalTraj(start_pos, start_vel, start_acc, end_pt_, end_vel_, end_acc);

                        //ROS_INFO("Guide trajectory %s", success ? "true" : "false");
                }

                if(success){
                        old_intercept_pt_ = intercept_pt_;
                }

                if(success){
                        ROS_WARN("Generating global trajectory!");
                        constexpr double step_size_t = 0.1;
                        int i_end = floor(guider_manager_->global_data_.global_duration_ / step_size_t);
                        //cout<<"i_end:"<<i_end<<endl;
                        vector<Eigen::Vector3d> global_traj(i_end);
                        for (int i = 0; i < i_end; i++){
                                global_traj[i] = guider_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
                                //cout<<"global_traj:"<<global_traj[i]<<endl;
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

                                changeFSMExecState(REGUIDE, "TRIG");
                        }
                        visualization_->displayDroneTraj(global_traj, 0.1, 0);
                }else{
                        changeFSMExecState(WAIT_TARGET, "TRIG");
                        //ROS_ERROR("Unable to generate global trajectory!");
                }
        }

        //   function 4:
        void OagFSM::execFsmCbk(const ros::TimerEvent &e){

                //printFSMExecState();
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
                                //cout<<"have_target_:"<<have_target_<<" have_trigger_:"<<have_trigger_<<endl;
                                if (!have_target_ || !have_trigger_)
                                        goto force_return;
                                else {
                                        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
                                }
                                break;
                        }
                        case GEN_NEW_TRAJ:{
                                start_pt_ = odom_pos_;
                                start_vel_ = odom_vel_;
                                start_acc_.setZero();
                                Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
                                start_yaw_(0) = atan2(rot_x(1), rot_x(0));
                                start_yaw_(1) = start_yaw_(2) = 0.0;

                                //ROS_WARN("GEN_NEW_TRAJ");
                                bool success = GuideFromGlobalTraj(10);

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

                                //ROS_WARN("EXEC_TRAJ");

                                Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

                                //ROS_WARN("pos : %f,%f,%f",pos(0),pos(1),pos(2));

                                if( (local_target_pt_ - intercept_pt_).norm() < 1e-3  ){          // close to the global target        
                                        if (t_cur > info->duration_ - 1e-2){
                                                have_target_ = false;
                                                have_trigger_ = false;

                                                changeFSMExecState(WAIT_TARGET, "FSM");
                                                goto force_return;
                                                // return;
                                                }
                                                else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
                                                {
                                                        ROS_WARN("TUNNEL 1!");
                                                        changeFSMExecState(REGUIDE, "FSM");
                                                }
                                }else if( t_cur > replan_thresh_ ){     
                                        ROS_WARN("TUNNEL 2! %f",replan_thresh_);

                                        changeFSMExecState(REGUIDE, "FSM");
                                }

                                break;
                        }
                        case REGUIDE:{
                                if (GuideFromCurrentTraj(1))
                                {
                                        changeFSMExecState(EXEC_TRAJ, "FSM");
                                }
                                else
                                {
                                         ROS_WARN("TUNNEL 4! %f",replan_thresh_);
                                        changeFSMExecState(REGUIDE, "FSM");
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

        //   function 6:
        bool OagFSM::GuideFromGlobalTraj(const int trial_times){
                ROS_WARN("GuideFromGlobalTraj!");

                start_pt_ = odom_pos_;
                start_vel_ = odom_vel_;
                start_acc_.setZero();

                for (int i = 0; i < trial_times; i++){
                        //ROS_WARN("GuideFromGlobalTraj");
                        if (callReboundReguide()){
                                //ROS_WARN("True");
                                return true;
                        }
                }
                ROS_WARN("False");
                return false;
        }

        //   function 7:
        bool OagFSM::GuideFromCurrentTraj(const int trial_times){
                ROS_WARN("GuideFromCurrentTraj!");

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

        //   function 8:
        bool OagFSM::callReboundReguide(){

                //ROS_WARN("callReboundReguide()");

                getLocalTarget();

                bool plan_and_refine_success =  guider_manager_->reboundReguide(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_);

        //bool reboundReguide(Eigen::Vector3d start_pt, 
        //Eigen::Vector3d start_vel, Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,Eigen::Vector3d local_target_vel){

                auto info = &guider_manager_->local_data_;
                //ROS_INFO("local duration %f",guider_manager_->local_data_.duration_);
                drone_trajs::Bspline bspline;
                /* 1. publish traj to traj_server */
                bspline.order = 3;
                bspline.start_time = info->start_time_;
                bspline.traj_id = info->traj_id_;
                //ROS_WARN("info->traj_id_:%d", info->traj_id_);

                Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
                bspline.pos_pts.reserve(pos_pts.cols());
                for (int i = 0; i < pos_pts.cols(); ++i)
                {
                        geometry_msgs::Point pt;
                        pt.x = pos_pts(0, i);
                        pt.y = pos_pts(1, i);
                        pt.z = pos_pts(2, i);
                        bspline.pos_pts.push_back(pt);
                        //cout<<"******* i:"<<i<<" ******"<<endl;
                }

                Eigen::VectorXd knots = info->position_traj_.getKnot();
                // cout << knots.transpose() << endl;
                bspline.knots.reserve(knots.rows());
                for (int i = 0; i < knots.rows(); ++i)
                {
                        bspline.knots.push_back(knots(i));
                }
        
                //ROS_WARN("bspline size %d", bspline.pos_pts.size());
                bspline_pub_.publish(bspline);
                /* 2. publish traj for visualization */
                visualization_->displayMatrixXdTraj(info->position_traj_.get_control_points(), 0);
                return true;
        }

        //   function 9:
        void OagFSM::getLocalTarget(){
                ROS_WARN("getLocalTarget!");
                double t = 0.0;
                double t_step = guide_horizon_ /20 /guider_manager_ -> gp_.maxVel_;
                //cout<<"guide_horizon_:"<<guide_horizon_<<endl;
                //cout<<"t_step:"<<t_step<<endl;
                 //cout<<"guider_manager_->gp_.maxVel_ :"<<guider_manager_->gp_.maxVel_ <<endl;

                double dist_min = 9999, dist_min_t = 0.0;

                for(t = guider_manager_->global_data_.last_progress_time_; t < guider_manager_->global_data_.global_duration_; t += t_step){
                        Eigen::Vector3d pos_t = guider_manager_->global_data_.getPosition(t);
                        //cout<<"start_pt_:"<<start_pt_<<"    pos_t:"<<pos_t<<endl;
                        double dist = (pos_t - start_pt_).norm();
                        //cout<<"t:"<<t<<endl;
                        if(dist < dist_min){
                                dist_min = dist;
                                dist_min_t = t;
                        }

                        if(dist >= guide_horizon_){
                                local_target_pt_ = pos_t;
                                guider_manager_->local_esti_duration_ = t;
                                guider_manager_  -> global_data_.last_progress_time_ = dist_min_t;
                                ROS_WARN("local_target_pt_1: %f, %f, %f", local_target_pt_(0),local_target_pt_(1),local_target_pt_(2));

                                break;
                        }
                }

                ROS_WARN("t > guider_manager_->global_data_.global_duration_: %f",guider_manager_->global_data_.global_duration_);
                if(t > guider_manager_->global_data_.global_duration_){
                        ROS_WARN("t > guider_manager_->global_data_.global_duration_");
                        local_target_pt_ = intercept_pt_;
                        ROS_WARN("local_target_pt_2: %f, %f, %f", local_target_pt_(0),local_target_pt_(1),local_target_pt_(2));
                        guider_manager_->local_esti_duration_ = t;

                        guider_manager_ -> global_data_.last_progress_time_ = guider_manager_ -> global_data_.global_duration_;
                }

                //cout<<"guider_manager_->gp_.maxVel_ :"<<guider_manager_->gp_.maxVel_ <<"   "<<"guider_manager_->gp_.maxAcc_:"<<guider_manager_->gp_.maxAcc_<<endl;
                if((intercept_pt_ - local_target_pt_).norm() < (guider_manager_->gp_.maxVel_ * guider_manager_ -> gp_.maxVel_) / (2*guider_manager_->gp_.maxAcc_) ){
                        //ROS_WARN("getLocalTarget____1");
                        local_target_vel_ = Eigen::Vector3d::Zero();
                        ROS_WARN("local_target_vel_1: %f, %f, %f", local_target_vel_(0),local_target_vel_(1),local_target_vel_(2));
                }else {
                        //ROS_WARN("getLocalTarget____2");
                        local_target_vel_ = guider_manager_ -> global_data_.getVelocity(t);

                        ROS_WARN("local_target_vel_2: %f, %f, %f", local_target_vel_(0),local_target_vel_(1),local_target_vel_(2));
                }

                ROS_WARN("local_target_pt_3: %f, %f, %f", local_target_pt_(0),local_target_pt_(1),local_target_pt_(2));
        }



        // // function 10:
        Eigen::Vector3d OagFSM::calculateInterceptPoint(Eigen::Vector3d startPt, Eigen::Vector3d startVel, Eigen::Vector3d targetCurrPt, Eigen::Vector3d targetCurVel){
                States current_drone(startPt,startVel,Eigen::Vector3d::Zero());
                States current_target(targetCurrPt, targetCurVel, Eigen::Vector3d::Zero());


                //cout<<"startPt:"<<startPt<<endl;
                //cout<<"targetCurrPt:"<<targetCurrPt<<endl;


                std::vector<Eigen::Vector3d>  d_traj,t_traj;
                Eigen::Vector3d InterceptPt;
                
                 //通过回调函数获取
                vehicle temp_drone("temp_drone", current_drone);
                vehicle temp_target("Targat", current_target);
                
                GuidanceLaw temp_guider(temp_drone, temp_target);

                temp_guider.calcPNGuideTraj(temp_drone, temp_target, d_traj, t_traj);

                InterceptPt = temp_drone.veh_states.pos;
                return InterceptPt;

        }

        // // function 11:
        bool OagFSM::callOaguiderTRAJ(const int trial_times){

                return true;
        }

        // // function 12:
        void OagFSM::checkCollisionCbk(const ros::TimerEvent &e){

        }

        // // function 13:
        void OagFSM::changeFSMExecState(FSM_STATE new_state, string pos_call){

                if (new_state == exec_state_)
                continously_called_times_++;
                else
                continously_called_times_ = 1;

                static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REGUIDE", "EXEC_TRAJ", "EMERGENCY_STOP"};
                int pre_s = int(exec_state_);
                exec_state_ = new_state;
                cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
        }

        // // function 14:
        bool OagFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
        {
                return true;
        }
        
        // // function 15:
        void OagFSM::printFSMExecState()
        {
                static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REGUIDE", "EXEC_TRAJ", "EMERGENCY_STOP"};

                cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
        }


}