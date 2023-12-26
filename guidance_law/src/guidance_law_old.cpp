#include "guidance_law/guidance_law.h"
        


        

        GuidanceLaw::GuidanceLaw(){
                ROS_INFO("Hello!");
        }

        GuidanceLaw::GuidanceLaw(ros::NodeHandle n,int obsnum){
                nh = n;
                ROS_INFO("Hello!");
                obs_num = obsnum;
                visualization_.reset(new OAGVisualization(nh,obs_num));

                Eigen::Vector3d T_pos(50.0,50.0,50.0), T_vel(2.0, 1.50, 0.0), T_acc(0.0, 0.0, 0.0);
                Eigen::Vector3d D_pos(0.0, 0.0, 0.0), D_vel(10.0, 1.0, 0.0), D_acc(0.0, 0.0, 0.0);

                States T_init(T_pos, T_vel, T_acc);
                States D_init(D_pos, D_vel, D_acc);
                std::string Dstr = "Drone";
                std::string Tstr = "Targat";
                Drone = vehicle(Dstr, D_init);
                Target = vehicle(Tstr, T_init);

                std::default_random_engine generator;  
                std::uniform_real_distribution<double> distribution(0.0, 50.0);

                for(int i = 0; i < obs_num; i++){
                        double rand_x = distribution(generator);
                        double rand_y = distribution(generator);
                        double rand_z = distribution(generator);
                        double rand_vx = 0.01*distribution(generator);
                        double rand_vy = 0.01*distribution(generator);
                        double rand_vz = 0.01*distribution(generator);
                        Eigen::Vector3d Obs_pos(rand_x,rand_y,rand_z), Obs_vel(rand_vx, rand_vy, rand_vz), Obs_acc(0.0, 0.0, 0.0);
                        States Obs_init(Obs_pos, Obs_vel, Obs_acc);
                        std::string Ostr = "obstacle";
                        std::string numstr = std::to_string(i);
                        Ostr += numstr;
                        vehicle Obs(Ostr, Obs_init);
                        Obstacles.push_back(Obs);
                }
        }

        void GuidanceLaw::init(ros::NodeHandle &nh){
                ROS_INFO("init!");
                Obs_sub = nh.subscribe("obsposes", 10, &GuidanceLaw::obsCbk, this, ros::TransportHints().tcpNoDelay());
                Tar_sub = nh.subscribe("tarpos", 10, &GuidanceLaw::targetCbk, this, ros::TransportHints().tcpNoDelay());
                odom_sub = nh.subscribe("odom", 10, &GuidanceLaw::odomCbk, this, ros::TransportHints().tcpNoDelay());
                imu_sub = nh.subscribe("imu", 10, &GuidanceLaw::imuCbk, this, ros::TransportHints().tcpNoDelay());
                max_vel_ = 0.5;
        }

        void GuidanceLaw::obsCbk(const sensor_msgs::PointCloudPtr &obs_pos){
                Eigen::Vector3d ob(0.0, 0.0, 0.0);
                // for(int i  = 0; obs_pos->points.size(); i++){
                //         ob<<obs_pos->points[i].x,obs_pos->points[i].y,obs_pos->points[i].z;
                //         obs.push_back(ob);
                // }
        }

        void GuidanceLaw::targetCbk(const nav_msgs::OdometryPtr &tar){
                ROS_INFO("tarcbk!--begin");
                double x = 50.0;
                double y = 50.0;
                double z = 20.0;
                double vx = 2.0;
                double vy = 2.0;
                double vz = 1.5;
                Eigen::Vector3d T_pos(x, y, z), T_vel(vx, vy, vz), T_acc(0.0, 0.0, 0.0);
                States T_init(T_pos, T_vel, T_acc);
                std::string Tstr = "Targat";
                Target.veh_states.pos = T_pos;
                Target.veh_states.vel = T_vel;
                Target.veh_states.acc = T_acc;
                //Target = vehicle(Tstr, T_init);
                ROS_INFO("tarcbk!--end");
         }

        void GuidanceLaw::odomCbk(const nav_msgs::OdometryPtr &msg){ 
                ROS_INFO("odomcbk!--begin");
                double _x =0.0;
                double _y =0.0;
                double _z =0.0;
                double vx = 10.0;
                double vy = 2.0;
                double vz =0.5;
                Eigen::Vector3d D_pos(_x, _y, _z), D_vel(vx, vy, vz), D_acc(0.0, 0.0, 0.0);
                States D_init(D_pos, D_vel, D_acc);
                std::string Dstr = "Drone";
                Drone.role = Dstr;
                Drone.veh_states.pos = D_pos;
                Drone.veh_states.vel = D_vel;
                Drone.veh_states.acc = D_acc;
                ROS_INFO("odomcbk!--end");
        }

        void GuidanceLaw::imuCbk(const sensor_msgs::ImuPtr &drone_imu){
                // drone.acc(0) = drone_imu->linear_acceleration.x;
                // drone.acc(1) = drone_imu->linear_acceleration.y;
                // drone.acc(2) = drone_imu->linear_acceleration.z;
                // drone.acc(0) = 0.0;
                // drone.acc(1) = 0.0;
                // drone.acc(2) = 0.0;
        }

        void GuidanceLaw::plan(){
                
                ros::Rate loop_rate(RATE);
                double t(0.0), tf(100.0);
                double distance(0.0);
                Eigen::Vector3d D_temp_, T_temp_, O_temp_, vel_temp_, acc_temp_;

                std::vector<Eigen::Vector3d> droneTraj_;
                std::vector<Eigen::Vector3d> targetTraj_;
                std::vector<Eigen::Vector3d> velTraj_;
                std::vector<Eigen::Vector3d> accTraj_;
                std::vector<std::vector<Eigen::Vector3d>> obstaclesTraj_(obs_num);
                ros::Time begin = ros::Time::now();
                ROS_INFO("PLAN---B");
                while(t<tf){
                        
                        Target.update(t);
                        Drone.update(Target.states().pos, Target.states().vel);
                        for(int i = 0; i < obs_num; i++){
                                Obstacles[i].update(t);
                        }

                        distance = (Drone.states().pos - Target.states().pos).norm();
                        D_temp_(0) = Drone.states().pos(0);
                        D_temp_(1) = Drone.states().pos(1);
                        D_temp_(2) = Drone.states().pos(2);
                        T_temp_(0) = Target.states().pos(0);
                        T_temp_(1) = Target.states().pos(1);
                        T_temp_(2) = Target.states().pos(2);
                        vel_temp_(0) = Drone.states().vel(0);
                        vel_temp_(1) = Drone.states().vel(1);
                        vel_temp_(2) = Drone.states().vel(2);
                        acc_temp_(0) = Drone.states().acc(0);
                        acc_temp_(1) = Drone.states().acc(1);
                        acc_temp_(2) = Drone.states().acc(2);
                        droneTraj_.push_back(D_temp_);
                        targetTraj_.push_back(T_temp_);
                        velTraj_.push_back(vel_temp_);
                        accTraj_.push_back(acc_temp_);
                        for(int i = 0; i < obs_num; i++){
                        O_temp_(0) = Obstacles[i].states().pos(0);
                        O_temp_(1) = Obstacles[i].states().pos(1);
                        O_temp_(2) = Obstacles[i].states().pos(2);
                        obstaclesTraj_[i].push_back(O_temp_);
                }


                        if(distance<0.5){
                                std::cout<<"Intercepted at:"<<t<<std::endl;
                                std::cout<<Target.states().pos<<std::endl;
                                std::cout<<Drone.states().pos<<std::endl;
                                break;
                        }
                        t+=ST;
                        //std::cout<<"Time:"<<t<<std::endl;
                }
                ros::Time end = ros::Time::now();
                std::cout<<"Cost Time:"<<(end-begin).toSec()<<std::endl;
                while(ros::ok()){
                        visualization_->displayDroneTraj(droneTraj_, 1, 0);
                        visualization_->displayTargetTraj(targetTraj_, 1, 0);
                        visualization_->displayInterceptPoint(targetTraj_.back(), Eigen::Vector4d(0.5,0,0.5,1),5.0, 0);
                        for(int i = 0; i < obs_num; i++){
                                visualization_->displayObstacleTraj(i, obstaclesTraj_[i],5,0);
                        }
                        visualization_->displayVel(droneTraj_,velTraj_,1, 0);
                        visualization_->displayAcc(droneTraj_,accTraj_,2, 0);
                        ros::spinOnce(); 
                        //loop_rate.sleep();
                }
        
        
        }

        void GuidanceLaw::visilization(PolynomialTraj traj){ }

        PolynomialTraj GuidanceLaw::topolyTraj(std::vector<Eigen::Vector3d>  path){
                double begin_time = ros::Time::now().toSec();
                Eigen::Vector3d start_vel(0.0, 0.0, 0.0), start_acc(0.0, 0.0, 0.0);
                Eigen::Vector3d end_vel(0.0, 0.0, 0.0), end_acc(0.0, 0.0, 0.0);
                Eigen::MatrixXd pos(3, path.size());

                for (int i = 0; i < path.size(); i++)
                        {pos.col(i) = path[i];}
                std::cout<<"path.size()"<<path.size()<<std::endl;
                Eigen::VectorXd time(path.size() - 1);
                for (int i = 0; i < path.size()/ - 1; i++)
                        {time(i) = (pos.col(i + 1) - pos.col(i)).norm() / max_vel_;}
                time(0) *= 2.0;
                time(time.rows() - 1) *= 2.0;
                double end_time = ros::Time::now().toSec();
                std::cout << "一次查找耗时:" << (end_time - begin_time) << std::endl;
                std::cout<<"******************************"<<std::endl;
                traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);

        }