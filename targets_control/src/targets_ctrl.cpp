#include "drone_trajs/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "drone_trajs/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <Eigen/Core>
using namespace std;

ros::Publisher pos_cmd_pub;

quadrotor_msgs::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

//using drone_trajs::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_(3);
double traj_duration_;
ros::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;

int  ctrlpts_size = 204;

Eigen::Vector3d startPoint(10.0, 0.0, 1.0);
Eigen::Vector3d  moveVector(0.1, 5.0, 0.0);

void bsplineCallback(drone_trajs::BsplineConstPtr msg)
{
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
    cout<<"knots_"<<i<<":"<<knots[i]<<endl;
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);

  pos_traj.setKnot(knots);

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  //UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  //start_time_ = msg->start_time;
    start_time_ = ros::Time::now();
  traj_id_ = msg->traj_id;
  cout<<"msg->traj_id"<<":"<< msg->traj_id<<endl;
  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
}

void bspline_()
{
        // parse pos traj

        Eigen::MatrixXd pos_pts(3, ctrlpts_size-4);

        Eigen::VectorXd knots(ctrlpts_size);
        for (int i = 0; i < ctrlpts_size; ++i)
        {
                knots(i) = -0.9 + i*0.3;
                if(knots(i)>-0.01&&knots(i)<0.01){
                        knots(i) = 0.0;
                }
        }

        for (int i = 0; i < ctrlpts_size - 4; ++i)
        {
                pos_pts(0, i) = startPoint[0] + 4.0*moveVector[0]*cos(0.031415926*i);
                pos_pts(1, i) = startPoint[1] + 4.0*moveVector[1]* sin(0.031415926*i);
                pos_pts(2, i) = startPoint[2] + moveVector[2]*i;
        }


        UniformBspline pos_traj(pos_pts, 3, 0.1);
        pos_traj.setKnot(knots);
                
        // parse yaw traj

        // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
        // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
        //   yaw_pts(i, 0) = msg->yaw_pts[i];
        // }

        //UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

        start_time_ = ros::Time::now();
        traj_id_ = 1 ;
                
        traj_.clear();
        traj_.push_back(pos_traj);
        traj_.push_back(traj_[0].getDerivative());
        traj_.push_back(traj_[1].getDerivative());

                
        traj_duration_ = traj_[0].getTimeSum();

        cout<<"traj_duration_:"<<traj_duration_<<endl;

        receive_traj_ = true;
        
}

void bspline(const ros::TimerEvent &e)
{
        // parse pos traj

        Eigen::MatrixXd pos_pts(3, ctrlpts_size-4);

        Eigen::VectorXd knots(ctrlpts_size);
        for (int i = 0; i < ctrlpts_size; ++i)
        {
                knots(i) = -0.9 + i*0.3;
                if(knots(i)>-0.01&&knots(i)<0.01){
                        knots(i) = 0.0;
                }
        }

        for (int i = 0; i < ctrlpts_size - 4; ++i)
        {
                pos_pts(0, i) = startPoint[0] + 4.0*moveVector[0]*cos(0.031415926*i);
                pos_pts(1, i) = startPoint[1] + 4.0*moveVector[1]* sin(0.031415926*i);
                pos_pts(2, i) = startPoint[2] + moveVector[2]*i;
        }


        UniformBspline pos_traj(pos_pts, 3, 0.1);
        pos_traj.setKnot(knots);
                
        // parse yaw traj

        // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
        // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
        //   yaw_pts(i, 0) = msg->yaw_pts[i];
        // }

        //UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

        start_time_ = ros::Time::now();
        traj_id_ = 1 ;
                
        traj_.clear();
        traj_.push_back(pos_traj);
        traj_.push_back(traj_[0].getDerivative());
        traj_.push_back(traj_[1].getDerivative());

                
        traj_duration_ = traj_[0].getTimeSum();

        cout<<"traj_duration_:"<<traj_duration_<<endl;

        receive_traj_ = true;
  
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void cmdCallback(const ros::TimerEvent &e)
{

        /* no publishing before receive traj_ */
        if (!receive_traj_)
        return;
        
        ros::Time time_now = ros::Time::now();
        double t_cur = (time_now - start_time_).toSec();

        Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
        std::pair<double, double> yaw_yawdot(0, 0);

        static ros::Time time_last = ros::Time::now();
        if (t_cur < traj_duration_ && t_cur >= 0.0)
        {
                pos = traj_[0].evaluateDeBoorT(t_cur);
                vel = traj_[1].evaluateDeBoorT(t_cur);
                acc = traj_[2].evaluateDeBoorT(t_cur);

                /*** calculate yaw ***/
                yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
                /*** calculate yaw ***/

                double tf = min(traj_duration_, t_cur + 2.0);
                pos_f = traj_[0].evaluateDeBoorT(tf);
        }
        else if (t_cur >= traj_duration_)
        {
                /* hover when finish traj_ */
                pos = traj_[0].evaluateDeBoorT(traj_duration_);
                vel.setZero();
                acc.setZero();

                yaw_yawdot.first = last_yaw_;
                yaw_yawdot.second = 0;

                pos_f = pos;
                return;
        }
        else
        {
                cout << "[Traj server]: invalid time." << endl;
        }
        time_last = time_now;

        cmd.header.stamp = time_now;
        cmd.header.frame_id = "world";
        cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
        cmd.trajectory_id = traj_id_;

        cmd.position.x = pos(0);
        cmd.position.y = pos(1);
        cmd.position.z = pos(2);

        cmd.velocity.x = vel(0);
        cmd.velocity.y = vel(1);
        cmd.velocity.z = vel(2);

        cmd.acceleration.x = acc(0);
        cmd.acceleration.y = acc(1);
        cmd.acceleration.z = acc(2);

        cmd.yaw = yaw_yawdot.first;
        cmd.yaw_dot = yaw_yawdot.second;

        last_yaw_ = cmd.yaw;

        pos_cmd_pub.publish(cmd);
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "targets_ctrl");
        // ros::NodeHandle node;
        ros::NodeHandle nh("~");
        double loop_rate_hz = 1.0 / 59.1; // 将周期转换为频率，即每秒循环的次数  
        ros::Rate loop_rate(loop_rate_hz);


        bspline_();
        


        //ros::Subscriber bspline_sub = nh.subscribe("/drone_0_planning/bspline", 10, bsplineCallback);


        ros::Timer bspline_timer = nh.createTimer(ros::Duration(59.1), bspline);

        pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/target2_cmd", 50);

        ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

        /* control parameter */
        cmd.kx[0] = pos_gain[0];
        cmd.kx[1] = pos_gain[1];
        cmd.kx[2] = pos_gain[2];

        cmd.kv[0] = vel_gain[0];
        cmd.kv[1] = vel_gain[1];
        cmd.kv[2] = vel_gain[2];

        time_forward_ = 1.0;
        last_yaw_ = 0.0;
        last_yaw_dot_ = 0.0;

        ros::Duration(1.0).sleep();

        ROS_WARN("[Traj server]: ready.");


        ros::spin();


        return 0;
}