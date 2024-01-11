#include "oag_visualization.h"



OAGVisualization::OAGVisualization(ros::NodeHandle &nh, int obs_num){
        node=nh;
        Drone_traj_pub = nh.advertise<visualization_msgs::Marker>("drone_traj", 2);
        Target_traj_pub = nh.advertise<visualization_msgs::Marker>("target_traj",2);
        MatrixXd_traj_pub = nh.advertise<visualization_msgs::Marker>("optimal_list", 2);
        for (int i=0;i<obs_num;i++){
                std::string obsStr = "obstacle_trajs";
                std::string numstr = std::to_string(i);
                obsStr += numstr;
                Obstacles_traj_pub.push_back(nh.advertise<visualization_msgs::Marker>(obsStr, 2));
        }
        //Obstacle_traj_pubs = nh.advertise<visualization_msgs::Marker>("obstacle_trajs",2);
        Acc_arrows_pub = nh.advertise<visualization_msgs::MarkerArray>("acc_arrows",2);
        Vel_arrows_pub = nh.advertise<visualization_msgs::MarkerArray>("vel_arrows",2);
        IntcPoint_pub = nh.advertise<visualization_msgs::Marker>("intc_point",2);
}

void OAGVisualization::displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id,  bool show_sphere/*true*/){

        visualization_msgs::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "map";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = color(0);
        sphere.color.g = line_strip.color.g = color(1);
        sphere.color.b = line_strip.color.b = color(2);
        sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
        sphere.scale.x = scale;
        sphere.scale.y = scale;
        sphere.scale.z = scale;
        line_strip.scale.x = scale / 2;
        geometry_msgs::Point pt;
        for (int i = 0; i < int(list.size()); i++)
        {
                pt.x = list[i](0);
                pt.y = list[i](1);
                pt.z = list[i](2);
                sphere.points.push_back(pt);
                line_strip.points.push_back(pt);
        }
        pub.publish(sphere);
        pub.publish(line_strip);

}


void OAGVisualization::displayDroneTraj(vector<Eigen::Vector3d> init_pts, const double scale, int id){
        if(Drone_traj_pub.getNumSubscribers() == 0){
                return;
        }
        Eigen::Vector4d color(1, 1.0, 0.1, 0);
        displayMarkerList(Drone_traj_pub, init_pts,scale, color, id);
}

  void OAGVisualization::displayMatrixXdTraj(Eigen::MatrixXd optimal_pts, int id)
  {

    if (MatrixXd_traj_pub.getNumSubscribers() == 0)
    {
      return;
    }

    vector<Eigen::Vector3d> list;
    for (int i = 0; i < optimal_pts.cols(); i++)
    {
      Eigen::Vector3d pt = optimal_pts.col(i).transpose();
      list.push_back(pt);
    }
    Eigen::Vector4d color(0.5, 1, 0, 0);
    displayMarkerList(MatrixXd_traj_pub, list, 0.15, color, id);
  }


void OAGVisualization::displayTargetTraj(vector<Eigen::Vector3d> init_pts, const double scale, int id){
        if(Target_traj_pub.getNumSubscribers() == 0){
                return;
        }
        Eigen::Vector4d color(0, 1.0, 0.1, 1);
        displayMarkerList(Target_traj_pub, init_pts,scale, color, id);
}

void OAGVisualization::displayObstacleTraj(int obs_num,vector<Eigen::Vector3d> init_pts, const double scale, int id){
        if(Obstacles_traj_pub[obs_num].getNumSubscribers() == 0){
                return;
        }
        Eigen::Vector4d color(0.84, 0.5, 0.1, 1);
        displayMarkerList(Obstacles_traj_pub[obs_num], init_pts,scale, color, id);
}

void OAGVisualization::displayAcc(vector<Eigen::Vector3d> pos_pts, vector<Eigen::Vector3d> acc_pts, const double scale, int id){
        if(Acc_arrows_pub.getNumSubscribers() == 0){
                return;
        }
        Eigen::Vector4d color(0.1,1.0,1.0,0.5);
        visualization_msgs::MarkerArray array;
        Acc_arrows_pub.publish(array);
        generateArrowDisplayArray(array, pos_pts, acc_pts, 0.5*scale, color, id);
        Acc_arrows_pub.publish(array);
}

void OAGVisualization::displayVel(vector<Eigen::Vector3d> pos_pts, vector<Eigen::Vector3d> vel_pts, const double scale, int id){
        if(Vel_arrows_pub.getNumSubscribers() == 0){
                return;
        }
        Eigen::Vector4d color(1.0,0.1,1.0,0.6);
        visualization_msgs::MarkerArray array;
        // clear
        Vel_arrows_pub.publish(array);

        generateArrowDisplayArray(array, pos_pts, vel_pts, 0.5*scale, color, id);

        Vel_arrows_pub.publish(array);
}

void OAGVisualization::displayInterceptPoint(Eigen::Vector3d intc_point, Eigen::Vector4d color, const double scale, int id){

        visualization_msgs::Marker sphere;
        sphere.header.frame_id = "map";
        sphere.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.action = visualization_msgs::Marker::ADD;
        sphere.id = id;

        sphere.pose.orientation.w = 1.0;
        sphere.color.r = color(0);
        sphere.color.g = color(1);
        sphere.color.b = color(2);
        sphere.color.a = color(3);
        sphere.scale.x = scale;
        sphere.scale.y = scale;
        sphere.scale.z = scale;
        sphere.pose.position.x = intc_point(0);
        sphere.pose.position.y = intc_point(1);
        sphere.pose.position.z = intc_point(2);

        IntcPoint_pub.publish(sphere);

}

  void OAGVisualization::generateArrowDisplayArray(visualization_msgs::MarkerArray &array,const vector<Eigen::Vector3d> &pos_pts,
                                                        const vector<Eigen::Vector3d> &v_or_a, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.header.stamp = ros::Time::now();
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    // geometry_msgs::Point start, end;
    // arrow.points
    arrow.color.r = color(0);
    arrow.color.g = color(1);
    arrow.color.b = color(2);
    arrow.color.a = color(3);
    arrow.scale.x = scale*0.2;
    arrow.scale.y = scale*1.5;
    arrow.scale.z = scale*2;

    geometry_msgs::Point start, end;
    for (int i = 0; i < int(pos_pts.size() / 2); i++)
    {

      start.x = pos_pts[2 * i](0);
      start.y = pos_pts[2 * i](1);
      start.z = pos_pts[2 * i](2);
      end.x = pos_pts[2 * i ](0) + v_or_a[2 * i](0)*5.0;
      end.y = pos_pts[2 * i ](1) + v_or_a[2 * i](1)*5.0;
      end.z = pos_pts[2 * i ](2) + v_or_a[2 * i](2)*5.0;
      arrow.points.clear();
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      arrow.id = i + id * 1000;

      array.markers.push_back(arrow);
    }
  }
