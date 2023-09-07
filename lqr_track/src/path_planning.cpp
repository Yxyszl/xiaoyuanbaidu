#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <vector>
#include "LQR_track/trajectory.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/time.h"
#include <math.h>
#include <tf/tf.h>

double x_start,y_start,limit_x,limit_y;
string trajectory_type;
nav_msgs::Path produce_path(){
    trajectory* trajec = new trajectory(x_start,y_start,trajectory_type,limit_x,limit_y);//实例化trajectory类；
    vector<waypoint> waypoint_vec;//定义vector类用于接收由trajec生成的路径,得到若干组[ID,x,y]
    nav_msgs::Path waypoints;//定义nav_msgs::Path类用于构造ROS的Path消息

    //获取路径
    trajec->refer_path();
    waypoint_vec = trajec->get_path();

    //构造适用于ROS的Path消息
    waypoints.header.stamp = ros::Time::now();
    waypoints.header.frame_id = "map";
    for(int i =0;i<waypoint_vec.size();i++){
        geometry_msgs::PoseStamped this_pose;
        this_pose.header.seq = i;
        //ROS_INFO("path_id is %d", this_pose.header.seq);
        this_pose.header.frame_id = "map";
        this_pose.pose.position.x = waypoint_vec[i].x;
        this_pose.pose.position.y = waypoint_vec[i].y;
        this_pose.pose.position.z = 0;
        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(waypoint_vec[i].yaw);
        //ROS_INFO("the yaw is %f",waypoint_vec[i].yaw);
        this_pose.pose.orientation.x = goal_quat.x;
        this_pose.pose.orientation.y = goal_quat.y;
        this_pose.pose.orientation.z = goal_quat.z;
        this_pose.pose.orientation.w = goal_quat.w;

        waypoints.poses.push_back(this_pose);
    }
    return waypoints;//返回适用于ROS的Path消息
}


int main(int argc,char **argv){
    ros::init(argc, argv, "path_produce");
    ros::NodeHandle n;
    ros::NodeHandle n_prv("~");
    n_prv.param<double>("x_start",x_start,0.0);
    n_prv.param<double>("y_start",y_start,0.0);
    n_prv.param<string>("trajectory_type",trajectory_type,"line");
    n_prv.param<double>("limit_x",limit_x,10);
    n_prv.param<double>("limit_y",limit_y,0.0);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/path",10);
    ros::Rate loop_rate(1);
    while(ros::ok()){
        nav_msgs::Path path = produce_path();
        ROS_INFO("the trajectory size is: %ld",path.poses.size());
        path_pub.publish(path);
        loop_rate.sleep();
    }
}