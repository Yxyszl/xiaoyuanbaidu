#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "yhs_can_msgs/ctrl_cmd.h"
#include"cmath"

ros::Publisher pub_cmd_ctrl;

  void cmdVelCallback(const geometry_msgs::Twist  & msg) {
      ROS_INFO("Received cmd_vel:\nlinear: x=%.2f, y=%.2f, z=%.2f\nangular: x=%.2f, y=%.2f, z=%.2f",
           msg.linear.x, msg.linear.y, msg.linear.z,
           msg.angular.x, msg.angular.y, msg.angular.z);

       yhs_can_msgs::ctrl_cmd   ctrl_cmd;
      ctrl_cmd.ctrl_cmd_gear=4;
      ctrl_cmd.ctrl_cmd_steering=msg.angular.z*180/M_PI;
      ctrl_cmd.ctrl_cmd_velocity=msg.linear.x;
      pub_cmd_ctrl.publish(ctrl_cmd);
    
  }
int main(int argc, char **argv) {
 ros::init(argc, argv, "cmd_vel_to_cmd_ctrl");
  ros::NodeHandle nh_;
  ros::Subscriber sub_cmd_vel_;
  sub_cmd_vel_ = nh_.subscribe("/cmd_vel", 1000, cmdVelCallback);
  pub_cmd_ctrl= nh_.advertise<yhs_can_msgs::ctrl_cmd>("/ctrl_cmd", 1);

   ros::Rate loop_rate(50);
   while (ros::ok())
      { 
                ros::spin();
                loop_rate.sleep();
      }

  return 0;
}