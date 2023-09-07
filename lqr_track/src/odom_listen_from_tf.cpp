#include <ros/ros.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

std::string base_frame;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_listener");

  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener odomListener(tfBuffer);
  nh.param<std::string>("base_frame", base_frame, "odom");
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",10);//修改了一下
  ros::Rate r(20);

  while(ros::ok()){
    geometry_msgs::TransformStamped transform;
    try{
      transform = tfBuffer.lookupTransform("map", base_frame, ros::Time(0),ros::Duration(1.0));
    }
      catch (tf2::TransformException &ex){
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    tf2::Vector3 translation;
    translation.setValue(transform.transform.translation.x,
                         transform.transform.translation.y,
                         transform.transform.translation.z);

    tf2::Quaternion rotation;
    rotation.setValue(transform.transform.rotation.x,
                      transform.transform.rotation.y,
                      transform.transform.rotation.z,
                      transform.transform.rotation.w);

    tf2::Matrix3x3 m(rotation);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    /*ROS_INFO("\n=== Got Transform ===\n"
             " Translation\n"
             " x : %f\n y : %f\n z : %f\n"
             " Quaternion\n"
             " x : %f\n y : %f\n z : %f\n w : %f\n"
             " RPY\n"
             " R : %f\n P : %f\n Y : %f",
             translation.x(), translation.y(), translation.z(),
             rotation.x(), rotation.y(), rotation.z(), rotation.w(),
             roll, pitch, yaw);*/

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = translation.x();
    odom.pose.pose.position.y = translation.y();
    odom.pose.pose.position.z = translation.z();

    odom.pose.pose.orientation.x = rotation.x();
    odom.pose.pose.orientation.y = rotation.y();
    odom.pose.pose.orientation.z = rotation.z();
    odom.pose.pose.orientation.w = rotation.w();

    odom.child_frame_id = base_frame;
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.linear.z = 0;

    odom_pub.publish(odom);
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
