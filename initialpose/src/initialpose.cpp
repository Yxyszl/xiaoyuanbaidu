#include <ros/ros.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>

struct my_pose
{
    double latitude;
    double longitude;
    double altitude;
};
//角度制转弧度制
double rad(double d) 
{
	return d * 3.1415926 / 180.0;
}
//全局变量
static double EARTH_RADIUS = 6378.137;//地球半径
ros::Publisher state_pub_;
geometry_msgs::PoseWithCovarianceStamped initial_pose_msg;
my_pose init_pose;


void gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr)
{
    //初始化
 
        init_pose.latitude =31.760766982166668 ;
        init_pose.longitude = 117.183684816333340;
        init_pose.altitude = 28.992999999999999;
 
    //计算相对位置
        double radLat1 ,radLat2, radLong1,radLong2,delta_lat,delta_long,x,y;
		radLat1 = rad(init_pose.latitude);
        radLong1 = rad(init_pose.longitude);
		radLat2 = rad(gps_msg_ptr->latitude);
		radLong2 = rad(gps_msg_ptr->longitude);
    
        //计算x
        delta_long = 0;
	delta_lat = radLat2 - radLat1;  //(radLat1,radLong1)-(radLat2,radLong1)
	if(delta_lat>0)
        x = 2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat1 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ));
        else
	x=-2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat1 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ));

        x = x*EARTH_RADIUS*1000;

        //计算y
	delta_lat = 0;
        delta_long = radLong2  - radLong1;   //(radLat1,radLong1)-(radLat1,radLong2)
	if(delta_long>0)
	y = 2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat2 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ) );
	else
	y=-2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat2 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ) );
        //double y = 2*asin( sin( delta_lat/2 ) + cos( radLat2 )*cos( radLat2)* sin( delta_long/2 )   );
        y = y*EARTH_RADIUS*1000;

        //计算z
        double z = gps_msg_ptr->altitude - init_pose.altitude;

        //发布initialpose
        initial_pose_msg.header.frame_id = "world";
        initial_pose_msg.header.stamp = ros::Time::now();  

        // geometry_msgs::PoseStamped pose;
        // pose.header = initial_pose_msg.header;

        initial_pose_msg.pose.pose.position.x = x;
        initial_pose_msg.pose.pose.position.y = y;
        // initial_pose_msg.pose.position.z = z;
  initial_pose_msg.pose.pose.orientation.w = 1.0;
       state_pub_.publish(initial_pose_msg); 
    
}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"initialpose");
    ros::NodeHandle n;
    ros::Subscriber pose_sub=n.subscribe("/fix",10,gpsCallback); 
    state_pub_ = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    ros::spin();
    return 0;
}

    // // # 设置坐标系原点的经纬度和高度
    //  double origin_lat = 31.760812809666668;
    //  double origin_lon = 117.183708873166665;
    //  double origin_alt = 28.992999999999999;
// ,31.760766982166668,117.183684816333340,23.559999999999999


