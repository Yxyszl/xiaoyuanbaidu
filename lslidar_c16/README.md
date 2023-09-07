# lslidar_c16
#version v3.1.1_210922

## version track
Author: zx
### ver3.0 zx

## Description
The `lslidar_c16` package is a linux ROS driver for lslidar c16.
The package is tested on Ubuntu 16.04/18.04 with ROS kinetic/melodic.

## Compling
This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace. And the normal procedure for compling a catkin package will work.

```
cd your_work_space
catkin_make 
```

## Example Usage

### lslidar_c16_decoder

**Parameters**

`lidar_ip` (`string`, `default: 192.168.1.200`)

By default, the IP address of the device is 192.168.1.200.

`frame_id` (`string`, `default: laser_link`)

The frame ID entry for the sent messages.

**Published Topics**

`lslidar_point_cloud`

Each message corresponds to a lslidar packet sent by the device through the Ethernet.

### lslidar_c16_decoder

**Parameters**

`min_range` (`double`, `0.3`)

`max_range` (`double`, `200.0`)

Points outside this range will be removed.

`frequency` (`frequency`, `10.0`)

Note that the driver does not change the frequency of the sensor. 

`publish_point_cloud` (`bool`, `true`)

If set to true, the decoder will additionally send out a local point cloud consisting of the points in each revolution.

**Published Topics**

`lslidar_sweep` (`lslidar_c16_msgs/LslidarChSweep`)

The message arranges the points within each sweep based on its scan index and azimuth.

`lslidar_pointclou_c16` (`sensor_msgs/PointCloud2`)

This is only published when the `publish_point_cloud` is set to `true` in the launch file.

**Node**

```
roslaunch lslidar_c16_decoder lslidar_c16.launch --screen
```
Note that this launch file launches both the driver and the decoder, which is the only launch file needed to be used.


## FAQ


## Bug Report


##Version changes
/***********2020-01-03****************/
Original version : lslidar_c16_v2.02_190919
Revised version  : lslidar_c16_v2.03_200103
Modify  		 : Add a new calibration decode for the new lslidar c16
Author			 : zx
Date			 : 2020-01-03


/***********2020-01-16****************/
Original version : lslidar_c16_v2.03_200103
Revised version  : lslidar_c16_v2.6.0_200116
Modify  		 : Adds the vertical Angle correction file lslidar_c16_db.yaml for the RoS program code
				   Change the range resolution to 0.25cm according to the v2.6 protocol
Author			 : zx
Date			 : 2020-01-16

/***********2020-04-02****************/
Original version : lslidar_c16_v2.6.0_200116
Revised version  : lslidar_c16_v2.6.1_200402
Modify  		 : 1. 增加了读取设备包并解析垂直角度值的功能，用于替换原有固定的垂直角度值。
		           2. 修改了lslidar_c16.launch文件，用于兼容选择参数和功能
Author			 : zx
Date			 : 2020-04-02

luanch文件说明: 
  <node pkg="lslidar_c16_decoder" type="lslidar_c16_decoder_node" name="lslidar_c16_decoder_node" output="screen">
    <param name="calibration_file" value="$(find lslidar_c16_decoder)/params/lslidar_c16_db.yaml" />
    <param name="min_range" value="0.15"/>
    <param name="max_range" value="150.0"/>
    <param name="cbMethod" value="true"/>		//cbMethod = true表示增加x,y坐标的偏移计算补偿, false则不加
    <param name="print_vert" value="true"/>		//print_vert = true 表示打印设备包角度信息， false表示关闭打印信息
    <param name="config_vert_file" value="false"/>	//config_vert_file = true 表示读取yaml文件中的垂直角度， false则关闭
    <param name="distance_unit" value="0.25"/>		//distance_unit = 0.25表示距离单位为0.25cm, = 1表示距离单位为1cm
    <param name="time_synchronization" value="$(arg time_synchronization)"/>
  </node>

/***********2020-06-06************/
Original version : lslidar_c16_v2.6.1_200402
Revised version  : lslidar_c16_v3.0_200606
Modify  		 : 1. 修改了读取设备包并解析垂直角度值和GPS时间的功能。

​	                      2. 增加了读取设备包并解析3号版版本号，用于兼容V2.0版本雷达。

Author			 : lqm
Date			 : 2020-06-06



/***********2020-06-19************/
Original version :  lslidar_c16_v3.0_200606
Revised version  : lslidar_c16_v3.0.1_200619
Modify  		 : 1. 点云校准距离计算方式更新。

Author			 : lqm
Date			 : 2020-06-19

/***********2020-08-18************/
Original version :  lslidar_c16_v3.0.1_200619
Revised version  : LSLIDAR_C16_V3.0.2_200818_ROSK
Modify  		 : 1. 增加点云中每个点的线号信息。

Author			 : lqm
Date			 : 2020-08-18

/***********2020-08-26************/
Original version :  LSLIDAR_C16_V3.0.2_200818_ROSK
Revised version  : LSLIDAR_C16_V3.0.3_200826_ROSK
Modify  		 : 1. 更新每一帧点云的起始角度固定在0°附近，结束角度在360°。

Author			 : lqm
Date			 : 2020-08-26

/***********2020-09-10************/
Original version : LSLIDAR_C16_V3.0.3_200826_ROSK
Revised version  : LSLIDAR_C16_V3.0.4_200910_ROSK
Modify  	:       
1. 新增兼容垂直角度分辨率为1.33°雷达。
2. 增加LaserScan 消息类型发布。
3. 新增扫描角度裁剪。
4. 更新gps时间解析，秒+1。
       
             
        launch文件说明：
​	            <param name="degree_mode" value="1"/>   <!--1表示垂直角度分辨率为1.33°，2表示垂直角度分辨率为2° -->
 		    <param name="scan_start_angle" value="0.0"/>     <!-- 扫描裁剪起始角-->
    ​                 <param name="scan_end_angle" value="36000.0"/>   <!-- 扫描裁剪结束角，单位0.01°-->
    ​                <param name="scan_num" value="8"/>      <!--LaserScan选择的通道-->
    ​                <param name="publish_scan" value="false"/>   <!--是否发布LaserScan消息类型-->

Author			 : lqm
Date			 : 2020-09-10



/***********2020-12-02************/
Original version : LSLIDAR_C16_V3.0.4_200910_ROSK
Revised version  : LSLIDAR_C16_V3.0.6_201202_ROSK
Modify  	:       

1. 兼容 Ubuntu18.04的ROS melodic。

2. 新增nodelet.launch文件。

3. 去除线号（lines/ring）信息，使用标准点云数据类型。
       
             

       launch文件说明：

   ​	           <arg name="time_synchronization" default="false" />

   <！--默认 ，时间同步为false。如果接GPS模块授时，则需改为 true。-->

Author			 : lqm
Date			 : 2020-12-02

/***********2021-04-08************/
Original version :  LSLIDAR_C16_V3.0.6_201202_ROSK
Revised version  : LSLIDAR_C16_V3.0.8_210408_ROS
Modify  	:       

1. 新增可以单独显示第二次回波点云模式，只针对双回波雷达有效。

2. 增加静态坐标转换示例。          

   ```xml
   lslidar_c16.launch文件说明：
       <arg name="return_mode" default="1" />
       <param name="echo_second" value="false"/>
    <!--node pkg="tf" type="static_transform_publisher" name="laser_link_to_world" args="0 0 0 0 0 0 world laser_link 100" /-->
   ```

   若需要只显示第二次回波点云的模式，参数改为如下：

      <arg name="return_mode" default="2" />
       <param name="echo_second" value="true"/>

   静态坐标转换示例，取消注释。

    <node pkg="tf" type="static_transform_publisher" name="laser_link_to_world" args="0 0 0 0 0 0 world laser_link 100" >

   并根据实际需求 ，调整 0 0 0 0 0 0 这6个参数值（XYZ和xyz轴旋转角度）。

Author			 : lqm
Date			 : 2021-04-08



/***********2021-07-01************/
Original version : LSLIDAR_C16_V3.0.8_210408_ROS
Revised version  : LSLIDAR_C16_V3.0.9_210701_ROS
Modify  	:       

1. 新增启动launch文件时，打开rviz。

2. 添加了service通信控制上下电的功能。

   另开终端，输入以下命令:   

   ```shell
    source devel/setup.bash  
    rosservice call /lslidarcontrol 1  //上电
    rosservice call /lslidarcontrol 0  //下电
   ```

Author			 : lqm
Date			 : 2021-07-01

/***********2021-07-26************/
Original version : LSLIDAR_C16_V3.0.9_210701_ROS
Revised version  : LSLIDAR_C16_V3.1.0_210726_ROS
Modify  	:       

1. 新增点的线号和时间戳。
2. 增加install编译。
3.  修改同一工作空间下编译不同雷达驱动报错问题。

Author			 : lqm
Date			 : 2021-07-26



2021-09-22

原始版本：LSLIDAR_C16_V3.1.0_210726_ROS

修订版本：LSLIDAR_C16_V3.1.1_210922_ROS

更新内容：

1、增加坐标系选择，默认雷达0度角对应x轴

launch 文件说明：

~~~xml
<arg name="time_synchronization" default="false" />  //默认未开启GPS授时
<arg name="return_mode" default="1" />  //默认单回波，双回波雷达改为2
<param name="degree_mode" value="2"/>  //雷达垂直角度分辨率，默认2代表2度分辨率，1代表1.33度分辨率
<param name="distance_unit" value="0.25"/>  //距离分辨率0.25
<param name="pointcloud_topic" value="lslidar_point_cloud"/> //lslidar_point_cloud为点云话题名，可改
<param name="coordinate_opt" value="true"/> // 坐标系，默认雷达0度角对应x轴，false为雷达0度角对应y轴

~~~



