# 校园无人摆渡车系统  
  
本系统面向拥有一定linux C++ ros基础 的成员  
  
##  （1） 离线建图与采集位置信息  
1. 运行雷达驱动功能包 gps驱动包 imu驱动包 获得点云数据 车辆经纬度 IMU_data  
   **驱动包具体操作流程在文件夹 yhs_说明**  
     
2. 遥控车辆底盘绕固定场景运动同时使用rosbag录制所有传感器数据  
     
3. 播放录制的的数据包同时运行 sc_lego_loam\src\SC-LeGO-LOAM\SC-LeGO-LOAM\LeGO-LOAM\launch\run.launch  
**订阅雷达点云数据实现点云pcd地图构建**  
   pcd点云数据保存在sc_lego_loam\src\SC-LeGO-LOAM\SC-LeGO-LOAM\LeGO-LOAM\map  
     
4. 运行imu_gps_localization\src\imu_gps_localization  
   \imu_gps_localization-master\ros_wrapper\launch\imu_gps_localization.launch  
   车辆在定位的经纬度数据最后记录在\imu_gps_localization\src\imu_gps_localization\imu_gps_localization-master\gps.csv
  
##  （2） 定位  
1.运行雷达驱动功能包 gps驱动包 获得点云数据 车辆经纬度  
  **驱动包具体操作流程在文件夹 yhs_说明**  
  
2.加载pcd地图 ，结合1.4中的经纬度运行功能包 initialpose 得知车辆出发点的坐标 实现定位初始配准 再运行ndt_localizer\launch\ndt_localizer.launch 实现点云配准定位  
**出发点配准好需要关闭功能包 initialpose**  

3.运行\lqr_track\launch\odom_listen_from_tf.launch 实现话题odom发布 为后续move_base导航框架 提供定位  
  
 ##  （3） 导航  
1.实现三维pcd地图转换为二维栅格地图 运行pcd2pgm_package/pcd2pgm/launch/中的run.launch 并且通过rosrun map_server map_saver 实现栅格地图的保存  
  
2. 运行底盘驱动功能包 **驱动包具体操作流程在文件夹 yhs_说明**
   同时运行功能包yhs\src\yhs_can_control\cmd_vel_change 将马上要订阅的move_base速度信息 转换为 无人车所需的信息  
     
3.通过mapserver 加载move_base所需的地图 配合使用定位 运行navigation_ws\src\navigation-melodic-devel\move_base\launch\navigation.launch  
通过rviz确定导航目标点也可通过自定义导航点功能包  实现定点导航
  
 ##  （4） 问题与展望  
 1.系统只是初步搭建了导航的框架 使用move_base的全局导航 **Astar算法**  使用的TEB局部路径规划算法由于参数与技术原因使得无人车无法实时避障  
   
 2.系统使用的sc-lego-loam算法 在回环检测部分存在比较大的不足，会导致在运行建图的时候 遇到旧的道路 会有偏差  
   
 3.系统在定位模块仅使用了ndt-matching 可以使用IMU GPS 信息进行融合 达到更高的定位精度  
   
 4.系统尚未使用视觉模块 后期可为系统添加可行区域检测功能 实现更好的路径规划  
   
 5.同时为了更好地实现校园摆渡车的应用场景 可以在用户手机中增加与无人车的5g,蓝牙交互模块，实现更好载客效果  
   
 6.在进行校园无人摆渡 点云建图可能不够实现大规模的导航需求 可选择高精地图  
   
 ## 如需交流 请邮箱szl000228@163.com 期待更好的无人校园摆渡车产品

 
