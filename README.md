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
    
2.通过map_server加载pcd地图 ，结合1.4中的经纬度运行功能包 initialpose 得知车辆出发点的坐标 实现定位初始配准 再运行ndt_localizer\launch\ndt_localizer.launch 实现点云配准定位  
**出发点配准好需要关闭功能包 initialpose**  
  
3.运行\lqr_track\launch\odom_listen_from_tf.launch 实现话题odom发布 为后续move_base导航框架 提供定位  

 ##  （3） 导航  
 1.实现pcd转换为栅格地图 [三维pcd地图转二维栅格地图]([http://blog.csdn.net/guodongxiaren](https://blog.csdn.net/valanria_steel/article/details/102903865?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169409080116800182757897%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=169409080116800182757897&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-15-102903865-null-null.142^v93^chatsearchT3_1&utm_term=%E7%A6%BB%E7%BA%BF%E5%B0%86pcd%E5%9C%B0%E5%9B%BE%E8%BD%AC%E4%B8%BA%E6%A0%85%E6%A0%BC%E5%9C%B0%E5%9B%BE%20%E7%9A%84%E5%8A%9F%E8%83%BD%E5%8C%85&spm=1018.2226.3001.4187)https://blog.csdn.net/valanria_steel/article/details/102903865?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169409080116800182757897%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=169409080116800182757897&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-15-102903865-null-null.142^v93^chatsearchT3_1&utm_term=%E7%A6%BB%E7%BA%BF%E5%B0%86pcd%E5%9C%B0%E5%9B%BE%E8%BD%AC%E4%B8%BA%E6%A0%85%E6%A0%BC%E5%9C%B0%E5%9B%BE%20%E7%9A%84%E5%8A%9F%E8%83%BD%E5%8C%85&spm=1018.2226.3001.4187 "链接")

  
