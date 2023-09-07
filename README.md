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
   **订阅车辆经纬度 IMU_data数据 实现iMU与gps 融合定位**  
   车辆在定位的经纬度数据实时记录在
