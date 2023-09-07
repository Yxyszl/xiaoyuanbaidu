# 校园无人摆渡车系统
##  （1） 离线建图与采集位置信息
1. 运行雷达驱动功能包 gps驱动包 imu驱动包 获得点云数据 车辆经纬度 IMU_data （驱动包具体操作流程在文件夹 yhs_说明）
2. 遥控车辆底盘 绕固定场景
3. 运行 sc_lego_loam\src\SC-LeGO-LOAM\SC-LeGO-LOAM\LeGO-LOAM\launch\run.launch  实现点云pcd地图构建
   pcd点云数据保存在sc_lego_loam\src\SC-LeGO-LOAM\SC-LeGO-LOAM\LeGO-LOAM\map
4. 
