# lc_2025_radar_station

------

LcRM2025雷达站mid70激光雷达站

- ### 项目依赖

  ubuntu22.04

  ros2Humble

  Opencv4(ONNX模块带GPU加速，可选)

  Eigen

  PCL

- ### 项目编译


```bash
cd 2025_radar_station
colcon build
```

- ### mid70激光雷达网络配置


修改<u>2025_radar_station/src/livox_ros2_driver/livox_ros2_driver/config/livox_lidar_config.json</u>文件中的广播码

**mid70 IP地址 ：192.168.1.112**

**SN：3GGDJ8H0010012**

**广播码：3GGDJ8H00100121**

```bash
#配置网卡ip，让网卡ip与雷达在同一网段下
sudo ip addr flush dev enx6c1ff71615c0
sudo ip addr add 192.168.1.50/24 dev enx6c1ff71615c0
sudo ip link set enx6c1ff71615c0 up
```

配置电脑ip网段与min70在同一网端下192.168.1.xxx

- **激光雷达启动**

```bash
source install/setup.bash 
ros2 launch livox_ros2_driver livox_lidar_rviz_launch.py #启动带rviz2可视化
ros2 launch livox_ros2_driver livox_lidar_launch.py #不带rviz2可视化
```

- ### mindvision相机启动与标定


```bash
ros2 launch mindvision_camera mv_launch.py
ros2 run camera_calibration cameracalibrator --size 8x11 --square 0.02000 image:=/image_raw
```

- ### 场地坐标标定


开启radar_world_calib和相机ros包

```bash
source install/setup.bash 
ros2 run radar_world_calib radar_world_calib 
```

左键标定，右键清除所有点，鼠标中键计算旋转和平移矩阵（标定点的数量需要等于在代码里面定义的现实中的点的数量，顺序也要对应）。将输出的旋转矩阵复制到radar_station的config中的参数里（如果标定后动了雷达的位置，就要重新标定）

后续可以尝试使用icp自动标定

- ### 启动雷达站


```bash
source install/setup.bash 
bash radar_station.sh
```

------

tips：项目处于半成品状态，裁判系统通讯还为实现，追踪算法和猜点都还有优化的地方。这只是本人在视觉组学习和探索过程的一个尝试，加上实验室一直没有雷达站兵种，所以才催生了这个项目。本人很菜，目前这个项目就我一个人开发，精力有限，所以代码有点乱，也有很多bug（