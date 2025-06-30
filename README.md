# lc_2025_radar_station

lc2025雷达站mid70激光雷达方案

### 项目编译

```bash
cd 2025_radar_station
colcon build
```

### mid70激光雷达网络配置

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

**激光雷达启动**

```bash
source install/setup.bash 
ros2 launch livox_ros2_driver livox_lidar_rviz_launch.py #启动带rviz2可视化
ros2 launch livox_ros2_driver livox_lidar_launch.py #不带rviz2可视化
```

### mindvision相机启动与标定

```bash
ros2 launch mindvision_camera mv_launch.py
ros2 run camera_calibration cameracalibrator --size 8x11 --square 0.02000 image:=/image_raw
```





### 开发日志

#### 6.29

完成雷达站驱动的构建，包括相机驱动，雷达驱动

创建了雷达站主程序包

- [x] 完成订阅雷达点云图并进行雷达点云转换到相机坐标系下的逻辑部分，具体变换矩阵还需要进行标定
- [x] 完成订阅相机话题数据，并且将雷达点云进行投影

具体效果图

![截图 2025-06-30 11-49-18](doc/lidar_rviz.png)

对于数据同步部分，由于相机帧率非常高，所以采用抽帧法来实现软同步，当雷达点云帧累积2帧时，抽取一帧相机帧（抽取帧数可调）

#### 6.30

整理了一下昨天写的代码

重新写了一下投影逻辑部分（昨天写的有点乱，逻辑不变，将定义分到头文件去）

**重写整理部分：**

```c++
for(size_t i = 0; i < point_cloud_->points.size(); i++){
            const auto& p = point_cloud_->points[i];
            intensity_values.push_back(p.intensity);
            //livox雷达坐标系与opencv坐标系定义不同，转换livox坐标系为opencv坐标系
            Eigen::Vector3f lidar_point(-p.y, -p.z, p.x);
            Eigen::Vector3f lidar_point_in_camera = R * lidar_point + T;
            cv::Point3f point_lidar_in_camera(lidar_point_in_camera.x(), lidar_point_in_camera.y(), lidar_point_in_camera.z());
            lidar_points.push_back(point_lidar_in_camera * 100);
        }
        cv::projectPoints(lidar_points, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), cameraMatrix, distCoeffs, lidar_points_projection);
        for(size_t i = 0; i < lidar_points_projection.size(); i++){
            cv::Scalar color;
            int index = static_cast<int>(255.0 * intensity_values[i] / 255);
            if(index > 0 && index < 255){
                color = colormap_[index];
            }else{
                color = cv::Scalar(0, 0, 0);
            }
            cv::circle(frame_, lidar_points_projection[i], 1, color, -1);
        }
```

新增了雷达点云反射率的色彩映射表，根据雷达点云反射率上色

**融合效果**

![截图 2025-06-30 12-53-54](doc/image1.png)

将debug图像由opencv的imshow转为用ros2的话题发布

买了块巨大的标定板

<img src="/home/spaaaaace/Code/mid70/2025_radar_station/doc/76.jpg" alt="76" style="zoom:25%;" />

用来标定雷达和相机之间的外参，标定用matlab的雷达外参标定算法

修改了一下代码，添加了保存点云和图像的代码，导出图像和对应的点云，用来标定(修改参数save_cloud_and_image_为1即可保存点云数据)

**添加保存部分代码**

```c++
if(save_cloud_and_image_ == 1){
    std::string cloud_filename = file_path + "/" + std::to_string(save_count) + ".pcd";
	std::string image_filename = file_path + "/" + std::to_string(save_count) + ".jpg";
    //用pcl库保存点云为.pcd格式
    pcl::io::savePCDFileBinary(cloud_filename, *point_cloud_);
    cv::imwrite(image_filename, frame_);
    save_count++;
}
```

尝试通过matlab标定相机和雷达的外参，但是在代码里面还有点问题

```c++
R << -0.0173, 0.0053, 0.9998,
     -0.0098, -0.0044, -0.0173,
     0.0043, -1.0000, 0.0054;
Eigen::Vector3f T(0.1098, 0.0189, -0.5067);

```





###### git

```bash
echo "# 2025_radar_station" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin git@github.com:Spaaaace-yyj/2025_radar_station.git
git push -u origin main
```