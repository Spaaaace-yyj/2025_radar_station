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



## 开发日志

## 6.29

完成雷达站驱动的构建，包括相机驱动，雷达驱动

创建了雷达站主程序包

- [x] 完成订阅雷达点云图并进行雷达点云转换到相机坐标系下的逻辑部分，具体变换矩阵还需要进行标定
- [x] 完成订阅相机话题数据，并且将雷达点云进行投影

具体效果图

![截图 2025-06-30 11-49-18](doc/lidar_rviz.png)

对于数据同步部分，由于相机帧率非常高，所以采用抽帧法来实现软同步，当雷达点云帧累积2帧时，抽取一帧相机帧（抽取帧数可调）

## 6.30

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

<img src="doc/image1.png" alt="截图 2025-06-30 12-53-54" style="zoom:33%;" />

将debug图像由opencv的imshow转为用ros2的话题发布

买了块巨大的标定板（花我95    o.O）

<img src="doc/76.jpg" alt="76" style="zoom: 25%;" />

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

设点云点坐标为P1，旋转矩阵R，平移矩阵T，变换后点P2

代码中的逻辑为 P2 = R * P1 + T

```c++
Eigen::Vector3f lidar_point_in_camera = R_lidar_to_camera * lidar_point + T_lidar_to_camera;
```

**逻辑上应该没有问题但是旋转矩阵左乘后出现大量的点跑到相机坐标系后面了（点的Z值都是负数），投影到图像上的点为零？？？？？**

## 7.1

重新采集了一次点云和图像进行标定

<img src="doc/capture_20250630205635268.bmp" alt="capture_20250630205635268" style="zoom: 33%;" />

最后重新算出来的旋转和平移矩阵

```c++
R = 	-0.0318712393232544   -0.999474459707108     0.00591848774478551
 		0.0111232139656527   -0.00627581384510766  -0.999918440809877
 		0.999430086706956    -0.0318028073252486    0.0113173862335329
    
T = 	[0.0428245893583569, 0.0104154355328730, -0.638969767020652]
```

但是结果还是错的，把矩阵转置后结果也是错的

怀疑是坐标轴的定义问题

livox-mid70激光雷达的坐标系和opencv的坐标轴的定义不一样

```c++
Eigen::Vector3f lidar_point(-p.y, -p.z, p.x);
```

根据定义将转换坐标轴后点云可以投影出来（前提是不进行雷达到相机坐标系转换P2 = R * P1 + T），但是加上坐标系转换，点云就跑飞了

看了下matlab标定出来的旋转矩阵

```c++
R = 	-0.0318712393232544   -0.999474459707108     0.00591848774478551
 		0.0111232139656527   -0.00627581384510766  -0.999918440809877
 		0.999430086706956    -0.0318028073252486    0.0113173862335329
```

四舍五入一下
```c++
R = 	0	-1	0
 		0	0	-1
 		1	0	0
```

发现matlab的标定已经将坐标轴的变换标出来了**~~（难崩）~~**

```c++
Eigen::Vector3f lidar_point(p.x, p.y, p.z);
```

把我之前写的手动变换去掉，加上旋转矩阵，点云基本能对上，但是还是有一点偏差，但是在matlab里面可视化出来基本上对齐的非常准（由于相机的内参是用matlab标定的，但代码里面的使用ros里的包标定的）

导出matlab标定的相机内参

```c++
K = 1635.80929422889	0					709.797419508020
    0					1636.89792281429	533.441903861457
    0					0					1
```

对比了一下用ros标定的相机内参，和ros标定出来的相差非常大！

**修改了内参后，点云基本全部都对齐了!!!!!!!!!!!!!!!!!!!!!!!!!!**

~~**（总算对齐了，cao）**~~

<img src="doc/cap.png" alt="cap" style="zoom: 67%;" />





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