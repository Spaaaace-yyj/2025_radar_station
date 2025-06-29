# lc_2025_radar_station

### 项目编译

```bash
cd 2025_radar_station
colcon build
```

### mid70激光雷达网络配置

修改2025_radar_station/src/livox_ros2_driver/livox_ros2_driver/config/livox_lidar_config.json文件中的广播码

**mid70 IP地址 ：192.168.1.112**

**SN：3GGDJ8H0010012**

**广播码：3GGDJ8H00100121**

```bash
sudo ip addr flush dev enx6c1ff71615c0
sudo ip addr add 192.168.1.50/24 dev enx6c1ff71615c0
sudo ip link set enx6c1ff71615c0 up
```

配置电脑ip网段与min70在同一网端下192.168.1.xxx

### mindvision相机启动与标定

```bash
ros2 launch mindvision_camera mv_launch.py
ros2 run camera_calibration cameracalibrator --size 8x11 --square 0.02000 image:=/image_raw
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