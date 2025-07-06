source install/setup.bash
export LD_LIBRARY_PATH=/home/spaaaaace/Code/mid70/2025_radar_station/env/opencv/install/lib:$LD_LIBRARY_PATH
ros2 launch radar_station_bringup lc_radar_ststion_startup.launch.py 