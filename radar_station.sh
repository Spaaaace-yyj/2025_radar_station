source install/setup.bash
export LD_LIBRARY_PATH=/home/spaaaaace/Code/mid70/2025_radar_station/env/opencv/install/lib:$LD_LIBRARY_PATH
ros2 launch radar_station radar_station.launch.py 