clear
source install/setup.bash
colcon build
ros2 run tb3_yaw_fusion "$1"