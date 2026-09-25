clear
source install/setup.bash
colcon build --packages-select icp_package
ros2 run icp_package scan_merge
