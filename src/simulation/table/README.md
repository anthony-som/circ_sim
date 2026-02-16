```
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select table
source install/setup.bash
ros2 launch table world.launch.py
```
