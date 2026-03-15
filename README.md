# xarm6_control_ros2
To run this package:

First launch the proper Docker image "saeidih/robotics:ros2p1".

In a new terminal, 

```bash
cd ~/dev_ws/src/
git clone https://github.com/hsaeidi-uncw/xarm6_control_ros2.git
cd ..
colcon build --packages-select xarm6_control_ros2 --symlink-install
source install/setup.bash
```
