# install

```
pip install xacro

echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

mkdir -p robot_ws/src
cd robot_ws/src
git clone <URL>
cd ..
rosdep update
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy 
colcon build --symlink-install
```

# Simulation
Simulation of X1 UGV robot

Source workspace:
```
source ~/robot_ws/install/setup.bash
```

Launch the simulation:
```
ros2 launch ign_simulation x1_empty.launch.py
```
or:
```
ros2 launch ign_simulation x1_empty.launch.py  ign_args:="cave_world.sdf" 
```


List ROS and ignition topics, not all ignition topics are mapped, only used ones:
```
ros2 topic list
ign topic -l
```
Echo ROS and ignition topics:
```
ros2 topic echo /model/x1_ugv/odometry  

ign topic -e -t /model/x1_ugv/odometry
```

Send ROS topics or ignition topics:
```
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}'

ign topic -t "/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0}, angular: {z: 0.0}"
```

