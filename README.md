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

