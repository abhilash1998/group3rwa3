# GROUP3_RWA3
GROUP3 RWA3 Package.

# Installation

Assuming your catkin workspace is `ariac_ws`
```bash
cd ~/ariac_ws/src
git clone https://github.com/Sameep2808/group3_rwa3.git
cd ..
rosdep install --from-paths ./src --ignore-packages-from-source -y
catkin build
source ~/ariac_ws/devel/setup.bash
```

# Run

- To run the code

```bash
roslaunch group3_rwa3 ariac.launch
rosrun group3_rwa3 kitting_movegroup_node
```

