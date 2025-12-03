# SLAM Competition

This package will introduce you to the SLAM competition environment. 

## Building

To build this package, make sure you have a ROS 2 workspace set up. Then, clone the repository into the `src` directory of your workspace and build it using `colcon`:

```bash
cd ~/your_ros2_workspace/src
git clone https://github.com/your_username/slam-competition.git
cd ~/your_ros2_workspace
colcon build
```

```bash 
source /opt/ros/humble/setup.bash

colcon build --packages-select slam_competition --symlink-install
```

```bash
source install/setup.bash
```

## Bag
Store the bag in the folder: `~/your_ros2_workspace/src/slam-competition/assets/`

## Running the Code and visualizing the results
To run everythoing use the following command:

```bash
ros2 launch slam_competition slam_competition.launch.py
```

### Maintainer

Tommaso Faraci
Universita' di Trento
mail: tommaso.faraci@unitn.it
