# SLAM Competition

This package will introduce you to the SLAM competition environment. 

## Download and Build

Star the repository and clone it to your ROS 2 workspace.

To build this package, make sure you have a ROS 2 workspace set up. Then, clone the repository into the `src` directory of your workspace and build it using `colcon`:

```bash
cd ~/limo_ros2_workspace/src
git clone https://github.com/idra-lab/slam-competition.git
cd ~/limo_ros2_workspace
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
