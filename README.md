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

## Running the SLAM Bag Processor
To run the SLAM Bag Processor node, use the following command:

```bash
ros2 run slam_competition slam_bag_processor --ros-args -p bag_uri:=/path/to/your/bagfile
```

