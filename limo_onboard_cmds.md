

ros2 launch limo_bringup limo_start.launch.py

ros2 launch orbbec_camera dabai.launch.py  

ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 launch limo_bringup cartographer.launch.py

<!-- ros2 run rqt_image_view rqt_image_view -->

ros2 launch nav2_bringup navigation_launch.py

ros2 bag record -a -o mapping_0

