from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node, SetParameter, SetRemap
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction



def generate_launch_description():

    slam_competition_node = Node(
        package='slam_competition',
        executable='slam_bag_processor',
        name='slam_competition_node',
        output='screen',
        parameters=[
           
            {'bag_uri': LaunchConfiguration('bag_uri')},
        ],
        
    )

    bag_uri_arg = DeclareLaunchArgument(
        'bag_uri',
        default_value='src/slam-competition/assets/mapping',
        description='Path to the ROS2 bag file for SLAM competition'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        prefix= 'gnome-terminal --',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='src/slam-competition/assets/fpv_config.rviz',
        description='Path to the RViz config file for SLAM competition'
    )

    slam_competition_node_delay_launch = TimerAction(
		period=10.0,  # delay in seconds
		actions=[slam_competition_node]
	)

    return LaunchDescription([
        bag_uri_arg,
        rviz_config_arg,
        rviz_node,
        slam_competition_node_delay_launch,
    ])