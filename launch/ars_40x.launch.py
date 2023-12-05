from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'visualize',
            default_value='false',
            description='Visualize radar data in RViz2'),
        DeclareLaunchArgument(
            'obstacle_array',
            default_value='false',
            description='Publish obstacle array for navigation'),

        Node(
            package='ars_40x',
            executable='ars_40x_ros',
            # namespace='ars_40x',
            # name='ars_40x_ros',
            output='screen',
            emulate_tty=True,
            # parameters=[{'frame_id': 'radar_link'}]
            remappings=[('odom', 'odom')]
        ),

        # Node(
        #     package='ars_40x',
        #     executable='ars_40x_rviz',
        #     name='ars_40x_rviz',
        #     condition=LaunchConfiguration('visualsouize')
        # ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     condition=LaunchConfiguration('visualize')
        #     # arguments=['-d', LaunchConfiguration('find-pkg-share ars_40x') + '/rviz_cfg/ars_40x.rviz']
        # ),

        # Node(
        #     package='ars_40x',
        #     executable='ars_40x_obstacle_array',
        #     name='ars_40x_obstacle_array',
        #     condition=LaunchConfiguration('obstacle_array')
        #     # remappings=[('obstacles', '/move_base/TebLocalPlannerROS/obstacles')]
        # )
    ])