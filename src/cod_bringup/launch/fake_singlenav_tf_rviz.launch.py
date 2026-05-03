import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('cod_bringup')
    default_rviz_config = os.path.join(bringup_dir, 'rviz', 'cod_nav.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    use_tf_tree = LaunchConfiguration('use_tf_tree')
    rviz_config = LaunchConfiguration('rviz_config')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2.',
    )

    declare_use_tf_tree = DeclareLaunchArgument(
        'use_tf_tree',
        default_value='false',
        description='Launch rqt_tf_tree if installed.',
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Full path to the RViz config file.',
    )

    # Keep frame names aligned with singlenav_launch.py and singlenav2_params.yaml.
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
        output='screen',
    )

    odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'odom',
            '--child-frame-id', 'base_link',
        ],
        output='screen',
    )

    base_to_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.305',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'livox_frame',
        ],
        output='screen',
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    tf_tree = Node(
        condition=IfCondition(use_tf_tree),
        package='rqt_tf_tree',
        executable='rqt_tf_tree',
        output='screen',
    )

    return LaunchDescription([
        declare_use_rviz,
        declare_use_tf_tree,
        declare_rviz_config,
        map_to_odom,
        odom_to_base,
        base_to_livox,
        rviz,
        tf_tree,
    ])
