import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 获取包的共享目录
    bring_up_dir = get_package_share_directory('cod_bringup')

    rviz_config_file = os.path.join(bring_up_dir, 'rviz', 'cod_nav.rviz')

    fast_lio_config_file = 'mid360.yaml'
    # 声明启动参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')
    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file', default_value=os.path.join(bring_up_dir,'params','mapper_params_async.yaml')
    )
    declare_nav2_params_file = DeclareLaunchArgument(
        'nav2_params_file',default_value=os.path.join(bring_up_dir,'params','singlenav2_params.yaml')
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    # 深度相机启动节点已注释（禁用）
    realsense_actions = []
    # try:
    #     realsense_dir = get_package_share_directory('realsense2_camera')
    #     realsense_actions.append(
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 os.path.join(realsense_dir, 'launch', 'rs_launch.py')
    #             ),
    #             launch_arguments={
    #                 'depth_module.depth_profile': '424x240x90',
    #                 'pointcloud.enable': 'true',
    #                 'pointcloud.ordered_pc': 'false',
    #                 'pointcloud.allow_no_texture_points': 'true',
    #                 'spatial_filter.enable': 'true',
    #                 'temporal_filter.enable': 'true',
    #                 'decimation_filter.enable': 'false',
    #                 'publish_tf': 'true',
    #                 'depth_module.enable_auto_exposure': 'true',
    #             }.items()
    #         )
    #     )
    # except PackageNotFoundError:
    #     print('[cod_bringup] realsense2_camera not found, skip RealSense node.')

    # 定义节点和包含的launch文件
    load_nodes = GroupAction(
        actions=[
            Node(
                package='cpp_lidar_filter',
                executable='lidar_filter_node',
                name='my_lidar_filter',
                output='screen',
                parameters=[{
                    'input_topic': '/livox/lidar',
                    'output_topic': '/livox/lidar_filtered',
                    'min_x': -0.3, 'max_x': 0.3,
                    'min_y': -0.3, 'max_y': 0.3,
                    'min_z': -0.1, 'max_z': 0.2,
                    'negative': True,   # 挖掉车身
                    'leaf_size': 0.05   # 降采样
                }]
            ),
            Node(
                    package="small_point_lio",
                    executable="small_point_lio_node",
                    name="small_point_lio",
                    output="screen",
                    parameters=[
                        PathJoinSubstitution(
                            [
                                FindPackageShare("small_point_lio"),
                                "config",
                                "mid360.yaml",
                            ]
                        ),
                        {"base_frame": "chassis"},
                    ],
            ),

            Node(
                package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                remappings=[('cloud_in',  '/livox/lidar'),
                            ('scan', '/scan')],
                parameters=[{
                    'target_frame': 'chassis',
                    'transform_tolerance': 0.5,
                    'min_height': 0.1,
                    'max_height': 1.00,
                    'angle_min': -3.1416,
                    'angle_max': 3.1416,
                    'angle_increment': 0.0087,
                    'scan_time': 0.3333,
                    'range_min': 0.5,
                    'range_max': 20.0,
                    'use_inf': True,
                    'inf_epsilon': 1.0
                }],
                name='pointcloud_to_laserscan'
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "0.0",
                    "--z",
                    "0.406",
                    "--roll",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--yaw",
                    "0.0",
                    "--frame-id",
                    "chassis",
                    "--child-frame-id",
                    "base_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "0.17",
                    "--z",
                    "0.305",
                    "--roll",
                    "0.0",
                    "--pitch",
                    "0.7854",
                    "--yaw",
                    "1.5708",
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "livox_frame",
                ],
            ),
            Node(
                package="cod_serial_ul26",
                executable="cod_serial",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            # *realsense_actions,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bring_up_dir,'launch','navigation_launch.py')),
                launch_arguments={
                                  'use_sim_time': "false",
                                  'autostart': "true",
                                  'params_file': nav2_params_file,
                                  'use_composition': 'False',
                                  'use_respawn': 'False',
                                  'container_name': 'nav2_container'}.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bring_up_dir,'launch','localization_launch.py')),
                launch_arguments={
                                  'use_sim_time': "false",
                                  'autostart': "true",
                                  'params_file': nav2_params_file,
                                  'use_composition': 'False',
                                  'use_respawn': 'False',
                                  'container_name': 'nav2_container'}.items()
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d',rviz_config_file],
                output='screen',
            ),
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params_file,
        declare_nav2_params_file,
        load_nodes
    ])
