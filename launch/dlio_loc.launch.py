from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
import yaml
import math
import os
import json

################### user configure parameters for Livox ROS 2 ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 1    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

livox_config_dir = os.path.join(
    os.getenv('WORKSPACE_DIR'),
    'config',
    'livox'
    )

user_config_path = os.path.join(livox_config_dir, 'MID360_config.json')

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]
################### user configure parameters for Livox ROS 2 ###################

def quaternion_to_rotation_matrix(q):
    """Convert a quaternion into a rotation matrix.

    Args:
        q (dict): A dictionary containing the quaternion components (x, y, z, w).

    Returns:
        list: A 3x3 rotation matrix represented as a flat list of 9 elements.
    """
    x, y, z, w = q['x'], q['y'], q['z'], q['w']
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    return [
        1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy),
        2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
        2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)
    ]

def generate_launch_description():
    dlio_loc_pkg_dir = FindPackageShare('direct_lidar_inertial_odometry')
    
    # Define workspace and config directories
    ws_dir = os.getenv('WORKSPACE_DIR')
    if not ws_dir:
        raise EnvironmentError("WORKSPACE_DIR environment variable is not set.")
    tf_dir = os.path.join(ws_dir, 'config', 'tf')
    
    # Launch configuration variables
    rviz = LaunchConfiguration('rviz', default='false')
    map_path = LaunchConfiguration('map_path', default=os.path.join(ws_dir, 'data', 'map', '250904_lap_around_buildings.ply')) # Set your map path here as a default
    points_topic = LaunchConfiguration('points_topic', default='/livox/lidar_front')
    imu_topic = LaunchConfiguration('imu_topic', default='/livox/imu_front')
    boot_lidar = LaunchConfiguration("boot_lidar", default="true")
    loc_mode = LaunchConfiguration("loc_mode", default="true")
    
    # Auto initial pose parameters
    use_auto_init_pose = LaunchConfiguration("use_auto_init_pose", default="false")
    init_x = LaunchConfiguration("init_x", default="0.0")
    init_y = LaunchConfiguration("init_y", default="0.0")
    init_z = LaunchConfiguration("init_z", default="0.0")
    init_qx = LaunchConfiguration("init_qx", default="0.0")
    init_qy = LaunchConfiguration("init_qy", default="0.0")
    init_qz = LaunchConfiguration("init_qz", default="0.0")
    init_qw = LaunchConfiguration("init_qw", default="1.0")

    # Declare launch arguments
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value=rviz,
        description='Launch RViz'
    )
    declare_map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=map_path,
        description='Path to the map file (.pcd or .ply)'
    )
    declare_points_topic_arg = DeclareLaunchArgument(
        'points_topic',
        default_value=points_topic,
        description='Pointcloud topic name'
    )
    declare_imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value=imu_topic,
        description='IMU topic name'
    )
    boot_lidar_arg = DeclareLaunchArgument(
        "boot_lidar",
        default_value="true",
        description="On/Off lidar driver"
    )
    loc_mode_arg = DeclareLaunchArgument(
        "loc_mode",
        default_value="true",
        description="true: localization mode, false: pure lio mode"
    )
    
    # Auto initial pose arguments
    use_auto_init_pose_arg = DeclareLaunchArgument(
        "use_auto_init_pose",
        default_value="false",
        description="Enable automatic initial pose setting from launch parameters"
    )
    init_x_arg = DeclareLaunchArgument("init_x", default_value="0.0", description="Initial position X")
    init_y_arg = DeclareLaunchArgument("init_y", default_value="0.0", description="Initial position Y")
    init_z_arg = DeclareLaunchArgument("init_z", default_value="0.0", description="Initial position Z")
    init_qx_arg = DeclareLaunchArgument("init_qx", default_value="0.0", description="Initial orientation X")
    init_qy_arg = DeclareLaunchArgument("init_qy", default_value="0.0", description="Initial orientation Y")
    init_qz_arg = DeclareLaunchArgument("init_qz", default_value="0.0", description="Initial orientation Z")
    init_qw_arg = DeclareLaunchArgument("init_qw", default_value="1.0", description="Initial orientation W")

    # Load extrinsic parameters from YAML files
    base2imu_yaml_path = os.path.join(tf_dir, 'base_to_imu.yaml')
    base2livox_yaml_path = os.path.join(tf_dir, 'base_to_livox.yaml')
    with open(base2imu_yaml_path, 'r') as f:
        base2imu_data = yaml.safe_load(f)
    with open(base2livox_yaml_path, 'r') as f:
        base2livox_data = yaml.safe_load(f)
        
    base_to_imu_params = base2imu_data['static_tf_publisher_base_to_imu']['ros__parameters']
    imu_t = base_to_imu_params['translation']
    imu_q = base_to_imu_params['rotation']
    imu_r_matrix = quaternion_to_rotation_matrix(imu_q)
    
    base_to_livox_params = base2livox_data['static_tf_publisher_base_to_livox']['ros__parameters']
    livox_t = base_to_livox_params['translation']
    livox_q = base_to_livox_params['rotation']
    livox_r_matrix = quaternion_to_rotation_matrix(livox_q)
    
    dlio_params = [
        PathJoinSubstitution([dlio_loc_pkg_dir, 'cfg', 'dlio.yaml']),
        PathJoinSubstitution([dlio_loc_pkg_dir, 'cfg', 'params.yaml']),
        {

            'extrinsics/baselink2imu/t': [imu_t['x'], imu_t['y'], imu_t['z']],
            'extrinsics/baselink2imu/R': imu_r_matrix,
            'extrinsics/baselink2lidar/t': [livox_t['x'], livox_t['y'], livox_t['z']],
            'extrinsics/baselink2lidar/R': livox_r_matrix,
            'localization_mode': loc_mode,
            'map_path': map_path,
            # Auto initial pose parameters
            'auto_initial_pose/enabled': use_auto_init_pose,
            'auto_initial_pose/position/x': init_x,
            'auto_initial_pose/position/y': init_y,
            'auto_initial_pose/position/z': init_z,
            'auto_initial_pose/orientation/x': init_qx,
            'auto_initial_pose/orientation/y': init_qy,
            'auto_initial_pose/orientation/z': init_qz,
            'auto_initial_pose/orientation/w': init_qw,
        }
    ]
    
    # Generate remappings based on livox_config_json     
    with open(user_config_path, 'r') as f:
        livox_config_json = json.load(f)
       
    livox_remappings = []
    lidar_configs = livox_config_json.get("lidar_configs", [])
    lidar_name = ['front', 'back']
    
    for idx, lidar in enumerate(lidar_configs):
        ip = lidar.get("ip")
        ip_string = ip.replace('.', '_')
        
        src_lidar = f"/livox/lidar_{ip_string}"
        src_imu = f"/livox/imu_{ip_string}"
        tgt_lidar = f"/livox/lidar_{lidar_name[idx]}"
        tgt_imu = f"/livox/imu_{lidar_name[idx]}"
        livox_remappings.append((src_lidar, tgt_lidar))
        livox_remappings.append((src_imu, tgt_imu))
        
    # Livox ROS2 Driver Node
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
        remappings=livox_remappings,
        condition=IfCondition(boot_lidar),
        )
    
    static_tf_map_to_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # DLIO Odometry Node for Localization
    dlio_odom_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_odom_node',
        output='screen',
        parameters=dlio_params,
        remappings=[
            ('pointcloud', points_topic),
            ('imu', imu_topic),
            ('odom', 'dlio/odom_node/odom'),
            ('pose', 'dlio/odom_node/pose'),
            ('path', 'dlio/odom_node/path'),
            ('map', 'dlio/odom_node/map'), # Remap map topic if needed
            ('kf_pose', 'dlio/odom_node/keyframes'),
            ('kf_cloud', 'dlio/odom_node/pointcloud/keyframe'),
            ('deskewed', 'dlio/odom_node/pointcloud/deskewed'),
        ],
    )

    # RViz node
    rviz_config_path = PathJoinSubstitution([dlio_loc_pkg_dir, 'launch', 'dlio.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='dlio_rviz',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        boot_lidar_arg,
        loc_mode_arg,
        declare_rviz_arg,
        declare_map_path_arg,
        declare_points_topic_arg,
        declare_imu_topic_arg,
        use_auto_init_pose_arg,
        init_x_arg,
        init_y_arg,
        init_z_arg,
        init_qx_arg,
        init_qy_arg,
        init_qz_arg,
        init_qw_arg,
        static_tf_map_to_odom_node,
        rviz_node,
        livox_driver,
        dlio_odom_node
    ])
