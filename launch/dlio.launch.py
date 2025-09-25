#
#   Copyright (c)     
#
#   The Verifiable & Control-Theoretic Robotics (VECTR) Lab
#   University of California, Los Angeles
#
#   Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez
#   Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition   
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml
import math

def quaternion_to_rotation_matrix(q):
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
    current_pkg = FindPackageShare('direct_lidar_inertial_odometry')
    base2imu_yaml_path = ''
    base2livox_yaml_path = ''
    
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
    
    # dlio_odom_node에 전달할 파라미터 딕셔너리 동적 생성
    dlio_params = [
        PathJoinSubstitution([current_pkg, 'cfg', 'dlio.yaml']),
        PathJoinSubstitution([current_pkg, 'cfg', 'params.yaml']),
        {
            # YAML에서 읽고 변환한 값으로 extrinsics 파라미터를 덮어씁니다.
            'extrinsics/baselink2imu/t': [imu_t['x'], imu_t['y'], imu_t['z']],
            'extrinsics/baselink2imu/R': imu_r_matrix,
            'extrinsics/baselink2lidar/t': [livox_t['x'], livox_t['y'], livox_t['z']],
            'extrinsics/baselink2lidar/R': livox_r_matrix,
        }
    ]

    # Set default arguments
    rviz = LaunchConfiguration('rviz', default='false')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='/livox/lidar_front')
    imu_topic = LaunchConfiguration('imu_topic', default='/livox/imu_front')

    # Define arguments
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value=rviz,
        description='Launch RViz'
    )
    declare_pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value=pointcloud_topic,
        description='Pointcloud topic name'
    )
    declare_imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value=imu_topic,
        description='IMU topic name'
    )

    # Load parameters
    dlio_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'dlio.yaml'])
    dlio_params_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'params.yaml'])

    # DLIO Odometry Node
    dlio_odom_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_odom_node',
        output='screen',
        parameters=dlio_params,
        remappings=[
            ('pointcloud', pointcloud_topic),
            ('imu', imu_topic),
            ('odom', 'dlio/odom_node/odom'),
            ('pose', 'dlio/odom_node/pose'),
            ('path', 'dlio/odom_node/path'),
            ('kf_pose', 'dlio/odom_node/keyframes'),
            ('kf_cloud', 'dlio/odom_node/pointcloud/keyframe'),
            ('deskewed', 'dlio/odom_node/pointcloud/deskewed'),
        ],
    )

    # DLIO Mapping Node
    dlio_map_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_map_node',
        output='screen',
        parameters=dlio_params,
        remappings=[
            ('keyframes', 'dlio/odom_node/pointcloud/keyframe'),
        ],
    )

    # RViz node
    rviz_config_path = PathJoinSubstitution([current_pkg, 'launch', 'dlio.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='dlio_rviz',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        declare_rviz_arg,
        declare_pointcloud_topic_arg,
        declare_imu_topic_arg,
        dlio_odom_node,
        dlio_map_node,
        rviz_node
    ])
