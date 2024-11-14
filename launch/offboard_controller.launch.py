import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import SetUseSimTime
def generate_launch_description():
    offboard_controller = Node(
        package='offboard_controller',
        executable='offboard_controller',
        name='offboard_controller',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', '/home/luca/.rviz2/px4_offboard.rviz'
        ]
        # condition=IfCondition(LaunchConfiguration('rviz'))
    )

    remappings = [('/camera', '/camera/image'),
                  ('/camera_info', '/camera/camera_info'),
                  ('/model/Apriltag36_11_00000/pose', '/tf'),
                  ('/model/x500_mono_cam_1/pose', '/tf')]
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/model/Apriltag36_11_00000/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/model/x500_mono_cam_1/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
            # '/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            # '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            # '/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            # '/rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            # '/model/sphere/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            # '/model/x500_mono_cam_1/pose@geometry_msgs/msg/TransformStamped@gz.msgs.Pose',
            # '/model/ground_plane/pose@geometry_msgs/msg/TransformStamped@gz.msgs.Pose'
        ],
        output='screen',
        remappings=remappings,
        parameters=[""]
    )
    # --ros-args -p config_file:='+os.path.join(pkg_trr_bringup, 'config', 'gz_bridge.yaml')
    # print(os.path.join(get_package_share_directory('offboard_controller'), 'cfg', 'bridge.yaml'))
    # remappings = [('/apriltag/image_rect', '/image_rect'),
    #               ('/camera_info', '/camera/camera_info')]
    # config = os.path.join(
    #     get_package_share_directory('apriltag_ros'),
    #     'cfg',
    #     'px4_tags.yaml'
    #     )
    # apriltag = Node(
    #     package='apriltag_ros',
    #     executable='apriltag_node',
    #     name='apriltag',
    #     namespace='apriltag',
    #     remappings=remappings,
    #     parameters=[config]
    # )
    ground_truth = Node (
        package='ground_truth',
        executable='ground_truth',
        name='ground_truth'
    )
    image_proc = Node (
            package='image_proc',
            executable='rectify_node',
            name='rectify_node',
            remappings=[
                ('image', '/camera/image'),
                ('camera_info', '/camera/camera_info'),
            ]
    )
    # gz_sim = 
    return LaunchDescription([
        offboard_controller,
        rviz,
        bridge,
        # apriltag,
        ground_truth,
        image_proc
    ])
