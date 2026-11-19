from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    rgb_topic = "/table_camera/realsense/color/image_raw"
    rgb_topic_suffix = "/compressed"
    depth_topic = "/table_camera/realsense/aligned_depth_to_color/image_raw"
    camera_info_topic = "/table_camera/realsense/aligned_depth_to_color/camera_info"
    kinova_base_link_frame = "base_link"
    camera_frame = "camera_color_frame"
    return LaunchDescription(
        [
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                name="realsense",
                namespace="table_camera",
                parameters=[
                    {
                        "enable_color": True,
                        "enable_depth": True,
                        "enable_infra1": False,
                        "enable_infra2": False,
                        "rgb_camera.color_profile": "640x480x30",
                        "depth_module.depth_profile": "640x480x30",
                        "align_depth.enable": True,
                        "pointcloud.enable": True,
                        "enable_gyro": False,
                        "enable_accel": False,
                        "base_frame_id": camera_frame,
                    }
                ],
                output="screen",
            ),
            Node(
                package="kinova_lfd_ros",
                executable="latest_rgbd_service",
                name="latest_rgbd_service",
                namespace="table_camera",
                output="screen",
                parameters=[
                    {
                        "rgb_topic": rgb_topic + rgb_topic_suffix,
                        "depth_topic": depth_topic,
                        "camera_info_topic": camera_info_topic,
                        "service_name": "/table_camera/realsense/get_latest_frame",
                        "world_frame": kinova_base_link_frame,
                        "camera_frame": camera_frame,
                    }
                ],
            ),
        ]
    )
