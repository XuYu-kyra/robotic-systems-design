from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    阶段 B：近距离启用“全颜色 + block/bin 区分”，用于抓取彩色积木并放入对应彩色 bin。

    - 2D 检测：使用包含 red/blue/yellow/white 的 HSV（tools/color_ranges.yaml）
    - 3D 投影：同一套深度 + 尺度过滤，输出 block:<color> / bin:<color>
    - 机械臂/任务节点：订阅 /color_blobs_3d，在 block:<color> 中选抓取目标，在 bin:<color> 中选投放目标。
    """

    # 这些默认话题名与 realsense2_camera rs_launch.py (常见 namespace: /camera/camera/*) 对齐
    detector = Node(
        package="color_blob_vision",
        executable="color_blob_detector",
        name="color_blob_detector_full",
        parameters=[
            {
                "yaml_path": "/home/student24/robotproject/tools/color_ranges.yaml",
                "image_topic": "/camera/camera/color/image_raw",
                "output_topic": "/color_blobs",
            }
        ],
    )

    depth_to_3d = Node(
        package="color_blob_vision",
        executable="blob_depth_to_3d",
        name="blob_depth_to_3d_full",
        parameters=[
            {
                "camera_info_topic": "/camera/camera/color/camera_info",
                "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
                "blobs_2d_topic": "/color_blobs",
                "output_topic": "/color_blobs_3d",
                # 彩色积木只能在较近距离可靠识别，这里保守设到 0.60m
                "depth_min_m": 0.05,
                "depth_max_m": 0.60,
                "block_size_max_m": 0.10,
                "bin_size_min_m": 0.15,
                "bin_size_max_m": 0.35,
            }
        ],
    )

    # 可选：调试节点（3D marker / 文本输出）
    markers = Node(
        package="color_blob_vision",
        executable="color_blob_markers",
        name="color_blob_markers",
        condition=IfCondition(LaunchConfiguration("use_markers")),
        parameters=[
            {
                "input_topic": "/color_blobs_3d",
                "output_topic": "/color_blobs_markers",
            }
        ],
    )

    summary = Node(
        package="color_blob_vision",
        executable="color_blob_summary",
        name="color_blob_summary",
        condition=IfCondition(LaunchConfiguration("use_summary")),
        parameters=[
            {
                "input_topic": "/color_blobs_3d",
                "print_interval": 1.0,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_markers",
                default_value="false",
                description="Whether to launch RViz MarkerArray publisher (color_blob_markers).",
            ),
            DeclareLaunchArgument(
                "use_summary",
                default_value="true",
                description="Whether to launch terminal summary printer (color_blob_summary).",
            ),
            detector,
            depth_to_3d,
            markers,
            summary,
            # 你的抓取/任务节点可以在自己的包里另写 launch，
            # 或者在这里一起起，只要订阅 /color_blobs_3d 即可。
        ]
    )


