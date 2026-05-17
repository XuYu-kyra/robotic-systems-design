from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    summary_condition = IfCondition(LaunchConfiguration("use_summary"))
    debug_condition = IfCondition(LaunchConfiguration("use_debug"))
    markers_condition = IfCondition(LaunchConfiguration("use_markers"))
    recorder_condition = IfCondition(LaunchConfiguration("use_recorder"))

    default_mode_arg = DeclareLaunchArgument(
        "default_mode",
        default_value="search_white_bin",
        description="Initial task mode for perception/task manager.",
    )

    use_summary_arg = DeclareLaunchArgument(
        "use_summary",
        default_value="false",
    )

    use_debug_arg = DeclareLaunchArgument(
        "use_debug",
        default_value="false",
        description="Whether to publish overlay debug image topics.",
    )

    use_markers_arg = DeclareLaunchArgument(
        "use_markers",
        default_value="false",
        description="Whether to publish RViz MarkerArray topics.",
    )

    use_recorder_arg = DeclareLaunchArgument(
        "use_recorder",
        default_value="false",
        description="Whether to save debug video and 3D detections to files.",
    )

    record_dir_arg = DeclareLaunchArgument(
        "record_dir",
        default_value="/home/student24/robotproject/ros_nd2/color_blob_runs",
        description="Directory where recorder run folders are written.",
    )

    record_name_arg = DeclareLaunchArgument(
        "record_name",
        default_value="",
        description="Run folder name. Empty uses current timestamp.",
    )

    record_video_fps_arg = DeclareLaunchArgument(
        "record_video_fps",
        default_value="10.0",
        description="FPS used only when recorder writes mp4 directly.",
    )

    record_write_video_arg = DeclareLaunchArgument(
        "record_write_video",
        default_value="false",
        description="Whether recorder writes debug_image.mp4 directly during playback.",
    )

    record_save_frames_arg = DeclareLaunchArgument(
        "record_save_frames",
        default_value="true",
        description="Whether recorder saves every debug frame with timestamps for offline rendering.",
    )

    record_image_ext_arg = DeclareLaunchArgument(
        "record_image_ext",
        default_value="jpg",
        description="Image format for saved debug frames: jpg or png.",
    )

    ee_frame_arg = DeclareLaunchArgument(
        "ee_frame",
        default_value="ee_link",
        description="Parent frame used for the assumed hand-eye mounting transform.",
    )

    camera_frame_arg = DeclareLaunchArgument(
        "camera_frame",
        default_value="camera_link",
        description="Camera root frame that receives the assumed hand-eye mounting transform.",
    )

    target_frame_arg = DeclareLaunchArgument(
        "target_frame",
        default_value="base_link",
        description="Robot frame where the selected target pose is published.",
    )

    camera_mount_x_arg = DeclareLaunchArgument(
        "camera_mount_x",
        default_value="-0.05",
        description="Assumed camera x offset from ee_frame in meters.",
    )

    camera_mount_y_arg = DeclareLaunchArgument(
        "camera_mount_y",
        default_value="0.03",
        description="Assumed camera y offset from ee_frame in meters.",
    )

    camera_mount_z_arg = DeclareLaunchArgument(
        "camera_mount_z",
        default_value="-0.04",
        description="Assumed camera z offset from ee_frame in meters.",
    )

    camera_mount_roll_arg = DeclareLaunchArgument(
        "camera_mount_roll",
        default_value="0.0",
        description="Assumed camera roll offset from ee_frame in radians.",
    )

    camera_mount_pitch_arg = DeclareLaunchArgument(
        "camera_mount_pitch",
        default_value="0.0",
        description="Assumed camera pitch offset from ee_frame in radians.",
    )

    camera_mount_yaw_arg = DeclareLaunchArgument(
        "camera_mount_yaw",
        default_value="0.0",
        description="Assumed camera yaw offset from ee_frame in radians.",
    )

    simulate_nav_arg = DeclareLaunchArgument(
        "simulate_nav",
        default_value="true",
        description="Whether task_manager auto-completes navigation commands.",
    )

    simulate_arm_arg = DeclareLaunchArgument(
        "simulate_arm",
        default_value="true",
        description="Whether task_manager auto-completes arm commands.",
    )

    pipeline_mode_arg = DeclareLaunchArgument(
        "pipeline_mode",
        default_value="offline_bag",
        description="offline_bag or real_robot. offline_bag allows TF fallback; real_robot expects robot TFs.",
    )

    allow_tf_fallback_arg = DeclareLaunchArgument(
        "allow_tf_fallback",
        default_value=PythonExpression(
            [
                "'true' if '",
                LaunchConfiguration("pipeline_mode"),
                "'.lower() == 'offline_bag' else 'false'"
            ]
        ),
        description="Whether perception_manager may keep targets in the source frame when TF to target_frame is unavailable.",
    )

    perception_target_frame_arg = DeclareLaunchArgument(
        "perception_target_frame",
        default_value=PythonExpression(
            [
                "'camera_color_optical_frame' if '",
                LaunchConfiguration("pipeline_mode"),
                "'.lower() == 'offline_bag' else '",
                LaunchConfiguration("target_frame"),
                "'"
            ]
        ),
        description="Frame used by perception_manager for published target poses. Defaults to camera frame for offline bag mode and target_frame for real robot mode.",
    )

    use_spatial_white_bin_dedup_arg = DeclareLaunchArgument(
        "use_spatial_white_bin_dedup",
        default_value=PythonExpression(
            [
                "'false' if '",
                LaunchConfiguration("pipeline_mode"),
                "'.lower() == 'offline_bag' else 'true'",
            ]
        ),
        description="Whether white-bin dedup uses spatial pose distance. Disabled for offline bag mode and enabled for real robot mode.",
    )

    require_reachable_targets_arg = DeclareLaunchArgument(
        "require_reachable_targets",
        default_value=PythonExpression(
            [
                "'false' if '",
                LaunchConfiguration("pipeline_mode"),
                "'.lower() == 'offline_bag' else 'true'",
            ]
        ),
        description="Whether task_manager requires perception_manager to mark targets reachable before accepting them.",
    )

    white_bin_confirm_hits_arg = DeclareLaunchArgument(
        "white_bin_confirm_hits",
        default_value="3",
        description="Consecutive white-bin candidate hits required before locking.",
    )

    block_confirm_hits_arg = DeclareLaunchArgument(
        "block_confirm_hits",
        default_value=PythonExpression(
            [
                "'2' if '",
                LaunchConfiguration("pipeline_mode"),
                "'.lower() == 'offline_bag' else '3'",
            ]
        ),
        description="Consecutive block candidate hits required before locking.",
    )

    home_bin_confirm_hits_arg = DeclareLaunchArgument(
        "home_bin_confirm_hits",
        default_value=PythonExpression(
            [
                "'1' if '",
                LaunchConfiguration("pipeline_mode"),
                "'.lower() == 'offline_bag' else '2'",
            ]
        ),
        description="Consecutive home-bin candidate hits required before switching from go-home navigation to final color-bin search.",
    )

    sim_home_nav_duration_arg = DeclareLaunchArgument(
        "sim_home_nav_duration_sec",
        default_value=PythonExpression(
            [
                "'12.0' if '",
                LaunchConfiguration("pipeline_mode"),
                "'.lower() == 'offline_bag' else '2.0'",
            ]
        ),
        description="Simulated go-home navigation duration budget before timing out into final color-bin search.",
    )

    # ---------------- WHITE PIPELINE ----------------
    white_detector = Node(
        package="color_blob_vision",
        executable="color_blob_detector",
        name="color_blob_detector_white",
        parameters=[
            {
                "yaml_path": "/home/student24/robotproject/tools/color_ranges_white_only.yaml",
                "image_topic": "/camera/camera/color/image_raw",
                "output_topic": "/white/color_blobs",
                "min_area": 6000,
                "kernel_size": 9,
                "resize_factor": 1.0,
                "min_score": 0.005,
            }
        ],
    )

    white_3d = Node(
        package="color_blob_vision",
        executable="blob_depth_to_3d",
        name="blob_depth_to_3d_white",
        parameters=[
            {
                "camera_info_topic": "/camera/camera/color/camera_info",
                "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
                "blobs_2d_topic": "/white/color_blobs",
                "output_topic": "/white/color_blobs_3d",

                "depth_min_m": 0.20,
                "depth_max_m": 1.50,
                "x_min_m": -0.45,
                "x_max_m":  0.45,
                "y_min_m": -10.0,
                "y_max_m":  10.0,

                "block_size_max_m": 0.10,
                "bin_size_min_m": 0.15,
                "bin_size_max_m": 0.35,

                "confirm_hits_bin": 5,
                "max_misses_bin": 1,
                "bin_match_dist_m": 0.07,
                "pos_alpha_bin": 0.18,
                "yaw_alpha_bin": 0.15,
                "score_alpha": 0.20,
                "patch_radius": 5,
            }
        ],
    )

    white_summary = Node(
        package="color_blob_vision",
        executable="color_blob_summary",
        name="color_blob_summary_white",
        parameters=[
            {
                "input_topic": "/white/color_blobs_3d",
                "print_interval": 1.0,
            }
        ],
        condition=summary_condition,
    )

    white_debug = Node(
        package="color_blob_vision",
        executable="color_blob_debug_image",
        name="color_blob_debug_image_white",
        parameters=[
            {
                "image_topic": "/camera/camera/color/image_raw",
                "blobs_topic": "/white/color_blobs",
                "blobs_3d_topic": "/white/color_blobs_3d",
                "camera_info_topic": "/camera/camera/color/camera_info",
                "output_topic": "/white/debug_image",
                "yaml_path": "/home/student24/robotproject/tools/color_ranges_white_only.yaml",
                "debug_fps": 30.0,
                "enable_roi_refine": False,
                "draw_unmatched_2d": True,
            }
        ],
        condition=debug_condition,
    )

    white_markers = Node(
        package="color_blob_vision",
        executable="color_blob_markers",
        name="color_blob_markers_white",
        parameters=[
            {
                "input_topic": "/white/color_blobs_3d",
                "output_topic": "/color_blobs_markers",
            }
        ],
        condition=markers_condition,
    )

    # ---------------- FULL COLOR PIPELINE ----------------
    full_detector = Node(
        package="color_blob_vision",
        executable="color_blob_detector",
        name="color_blob_detector_full",
        parameters=[
            {
                "yaml_path": "/home/student24/robotproject/tools/color_ranges.yaml",
                "image_topic": "/camera/camera/color/image_raw",
                "output_topic": "/full/color_blobs",
                "min_area": 2500,
                "min_score": 0.0025,
                "roi_y_min": 0.35,
                "roi_y_max": 1.0,
                "resize_factor": 1.0,
            }
        ],
    )

    full_3d = Node(
        package="color_blob_vision",
        executable="blob_depth_to_3d",
        name="blob_depth_to_3d_full",
        parameters=[
            {
                "camera_info_topic": "/camera/camera/color/camera_info",
                "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
                "blobs_2d_topic": "/full/color_blobs",
                "output_topic": "/full/color_blobs_3d",

                "depth_min_m": 0.05,
                "depth_max_m": 0.70,
                "x_min_m": -0.45,
                "x_max_m":  0.45,
                "y_min_m": -10.0,
                "y_max_m":  10.0,

                "block_size_max_m": 0.10,
                "bin_size_min_m": 0.15,
                "bin_size_max_m": 0.35,

                "confirm_hits_block": 2,
                "max_misses_block": 2,
                "confirm_hits_bin": 3,
                "max_misses_bin": 2,
                "patch_radius": 2,
            }
        ],
    )

    full_summary = Node(
        package="color_blob_vision",
        executable="color_blob_summary",
        name="color_blob_summary_full",
        parameters=[
            {
                "input_topic": "/full/color_blobs_3d",
                "print_interval": 1.0,
            }
        ],
        condition=summary_condition,
    )

    full_debug = Node(
        package="color_blob_vision",
        executable="color_blob_debug_image",
        name="color_blob_debug_image_full",
        parameters=[
            {
                "image_topic": "/camera/camera/color/image_raw",
                "blobs_topic": "/full/color_blobs",
                "blobs_3d_topic": "/full/color_blobs_3d",
                "camera_info_topic": "/camera/camera/color/camera_info",
                "output_topic": "/full/debug_image",
                "yaml_path": "/home/student24/robotproject/tools/color_ranges.yaml",
                "debug_fps": 30.0,
                "enable_roi_refine": False,
                "draw_unmatched_2d": False,
            }
        ],
        condition=debug_condition,
    )

    full_markers = Node(
        package="color_blob_vision",
        executable="color_blob_markers",
        name="color_blob_markers_full",
        parameters=[
            {
                "input_topic": "/full/color_blobs_3d",
                "output_topic": "/color_blobs_markers",
            }
        ],
        condition=markers_condition,
    )

    camera_mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_mount_static_tf",
        arguments=[
            "--x", LaunchConfiguration("camera_mount_x"),
            "--y", LaunchConfiguration("camera_mount_y"),
            "--z", LaunchConfiguration("camera_mount_z"),
            "--roll", LaunchConfiguration("camera_mount_roll"),
            "--pitch", LaunchConfiguration("camera_mount_pitch"),
            "--yaw", LaunchConfiguration("camera_mount_yaw"),
            "--frame-id", LaunchConfiguration("ee_frame"),
            "--child-frame-id", LaunchConfiguration("camera_frame"),
        ],
    )

    # ---------------- MANAGER ----------------
    manager = Node(
        package="color_blob_vision",
        executable="perception_manager",
        name="perception_manager",
        parameters=[
            {
                "default_mode": LaunchConfiguration("default_mode"),
                "white_topic": "/white/color_blobs_3d",
                "full_topic": "/full/color_blobs_3d",
                "print_interval": 0.5,
                "target_hold_timeout": 1.0,
                "target_frame": LaunchConfiguration("perception_target_frame"),
                "mode_topic": "/task/mode",
                "target_color_hint_topic": "/task/target_color_hint",
                "target_pose_topic": "/task/current_target_pose",
                "target_label_topic": "/task/current_target_label",
                "target_visible_topic": "/task/current_target_visible",
                "target_reachable_topic": "/task/current_target_reachable",
                "target_status_topic": "/task/current_target_status",
                "allow_fallback_to_source_frame": LaunchConfiguration("allow_tf_fallback"),
            }
        ],
    )

    task_manager = Node(
        package="color_blob_vision",
        executable="task_manager",
        name="task_manager",
        parameters=[
            {
                "initial_state": "SEARCH_WHITE_BIN",
                "simulate_nav": LaunchConfiguration("simulate_nav"),
                "simulate_arm": LaunchConfiguration("simulate_arm"),
                "mode_topic": "/task/mode",
                "state_topic": "/task/state",
                "event_topic": "/task/event",
                "nav_goal_topic": "/task/nav_goal_name",
                "arm_command_topic": "/task/arm_command",
                "arm_target_topic": "/task/arm_target_pose",
                "target_label_topic": "/task/current_target_label",
                "target_visible_topic": "/task/current_target_visible",
                "target_reachable_topic": "/task/current_target_reachable",
                "target_pose_topic": "/task/current_target_pose",
                "target_color_hint_topic": "/task/target_color_hint",
                "use_spatial_white_bin_dedup": LaunchConfiguration("use_spatial_white_bin_dedup"),
                "require_reachable_targets": LaunchConfiguration("require_reachable_targets"),
                "white_bin_confirm_hits": LaunchConfiguration("white_bin_confirm_hits"),
                "block_confirm_hits": LaunchConfiguration("block_confirm_hits"),
                "home_bin_confirm_hits": LaunchConfiguration("home_bin_confirm_hits"),
                "sim_home_nav_duration_sec": LaunchConfiguration("sim_home_nav_duration_sec"),
            }
        ],
    )

    recorder = Node(
        package="color_blob_vision",
        executable="color_blob_run_recorder",
        name="color_blob_run_recorder",
        parameters=[
            {
                "image_topic": "/full/debug_image",
                "white_3d_topic": "/white/color_blobs_3d",
                "full_3d_topic": "/full/color_blobs_3d",
                "output_dir": LaunchConfiguration("record_dir"),
                "run_name": LaunchConfiguration("record_name"),
                "video_fps": LaunchConfiguration("record_video_fps"),
                "write_video": LaunchConfiguration("record_write_video"),
                "save_frames": LaunchConfiguration("record_save_frames"),
                "image_ext": LaunchConfiguration("record_image_ext"),
            }
        ],
        condition=recorder_condition,
    )

    return LaunchDescription(
        [
            default_mode_arg,
            use_summary_arg,
            use_debug_arg,
            use_markers_arg,
            use_recorder_arg,
            record_dir_arg,
            record_name_arg,
            record_video_fps_arg,
            record_write_video_arg,
            record_save_frames_arg,
            record_image_ext_arg,
            ee_frame_arg,
            camera_frame_arg,
            target_frame_arg,
            pipeline_mode_arg,
            allow_tf_fallback_arg,
            perception_target_frame_arg,
            use_spatial_white_bin_dedup_arg,
            require_reachable_targets_arg,
            white_bin_confirm_hits_arg,
            block_confirm_hits_arg,
            home_bin_confirm_hits_arg,
            sim_home_nav_duration_arg,
            camera_mount_x_arg,
            camera_mount_y_arg,
            camera_mount_z_arg,
            camera_mount_roll_arg,
            camera_mount_pitch_arg,
            camera_mount_yaw_arg,
            simulate_nav_arg,
            simulate_arm_arg,

            camera_mount_tf,
            white_detector,
            white_3d,
            white_summary,
            white_debug,
            white_markers,

            full_detector,
            full_3d,
            full_summary,
            full_debug,
            full_markers,

            manager,
            task_manager,
            recorder,
        ]
    )
