"""Configuration for the MoveIt 2 Any6D robot controller (ROS 2 Humble)."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class RobotConfig:
    robot_id: str = "dsr01"
    robot_model: str = "m0609"
    home_joint: tuple[float, ...] = (0.0, 0.0, 90.0, 0.0, 90.0, 180.0)
    joint_vel: float = 30.0
    joint_acc: float = 30.0
    enable_motion: bool = True


@dataclass(frozen=True)
class MoveItConfig:
    """Names and limits that must match the M0609 MoveIt configuration."""

    planning_group: str = "manipulator"
    base_frame: str = "base_link"
    eef_link: str = "rg2_tcp"
    joint_names: tuple[str, ...] = (
        "joint_1", "joint_2", "joint_3",
        "joint_4", "joint_5", "joint_6",
    )
    move_action: str = "/move_action"
    execute_trajectory_action: str = "/execute_trajectory"
    compute_ik_service: str = "/compute_ik"
    compute_cartesian_path_service: str = "/compute_cartesian_path"
    joint_states_topic: str = "/joint_states"
    planning_pipeline_id: str = "ompl"
    planner_id: str = ""
    planning_time_sec: float = 5.0
    planning_attempts: int = 5
    service_wait_timeout_sec: float = 15.0
    action_wait_timeout_sec: float = 15.0
    execution_timeout_sec: float = 60.0
    ik_timeout_sec: float = 1.0
    cartesian_max_step_m: float = 0.005
    cartesian_jump_threshold: float = 0.0
    cartesian_min_fraction: float = 0.99
    position_tolerance_m: float = 0.002
    orientation_tolerance_rad: float = 0.02
    joint_tolerance_rad: float = 0.01
    velocity_scaling: float = 0.25
    acceleration_scaling: float = 0.25
    allow_replanning: bool = True


@dataclass(frozen=True)
class GripperConfig:
    name: str = "rg2"
    toolchanger_ip: str = "192.168.1.1"
    toolchanger_port: int = 502
    force_tenth_newton: int = 200
    yellow_can_target_width_mm: float = 53.0
    timeout_sec: float = 8.0


@dataclass(frozen=True)
class MotionConfig:
    approach_vel: float = 30.0
    approach_acc: float = 60.0
    grasp_vel: float = 15.0
    grasp_acc: float = 30.0
    lift_vel: float = 20.0
    lift_acc: float = 40.0
    # Primary pick moves from the current pose directly to the grasp pose.
    approach_distance_mm: float = 0.0
    lift_distance_mm: float = 150.0
    approach_mode: str = "tool_z"
    tool_insertion_axis: tuple[float, float, float] = (0.0, 0.0, 1.0)
    tool_force_service: str = "/dsr01/aux_control/get_tool_force"
    tool_force_reference: int = 1  # DR_TOOL
    tool_force_service_wait_timeout_sec: float = 2.0
    tool_force_response_timeout_sec: float = 1.0
    force_release_settle_sec: float = 0.5
    force_release_baseline_samples: int = 10
    force_release_sample_interval_sec: float = 0.05
    force_release_threshold_n: float = 10.0
    force_release_consecutive_samples: int = 3
    force_release_timeout_sec: float = 0.0


@dataclass(frozen=True)
class SearchConfig:
    supported_object_names: tuple[str, ...] = (
        "yellow_can", "green_box", "gray_box", "white_bear",
        "aircon_remote", "green_frog", "otter_in_can",
    )
    search_zone_joints_deg: tuple[tuple[float, ...], ...] = (
        (1.996480318, -3.487940051, 111.818351968, -60.701755774, 86.472933819, 196.032262170),
        (0.101250008, 31.467996332, 69.414809533, -63.028302639, 85.648584603, 191.325223661),
        (-11.122401276, 22.100840948, 55.981835512, 1.028703093, 92.621606677, 79.133683826),
        (-16.628486555, -21.200191745, 107.438418044, -2.247268643, 97.683856043, 71.185741004),
        (23.901415422, -18.524249945, 105.178349409, 1.706448566, 95.826591005, 114.023922519),
        (12.143585444, 22.670610233, 60.663773141, 3.168730939, 94.347274482, 105.923693818),
    )
    # DB x/y/z is an object position in the robot base frame, not a TCP pose.
    # Place the DB position on the camera optical +Z axis before requesting a
    # zone-0 detection.  The current camera orientation is preserved.
    db_search_camera_clearance_mm: float = 300.0
    db_search_workspace_min_xyz_mm: tuple[float, float, float] = (
        100.0, -700.0, -50.0,
    )
    db_search_workspace_max_xyz_mm: tuple[float, float, float] = (
        850.0, 700.0, 600.0,
    )
    linear_vel: float = 30.0
    linear_acc: float = 60.0
    detection_service: str = "/find_object_pose"
    tcp_pose_service: str = "/update_robot_tcp_pose"
    detection_service_wait_timeout_sec: float = 2.0
    tcp_pose_service_wait_timeout_sec: float = 2.0
    tcp_pose_response_timeout_sec: float = 2.0
    detection_timeout_sec: float = 20.0
    landmark_targets: tuple[tuple[str, str], ...] = (
        ("green_box", "green_box"), ("gray_box", "gray_box"),
    )
    landmark_dwell_sec: float = 3.0
    # Offset from the detected green_box CAD origin, expressed in the
    # green_box local frame.  Only local +Z is applied.
    green_box_handle_local_xyz_mm: tuple[float, float, float] = (0.0, 0.0, 40.0)
    green_box_approach_clearance_mm: float = 120.0
    green_box_lift_clearance_mm: float = 120.0
    green_box_lid_place_local_xy_mm: tuple[float, float] = (0.0, 200.0)
    green_box_lid_place_z_offset_mm: float = 0.0
    green_box_close_delay_sec: float = 2.0
    # Sliding-drawer geometry, ported from moveit_test v4.  All offsets
    # except the observation height are expressed in the detected drawer's
    # local frame; +X is the physical pull direction.
    gray_box_handle_local_xyz_mm: tuple[float, float, float] = (116.5, 0.0, -6.0)
    gray_box_approach_clearance_mm: float = 50.0
    gray_box_open_distance_mm: float = 155.0
    gray_box_release_retreat_mm: float = 40.0
    gray_box_observe_lift_mm: float = 150.0
    gray_box_shell_front_local_x_mm: float = 103.5
    gray_box_observe_height_mm: float = 380.0
    gray_box_observe_yaw_offset_deg: float = 135.0
    gray_box_azimuth_search_limit_deg: int = 20


@dataclass(frozen=True)
class RecenterConfig:
    """Camera recentring policy for clipped detector observations."""

    enabled: bool = True
    expected_frame_id: str = "camera_color_optical_frame"
    max_attempts: int = 3
    settle_sec: float = 0.5
    velocity: float = 10.0
    acceleration: float = 20.0


@dataclass(frozen=True)
class PoseConfig:
    input_mode: str = "any6d"
    accepted_camera_frames: tuple[str, ...] = (
        "camera", "camera_link", "camera_color_optical_frame",
    )
    tcp_to_camera: tuple[tuple[float, ...], ...] = (
        (-0.999956248, 0.007801202, 0.005161742, 34.1555613),
        (-0.007796429, -0.999969162, 0.000944147, 77.5664148),
        (0.005168948, 0.000903863, 0.999986232, -182.50),
        (0.0, 0.0, 0.0, 1.0),
    )
    camera_position_scale_to_mm: float = 1000.0
    min_depth_mm: float = -5.0
    max_age_sec: float = 0.5
    pose_is_tcp_grasp: bool = True
    object_to_grasp_npy: str = ""
    wait_timeout_sec: float = 30.0


@dataclass(frozen=True)
class InterfaceConfig:
    control_init_service: str = "/control/init"
    control_task_service: str = "/control/task"
    control_search_action: str = "/control/search"
    state_result_service: str = "/state/robot_result"
    db_load_service: str = "/db/load"
    db_service_wait_timeout_sec: float = 2.0
    db_response_timeout_sec: float = 5.0
    max_pending_tasks: int = 10


@dataclass(frozen=True)
class AppConfig:
    robot: RobotConfig = field(default_factory=RobotConfig)
    moveit: MoveItConfig = field(default_factory=MoveItConfig)
    gripper: GripperConfig = field(default_factory=GripperConfig)
    motion: MotionConfig = field(default_factory=MotionConfig)
    search: SearchConfig = field(default_factory=SearchConfig)
    recenter: RecenterConfig = field(default_factory=RecenterConfig)
    pose: PoseConfig = field(default_factory=PoseConfig)
    interface: InterfaceConfig = field(default_factory=InterfaceConfig)


DEFAULT_CONFIG = AppConfig()
