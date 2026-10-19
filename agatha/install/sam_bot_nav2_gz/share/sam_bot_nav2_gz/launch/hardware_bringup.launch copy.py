import launch
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    NotSubstitution,
    AndSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros
import os
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="sam_bot_nav2_gz"
    ).find("sam_bot_nav2_gz")

    default_model_path = os.path.join(
        pkg_share, "src/description/sam_bot_description.urdf"
    )
    default_rviz_config_path = os.path.join(pkg_share, "rviz/urdf_config.rviz")
    controller_params_file = os.path.join(
        pkg_share, "config", "diff_drive_controller_velocity.yaml"
    )

    # ---------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------
    use_rviz        = LaunchConfiguration("use_rviz")
    log_level       = LaunchConfiguration("log_level")

    # ---------------------------------------------------------------
    # Nodes
    # ---------------------------------------------------------------

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", LaunchConfiguration("model")]),
                    value_type=str,
                ),
                "use_sim_time": False,
            },
            controller_params_file,
        ],
    )

    # --- IMU pipeline ---

    imu_node = Node(
        package="icm20948_ros2",
        executable="imu_node",
        name="imu_icm20948",
        output="screen",
        parameters=[{"i2c_address": 0x68}],
    )

    madgwick_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick",
        output="screen",
        parameters=[{
            "use_mag": False,
            "publish_tf": False,
            "world_frame": "enu",
            "use_sim_time": False,
        }],
        remappings=[
            ("/imu/data_raw", "/imu/data_raw"),
            ("/imu/data",     "/imu/data"),
        ],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/ekf.yaml"),
            {"use_sim_time": False},
        ],
    )

    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        name="rplidar_node",
        output="screen",
        parameters=[{
            "serial_port": "/dev/rplidar",
            "serial_baudrate": 115200,
            "channel_type": "serial",       # <-- add this
            "frame_id": "lidar",
            "angle_compensate": True,
            "scan_mode": "Standard" # Sometimes "Boost" causes timeouts on Pi
        }],
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    # --- ros2_control (uncomment when motors are ready) ---

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": ParameterValue(
                Command(["xacro ", LaunchConfiguration("model")]),
                value_type=str,
            )},
            controller_params_file,
            {"use_sim_time": False},
        ],
        output="screen",
    )

    load_joint_state_controller = ExecuteProcess(
        name="activate_joint_state_broadcaster",
        cmd=["ros2", "control", "load_controller",
             "--set-state", "active", "joint_state_broadcaster"],
        shell=False,
        output="screen",
    )

    load_diff_drive_controller = ExecuteProcess(
        name="activate_diff_drive_base_controller",
        cmd=["ros2", "control", "load_controller",
             "--set-state", "active", "diff_drive_base_controller"],
        shell=False,
        output="screen",
    )

    # Load joint state broadcaster after ros2_control_node starts
    load_jsb_after_control = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ros2_control_node,
            on_exit=[load_joint_state_controller],
        )
    )

    # Load diff drive controller after joint state broadcaster loads
    load_ddc_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_diff_drive_controller],
        )
    )

    # relay_cmd_vel = Node(
    #     name="relay_cmd_vel",
    #     package="topic_tools",
    #     executable="relay",
    #     parameters=[{
    #         "input_topic":  "/cmd_vel",
    #         "output_topic": "/diff_drive_base_controller/cmd_vel_unstamped",
    #     }],
    #     output="screen",
    # )

    # RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=ros2_control_node,
    #         on_exit=[load_joint_state_controller],
    #     )
    # ),
    # RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=load_joint_state_controller,
    #         on_exit=[load_diff_drive_controller],
    #     )
    # ),

    # ---------------------------------------------------------------
    # Launch description
    # ---------------------------------------------------------------
    return launch.LaunchDescription([

        # --- Arguments ---
        DeclareLaunchArgument(
            name="model",
            default_value=default_model_path,
            description="Absolute path to robot URDF/xacro file",
        ),
        DeclareLaunchArgument(
            name="rvizconfig",
            default_value=default_rviz_config_path,
            description="Absolute path to RViz config file",
        ),
        DeclareLaunchArgument(
            name="use_rviz",
            default_value="True",
            description="Launch RViz",
        ),
        DeclareLaunchArgument(
            name="log_level",
            default_value="warn",
            description="Logging level for all nodes",
        ),

        # --- Hardware nodes ---
        robot_state_publisher_node,
        imu_node,
        madgwick_node,
        lidar_node,
        ekf_node,
        ros2_control_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ros2_control_node,
                on_exit=[load_joint_state_controller],
            )
        ),

        # Load diff drive controller after joint state broadcaster loads
        # load_ddc_after_jsb = 
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_drive_controller],
            )
        ),
        load_jsb_after_control,
        load_ddc_after_jsb,




        rviz_node,

        # Uncomment below when motors are wired up:
        # ros2_control_node,
        # relay_cmd_vel,
    ])