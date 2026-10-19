import launch
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    AndSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros
import os
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from lzma import PRESET_DEFAULT
from launch.actions import TimerAction
from launch.conditions import IfCondition, UnlessCondition




def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="sam_bot_nav2_gz"
    ).find("sam_bot_nav2_gz")
    default_model_path = os.path.join(
        pkg_share, "src/description/sam_bot_description.urdf"
    )
    default_rviz_config_path = os.path.join(pkg_share, "rviz/urdf_config.rviz")

    # ... inside generate_launch_description ...

    # Get the parent directory of your package share
    # This allows Gazebo to find 'sam_bot_nav2_gz' as a folder name
    package_parent_dir = os.path.join(pkg_share, '..')

    # Add the parent directory to the resource path
    gz_models_path = package_parent_dir

    # Inside generate_launch_description of display.launch.py

    # Define the path to your controller params
    controller_params_file = os.path.join(pkg_share, "config", "diff_drive_controller_velocity.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_localization = LaunchConfiguration("use_localization")
    use_rviz = LaunchConfiguration("use_rviz")
    log_level = LaunchConfiguration("log_level")
    gz_verbosity = LaunchConfiguration("gz_verbosity")
    run_headless = LaunchConfiguration("run_headless")
    world_file_name = LaunchConfiguration("world_file")

    gz_models_path = ":".join([package_parent_dir, 
                               os.path.join(pkg_share, "models")])
    #gz_models_path = os.path.join(pkg_share, "models")
    
    # # this is what was added
    # gz_models_path = ":".join([
    #     pkg_share,
    #     os.path.join(pkg_share, "models")
    # ])

    world_path = PathJoinSubstitution([pkg_share, "world", world_file_name])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", LaunchConfiguration("model")]),
                    value_type=str,
                ),
                "use_sim_time": use_sim_time,
            },
            # This allows the controller manager inside Gazebo to find your YAML
            controller_params_file,
        ],
    )

    rviz_node = Node(
        condition=IfCondition(AndSubstitution(NotSubstitution(run_headless), use_rviz)),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    # Localize using odometry and IMU data. 
    # It can be turned off because the navigation stack uses AMCL with lidar data for localization
    robot_localization_node = Node(
        condition=launch.conditions.IfCondition(use_localization),
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/ekf.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    # gazebo have to be executed with shell=False, or test_launch won't terminate it
    #   see: https://github.com/ros2/launch/issues/545
    # This code is form taken ros_gz_sim and modified to work with shell=False
    #   see: https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim/launch/gz_sim.launch.py.in
    gz_env = {'GZ_SIM_SYSTEM_PLUGIN_PATH':
           ':'.join([os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                     os.environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':  # TODO(CH3): To support pre-garden. Deprecated.
                      ':'.join([os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                                os.environ.get('LD_LIBRARY_PATH', default='')])}
    # CORRECTED gazebo section
    gazebo = [
        ExecuteProcess(
            condition=IfCondition(run_headless),
            cmd=['ruby', FindExecutable(name="ign"), 'gazebo', '-r', '-v', gz_verbosity, '-s', '--headless-rendering', world_path],
            output='screen',
            additional_env=gz_env,
            shell=False,
        ),
        ExecuteProcess(
            condition=IfCondition(run_headless),
            cmd=['ruby', FindExecutable(name="ign"), 'gazebo', '-r', '-v', gz_verbosity, '-s', '--headless-rendering', world_path],
            output='screen',
            additional_env={
                **gz_env,
                'LIBGL_ALWAYS_SOFTWARE': '1',
                'GALLIUM_DRIVER': 'llvmpipe',
                'GZ_RENDERING_ENGINE': 'ogre',          # force older engine
                'IGN_RENDERING_ENGINE': 'ogre',
            },
            shell=False,
        ),
    ]

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "sam_bot",
            "-topic",
            "robot_description",
            "-z",
            "1.0",
            "-x",
            "-2.0",
            "--ros-args",
            "--log-level",
            log_level,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     arguments=[
    #         # Lidar
    #         "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            
    #         # IMU
    #         "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            
    #         # Cameras
    #         "/sky_cam@sensor_msgs/msg/Image@ignition.msgs.Image",
    #         "/robot_cam@sensor_msgs/msg/Image@ignition.msgs.Image",
    #         "/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            
    #         # Clock
    #         "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",

    #         # --- NEW SENSORS ---
            
    #         # Front Ultrasonic
    #         "/front_ultrasonic@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            
    #         # Rear Ultrasonic
    #         "/rear_ultrasonic@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            
    #         # Left Ultrasonic
    #         "/left_ultrasonic@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            
    #         # Right Ultrasonic
    #         "/right_ultrasonic@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            
    #         # ToF Sensor (Mapping it as a LaserScan for easy viewing in RViz)
    #         "/tof@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
    #     ],
    #     parameters=[{'use_sim_time': use_sim_time}], # <--- ADD THIS
    #     output="screen",
    # )

    load_joint_state_controller = ExecuteProcess(
        name="activate_joint_state_broadcaster",
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        shell=False,
        output="screen",
    )

    # load_caster_broadcaster = ExecuteProcess(
    # cmd=["ros2", "control", "load_controller", "--set-state", "active", "caster_state_broadcaster"],
    # output="screen"
    # )

    # Chain it after spawn or joint_state_broadcaster
    # RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=spawn_entity,  # or your joint_state load
    #         on_exit=[load_caster_broadcaster]
    #     )
    # )

    load_joint_trajectory_controller = ExecuteProcess(
        name="activate_diff_drive_base_controller",
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "diff_drive_base_controller",
        ],
        shell=False,
        output="screen",
    )

    relay_odom = Node(
        name="relay_odom",
        package="topic_tools",
        executable="relay",
        parameters=[
            {
                "input_topic": "/diff_drive_base_controller/odom",
                "output_topic": "/odom",
            }
        ],
        output="screen",
    )

    relay_cmd_vel = Node(
        name="relay_cmd_vel",
        package="topic_tools",
        executable="relay",
        parameters=[
            {
                "input_topic": "/cmd_vel",
                "output_topic": "/diff_drive_base_controller/cmd_vel_unstamped",
            }
        ],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            # 1. Environment variables FIRST — needed for model loading
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=gz_models_path,
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=gz_models_path,
            ),

            # 2. All launch arguments next
            DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            DeclareLaunchArgument(
                name="use_rviz",
                default_value="True",
                description="Start RViz",
            ),
            DeclareLaunchArgument(
                name="run_headless",
                default_value="False",
                description="Start GZ in headless mode and don't start RViz (overrides use_rviz)",
            ),
            DeclareLaunchArgument(
                name="world_file",
                default_value="empty.sdf",
            ),
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),
            DeclareLaunchArgument(
                name="use_localization",
                default_value="True",
                description="Use EKF to estimate odom->base_link transform from IMU + wheel odometry",
            ),
            DeclareLaunchArgument(
                "gz_verbosity",
                default_value="3",
                description="Verbosity level for Ignition Gazebo (0~4).",
            ),
            DeclareLaunchArgument(
                "gz_args",
                default_value="",
                description="Extra args for Gazebo (ie. '-s' for running headless)",
            ),
            DeclareLaunchArgument(
                name="log_level",
                default_value="warn",
                description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
            ),

            # 3. Start GAZEBO VERY EARLY — most important change
            *gazebo,   # This launches gazebo before almost everything else

            # 4. Bridge immediately after gazebo — force use_sim_time=True
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                    "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
                    "/sky_cam@sensor_msgs/msg/Image@ignition.msgs.Image",
                    "/robot_cam@sensor_msgs/msg/Image@ignition.msgs.Image",
                    "/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
                    "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                    "/front_ultrasonic@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                    "/rear_ultrasonic@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                    "/left_ultrasonic@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                    "/right_ultrasonic@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                    "/tof@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                ],
                parameters=[{'use_sim_time': True}],  # ← hard-coded here
                output="screen",
            ),

            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[
                    os.path.join(pkg_share, "config/ekf.yaml"),
                    {"use_sim_time": True},
                ],
                remappings=[
                    ("/odometry/filtered", "/odom"),
                ],
            ),

            # Robot state publisher
            robot_state_publisher_node,

            # 5. Delay robot spawn by 6 seconds — critical for clock stability
            TimerAction(
                period=10.0,
                actions=[spawn_entity]
            ),

            # 6. Delay EKF a bit more (optional but helps a lot)
            TimerAction(
                period=2.0,  # starts ~2 seconds after spawn
                actions=[robot_localization_node]
            ),

            # RViz
            rviz_node,

            # Controller loading chain
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[load_joint_state_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_controller,
                    on_exit=[load_joint_trajectory_controller],
                )
            ),

            # Relays at the end
            relay_cmd_vel,
        ]
    )