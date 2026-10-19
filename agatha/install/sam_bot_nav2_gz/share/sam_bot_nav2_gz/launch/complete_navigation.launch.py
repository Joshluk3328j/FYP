import launch
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    NotSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch.events.process import ProcessIO
from launch.event_handlers import OnProcessIO


# Wait for a specific string in process output before firing next actions
def on_matching_output(matcher: str, result: launch.SomeActionsType):
    def on_output(event: ProcessIO):
        for line in event.text.decode().splitlines():
            if matcher in line:
                return result
    return on_output


def generate_launch_description():

    # Output messages to wait for before advancing the startup chain
    diff_drive_loaded_message = (
        "Successfully loaded controller diff_drive_base_controller into state active"
    )
    toolbox_ready_message  = "Registering sensor"
    navigation_ready_message = "Creating bond timer"

    use_sim    = LaunchConfiguration("use_sim")
    run_headless = LaunchConfiguration("run_headless")
    world_file = LaunchConfiguration("world_file")

    # ---------------------------------------------------------------
    # SIMULATION bringup  (use_sim:=true)
    # ---------------------------------------------------------------
    sim_bringup = ExecuteProcess(
        condition=IfCondition(use_sim),
        name="launch_bringup_sim",
        cmd=[
            "ros2", "launch",
            PathJoinSubstitution([
                FindPackageShare("sam_bot_nav2_gz"),
                "launch", "display.launch.py",
            ]),
            "use_rviz:=false",
            "use_sim_time:=true",
            ["run_headless:=", run_headless],
            "use_localization:=false",   # EKF launched separately below
            ["world_file:=", world_file],
        ],
        shell=False,
        output="screen",
    )

    # ---------------------------------------------------------------
    # HARDWARE bringup  (use_sim:=false)
    # ---------------------------------------------------------------
    hw_bringup = ExecuteProcess(
        condition=UnlessCondition(use_sim),
        name="launch_bringup_hw",
        cmd=[
            "ros2", "launch",
            PathJoinSubstitution([
                FindPackageShare("sam_bot_nav2_gz"),
                "launch", "hardware_bringup.launch.py",
            ]),
            "use_rviz:=false",   # RViz launched later after nav is ready
        ],
        shell=False,
        output="screen",
    )

    # ---------------------------------------------------------------
    # SLAM Toolbox  (same for both sim and hardware)
    # ---------------------------------------------------------------
    toolbox = ExecuteProcess(
        name="launch_slam_toolbox",
        cmd=[
            "ros2", "launch",
            PathJoinSubstitution([
                FindPackageShare("slam_toolbox"),
                "launch", "online_async_launch.py",
            ]),
            ["use_sim_time:=", use_sim],
        ],
        shell=False,
        output="screen",
    )

    # ---------------------------------------------------------------
    # Nav2  (same for both sim and hardware)
    # ---------------------------------------------------------------
    # navigation = ExecuteProcess(
    #     name="launch_navigation",
    #     cmd=[
    #         "ros2", "launch",
    #         PathJoinSubstitution([
    #             FindPackageShare("nav2_bringup"),
    #             "launch", "navigation_launch.py",
    #         ]),
    #         ["use_sim_time:=", use_sim],
    #         ["params_file:=", LaunchConfiguration("params_file")],
    #     ],
    #     shell=False,
    #     output="screen",
    # )

    navigation = ExecuteProcess(
        name="launch_navigation",
        cmd=[
            "ros2", "launch",
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch", "navigation_launch.py",
            ]),
            ["use_sim_time:=", use_sim],
            ["params_file:=", LaunchConfiguration("params_file")],
            # Add this remapping:
            "cmd_vel_topic:=/diff_drive_base_controller/cmd_vel_unstamped",  # ← won't work for ExecuteProcess
        ],
        shell=False,
        output="screen",
    )

    # ---------------------------------------------------------------
    # RViz  (launched after nav is ready)
    # ---------------------------------------------------------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    

    # ---------------------------------------------------------------
    # Startup chain:
    #   bringup → (diff drive loaded) → toolbox
    #          → (toolbox ready)      → navigation + rviz
    #          → (nav ready)          → log "Ready!"
    # ---------------------------------------------------------------

    # For simulation, wait for diff_drive controller message
    waiting_toolbox_sim = RegisterEventHandler(
        condition=IfCondition(use_sim),
        event_handler=OnProcessIO(
            target_action=sim_bringup,
            on_stdout=on_matching_output(
                diff_drive_loaded_message,
                [
                    LogInfo(msg="[SIM] Diff drive loaded. Starting SLAM Toolbox..."),
                    toolbox,
                ],
            ),
        ),
    )

    # For hardware, start toolbox immediately after bringup starts
    # (no need to wait for diff drive controller message)
    waiting_toolbox_hw = RegisterEventHandler(
        condition=UnlessCondition(use_sim),
        event_handler=OnProcessIO(
            target_action=hw_bringup,
            on_stdout=on_matching_output(
                "imu_icm20948]: IMU has started",          # from imu_icm20948 node
                [
                    LogInfo(msg="[HW] IMU started. Starting SLAM Toolbox..."),
                    TimerAction(period=3.0, actions=[toolbox]),
                ],
            ),
        ),
    )

    waiting_navigation = RegisterEventHandler(
        event_handler=OnProcessIO(
            target_action=toolbox,
            on_stdout=on_matching_output(
                toolbox_ready_message,
                [
                    LogInfo(msg="SLAM Toolbox ready. Starting Nav2..."),
                    TimerAction(
                        period=20.0,
                        actions=[navigation],
                    ),
                    rviz_node,
                ],
            ),
        )
    )

    waiting_success = RegisterEventHandler(
        event_handler=OnProcessIO(
            target_action=navigation,
            on_stdout=on_matching_output(
                navigation_ready_message,
                [LogInfo(msg="Ready for navigation!")],
            ),
        )
    )

    # ---------------------------------------------------------------
    return launch.LaunchDescription([

        DeclareLaunchArgument(
            name="use_sim",
            default_value="False",
            description="Set to true for Gazebo simulation, false for real hardware",
        ),
        DeclareLaunchArgument(
                name="rvizconfig",
                default_value=[
                    FindPackageShare("sam_bot_nav2_gz"),
                    "/rviz/navigation_config.rviz",
                ],
                description="Absolute path to rviz config file",
            ),
        DeclareLaunchArgument(
            name="run_headless",
            default_value="False",
            description="(Sim only) Run Gazebo headless",
        ),
        DeclareLaunchArgument(
            name="world_file",
            default_value="empty.sdf",
            description="(Sim only) World file to load",
        ),
        # DeclareLaunchArgument(
        #     name="params_file",
        #     default_value=[
        #         FindPackageShare("sam_bot_nav2_gz"), "/config/nav2_params.yaml"
        #     ],
        #     description="Full path to Nav2 params file",
        # ),
        DeclareLaunchArgument(
            "planner_set",
            default_value="dstar_apf",
            description="Planner combo: navfn_apf, navfn_rpp, navfn_dwa, dstar_rpp, dstar_dwa, dstar_apf",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("sam_bot_nav2_gz"),
                "config", "planners",
                [LaunchConfiguration("planner_set"), ".yaml"]
            ]),
            description="Full path to the ROS2 parameters file (overrides planner_set if set explicitly)",
        ),

        # Bringup (only one fires depending on use_sim)
        sim_bringup,
        hw_bringup,

        # Startup chain event handlers
        waiting_toolbox_sim,
        waiting_toolbox_hw,
        waiting_navigation,
        waiting_success,
    ])