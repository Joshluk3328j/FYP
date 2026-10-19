# AGATHA

**AGATHA** is an autonomous mobile robot built on **ROS 2** and **Nav2**. This repository contains everything behind the project: the mechanical design, the robot's firmware and hardware drivers, a Gazebo simulation, custom navigation algorithms, and the benchmarks used to evaluate them.

The robot can navigate on its own around static and dynamic obstacles using:

- **D\* Lite** as the global path planner (re-plans efficiently when the map changes)
- **Artificial Potential Field (APF)** as the local controller (steers toward the goal, away from obstacles)

Both are written as Nav2 plugins, so they can be swapped in for the default Nav2 planner and controller. The same software runs on the real robot and in simulation.

---

## Demo

> 🎥 **Demo video:** [Watch AGATHA in action](ADD_VIDEO_LINK_HERE)
>
> <!-- Tip: you can also embed a GIF: ![demo](AGATHA_RENDERED/demo.gif) -->

## What the robot looks like

| Render | Render |
|:---:|:---:|
| ![AGATHA render 1](AGATHA_RENDERED/ADD_IMAGE_NAME_1.png) | ![AGATHA render 2](AGATHA_RENDERED/ADD_IMAGE_NAME_2.png) |

<!-- Add a photo of the real robot too: ![AGATHA real](AGATHA_RENDERED/real_robot.jpg) -->

---

## Repository structure

```
.
├── agatha/                     # ROS 2 (colcon) workspace
│   ├── src/
│   │   ├── agatha_hardware/    # ros2_control hardware interface for the real robot
│   │   ├── icm20948-ros2/      # ROS 2 driver for the ICM-20948 IMU
│   │   └── nav2_ign_example/
│   │       └── src/
│   │           ├── sam_bot_nav2_gz/    # Robot description, worlds, Nav2 config, launch files (sim + robot)
│   │           ├── dstar_lite_planner/ # Custom global planner (Nav2 plugin)
│   │           └── apf_controller/     # Custom local controller (Nav2 plugin)
│   └── agatha_arduino_firmware/ # Microcontroller firmware (motors / encoders)
├── AGATHA_CAD_Model/           # CAD assembly, parts, STLs, drawings, G-code, URDF export
├── AGATHA_RENDERED/            # Renders of the robot
├── benchmark_results/          # Static and dynamic navigation benchmark data and figures
└── motors_test/                # Standalone motor test code
```

## How it works (quick overview)

1. **Firmware** on the microcontroller drives the motors and reads the encoders.
2. **`agatha_hardware`** exposes the motors/encoders to ROS 2 through `ros2_control`.
3. **`icm20948-ros2`** publishes IMU data used for state estimation.
4. **Nav2** handles localization, costmaps and behaviour, using **D\* Lite** for global planning and **APF** for local control.
5. **`sam_bot_nav2_gz`** provides the robot model, the Gazebo world, the Nav2 parameters and the launch files. With `use_simulation` enabled, it runs everything in Gazebo instead of on the real hardware.

---

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Nav2, `ros2_control`, `robot_localization`, `xacro`, RViz2
- Gazebo (Ignition) and `ros_gz` (for simulation)
- `colcon`

```bash
sudo apt update
sudo apt install \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-robot-localization ros-humble-xacro \
  ros-humble-ros-gz python3-colcon-common-extensions
```

For the real robot you also need the Arduino IDE (or `arduino-cli`) to flash the firmware.

---

## Build

```bash
git clone <YOUR_REPO_URL>
cd <repo>/agatha

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

> If `icm20948-ros2` or `nav2_ign_example` are separate git repositories, clone with `--recurse-submodules`, or clone them into `agatha/src/` manually.

---

## Run on the real robot

1. **Flash the firmware.** Open `agatha/agatha_arduino_firmware/` in the Arduino IDE, select your board and port, and upload.
2. **Connect** the microcontroller and the IMU to the computer on the robot (check the serial port name in the `agatha_hardware` config, e.g. `/dev/ttyUSB0`).
3. **Clone and build** the workspace on the robot's computer (see [Build](#build)).
4. **Launch:**

```bash
cd agatha
source install/setup.bash

# Bring up hardware, IMU, and the Nav2 stack
ros2 launch sam_bot_nav2_gz display.launch.py use_simulation:=false
```

5. In RViz, set an initial pose with **2D Pose Estimate**, then send a goal with **Nav2 Goal**.

## Run the simulation

No robot needed.

```bash
git clone <YOUR_REPO_URL>
cd <repo>/agatha

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch sam_bot_nav2_gz display.launch.py use_simulation:=true
```

Gazebo opens with the retail-store world and AGATHA spawned in it, and RViz opens alongside. Set the initial pose and send a **Nav2 Goal** to watch it plan and drive.

### Choosing the planner and controller

The planner and controller plugins are set in the Nav2 params file under `sam_bot_nav2_gz/config/` (and `config/planners/`). To use the custom algorithms, point the planner plugin at `dstar_lite_planner` and the controller plugin at `apf_controller`.

---

## Mechanical design

All CAD work is in `AGATHA_CAD_Model/`:

- `parts/` and `block/`: individual parts
- `stl/`: ready for 3D printing
- `g_code/`: machining/printing files
- `drawing/`: technical drawings
- `Assem2/`: full assembly with the URDF export (meshes, config, launch)

## Benchmarks

`benchmark_results/` contains the raw data and figures from testing the navigation stack in **static** environments and **dynamic** environments (moving obstacles and changing goals).

## Motor tests

`motors_test/` has standalone code for verifying the motors and encoders before running the full stack.

---

## Troubleshooting

- **Package not found after build:** run `source install/setup.bash` in every new terminal.
- **Gazebo can't find models:** make sure the workspace is built and sourced so the `models/` and `world/` folders are on the resource path.
- **No serial connection to the robot:** check the port name and that your user is in the `dialout` group (`sudo usermod -aG dialout $USER`, then log out and back in).

## Acknowledgements

- Simulation base adapted from the [Nav2 / sam_bot Gazebo example](https://github.com/ros-planning/navigation2_tutorials)
- Retail-store world models by AWS RoboMaker
- ICM-20948 ROS 2 driver

## License

ADD_LICENSE_HERE

## Author

ADD_YOUR_NAME_AND_CONTACT_HERE
