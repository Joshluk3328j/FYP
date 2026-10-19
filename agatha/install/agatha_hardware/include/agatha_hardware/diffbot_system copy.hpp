#ifndef AGATHA_HARDWARE__DIFFBOT_SYSTEM_HPP_
#define AGATHA_HARDWARE__DIFFBOT_SYSTEM_HPP_

#include <string>
#include <vector>
#include <array>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"   // ← was Range, now LaserScan

namespace agatha_hardware
{

class DiffBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  bool openSerial(const std::string & port, int baud);
  void closeSerial();
  bool readLine(std::string & line);

  // ── Serial ───────────────────────────────────────────────────────────────
  int         serial_fd_{-1};
  std::string serial_buf_;

  // ── Hardware parameters (from URDF <ros2_control> tag) ───────────────────
  std::string serial_port_;
  int         serial_baud_{115200};
  double      ticks_per_rev_{1980.0};
  double      wheel_radius_{0.068};

  // ── Wheel state ───────────────────────────────────────────────────────────
  // Index 0 = left wheel, 1 = right wheel
  double hw_positions_[2]{0.0, 0.0};
  double hw_velocities_[2]{0.0, 0.0};
  double hw_commands_[2]{0.0, 0.0};
  long   prev_ticks_[2]{0L, 0L};

  // ── Ultrasonic publishers (fake single-beam LaserScan) ────────────────────
  // Each HC-SR04 is published as a 1-beam LaserScan pointing straight ahead
  // of its sensor frame. The voxel_layer raytrace-clears and marks cells
  // exactly as it does for the real LiDAR — no separate plugin needed.
  rclcpp::Node::SharedPtr ultrasonic_node_;

  static constexpr size_t NUM_SONAR = 4;

  // Ordered: front, rear, left, right — must match Arduino U-line order
  const std::array<std::string, NUM_SONAR> sonar_topics_{
    "/front_ultrasonic", "/rear_ultrasonic",
    "/left_ultrasonic",  "/right_ultrasonic"
  };
  const std::array<std::string, NUM_SONAR> sonar_frames_{
    "front_ultrasonic", "rear_ultrasonic",
    "left_ultrasonic",  "right_ultrasonic"
  };

  std::array<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr, NUM_SONAR>
    sonar_pubs_;

  // HC-SR04 operating limits in metres
  static constexpr float SONAR_MIN_RANGE = 0.02f;   // 2 cm blind zone
  static constexpr float SONAR_MAX_RANGE = 1.00f;   // 1 m — open-air reading

  rclcpp::Logger logger_{rclcpp::get_logger("DiffBotSystemHardware")};
};

}  // namespace agatha_hardware

#endif  // AGATHA_HARDWARE__DIFFBOT_SYSTEM_HPP_