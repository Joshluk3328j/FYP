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
#include "sensor_msgs/msg/range.hpp"

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

  // ── Ultrasonic publishers ─────────────────────────────────────────────────
  // We need a plain rclcpp::Node to publish from inside the hardware plugin.
  // The node is created in on_activate() and destroyed in on_deactivate().
  rclcpp::Node::SharedPtr ultrasonic_node_;

  static constexpr size_t NUM_SONAR = 4;

  // Topic names and matching tf frame ids, ordered: front, rear, left, right
  const std::array<std::string, NUM_SONAR> sonar_topics_{
    "/front_ultrasonic", "/rear_ultrasonic",
    "/left_ultrasonic",  "/right_ultrasonic"
  };
  const std::array<std::string, NUM_SONAR> sonar_frames_{
    "front_ultrasonic", "rear_ultrasonic",
    "left_ultrasonic",  "right_ultrasonic"
  };

  std::array<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr, NUM_SONAR>
    sonar_pubs_;

  // HC-SR04 sensor constants
  static constexpr float SONAR_MIN_RANGE  = 0.02f;
  static constexpr float SONAR_MAX_RANGE  = 0.45f;  // realistic HC-SR04 reliable range
  static constexpr float SONAR_FOV        = 0.2618f; // radians (~15°)

  rclcpp::Logger logger_{rclcpp::get_logger("DiffBotSystemHardware")};
};

}  // namespace agatha_hardware

#endif  // AGATHA_HARDWARE__DIFFBOT_SYSTEM_HPP_