#include "agatha_hardware/diffbot_system.hpp"

#include <cstring>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  agatha_hardware::DiffBotSystemHardware,
  hardware_interface::SystemInterface)

namespace agatha_hardware
{

// ── Lifecycle ────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn
DiffBotSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  serial_port_ = info_.hardware_parameters.count("serial_port")
    ? info_.hardware_parameters.at("serial_port") : "/dev/arduino_nano";

  serial_baud_ = info_.hardware_parameters.count("serial_baud")
    ? std::stoi(info_.hardware_parameters.at("serial_baud")) : 115200;

  ticks_per_rev_ = info_.hardware_parameters.count("ticks_per_rev")
    ? std::stod(info_.hardware_parameters.at("ticks_per_rev")) : 1980.0;

  wheel_radius_ = info_.hardware_parameters.count("wheel_radius")
    ? std::stod(info_.hardware_parameters.at("wheel_radius")) : 0.068;

  if (info_.joints.size() != 2) {
    RCLCPP_FATAL(logger_,
      "Expected exactly 2 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    bool has_vel_cmd = false;
    for (const auto & cmd : joint.command_interfaces) {
      if (cmd.name == hardware_interface::HW_IF_VELOCITY) has_vel_cmd = true;
    }
    if (!has_vel_cmd) {
      RCLCPP_FATAL(logger_,
        "Joint '%s' missing velocity command interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(logger_, "on_init OK — port: %s  baud: %d  ticks/rev: %.0f",
    serial_port_.c_str(), serial_baud_, ticks_per_rev_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DiffBotSystemHardware::on_configure(const rclcpp_lifecycle::State &)
{
  hw_positions_[0] = hw_positions_[1] = 0.0;
  hw_velocities_[0] = hw_velocities_[1] = 0.0;
  hw_commands_[0] = hw_commands_[1] = 0.0;
  prev_ticks_[0] = prev_ticks_[1] = 0;
  serial_buf_.clear();

  RCLCPP_INFO(logger_, "on_configure OK");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DiffBotSystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating Agatha hardware...");

  if (!openSerial(serial_port_, serial_baud_)) {
    RCLCPP_ERROR(logger_, "Failed to open serial port: %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  tcflush(serial_fd_, TCIOFLUSH);

  // ── Create publisher node ─────────────────────────────────────────────────
  ultrasonic_node_ = std::make_shared<rclcpp::Node>("agatha_ultrasonic_publisher");

  for (size_t i = 0; i < NUM_SONAR; i++) {
    sonar_pubs_[i] =
      ultrasonic_node_->create_publisher<sensor_msgs::msg::LaserScan>(
        sonar_topics_[i], rclcpp::QoS(10).reliable().durability_volatile());
  }

  RCLCPP_INFO(logger_, "Agatha connected on %s — ultrasonic LaserScan publishers ready",
    serial_port_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DiffBotSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (serial_fd_ >= 0) {
    const char * stop = "STOP\n";
    ::write(serial_fd_, stop, strlen(stop));
  }
  closeSerial();

  sonar_pubs_.fill(nullptr);
  ultrasonic_node_.reset();

  RCLCPP_INFO(logger_, "Hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DiffBotSystemHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  closeSerial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── Interface export ─────────────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface>
DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < 2; i++) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < 2; i++) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }
  return command_interfaces;
}

// ── Read ─────────────────────────────────────────────────────────────────────

hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (serial_fd_ < 0) return hardware_interface::return_type::ERROR;

  std::string line;
  bool got_encoder = false;
  long new_left  = prev_ticks_[0];
  long new_right = prev_ticks_[1];

  int max_reads = 20;
  while (max_reads-- > 0 && readLine(line)) {
    if (line.empty()) continue;

    // ── Encoder line: "E <leftTicks> <rightTicks>" ───────────────────────
    if (line[0] == 'E') {
      long lt = 0, rt = 0;
      if (sscanf(line.c_str(), "E %ld %ld", &lt, &rt) == 2) {
        new_left  = lt;
        new_right = rt;
        got_encoder = true;
      }
      continue;
    }

    // ── Ultrasonic line: "U <front_cm> <rear_cm> <left_cm> <right_cm>" ──
    if (line[0] == 'U') {
      float f_cm = 0, r_cm = 0, l_cm = 0, ri_cm = 0;
      if (sscanf(line.c_str(), "U %f %f %f %f",
                 &f_cm, &r_cm, &l_cm, &ri_cm) == 4)
      {
        const float cm_to_m = 0.01f;
        const float values_cm[NUM_SONAR] = { f_cm, r_cm, l_cm, ri_cm };

        for (size_t i = 0; i < NUM_SONAR; i++) {
          float dist_m = values_cm[i] * cm_to_m;

          // Clamp to sensor operating range
          dist_m = std::max(SONAR_MIN_RANGE,
                   std::min(dist_m, SONAR_MAX_RANGE));

          // Build a single-beam LaserScan
          // angle_min == angle_max == 0.0 → one ray pointing straight ahead
          // of the sensor frame. The voxel_layer treats it identically to a
          // real LiDAR ray: raytrace-clears up to the measured range, marks
          // the cell at that range as occupied.
          sensor_msgs::msg::LaserScan msg;
          msg.header.stamp    = time;
          msg.header.frame_id = sonar_frames_[i];

          // Give it a tiny 1-degree field of view spread centered at 0
          msg.angle_min       = -0.0087f;  // -0.5 degrees in radians
          msg.angle_max       =  0.0087f;  // +0.5 degrees in radians
          msg.angle_increment =  0.0174f;  // 1.0 degree step
          msg.time_increment  =  0.0f;
          msg.scan_time       =  0.1f;   
          msg.range_min       = SONAR_MIN_RANGE;
          msg.range_max       = SONAR_MAX_RANGE;

          // If the Arduino reports max_range the sensor saw nothing —
          // publish +inf so the voxel_layer clears the ray without marking.
          if (dist_m >= SONAR_MAX_RANGE) {
            msg.ranges = { std::numeric_limits<float>::infinity() };
          } else {
            msg.ranges = { dist_m };
          }

          sonar_pubs_[i]->publish(msg);
        }

        rclcpp::spin_some(ultrasonic_node_);
      }
      continue;
    }
  }

  // ── Update wheel odometry ─────────────────────────────────────────────────
  if (got_encoder) {
    double dt = period.seconds();
    if (dt <= 0.0) dt = 0.02;

    double left_delta  = static_cast<double>(new_left  - prev_ticks_[0]);
    double right_delta = static_cast<double>(new_right - prev_ticks_[1]);

    hw_velocities_[0] = (left_delta  / ticks_per_rev_) * 2.0 * M_PI / dt;
    hw_velocities_[1] = (right_delta / ticks_per_rev_) * 2.0 * M_PI / dt;

    hw_positions_[0] += (left_delta  / ticks_per_rev_) * 2.0 * M_PI;
    hw_positions_[1] += (right_delta / ticks_per_rev_) * 2.0 * M_PI;

    prev_ticks_[0] = new_left;
    prev_ticks_[1] = new_right;
  }

  return hardware_interface::return_type::OK;
}

// ── Write ────────────────────────────────────────────────────────────────────

hardware_interface::return_type DiffBotSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (serial_fd_ < 0) return hardware_interface::return_type::OK;

  std::string command = "V," + std::to_string(hw_commands_[0]) + "," +
                        std::to_string(hw_commands_[1]) + "\n";

  ssize_t bytes_written = ::write(serial_fd_, command.c_str(), command.size());

  if (bytes_written < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) return hardware_interface::return_type::OK;
    RCLCPP_ERROR(logger_, "Serial write failed: %s", strerror(errno));
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// ── Serial helpers ───────────────────────────────────────────────────────────

bool DiffBotSystemHardware::openSerial(const std::string & port, int baud)
{
  serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(logger_, "open(%s) failed: %s", port.c_str(), strerror(errno));
    return false;
  }

  struct termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(logger_, "tcgetattr failed: %s", strerror(errno));
    closeSerial();
    return false;
  }

  speed_t speed = B115200;
  if (baud == 57600) speed = B57600;
  if (baud == 9600)  speed = B9600;

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  cfmakeraw(&tty);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(logger_, "tcsetattr failed: %s", strerror(errno));
    closeSerial();
    return false;
  }

  tcflush(serial_fd_, TCIOFLUSH);
  return true;
}

void DiffBotSystemHardware::closeSerial()
{
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool DiffBotSystemHardware::readLine(std::string & line)
{
  char tmp[128];
  ssize_t n = ::read(serial_fd_, tmp, sizeof(tmp) - 1);

  if (n > 0) {
    serial_buf_.append(tmp, n);
  } else if (n < 0 && (errno != EAGAIN && errno != EWOULDBLOCK)) {
    return false;
  }

  auto last_nl = serial_buf_.find_last_of('\n');
  if (last_nl == std::string::npos) return false;

  auto prev_nl = (last_nl > 0)
    ? serial_buf_.rfind('\n', last_nl - 1)
    : std::string::npos;

  size_t start = (prev_nl == std::string::npos) ? 0 : prev_nl + 1;
  line = serial_buf_.substr(start, last_nl - start);
  if (!line.empty() && line.back() == '\r') line.pop_back();

  serial_buf_ = serial_buf_.substr(last_nl + 1);

  return !line.empty();
}

}  // namespace agatha_hardware