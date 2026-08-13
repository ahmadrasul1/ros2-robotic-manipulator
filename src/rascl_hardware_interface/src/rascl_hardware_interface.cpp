#include "rascl_hardware_interface/rascl_hardware_interface.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <exception>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rascl_robot_control
{

namespace
{
constexpr char kCommandMagic[4] = {'R', 'C', 'M', 'D'};
constexpr char kFeedbackMagic[4] = {'R', 'F', 'D', 'B'};
constexpr std::array<const char *, 4> kExpectedJointNames = {
  "shoulder_joint",
  "upperarm_joint",
  "lowerarm_joint",
  "end_effector_joint",
};
}  // namespace

RasclHardwareInterface::RasclHardwareInterface()
: server_socket_path_("/tmp/rascl_csp.sock"),
  socket_fd_(-1),
  feedback_timeout_ms_(250),
  startup_feedback_timeout_ms_(3000),
  command_sequence_(0),
  feedback_sequence_(0),
  bridge_state_(kBridgeStateStarting),
  working_counter_(0),
  statuswords_{0, 0, 0, 0},
  error_flags_(0),
  have_feedback_(false),
  active_(false),
  logged_soft_error_(false)
{
}

RasclHardwareInterface::~RasclHardwareInterface()
{
  close_ipc_socket();
}

hardware_interface::CallbackReturn RasclHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kExpectedJointCount)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RasclHardwareInterface"),
      "Expected exactly %zu joints, got %zu",
      kExpectedJointCount,
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();
  hw_positions_.assign(kExpectedJointCount, 0.0);
  hw_commands_.assign(kExpectedJointCount, 0.0);

  for (std::size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index)
  {
    const auto & joint = info_.joints[joint_index];
    if (joint.name != kExpectedJointNames[joint_index])
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("RasclHardwareInterface"),
        "Joint order mismatch at index %zu: expected '%s', got '%s'",
        joint_index,
        kExpectedJointNames[joint_index],
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("RasclHardwareInterface"),
        "Joint '%s' must have exactly one position command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("RasclHardwareInterface"),
        "Joint '%s' must have exactly one position state interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    joint_names_.push_back(joint.name);
  }

  try
  {
    if (info_.hardware_parameters.count("command_socket") > 0)
    {
      server_socket_path_ = info_.hardware_parameters.at("command_socket");
    }
    if (info_.hardware_parameters.count("feedback_timeout_ms") > 0)
    {
      feedback_timeout_ms_ = std::stoi(
        info_.hardware_parameters.at("feedback_timeout_ms"));
    }
    if (info_.hardware_parameters.count("startup_feedback_timeout_ms") > 0)
    {
      startup_feedback_timeout_ms_ = std::stoi(
        info_.hardware_parameters.at("startup_feedback_timeout_ms"));
    }
  }
  catch (const std::exception & exc)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RasclHardwareInterface"),
      "Invalid hardware parameter: %s",
      exc.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (feedback_timeout_ms_ <= 0 || startup_feedback_timeout_ms_ <= 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RasclHardwareInterface"),
      "Feedback timeouts must be positive");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("RasclHardwareInterface"),
    "Initialized PDO hardware interface: socket=%s, feedback_timeout=%d ms",
    server_socket_path_.c_str(),
    feedback_timeout_ms_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RasclHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  close_ipc_socket();

  if (!open_ipc_socket())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!send_command(kCommandFlagRegister))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RasclHardwareInterface"),
      "Failed to register with PDO bridge");
    close_ipc_socket();
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!receive_feedback(
      true, std::chrono::milliseconds(startup_feedback_timeout_ms_)))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RasclHardwareInterface"),
      "No initial PDO feedback received within %d ms",
      startup_feedback_timeout_ms_);
    close_ipc_socket();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // The first command sent by ros2_control must be the measured post-homing
  // position, never a URDF placeholder.
  hw_commands_ = hw_positions_;
  active_ = false;
  logged_soft_error_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger("RasclHardwareInterface"),
    "Configured from real PDO feedback: [%.6f, %.6f, %.6f, %.6f]",
    hw_positions_[0], hw_positions_[1], hw_positions_[2], hw_positions_[3]);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RasclHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (socket_fd_ >= 0)
  {
    send_command(kCommandFlagHalt);
  }
  active_ = false;
  close_ipc_socket();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RasclHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  if (socket_fd_ >= 0)
  {
    send_command(kCommandFlagHalt);
  }
  active_ = false;
  close_ipc_socket();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RasclHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!receive_feedback(true, std::chrono::milliseconds(feedback_timeout_ms_)))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RasclHardwareInterface"),
      "Cannot activate without fresh PDO feedback");
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_ = hw_positions_;
  active_ = true;
  logged_soft_error_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger("RasclHardwareInterface"),
    "PDO hardware interface active; first target synchronized to actual position");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RasclHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  if (!send_command(kCommandFlagHalt))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RasclHardwareInterface"),
      "Failed to send PDO halt request during deactivation");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RasclHardwareInterface::on_error(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  send_command(kCommandFlagHalt);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RasclHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(kExpectedJointCount);
  for (std::size_t index = 0; index < kExpectedJointCount; ++index)
  {
    interfaces.emplace_back(
      joint_names_[index],
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[index]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
RasclHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(kExpectedJointCount);
  for (std::size_t index = 0; index < kExpectedJointCount; ++index)
  {
    interfaces.emplace_back(
      joint_names_[index],
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[index]);
  }
  return interfaces;
}

hardware_interface::return_type RasclHardwareInterface::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (socket_fd_ < 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  receive_feedback(false, std::chrono::milliseconds(0));

  if (!have_feedback_ || feedback_is_stale())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RasclHardwareInterface"),
      "PDO feedback is missing or stale");
    return hardware_interface::return_type::ERROR;
  }

  if (bridge_state_ == kBridgeStateFault ||
      bridge_state_ == kBridgeStateStopped || has_hard_bridge_error())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RasclHardwareInterface"),
      "PDO bridge fault: state=%u, errors=0x%08X, WKC=%d",
      bridge_state_, error_flags_, working_counter_);
    return hardware_interface::return_type::ERROR;
  }

  const std::uint32_t soft_errors = kErrorWarning | kErrorMotionDisabled;
  if ((error_flags_ & soft_errors) != 0U && !logged_soft_error_)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("RasclHardwareInterface"),
      "PDO bridge hold/warning state: state=%u, errors=0x%08X. "
      "Motion may be disabled in ethercat_pdo.yaml.",
      bridge_state_, error_flags_);
    logged_soft_error_ = true;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RasclHardwareInterface::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!active_)
  {
    return hardware_interface::return_type::OK;
  }

  for (std::size_t index = 0; index < kExpectedJointCount; ++index)
  {
    if (!std::isfinite(hw_commands_[index]))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("RasclHardwareInterface"),
        "Non-finite command for joint '%s'",
        joint_names_[index].c_str());
      send_command(kCommandFlagHalt);
      return hardware_interface::return_type::ERROR;
    }
  }

  if (!send_command(kCommandFlagPositionValid))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RasclHardwareInterface"),
      "Failed to send cyclic position command to PDO bridge");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

bool RasclHardwareInterface::open_ipc_socket()
{
  socket_fd_ = ::socket(AF_UNIX, SOCK_DGRAM, 0);
  if (socket_fd_ < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RasclHardwareInterface"),
      "socket(AF_UNIX) failed: %s",
      std::strerror(errno));
    return false;
  }

  const int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags < 0 || fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RasclHardwareInterface"),
      "Failed to make CSP socket non-blocking: %s",
      std::strerror(errno));
    close_ipc_socket();
    return false;
  }

  client_socket_path_ =
    "/tmp/rascl_csp_client_" + std::to_string(::getpid()) + ".sock";
  ::unlink(client_socket_path_.c_str());

  sockaddr_un client_address{};
  client_address.sun_family = AF_UNIX;
  if (client_socket_path_.size() >= sizeof(client_address.sun_path))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RasclHardwareInterface"),
      "Client socket path is too long: %s",
      client_socket_path_.c_str());
    close_ipc_socket();
    return false;
  }
  std::strncpy(
    client_address.sun_path,
    client_socket_path_.c_str(),
    sizeof(client_address.sun_path) - 1);

  if (::bind(
      socket_fd_,
      reinterpret_cast<const sockaddr *>(&client_address),
      sizeof(client_address)) < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RasclHardwareInterface"),
      "Failed to bind client CSP socket '%s': %s",
      client_socket_path_.c_str(),
      std::strerror(errno));
    close_ipc_socket();
    return false;
  }

  sockaddr_un server_address{};
  server_address.sun_family = AF_UNIX;
  if (server_socket_path_.size() >= sizeof(server_address.sun_path))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RasclHardwareInterface"),
      "Server socket path is too long: %s",
      server_socket_path_.c_str());
    close_ipc_socket();
    return false;
  }
  std::strncpy(
    server_address.sun_path,
    server_socket_path_.c_str(),
    sizeof(server_address.sun_path) - 1);

  if (::connect(
      socket_fd_,
      reinterpret_cast<const sockaddr *>(&server_address),
      sizeof(server_address)) < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RasclHardwareInterface"),
      "Failed to connect to PDO bridge socket '%s': %s",
      server_socket_path_.c_str(),
      std::strerror(errno));
    close_ipc_socket();
    return false;
  }

  command_sequence_ = 0;
  feedback_sequence_ = 0;
  have_feedback_ = false;
  error_flags_ = 0;
  return true;
}

void RasclHardwareInterface::close_ipc_socket()
{
  if (socket_fd_ >= 0)
  {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
  if (!client_socket_path_.empty())
  {
    ::unlink(client_socket_path_.c_str());
  }
  have_feedback_ = false;
}

bool RasclHardwareInterface::send_command(std::uint32_t flags)
{
  if (socket_fd_ < 0)
  {
    return false;
  }

  CommandPacket packet{};
  std::memcpy(packet.magic, kCommandMagic, sizeof(packet.magic));
  packet.version = kIpcVersion;
  packet.flags = flags;
  packet.sequence = ++command_sequence_;
  packet.timestamp_ns = steady_time_ns();

  for (std::size_t index = 0; index < kExpectedJointCount; ++index)
  {
    packet.positions[index] = hw_commands_[index];
  }

  const ssize_t sent = ::send(socket_fd_, &packet, sizeof(packet), 0);
  if (sent == static_cast<ssize_t>(sizeof(packet)))
  {
    return true;
  }
  if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
  {
    return false;
  }
  RCLCPP_ERROR(
    rclcpp::get_logger("RasclHardwareInterface"),
    "CSP IPC send failed: %s",
    sent < 0 ? std::strerror(errno) : "short datagram");
  return false;
}

bool RasclHardwareInterface::receive_feedback(
  bool wait_for_packet,
  std::chrono::milliseconds timeout)
{
  if (socket_fd_ < 0)
  {
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  bool received_any = false;

  while (true)
  {
    FeedbackPacket packet{};
    const ssize_t received = ::recv(socket_fd_, &packet, sizeof(packet), 0);
    if (received == static_cast<ssize_t>(sizeof(packet)))
    {
      if (parse_feedback(packet))
      {
        received_any = true;
      }
      // Drain to the latest feedback packet.
      continue;
    }

    if (received > 0)
    {
      RCLCPP_WARN(
        rclcpp::get_logger("RasclHardwareInterface"),
        "Ignored CSP feedback datagram with size %zd; expected %zu",
        received,
        sizeof(packet));
      continue;
    }

    if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("RasclHardwareInterface"),
        "CSP feedback receive failed: %s",
        std::strerror(errno));
      return received_any;
    }

    if (received_any || !wait_for_packet ||
        std::chrono::steady_clock::now() >= deadline)
    {
      return received_any;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

bool RasclHardwareInterface::parse_feedback(const FeedbackPacket & packet)
{
  if (std::memcmp(packet.magic, kFeedbackMagic, sizeof(packet.magic)) != 0 ||
      packet.version != kIpcVersion)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("RasclHardwareInterface"),
      "Ignored CSP feedback with invalid magic/version");
    return false;
  }

  if (have_feedback_ && packet.sequence < feedback_sequence_)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("RasclHardwareInterface"),
      "Ignored out-of-order CSP feedback sequence %lu after %lu",
      static_cast<unsigned long>(packet.sequence),
      static_cast<unsigned long>(feedback_sequence_));
    return false;
  }

  for (std::size_t index = 0; index < kExpectedJointCount; ++index)
  {
    if (!std::isfinite(packet.positions[index]))
    {
      RCLCPP_WARN(
        rclcpp::get_logger("RasclHardwareInterface"),
        "Ignored CSP feedback with non-finite position at index %zu",
        index);
      return false;
    }
  }

  feedback_sequence_ = packet.sequence;
  bridge_state_ = packet.bridge_state;
  working_counter_ = packet.working_counter;
  error_flags_ = packet.error_flags;

  for (std::size_t index = 0; index < kExpectedJointCount; ++index)
  {
    hw_positions_[index] = packet.positions[index];
    statuswords_[index] = packet.statuswords[index];
  }

  have_feedback_ = true;
  last_feedback_time_ = std::chrono::steady_clock::now();
  return true;
}

bool RasclHardwareInterface::feedback_is_stale() const
{
  if (!have_feedback_)
  {
    return true;
  }
  const auto age = std::chrono::steady_clock::now() - last_feedback_time_;
  return age > std::chrono::milliseconds(feedback_timeout_ms_);
}

bool RasclHardwareInterface::has_hard_bridge_error() const
{
  // WKC, CSP bit-12 and timing transients are debounced in the Python
  // bridge; they become hard when the bridge enters kBridgeStateFault.
  constexpr std::uint32_t hard_errors =
    kErrorDriveFault |
    kErrorInternalLimit |
    kErrorCommandWatchdog |
    kErrorInvalidCommand |
    kErrorEthercatState |
    kErrorTracking;
  return (error_flags_ & hard_errors) != 0U;
}

std::uint64_t RasclHardwareInterface::steady_time_ns() const
{
  return static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

}  // namespace rascl_robot_control

PLUGINLIB_EXPORT_CLASS(
  rascl_robot_control::RasclHardwareInterface,
  hardware_interface::SystemInterface)
