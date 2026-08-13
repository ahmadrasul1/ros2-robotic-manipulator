#ifndef RASCL_ROBOT_CONTROL__RASCL_HARDWARE_INTERFACE_HPP_
#define RASCL_ROBOT_CONTROL__RASCL_HARDWARE_INTERFACE_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace rascl_robot_control
{

/**
 * @brief ros2_control interface backed by the cyclic PySOEM PDO process.
 *
 * Target joint positions and measured encoder positions are exchanged through
 * a local Unix datagram socket. EtherCAT ownership, PDO exchange and CiA 402
 * status handling remain in the Python bridge process.
 */
class RasclHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RasclHardwareInterface)

  RasclHardwareInterface();
  ~RasclHardwareInterface() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  static constexpr std::size_t kExpectedJointCount = 4;
  static constexpr std::uint32_t kIpcVersion = 1;

  static constexpr std::uint32_t kCommandFlagRegister = 1U << 0;
  static constexpr std::uint32_t kCommandFlagPositionValid = 1U << 1;
  static constexpr std::uint32_t kCommandFlagHalt = 1U << 2;

  static constexpr std::uint32_t kBridgeStateStarting = 0;
  static constexpr std::uint32_t kBridgeStateReady = 1;
  static constexpr std::uint32_t kBridgeStateHold = 2;
  static constexpr std::uint32_t kBridgeStateFault = 3;
  static constexpr std::uint32_t kBridgeStateStopped = 4;

  static constexpr std::uint32_t kErrorWkc = 1U << 0;
  static constexpr std::uint32_t kErrorDriveFault = 1U << 1;
  static constexpr std::uint32_t kErrorNotFollowing = 1U << 2;
  static constexpr std::uint32_t kErrorFollowingError = 1U << 3;
  static constexpr std::uint32_t kErrorInternalLimit = 1U << 4;
  static constexpr std::uint32_t kErrorWarning = 1U << 5;
  static constexpr std::uint32_t kErrorCommandWatchdog = 1U << 6;
  static constexpr std::uint32_t kErrorMotionDisabled = 1U << 7;
  static constexpr std::uint32_t kErrorInvalidCommand = 1U << 8;
  static constexpr std::uint32_t kErrorEthercatState = 1U << 9;
  static constexpr std::uint32_t kErrorTracking = 1U << 10;
  static constexpr std::uint32_t kErrorTiming = 1U << 11;

#pragma pack(push, 1)
  struct CommandPacket
  {
    char magic[4];
    std::uint32_t version;
    std::uint32_t flags;
    std::uint64_t sequence;
    std::uint64_t timestamp_ns;
    double positions[kExpectedJointCount];
  };

  struct FeedbackPacket
  {
    char magic[4];
    std::uint32_t version;
    std::uint64_t sequence;
    std::uint64_t timestamp_ns;
    std::uint32_t bridge_state;
    std::int32_t working_counter;
    double positions[kExpectedJointCount];
    std::uint16_t statuswords[kExpectedJointCount];
    std::uint32_t error_flags;
  };
#pragma pack(pop)

  static_assert(sizeof(CommandPacket) == 60, "Unexpected CSP command-packet size");
  static_assert(sizeof(FeedbackPacket) == 76, "Unexpected CSP feedback-packet size");

  bool open_ipc_socket();
  void close_ipc_socket();
  bool send_command(std::uint32_t flags);
  bool receive_feedback(bool wait_for_packet, std::chrono::milliseconds timeout);
  bool parse_feedback(const FeedbackPacket & packet);
  bool feedback_is_stale() const;
  bool has_hard_bridge_error() const;
  std::uint64_t steady_time_ns() const;

  std::string server_socket_path_;
  std::string client_socket_path_;
  int socket_fd_;

  int feedback_timeout_ms_;
  int startup_feedback_timeout_ms_;

  std::vector<std::string> joint_names_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_commands_;

  std::uint64_t command_sequence_;
  std::uint64_t feedback_sequence_;
  std::uint32_t bridge_state_;
  std::int32_t working_counter_;
  std::array<std::uint16_t, kExpectedJointCount> statuswords_;
  std::uint32_t error_flags_;

  bool have_feedback_;
  bool active_;
  bool logged_soft_error_;
  std::chrono::steady_clock::time_point last_feedback_time_;
};

}  // namespace rascl_robot_control

#endif  // RASCL_ROBOT_CONTROL__RASCL_HARDWARE_INTERFACE_HPP_
