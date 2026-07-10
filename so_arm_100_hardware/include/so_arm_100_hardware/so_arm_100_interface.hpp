#ifndef SOARM100_INTERFACE_H
#define SOARM100_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/macros.hpp"
#include <hardware_interface/system_interface.hpp>

#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <vector>
#include <string>
#include <memory>
#include <termios.h>
#include <map>
#include <atomic>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <SCServo_Linux/SCServo.h>
#include "std_srvs/srv/trigger.hpp"
#include <yaml-cpp/yaml.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>

namespace so_arm_100_controller
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SOARM100Interface : public hardware_interface::SystemInterface
{
public:
  SOARM100Interface();
  virtual ~SOARM100Interface();

  // LifecycleNodeInterface
  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  // SystemInterface
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Command mode switching
  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string> & start_interfaces,
      const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string> & start_interfaces,
      const std::vector<std::string> & stop_interfaces) override;

private:
  // Command and state storage for all joints
  std::vector<double> position_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_commands_;
  std::vector<double> velocity_states_;
  std::vector<double> effort_commands_;
  std::vector<double> effort_states_;

  // Active control mode per joint (0=position, 1=velocity, 2=effort/PWM)
  std::vector<uint8_t> active_control_mode_;

  // Keep these until we fully transition to calibration
  std::vector<int> zero_positions_{2048, 2048, 2048, 2048, 2048, 2048};  // Center positions
  std::vector<int> servo_directions_{1, 1, 1, 1, 1, 1};  // Direction multipliers

  // Calibration data
  struct JointCalibration {
    int zero_ticks{2048};  // tick value that maps to 0 radians
    int direction{1};      // +1 or -1 depending on servo mounting orientation
    // Feedforward scale: delta_cmd (rad) = gravity_coefficient * gravity_torque (Nm),
    // where gravity_torque comes from KDL::ChainDynParam::JntToGravity() computed
    // over the arm's full kinematic chain (see kdl_chain_ below) — so a joint's
    // compensation correctly accounts for the load imposed by every joint further
    // out on the chain, not just its own angle. Falls back to a simpler
    // gravity_coefficient * sin(q_target) approximation for any joint not part of
    // that chain (currently just Gripper) or before the URDF has been received.
    // Units changed when KDL support was added: previously "radians per unit
    // sin()", now "radians per Nm" — old tuned values are not directly reusable.
    double gravity_coefficient{0.0};
    // Integral gain (1/s) for the closed-loop steady-state trim — see
    // live_integral_gain_ below. Unlike gravity_coefficient, this needs no
    // physics model: it directly measures (target - actual) from the real
    // encoder and grows a correction until that error disappears, so it
    // adapts automatically to whatever the true disturbance is at any pose.
    double integral_gain{0.0};
    // legacy fields kept for load_calibration backward-compat
    int min_ticks{0};
    int center_ticks{2048};
    int max_ticks{4095};
    double range_ticks{4095};
  };
  std::map<std::string, JointCalibration> joint_calibration_;

  // Live-tunable per-joint corrections, exposed as ROS2 parameters on the
  // so_arm_100_driver node (e.g. via `ros2 param set` or rqt_reconfigure).
  // Changing these takes effect on the next control cycle — no rebuild or
  // relaunch needed. Seeded from calibration.yaml at activation, then
  // overridable live. Indexed by servo_idx; std::atomic so the parameter
  // callback (runs on node_'s executor thread) and write()/read() (run on
  // the realtime control loop thread) can't race.
  //   gravity_coefficient : see JointCalibration::gravity_coefficient above.
  //   zero_trim_rad       : radians added to the reported/commanded angle —
  //                         a live "nudge" for fixing zero-point drift
  //                         without touching zero_ticks and rebuilding.
  std::vector<std::atomic<double>> live_gravity_coeff_;
  std::vector<std::atomic<double>> live_zero_trim_rad_;
  std::vector<std::atomic<double>> live_integral_gain_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
      const std::vector<rclcpp::Parameter> & parameters);

  // Closed-loop steady-state trim (an integral-only outer loop around the
  // servo's own internal position control). Every write() cycle:
  //   error = position_commands_[i] - position_states_[i]   (target - actual)
  //   if |error| < kIntegralDeadband:        // "close enough" — any remaining
  //       integral_trim_[i] += integral_gain * error * dt   // error is steady-
  //       clamp to +/- kIntegralTrimClamp    // state (sag/friction/etc), not
  //   cmd += integral_trim_[i]               // just trajectory-tracking lag
  // Deliberately frozen (not updated) outside the deadband: during active
  // fast motion, tracking error is large but expected and NOT what this is
  // meant to correct — integrating it there would fight the servo's own
  // transient response and reproduce the jerk this replaces. Only touched
  // from write() (the realtime control thread), so plain doubles, no atomics.
  std::vector<double> integral_trim_;
  static constexpr double kIntegralDeadband = 0.3;   // rad
  static constexpr double kIntegralTrimClamp = 0.5;  // rad

  // Configuration-aware gravity compensation. Built once from the full robot
  // URDF (received on /robot_description, published latched by
  // robot_state_publisher) rather than just the <ros2_control> hardware
  // block, since computing gravity torque needs every link's mass/inertia.
  // kdl_ready_ is set exactly once, after kdl_chain_/kdl_dyn_param_ are fully
  // constructed; write() checks it before touching either, so no separate
  // lock is needed for that read/publish handoff between the executor thread
  // (subscription callback) and the realtime control thread (write()).
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  void robot_description_callback(const std_msgs::msg::String::SharedPtr msg);
  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainDynParam> kdl_dyn_param_;
  // joint_idx_to_kdl_idx_[i] = index of info_.joints[i] within the KDL chain's
  // joint array, or -1 if that joint isn't part of the chain (e.g. Gripper).
  std::vector<int> joint_idx_to_kdl_idx_;
  std::atomic<bool> kdl_ready_{false};

  // Communication configuration
  bool use_serial_;
  std::string serial_port_;
  int serial_baudrate_;

  // Servo speed in ticks per second
  u16 servo_speed_;

  // Servo acceleration in ticks per second per second
  u8 servo_acceleration_;

  // Default control mode (0=position, 1=velocity, 2=effort/PWM)
  uint8_t default_control_mode_;

  // Servo position offsets in ticks 
  std::vector<int> servo_position_offsets_;

  // Serial communication
  int SerialPort;
  struct termios tty;
  int WriteToSerial(const unsigned char* buf, int nBytes);
  int ReadSerial(unsigned char* buf, int nBytes);
  bool ConfigureSerialPort();

  // ROS interfaces
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_subscriber_;

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;

  // Store last received feedback message
  sensor_msgs::msg::JointState::SharedPtr last_feedback_msg_;
  std::mutex feedback_mutex_;

  SMS_STS st3215_;

  void feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // Calibration methods
  void calibrate_servo(uint8_t servo_id, int current_pos);
  double ticks_to_radians(int ticks, size_t servo_idx);
  int radians_to_ticks(double radians, size_t servo_idx);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calib_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr torque_service_;

  void calibration_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  
  void torque_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void record_current_position();
  void set_torque_enable(bool enable);

  std::string last_calibration_data_;
  bool torque_enabled_{true};

  bool load_calibration(const std::string& filepath);
  double normalize_position(const std::string& joint_name, int ticks);
};

}  // namespace so_arm_100_controller

#endif  // SOARM100_INTERFACE_H
