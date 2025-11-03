/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/geometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <Eigen/Core>

using namespace px4_ros2::literals; // NOLINT

class LineFlightMode : public px4_ros2::ModeBase
{
public:
  explicit LineFlightMode(rclcpp::Node & node)
  : ModeBase(node, getModeName(node), getTopicNamespacePrefix(node))
  {
    // Declare and get parameters (only if not already declared)
    if (!node.has_parameter("line_distance")) {
      node.declare_parameter<double>("line_distance", 100.0);
    }

    _line_distance = static_cast<float>(node.get_parameter("line_distance").as_double());

    _goto_setpoint = std::make_shared<px4_ros2::GotoSetpointType>(*this);

    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
  }

  void onActivate() override
  {
    _state = State::SettlingAtStart;
    _start_position_set = false;
    _wait_start_time = std::chrono::steady_clock::now();
  }

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    if (!_start_position_set) {
      _start_position_m = _vehicle_local_position->positionNed();
      _start_position_set = true;
    }

    switch (_state) {
      case State::SettlingAtStart: {
          // Settle at the starting position
          _goto_setpoint->update(_start_position_m);
          if (positionReached(_start_position_m)) {
            _state = State::GoingWest;
          }
        }
        break;

      case State::GoingWest: {
          // Go West: -Y direction
          const Eigen::Vector3f target_position_m = _start_position_m +
            Eigen::Vector3f{0.f, -_line_distance, 0.f};

          const Eigen::Vector2f vehicle_to_target_xy = target_position_m.head(2) -
            _vehicle_local_position->positionNed().head(2);
          const float heading_target_rad = atan2f(vehicle_to_target_xy(1), vehicle_to_target_xy(0));

          if (vehicle_to_target_xy.norm() < 0.1f) {
            _goto_setpoint->update(target_position_m);
          } else {
            _goto_setpoint->update(target_position_m, heading_target_rad);
          }

          if (positionReached(target_position_m)) {
            _state = State::WaitingWest;
            _wait_start_time = std::chrono::steady_clock::now();
          }
        }
        break;

      case State::WaitingWest: {
          // Wait for 3 seconds at West position
          const Eigen::Vector3f target_position_m = _start_position_m +
            Eigen::Vector3f{0.f, -_line_distance, 0.f};
          _goto_setpoint->update(target_position_m);

          auto now = std::chrono::steady_clock::now();
          auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - _wait_start_time);
          if (elapsed.count() >= 3000) {
            _state = State::GoingEast;
          }
        }
        break;

      case State::GoingEast: {
          // Go East: back to start position (+Y direction)
          const Eigen::Vector3f target_position_m = _start_position_m;

          const Eigen::Vector2f vehicle_to_target_xy = target_position_m.head(2) -
            _vehicle_local_position->positionNed().head(2);
          const float heading_target_rad = atan2f(vehicle_to_target_xy(1), vehicle_to_target_xy(0));

          if (vehicle_to_target_xy.norm() < 0.1f) {
            _goto_setpoint->update(target_position_m);
          } else {
            _goto_setpoint->update(target_position_m, heading_target_rad);
          }

          if (positionReached(target_position_m)) {
            _state = State::WaitingEast;
            _wait_start_time = std::chrono::steady_clock::now();
          }
        }
        break;

      case State::WaitingEast: {
          // Wait for 3 seconds at East position (start position)
          _goto_setpoint->update(_start_position_m);

          auto now = std::chrono::steady_clock::now();
          auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - _wait_start_time);
          if (elapsed.count() >= 3000) {
            // Repeat the pattern infinitely
            _state = State::GoingWest;
          }
        }
        break;
    }
  }

private:
  static std::string getModeName(rclcpp::Node & node)
  {
    // Declare px4_sysid parameter if not already declared
    if (!node.has_parameter("px4_sysid")) {
      node.declare_parameter<int>("px4_sysid", 1);
    }
    int sysid = node.get_parameter("px4_sysid").as_int();
    return "Custom Mode " + std::to_string(sysid);
  }

  static std::string getTopicNamespacePrefix(rclcpp::Node & node)
  {
    // Declare px4_sysid parameter if not already declared
    if (!node.has_parameter("px4_sysid")) {
      node.declare_parameter<int>("px4_sysid", 1);
    }
    int sysid = node.get_parameter("px4_sysid").as_int();
    return "vehicle" + std::to_string(sysid) + "/";
  }

  enum class State
  {
    SettlingAtStart = 0,
    GoingWest,
    WaitingWest,
    GoingEast,
    WaitingEast
  } _state;

  // Parameters
  float _line_distance;

  // NED earth-fixed frame. Starting position
  Eigen::Vector3f _start_position_m;
  bool _start_position_set{false};

  // Wait time tracking
  std::chrono::time_point<std::chrono::steady_clock> _wait_start_time;

  std::shared_ptr<px4_ros2::GotoSetpointType> _goto_setpoint;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;

  bool positionReached(const Eigen::Vector3f & target_position_m) const
  {
    static constexpr float kPositionErrorThreshold = 0.5f; // [m]
    static constexpr float kVelocityErrorThreshold = 0.3f; // [m/s]
    const Eigen::Vector3f position_error_m = target_position_m -
      _vehicle_local_position->positionNed();
    return (position_error_m.norm() < kPositionErrorThreshold) &&
           (_vehicle_local_position->velocityNed().norm() < kVelocityErrorThreshold);
  }
};

// Executor for managing arm, takeoff, and mode switching
class LineFlightExecutor : public px4_ros2::ModeExecutorBase
{
public:
  explicit LineFlightExecutor(
    rclcpp::Node & node,
    LineFlightMode & owned_mode,
    bool activate_immediately = true)
  : ModeExecutorBase(
      node,
      px4_ros2::ModeExecutorBase::Settings{
        activate_immediately ?
          px4_ros2::ModeExecutorBase::Settings::Activation::ActivateImmediately :
          px4_ros2::ModeExecutorBase::Settings::Activation::ActivateAlways
      },
      owned_mode,
      getTopicNamespacePrefix(node)),
    _node(node)
  {
    RCLCPP_INFO(_node.get_logger(), "LineFlightExecutor constructor - topic namespace: %s",
                getTopicNamespacePrefix(node).c_str());

    // Get takeoff altitude parameter (relative altitude)
    if (!node.has_parameter("takeoff_altitude")) {
      node.declare_parameter<double>("takeoff_altitude", 33.0);
    }
    _takeoff_altitude_relative = static_cast<float>(node.get_parameter("takeoff_altitude").as_double());

    // Subscribe to global position
    std::string global_pos_topic = getTopicNamespacePrefix(node) + "fmu/out/vehicle_global_position";
    _global_position_sub = _node.create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      global_pos_topic, rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
        _current_altitude_amsl = msg->alt;
        _global_position_received = true;
      });

    RCLCPP_INFO(_node.get_logger(), "LineFlightExecutor constructor complete - relative altitude: %.1fm, activation: %s",
                _takeoff_altitude_relative, activate_immediately ? "ActivateImmediately" : "ActivateAlways");
  }

  void onActivate() override
  {
    RCLCPP_INFO(_node.get_logger(), "LineFlightExecutor activated, starting sequence...");
    runState(State::WaitForArming, px4_ros2::Result::Success);
  }

  void onDeactivate(DeactivateReason reason) override
  {
    RCLCPP_INFO(_node.get_logger(), "LineFlightExecutor deactivated");
    _trigger_sub.reset();
  }

private:
  enum class State
  {
    WaitForArming,
    Arming,
    TakingOff,
    SwitchingToHold,
    WaitingForTrigger,
    RunningCustomMode
  };

  void runState(State state, px4_ros2::Result previous_result)
  {
    if (previous_result != px4_ros2::Result::Success) {
      RCLCPP_ERROR(_node.get_logger(), "State failed: %s, aborting sequence",
                   resultToString(previous_result));
      return;
    }

    switch (state) {
      case State::WaitForArming:
        RCLCPP_INFO(_node.get_logger(), "Waiting until ready to arm...");
        waitReadyToArm([this](px4_ros2::Result result) {
          runState(State::Arming, result);
        });
        break;

      case State::Arming:
        RCLCPP_INFO(_node.get_logger(), "Arming...");
        arm([this](px4_ros2::Result result) {
          runState(State::TakingOff, result);
        });
        break;

      case State::TakingOff: {
        if (!_global_position_received) {
          RCLCPP_WARN(_node.get_logger(), "Waiting for global position...");
          // Retry after a short delay
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          runState(State::TakingOff, px4_ros2::Result::Success);
          return;
        }

        float target_altitude_amsl = _current_altitude_amsl + _takeoff_altitude_relative;
        RCLCPP_INFO(_node.get_logger(), "Taking off: current %.1fm AMSL + %.1fm relative = %.1fm AMSL",
                    _current_altitude_amsl, _takeoff_altitude_relative, target_altitude_amsl);
        takeoff([this](px4_ros2::Result result) {
          runState(State::SwitchingToHold, result);
        }, target_altitude_amsl);
        break;
      }

      case State::SwitchingToHold: {
        RCLCPP_INFO(_node.get_logger(), "Switching to HOLD mode...");
        // Send HOLD command directly
        auto result = sendCommandSync(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE,
          4.0f,  // param1: 4 = HOLD
          NAN, NAN, NAN, NAN, NAN, NAN);

        RCLCPP_INFO(_node.get_logger(), "HOLD mode command result: %s", resultToString(result));
        runState(State::WaitingForTrigger, result);
        break;
      }

      case State::WaitingForTrigger:
        RCLCPP_INFO(_node.get_logger(), "Hovering, waiting for mission trigger...");
        // Subscribe to trigger topic
        _trigger_sub = _node.create_subscription<std_msgs::msg::Empty>(
          "/mission_trigger", 10,
          [this](const std_msgs::msg::Empty::SharedPtr) {
            RCLCPP_INFO(_node.get_logger(), "Trigger received! Starting custom mode...");
            _trigger_sub.reset(); // Unsubscribe after receiving trigger
            scheduleMode(
              ownedMode().id(),
              [this](px4_ros2::Result result) {
                runState(State::RunningCustomMode, result);
              });
          });
        break;

      case State::RunningCustomMode:
        RCLCPP_INFO(_node.get_logger(), "Custom line flight mode is now active");
        // Mode will run until manually deactivated
        break;
    }
  }

  static std::string getTopicNamespacePrefix(rclcpp::Node & node)
  {
    if (!node.has_parameter("px4_sysid")) {
      node.declare_parameter<int>("px4_sysid", 1);
    }
    int sysid = node.get_parameter("px4_sysid").as_int();
    return "vehicle" + std::to_string(sysid) + "/";
  }

  rclcpp::Node & _node;
  float _takeoff_altitude_relative;
  float _current_altitude_amsl{0.0f};
  bool _global_position_received{false};
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _trigger_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr _global_position_sub;
};
