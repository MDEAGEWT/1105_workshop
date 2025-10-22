/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/geometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>

using namespace px4_ros2::literals; // NOLINT

class SquareFlightMode : public px4_ros2::ModeBase
{
public:
  explicit SquareFlightMode(rclcpp::Node & node)
  : ModeBase(node, getModeName(node), getTopicNamespacePrefix(node))
  {
    // Declare and get parameters (only if not already declared)
    if (!node.has_parameter("reverse_direction")) {
      node.declare_parameter<bool>("reverse_direction", false);
    }
    if (!node.has_parameter("square_size")) {
      node.declare_parameter<double>("square_size", 100.0);
    }

    _reverse_direction = node.get_parameter("reverse_direction").as_bool();
    _square_size = static_cast<float>(node.get_parameter("square_size").as_double());

    _goto_setpoint = std::make_shared<px4_ros2::GotoSetpointType>(*this);

    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
  }

  void onActivate() override
  {
    _state = State::SettlingAtStart;
    _start_position_set = false;
    _first_waypoint_set = false;
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
            // Start from different direction based on reverse_direction
            _state = _reverse_direction ? State::GoingWest : State::GoingNorth;
          }
        }
        break;

      case State::GoingNorth: {
          // Go North: +X direction
          // Normal: (0,0) → (100,0), Reverse: (0,-100) → (100,-100)
          const float y_offset = _reverse_direction ? -_square_size : 0.f;
          const Eigen::Vector3f target_position_m = _start_position_m +
            Eigen::Vector3f{_square_size, y_offset, 0.f};

          const Eigen::Vector2f vehicle_to_target_xy = target_position_m.head(2) -
            _vehicle_local_position->positionNed().head(2);
          const float heading_target_rad = atan2f(vehicle_to_target_xy(1), vehicle_to_target_xy(0));

          if (vehicle_to_target_xy.norm() < 0.1f) {
            _goto_setpoint->update(target_position_m);
          } else {
            _goto_setpoint->update(target_position_m, heading_target_rad);
          }

          if (positionReached(target_position_m)) {
            _state = _reverse_direction ? State::GoingEast : State::GoingWest;
          }
        }
        break;

      case State::GoingWest: {
          // Go West: -Y direction
          // Normal: (100,0) → (100,-100), Reverse: (0,0) → (0,-100)
          const float x_offset = _reverse_direction ? 0.f : _square_size;
          const Eigen::Vector3f target_position_m = _start_position_m +
            Eigen::Vector3f{x_offset, -_square_size, 0.f};

          const Eigen::Vector2f vehicle_to_target_xy = target_position_m.head(2) -
            _vehicle_local_position->positionNed().head(2);
          const float heading_target_rad = atan2f(vehicle_to_target_xy(1), vehicle_to_target_xy(0));

          if (vehicle_to_target_xy.norm() < 0.1f) {
            _goto_setpoint->update(target_position_m);
          } else {
            _goto_setpoint->update(target_position_m, heading_target_rad);
          }

          if (positionReached(target_position_m)) {
            // Normal: West → South, Reverse: West → North
            _state = _reverse_direction ? State::GoingNorth : State::GoingSouth;
          }
        }
        break;

      case State::GoingEast: {
          // Go East: +Y direction
          // Normal: (0,-100) → (0,0), Reverse: (100,-100) → (100,0)
          const float x_offset = _reverse_direction ? _square_size : 0.f;
          const Eigen::Vector3f target_position_m = _start_position_m +
            Eigen::Vector3f{x_offset, 0.f, 0.f};

          const Eigen::Vector2f vehicle_to_target_xy = target_position_m.head(2) -
            _vehicle_local_position->positionNed().head(2);
          const float heading_target_rad = atan2f(vehicle_to_target_xy(1), vehicle_to_target_xy(0));

          if (vehicle_to_target_xy.norm() < 0.1f) {
            _goto_setpoint->update(target_position_m);
          } else {
            _goto_setpoint->update(target_position_m, heading_target_rad);
          }

          if (positionReached(target_position_m)) {
            // Normal: East → North (repeat), Reverse: East → South
            _state = _reverse_direction ? State::GoingSouth : State::GoingNorth;
          }
        }
        break;

      case State::GoingSouth: {
          // Go South: -X direction
          // Normal: (100,-100) → (0,-100), Reverse: (100,0) → (0,0)
          const float y_offset = _reverse_direction ? 0.f : -_square_size;
          const Eigen::Vector3f target_position_m = _start_position_m +
            Eigen::Vector3f{0.f, y_offset, 0.f};

          const Eigen::Vector2f vehicle_to_target_xy = target_position_m.head(2) -
            _vehicle_local_position->positionNed().head(2);
          const float heading_target_rad = atan2f(vehicle_to_target_xy(1), vehicle_to_target_xy(0));

          if (vehicle_to_target_xy.norm() < 0.1f) {
            _goto_setpoint->update(target_position_m);
          } else {
            _goto_setpoint->update(target_position_m, heading_target_rad);
          }

          if (positionReached(target_position_m)) {
            // Normal: South → East, Reverse: South → West (repeat)
            _state = _reverse_direction ? State::GoingWest : State::GoingEast;
          }
        }
        break;

      case State::GoingBackToStart: {
          // Return to start position (only used for normal direction)
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
            // Repeat the square pattern infinitely
            _state = State::GoingNorth;
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
    GoingNorth,
    GoingWest,
    GoingEast,
    GoingSouth,
    GoingBackToStart
  } _state;

  // Parameters
  bool _reverse_direction;
  float _square_size;

  // NED earth-fixed frame. Starting corner position
  Eigen::Vector3f _start_position_m;
  bool _start_position_set{false};
  bool _first_waypoint_set{false};

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
