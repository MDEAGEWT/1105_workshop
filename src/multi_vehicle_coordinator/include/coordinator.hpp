/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

#include <map>
#include <vector>
#include <thread>
#include <atomic>

class MultiVehicleCoordinator : public rclcpp::Node
{
public:
  explicit MultiVehicleCoordinator(const std::vector<int> & vehicle_ids)
  : Node("multi_vehicle_coordinator"),
    _vehicle_ids(vehicle_ids),
    _trigger_sent(false),
    _all_vehicles_ready(false)
  {
    // Initialize vehicle status tracking
    for (int id : _vehicle_ids) {
      _vehicle_status[id] = VehicleState{};
    }

    // Create subscriptions for each vehicle's status
    for (int id : _vehicle_ids) {
      // VehicleStatus subscription
      auto status_callback = [this, id](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        vehicleStatusCallback(id, msg);
      };
      std::string status_topic = "vehicle" + std::to_string(id) + "/fmu/out/vehicle_status_v1";
      auto status_sub = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        status_topic, rclcpp::SensorDataQoS(), status_callback);
      _vehicle_status_subs.push_back(status_sub);

      // VehicleLandDetected subscription
      auto land_callback = [this, id](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
        vehicleLandDetectedCallback(id, msg);
      };
      std::string land_topic = "vehicle" + std::to_string(id) + "/fmu/out/vehicle_land_detected";
      auto land_sub = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
        land_topic, rclcpp::SensorDataQoS(), land_callback);
      _vehicle_land_detected_subs.push_back(land_sub);
    }

    // Create trigger publisher
    _trigger_pub = this->create_publisher<std_msgs::msg::Empty>("/mission_trigger", 10);

    // Create timer for periodic status checks
    _status_timer = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MultiVehicleCoordinator::checkAllVehiclesStatus, this));

    // Start input thread
    _input_thread = std::thread(&MultiVehicleCoordinator::inputThreadFunction, this);

    RCLCPP_INFO(this->get_logger(), "Multi-vehicle coordinator started, monitoring %zu vehicles",
                _vehicle_ids.size());
  }

  ~MultiVehicleCoordinator()
  {
    _shutdown = true;
    if (_input_thread.joinable()) {
      _input_thread.join();
    }
  }

private:
  struct VehicleState
  {
    bool status_received{false};
    bool land_detected_received{false};
    bool in_air{false};
    uint8_t arming_state{0};
    uint8_t nav_state{0};
    bool land_detected_landed{false};
    std::chrono::steady_clock::time_point last_update;
  };

  void vehicleStatusCallback(int vehicle_id, const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    auto & state = _vehicle_status[vehicle_id];
    state.status_received = true;
    state.arming_state = msg->arming_state;
    state.nav_state = msg->nav_state;

    updateInAirStatus(state);
    state.last_update = std::chrono::steady_clock::now();
  }

  void vehicleLandDetectedCallback(int vehicle_id, const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
  {
    auto & state = _vehicle_status[vehicle_id];
    state.land_detected_received = true;
    state.land_detected_landed = msg->landed;

    updateInAirStatus(state);
    state.last_update = std::chrono::steady_clock::now();
  }

  void updateInAirStatus(VehicleState & state)
  {
    // Check if vehicle is in air and hovering
    // Must be: ARMED + NOT landed + (LOITER or POSCTL or ALTCTL)
    state.in_air = state.status_received &&
                   state.land_detected_received &&
                   (state.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) &&
                   (!state.land_detected_landed) &&  // NOT landed (according to land detector)
                   (state.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER ||
                    state.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL ||
                    state.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ALTCTL);
  }

  void checkAllVehiclesStatus()
  {
    if (_trigger_sent) {
      return; // Already sent trigger
    }

    bool all_ready = true;
    bool all_received = true;

    for (const auto & [id, state] : _vehicle_status) {
      if (!state.status_received || !state.land_detected_received) {
        all_received = false;
        all_ready = false;
        break;
      }

      // Check if vehicle is hovering (in air and loitering)
      if (!state.in_air) {
        all_ready = false;
      }
    }

    if (all_received && !_vehicle_status.empty()) {
      if (all_ready && !_all_vehicles_ready) {
        _all_vehicles_ready = true;
        RCLCPP_INFO(this->get_logger(),
                    "\n========================================\n"
                    "All %zu vehicles are in the air and hovering!\n"
                    "Press ENTER to start mission...\n"
                    "========================================",
                    _vehicle_ids.size());
      } else if (!all_ready && _all_vehicles_ready) {
        // Reset if vehicles are no longer all ready
        _all_vehicles_ready = false;
      }

      // Log vehicle states periodically (every 2 seconds)
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - _last_log_time).count() >= 2) {
        logVehicleStates();
        _last_log_time = now;
      }
    }
  }

  void logVehicleStates()
  {
    int ready_count = 0;
    int total_count = _vehicle_status.size();

    RCLCPP_INFO(this->get_logger(), "Vehicle Status:");
    for (const auto & [id, state] : _vehicle_status) {
      if (state.status_received && state.land_detected_received) {
        const char* status = state.in_air ? "[HOVERING]" : "[WAITING]";
        const char* land_status = state.land_detected_landed ? "LANDED" : "IN_AIR";

        if (state.in_air) ready_count++;

        RCLCPP_INFO(this->get_logger(),
                    "  Vehicle %d: %s (arming=%u, land_det=%s, nav=%u) %s",
                    id, status, state.arming_state, land_status, state.nav_state,
                    state.in_air ? "✓ READY" : "✗ NOT READY");
      } else {
        RCLCPP_INFO(this->get_logger(), "  Vehicle %d: [NO DATA] (status_rx=%d, land_rx=%d)",
                    id, state.status_received, state.land_detected_received);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Ready: %d/%d vehicles", ready_count, total_count);

    if (ready_count == total_count && total_count > 0) {
      RCLCPP_WARN(this->get_logger(), "All vehicles ready but waiting for coordinator to trigger!");
    }
  }

  void inputThreadFunction()
  {
    RCLCPP_INFO(this->get_logger(), "Input thread started. Waiting for all vehicles...");

    while (!_shutdown && rclcpp::ok()) {
      // Wait for all vehicles to be ready
      if (_all_vehicles_ready && !_trigger_sent) {
        // Wait for Enter key
        std::string line;
        std::getline(std::cin, line);

        if (!_trigger_sent) {
          sendTrigger();
        }
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void sendTrigger()
  {
    RCLCPP_INFO(this->get_logger(),
                "\n========================================\n"
                "MISSION TRIGGER SENT!\n"
                "All vehicles starting custom modes...\n"
                "========================================");

    std_msgs::msg::Empty trigger_msg;
    _trigger_pub->publish(trigger_msg);
    _trigger_sent = true;
  }

  std::vector<int> _vehicle_ids;
  std::map<int, VehicleState> _vehicle_status;
  std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr> _vehicle_status_subs;
  std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr> _vehicle_land_detected_subs;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _trigger_pub;
  rclcpp::TimerBase::SharedPtr _status_timer;

  std::thread _input_thread;
  std::atomic<bool> _shutdown{false};
  std::atomic<bool> _trigger_sent{false};
  std::atomic<bool> _all_vehicles_ready{false};
  std::chrono::steady_clock::time_point _last_log_time{std::chrono::steady_clock::now()};
};
