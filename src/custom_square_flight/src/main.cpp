/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "rclcpp/rclcpp.hpp"

#include <mode.hpp>

static const std::string kNodeName = "custom_square_flight";
static const bool kEnableDebugOutput = true;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(kNodeName);

  // Create mode and executor
  RCLCPP_INFO(node->get_logger(), "Creating SquareFlightMode...");
  auto mode = std::make_shared<SquareFlightMode>(*node);

  RCLCPP_INFO(node->get_logger(), "Creating SquareFlightExecutor with ActivateImmediately...");
  auto executor = std::make_shared<SquareFlightExecutor>(*node, *mode, true);

  // Register executor only (it will register the mode automatically)
  RCLCPP_INFO(node->get_logger(), "Registering executor...");
  executor->doRegister();

  RCLCPP_INFO(node->get_logger(), "Square flight node started with executor - waiting for activation...");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
