/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "../include/coordinator.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Vehicle IDs: 4 (Lift&Cruise), 3, 11, 12, 13 (Drones)
  std::vector<int> vehicle_ids = {3, 4, 11, 12, 13};

  auto coordinator = std::make_shared<MultiVehicleCoordinator>(vehicle_ids);

  RCLCPP_INFO(coordinator->get_logger(),
              "Multi-vehicle coordinator initialized for vehicles: 3, 4, 11, 12, 13");

  rclcpp::spin(coordinator);
  rclcpp::shutdown();

  return 0;
}
