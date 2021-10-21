/*
 * traversability_estimation_node.cpp
 *
 *  Created on: Oct 22, 2014
 *      Author: Ralf Kaestner, Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "traversability_estimation/TraversabilityEstimation.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  rclcpp::NodeOptions options;
  auto node = std::make_shared<traversability_estimation::TraversabilityEstimation>(options);

  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
