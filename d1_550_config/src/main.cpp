#include "d1_550_mtc/pick_and_place.h"

#include <rclcpp/rclcpp.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("main");

class PickAndPlace;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "pick_and_place_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&]() { executor.spin(); });

  PickAndPlace app(node);

  app.setupPlanningScene();

  if (!app.doPickAndPlaceTask())
    RCLCPP_ERROR(node->get_logger(), "Task failed");

  spinner.join();
  rclcpp::shutdown();
  return 0;
}