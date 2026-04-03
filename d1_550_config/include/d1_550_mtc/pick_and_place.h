#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#pragma once

namespace mtc = moveit::task_constructor;

struct ObjectParams {
  double pick_x, pick_y, pick_z;
  double place_x, place_y, place_z;
  std::string shape;
  double dimension_x, dimension_y, dimension_z;
};

class PickAndPlace
{
public:
  explicit PickAndPlace(const rclcpp::Node::SharedPtr& node);

  void setupPlanningScene(const ObjectParams& params);
  bool doPickAndPlaceTask(const ObjectParams& params);

private:
  mtc::Task createPickAndPlaceTask(const ObjectParams& params);

  rclcpp::Node::SharedPtr node_;
  mtc::Task task_;
};