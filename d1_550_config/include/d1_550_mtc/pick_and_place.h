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

class PickAndPlace
{
public:
  explicit PickAndPlace(const rclcpp::Node::SharedPtr& node);

  void setupPlanningScene();
  bool doPickAndPlaceTask();

private:
  mtc::Task createPickAndPlaceTask();

  rclcpp::Node::SharedPtr node_;
  mtc::Task task_;
};