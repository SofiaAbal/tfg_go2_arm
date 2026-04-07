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
  // pick params
  double pick_x, pick_y, pick_z;
  std::string shape;
  double dimension_x, dimension_y = 0.0, dimension_z = 0.0;
  double rot_x = 0.0, rot_y = 0.0, rot_z = 0.0; // rad

  // place params
  double place_x, place_y, place_z;
};

class PickAndPlace
{
public:
  explicit PickAndPlace(const rclcpp::Node::SharedPtr& node);

  void setupPlanningScene(const ObjectParams& params);
  bool doPickAndPlaceTask(const ObjectParams& params);
  bool doPickTask(const ObjectParams& params);
  bool doPlaceTask(const ObjectParams& params);
  bool hasObject() const { return has_object_; }

private:
  mtc::Task createPickAndPlaceTask(const ObjectParams& params);
  mtc::Task createPickTask(const ObjectParams& params);
  mtc::Task createPlaceTask(const ObjectParams& params);

  rclcpp::Node::SharedPtr node_;
  mtc::Task task_;
  bool has_object_ = false;
};