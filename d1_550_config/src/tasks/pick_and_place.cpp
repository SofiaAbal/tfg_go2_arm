#include "d1_550_mtc/pick_and_place.h"

namespace mtc = moveit::task_constructor;

PickAndPlace::PickAndPlace(const rclcpp::Node::SharedPtr& node)
: node_(node) {}

void setObjectData(shape_msgs::msg::SolidPrimitive& primitive, const ObjectParams& params)
{
  std::string shape_upper = params.shape;
  std::transform(shape_upper.begin(), shape_upper.end(), shape_upper.begin(), [](unsigned char c){ return std::toupper(c); });

  if (shape_upper == "BOX") {
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = { params.dimension_x, params.dimension_y, params.dimension_z };
  } else if (shape_upper == "CYLINDER") {
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    primitive.dimensions = { params.dimension_x, params.dimension_y };
  } else if (shape_upper == "CONE") {
    primitive.type = shape_msgs::msg::SolidPrimitive::CONE;
    primitive.dimensions = { params.dimension_x, params.dimension_y };
  } else if (shape_upper == "SPHERE") {
    primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    primitive.dimensions = { params.dimension_x };
  } else {
    throw std::runtime_error("La forma '"+params.shape+"' no es valida. Usar 'BOX', 'CYLINDER', 'CONE' o 'SPHERE'.");
  }
}

moveit_msgs::msg::CollisionObject defineObject(const ObjectParams& params)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";

  object.primitives.resize(1);
  setObjectData(object.primitives[0], params);

  geometry_msgs::msg::Pose pose;
  pose.position.x = params.pick_x;
  pose.position.y = params.pick_y;
  pose.position.z = params.pick_z;
  /* pose.orientation.w = 1.0; //id, no rota -> necesito quaternion */

  // documentacion
  /* tf2::Quaternion q_orig, q_rot, q_new;
  q_orig.setRPY(0.0, 0.0, 0.0);
  q_rot.setRPY(params.rot_x, params.rot_y, params.rot_z);
  q_new = q_rot * q_orig;
  q_new.normalize();
  pose.orientation = tf2::toMsg(q_new); */

  tf2::Quaternion q;
  q.setRPY(params.rot_x, params.rot_y, params.rot_z);
  q.normalize();
  pose.orientation = tf2::toMsg(q);

  object.pose = pose;

  return object;
}

moveit_msgs::msg::CollisionObject defineDog() {
  moveit_msgs::msg::CollisionObject object;
  object.id = "dog";
  object.header.frame_id = "world";

  object.primitives.resize(1);
  setObjectData(object.primitives[0], ObjectParams{.shape="BOX", .dimension_x=0.7, .dimension_y=0.3, .dimension_z=0.2});

  geometry_msgs::msg::Pose pose;
  pose.position.x = -0.25;
  pose.position.y = 0;
  pose.position.z = -0.1;
  pose.orientation.w = 1.0;

  object.pose = pose;

  return object;
}

moveit_msgs::msg::CollisionObject defineGround() {
  moveit_msgs::msg::CollisionObject object;
  object.id = "ground";
  object.header.frame_id = "world";

  object.primitives.resize(1);
  setObjectData(object.primitives[0], ObjectParams{.shape="BOX", .dimension_x=2.0, .dimension_y=2.0, .dimension_z=0.01});

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = -0.2;
  pose.orientation.w = 1.0;

  object.pose = pose;

  return object;
}

void PickAndPlace::setupPlanningScene(const ObjectParams& params)
{
  moveit_msgs::msg::CollisionObject object = defineObject(params);
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void defineObstaclesInPlanningScene() {
  moveit_msgs::msg::CollisionObject dog = defineDog();
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(dog);

  moveit_msgs::msg::CollisionObject ground = defineGround();
  psi.applyCollisionObject(ground);
}

bool PickAndPlace::doPickAndPlaceTask(const ObjectParams& params)
{
  task_ = createPickAndPlaceTask(params);

  /* defineObstaclesInPlanningScene(); */

  try {
    task_.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), e);
    return false;
  }

  if (!task_.plan(5)) {
    RCLCPP_ERROR(node_->get_logger(), "Planning failed");
    return false;
  }

  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  return result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
}

bool PickAndPlace::doPickTask(const ObjectParams& params)
{
  task_ = createPickTask(params);

  defineObstaclesInPlanningScene();

  try {
    task_.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), e);
    return false;
  }

  if (!task_.plan(5)) {
    RCLCPP_ERROR(node_->get_logger(), "Planning failed");
    return false;
  }

  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());

  auto success = result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

  if (success) {
    has_object_ = true;
  }

  return success;
}

bool PickAndPlace::doPlaceTask(const ObjectParams& params)
{
  task_ = createPlaceTask(params);

  /* defineObstaclesInPlanningScene(); */

  try {
    task_.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), e);
    return false;
  }

  if (!task_.plan(5)) {
    RCLCPP_ERROR(node_->get_logger(), "Planning failed");
    return false;
  }

  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  auto success = result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  if (success) {
    has_object_ = false;
  }
  
  return success;
}

mtc::Task PickAndPlace::createPickAndPlaceTask(const ObjectParams& params)
{
  mtc::Task task;
  task.stages()->setName("pick and place task");
  task.loadRobotModel(node_);

  const auto& arm_group_name  = "d1_arm";
  const auto& hand_group_name = "d1_gripper";
  const auto& hand_frame      = "Empty_Link6";
  const auto& eef_name        = "gripper";

  task.setProperty("group", arm_group_name);
  task.setProperty("eef", eef_name);
  task.setProperty("ik_frame", hand_frame);

  mtc::Stage* current_state_ptr = nullptr;
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));
/* 
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow collision (object+hand, ground)");
    stage->allowCollisions("object", {"ground"}, true);
    stage->allowCollisions("ground",
        task.getRobotModel()
            ->getJointModelGroup(hand_group_name)
            ->getLinkModelNamesWithCollisionGeometry(),
        true);
    task.add(std::move(stage));
  } */

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.0001);

  // Empezamos con el brazo recodigo
  // auto stage = std::make_unique<mtc::stages::MoveTo>("move to start", interpolation_planner);
  // stage->setGroup(arm_group_name);
  // stage->setGoal("arm_start");
  // task.add(std::move(stage));

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("gripper_open");
  task.add(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });

  stage_move_to_pick->setTimeout(10.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage = nullptr;

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });

    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);

      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.01, 0.10);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.x = 1.0;
      vec.vector.y = 1.0;
      vec.vector.z = 1.0;
      stage->setDirection(vec);

      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("gripper_open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr);

      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      grasp_frame_transform.translation().x() = 0.1;

      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));

      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

      // mas solucions para mas probabilkidades de encontrar una buena
      wrapper->setMaxIKSolutions(50);

      grasp->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow collision (hand,object)");

      stage->allowCollisions(
          "object",
          task.getRobotModel()
              ->getJointModelGroup(hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          true);

      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("gripper_closed");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);

      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);

      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow collision (object, ground)");

    stage->allowCollisions("object", {"ground"}, true);

    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("arm_start");
    task.add(std::move(stage));
  }

  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{
            { arm_group_name, sampling_planner },
            { hand_group_name, interpolation_planner } });

    stage_move_to_place->setTimeout(10.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });

    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

/*   {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow collision (object,ground) for place IK");
    stage->allowCollisions("object", {"ground"}, true);
    place->insert(std::move(stage));
  }    */                             
    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "world";
      target_pose_msg.pose.position.x = params.place_x;
      target_pose_msg.pose.position.y = params.place_y;
      target_pose_msg.pose.position.z = params.place_z;
      target_pose_msg.pose.orientation.w = 1.0;

      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);

      Eigen::Isometry3d place_frame_transform = Eigen::Isometry3d::Identity();
      place_frame_transform.translation().x() = 0.1;

      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));

      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(place_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("gripper_open");
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "forbid collision (hand,object)");

      stage->allowCollisions(
          "object",
          task.getRobotModel()
              ->getJointModelGroup(hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          false);

      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);

      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.15);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);

      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("arm_start");
    task.add(std::move(stage));
  }

  return task;
}


mtc::Task PickAndPlace::createPickTask(const ObjectParams& params)
{
  mtc::Task task;
  task.stages()->setName("pick and place task");
  task.loadRobotModel(node_);

  const auto& arm_group_name  = "d1_arm";
  const auto& hand_group_name = "d1_gripper";
  const auto& hand_frame      = "Empty_Link6";
  const auto& eef_name        = "gripper";

  task.setProperty("group", arm_group_name);
  task.setProperty("eef", eef_name);
  task.setProperty("ik_frame", hand_frame);

  mtc::Stage* current_state_ptr = nullptr;
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));
/* 
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow collision (object+hand, ground)");
    stage->allowCollisions("object", {"ground"}, true);
    stage->allowCollisions("ground",
        task.getRobotModel()
            ->getJointModelGroup(hand_group_name)
            ->getLinkModelNamesWithCollisionGeometry(),
        true);
    task.add(std::move(stage));
  } */

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.0001);

  // Empezamos con el brazo recodigo
  // auto stage = std::make_unique<mtc::stages::MoveTo>("move to start", interpolation_planner);
  // stage->setGroup(arm_group_name);
  // stage->setGoal("arm_start");
  // task.add(std::move(stage));

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("gripper_open");
  task.add(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });

  stage_move_to_pick->setTimeout(10.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage = nullptr;

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });

    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);

      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.01, 0.10);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.x = 1.0;
      vec.vector.y = 1.0;
      vec.vector.z = 1.0;
      stage->setDirection(vec);

      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("gripper_open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr);

      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      grasp_frame_transform.translation().x() = 0.1;

      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));

      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

      // mas solucions para mas probabilkidades de encontrar una buena
      wrapper->setMaxIKSolutions(50);

      grasp->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow collision (hand,object)");

      stage->allowCollisions(
          "object",
          task.getRobotModel()
              ->getJointModelGroup(hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          true);

      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("gripper_closed");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);

      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);

      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow collision (object, ground)");

    stage->allowCollisions("object", {"ground"}, true);

    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("arm_start");
    task.add(std::move(stage));
  }

  return task;
}

mtc::Task PickAndPlace::createPlaceTask(const ObjectParams& params)
{
  mtc::Task task;
  task.stages()->setName("place task");
  task.loadRobotModel(node_);

  const auto& arm_group_name  = "d1_arm";
  const auto& hand_group_name = "d1_gripper";
  const auto& hand_frame      = "Empty_Link6";
  const auto& eef_name        = "gripper";

  task.setProperty("group", arm_group_name);
  task.setProperty("eef", eef_name);
  task.setProperty("ik_frame", hand_frame);

  // planners
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();

  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.0001);

  // current state
  auto current_state = std::make_unique<mtc::stages::CurrentState>("current");
  mtc::stages::CurrentState* current_state_ptr = current_state.get();
  task.add(std::move(current_state));

  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{
            { arm_group_name, sampling_planner },
            { hand_group_name, interpolation_planner } });

    stage_move_to_place->setTimeout(10.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });

    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

/*   {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow collision (object,ground) for place IK");
    stage->allowCollisions("object", {"ground"}, true);
    place->insert(std::move(stage));
  }    */                             
    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "world";
      target_pose_msg.pose.position.x = params.place_x;
      target_pose_msg.pose.position.y = params.place_y;
      target_pose_msg.pose.position.z = params.place_z;
      target_pose_msg.pose.orientation.w = 1.0;

      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(current_state_ptr);

      Eigen::Isometry3d place_frame_transform = Eigen::Isometry3d::Identity();
      place_frame_transform.translation().x() = 0.1;

      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));

      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(place_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("gripper_open");
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "forbid collision (hand,object)");

      stage->allowCollisions(
          "object",
          task.getRobotModel()
              ->getJointModelGroup(hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          false);

      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);

      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.15);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);

      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("arm_start");
    task.add(std::move(stage));
  }

  /* task.add(std::move(place)); */

  return task;
}