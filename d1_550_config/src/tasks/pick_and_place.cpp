#include "d1_550_mtc/pick_and_place.h"
#include "d1_550_mtc/constants.h"
#include <fmt/core.h>

namespace mtc = moveit::task_constructor;


PickAndPlace::PickAndPlace(const rclcpp::Node::SharedPtr& node)
: node_(node) {
  setupObstacles();
  rclcpp::sleep_for(std::chrono::milliseconds(500));
}

void setObjectData(shape_msgs::msg::SolidPrimitive& primitive, const ObjectParams& params)
{
  std::string shape = params.shape;

  if (shape == BOX) {
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = { params.dimension_x, params.dimension_y, params.dimension_z };
  } else if (shape == CYLINDER) {
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    primitive.dimensions = { params.dimension_x, params.dimension_y };
  } else if (shape == CONE) {
    primitive.type = shape_msgs::msg::SolidPrimitive::CONE;
    primitive.dimensions = { params.dimension_x, params.dimension_y };
  } else if (shape == SPHERE) {
    primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    primitive.dimensions = { params.dimension_y };
  } else {
    throw std::runtime_error(fmt::format(ERROR_INVALID_SHAPE, shape));
  }
}

moveit_msgs::msg::CollisionObject defineObject(const ObjectParams& params)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = OBJECT;
  object.header.frame_id = WORLD;

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

  /* object.pose = pose; */
  object.primitive_poses.push_back(pose);

  return object;
}

moveit_msgs::msg::CollisionObject defineDog() {
  moveit_msgs::msg::CollisionObject object;
  object.id = DOG;
  object.header.frame_id = WORLD;

  object.primitives.resize(1);
  setObjectData(object.primitives[0], ObjectParams{.shape=BOX, .dimension_x=0.7, .dimension_y=0.35, .dimension_z=0.2});

  geometry_msgs::msg::Pose pose;
  pose.position.x = -0.25;
  pose.position.y = 0;
  pose.position.z = -0.1;
  pose.orientation.w = 1.0;

  /* object.pose = pose; */
  object.primitive_poses.push_back(pose);

  return object;
}

moveit_msgs::msg::CollisionObject defineGround() {
  moveit_msgs::msg::CollisionObject object;
  object.id = GROUND;
  object.header.frame_id = WORLD;

  object.primitives.resize(1);
  setObjectData(object.primitives[0], ObjectParams{.shape=BOX, .dimension_x=2.0, .dimension_y=2.0, .dimension_z=0.01});

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = -0.2;
  pose.orientation.w = 1.0;

  /* object.pose = pose; */
  object.primitive_poses.push_back(pose);

  return object;
}

void PickAndPlace::normalizeShape(ObjectParams& params)
{
  std::transform(params.shape.begin(), params.shape.end(), params.shape.begin(), [](unsigned char c){ return std::toupper(c); });
}

void PickAndPlace::setupPlanningScene(const ObjectParams& params)
{
  // Definimos el objeto a manipular
  moveit_msgs::msg::CollisionObject object = defineObject(params);
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void PickAndPlace::setupObstacles()
{
  // Definimos obstáculos en la escena: perro y suelo
  moveit::planning_interface::PlanningSceneInterface psi;

  moveit_msgs::msg::CollisionObject ground = defineGround();
  psi.applyCollisionObject(ground);

  moveit_msgs::msg::CollisionObject dog = defineDog();
  psi.applyCollisionObject(dog);
}

bool PickAndPlace::doPickTask(const ObjectParams& params)
{
  task_ = createPickTask(params);

  try {
    task_.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), e);
    return false;
  }

  if (!task_.plan(5)) {
    RCLCPP_ERROR(node_->get_logger(), ERROR_PLANNING_FAILED);
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

  try {
    task_.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), e);
    return false;
  }

  if (!task_.plan(5)) {
    RCLCPP_ERROR(node_->get_logger(), ERROR_PLANNING_FAILED);
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

mtc::Task PickAndPlace::createPickTask(const ObjectParams& params)
{
  mtc::Task task;
  task.stages()->setName(PICK_TASK);
  task.loadRobotModel(node_);

  const auto& arm_group_name  = ARM_GROUP;
  const auto& hand_group_name = HAND_GROUP;
  const auto& hand_frame      = HAND_FRAME;
  const auto& eef_name        = EEF_NAME;

  task.setProperty(GROUP_PROPERTY, arm_group_name);
  task.setProperty(EEF_PROPERTY, eef_name);
  task.setProperty(IK_FRAME_PROPERTY, hand_frame);

  mtc::Stage* current_state_ptr = nullptr;
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>(CURRENT_STATE_TASK);
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.0001);

  // Abrimos la mano
  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>(OPEN_HAND_STAGE, interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal(STAGE_GOAL_GRIPPER_OPEN);
  task.add(std::move(stage_open_hand));

  // Movemos el efector final cerca del objeto a manipular con un planificador de movimientos de robot (RRTConnect) ??
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(MOVE_TO_PICK_STAGE,
    mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(10.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage = nullptr;

  // Secuencia de etapas para aproximarse al objeto, agarrarlo, levantarlo y volver a la posicion inicial
  {
    auto grasp = std::make_unique<mtc::SerialContainer>(PICK_OBJECT_CONTAINER);
    task.properties().exposeTo(grasp->properties(), { EEF_PROPERTY, GROUP_PROPERTY, IK_FRAME_PROPERTY });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { EEF_PROPERTY, GROUP_PROPERTY, IK_FRAME_PROPERTY });

    // Aproximamos brazo a objeto a manipular
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>(APPROACH_OBJECT_STAGE, cartesian_planner);

      stage->properties().set(STAGE_PROPERTIES_MARKER_NS, STAGE_MARKER_NS_APPROACH_OBJECT);
      stage->properties().set(STAGE_PROPERTIES_LINK, hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { GROUP_PROPERTY });
      stage->setMinMaxDistance(0.01, 0.10);

      geometry_msgs::msg::Vector3Stamped vec;
      if(params.pick_grasp == "side") {
        vec.header.frame_id = hand_frame;
        vec.vector.x = 1.0;
        vec.vector.y = 1.0;
        vec.vector.z = 1.0;
      } else {
        vec.header.frame_id = WORLD;
        vec.vector.z = -1.0;
      }
      stage->setDirection(vec);

      grasp->insert(std::move(stage));
    }

    // Permitimos colisión entre la mano y el objeto a manipular para poder agarrarlo
    {
      if(params.pick_grasp == "top") {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
            ALLOW_COLLISIONS_HAND_OBJECT_STAGE);

        stage->allowCollisions(
            OBJECT,
            task.getRobotModel()
                ->getJointModelGroup(hand_group_name)
                ->getLinkModelNamesWithCollisionGeometry(),
            true);

        grasp->insert(std::move(stage));
      }
    }

    // Generamos pose de agarre
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>(GENERATE_GRASP_POSE_STAGE);
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set(STAGE_PROPERTIES_MARKER_NS, STAGE_MARKER_NS_GRASP_POSE);
      stage->setPreGraspPose(STAGE_GRASP_POSE);
      stage->setObject(OBJECT);
      stage->setMonitoredStage(current_state_ptr);

      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      if(params.pick_grasp == "side") {
        stage->setAngleDelta(M_PI / 6);
        grasp_frame_transform.translation().x() = 0.1;
      } else {
        Eigen::AngleAxisd rot_y(-M_PI/2, Eigen::Vector3d::UnitY());
        grasp_frame_transform.rotate(rot_y);
        grasp_frame_transform.translation().x() = 0.10 + GRASP_OFFSET;
      }

      auto wrapper =std::make_unique<mtc::stages::ComputeIK>(GRASP_POSE_IK_STAGE, std::move(stage));
      //wrapper->setMaxIKSolutions(if (params.pick_grasp == "side" ? 8 : 50));
      wrapper->setMinSolutionDistance(0.1);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { EEF_PROPERTY, GROUP_PROPERTY });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { STAGE_TARGET_POSE });

      // mas solucions para mas probabilkidades de encontrar una buena
      wrapper->setMaxIKSolutions(50);

      grasp->insert(std::move(wrapper));
    }

    // Permitimos colisión entre la mano y el objeto a manipular para poder agarrarlo
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          ALLOW_COLLISIONS_HAND_OBJECT_STAGE);

      stage->allowCollisions(
          OBJECT,
          task.getRobotModel()
              ->getJointModelGroup(hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          true);

      grasp->insert(std::move(stage));
    }

    // Cerramos la mano
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>(CLOSE_HAND_STAGE, interpolation_planner);
      stage->setGroup(hand_group_name);
      /* stage->setGoal(STAGE_GOAL_GRIPPER_CLOSED); */

      double object_width = (params.shape == BOX) ? params.dimension_y / 2 : params.dimension_y;
      double grasp_width = 0.033 - object_width + STRENGTH;
      // explicación-> 0: pinza en la esquina, 0.033: pinza se desplaza hacia el centro

        std::map<std::string, double> target;
        target[JOINT_L] = grasp_width;
        target[JOINT_R] = grasp_width;

        stage->setGoal(target);
      
      grasp->insert(std::move(stage));
    }

    // Al agarrar el objeto, pasa a forma parte del brazo, así que lo añadimos comoobjecto adherido al efector final para que el planificador lo tenga en cuenta como parte del robot y no lo considere un obstáculo
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(ATTACH_OBJECT_STAGE);
      stage->attachObject(OBJECT, hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    // Levantamos el objeto agarrado con un movimiento vertical (para que no choque con el perro/seulo)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>(LIFT_OBJECT_STAGE, cartesian_planner);

      stage->properties().configureInitFrom(mtc::Stage::PARENT, { GROUP_PROPERTY });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set(STAGE_PROPERTIES_MARKER_NS, STAGE_MARKER_NS_LIFT_OBJECT);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = WORLD;
      vec.vector.z = 1.0;
      stage->setDirection(vec);

      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  // Volvemos a la posicion inicial de brazo recogido
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>(RETURN_HOME_STAGE, interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { GROUP_PROPERTY });
    stage->setGoal(STAGE_GOAL_ARM_START);
    task.add(std::move(stage));
  }

  return task;
}

mtc::Task PickAndPlace::createPlaceTask(const ObjectParams& params)
{
  mtc::Task task;
  task.stages()->setName(PLACE_TASK);
  task.loadRobotModel(node_);

  const auto& arm_group_name  = ARM_GROUP;
  const auto& hand_group_name = HAND_GROUP;
  const auto& hand_frame      = HAND_FRAME;
  const auto& eef_name        = EEF_NAME;

  task.setProperty(GROUP_PROPERTY, arm_group_name);
  task.setProperty(EEF_PROPERTY, eef_name);
  task.setProperty(IK_FRAME_PROPERTY, hand_frame);

  // planners
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();

  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.0001);

  // current state
  auto current_state = std::make_unique<mtc::stages::CurrentState>(CURRENT_STATE_TASK);
  mtc::stages::CurrentState* current_state_ptr = current_state.get();
  task.add(std::move(current_state));

  // ??
  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        MOVE_TO_PLACE_STAGE,
        mtc::stages::Connect::GroupPlannerVector{
            { arm_group_name, sampling_planner },
            { hand_group_name, interpolation_planner } });

    stage_move_to_place->setTimeout(10.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

   // Secuencia de etapas para aproximarse a la zona de colocación, soltar el objeto, retirar la mano y volver a la posicion inicial
  {
    auto place = std::make_unique<mtc::SerialContainer>(PLACE_OBJECT_STAGE);
    task.properties().exposeTo(place->properties(), { EEF_PROPERTY, GROUP_PROPERTY, IK_FRAME_PROPERTY });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { EEF_PROPERTY, GROUP_PROPERTY, IK_FRAME_PROPERTY });

    {
      // Generamos la pose de colocación
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>(GENERATE_PLACE_POSE_STAGE);
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set(STAGE_PROPERTIES_MARKER_NS, STAGE_MARKER_NS_PLACE_POSE);
      stage->setObject(OBJECT);

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = WORLD;
      target_pose_msg.pose.position.x = params.place_x;
      target_pose_msg.pose.position.y = params.place_y;
      target_pose_msg.pose.position.z = params.place_z;
      
      if(params.pick_grasp == "side") {
        target_pose_msg.pose.orientation.w = 1.0;
      } else {
        tf2::Quaternion q;
        q.setRPY(0, M_PI/2, 0); // o equivalente a tu rot_y
        q.normalize();
        target_pose_msg.pose.orientation = tf2::toMsg(q);
      }

      Eigen::Isometry3d place_frame_transform = Eigen::Isometry3d::Identity();
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(current_state_ptr);

      if(params.pick_grasp == "top") {
        Eigen::AngleAxisd rot_y(-M_PI/2, Eigen::Vector3d::UnitY());
        place_frame_transform.rotate(rot_y);
        place_frame_transform.translation().x() = 0.10 + GRASP_OFFSET;
      } else {
        place_frame_transform.translation().x() = 0.1;
      }      

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>(PLACE_POSE_IK_STAGE, std::move(stage));

      wrapper->setMaxIKSolutions(50);
      wrapper->setMinSolutionDistance(0.1);
      wrapper->setIKFrame(place_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { EEF_PROPERTY, GROUP_PROPERTY });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { STAGE_TARGET_POSE });

      place->insert(std::move(wrapper));
    }

    // Abrimos la mano
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>(OPEN_HAND_STAGE, interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal(STAGE_GRASP_POSE);
      place->insert(std::move(stage));
    }

    // Prohibimos colision mano-objeto para soltarlo sin problemas y que no se lea como obstaculo
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(FORBID_COLLISIONS_HAND_OBJECT_STAGE);

      stage->allowCollisions(
          OBJECT,
          task.getRobotModel()
              ->getJointModelGroup(hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          false);

      place->insert(std::move(stage));
    }

    // Desvinculamos el objeto de la pinza
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(DETACH_OBJECT_STAGE);
      stage->detachObject(OBJECT, hand_frame);
      place->insert(std::move(stage));
    }

    // Retiramos la mano de forma verticar para evitar colis iones con el perro/suelo
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>(RETREAT_STAGE, cartesian_planner);

      stage->properties().configureInitFrom(mtc::Stage::PARENT, { GROUP_PROPERTY });
      stage->setMinMaxDistance(0.01, 0.1);
      stage->setIKFrame(hand_frame);
      stage->properties().set(STAGE_PROPERTIES_MARKER_NS, STAGE_MARKER_NS_RETREAT);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = WORLD;
      if(params.pick_grasp == "side") {
        vec.vector.z = 1.0;
      } else {
        vec.vector.z = -1.0;
      }
      stage->setDirection(vec);

      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  // Volvemos a la posicion inicial de brazo recogido
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>(RETURN_HOME_STAGE, interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { GROUP_PROPERTY });
    stage->setGoal(STAGE_GOAL_ARM_START);
    task.add(std::move(stage));
  }

  return task;
}