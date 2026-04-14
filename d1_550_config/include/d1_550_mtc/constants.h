#pragma once

//===========
// CONSTANTS
//===========

// Primitives
static constexpr const char* BOX = "BOX";
static constexpr const char* CYLINDER = "CYLINDER";
static constexpr const char* CONE = "CONE";
static constexpr const char* SPHERE = "SPHERE";

// Scene objects
static constexpr const char* DOG = "dog";
static constexpr const char* GROUND = "ground"; 
static constexpr const char* OBJECT = "object";
static constexpr const char* WORLD = "world";

// Arm config
static constexpr const char* ARM_GROUP = "d1_arm";
static constexpr const char* HAND_GROUP = "d1_gripper";
static constexpr const char* EEF_NAME = "gripper";
static constexpr const char* HAND_FRAME = "Empty_Link6";
static constexpr const char* GROUP_PROPERTY = "group";

// Properties
static constexpr const char* EEF_PROPERTY = "eef";
static constexpr const char* IK_FRAME_PROPERTY = "ik_frame";

// Tasks
static constexpr const char* PICK_TASK = "pick task";
static constexpr const char* PLACE_TASK = "place task";
static constexpr const char* CURRENT_STATE_TASK = "current";

// Stages
static constexpr const char* OPEN_HAND_STAGE = "open hand";
static constexpr const char* CLOSE_HAND_STAGE = "close hand";
static constexpr const char* MOVE_TO_PICK_STAGE = "move to pick";
static constexpr const char* MOVE_TO_PLACE_STAGE = "move to place";
static constexpr const char* APPROACH_OBJECT_STAGE = "approach object";
static constexpr const char* ATTACH_OBJECT_STAGE = "attach object";
static constexpr const char* LIFT_OBJECT_STAGE = "lift object";
static constexpr const char* GENERATE_GRASP_POSE_STAGE = "generate grasp pose";
static constexpr const char* ALLOW_COLLISIONS_HAND_OBJECT_STAGE = "allow collision (hand,object)";
static constexpr const char* RETURN_HOME_STAGE = "return home";
static constexpr const char* GENERATE_PLACE_POSE_STAGE = "generate place pose";
static constexpr const char* RETREAT_STAGE = "retreat";
static constexpr const char* PLACE_OBJECT_STAGE = "place object";
static constexpr const char* GRASP_POSE_IK_STAGE = "grasp pose IK";
static constexpr const char* PLACE_POSE_IK_STAGE = "place pose IK";
static constexpr const char* FORBID_COLLISIONS_HAND_OBJECT_STAGE = "forbid collision (hand,object)";
static constexpr const char* DETACH_OBJECT_STAGE = "detach object";

// Properties
static constexpr const char* STAGE_PROPERTIES_MARKER_NS = "marker_ns";
static constexpr const char* STAGE_PROPERTIES_LINK = "link";

static constexpr const char* STAGE_MARKER_NS_APPROACH_OBJECT = "approach_object";
static constexpr const char* STAGE_MARKER_NS_GRASP_POSE = "grasp_pose";
static constexpr const char* STAGE_MARKER_NS_LIFT_OBJECT = "lift_object";
static constexpr const char* STAGE_MARKER_NS_PLACE_POSE = "place_pose";
static constexpr const char* STAGE_MARKER_NS_RETREAT = "retreat";

static constexpr const char* STAGE_TARGET_POSE = "target_pose";
static constexpr const char* STAGE_GRASP_POSE = "gripper_open";

static constexpr const char* STAGE_GOAL_GRIPPER_OPEN = "gripper_open";
static constexpr const char* STAGE_GOAL_GRIPPER_CLOSED = "gripper_closed";
static constexpr const char* STAGE_GOAL_ARM_START = "arm_start";

static constexpr const char* PICK_OBJECT_CONTAINER = "pick object";

// Error messages
static constexpr const char* ERROR_INVALID_SHAPE = "Shape '{}' is not valid. Use 'BOX', 'CYLINDER', 'CONE' or 'SPHERE'.";
static constexpr const char* ERROR_PLANNING_FAILED = "Planning failed";

// Joints
static constexpr const char* JOINT_L = "Joint_L";
static constexpr const char* JOINT_R = "Joint_R";

// Values
static constexpr double STRENGTH = 0.003;