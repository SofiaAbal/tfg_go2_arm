#include "d1_550_mtc/pick_and_place.h"
#include "d1_550_config/srv/pick_and_place_object.hpp"
#include "d1_550_config/srv/pick_object.hpp"
#include "d1_550_config/srv/place_object.hpp"

#include <rclcpp/rclcpp.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("main");

class PickAndPlace;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pick_and_place_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  /* std::thread spin_thread([&executor]() { executor.spin(); }); */

  auto pick_place_task = std::make_shared<PickAndPlace>(node);
  auto servicePick = node->create_service<d1_550_config::srv::PickObject>("pick_object",
      [&](const std::shared_ptr<d1_550_config::srv::PickObject::Request> request,
      std::shared_ptr<d1_550_config::srv::PickObject::Response> response)
      {
        RCLCPP_INFO(LOGGER,
            "Request: 'pick(%.2f, %.2f, %.2f) - shape: %s' - dimensions(%.2f, %.2f, %.2f)",
            request->pick_x, request->pick_y, request->pick_z,
            request->shape.c_str(),
            request->dimension_x, request->dimension_y, request->dimension_z
          );
        
        try
          {
          ObjectParams params {
          .pick_x  = request->pick_x,
          .pick_y  = request->pick_y,
          .pick_z  = request->pick_z,
          .shape   = request->shape,
          .dimension_x = request->dimension_x,
          .dimension_y = request->dimension_y,
          .dimension_z = request->dimension_z
          };

          pick_place_task->setupPlanningScene(params);
          response->success = pick_place_task->doPickTask(params);
          if (response->success) {
            response->message = "Tarea pick realizada correctamente";
            moveit::planning_interface::PlanningSceneInterface psi;

            moveit_msgs::msg::AttachedCollisionObject aco;
            aco.object.id = "object";
            aco.link_name = "Empty_Link6";  // hand_frame
            aco.object.operation = aco.object.ADD;

            psi.applyAttachedCollisionObject(aco);

            // opcional pero recomendado
            rclcpp::sleep_for(std::chrono::milliseconds(500));
          } else {
            response->message = "Error durante la planificación o ejecución de pick";
          }
        }
        catch(const std::exception& e)
        {
          response->success = false;
          response->message = e.what();
        }
        RCLCPP_INFO(LOGGER, "Response: [%s] %s", response->success ? "OK" : "FAIL", response->message.c_str());
      });
  

  RCLCPP_INFO(LOGGER, "Servicio pick_object activo. Esperando parámetros del objeto...");

  auto servicePlace = node->create_service<d1_550_config::srv::PlaceObject>("place_object",
      [&](const std::shared_ptr<d1_550_config::srv::PlaceObject::Request> request,
      std::shared_ptr<d1_550_config::srv::PlaceObject::Response> response)
      {
        RCLCPP_INFO(LOGGER,
            "Request: 'place(%.2f, %.2f, %.2f)'",
            request->place_x, request->place_y, request->place_z
          );
        
        try
        {
          ObjectParams params {
          .place_x = request->place_x,
          .place_y = request->place_y,
          .place_z = request->place_z
        };

        response->success = pick_place_task->doPlaceTask(params);
        response->message = response->success
            ? "Tarea place completada correctamente"
            : "Error durante la planificación o ejecución de place";
        }
        catch(const std::exception& e)
        {
          response->success = false;
          response->message = e.what();
        }
        RCLCPP_INFO(LOGGER, "Response: [%s] %s", response->success ? "OK" : "FAIL", response->message.c_str());
      });
  

  RCLCPP_INFO(LOGGER, "Servicio place_object activo. Esperando parámetros del objeto...");

  /* spin_thread.join(); */

  executor.spin();
  rclcpp::shutdown();

  return 0;
}

/* void pick_place_task(){
  auto pick_and_place_task = std::make_shared<PickAndPlace>(node);

  auto service = node->create_service<d1_550_config::srv::PickAndPlaceObject>("pick_and_place_object",
      [&](const std::shared_ptr<d1_550_config::srv::PickAndPlaceObject::Request> request,
      std::shared_ptr<d1_550_config::srv::PickAndPlaceObject::Response> response)
      {
        RCLCPP_INFO(LOGGER,
            "Request: 'pick(%.2f, %.2f, %.2f) - place(%.2f, %.2f, %.2f) - shape: %s' - dimensions(%.2f, %.2f, %.2f)",
            request->pick_x, request->pick_y, request->pick_z,
            request->place_x, request->place_y, request->place_z,
            request->shape.c_str(),
            request->dimension_x, request->dimension_y, request->dimension_z
          );
        
        try
        {
          ObjectParams params {
          .pick_x  = request->pick_x,
          .pick_y  = request->pick_y,
          .pick_z  = request->pick_z,
          .place_x = request->place_x,
          .place_y = request->place_y,
          .place_z = request->place_z,
          .shape   = request->shape,
          .dimension_x = request->dimension_x,
          .dimension_y = request->dimension_y,
          .dimension_z = request->dimension_z
        };

        pick_and_place_task->setupPlanningScene(params);
        response->success = pick_and_place_task->doPickAndPlaceTask(params);
        response->message = response->success
            ? "Tarea completada correctamente"
            : "Error durante la planificación o ejecución";
        }
        catch(const std::exception& e)
        {
          response->success = false;
          response->message = e.what();
        }
        RCLCPP_INFO(LOGGER, "Response: [%s] %s", response->success ? "OK" : "FAIL", response->message.c_str());
      });
} */