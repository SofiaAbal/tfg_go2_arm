#include "d1_550_mtc/pick_and_place.h"
#include "d1_550_config/srv/pick_and_place_object.hpp"

#include <rclcpp/rclcpp.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("main");

class PickAndPlace;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pick_and_place_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

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

  RCLCPP_INFO(LOGGER, "Servicio pick_and_place_object activo. Esperando parámetros del objeto...");

  spin_thread.join();
  rclcpp::shutdown();

  return 0;
}