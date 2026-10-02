#!/usr/bin/env python3
import csv
import itertools
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from d1_550_config.srv import PickObject
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# --- Configuración del barrido ---
X_RANGE = (-0.5, 0.5, 0.05)    # (min, max, step)
Y_RANGE = (-0.5, 0.5, 0.05)
Z_VALUES = [0.05]
GRASPS = ["side", "top"]
SHAPE = "cylinder"
DIMENSION_X = 0.09
DIMENSION_Y = 0.025
OUTPUT_CSV = f"pick_reachability_{int(time.time())}.csv"
MARKER_TOPIC = "/pick_reachability_markers"


def frange(start, stop, step):
    n = round((stop - start) / step)
    return [round(start + i * step, 4) for i in range(n + 1)]


class ReachabilitySweeper(Node):
    def __init__(self):
        super().__init__("pick_reachability_sweeper")
        self.client = self.create_client(PickObject, "plan_pick_object")
        self.marker_pub = self.create_publisher(MarkerArray, MARKER_TOPIC, 10)
        self.markers = MarkerArray()
        self._marker_id = 0

        self.get_logger().info("Esperando servicio 'plan_pick_object'...")
        self.client.wait_for_service()

    def check_point(self, x, y, z, grasp):
        request = PickObject.Request()
        request.pick_x = x
        request.pick_y = y
        request.pick_z = z
        request.shape = SHAPE
        request.dimension_x = DIMENSION_X
        request.dimension_y = DIMENSION_Y
        request.pick_grasp = grasp

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def add_marker(self, x, y, z, success):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pick_reachability"
        marker.id = self._marker_id
        self._marker_id += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Point(x=x, y=y, z=z)
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.02
        marker.color = (ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8) if success
                         else ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8))

        self.markers.markers.append(marker)
        self.marker_pub.publish(self.markers)

    def run(self):
        rows = []
        points = list(itertools.product(
            frange(*X_RANGE), frange(*Y_RANGE), Z_VALUES, GRASPS))

        for i, (x, y, z, grasp) in enumerate(points):
            success = self.check_point(x, y, z, grasp)
            self.add_marker(x, y, z, success)
            self.get_logger().info(
                f"[{i+1}/{len(points)}] ({x:.2f}, {y:.2f}, {z:.2f}) grasp={grasp} -> "
                f"{'OK' if success else 'FAIL'}")
            rows.append({
                "x": x, "y": y, "z": z,
                "shape": SHAPE, "dimension_x": DIMENSION_X, "dimension_y": DIMENSION_Y,
                "grasp": grasp, "success": success,
            })

        self.write_csv(rows)

    def write_csv(self, rows):
        path = Path(OUTPUT_CSV)
        with path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
        self.get_logger().info(f"Resultados guardados en {path.resolve()}")


def main():
    rclpy.init()
    node = ReachabilitySweeper()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
