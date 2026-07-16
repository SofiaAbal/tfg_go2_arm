#!/usr/bin/env python3
#import os
import time
import rclpy
#import sys
import math
import threading
#import time
from rclpy.node import Node
from sensor_msgs.msg import JointState

from d1_arm import D1Arm

D2R = math.pi / 180.0
R2D = 180.0 / math.pi
GRIPPER_MM_TO_POS = 0.033 / 65.0
GRIPPER_POS_TO_MM = 65.0 / 0.033

# inviereto signo
JOINT_SIGN = {
    'Joint1': 1.0, 'Joint2': 1.0, 'Joint3': 1.0,
    'Joint4': -1.0, 'Joint5': 1.0, 'Joint6': -1.0,
}

class D1Driver(Node):
    def __init__(self):
        super().__init__('d1_driver')
        #self._seq = 4
        #self._lock = threading.Lock()
        self._running = True

        self.arm = D1Arm()

        self.state_pub = self.create_publisher(JointState, '/arm_joint_states', 10)
        self.cmd_sub = self.create_subscription(
            JointState, '/arm_joint_commands', self._on_command, 10)
        
        self._start_arm()

        self._state_timer = self.create_timer(0.01, self._publish_state)

    
    def _start_arm(self):
        self.arm.enable_motors()
        self.get_logger().info(f'<< Arm enabled >>', throttle_duration_sec=0.5)
        time.sleep(0.5)

        self.arm.enable_gripper()
        self.get_logger().info(f'<< Gripper enabled >>', throttle_duration_sec=0.5)
        time.sleep(2.0)

        self.arm.zero()
        self.get_logger().info(f'<< Arm moving to zero position >>', throttle_duration_sec=0.5)
        time.sleep(3.0)

        #self.arm.open_gripper()
        #self.get_logger().info(f'<< Gripper opened >>', throttle_duration_sec=0.5)
        #time.sleep(2.0)

    
    def _on_command(self, msg: JointState):
        if not self._running:
            return

        joint_deg = [msg.position[i] * R2D * JOINT_SIGN[f'Joint{i+1}'] for i in range(6)]
        gripper_mm = msg.position[6] * GRIPPER_POS_TO_MM
        self.arm.move_joints(joint_deg, gripper_mm)

    def _publish_state(self):
        for joint_deg, gripper_mm in self.arm.read_joints():
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Joint_L', 'Joint_R']

            joint_rad = [
                JOINT_SIGN[name] * deg * D2R
                for name, deg in zip(js.name[:6], joint_deg)
            ]
            #  0=abierta 0.033=cerrada
            gripper_pos = 0.033 - gripper_mm * GRIPPER_MM_TO_POS
            js.position = joint_rad + [gripper_pos, gripper_pos]

            self.get_logger().info(f' >>>> Joint positions: {js.position[:7]}, Gripper position: {js.position[7]} <<', throttle_duration_sec=0.5)
        

            self.state_pub.publish(js)

    def destroy_node(self):
        self._running = False
        self.arm.disable_gripper()
        self.get_logger().info(f'<< Gripper disabled >>', throttle_duration_sec=0.5)
        time.sleep(0.5)

        self.arm.disable_motors()
        self.get_logger().info(f'<< Arm disabled >>', throttle_duration_sec=0.5)
        time.sleep(0.5)

        super().destroy_node()

def main(args=None):
    rclpy.init()
    node = D1Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    #rclpy.init(args=args)
    #roarm_driver = D1Driver()
    #rclpy.spin(roarm_driver)
    #roarm_driver.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()