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

from dataclasses import dataclass
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import float32
from cyclonedds.domain import Domain, DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader

D2R = math.pi / 180.0
R2D = 180.0 / math.pi
GRIPPER_MM_TO_POS = 0.033 / 65.0
GRIPPER_POS_TO_MM = 65.0 / 0.033

# inviereto signo
JOINT_SIGN = {
    'Joint1': 1.0, 'Joint2': 1.0, 'Joint3': 1.0,
    'Joint4': -1.0, 'Joint5': 1.0, 'Joint6': -1.0,
}

@dataclass
class ArmString_(IdlStruct, typename='unitree_arm::msg::dds_::ArmString_'):
    data_: str = ''

@dataclass
class PubServoInfo_(IdlStruct, typename='unitree_arm::msg::dds_::PubServoInfo_'):
    servo0_data_: float32 = float32(0)
    servo1_data_: float32 = float32(0)
    servo2_data_: float32 = float32(0)
    servo3_data_: float32 = float32(0)
    servo4_data_: float32 = float32(0)
    servo5_data_: float32 = float32(0)
    servo6_data_: float32 = float32(0)

class D1Driver(Node):
    def __init__(self):
        super().__init__('d1_driver')
        self._seq = 4
        self._lock = threading.Lock()
        self._running = True

        # ROS2 pub/sub (FastDDS, set via RMW_IMPLEMENTATION in launch file)
        self.state_pub = self.create_publisher(JointState, '/arm_joint_states', 10)
        
        #self.cmd_sub = self.create_subscription(
        #    JointState, '/arm_joint_commands', self._read_loop, 10)

        self.cmd_sub = self.create_subscription(
            JointState, '/arm_joint_commands', self._on_command, 10)

        # CycloneDDS entities (Python cyclonedds bundled libddsc — no iceoryx conflict)
        self._dp = DomainParticipant(0)
        cmd_topic = Topic(self._dp, 'rt/arm_Command', ArmString_)
        self._writer = DataWriter(self._dp, cmd_topic)

        # Enable motors and gripper
        self._send('{"seq":1,"address":1,"funcode":5,"data":{"mode":80000}}')
        self._send('{"seq":2,"address":1,"funcode":4,"data":{"id":6,"mode":80000}}')
        time.sleep(2)
        # Zero arm
        self._send('{"seq":3,"address":1,"funcode":7}')
        self.get_logger().info('Arm zeroing (3 s)...')
        time.sleep(3)

        startup = JointState()
        startup.header.stamp = self.get_clock().now().to_msg()
        startup.name = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Joint_L', 'Joint_R']
        startup.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.033, 0.033]
        self.state_pub.publish(startup)
       
        # State reader in background thread
        #state_topic = Topic(self._dp, 'current_servo_angle', PubServoInfo_)
        #self._reader = DataReader(self._dp, state_topic)
        """ self._reader_thread = threading.Thread(
            target=self._read_loop, name='d1_state_reader', daemon=True) """
        #self._reader_thread = threading.Thread(
        #    target=self.send_and_wait, name='d1_state_reader', daemon=True)
        #self._reader_thread.start()

        #self.get_logger().info(f'D1Driver ready — iface={IFACE}')


    def _read_loop(self):
        """Background thread: polls arm DDS state at ~100 Hz."""
        while self._running:
            try:
                for sample in self._reader.take(10):
                    if not sample.sample_info.valid_data:
                        continue
                    js = JointState()
                    js.header.stamp = self.get_clock().now().to_msg()
                    js.name = ['Joint1', 'Joint2', 'Joint3',
                               'Joint4', 'Joint5', 'Joint6',
                               'Joint_L', 'Joint_R']
                    # Real gripper: 0mm=closed, 65mm=open. Sim (Joint_L/R): 0=open, 0.033=closed — inverted.
                    gripper_pos = 0.033 - float(sample.servo6_data_) * GRIPPER_MM_TO_POS
                    js.position = [
                        JOINT_SIGN['Joint1'] * float(sample.servo0_data_) * D2R,
                        JOINT_SIGN['Joint2'] * float(sample.servo1_data_) * D2R,
                        JOINT_SIGN['Joint3'] * float(sample.servo2_data_) * D2R,
                        JOINT_SIGN['Joint4'] * float(sample.servo3_data_) * D2R,
                        JOINT_SIGN['Joint5'] * float(sample.servo4_data_) * D2R,
                        JOINT_SIGN['Joint6'] * float(sample.servo5_data_) * D2R,
                        gripper_pos,
                        gripper_pos,
                    ]
                    self.state_pub.publish(js)
            except Exception as e:
                self.get_logger().warn(f'State read error: {e}')
            time.sleep(0.01)

    def _send(self, json_str: str):
        self.get_logger().info(f'>> Enviado: {json_str}', throttle_duration_sec=0.5)
        with self._lock:
            self._writer.write(ArmString_(data_=json_str))
            self._seq += 1


#    def _on_command(self, msg: JointState):
#        self.get_logger().info(f'<< Recibido: {msg}', throttle_duration_sec=0.5)
#        a = [0.0] * 6
#        gripper_mm = 0.0
#        for name, pos in zip(msg.name, msg.position):
#            if   name == 'Joint1': a[0] = JOINT_SIGN['Joint1'] * pos * R2D
#            elif name == 'Joint2': a[1] = JOINT_SIGN['Joint2'] * pos * R2D
#            elif name == 'Joint3': a[2] = JOINT_SIGN['Joint3'] * pos * R2D
#            elif name == 'Joint4': a[3] = JOINT_SIGN['Joint4'] * pos * R2D
#            elif name == 'Joint5': a[4] = JOINT_SIGN['Joint5'] * pos * R2D
#            elif name == 'Joint6': a[5] = JOINT_SIGN['Joint6'] * pos * R2D
#            elif name == 'Joint_L': gripper_mm = 65.0 - pos * GRIPPER_POS_TO_MM
#        seq = self._seq
#        cmd = (f'{{"seq":{seq},"address":1,"funcode":2,"data":{{'
#               f'"mode":1'
#               f',"angle0":{a[0]:.2f},"angle1":{a[1]:.2f}'
#               f',"

    def _on_command(self, msg: JointState):
        self.get_logger().info(f'<< Recibido: {msg}', throttle_duration_sec=0.5)
        a = [0.0] * 6
        gripper_mm = 0.0
        for name, pos in zip(msg.name, msg.position):
            if   name == 'Joint1': a[0] = JOINT_SIGN['Joint1'] * pos * R2D
            elif name == 'Joint2': a[1] = JOINT_SIGN['Joint2'] * pos * R2D
            elif name == 'Joint3': a[2] = JOINT_SIGN['Joint3'] * pos * R2D
            elif name == 'Joint4': a[3] = JOINT_SIGN['Joint4'] * pos * R2D
            elif name == 'Joint5': a[4] = JOINT_SIGN['Joint5'] * pos * R2D
            elif name == 'Joint6': a[5] = JOINT_SIGN['Joint6'] * pos * R2D
            elif name == 'Joint_L': gripper_mm = 65.0 - pos * GRIPPER_POS_TO_MM
        seq = self._seq
        cmd = (f'{{"seq":{seq},"address":1,"funcode":2,"data":{{'
               f'"mode":1'
               f',"angle0":{a[0]:.2f},"angle1":{a[1]:.2f}'
               f',"angle2":{a[2]:.2f},"angle3":{a[3]:.2f}'
               f',"angle4":{a[4]:.2f},"angle5":{a[5]:.2f}'
               f',"angle6":{gripper_mm:.2f}}}}}')
        self._send(cmd)

    def destroy_node(self):
        self._running = False
        seq = self._seq
        try:
            self._send(
                f'{{"seq":{seq},"address":1,"funcode":4,"data":{{"id":6,"mode":0}}}}')
            self._send(
                f'{{"seq":{seq+1},"address":1,"funcode":5,"data":{{"mode":0}}}}')
            self.get_logger().info('Motors disabled.')
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = D1Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()