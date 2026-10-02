import threading
from dataclasses import dataclass
from typing import List, Tuple

from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import float32
from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader


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


class D1Arm:
    def __init__(self):
        self._seq = 0
        self._lock = threading.Lock()

        self._dp = DomainParticipant(0)
        # If a DomainParticipant has not been created, do so with the updated configuration
        #if DDSCommunicator._participant is None:
            # Directly use the path to the XML configuration file when creating the Domain
            #DDSCommunicator._participant = DomainParticipant(domain_id=0)

        cmd_topic = Topic(self._dp, 'rt/arm_Command', ArmString_)
        self._writer = DataWriter(self._dp, cmd_topic)

        state_topic = Topic(self._dp, 'current_servo_angle', PubServoInfo_)
        self._reader = DataReader(self._dp, state_topic)

    def enable_motors(self):
        self._send(5, '{"mode":80000}')

    def disable_motors(self):
        self._send(5, '{"mode":0}')

    def enable_gripper(self):
        self._send(4, '{"id":6,"mode":80000}')

    def disable_gripper(self):
        self._send(4, '{"id":6,"mode":0}')

    def open_gripper(self):
        self._send(2, '{"mode":1,"angle6":65,"delay_ms":2000}')

    def close_gripper(self):
        self._send(2, '{"mode":1,"angle6":0.0,"delay_ms":2000}')

    def zero(self):
        self._send(7)

    def move_joints(self, joint_deg: List[float], gripper_mm: float):
        """joint_deg: [Joint1..Joint6] en grados. gripper_mm: 0 (cerrada) a 65 (abierta)."""
        assert len(joint_deg) == 6
        angles = ','.join(f'"angle{i}":{deg:.2f}' for i, deg in enumerate(joint_deg))
        self._send(2, f'{{"mode":1,{angles},"angle6":{gripper_mm:.2f}}}')

    def read_joints(self) -> List[Tuple[List[float], float]]:
        samples = []
        for sample in self._reader.take(10):
            if not sample.sample_info.valid_data:
                continue
            joint_deg = [
                float(sample.servo0_data_), float(sample.servo1_data_),
                float(sample.servo2_data_), float(sample.servo3_data_),
                float(sample.servo4_data_), float(sample.servo5_data_),
            ]
            gripper_mm = float(sample.servo6_data_)
            samples.append((joint_deg, gripper_mm))
        return samples
    
    def _send(self, funcode: int, data: str = ''):
        with self._lock:
            self._seq += 1
            seq = self._seq
            data_part = f',"data":{data}' if data else ''
            json_str = f'{{"seq":{seq},"address":1,"funcode":{funcode}{data_part}}}'
            self._writer.write(ArmString_(data_=json_str))

    def shutdown(self):
        try:
            self.disable_gripper()
            self.disable_motors()
        except Exception:
            pass
