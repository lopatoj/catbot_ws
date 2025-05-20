import rclpy
import pybullet as p
import pybullet_data
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from catbot_msgs.msg import Control
from odrive.enums import ControlMode


class PyBulletNode(Node):
    def __init__(self):
        super().__init__("pybullet_node")

        self.dt = self.declare_parameter("dt", 0.01).value
        self.max_torque = self.declare_parameter("max_torque", 10.0).value
        self.angle_offsets = self.declare_parameter(
                "angle_offsets", [3.141592653, 5.23718530718]
            ).value

        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)

        self.plane_id = p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF(
            "install/catbot_pybullet/share/catbot_pybullet/description/robot.urdf",
            [0, 0, 0.5],
            p.getQuaternionFromEuler([0, 0, 0]),
            flags=p.URDF_MAINTAIN_LINK_ORDER,
        )
        self.num_joints = p.getNumJoints(bodyUniqueId=self.robot_id)
        
        self.num_motors = 2
        self.motor_indices = [0, 0]

        self.joint_states = JointState()
        self.joint_states.name = ["motor1", "motor2"]
        self.joint_states.position = self.angle_offsets
        self.joint_states.velocity = [0.0, 0.0]
        self.joint_states.effort = [0.0, 0.0]
        
        self.joint_name_to_index = {}

        for i in range(self.num_joints):
            joint_info = p.getJointInfo(
                bodyUniqueId=self.robot_id, jointIndex=i
            )
            name: str = joint_info[1].decode("utf-8").strip()
            self.joint_name_to_index[name] = i

            if name == "motor1":
                self.motor_indices[0] = i
            if name == "motor2":
                self.motor_indices[1] = i

        self.get_logger().info(f"joints: {self.joint_name_to_index}")
        self.get_logger().info(f"motor indices: {self.motor_indices}")
        self.get_logger().info(f"initial motor state: {self.joint_states}")

        self.zeros = [0.0] * 2
        self.max_torques = [self.max_torque] * 2

        p.createConstraint(
            self.robot_id,
            self.joint_name_to_index["closing_passive5_1_frame"],
            self.robot_id,
            self.joint_name_to_index["closing_passive5_2_frame"],
            p.JOINT_POINT2POINT,
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
        )

        p.createConstraint(
            self.robot_id,
            self.joint_name_to_index["closing_passive3_1_frame"],
            self.robot_id,
            self.joint_name_to_index["closing_passive3_2_frame"],
            p.JOINT_POINT2POINT,
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
        )

        p.createConstraint(
            self.plane_id,
            -1,
            self.robot_id,
            -1,
            p.JOINT_PRISMATIC,
            [0, 0, 1],
            [0, 0, 0],
            [0, 0, 0],
        )

        p.setJointMotorControlArray(
            bodyIndex=self.robot_id,
            jointIndices=range(self.num_joints),
            controlMode=p.VELOCITY_CONTROL,
            forces=[0.0] * self.num_joints,
        )

        self.control = Control()
        self.control.control_mode = ControlMode.TORQUE_CONTROL
        self.control.values = self.zeros
        self.control_sub = self.create_subscription(
            Control, "/control_cmd", self._control_callback, 10
        )

        self.joint_states_pub = self.create_publisher(
            JointState, "/joint_states", 10
        )

        self.create_timer(self.dt, self._step_callback)

    def _step_callback(self):
        if self.control.control_mode == ControlMode.POSITION_CONTROL:
            p.setJointMotorControlArray(
                bodyIndex=self.robot_id,
                jointIndices=self.motor_indices,
                controlMode=p.POSITION_CONTROL,
                targetPositions=np.array(self.control.values) - self.angle_offsets,
                forces=self.max_torques,
            )
        if self.control.control_mode == ControlMode.VELOCITY_CONTROL:
            p.setJointMotorControlArray(
                bodyIndex=self.robot_id,
                jointIndices=self.motor_indices,
                controlMode=p.VELOCITY_CONTROL,
                targetPositions=self.control.values,
                forces=self.max_torques,
            )
        if self.control.control_mode == ControlMode.TORQUE_CONTROL:
            p.setJointMotorControlArray(
                bodyIndex=self.robot_id,
                jointIndices=range(self.num_joints),
                controlMode=p.VELOCITY_CONTROL,
                forces=[0.0] * self.num_joints,
            )
            p.setJointMotorControlArray(
                bodyIndex=self.robot_id,
                jointIndices=range(self.num_joints),
                controlMode=p.POSITION_CONTROL,
                forces=[0.0] * self.num_joints,
            )
            self.get_logger().info(f"{self.control.values}")
            p.setJointMotorControlArray(
                bodyIndex=self.robot_id,
                jointIndices=self.motor_indices,
                controlMode=p.TORQUE_CONTROL,
                forces=self.control.values,
            )

        p.stepSimulation()

        for i in range(self.num_motors):
            joint_state = p.getJointState(
                bodyUniqueId=self.robot_id, jointIndex=self.motor_indices[i]
            )
            
            self.joint_states.position[i] = joint_state[0] + self.angle_offsets[i]
            self.joint_states.velocity[i] = joint_state[1]
            self.joint_states.effort[i] = joint_state[3]

        self.joint_states_pub.publish(self.joint_states)

    def _control_callback(self, msg: Control):
        self.control = msg


def main():
    rclpy.init()
    node = PyBulletNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
