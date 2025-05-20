import rclpy
import pybullet as p
import pybullet_data

from rclpy import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class PyBulletNode(Node):
    def __init__(self):
        super().__init__("pybullet_node")
        
        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)

        self.plane_id = p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF(
            "install/catbot_pybullet/share/catbot_pybullet/description/robot.urdf",
            [0, 0, 0],
            p.getQuaternionFromEuler([0, 0, 0]),
            flags=p.URDF_USE_INERTIA_FROM_FILE | p.URDF_MAINTAIN_LINK_ORDER,
        )
        self.num_joints = p.getNumJoints(bodyUniqueId=self.robot_id)

        self.joint_states = JointState()

        for i in range(self.num_joints):
            joint_info = p.getJointInfo(
                bodyUniqueId=self.robot_id, jointIndex=i
            )
            self.joint_states.name.append(str(joint_info[1]))
            self.joint_states.position.append(0.0)
            self.joint_states.velocity.append(0.0)
            self.joint_states.effort.append(0.0)

        p.setJointMotorControlArray(
            bodyIndex=self.robot_id,
            jointIndices=range(self.num_joints),
            controlMode=p.VELOCITY_CONTROL,
            forces=[0.0] * self.num_joints,
        )

        self.cmd_sub = self.create_subscription(
            Float64MultiArray, "ctrl_cmd", self._cmd_callback, 10
        )
        self.joint_states_pub = self.create_publisher(
            JointState, "joint_states", 10
        )

        self.create_timer(self.get_parameter("dt").value, self._step_callback)
        
    def _step_callback(self):
        p.setJointMotorControlArray(
            bodyIndex=self.robot_id,
            jointIndices=range(self.num_joints),
            controlMode=p.TORQUE_CONTROL,
            forces=self.joint_states.effort,
        )

        p.stepSimulation()

        for i in range(self.num_joints):
            joint_state = p.getJointState(
                bodyUniqueId=self.robot_id, jointIndex=i
            )
            self.joint_states.position[i] = joint_state[0]
            self.joint_states.velocity[i] = joint_state[1]

        self.joint_states_pub.publish(self.joint_states)

    def _cmd_callback(self, msg: Float64MultiArray):
        self.joint_states.effort = msg.data


def main():
    rclpy.init()
    node = PyBulletNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
