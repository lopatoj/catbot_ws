import rclpy
from rclpy.node import Node
from .kinematics import FKController
from sensor_msgs.msg import Joy
from catbot_msgs.msg import Control
from sensor_msgs.msg import JointState
from odrive.enums import ControlMode
import numpy as np
from collections.abc import Callable
import typing


class JumpNode(Node):
    def __init__(self):
        super().__init__("jump_node")

        self.dt = self.declare_parameter("dt", 0.01).value
        self.max_torque = self.declare_parameter("max_torque", 10.0).value

        # setpoint angles in radians
        self.min_angle0 = self.declare_parameter(
            "min_angle0", 2.70526030718
        ).value
        self.min_angle1 = self.declare_parameter(
            "min_angle1", 5.84685330718
        ).value
        self.mid_angle0 = self.declare_parameter(
            "mid_angle0", 3.49065847058
        ).value
        self.mid_angle1 = self.declare_parameter(
            "mid_angle1", 5.45415422548
        ).value
        self.max_angle0 = self.declare_parameter(
            "max_angle0", 4.43359265359
        ).value
        self.max_angle1 = self.declare_parameter(
            "max_angle1", 5.06159265359
        ).value

        # initalizes fields corresponding to each parameter
        self.update_parameters()

        a1 = 0.129
        a2 = 0.080
        a3 = 0.104
        a4 = 0.180
        l1 = 0.225
        l2 = 0.159

        self.fk = FKController(a1, a2, a3, a4, l1, l2)

        self.control = Control()
        self.control.control_mode = ControlMode.TORQUE_CONTROL
        self.control.values = [0.0, 0.0]
        self.control_pub = self.create_publisher(Control, "/control_cmd", 10)

        self.joint_states = JointState()
        self.joint_states_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_states_callback, 10
        )

        self.joy = self.create_subscription(Joy, "/joy", self._joy_callback, 10)

        # defines the sequence of phases for a jump
        # each function runs an iteration of its phase, and returns a boolean
        # value corresponding to whether the node should move to the next phase
        self.phases: list[Callable[[], bool]] = [
            # self.update_parameters,  # should always be first, since phases depend on parameters
            self.poising_phase,
            self.jumping_phase,
            self.landing_phase,
        ]

        self.current_phase = 0

        self.create_timer(self.dt, self._step_callback)

    def next_phase(self):
        self.current_phase = (self.current_phase + 1) % len(self.phases)

    def _step_callback(self):
        if self.phases[self.current_phase]():
            self.next_phase()

    def _joy_callback(self, msg: Joy):
        if msg.buttons[1] == 1 and self.current_phase == 0:
            self.current_phase = 1
        if msg.buttons[2] == 1:
            self.current_phase = 0

    def _joint_states_callback(self, msg: JointState):
        self.joint_states = msg

    def update_parameters(self):
        """Updates all parameters for this node. Should be called at the beginning of each jump."""
        self.get_logger().info("updating parameter values", once=True)

        for p in self._parameters:
            self.__setattr__(p, self.get_parameter(p).value)

        return False

    def poising_phase(self):
        """Moves both linkages to their minimum positions, in preparation for the jump."""
        self.get_logger().info("starting poising_phase")

        self.control.control_mode = ControlMode.POSITION_CONTROL
        self.control.values = [self.min_angle0, self.min_angle1]
        self.control_pub.publish(self.control)

        return False

    def jumping_phase(self):
        """Calculates leg jacobian based on current motor angles, and uses transpose of
        jacobian to calculate torques required to exert downward force at the foot. Then
        scales up this torque vector to maximum torque limits.
        """
        self.get_logger().info("starting jumping_phase")

        th1 = self.joint_states.position[0]
        th2 = self.joint_states.position[1]

        try:
            torques = (
                self.fk.jacobian(th1, th2).T @ np.array([[0], [-1]])
            ).flatten()

            torques *= self.max_torque / abs(max(torques, key=abs))

            self.control.control_mode = ControlMode.TORQUE_CONTROL
            self.control.values = torques.tolist()
            self.control_pub.publish(self.control)
        except:
            return True

        return self.is_about(self.max_angle0, self.max_angle1)

    def landing_phase(self):
        """Moves both linkages back to their default positions."""
        self.get_logger().info("starting landing_phase")

        self.control.control_mode = ControlMode.TORQUE_CONTROL
        self.control.values = [0.0, 0.0]
        self.control_pub.publish(self.control)

        return True

    def is_about(self, a0, a1):
        return (
            abs(self.joint_states.position[0] - a0) < 0.1
            or abs(self.joint_states.position[1] - a1) < 0.1
        )


def main(args=None):
    rclpy.init(args=args)
    jump_node = JumpNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    rclpy.spin(jump_node, executor=executor)
    jump_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
