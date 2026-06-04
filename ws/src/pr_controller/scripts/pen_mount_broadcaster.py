#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class PenMountBroadcaster(Node):
    def __init__(self):
        super().__init__('pen_mount_broadcaster')
        self.declare_parameter('use_sim', False)
        self._use_sim = self.get_parameter('use_sim').get_parameter_value().bool_value

        if self._use_sim:
            self.pub = self.create_publisher(Float64MultiArray, '/pen_mount_controller/commands', 10)
        else:
            self.pub = self.create_publisher(JointState, '/joint_states', 10)

        self.create_subscription(JointState, '/joint_states', self._cb, 10)

    def _cb(self, msg: JointState):
        if not self._use_sim and 'pen_mount_joint' in msg.name:
            return  # avoid feedback loop from our own published message (hardware only)
        try:
            b = msg.position[msg.name.index('bicep_joint')]
            f = msg.position[msg.name.index('forearm_joint')]
        except ValueError:
            return

        if self._use_sim:
            out = Float64MultiArray()
            out.data = [b + f]
        else:
            out = JointState()
            out.header.stamp = msg.header.stamp
            out.name = ['pen_mount_joint']
            out.position = [b + f]

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PenMountBroadcaster())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
