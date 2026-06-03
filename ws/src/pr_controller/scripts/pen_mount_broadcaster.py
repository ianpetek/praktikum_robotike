#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PenMountBroadcaster(Node):
    def __init__(self):
        super().__init__('pen_mount_broadcaster')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(JointState, '/joint_states', self._cb, 10)

    def _cb(self, msg: JointState):
        if 'pen_mount_joint' in msg.name:
            return  # avoid feedback loop from our own published messages
        try:
            b = msg.position[msg.name.index('bicep_joint')]
            f = msg.position[msg.name.index('forearm_joint')]
        except ValueError:
            return
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
