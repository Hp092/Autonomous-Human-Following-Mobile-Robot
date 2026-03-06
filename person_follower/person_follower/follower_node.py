import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FollowerNode(Node):
    """Minimal ROS 2 node to satisfy Milestone 1 package-structure requirement.

    Replace this with your perception/tracking/follow-control pipeline later.
    """

    def __init__(self):
        super().__init__('follower_node')
        self.pub = self.create_publisher(String, 'person_follower/status', 10)
        self.timer = self.create_timer(1.0, self._tick)
        self.counter = 0
        self.get_logger().info('person_follower follower_node started')

    def _tick(self):
        self.counter += 1
        msg = String()
        msg.data = f'heartbeat {self.counter}'
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
