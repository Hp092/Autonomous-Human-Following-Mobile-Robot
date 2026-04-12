import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header


class SimPersonPublisher(Node):
    def __init__(self):
        super().__init__('sim_person_publisher')

        self.declare_parameter('center_x', 320.0)
        self.declare_parameter('center_y', 240.0)
        self.declare_parameter('box_width', 100.0)
        self.declare_parameter('box_height', 200.0)
        self.declare_parameter('publish_hz', 10.0)

        self.pub = self.create_publisher(Detection2D, '/detections', 10)

        hz = self.get_parameter('publish_hz').value
        self.timer = self.create_timer(1.0 / hz, self.publish_detection)

        self.get_logger().info('SimPersonPublisher started')

    def publish_detection(self):
        cx = self.get_parameter('center_x').value
        cy = self.get_parameter('center_y').value
        bw = self.get_parameter('box_width').value
        bh = self.get_parameter('box_height').value

        msg = Detection2D()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'

        msg.bbox.center.position.x = cx
        msg.bbox.center.position.y = cy
        msg.bbox.size_x = bw
        msg.bbox.size_y = bh

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = 'person'
        hyp.hypothesis.score = 1.0
        msg.results.append(hyp)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimPersonPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()