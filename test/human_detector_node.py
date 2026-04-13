import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
import cv2
import numpy as np
from ultralytics import YOLO


def frame_to_imgmsg(frame, stamp, frame_id='camera'):
    """Convert a BGR numpy frame to a sensor_msgs/Image without cv_bridge."""
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = frame.shape[0]
    msg.width  = frame.shape[1]
    msg.encoding = 'bgr8'
    msg.is_bigendian = False
    msg.step = frame.shape[1] * 3
    msg.data = frame.tobytes()
    return msg


class HumanDetectorNode(Node):
    def __init__(self):
        super().__init__('human_detector')

        self.declare_parameter('model_path',   'yolov8n.pt')
        self.declare_parameter('confidence',    0.5)
        self.declare_parameter('camera_index',  0)
        self.declare_parameter('video_path',   '')

        model_path = self.get_parameter('model_path').value
        self.conf  = self.get_parameter('confidence').value
        cam_index  = self.get_parameter('camera_index').value
        video_path = self.get_parameter('video_path').value

        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self.model = YOLO(model_path)

        if video_path:
            self.get_logger().info(f'Using video file: {video_path}')
            self.cap = cv2.VideoCapture(video_path)
        else:
            self.get_logger().info(f'Using webcam index: {cam_index}')
            self.cap = cv2.VideoCapture(cam_index)

        if not self.cap.isOpened():
            self.get_logger().error('Cannot open video source!')
            raise RuntimeError('Video source not found')

        self.img_pub = self.create_publisher(Image,            '/human_detection/image',      10)
        self.det_pub = self.create_publisher(Detection2DArray, '/human_detection/detections', 10)

        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info('Human Detector Node started.')

    def timer_callback(self):
        ret, frame = self.cap.read()

        # Loop video when it ends
        if not ret:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()
            if not ret:
                return

        results = self.model(frame, conf=self.conf, verbose=False)[0]
        stamp   = self.get_clock().now().to_msg()

        # Build and publish detections
        det_array            = Detection2DArray()
        det_array.header.stamp    = stamp
        det_array.header.frame_id = 'camera'

        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            conf = float(box.conf[0])
            cls  = int(box.cls[0])

            det = Detection2D()
            det.header = det_array.header
            det.bbox.center.position.x = (x1 + x2) / 2.0
            det.bbox.center.position.y = (y1 + y2) / 2.0
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(cls)
            hyp.hypothesis.score    = conf
            det.results.append(hyp)

            det_array.detections.append(det)

        self.det_pub.publish(det_array)

        # Annotate frame and publish image (no cv_bridge needed)
        annotated = results.plot()
        self.img_pub.publish(frame_to_imgmsg(annotated, stamp))

        # Show locally
        cv2.imshow('Human Detection', annotated)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HumanDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()