import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import CameraInfo, Image
from yolo_msgs.msg import DetectionArray

class YoloProcessingNode(Node):
    def __init__(self):
        super().__init__('yolo_processing_node')
        
        self.bridge = CvBridge()
        # Odmah inicijaliziraj sliku — ne čekaj camera_info
        self.image = np.zeros((480, 640, 3), dtype=np.uint8)

        self.declare_parameter('camera_info_topic', 'camera/camera_info')
        self.declare_parameter('yolo_detections_topic', '/yolo/detections')
        self.declare_parameter('output_topic', 'detection_boxes')

        info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        yolo_topic = self.get_parameter('yolo_detections_topic').get_parameter_value().string_value
        out_topic  = self.get_parameter('output_topic').get_parameter_value().string_value

        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self.camera_info_callback, 10
        )
        self.yolo_sub = self.create_subscription(
            DetectionArray, yolo_topic, self.yolo_callback, 10
        )
        self.image_pub = self.create_publisher(Image, out_topic, 10)

    def camera_info_callback(self, msg):
        self.image = np.zeros((msg.height, msg.width, 3), dtype=np.uint8)
        self.get_logger().info(f"Slika inicijalizirana: {msg.width}x{msg.height}")
        self.destroy_subscription(self.info_sub)

    def yolo_callback(self, msg):
        if self.image is None:
            self.get_logger().warn("Skipping — slika još nije inicijalizirana")
            return

        ALLOWED_IDS = {56, 64}  # chair=56, mouse=64

        out_image = self.image.copy()

        for i, detection in enumerate(msg.detections):

            # 1. Pokušaj dohvatiti numerički ID
            class_id = None
            if hasattr(detection, 'class_id'):
                class_id = detection.class_id
            elif hasattr(detection, 'id'):
                class_id = detection.id

            # 2. Pokušaj dohvatiti ime klase
            class_name = ""
            if hasattr(detection, 'class_name'):
                class_name = str(detection.class_name).strip().lower()

            # 3. Filter — continue ako nije chair ili mouse
            if class_id is not None:
                if int(class_id) not in ALLOWED_IDS:
                    continue
            else:
                if class_name not in {'chair', 'mouse'}:
                    continue

            # 4. Debug — ispiši samo što je prošlo filter
            self.get_logger().info(f"Filtrirano: [{class_name}] [{class_id}]")

            # 5. Koordinate
            cx = detection.bbox.center.position.x
            cy = detection.bbox.center.position.y
            w  = detection.bbox.size.x
            h  = detection.bbox.size.y

            x1 = int(cx - w / 2)
            y1 = int(cy - h / 2)
            x2 = int(cx + w / 2)
            y2 = int(cy + h / 2)

            start_point = (x1, y1)
            end_point   = (x2, y2)

            self.get_logger().info(
                f"[{class_name}] TL={start_point} BR={end_point}"
            )

            color = (
                int((i * 75)  % 255),
                int((i * 150) % 255),
                int((i * 225) % 255)
            )

            cv2.rectangle(out_image, start_point, end_point, color, thickness=3)
            cv2.putText(
                out_image,
                f"{class_name} ({class_id})",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1
            )

        # Publish — jednom, izvan for petlje
        output_msg = self.bridge.cv2_to_imgmsg(out_image, encoding="bgr8")
        self.image_pub.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloProcessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()