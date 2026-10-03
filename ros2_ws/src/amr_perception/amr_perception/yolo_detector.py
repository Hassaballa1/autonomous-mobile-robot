"""YOLOv8 object detection on the robot camera.

Subscribes:  image (sensor_msgs/Image)
Publishes:   detections (vision_msgs/Detection2DArray), detections/image (annotated, optional)

Inference runs in its own thread on the newest frame only, so a slow model on the Pi
drops frames instead of building up latency.
"""
import os
import threading
import time

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')
        model_path = self.declare_parameter('model', 'yolov8n-oiv7.pt').value
        model_dir = os.path.expanduser(
            self.declare_parameter('model_dir', '~/.cache/amr_perception').value)
        self.confidence = self.declare_parameter('confidence', 0.4).value
        self.image_size = self.declare_parameter('image_size', 320).value
        self.device = self.declare_parameter('device', 'cpu').value
        self.max_rate = self.declare_parameter('max_rate_hz', 5.0).value
        self.publish_annotated = self.declare_parameter('publish_annotated', True).value

        # A bare model name is kept in model_dir, so it is downloaded once and found again
        # regardless of the working directory the node was started from (e.g. systemd)
        if not os.path.dirname(model_path):
            os.makedirs(model_dir, exist_ok=True)
            model_path = os.path.join(model_dir, model_path)
        self.get_logger().info(f'Loading model {model_path}')
        self.model = YOLO(model_path, task='detect')
        self.bridge = CvBridge()

        self.latest = None
        self.frame_ready = threading.Condition()

        self.det_pub = self.create_publisher(Detection2DArray, 'detections', 10)
        self.img_pub = self.create_publisher(Image, 'detections/image', 1)
        self.create_subscription(Image, 'image', self.on_image, qos_profile_sensor_data)

        self.running = True
        self.worker = threading.Thread(target=self.inference_loop, daemon=True)
        self.worker.start()

    def on_image(self, msg):
        with self.frame_ready:
            self.latest = msg
            self.frame_ready.notify()

    def inference_loop(self):
        period = 1.0 / self.max_rate if self.max_rate > 0 else 0.0
        while self.running and rclpy.ok():
            with self.frame_ready:
                self.frame_ready.wait_for(lambda: self.latest is not None or not self.running,
                                          timeout=1.0)
                msg, self.latest = self.latest, None
            if msg is None:
                continue

            started = time.monotonic()
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            result = self.model.predict(
                frame, imgsz=self.image_size, conf=self.confidence,
                device=self.device, verbose=False)[0]
            self.publish(msg.header, result)

            elapsed = time.monotonic() - started
            self.get_logger().debug(f'{len(result.boxes)} detections in {elapsed * 1000:.0f} ms')
            if elapsed < period:
                time.sleep(period - elapsed)

    def publish(self, header, result):
        out = Detection2DArray()
        out.header = header
        names = result.names
        for xywh, cls, score in zip(result.boxes.xywh.tolist(),
                                    result.boxes.cls.tolist(),
                                    result.boxes.conf.tolist()):
            det = Detection2D()
            det.header = header
            det.bbox.center.position.x = xywh[0]
            det.bbox.center.position.y = xywh[1]
            det.bbox.size_x = xywh[2]
            det.bbox.size_y = xywh[3]
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = names[int(cls)]
            hyp.hypothesis.score = float(score)
            det.results.append(hyp)
            out.detections.append(det)
        self.det_pub.publish(out)

        if self.publish_annotated and self.img_pub.get_subscription_count() > 0:
            annotated = self.bridge.cv2_to_imgmsg(result.plot(), encoding='bgr8')
            annotated.header = header
            self.img_pub.publish(annotated)

    def destroy_node(self):
        self.running = False
        with self.frame_ready:
            self.frame_ready.notify()
        super().destroy_node()


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
