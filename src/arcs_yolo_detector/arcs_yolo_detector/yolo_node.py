import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import onnxruntime as ort

class YoloOnnxNode(Node):
    def __init__(self):
        super().__init__('yolo_onnx_node')
        
        # Publisher for detections
        self.pub = self.create_publisher(Detection2DArray, '/yolo_detections', 10)
        
        # Subscribe to ZED camera images
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_callback,
            10
        )
        
        # CV Bridge for converting ROS images to OpenCV
        self.bridge = CvBridge()
        
        # Load ONNX model
        self.model_path = "/ros2_ws/onnx/model.onnx"
        self.session = ort.InferenceSession(self.model_path, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        
        self.get_logger().info("YOLO ONNX Node started - waiting for ZED camera images...")

    def preprocess(self, frame):
        img = cv2.resize(frame, (640, 640))
        img = img[:, :, ::-1]  # BGR → RGB
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # CHW
        img = np.expand_dims(img, axis=0)
        return img

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run inference
            input_tensor = self.preprocess(frame)
            outputs = self.session.run(None, {self.input_name: input_tensor})[0]
            
            # Create detection message
            detection_msg = Detection2DArray()
            detection_msg.header = msg.header
            
            for det in outputs:
                x1, y1, x2, y2, score, class_id = det[:6]
                if score < 0.5:
                    continue
                
                detection = Detection2D()
                detection.bbox.center.position.x = float((x1 + x2) / 2)
                detection.bbox.center.position.y = float((y1 + y2) / 2)
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)
                
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(int(class_id))
                hypothesis.hypothesis.score = float(score)
                detection.results.append(hypothesis)
                
                detection_msg.detections.append(detection)
            
            self.pub.publish(detection_msg)
            self.get_logger().info(f'Published {len(detection_msg.detections)} detections')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloOnnxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
