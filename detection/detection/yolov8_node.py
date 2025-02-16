#!/usr/bin/env python3
"""
Ultra-Optimized YOLOv8 ROS2 Node for CPU-Only Real-Time Inference (Revised)
Key Improvements:
  - Robust ONNX session creation with error handling
  - Clear preprocessing, inference, and postprocessing pipelines
  - Safe asynchronous execution with proper thread pool shutdown
  - Improved visualization with a fixed color palette and bounds checking
  - Detailed logging and parameter configuration
  - Corrected objectness logic for YOLOv8 (no separate objectness output)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesis
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from typing import List, Tuple
import onnxruntime as ort
import concurrent.futures
import threading
import os
import math
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Pose2D


class YOLOv8Engine:
    """Optimized Inference Engine with ONNX Runtime for YOLOv8."""
    def __init__(self, model_path: str, conf_thresh: float = 0.0, iou_thresh: float = 0.45):
        try:
            options = ort.SessionOptions()
            # These options may be made configurable
            options.intra_op_num_threads = 4
            options.inter_op_num_threads = 4
            options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
            options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

            self.session = ort.InferenceSession(model_path, sess_options=options,
                                                providers=['CPUExecutionProvider'])
        except Exception as e:
            raise RuntimeError(f"Failed to load ONNX model from {model_path}: {str(e)}")

        # Retrieve fixed input shape: assume [batch, channels, height, width]
        inputs = self.session.get_inputs()
        for inp in inputs:
            print(f"Input name: {inp.name}, shape: {inp.shape}")
        input_shape = self.session.get_inputs()[0].shape
        if input_shape[2] is None or input_shape[3] is None:
            raise ValueError("Dynamic input shapes are not supported. Please use a fixed model input size.")
        self.input_shape = (input_shape[2], input_shape[3])  # (height, width)

        self.classes = self._load_class_names() 
        self.conf_thresh = conf_thresh
        self.iou_thresh = iou_thresh

        # Set a default sigma for soft NMS; can be made configurable.
        self.soft_nms_sigma = 0.5

        # Pre-allocate input buffer (used via np.copyto in infer)
        self.input_buffer = np.zeros((1, 3, self.input_shape[0], self.input_shape[1]), dtype=np.float32)

    def _load_class_names(self) -> List[str]:
        """Load class names from file or use default dummy names."""
        try:
            # Determine the directory of the current file
            base_dir = os.path.dirname(os.path.abspath(__file__))
            file_path = os.path.join(base_dir, "yolov8_class.txt")
            
            if os.path.isfile(file_path):
                with open(file_path, "r") as f:
                    # Read all non-empty lines, stripping whitespace
                    names = [line.strip() for line in f if line.strip()]
                if names:
                    msg = "Loaded class names: " + ", ".join(names)
                    if hasattr(self, "get_logger"):
                        self.get_logger().info(msg)
                    else:
                        print(msg)
                    return names
                else:
                    msg = "yolov8_class.txt is empty. Using default dummy names."
                    if hasattr(self, "get_logger"):
                        self.get_logger().warn(msg)
                    else:
                        print("Warning:", msg)
            else:
                msg = f"yolov8_class.txt not found at {file_path}. Using default dummy names."
                if hasattr(self, "get_logger"):
                    self.get_logger().warn(msg)
                else:
                    print("Warning:", msg)
        except Exception as e:
            msg = f"Error loading class names: {e}. Using default dummy names."
            if hasattr(self, "get_logger"):
                self.get_logger().error(msg)
            else:
                print("Error:", msg)

        # Fallback: default dummy names for 80 classes
        fallback = [str(i) for i in range(80)]
        msg = "Using fallback class names: " + ", ".join(fallback)
        if hasattr(self, "get_logger"):
            self.get_logger().info(msg)
        else:
            print(msg)
        return fallback

    def preprocess(self, image: np.ndarray) -> np.ndarray:
        """Preprocess image: resize, pad, normalize, and transpose."""
        original_h, original_w = image.shape[:2]
        target_h, target_w = self.input_shape
        scale = min(target_h / original_h, target_w / original_w)
        new_w = int(original_w * scale)
        new_h = int(original_h * scale)
        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        # Calculate padding amounts (pad on bottom and right)
        pad_w = target_w - new_w
        pad_h = target_h - new_h
        padded = cv2.copyMakeBorder(resized, 0, pad_h, 0, pad_w,
                                    borderType=cv2.BORDER_CONSTANT, value=(114, 114, 114))
        normalized = padded.astype(np.float32) / 255.0
        # Convert to CHW format and add a batch dimension
        transposed = np.transpose(normalized, (2, 0, 1))
        preprocessed = np.expand_dims(transposed, axis=0)
        return preprocessed

    def infer(self, preprocessed: np.ndarray) -> np.ndarray:
        """Run inference on preprocessed image and return raw predictions.
           Expected raw output shape from the model: (1, 84, num_candidates)
           where 84 = 4 (bbox) + 80 (class scores).
        """
        np.copyto(self.input_buffer, preprocessed)
        try:
            outputs = self.session.run(
                output_names=[out.name for out in self.session.get_outputs()],
                input_feed={self.session.get_inputs()[0].name: self.input_buffer}
            )
        except Exception as e:
            raise RuntimeError(f"Inference failed: {str(e)}")

        raw = outputs[0]  # raw shape is (1, 84, num_candidates)
        # Squeeze out the batch dimension and transpose so that shape becomes (num_candidates, 84)
        raw = np.squeeze(raw, axis=0)      # now shape is (84, num_candidates)
        raw = raw.transpose(1, 0)           # now shape is (num_candidates, 84)
        return raw

    def _compute_iou(self, det_a: np.ndarray, det_b: np.ndarray) -> float:
        """
        Compute Intersection over Union (IoU) between two detections.
        Each detection is in the format: [x_center, y_center, width, height, conf, ...]
        """
        try:
            xa, ya, wa, ha = float(det_a[0]), float(det_a[1]), float(det_a[2]), float(det_a[3])
            xb, yb, wb, hb = float(det_b[0]), float(det_b[1]), float(det_b[2]), float(det_b[3])
        except (IndexError, ValueError) as e:
            if hasattr(self, "get_logger"):
                self.get_logger().error(f"Invalid detection format in _compute_iou: {e}")
            return 0.0

        # Ensure width and height are positive.
        if wa <= 0 or ha <= 0 or wb <= 0 or hb <= 0:
            return 0.0

        # Convert center coordinates to corner coordinates: (x1, y1, x2, y2)
        a_x1, a_y1 = xa - wa / 2.0, ya - ha / 2.0
        a_x2, a_y2 = xa + wa / 2.0, ya + ha / 2.0
        b_x1, b_y1 = xb - wb / 2.0, yb - hb / 2.0
        b_x2, b_y2 = xb + wb / 2.0, yb + hb / 2.0

        # Compute intersection rectangle.
        inter_x1 = max(a_x1, b_x1)
        inter_y1 = max(a_y1, b_y1)
        inter_x2 = min(a_x2, b_x2)
        inter_y2 = min(a_y2, b_y2)
        inter_w = max(0.0, inter_x2 - inter_x1)
        inter_h = max(0.0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h

        area_a = wa * ha
        area_b = wb * hb
        union_area = area_a + area_b - inter_area
        if union_area <= 0:
            return 0.0
        return inter_area / union_area

    def _soft_nms(self, detections: np.ndarray) -> np.ndarray:
        """
        Apply Soft Non-Maximum Suppression (Soft-NMS) to detections.
        Detections are expected in the format:
            [x_center, y_center, width, height, conf, class_scores...]
        where the length is 85 (4 bbox + 1 confidence + 80 class scores).
        This method decays the confidence scores of overlapping detections using a Gaussian penalty.
        """
        if detections.ndim != 2 or detections.shape[1] < 6:
            if hasattr(self, "get_logger"):
                self.get_logger().error("Invalid detection format in _soft_nms.")
            return np.empty((0, detections.shape[1]))

        # Make a copy to avoid modifying original detections.
        D = detections.copy()
        N = D.shape[0]
        # Sort detections by confidence (index 4) in descending order.
        order = D[:, 4].argsort()[::-1]
        D = D[order]

        for i in range(N):
            for j in range(i + 1, N):  # j = i+1 has lower confidence
                iou = self._compute_iou(D[i], D[j])
                if iou > self.iou_thresh:
                    # Decay the confidence score of detection j using a Gaussian penalty.
                    decay = math.exp(- (iou * iou) / self.soft_nms_sigma)
                    D[j, 4] *= decay

        # Filter out detections with decayed confidence below the threshold.
        keep = D[:, 4] >= self.conf_thresh
        D = D[keep]

        # Optionally re-sort by confidence in descending order.
        if D.shape[0] > 0:
            order = D[:, 4].argsort()[::-1]
            D = D[order]

        return D

    def postprocess(self, predictions: np.ndarray) -> List[Detection2D]:
        """
        Convert raw predictions into Detection2D messages.
        For YOLOv8, each prediction vector is of length 84:
            [x_center, y_center, width, height, class_scores (80 values)]
        We first compute the confidence as the maximum class score.
        Then, we filter out predictions below the confidence threshold and apply soft NMS.
        Finally, we create Detection2D messages.
        """
        # Check for expected prediction shape
        if predictions.ndim != 2 or predictions.shape[1] != 84:
            raise ValueError("Unexpected prediction format from model.")

        # Compute confidence as the max class score for each prediction.
        class_scores = predictions[:, 4:]  # shape: (num_detections, 80)
        max_confidences = np.max(class_scores, axis=1)  # shape: (num_detections,)
        print(max_confidences)

        # Filter out predictions with confidence below threshold.
        conf_mask = max_confidences >= self.conf_thresh
        filtered_preds = predictions[conf_mask]
        if filtered_preds.size == 0:
            print("No detection left after confidence threshold filtering.")
            return []

        # For each remaining prediction, insert the computed confidence at index 4.
        filtered_class_scores = filtered_preds[:, 4:]
        filtered_confidences = np.max(filtered_class_scores, axis=1, keepdims=True)  # shape: (N, 1)
        # Create new prediction vectors in the format:
        # [x_center, y_center, width, height, confidence, class_scores...]
        new_preds = np.concatenate([filtered_preds[:, :4], filtered_confidences, filtered_preds[:, 4:]], axis=1)
        # new_preds shape is now (N, 85)

        # Apply soft NMS on the new predictions.
        nms_preds = self._soft_nms(new_preds)
        if nms_preds.size == 0:
            print("No detection left after soft NMS.")
            return []

        detections = []
        for pred in nms_preds:
            # pred format: [x_center, y_center, width, height, conf, class_scores (80 values)]
            bbox = pred[:4]
            conf = pred[4]
            scores = pred[5:]
            class_id = int(np.argmax(scores))
            det = Detection2D()
            # Assume coordinates are normalized.
            det.bbox.center = Pose2D() 
            det.bbox.center.x = float(bbox[0])
            det.bbox.center.y = float(bbox[1])
            det.bbox.size_x = float(bbox[2])
            det.bbox.size_y = float(bbox[3])
            hypothesis = ObjectHypothesis()
            hypothesis.class_id = str(class_id)
            hypothesis.score = float(conf)
            det.results.append(hypothesis)
            detections.append(det)
        return detections

    def run(self, image: np.ndarray) -> List[Detection2D]:
        """Convenience method: preprocess, run inference, and postprocess."""
        preprocessed = self.preprocess(image)
        print("Preprocessed shape:", preprocessed.shape)
        model_predictions = self.infer(preprocessed)
        print("Model predictions shape:", model_predictions.shape)
        detections = self.postprocess(model_predictions)
        print("Number of detections:", len(detections))
        return detections


class DetectionVisualizer:
    """Helper class for drawing detections on an image."""
    def __init__(self, class_names: List[str]):
        self.class_names = class_names
        # Pre-generate a fixed color for each class
        np.random.seed(42)
        self.colors = {i: tuple(np.random.randint(0, 256, size=3).tolist())
                       for i in range(len(class_names))}

    def draw(self, image: np.ndarray, detections: List[Detection2D]) -> np.ndarray:
        """Draw bounding boxes and labels on the image."""
        output = image.copy()
        height, width = image.shape[:2]
        print(f"Number of detections = {len(detections)}")
        for det in detections:
            # Convert normalized center and size to pixel coordinates
            cx = int(det.bbox.center.x * width)
            cy = int(det.bbox.center.y * height)
            box_w = int(det.bbox.size_x * width / 2)
            box_h = int(det.bbox.size_y * height / 2)
            x1, y1 = max(0, cx - box_w), max(0, cy - box_h)
            x2, y2 = min(width, cx + box_w), min(height, cy + box_h)
            if det.results:
                try:
                    class_id = int(det.results[0].class_id)
                except ValueError:
                    class_id = 0
                score = det.results[0].score
                label = f"{self.class_names[class_id]}: {score:.2f}"
            else:
                label = "Unknown"
                class_id = 0
            color = self.colors.get(class_id, (0, 255, 0))
            cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
            cv2.putText(output, label, (x1, max(y1 - 10, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        return output


class YOLOv8Node(Node):
    """ROS2 Node that wraps the YOLOv8 engine and publishes detections."""
    def __init__(self):
        super().__init__('yolov8_node')
        self.get_logger().info("Detection Node Started.")
        # Declare and read parameters
        self.declare_parameter('model_path', '/home/akarshan/mobile_cobot_ws/yolov8n.onnx')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('max_fps', 30)
        self.declare_parameter('enable_visualization', True)
        # Declare the parameter to control activation of detection
        self.declare_parameter('activate_detection', False)
        
        model_path = self.get_parameter('model_path').value
        conf_thresh = self.get_parameter('conf_threshold').value
        iou_thresh = self.get_parameter('iou_threshold').value
        max_fps = self.get_parameter('max_fps').value
        self.enable_visualization = self.get_parameter('enable_visualization').value
        
        # QoS settings (increased depth for robustness)
        self.qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize core components
        self.bridge = CvBridge()
        try:
            self.engine = YOLOv8Engine(model_path, conf_thresh, iou_thresh)
        except Exception as e:
            self.get_logger().error(str(e))
            rclpy.shutdown()
            return
        self.visualizer = DetectionVisualizer(self.engine.classes)
        self.thread_pool = concurrent.futures.ThreadPoolExecutor(max_workers=2)
        
        # Create publishers for detections and visualization
        self.det_pub = self.create_publisher(Detection2DArray, '/detections', self.qos)
        self.vis_pub = (self.create_publisher(Image, '/detections_visualized', self.qos)
                        if self.enable_visualization else None)
            
        # Timing control for max FPS
        self.max_fps = max_fps
        self.frame_interval_sec = 1.0 / float(max_fps)
        self.last_process_time = self.get_clock().now().nanoseconds / 1e9  # seconds
        self.time_lock = threading.Lock()

        # Initially, create or not the subscription based on the activate_detection parameter.
        if self.get_parameter('activate_detection').value is True:
            self.get_logger().info("Detection activated at startup.")
            self.sub = self.create_subscription(Image, '/arm_rgbd_camera/image_raw', self.image_callback, self.qos)
        else:
            self.get_logger().info("Detection not activated at startup.")
            self.sub = None
        
        # Add parameter callback to monitor changes to 'activate_detection'
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        """Callback for parameter changes, particularly for 'activate_detection'."""
        for param in params:
            if param.name == 'activate_detection':
                if param.value is True and self.sub is None:
                    self.get_logger().info("Activating detection: creating subscription.")
                    self.sub = self.create_subscription(Image, '/arm_rgbd_camera/image_raw', self.image_callback, self.qos)
                elif param.value is False and self.sub is not None:
                    self.get_logger().info("Deactivating detection: destroying subscription.")
                    self.destroy_subscription(self.sub)
                    self.sub = None
        # Accept all parameter changes
        return SetParametersResult(successful=True)
    
    def image_callback(self, msg: Image):
        current_time = self.get_clock().now().nanoseconds / 1e9  # convert to seconds
        with self.time_lock:
            if current_time - self.last_process_time < self.frame_interval_sec:
                return
            self.last_process_time = current_time

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge conversion failed: {str(e)}")
            return
        
        # Submit the frame for asynchronous processing
        future = self.thread_pool.submit(self.process_frame, cv_image)
        future.add_done_callback(lambda f: self.handle_results(f, msg.header))

    def process_frame(self, image: np.ndarray) -> Tuple[np.ndarray, List[Detection2D]]:
        """Process a frame: run preprocessing, inference, and postprocessing."""
        try:
            detections = self.engine.run(image)
            return image, detections
        except Exception as e:
            self.get_logger().error(f"Frame processing failed: {str(e)}")
            return image, []

    def handle_results(self, future: concurrent.futures.Future, header):
        try:
            image, detections = future.result()
        except Exception as e:
            self.get_logger().error(f"Processing future failed: {str(e)}")
            return

        # Publish detection results
        det_array = Detection2DArray()
        det_array.header = header
        det_array.detections = detections
        self.det_pub.publish(det_array)
        
        # Publish visualization image if enabled
        if self.vis_pub:
            try:
                visualized = self.visualizer.draw(image, detections)
                img_msg = self.bridge.cv2_to_imgmsg(visualized, encoding='bgr8')
                img_msg.header = header  # Set header manually
                self.vis_pub.publish(img_msg)
            except CvBridgeError as e:
                self.get_logger().error(f"CV Bridge conversion for visualization failed: {str(e)}")


    def destroy_node(self):
        # Shutdown thread pool executor gracefully
        self.thread_pool.shutdown(wait=True)
        # Destroy the subscription if it exists
        if self.sub is not None:
            self.destroy_subscription(self.sub)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8Node()
    executor = rclpy.executors.MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down YOLOv8Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
