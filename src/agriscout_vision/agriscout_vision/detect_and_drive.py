# Add this import at the top of your file
from ament_index_python.packages import get_package_share_directory
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from visualization_msgs.msg import Marker
import cv2
from ultralytics import YOLO
import time
import os

class DetectAndDriveNode(Node):
    def __init__(self):
        super().__init__('detect_and_drive_node')
        
        # 1. Publishers
        # FIX 1: Use the correct namespaced topic that the diff_drive_controller listens on
        self.cmd_pub = self.create_publisher(TwistStamped, '/agriscout_base_controller/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/spray_marker', 10)
        
        # 2. State Machine Variables
        self.STATE_SCOUTING = 0
        self.STATE_ALIGNING = 1
        self.STATE_SPRAYING = 2
        self.STATE_COOLDOWN = 3
        
        self.current_state = self.STATE_SCOUTING
        self.spray_start_time = 0.0
        self.cooldown_end_time = 0.0
        
        # Parameters
        self.target_class_id = 0        # Maize/weed class ID from your dataset
        self.confidence_threshold = 0.6 # FIX 2: Only act on detections >= 60% confidence
                                        #         Prevents false positives (phone screens, backgrounds)
        self.center_tolerance = 40      # Pixels of leeway to consider the weed "centered"
        self.forward_speed = 0.3
        self.turn_speed_multiplier = 0.002
        
        # 3. Load YOLOv8 Model
        # This looks in the exact directory where THIS script is currently running
        current_dir = os.path.dirname(__file__)
        model_path = os.path.join(current_dir, 'models', 'best.pt')
        
        # DEBUG PRINT: This will tell us exactly where ROS is searching
        self.get_logger().info(f"========== LOOKING FOR MODEL HERE: {model_path} ==========")
        
        if os.path.exists(model_path):
            self.get_logger().info(f"Loading custom model: {model_path}")
            self.yolo = YOLO(model_path)
        else:
            self.get_logger().warn("Custom best.pt not found! Falling back to yolov8n.pt")
            self.yolo = YOLO("yolov8n.pt")
            self.target_class_id = 64 # Default class ID for "potted plant" in COCO dataset
            
        # 4. Initialize Webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam!")
            
        # 5. Main Processing Loop (10 Hz)
        self.timer = self.create_timer(0.1, self.process_frame)
        self.get_logger().info("Vision Node Started! Scouting for maize...")

    def make_cmd(self):
        """Helper: create a TwistStamped with the current ROS time stamp."""
        cmd = TwistStamped()
        # FIX 3: Stamp every command with sim time so the controller doesn't discard it
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        return cmd

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame_center_x = frame.shape[1] // 2

        # --- STATE: COOLDOWN ---
        if self.current_state == self.STATE_COOLDOWN:
            if time.time() > self.cooldown_end_time:
                self.get_logger().info("Cooldown finished. Resuming scouting.")
                self.current_state = self.STATE_SCOUTING
            else:
                cmd = self.make_cmd()
                cmd.twist.linear.x = self.forward_speed
                self.cmd_pub.publish(cmd)
                cv2.putText(frame, "COOLDOWN (IGNORING WEEDS)", (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                cv2.imshow("AgriScout Vision", frame)
                cv2.waitKey(1)
                return

        # --- STATE: SPRAYING ---
        if self.current_state == self.STATE_SPRAYING:
            if time.time() - self.spray_start_time < 3.0:
                cmd = self.make_cmd()  # linear.x = 0 → robot stays stopped
                self.cmd_pub.publish(cmd)
                self.publish_spray_marker(Marker.ADD)
                cv2.putText(frame, "SPRAYING HERBICIDE...", (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            else:
                self.publish_spray_marker(Marker.DELETE)
                self.current_state = self.STATE_COOLDOWN
                self.cooldown_end_time = time.time() + 5.0
                self.get_logger().info("Spraying complete. Entering cooldown.")
            cv2.imshow("AgriScout Vision", frame)
            cv2.waitKey(1)
            return

        # --- RUN YOLO INFERENCE ---
        results = self.yolo(frame, verbose=False)[0]
        weed_detected = False
        closest_weed_x = None
        max_box_area = 0

        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])

            # FIX 2 (applied): skip low-confidence detections entirely
            if cls_id != self.target_class_id or conf < self.confidence_threshold:
                continue

            weed_detected = True
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            area = (x2 - x1) * (y2 - y1)

            # FIX 4: Draw bounding box + label with class name AND confidence %
            label = f"Maize {conf * 100:.1f}%"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

            # Background rectangle behind text for readability
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(frame, (x1, y1 - th - 8), (x1 + tw + 4, y1), (0, 0, 255), -1)
            cv2.putText(frame, label, (x1 + 2, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            if area > max_box_area:
                max_box_area = area
                closest_weed_x = (x1 + x2) // 2

        # --- STATE: SCOUTING & ALIGNING ---
        cmd = self.make_cmd()

        if not weed_detected:
            self.current_state = self.STATE_SCOUTING
            cmd.twist.linear.x = self.forward_speed
            cv2.putText(frame, "SCOUTING...", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            error_x = frame_center_x - closest_weed_x
            cv2.line(frame, (frame_center_x, 0), (frame_center_x, frame.shape[0]),
                     (255, 255, 255), 1)
            cv2.line(frame, (closest_weed_x, frame.shape[0] // 2),
                     (frame_center_x, frame.shape[0] // 2), (0, 0, 255), 2)

            if abs(error_x) > self.center_tolerance:
                self.current_state = self.STATE_ALIGNING
                cmd.twist.angular.z = float(error_x * self.turn_speed_multiplier)
                cv2.putText(frame, f"ALIGNING... Error: {error_x}", (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)
            else:
                self.current_state = self.STATE_SPRAYING
                self.spray_start_time = time.time()
                self.get_logger().info("Target Locked! Initiating Spray Sequence.")

        self.cmd_pub.publish(cmd)
        cv2.imshow("AgriScout Vision", frame)
        cv2.waitKey(1)

    def publish_spray_marker(self, action):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sprayer"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = action
        marker.pose.position.x = 0.25
        marker.pose.position.y = 0.0
        marker.pose.position.z = -0.05
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.6
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = DetectAndDriveNode()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()