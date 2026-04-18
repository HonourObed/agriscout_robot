from ament_index_python.packages import get_package_share_directory
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from visualization_msgs.msg import Marker
import cv2
from ultralytics import YOLO
import time

class DetectAndDriveNode(Node):
    def __init__(self):
        super().__init__('detect_and_drive_node')
        
        self.cmd_pub = self.create_publisher(TwistStamped, '/agriscout_base_controller/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/spray_marker', 10)
        
        # New State Machine
        self.STATE_SCOUTING = 0
        self.STATE_ALIGNING = 1
        self.STATE_FERTILIZING = 2
        
        self.current_state = self.STATE_SCOUTING
        self.last_detection_time = 0.0
        self.pump_overrun_time = 2.0  # Seconds to keep pumping after maize leaves the frame
        
        # Parameters
        self.target_class_id = 0        
        self.confidence_threshold = 0.6 
        self.center_tolerance = 40      
        self.forward_speed = 0.3
        self.turn_speed_multiplier = 0.002
        
        current_dir = os.path.dirname(__file__)
        model_path = os.path.join(current_dir, 'models', 'best.pt')
        
        self.get_logger().info(f"========== LOOKING FOR MODEL HERE: {model_path} ==========")
        
        if os.path.exists(model_path):
            self.get_logger().info(f"Loading custom model: {model_path}")
            self.yolo = YOLO(model_path)
        else:
            self.get_logger().warn("Custom best.pt not found! Falling back to yolov8n.pt")
            self.yolo = YOLO("yolov8n.pt")
            self.target_class_id = 64 
            
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam!")
            
        self.timer = self.create_timer(0.1, self.process_frame)
        self.get_logger().info("Vision Node Started! Scouting for maize to fertilize...")

    def make_cmd(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        return cmd

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame_center_x = frame.shape[1] // 2
        results = self.yolo(frame, verbose=False)[0]
        
        maize_detected = False
        closest_maize_x = None
        max_box_area = 0

        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])

            if cls_id != self.target_class_id or conf < self.confidence_threshold:
                continue

            maize_detected = True
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            area = (x2 - x1) * (y2 - y1)

            # Updated visual styling for Maize (Green)
            label = f"Maize {conf * 100:.1f}%"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(frame, (x1, y1 - th - 8), (x1 + tw + 4, y1), (0, 255, 0), -1)
            cv2.putText(frame, label, (x1 + 2, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

            if area > max_box_area:
                max_box_area = area
                closest_maize_x = (x1 + x2) // 2

        cmd = self.make_cmd()

        if maize_detected:
            self.last_detection_time = time.time()
            error_x = frame_center_x - closest_maize_x

            cv2.line(frame, (frame_center_x, 0), (frame_center_x, frame.shape[0]), (255, 255, 255), 1)
            cv2.line(frame, (closest_maize_x, frame.shape[0] // 2), (frame_center_x, frame.shape[0] // 2), (0, 0, 255), 2)

            if abs(error_x) > self.center_tolerance:
                self.current_state = self.STATE_ALIGNING
                cmd.twist.angular.z = float(error_x * self.turn_speed_multiplier)
                cmd.twist.linear.x = self.forward_speed * 0.5 # Slow down while aligning
                cv2.putText(frame, f"ALIGNING... Error: {error_x}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)
                self.publish_spray_marker(Marker.DELETE)
            else:
                self.current_state = self.STATE_FERTILIZING
                cmd.twist.linear.x = self.forward_speed * 0.3 # Creep forward while spraying
                cv2.putText(frame, "FERTILIZING MAIZE...", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                self.publish_spray_marker(Marker.ADD)
        else:
            # Overrun logic: Keep pumping and moving for a short duration after losing sight of maize
            if (time.time() - self.last_detection_time) < self.pump_overrun_time:
                self.current_state = self.STATE_FERTILIZING
                cmd.twist.linear.x = self.forward_speed
                cv2.putText(frame, "FERTILIZING (TRAILING)...", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 200, 255), 2)
                self.publish_spray_marker(Marker.ADD)
            else:
                self.current_state = self.STATE_SCOUTING
                cmd.twist.linear.x = self.forward_speed
                cv2.putText(frame, "SCOUTING...", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                self.publish_spray_marker(Marker.DELETE)

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
        # Changed to a golden-yellow color for fertilizer
        marker.color.r = 1.0
        marker.color.g = 0.84
        marker.color.b = 0.0
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