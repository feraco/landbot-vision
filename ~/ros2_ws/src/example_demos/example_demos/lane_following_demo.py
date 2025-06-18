#!/usr/bin/env python3
# encoding: utf-8
"""
Lane Following Demo - Focused demonstration of lane detection and following
Displays live camera feed with detailed lane analysis and following behavior
"""

import cv2
import time
import math
import queue
import rclpy
import signal
import threading
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LaneFollowingDemo(Node):
    def __init__(self):
        super().__init__('lane_following_demo')
        
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.running = True
        self.current_status = "Waiting for camera..."
        self.following_enabled = False
        
        # Lane detection parameters
        self.lane_color = "yellow"
        self.detection_roi = {
            'y_start': 0.6,  # Start from 60% down the image
            'y_end': 1.0,    # To bottom of image
            'x_margin': 0.1  # 10% margin on sides
        }
        
        # PID controller for lane following
        self.pid_kp = 0.5
        self.pid_ki = 0.01
        self.pid_kd = 0.1
        self.pid_error_sum = 0.0
        self.pid_last_error = 0.0
        
        # Lane following state
        self.lane_center_x = 0
        self.lane_angle = 0.0
        self.lane_width = 0
        self.lane_confidence = 0.0
        self.steering_command = 0.0
        self.speed_command = 0.1
        
        # Statistics
        self.total_corrections = 0
        self.max_deviation = 0.0
        self.avg_confidence = 0.0
        self.confidence_samples = []
        
        # Camera topic options
        self.camera_topics = [
            '/ascamera/camera_publisher/rgb0/image',
            '/camera/image_raw',
            '/usb_cam/image_raw',
            '/image_raw'
        ]
        
        self.image_sub = None
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        
        self.find_camera_topic()
        
        signal.signal(signal.SIGINT, self.shutdown_handler)
        threading.Thread(target=self.main_loop, daemon=True).start()
        
        self.get_logger().info('Lane Following Demo started. Press "q" or ESC to quit.')

    def find_camera_topic(self):
        """Try to find an available camera topic"""
        available_topics = self.get_topic_names_and_types()
        topic_names = [topic[0] for topic in available_topics]
        
        for topic in self.camera_topics:
            if topic in topic_names:
                self.get_logger().info(f'Found camera topic: {topic}')
                self.image_sub = self.create_subscription(
                    Image, topic, self.image_callback, 1)
                self.current_status = "Camera connected"
                return
        
        self.current_status = "No camera found"

    def image_callback(self, ros_image):
        """Handle incoming camera images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            if self.image_queue.full():
                self.image_queue.get()
            self.image_queue.put(cv_image)
            self.current_status = "Camera active"
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')

    def detect_lane_advanced(self, image):
        """Advanced lane detection with multiple techniques"""
        h, w = image.shape[:2]
        
        # Define ROI
        roi_y_start = int(h * self.detection_roi['y_start'])
        roi_y_end = int(h * self.detection_roi['y_end'])
        roi_x_start = int(w * self.detection_roi['x_margin'])
        roi_x_end = int(w * (1 - self.detection_roi['x_margin']))
        
        # Extract ROI
        roi = image[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
        
        # Convert to different color spaces for robust detection
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
        
        # Multiple color ranges for yellow lane detection
        yellow_ranges = [
            # HSV ranges
            ([20, 50, 50], [30, 255, 255]),
            ([15, 30, 100], [35, 255, 255]),
            # LAB ranges (converted to HSV equivalent)
            ([25, 40, 80], [35, 255, 255])
        ]
        
        combined_mask = np.zeros(roi.shape[:2], dtype=np.uint8)
        
        for lower, upper in yellow_ranges:
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            combined_mask = cv2.bitwise_or(combined_mask, mask)
        
        # Morphological operations to clean up the mask
        kernel = np.ones((3, 3), np.uint8)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        result_image = image.copy()
        
        # Draw ROI
        cv2.rectangle(result_image, (roi_x_start, roi_y_start), 
                     (roi_x_end, roi_y_end), (255, 255, 0), 2)
        cv2.putText(result_image, "Detection ROI", (roi_x_start, roi_y_start - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        lane_detected = False
        lane_center_x = w // 2
        lane_angle = 0.0
        lane_width = 0
        confidence = 0.0
        
        if contours:
            # Find the largest contour (assumed to be the main lane)
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > 200:  # Minimum area threshold
                lane_detected = True
                
                # Adjust contour coordinates back to full image
                adjusted_contour = largest_contour.copy()
                adjusted_contour[:, :, 0] += roi_x_start
                adjusted_contour[:, :, 1] += roi_y_start
                
                # Get bounding rectangle
                x, y, w_lane, h_lane = cv2.boundingRect(adjusted_contour)
                lane_center_x = x + w_lane // 2
                lane_width = w_lane
                
                # Calculate confidence based on area and shape
                confidence = min(area / 2000.0, 1.0)
                
                # Fit line to get lane angle
                if len(largest_contour) >= 5:
                    [vx, vy, x0, y0] = cv2.fitLine(largest_contour, cv2.DIST_L2, 0, 0.01, 0.01)
                    lane_angle = math.atan2(vy[0], vx[0])
                
                # Draw lane detection
                cv2.drawContours(result_image, [adjusted_contour], -1, (0, 255, 255), 2)
                cv2.rectangle(result_image, (x, y), (x + w_lane, y + h_lane), (0, 255, 0), 2)
                
                # Draw lane center point
                cv2.circle(result_image, (lane_center_x, y + h_lane//2), 8, (0, 0, 255), -1)
                
                # Draw lane center line
                line_top_y = roi_y_start
                line_bottom_y = roi_y_end
                cv2.line(result_image, (lane_center_x, line_top_y), 
                        (lane_center_x, line_bottom_y), (255, 0, 0), 3)
                
                # Draw angle indicator
                if abs(lane_angle) > 0.1:
                    angle_length = 50
                    end_x = int(lane_center_x + angle_length * math.cos(lane_angle))
                    end_y = int(y + h_lane//2 + angle_length * math.sin(lane_angle))
                    cv2.arrowedLine(result_image, (lane_center_x, y + h_lane//2), 
                                   (end_x, end_y), (255, 0, 255), 3)
        
        # Draw image center line for reference
        cv2.line(result_image, (w//2, 0), (w//2, h), (255, 255, 255), 2)
        cv2.putText(result_image, "Image Center", (w//2 + 10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return result_image, lane_detected, lane_center_x, lane_angle, lane_width, confidence

    def calculate_pid_control(self, lane_center_x, image_width):
        """Calculate PID control for lane following"""
        if not self.following_enabled:
            return 0.0
        
        # Calculate error (deviation from image center)
        target_x = image_width // 2
        error = lane_center_x - target_x
        
        # Normalize error to [-1, 1]
        normalized_error = error / (image_width // 2)
        
        # PID calculation
        self.pid_error_sum += normalized_error
        error_diff = normalized_error - self.pid_last_error
        
        # Calculate PID output
        pid_output = (self.pid_kp * normalized_error + 
                     self.pid_ki * self.pid_error_sum + 
                     self.pid_kd * error_diff)
        
        # Limit output
        pid_output = max(-1.0, min(1.0, pid_output))
        
        self.pid_last_error = normalized_error
        
        # Update statistics
        if abs(normalized_error) > 0.1:
            self.total_corrections += 1
        
        self.max_deviation = max(self.max_deviation, abs(normalized_error))
        
        return pid_output

    def update_statistics(self, confidence):
        """Update lane following statistics"""
        self.confidence_samples.append(confidence)
        if len(self.confidence_samples) > 100:  # Keep last 100 samples
            self.confidence_samples.pop(0)
        
        if self.confidence_samples:
            self.avg_confidence = sum(self.confidence_samples) / len(self.confidence_samples)

    def draw_ui_overlay(self, image, lane_detected, lane_center_x, lane_angle, lane_width, confidence):
        """Draw comprehensive UI overlay"""
        h, w = image.shape[:2]
        
        # Create semi-transparent overlay panel
        overlay = image.copy()
        panel_height = 280
        cv2.rectangle(overlay, (0, 0), (w, panel_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.8, image, 0.2, 0, image)
        
        # Status information
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        color = (0, 255, 0)
        thickness = 2
        
        # Main status
        y_offset = 25
        cv2.putText(image, f"Status: {self.current_status}", (10, y_offset), 
                   font, font_scale, color, thickness)
        
        y_offset += 25
        following_status = "ENABLED" if self.following_enabled else "DISABLED"
        status_color = (0, 255, 0) if self.following_enabled else (0, 0, 255)
        cv2.putText(image, f"Lane Following: {following_status}", (10, y_offset), 
                   font, font_scale, status_color, thickness)
        
        # Lane detection info
        y_offset += 25
        lane_color = (0, 255, 0) if lane_detected else (0, 0, 255)
        lane_status = f"Lane: {'DETECTED' if lane_detected else 'NOT FOUND'}"
        cv2.putText(image, lane_status, (10, y_offset), 
                   font, font_scale, lane_color, thickness)
        
        if lane_detected:
            # Lane details
            y_offset += 25
            deviation = lane_center_x - w//2
            cv2.putText(image, f"Center Deviation: {deviation}px", (10, y_offset), 
                       font, font_scale, (0, 255, 255), thickness)
            
            y_offset += 25
            cv2.putText(image, f"Lane Angle: {math.degrees(lane_angle):.1f}°", (10, y_offset), 
                       font, font_scale, (0, 255, 255), thickness)
            
            y_offset += 25
            cv2.putText(image, f"Lane Width: {lane_width}px", (10, y_offset), 
                       font, font_scale, (0, 255, 255), thickness)
            
            y_offset += 25
            cv2.putText(image, f"Confidence: {confidence:.2f}", (10, y_offset), 
                       font, font_scale, (0, 255, 255), thickness)
        
        # Control commands
        y_offset += 30
        cv2.putText(image, "Control Commands:", (10, y_offset), 
                   font, 0.7, (255, 255, 255), 2)
        
        y_offset += 25
        cv2.putText(image, f"Steering: {self.steering_command:.3f}", (10, y_offset), 
                   font, font_scale, (255, 0, 255), thickness)
        
        y_offset += 25
        cv2.putText(image, f"Speed: {self.speed_command:.2f} m/s", (10, y_offset), 
                   font, font_scale, (255, 0, 255), thickness)
        
        # Statistics
        y_offset += 30
        cv2.putText(image, "Statistics:", (10, y_offset), 
                   font, 0.7, (255, 255, 255), 2)
        
        y_offset += 25
        cv2.putText(image, f"Corrections: {self.total_corrections}", (10, y_offset), 
                   font, font_scale, (255, 255, 255), thickness)
        
        y_offset += 25
        cv2.putText(image, f"Max Deviation: {self.max_deviation:.2f}", (10, y_offset), 
                   font, font_scale, (255, 255, 255), thickness)
        
        y_offset += 25
        cv2.putText(image, f"Avg Confidence: {self.avg_confidence:.2f}", (10, y_offset), 
                   font, font_scale, (255, 255, 255), thickness)
        
        # Controls
        y_offset += 30
        cv2.putText(image, "Controls: SPACE=Toggle, R=Reset, +/-=Speed, Q/ESC=Quit", 
                   (10, y_offset), font, 0.4, (255, 255, 255), 1)
        
        # Draw control visualization
        self.draw_control_visualization(image, lane_detected, deviation if lane_detected else 0)
        
        return image

    def draw_control_visualization(self, image, lane_detected, deviation):
        """Draw control system visualization"""
        h, w = image.shape[:2]
        panel_x = w - 200
        panel_y = 50
        panel_w = 150
        panel_h = 200
        
        # Background panel
        cv2.rectangle(image, (panel_x, panel_y), (panel_x + panel_w, panel_y + panel_h), 
                     (50, 50, 50), -1)
        cv2.rectangle(image, (panel_x, panel_y), (panel_x + panel_w, panel_y + panel_h), 
                     (255, 255, 255), 2)
        
        # Title
        cv2.putText(image, "Control", (panel_x + 10, panel_y + 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Steering indicator
        steering_y = panel_y + 50
        steering_center_x = panel_x + panel_w // 2
        steering_bar_w = panel_w - 20
        
        # Background bar
        cv2.rectangle(image, (panel_x + 10, steering_y), 
                     (panel_x + panel_w - 10, steering_y + 20), 
                     (100, 100, 100), -1)
        
        # Steering position
        steering_pos = int(steering_center_x + self.steering_command * (steering_bar_w // 4))
        cv2.rectangle(image, (steering_pos - 5, steering_y), 
                     (steering_pos + 5, steering_y + 20), 
                     (0, 255, 0), -1)
        
        cv2.putText(image, "Steering", (panel_x + 10, steering_y - 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Deviation indicator
        deviation_y = steering_y + 40
        max_deviation_display = 200  # pixels
        
        if lane_detected:
            deviation_normalized = deviation / max_deviation_display
            deviation_pos = int(steering_center_x + deviation_normalized * (steering_bar_w // 4))
            
            # Background bar
            cv2.rectangle(image, (panel_x + 10, deviation_y), 
                         (panel_x + panel_w - 10, deviation_y + 20), 
                         (100, 100, 100), -1)
            
            # Deviation position
            deviation_color = (0, 255, 0) if abs(deviation) < 50 else (0, 255, 255) if abs(deviation) < 100 else (0, 0, 255)
            cv2.rectangle(image, (deviation_pos - 3, deviation_y), 
                         (deviation_pos + 3, deviation_y + 20), 
                         deviation_color, -1)
        
        cv2.putText(image, "Deviation", (panel_x + 10, deviation_y - 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Speed indicator
        speed_y = deviation_y + 40
        max_speed = 0.3
        speed_fill = int((self.speed_command / max_speed) * (panel_w - 20))
        
        # Background bar
        cv2.rectangle(image, (panel_x + 10, speed_y), 
                     (panel_x + panel_w - 10, speed_y + 20), 
                     (100, 100, 100), -1)
        
        # Speed fill
        cv2.rectangle(image, (panel_x + 10, speed_y), 
                     (panel_x + 10 + speed_fill, speed_y + 20), 
                     (0, 255, 0), -1)
        
        cv2.putText(image, "Speed", (panel_x + 10, speed_y - 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    def main_loop(self):
        """Main processing loop"""
        while self.running:
            try:
                if self.image_sub is None:
                    # Show waiting screen with instructions
                    waiting_image = np.zeros((480, 640, 3), dtype=np.uint8)
                    
                    instructions = [
                        "LANE FOLLOWING DEMO",
                        "",
                        "Waiting for camera topic...",
                        "",
                        "Features:",
                        "• Advanced lane detection",
                        "• PID-based lane following",
                        "• Real-time control feedback",
                        "• Detailed statistics",
                        "",
                        "Controls:",
                        "SPACE - Enable/Disable following",
                        "R - Reset statistics",
                        "+ / - - Adjust speed",
                        "Q/ESC - Quit demo",
                        "",
                        "Lane Detection:",
                        "• Detects yellow lane markings",
                        "• Uses multiple color spaces",
                        "• Calculates lane center and angle",
                        "• Provides confidence scoring"
                    ]
                    
                    for i, text in enumerate(instructions):
                        y_pos = 30 + i * 20
                        color = (255, 255, 255)
                        if text == "LANE FOLLOWING DEMO":
                            color = (0, 255, 255)
                        elif text.startswith("•"):
                            color = (0, 255, 0)
                        elif text in ["Controls:", "Features:", "Lane Detection:"]:
                            color = (255, 255, 0)
                        
                        cv2.putText(waiting_image, text, (50, y_pos), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                    
                    cv2.imshow('Lane Following Demo', waiting_image)
                    key = cv2.waitKey(100)
                    if key == ord('q') or key == 27:
                        break
                    continue
                
                try:
                    image = self.image_queue.get(block=True, timeout=0.1)
                except queue.Empty:
                    continue
                
                # Detect lane
                processed_image, lane_detected, lane_center_x, lane_angle, lane_width, confidence = self.detect_lane_advanced(image.copy())
                
                # Store lane information
                self.lane_center_x = lane_center_x
                self.lane_angle = lane_angle
                self.lane_width = lane_width
                self.lane_confidence = confidence
                
                # Calculate control commands
                if lane_detected and self.following_enabled:
                    self.steering_command = self.calculate_pid_control(lane_center_x, image.shape[1])
                else:
                    self.steering_command = 0.0
                
                # Publish control commands
                twist = Twist()
                if self.following_enabled and lane_detected:
                    twist.linear.x = self.speed_command
                    twist.angular.z = self.steering_command
                
                self.cmd_vel_pub.publish(twist)
                
                # Update statistics
                self.update_statistics(confidence)
                
                # Draw UI overlay
                display_image = self.draw_ui_overlay(processed_image, lane_detected, 
                                                   lane_center_x, lane_angle, lane_width, confidence)
                
                # Display result
                cv2.imshow('Lane Following Demo', display_image)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # Q or ESC
                    break
                elif key == ord(' '):  # Space to toggle following
                    self.following_enabled = not self.following_enabled
                    if not self.following_enabled:
                        self.cmd_vel_pub.publish(Twist())  # Stop robot
                elif key == ord('r'):  # Reset statistics
                    self.total_corrections = 0
                    self.max_deviation = 0.0
                    self.confidence_samples = []
                    self.avg_confidence = 0.0
                    self.pid_error_sum = 0.0
                    self.pid_last_error = 0.0
                elif key == ord('+') or key == ord('='):  # Increase speed
                    self.speed_command = min(0.3, self.speed_command + 0.01)
                elif key == ord('-'):  # Decrease speed
                    self.speed_command = max(0.05, self.speed_command - 0.01)
                
            except Exception as e:
                self.get_logger().error(f'Main loop error: {e}')
                time.sleep(0.1)
        
        # Stop robot on exit
        self.cmd_vel_pub.publish(Twist())
        cv2.destroyAllWindows()
        self.running = False

    def shutdown_handler(self, signum, frame):
        """Handle shutdown signal"""
        self.get_logger().info('Shutting down Lane Following Demo...')
        self.running = False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        demo = LaneFollowingDemo()
        rclpy.spin(demo)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()