#!/usr/bin/env python3
# encoding: utf-8
"""
Self-Driving Demo - Comprehensive autonomous driving demonstration
Displays live camera feed with lane detection, traffic sign recognition, and navigation
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
from std_msgs.msg import String

class SelfDrivingDemo(Node):
    def __init__(self):
        super().__init__('self_driving_demo')
        
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.running = True
        self.current_status = "Waiting for camera..."
        self.driving_enabled = False
        
        # Driving state variables
        self.current_speed = 0.0
        self.target_speed = 0.1
        self.steering_angle = 0.0
        self.lane_center_offset = 0.0
        self.detected_objects = []
        self.traffic_light_status = "Unknown"
        self.current_action = "Stopped"
        
        # Lane detection parameters
        self.lane_color = "yellow"
        self.lane_detected = False
        self.lane_confidence = 0.0
        
        # Traffic signs and objects
        self.traffic_signs = {
            'stop': {'detected': False, 'confidence': 0.0, 'distance': 0},
            'go': {'detected': False, 'confidence': 0.0, 'distance': 0},
            'right': {'detected': False, 'confidence': 0.0, 'distance': 0},
            'park': {'detected': False, 'confidence': 0.0, 'distance': 0},
            'crosswalk': {'detected': False, 'confidence': 0.0, 'distance': 0}
        }
        
        # Driving statistics
        self.total_distance = 0.0
        self.driving_time = 0.0
        self.start_time = time.time()
        self.signs_detected_count = 0
        
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
        
        self.get_logger().info('Self-Driving Demo started. Press "q" or ESC to quit.')

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

    def detect_lane(self, image):
        """Simulate lane detection using color-based approach"""
        if not self.driving_enabled:
            return image, False, 0.0, 0.0
        
        h, w = image.shape[:2]
        
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define yellow lane color range
        lower_yellow = np.array([20, 50, 50])
        upper_yellow = np.array([30, 255, 255])
        
        # Create mask for yellow lanes
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # Focus on lower half of image (road area)
        roi_mask = np.zeros_like(mask)
        roi_mask[h//2:, :] = 255
        mask = cv2.bitwise_and(mask, roi_mask)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        result_image = image.copy()
        lane_detected = False
        center_offset = 0.0
        confidence = 0.0
        
        if contours:
            # Find largest contour (main lane)
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > 500:  # Minimum area threshold
                lane_detected = True
                confidence = min(area / 5000.0, 1.0)  # Normalize confidence
                
                # Get bounding rectangle
                x, y, w_lane, h_lane = cv2.boundingRect(largest_contour)
                lane_center_x = x + w_lane // 2
                
                # Calculate offset from image center
                image_center_x = w // 2
                center_offset = (lane_center_x - image_center_x) / (w // 2)  # Normalize to [-1, 1]
                
                # Draw lane detection
                cv2.drawContours(result_image, [largest_contour], -1, (0, 255, 255), 2)
                cv2.rectangle(result_image, (x, y), (x + w_lane, y + h_lane), (0, 255, 0), 2)
                cv2.circle(result_image, (lane_center_x, y + h_lane//2), 10, (0, 0, 255), -1)
                
                # Draw center line
                cv2.line(result_image, (image_center_x, 0), (image_center_x, h), (255, 255, 255), 2)
                cv2.line(result_image, (image_center_x, h//2), (lane_center_x, y + h_lane//2), (255, 0, 0), 3)
        
        return result_image, lane_detected, center_offset, confidence

    def detect_traffic_signs(self, image):
        """Simulate traffic sign detection"""
        if not self.driving_enabled:
            return image, []
        
        h, w = image.shape[:2]
        current_time = time.time()
        detected_signs = []
        
        # Simulate different traffic signs appearing over time
        sign_cycle = int(current_time * 0.2) % 6
        
        if sign_cycle == 0:
            # Stop sign
            sign_type = 'stop'
            color = (0, 0, 255)
            x, y = w//4, h//4
        elif sign_cycle == 1:
            # Go sign
            sign_type = 'go'
            color = (0, 255, 0)
            x, y = 3*w//4, h//4
        elif sign_cycle == 2:
            # Right turn sign
            sign_type = 'right'
            color = (255, 0, 0)
            x, y = w//2, h//6
        elif sign_cycle == 3:
            # Parking sign
            sign_type = 'park'
            color = (255, 255, 0)
            x, y = w//6, h//3
        elif sign_cycle == 4:
            # Crosswalk sign
            sign_type = 'crosswalk'
            color = (255, 0, 255)
            x, y = 5*w//6, h//3
        else:
            return image, []
        
        # Simulate sign detection with varying confidence
        confidence = 0.7 + 0.3 * np.sin(current_time * 3)
        distance = 100 + 50 * np.sin(current_time * 0.5)  # Distance in cm
        
        # Draw sign detection
        result_image = image.copy()
        sign_size = 60
        cv2.rectangle(result_image, (x - sign_size//2, y - sign_size//2), 
                     (x + sign_size//2, y + sign_size//2), color, 3)
        
        # Draw sign label
        label = f"{sign_type.upper()}"
        cv2.putText(result_image, label, (x - 30, y - sign_size//2 - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # Draw confidence and distance
        info_text = f"{confidence:.1f} | {distance:.0f}cm"
        cv2.putText(result_image, info_text, (x - 40, y + sign_size//2 + 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        detected_signs.append({
            'type': sign_type,
            'confidence': confidence,
            'distance': distance,
            'position': (x, y)
        })
        
        return result_image, detected_signs

    def calculate_driving_command(self, lane_offset, detected_signs):
        """Calculate driving commands based on lane and sign detection"""
        if not self.driving_enabled:
            self.current_action = "Disabled"
            return Twist()
        
        twist = Twist()
        
        # Check for stop conditions
        stop_conditions = ['stop']
        should_stop = any(sign['type'] in stop_conditions and sign['distance'] < 150 
                         for sign in detected_signs)
        
        if should_stop:
            self.current_action = "Stopping for sign"
            self.current_speed = 0.0
            return twist
        
        # Check for speed adjustments
        slow_conditions = ['crosswalk', 'park']
        should_slow = any(sign['type'] in slow_conditions and sign['distance'] < 200 
                         for sign in detected_signs)
        
        if should_slow:
            self.target_speed = 0.05
            self.current_action = "Slowing down"
        else:
            self.target_speed = 0.1
            self.current_action = "Normal driving"
        
        # Smooth speed control
        speed_diff = self.target_speed - self.current_speed
        self.current_speed += np.sign(speed_diff) * min(abs(speed_diff), 0.01)
        
        # Lane following with PID-like control
        if self.lane_detected:
            # Proportional control for steering
            kp = 0.5
            self.steering_angle = -kp * lane_offset
            self.steering_angle = max(-0.5, min(0.5, self.steering_angle))  # Limit steering
            
            twist.linear.x = self.current_speed
            twist.angular.z = self.steering_angle
            
            if abs(lane_offset) < 0.1:
                self.current_action = "Following lane"
            elif lane_offset > 0:
                self.current_action = "Correcting left"
            else:
                self.current_action = "Correcting right"
        else:
            self.current_action = "Searching for lane"
            # Gentle search pattern when no lane detected
            twist.angular.z = 0.1 * np.sin(time.time())
        
        return twist

    def update_statistics(self, twist):
        """Update driving statistics"""
        if self.driving_enabled:
            dt = 0.1  # Approximate time step
            self.driving_time += dt
            self.total_distance += abs(twist.linear.x) * dt

    def draw_ui_overlay(self, image, lane_detected, lane_offset, confidence, detected_signs):
        """Draw comprehensive UI overlay with driving information"""
        h, w = image.shape[:2]
        
        # Create semi-transparent overlay panel
        overlay = image.copy()
        panel_height = 300
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
        driving_status = "ENABLED" if self.driving_enabled else "DISABLED"
        status_color = (0, 255, 0) if self.driving_enabled else (0, 0, 255)
        cv2.putText(image, f"Autonomous Driving: {driving_status}", (10, y_offset), 
                   font, font_scale, status_color, thickness)
        
        # Current action
        y_offset += 25
        cv2.putText(image, f"Action: {self.current_action}", (10, y_offset), 
                   font, font_scale, (255, 255, 0), thickness)
        
        # Speed and steering
        y_offset += 25
        cv2.putText(image, f"Speed: {self.current_speed:.2f} m/s", (10, y_offset), 
                   font, font_scale, (0, 255, 255), thickness)
        
        y_offset += 25
        cv2.putText(image, f"Steering: {self.steering_angle:.2f} rad", (10, y_offset), 
                   font, font_scale, (0, 255, 255), thickness)
        
        # Lane detection info
        y_offset += 25
        lane_color = (0, 255, 0) if lane_detected else (0, 0, 255)
        lane_status = f"Lane: {'DETECTED' if lane_detected else 'NOT FOUND'}"
        cv2.putText(image, lane_status, (10, y_offset), 
                   font, font_scale, lane_color, thickness)
        
        if lane_detected:
            y_offset += 25
            cv2.putText(image, f"Lane Offset: {lane_offset:.2f}", (10, y_offset), 
                       font, font_scale, (0, 255, 255), thickness)
            y_offset += 25
            cv2.putText(image, f"Lane Confidence: {confidence:.2f}", (10, y_offset), 
                       font, font_scale, (0, 255, 255), thickness)
        
        # Traffic signs
        y_offset += 30
        cv2.putText(image, "Traffic Signs:", (10, y_offset), 
                   font, 0.7, (255, 255, 255), 2)
        
        if detected_signs:
            for i, sign in enumerate(detected_signs[:3]):  # Show max 3 signs
                y_offset += 20
                sign_text = f"  {sign['type'].upper()}: {sign['confidence']:.1f} ({sign['distance']:.0f}cm)"
                cv2.putText(image, sign_text, (10, y_offset), 
                           font, 0.5, (255, 255, 0), 1)
        else:
            y_offset += 20
            cv2.putText(image, "  No signs detected", (10, y_offset), 
                       font, 0.5, (100, 100, 100), 1)
        
        # Statistics
        y_offset += 30
        cv2.putText(image, "Statistics:", (10, y_offset), 
                   font, 0.7, (255, 255, 255), 2)
        
        y_offset += 20
        cv2.putText(image, f"  Distance: {self.total_distance:.1f}m", (10, y_offset), 
                   font, 0.5, (255, 255, 255), 1)
        
        y_offset += 20
        cv2.putText(image, f"  Time: {self.driving_time:.1f}s", (10, y_offset), 
                   font, 0.5, (255, 255, 255), 1)
        
        # Controls
        y_offset += 30
        cv2.putText(image, "Controls: SPACE=Toggle, R=Reset Stats, Q/ESC=Quit", 
                   (10, y_offset), font, 0.4, (255, 255, 255), 1)
        
        # Draw driving visualization on the right side
        self.draw_driving_visualization(image, lane_detected, lane_offset, detected_signs)
        
        return image

    def draw_driving_visualization(self, image, lane_detected, lane_offset, detected_signs):
        """Draw driving visualization panel"""
        h, w = image.shape[:2]
        panel_x = w - 250
        panel_y = 50
        panel_w = 200
        panel_h = 300
        
        # Background panel
        cv2.rectangle(image, (panel_x, panel_y), (panel_x + panel_w, panel_y + panel_h), 
                     (50, 50, 50), -1)
        cv2.rectangle(image, (panel_x, panel_y), (panel_x + panel_w, panel_y + panel_h), 
                     (255, 255, 255), 2)
        
        # Title
        cv2.putText(image, "Driving View", (panel_x + 10, panel_y + 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Draw road representation
        road_y = panel_y + 50
        road_h = 200
        road_center_x = panel_x + panel_w // 2
        
        # Road edges
        cv2.line(image, (panel_x + 30, road_y), (panel_x + 30, road_y + road_h), 
                (255, 255, 255), 3)
        cv2.line(image, (panel_x + panel_w - 30, road_y), (panel_x + panel_w - 30, road_y + road_h), 
                (255, 255, 255), 3)
        
        # Lane markings
        if lane_detected:
            lane_x = int(road_center_x + lane_offset * 50)  # Scale offset
            cv2.line(image, (lane_x, road_y), (lane_x, road_y + road_h), 
                    (0, 255, 255), 2)
        
        # Center line
        for y in range(road_y, road_y + road_h, 20):
            cv2.line(image, (road_center_x, y), (road_center_x, y + 10), 
                    (255, 255, 0), 2)
        
        # Vehicle representation
        vehicle_y = road_y + road_h - 40
        vehicle_x = road_center_x
        cv2.rectangle(image, (vehicle_x - 15, vehicle_y), (vehicle_x + 15, vehicle_y + 30), 
                     (0, 0, 255), -1)
        
        # Steering indicator
        if abs(self.steering_angle) > 0.1:
            arrow_end_x = vehicle_x + int(self.steering_angle * 30)
            cv2.arrowedLine(image, (vehicle_x, vehicle_y), (arrow_end_x, vehicle_y - 20), 
                           (255, 0, 255), 2)
        
        # Speed indicator
        speed_bar_x = panel_x + 10
        speed_bar_y = panel_y + panel_h - 50
        speed_bar_w = panel_w - 20
        speed_bar_h = 20
        
        # Background bar
        cv2.rectangle(image, (speed_bar_x, speed_bar_y), 
                     (speed_bar_x + speed_bar_w, speed_bar_y + speed_bar_h), 
                     (100, 100, 100), -1)
        
        # Speed fill
        speed_fill = int((self.current_speed / 0.2) * speed_bar_w)  # Max speed 0.2 m/s
        cv2.rectangle(image, (speed_bar_x, speed_bar_y), 
                     (speed_bar_x + speed_fill, speed_bar_y + speed_bar_h), 
                     (0, 255, 0), -1)
        
        cv2.putText(image, f"Speed: {self.current_speed:.2f}", 
                   (speed_bar_x, speed_bar_y - 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    def main_loop(self):
        """Main processing loop"""
        while self.running:
            try:
                if self.image_sub is None:
                    # Show waiting screen
                    waiting_image = np.zeros((480, 640, 3), dtype=np.uint8)
                    
                    # Instructions
                    instructions = [
                        "SELF-DRIVING DEMO",
                        "",
                        "Waiting for camera topic...",
                        "",
                        "Features:",
                        "• Lane detection and following",
                        "• Traffic sign recognition",
                        "• Autonomous navigation",
                        "• Real-time driving statistics",
                        "",
                        "Controls:",
                        "SPACE - Enable/Disable driving",
                        "R - Reset statistics",
                        "Q/ESC - Quit demo"
                    ]
                    
                    for i, text in enumerate(instructions):
                        y_pos = 50 + i * 25
                        color = (255, 255, 255)
                        if text == "SELF-DRIVING DEMO":
                            color = (0, 255, 255)
                        elif text.startswith("•"):
                            color = (0, 255, 0)
                        elif text in ["Controls:", "Features:"]:
                            color = (255, 255, 0)
                        
                        cv2.putText(waiting_image, text, (50, y_pos), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
                    
                    cv2.imshow('Self-Driving Demo', waiting_image)
                    key = cv2.waitKey(100)
                    if key == ord('q') or key == 27:
                        break
                    continue
                
                try:
                    image = self.image_queue.get(block=True, timeout=0.1)
                except queue.Empty:
                    continue
                
                # Detect lanes
                processed_image, lane_detected, lane_offset, confidence = self.detect_lane(image.copy())
                self.lane_detected = lane_detected
                self.lane_center_offset = lane_offset
                self.lane_confidence = confidence
                
                # Detect traffic signs
                processed_image, detected_signs = self.detect_traffic_signs(processed_image)
                self.detected_objects = detected_signs
                
                # Calculate driving command
                twist = self.calculate_driving_command(lane_offset, detected_signs)
                if self.driving_enabled:
                    self.cmd_vel_pub.publish(twist)
                
                # Update statistics
                self.update_statistics(twist)
                
                # Draw UI overlay
                display_image = self.draw_ui_overlay(processed_image, lane_detected, 
                                                   lane_offset, confidence, detected_signs)
                
                # Display result
                cv2.imshow('Self-Driving Demo', display_image)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # Q or ESC
                    break
                elif key == ord(' '):  # Space to toggle driving
                    self.driving_enabled = not self.driving_enabled
                    if not self.driving_enabled:
                        self.cmd_vel_pub.publish(Twist())  # Stop robot
                        self.current_action = "Disabled"
                elif key == ord('r'):  # Reset statistics
                    self.total_distance = 0.0
                    self.driving_time = 0.0
                    self.start_time = time.time()
                    self.signs_detected_count = 0
                
            except Exception as e:
                self.get_logger().error(f'Main loop error: {e}')
                time.sleep(0.1)
        
        # Stop robot on exit
        self.cmd_vel_pub.publish(Twist())
        cv2.destroyAllWindows()
        self.running = False

    def shutdown_handler(self, signum, frame):
        """Handle shutdown signal"""
        self.get_logger().info('Shutting down Self-Driving Demo...')
        self.running = False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        demo = SelfDrivingDemo()
        rclpy.spin(demo)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()