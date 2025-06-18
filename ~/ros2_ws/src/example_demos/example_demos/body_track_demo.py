#!/usr/bin/env python3
# encoding: utf-8
"""
Body Tracking Demo - Standalone demonstration of body tracking
Displays live camera feed with person detection and tracking
"""

import cv2
import time
import queue
import rclpy
import signal
import threading
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class BodyTrackDemo(Node):
    def __init__(self):
        super().__init__('body_track_demo')
        
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.running = True
        self.current_status = "Waiting for camera..."
        self.tracking_enabled = True
        self.person_detected = False
        self.person_center = None
        self.person_distance = 0
        self.robot_command = "Stop"
        
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
        
        self.get_logger().info('Body Tracking Demo started. Press "q" or ESC to quit.')

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

    def detect_person(self, image):
        """Simulate person detection using simple motion detection"""
        if not self.tracking_enabled:
            return image, False, None, 0
        
        # Simple simulation: create a moving "person" detection
        h, w = image.shape[:2]
        current_time = time.time()
        
        # Simulate a person moving in a pattern
        center_x = int(w/2 + 100 * np.sin(current_time * 0.5))
        center_y = int(h/2 + 50 * np.cos(current_time * 0.3))
        
        # Ensure the "person" stays within image bounds
        center_x = max(50, min(w-50, center_x))
        center_y = max(50, min(h-50, center_y))
        
        # Simulate distance based on y position (closer to bottom = closer to camera)
        distance = int(200 + (h - center_y) * 2)  # Distance in cm
        
        result_image = image.copy()
        
        # Draw person bounding box
        box_width, box_height = 80, 120
        x1 = center_x - box_width // 2
        y1 = center_y - box_height // 2
        x2 = center_x + box_width // 2
        y2 = center_y + box_height // 2
        
        cv2.rectangle(result_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(result_image, (center_x, center_y), 5, (0, 0, 255), -1)
        
        # Draw person label
        cv2.putText(result_image, f"Person ({distance}cm)", (x1, y1 - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Draw tracking line from center to person
        image_center_x, image_center_y = w // 2, h // 2
        cv2.line(result_image, (image_center_x, image_center_y), 
                (center_x, center_y), (255, 0, 0), 2)
        
        return result_image, True, (center_x, center_y), distance

    def calculate_robot_command(self, person_center, distance, image_shape):
        """Calculate robot movement command based on person position"""
        if not self.tracking_enabled or person_center is None:
            self.robot_command = "Stop"
            return Twist()
        
        h, w = image_shape[:2]
        center_x, center_y = w // 2, h // 2
        
        # Calculate errors
        error_x = person_center[0] - center_x
        error_distance = distance - 150  # Target distance: 150cm
        
        twist = Twist()
        
        # Proportional control gains
        kp_angular = 0.002
        kp_linear = 0.001
        
        # Angular velocity (turn towards person)
        if abs(error_x) > 30:  # Dead zone
            twist.angular.z = -error_x * kp_angular
            if error_x > 0:
                self.robot_command = "Turn Right"
            else:
                self.robot_command = "Turn Left"
        
        # Linear velocity (maintain distance)
        if abs(error_distance) > 20:  # Dead zone
            twist.linear.x = -error_distance * kp_linear
            if error_distance > 0:
                self.robot_command = "Move Forward"
            else:
                self.robot_command = "Move Backward"
        
        # If both errors are small, stop
        if abs(error_x) <= 30 and abs(error_distance) <= 20:
            self.robot_command = "Following"
        
        # Limit velocities
        twist.linear.x = max(-0.5, min(0.5, twist.linear.x))
        twist.angular.z = max(-1.0, min(1.0, twist.angular.z))
        
        return twist

    def draw_ui_overlay(self, image, person_detected, person_center, distance):
        """Draw status information and controls on the image"""
        h, w = image.shape[:2]
        
        # Create semi-transparent overlay panel
        overlay = image.copy()
        panel_height = 220
        cv2.rectangle(overlay, (0, 0), (w, panel_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)
        
        # Status information
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        color = (0, 255, 0)
        thickness = 2
        
        y_offset = 30
        cv2.putText(image, f"Status: {self.current_status}", (10, y_offset), 
                   font, font_scale, color, thickness)
        
        y_offset += 30
        tracking_status = "Enabled" if self.tracking_enabled else "Disabled"
        cv2.putText(image, f"Tracking: {tracking_status}", (10, y_offset), 
                   font, font_scale, color, thickness)
        
        y_offset += 30
        detection_color = (0, 255, 0) if person_detected else (0, 0, 255)
        cv2.putText(image, f"Person: {'Detected' if person_detected else 'Not Found'}", 
                   (10, y_offset), font, font_scale, detection_color, thickness)
        
        if person_detected and person_center:
            y_offset += 30
            cv2.putText(image, f"Position: ({person_center[0]}, {person_center[1]})", 
                       (10, y_offset), font, font_scale, (0, 255, 255), thickness)
            
            y_offset += 30
            cv2.putText(image, f"Distance: {distance}cm", (10, y_offset), 
                       font, font_scale, (0, 255, 255), thickness)
        
        y_offset += 30
        cv2.putText(image, f"Command: {self.robot_command}", (10, y_offset), 
                   font, font_scale, (255, 0, 255), thickness)
        
        y_offset += 30
        cv2.putText(image, "Controls: SPACE=Toggle, Q/ESC=Quit", (10, y_offset), 
                   font, 0.5, (255, 255, 255), 1)
        
        # Draw center crosshair
        center_x, center_y = w // 2, h // 2
        cv2.line(image, (center_x - 20, center_y), (center_x + 20, center_y), (255, 255, 255), 2)
        cv2.line(image, (center_x, center_y - 20), (center_x, center_y + 20), (255, 255, 255), 2)
        cv2.circle(image, (center_x, center_y), 5, (255, 255, 255), -1)
        
        # Draw tracking zone
        zone_size = 100
        cv2.rectangle(image, (center_x - zone_size, center_y - zone_size), 
                     (center_x + zone_size, center_y + zone_size), (255, 255, 0), 2)
        cv2.putText(image, "Target Zone", (center_x - 50, center_y - zone_size - 10), 
                   font, 0.5, (255, 255, 0), 1)
        
        # Draw distance indicator
        distance_panel_x = w - 200
        distance_panel_y = 50
        
        # Distance bar (0-400cm range)
        bar_length = 150
        if distance > 0:
            distance_normalized = min(distance / 400.0, 1.0)
            bar_fill = int(distance_normalized * bar_length)
            
            # Background bar
            cv2.rectangle(image, (distance_panel_x, distance_panel_y), 
                         (distance_panel_x + bar_length, distance_panel_y + 20), (100, 100, 100), -1)
            
            # Distance bar (green = good, red = too close/far)
            bar_color = (0, 255, 0) if 100 <= distance <= 200 else (0, 0, 255)
            cv2.rectangle(image, (distance_panel_x, distance_panel_y), 
                         (distance_panel_x + bar_fill, distance_panel_y + 20), bar_color, -1)
            
            # Target zone indicator
            target_pos = int((150 / 400.0) * bar_length)
            cv2.line(image, (distance_panel_x + target_pos, distance_panel_y - 5), 
                    (distance_panel_x + target_pos, distance_panel_y + 25), (255, 255, 255), 2)
        
        cv2.putText(image, "Distance", (distance_panel_x, distance_panel_y - 10), 
                   font, 0.5, (255, 255, 255), 1)
        cv2.putText(image, "0cm", (distance_panel_x, distance_panel_y + 35), 
                   font, 0.4, (255, 255, 255), 1)
        cv2.putText(image, "400cm", (distance_panel_x + bar_length - 30, distance_panel_y + 35), 
                   font, 0.4, (255, 255, 255), 1)
        
        return image

    def main_loop(self):
        """Main processing loop"""
        while self.running:
            try:
                if self.image_sub is None:
                    # Show waiting screen
                    waiting_image = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(waiting_image, "Waiting for camera topic...", 
                               (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    cv2.imshow('Body Tracking Demo', waiting_image)
                    key = cv2.waitKey(100)
                    if key == ord('q') or key == 27:
                        break
                    continue
                
                try:
                    image = self.image_queue.get(block=True, timeout=0.1)
                except queue.Empty:
                    continue
                
                # Detect person
                processed_image, person_detected, person_center, distance = self.detect_person(image.copy())
                self.person_detected = person_detected
                self.person_center = person_center
                self.person_distance = distance
                
                # Calculate robot command
                twist = self.calculate_robot_command(person_center, distance, image.shape)
                if self.tracking_enabled:
                    self.cmd_vel_pub.publish(twist)
                
                # Draw UI overlay
                display_image = self.draw_ui_overlay(processed_image, person_detected, person_center, distance)
                
                # Display result
                cv2.imshow('Body Tracking Demo', display_image)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # Q or ESC
                    break
                elif key == ord(' '):  # Space to toggle tracking
                    self.tracking_enabled = not self.tracking_enabled
                    if not self.tracking_enabled:
                        self.cmd_vel_pub.publish(Twist())  # Stop robot
                        self.robot_command = "Disabled"
                
            except Exception as e:
                self.get_logger().error(f'Main loop error: {e}')
                time.sleep(0.1)
        
        # Stop robot on exit
        self.cmd_vel_pub.publish(Twist())
        cv2.destroyAllWindows()
        self.running = False

    def shutdown_handler(self, signum, frame):
        """Handle shutdown signal"""
        self.get_logger().info('Shutting down Body Tracking Demo...')
        self.running = False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        demo = BodyTrackDemo()
        rclpy.spin(demo)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()