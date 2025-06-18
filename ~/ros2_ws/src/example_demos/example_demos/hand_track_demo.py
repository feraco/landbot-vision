#!/usr/bin/env python3
# encoding: utf-8
"""
Hand Tracking Demo - Standalone demonstration of hand tracking
Displays live camera feed with hand tracking and servo control simulation
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

class HandTrackDemo(Node):
    def __init__(self):
        super().__init__('hand_track_demo')
        
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.running = True
        self.current_status = "Waiting for camera..."
        self.tracking_enabled = True
        self.hand_position = (0, 0)
        self.servo_positions = [1500, 1500]  # Pan, Tilt
        self.target_position = (320, 240)  # Center of 640x480 image
        
        # Camera topic options
        self.camera_topics = [
            '/ascamera/camera_publisher/rgb0/image',
            '/camera/image_raw',
            '/usb_cam/image_raw',
            '/image_raw'
        ]
        
        self.image_sub = None
        self.find_camera_topic()
        
        signal.signal(signal.SIGINT, self.shutdown_handler)
        threading.Thread(target=self.main_loop, daemon=True).start()
        
        self.get_logger().info('Hand Tracking Demo started. Press "q" or ESC to quit.')

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

    def detect_hand(self, image):
        """Simulate hand detection using simple color detection"""
        if not self.tracking_enabled:
            return image, None
        
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define skin color range (simplified)
        lower_skin = np.array([0, 20, 70])
        upper_skin = np.array([20, 255, 255])
        
        # Create mask
        mask = cv2.inRange(hsv, lower_skin, upper_skin)
        
        # Apply morphological operations to clean up the mask
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        result_image = image.copy()
        hand_center = None
        
        if contours:
            # Find the largest contour (assumed to be the hand)
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > 1000:  # Minimum area threshold
                # Get bounding rectangle
                x, y, w, h = cv2.boundingRect(largest_contour)
                hand_center = (x + w//2, y + h//2)
                
                # Draw bounding box
                cv2.rectangle(result_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # Draw center point
                cv2.circle(result_image, hand_center, 10, (0, 0, 255), -1)
                
                # Draw contour
                cv2.drawContours(result_image, [largest_contour], -1, (255, 0, 0), 2)
        
        return result_image, hand_center

    def update_servo_positions(self, hand_center, image_shape):
        """Update servo positions based on hand position"""
        if hand_center is None:
            return
        
        h, w = image_shape[:2]
        center_x, center_y = w // 2, h // 2
        
        # Calculate error from center
        error_x = hand_center[0] - center_x
        error_y = hand_center[1] - center_y
        
        # Simple proportional control
        kp = 2.0  # Proportional gain
        
        # Update servo positions (simulate PWM values 800-1900)
        self.servo_positions[0] -= int(error_x * kp)  # Pan (horizontal)
        self.servo_positions[1] += int(error_y * kp)  # Tilt (vertical)
        
        # Clamp servo positions
        self.servo_positions[0] = max(800, min(1900, self.servo_positions[0]))
        self.servo_positions[1] = max(800, min(1900, self.servo_positions[1]))
        
        self.hand_position = hand_center

    def draw_ui_overlay(self, image, hand_center):
        """Draw status information and controls on the image"""
        h, w = image.shape[:2]
        
        # Create semi-transparent overlay panel
        overlay = image.copy()
        panel_height = 200
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
        if hand_center:
            cv2.putText(image, f"Hand: ({hand_center[0]}, {hand_center[1]})", (10, y_offset), 
                       font, font_scale, (0, 255, 255), thickness)
        else:
            cv2.putText(image, "Hand: Not detected", (10, y_offset), 
                       font, font_scale, (0, 0, 255), thickness)
        
        y_offset += 30
        cv2.putText(image, f"Pan Servo: {self.servo_positions[0]}", (10, y_offset), 
                   font, font_scale, color, thickness)
        
        y_offset += 30
        cv2.putText(image, f"Tilt Servo: {self.servo_positions[1]}", (10, y_offset), 
                   font, font_scale, color, thickness)
        
        y_offset += 30
        cv2.putText(image, "Controls: SPACE=Toggle, R=Reset, Q/ESC=Quit", (10, y_offset), 
                   font, 0.5, (255, 255, 255), 1)
        
        # Draw crosshair at center
        center_x, center_y = w // 2, h // 2
        cv2.line(image, (center_x - 20, center_y), (center_x + 20, center_y), (255, 255, 255), 2)
        cv2.line(image, (center_x, center_y - 20), (center_x, center_y + 20), (255, 255, 255), 2)
        cv2.circle(image, (center_x, center_y), 5, (255, 255, 255), -1)
        
        # Draw tracking area
        cv2.rectangle(image, (50, 50), (w - 50, h - 50), (255, 255, 0), 2)
        cv2.putText(image, "Tracking Area", (60, 45), font, 0.5, (255, 255, 0), 1)
        
        # Draw servo position indicators
        servo_panel_x = w - 150
        servo_panel_y = 50
        
        # Pan indicator
        pan_normalized = (self.servo_positions[0] - 800) / (1900 - 800)
        pan_x = int(servo_panel_x + pan_normalized * 100)
        cv2.rectangle(image, (servo_panel_x, servo_panel_y), (servo_panel_x + 100, servo_panel_y + 10), (100, 100, 100), -1)
        cv2.rectangle(image, (pan_x - 2, servo_panel_y), (pan_x + 2, servo_panel_y + 10), (0, 255, 0), -1)
        cv2.putText(image, "Pan", (servo_panel_x, servo_panel_y - 5), font, 0.4, (255, 255, 255), 1)
        
        # Tilt indicator
        tilt_normalized = (self.servo_positions[1] - 800) / (1900 - 800)
        tilt_y = int(servo_panel_y + 30 + tilt_normalized * 100)
        cv2.rectangle(image, (servo_panel_x, servo_panel_y + 30), (servo_panel_x + 10, servo_panel_y + 130), (100, 100, 100), -1)
        cv2.rectangle(image, (servo_panel_x, tilt_y - 2), (servo_panel_x + 10, tilt_y + 2), (0, 255, 0), -1)
        cv2.putText(image, "Tilt", (servo_panel_x + 15, servo_panel_y + 40), font, 0.4, (255, 255, 255), 1)
        
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
                    cv2.imshow('Hand Tracking Demo', waiting_image)
                    key = cv2.waitKey(100)
                    if key == ord('q') or key == 27:
                        break
                    continue
                
                try:
                    image = self.image_queue.get(block=True, timeout=0.1)
                except queue.Empty:
                    continue
                
                # Detect hand
                processed_image, hand_center = self.detect_hand(image.copy())
                
                # Update servo positions
                if self.tracking_enabled:
                    self.update_servo_positions(hand_center, image.shape)
                
                # Draw UI overlay
                display_image = self.draw_ui_overlay(processed_image, hand_center)
                
                # Display result
                cv2.imshow('Hand Tracking Demo', display_image)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # Q or ESC
                    break
                elif key == ord(' '):  # Space to toggle tracking
                    self.tracking_enabled = not self.tracking_enabled
                elif key == ord('r'):  # Reset servo positions
                    self.servo_positions = [1500, 1500]
                
            except Exception as e:
                self.get_logger().error(f'Main loop error: {e}')
                time.sleep(0.1)
        
        cv2.destroyAllWindows()
        self.running = False

    def shutdown_handler(self, signum, frame):
        """Handle shutdown signal"""
        self.get_logger().info('Shutting down Hand Tracking Demo...')
        self.running = False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        demo = HandTrackDemo()
        rclpy.spin(demo)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()