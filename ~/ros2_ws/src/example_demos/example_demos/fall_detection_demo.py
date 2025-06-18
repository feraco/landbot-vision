#!/usr/bin/env python3
# encoding: utf-8
"""
Fall Detection Demo - Standalone demonstration of fall detection
Displays live camera feed with pose analysis and fall detection
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

class FallDetectionDemo(Node):
    def __init__(self):
        super().__init__('fall_detection_demo')
        
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.running = True
        self.current_status = "Waiting for camera..."
        self.detection_enabled = True
        self.fall_detected = False
        self.person_height_ratio = 0.0
        self.alert_active = False
        self.alert_start_time = 0
        self.fall_count = 0
        
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
        
        self.get_logger().info('Fall Detection Demo started. Press "q" or ESC to quit.')

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

    def detect_person_pose(self, image):
        """Simulate person pose detection and fall analysis"""
        if not self.detection_enabled:
            return image, False, 0.0
        
        # Simple simulation based on time and image analysis
        h, w = image.shape[:2]
        current_time = time.time()
        
        # Simulate a person's pose changing over time
        pose_cycle = (current_time * 0.3) % (2 * np.pi)
        
        # Normal standing height ratio: ~0.7-0.8
        # Fallen height ratio: ~0.2-0.4
        base_height_ratio = 0.75 + 0.1 * np.sin(pose_cycle)
        
        # Simulate occasional falls
        fall_trigger = np.sin(current_time * 0.1) > 0.95
        if fall_trigger:
            height_ratio = 0.3 + 0.1 * np.random.random()
            is_fall = True
        else:
            height_ratio = base_height_ratio
            is_fall = height_ratio < 0.5
        
        result_image = image.copy()
        
        # Draw simulated person
        person_center_x = w // 2 + int(50 * np.sin(current_time * 0.2))
        person_bottom_y = h - 50
        person_height = int(height_ratio * (h - 100))
        person_top_y = person_bottom_y - person_height
        person_width = 60
        
        # Draw person rectangle
        person_color = (0, 0, 255) if is_fall else (0, 255, 0)
        cv2.rectangle(result_image, 
                     (person_center_x - person_width//2, person_top_y),
                     (person_center_x + person_width//2, person_bottom_y),
                     person_color, 2)
        
        # Draw head
        head_radius = 20
        head_center_y = person_top_y - head_radius
        cv2.circle(result_image, (person_center_x, head_center_y), head_radius, person_color, 2)
        
        # Draw pose landmarks (simplified)
        landmarks = [
            (person_center_x, head_center_y),  # Head
            (person_center_x, person_top_y + person_height//4),  # Shoulders
            (person_center_x, person_top_y + person_height//2),  # Waist
            (person_center_x, person_bottom_y),  # Feet
        ]
        
        for i, (x, y) in enumerate(landmarks):
            cv2.circle(result_image, (x, y), 5, person_color, -1)
            if i > 0:
                cv2.line(result_image, landmarks[i-1], (x, y), person_color, 2)
        
        # Draw height measurement line
        cv2.line(result_image, (person_center_x + person_width//2 + 10, person_top_y),
                (person_center_x + person_width//2 + 10, person_bottom_y), (255, 255, 0), 2)
        cv2.putText(result_image, f"H: {height_ratio:.2f}", 
                   (person_center_x + person_width//2 + 20, person_top_y + 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Fall detection label
        if is_fall:
            cv2.putText(result_image, "FALL DETECTED!", 
                       (person_center_x - 80, person_top_y - 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        return result_image, is_fall, height_ratio

    def update_fall_status(self, fall_detected):
        """Update fall detection status and alerts"""
        current_time = time.time()
        
        if fall_detected and not self.fall_detected:
            # New fall detected
            self.fall_detected = True
            self.alert_active = True
            self.alert_start_time = current_time
            self.fall_count += 1
            self.get_logger().warn(f'Fall detected! Count: {self.fall_count}')
        
        elif not fall_detected and self.fall_detected:
            # Person recovered
            self.fall_detected = False
        
        # Auto-clear alert after 5 seconds
        if self.alert_active and (current_time - self.alert_start_time) > 5.0:
            self.alert_active = False

    def draw_ui_overlay(self, image, fall_detected, height_ratio):
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
        detection_status = "Enabled" if self.detection_enabled else "Disabled"
        cv2.putText(image, f"Detection: {detection_status}", (10, y_offset), 
                   font, font_scale, color, thickness)
        
        y_offset += 30
        fall_color = (0, 0, 255) if fall_detected else (0, 255, 0)
        fall_status = "FALL DETECTED" if fall_detected else "Normal"
        cv2.putText(image, f"Status: {fall_status}", (10, y_offset), 
                   font, font_scale, fall_color, thickness)
        
        y_offset += 30
        cv2.putText(image, f"Height Ratio: {height_ratio:.3f}", (10, y_offset), 
                   font, font_scale, (0, 255, 255), thickness)
        
        y_offset += 30
        cv2.putText(image, f"Fall Count: {self.fall_count}", (10, y_offset), 
                   font, font_scale, (255, 0, 255), thickness)
        
        y_offset += 30
        if self.alert_active:
            alert_color = (0, 0, 255) if int(time.time() * 4) % 2 else (255, 255, 255)  # Blinking
            cv2.putText(image, "ALERT ACTIVE", (10, y_offset), 
                       font, font_scale, alert_color, thickness)
        else:
            cv2.putText(image, "No Alert", (10, y_offset), 
                       font, font_scale, (100, 100, 100), thickness)
        
        y_offset += 30
        cv2.putText(image, "Controls: SPACE=Toggle, R=Reset, Q/ESC=Quit", (10, y_offset), 
                   font, 0.5, (255, 255, 255), 1)
        
        # Draw detection zone
        zone_margin = 100
        cv2.rectangle(image, (zone_margin, zone_margin), 
                     (w - zone_margin, h - zone_margin), (255, 255, 0), 2)
        cv2.putText(image, "Detection Zone", (zone_margin + 10, zone_margin - 10), 
                   font, 0.5, (255, 255, 0), 1)
        
        # Draw height ratio indicator
        indicator_x = w - 150
        indicator_y = 50
        indicator_height = 200
        
        # Background bar
        cv2.rectangle(image, (indicator_x, indicator_y), 
                     (indicator_x + 20, indicator_y + indicator_height), (100, 100, 100), -1)
        
        # Height ratio bar
        ratio_fill = int(height_ratio * indicator_height)
        bar_color = (0, 255, 0) if height_ratio > 0.5 else (0, 0, 255)
        cv2.rectangle(image, (indicator_x, indicator_y + indicator_height - ratio_fill), 
                     (indicator_x + 20, indicator_y + indicator_height), bar_color, -1)
        
        # Threshold line
        threshold_y = indicator_y + int((1.0 - 0.5) * indicator_height)
        cv2.line(image, (indicator_x - 5, threshold_y), (indicator_x + 25, threshold_y), 
                (255, 255, 255), 2)
        
        cv2.putText(image, "Height", (indicator_x - 10, indicator_y - 10), 
                   font, 0.5, (255, 255, 255), 1)
        cv2.putText(image, "1.0", (indicator_x + 25, indicator_y + 5), 
                   font, 0.4, (255, 255, 255), 1)
        cv2.putText(image, "0.5", (indicator_x + 25, threshold_y + 5), 
                   font, 0.4, (255, 255, 255), 1)
        cv2.putText(image, "0.0", (indicator_x + 25, indicator_y + indicator_height + 5), 
                   font, 0.4, (255, 255, 255), 1)
        
        # Emergency alert overlay
        if self.alert_active:
            alert_overlay = image.copy()
            alert_alpha = 0.3 + 0.2 * np.sin(time.time() * 8)  # Pulsing effect
            cv2.rectangle(alert_overlay, (0, 0), (w, h), (0, 0, 255), -1)
            cv2.addWeighted(alert_overlay, alert_alpha, image, 1 - alert_alpha, 0, image)
            
            # Alert text
            alert_text = "EMERGENCY: FALL DETECTED"
            text_size = cv2.getTextSize(alert_text, font, 1.2, 3)[0]
            text_x = (w - text_size[0]) // 2
            text_y = h // 2
            cv2.putText(image, alert_text, (text_x, text_y), 
                       font, 1.2, (255, 255, 255), 3)
        
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
                    cv2.imshow('Fall Detection Demo', waiting_image)
                    key = cv2.waitKey(100)
                    if key == ord('q') or key == 27:
                        break
                    continue
                
                try:
                    image = self.image_queue.get(block=True, timeout=0.1)
                except queue.Empty:
                    continue
                
                # Detect person pose and analyze for falls
                processed_image, fall_detected, height_ratio = self.detect_person_pose(image.copy())
                self.person_height_ratio = height_ratio
                
                # Update fall status
                self.update_fall_status(fall_detected)
                
                # Draw UI overlay
                display_image = self.draw_ui_overlay(processed_image, fall_detected, height_ratio)
                
                # Display result
                cv2.imshow('Fall Detection Demo', display_image)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # Q or ESC
                    break
                elif key == ord(' '):  # Space to toggle detection
                    self.detection_enabled = not self.detection_enabled
                elif key == ord('r'):  # Reset counters and alerts
                    self.fall_count = 0
                    self.alert_active = False
                    self.fall_detected = False
                
            except Exception as e:
                self.get_logger().error(f'Main loop error: {e}')
                time.sleep(0.1)
        
        cv2.destroyAllWindows()
        self.running = False

    def shutdown_handler(self, signum, frame):
        """Handle shutdown signal"""
        self.get_logger().info('Shutting down Fall Detection Demo...')
        self.running = False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        demo = FallDetectionDemo()
        rclpy.spin(demo)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()