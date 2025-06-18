#!/usr/bin/env python3
# encoding: utf-8
"""
Color Detection Demo - Standalone demonstration of color detection
Displays live camera feed with color detection overlay and controls
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

class ColorDetectDemo(Node):
    def __init__(self):
        super().__init__('color_detect_demo')
        
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.running = True
        self.current_status = "Waiting for camera..."
        self.detection_enabled = True
        self.target_colors = ['red', 'green', 'blue']
        self.detected_colors = []
        
        # Color ranges in HSV
        self.color_ranges = {
            'red': [(0, 50, 50), (10, 255, 255)],
            'green': [(40, 50, 50), (80, 255, 255)],
            'blue': [(100, 50, 50), (130, 255, 255)]
        }
        
        self.color_bgr = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0)
        }
        
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
        
        self.get_logger().info('Color Detection Demo started. Press "q" or ESC to quit.')

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

    def detect_colors(self, image):
        """Detect colors in the image"""
        if not self.detection_enabled:
            return image, []
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        detected = []
        result_image = image.copy()
        
        for color_name in self.target_colors:
            if color_name in self.color_ranges:
                lower, upper = self.color_ranges[color_name]
                mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
                
                # Find contours
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > 500:  # Minimum area threshold
                        # Get bounding rectangle
                        x, y, w, h = cv2.boundingRect(contour)
                        
                        # Draw bounding box
                        color_bgr = self.color_bgr[color_name]
                        cv2.rectangle(result_image, (x, y), (x + w, y + h), color_bgr, 2)
                        
                        # Draw label
                        label = f"{color_name.capitalize()}"
                        cv2.putText(result_image, label, (x, y - 10), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_bgr, 2)
                        
                        # Add to detected list
                        detected.append({
                            'color': color_name,
                            'position': (x + w//2, y + h//2),
                            'area': area
                        })
        
        return result_image, detected

    def draw_ui_overlay(self, image, detected_colors):
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
        detection_status = "Enabled" if self.detection_enabled else "Disabled"
        cv2.putText(image, f"Detection: {detection_status}", (10, y_offset), 
                   font, font_scale, color, thickness)
        
        y_offset += 30
        cv2.putText(image, f"Colors Found: {len(detected_colors)}", (10, y_offset), 
                   font, font_scale, color, thickness)
        
        # List detected colors
        y_offset += 30
        for i, detection in enumerate(detected_colors[:3]):  # Show max 3
            color_name = detection['color']
            pos = detection['position']
            cv2.putText(image, f"  {color_name.capitalize()} at ({pos[0]}, {pos[1]})", 
                       (10, y_offset), font, 0.5, self.color_bgr[color_name], 1)
            y_offset += 20
        
        y_offset += 20
        cv2.putText(image, "Controls: SPACE=Toggle, R/G/B=Target, Q/ESC=Quit", 
                   (10, y_offset), font, 0.5, (255, 255, 255), 1)
        
        # Draw detection area indicator
        center_x, center_y = w // 2, h // 2
        cv2.rectangle(image, (center_x - 150, center_y - 100), 
                     (center_x + 150, center_y + 100), (255, 255, 0), 2)
        cv2.putText(image, "Detection Area", (center_x - 70, center_y - 110), 
                   font, 0.5, (255, 255, 0), 1)
        
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
                    cv2.imshow('Color Detection Demo', waiting_image)
                    key = cv2.waitKey(100)
                    if key == ord('q') or key == 27:
                        break
                    continue
                
                try:
                    image = self.image_queue.get(block=True, timeout=0.1)
                except queue.Empty:
                    continue
                
                # Detect colors
                processed_image, detected_colors = self.detect_colors(image.copy())
                self.detected_colors = detected_colors
                
                # Draw UI overlay
                display_image = self.draw_ui_overlay(processed_image, detected_colors)
                
                # Display result
                cv2.imshow('Color Detection Demo', display_image)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # Q or ESC
                    break
                elif key == ord(' '):  # Space to toggle detection
                    self.detection_enabled = not self.detection_enabled
                elif key == ord('r'):  # Toggle red detection
                    if 'red' in self.target_colors:
                        self.target_colors.remove('red')
                    else:
                        self.target_colors.append('red')
                elif key == ord('g'):  # Toggle green detection
                    if 'green' in self.target_colors:
                        self.target_colors.remove('green')
                    else:
                        self.target_colors.append('green')
                elif key == ord('b'):  # Toggle blue detection
                    if 'blue' in self.target_colors:
                        self.target_colors.remove('blue')
                    else:
                        self.target_colors.append('blue')
                
            except Exception as e:
                self.get_logger().error(f'Main loop error: {e}')
                time.sleep(0.1)
        
        cv2.destroyAllWindows()
        self.running = False

    def shutdown_handler(self, signum, frame):
        """Handle shutdown signal"""
        self.get_logger().info('Shutting down Color Detection Demo...')
        self.running = False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        demo = ColorDetectDemo()
        rclpy.spin(demo)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()