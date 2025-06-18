#!/usr/bin/env python3
# encoding: utf-8
"""
Color Sorting Demo - Standalone demonstration of color sorting
Displays live camera feed with color detection and sorting simulation
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

class ColorSortingDemo(Node):
    def __init__(self):
        super().__init__('color_sorting_demo')
        
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.running = True
        self.current_status = "Waiting for camera..."
        self.sorting_enabled = True
        self.current_target = "red"
        self.detected_objects = []
        self.sorting_stats = {"red": 0, "green": 0, "blue": 0}
        self.arm_position = "idle"
        
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
        
        # Sorting positions
        self.sort_positions = {
            'red': "Center Bin",
            'green': "Left Bin", 
            'blue': "Right Bin"
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
        
        self.get_logger().info('Color Sorting Demo started. Press "q" or ESC to quit.')

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

    def detect_colored_objects(self, image):
        """Detect colored objects in the image"""
        if not self.sorting_enabled:
            return image, []
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        detected = []
        result_image = image.copy()
        
        # Define pickup zone
        h, w = image.shape[:2]
        pickup_zone = {
            'x_min': w//2 - 100,
            'x_max': w//2 + 100,
            'y_min': h//2 - 50,
            'y_max': h//2 + 50
        }
        
        for color_name in self.color_ranges.keys():
            lower, upper = self.color_ranges[color_name]
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 300:  # Minimum area threshold
                    # Get bounding rectangle
                    x, y, w_obj, h_obj = cv2.boundingRect(contour)
                    center_x, center_y = x + w_obj//2, y + h_obj//2
                    
                    # Check if object is in pickup zone
                    in_pickup_zone = (pickup_zone['x_min'] <= center_x <= pickup_zone['x_max'] and
                                    pickup_zone['y_min'] <= center_y <= pickup_zone['y_max'])
                    
                    # Draw bounding box
                    color_bgr = self.color_bgr[color_name]
                    box_color = (255, 255, 255) if in_pickup_zone else color_bgr
                    thickness = 3 if in_pickup_zone else 2
                    cv2.rectangle(result_image, (x, y), (x + w_obj, y + h_obj), box_color, thickness)
                    
                    # Draw label
                    label = f"{color_name.capitalize()}"
                    if in_pickup_zone:
                        label += " [READY]"
                    cv2.putText(result_image, label, (x, y - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                    
                    # Add to detected list
                    detected.append({
                        'color': color_name,
                        'position': (center_x, center_y),
                        'area': area,
                        'in_pickup_zone': in_pickup_zone,
                        'bbox': (x, y, w_obj, h_obj)
                    })
        
        return result_image, detected

    def simulate_sorting_action(self, detected_objects):
        """Simulate the sorting action"""
        if not self.sorting_enabled:
            self.arm_position = "idle"
            return
        
        # Find objects in pickup zone that match current target
        target_objects = [obj for obj in detected_objects 
                         if obj['color'] == self.current_target and obj['in_pickup_zone']]
        
        if target_objects:
            # Simulate picking and sorting
            current_time = time.time()
            cycle_time = 3.0  # 3 second cycle
            cycle_position = (current_time % cycle_time) / cycle_time
            
            if cycle_position < 0.3:
                self.arm_position = "picking"
            elif cycle_position < 0.7:
                self.arm_position = f"moving to {self.sort_positions[self.current_target]}"
            elif cycle_position < 0.9:
                self.arm_position = "placing"
                # Increment counter at the end of placing
                if cycle_position > 0.85:
                    self.sorting_stats[self.current_target] += 1
            else:
                self.arm_position = "returning"
        else:
            self.arm_position = "waiting for object"

    def draw_ui_overlay(self, image, detected_objects):
        """Draw status information and controls on the image"""
        h, w = image.shape[:2]
        
        # Create semi-transparent overlay panel
        overlay = image.copy()
        panel_height = 280
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
        sorting_status = "Enabled" if self.sorting_enabled else "Disabled"
        cv2.putText(image, f"Sorting: {sorting_status}", (10, y_offset), 
                   font, font_scale, color, thickness)
        
        y_offset += 30
        target_color = self.color_bgr[self.current_target]
        cv2.putText(image, f"Target: {self.current_target.capitalize()}", (10, y_offset), 
                   font, font_scale, target_color, thickness)
        
        y_offset += 30
        cv2.putText(image, f"Arm: {self.arm_position}", (10, y_offset), 
                   font, font_scale, (255, 0, 255), thickness)
        
        y_offset += 30
        cv2.putText(image, f"Objects Found: {len(detected_objects)}", (10, y_offset), 
                   font, font_scale, color, thickness)
        
        # Sorting statistics
        y_offset += 40
        cv2.putText(image, "Sorted Count:", (10, y_offset), font, 0.6, (255, 255, 255), 1)
        for i, (color_name, count) in enumerate(self.sorting_stats.items()):
            y_pos = y_offset + 25 + i * 20
            color_bgr = self.color_bgr[color_name]
            cv2.putText(image, f"  {color_name.capitalize()}: {count}", (10, y_pos), 
                       font, 0.5, color_bgr, 1)
        
        y_offset += 100
        cv2.putText(image, "Controls: SPACE=Toggle, R/G/B=Target, C=Clear, Q/ESC=Quit", 
                   (10, y_offset), font, 0.4, (255, 255, 255), 1)
        
        # Draw pickup zone
        pickup_zone = {
            'x_min': w//2 - 100,
            'x_max': w//2 + 100,
            'y_min': h//2 - 50,
            'y_max': h//2 + 50
        }
        cv2.rectangle(image, (pickup_zone['x_min'], pickup_zone['y_min']), 
                     (pickup_zone['x_max'], pickup_zone['y_max']), (255, 255, 0), 2)
        cv2.putText(image, "Pickup Zone", (pickup_zone['x_min'], pickup_zone['y_min'] - 10), 
                   font, 0.5, (255, 255, 0), 1)
        
        # Draw sorting bins
        bin_width = 80
        bin_height = 60
        bin_y = h - 100
        
        # Left bin (Green)
        left_bin_x = 50
        cv2.rectangle(image, (left_bin_x, bin_y), (left_bin_x + bin_width, bin_y + bin_height), 
                     self.color_bgr['green'], 2)
        cv2.putText(image, "Green", (left_bin_x + 10, bin_y + 35), font, 0.5, self.color_bgr['green'], 1)
        
        # Center bin (Red)
        center_bin_x = w//2 - bin_width//2
        cv2.rectangle(image, (center_bin_x, bin_y), (center_bin_x + bin_width, bin_y + bin_height), 
                     self.color_bgr['red'], 2)
        cv2.putText(image, "Red", (center_bin_x + 20, bin_y + 35), font, 0.5, self.color_bgr['red'], 1)
        
        # Right bin (Blue)
        right_bin_x = w - 50 - bin_width
        cv2.rectangle(image, (right_bin_x, bin_y), (right_bin_x + bin_width, bin_y + bin_height), 
                     self.color_bgr['blue'], 2)
        cv2.putText(image, "Blue", (right_bin_x + 15, bin_y + 35), font, 0.5, self.color_bgr['blue'], 1)
        
        # Draw arm position indicator
        arm_panel_x = w - 200
        arm_panel_y = 50
        cv2.putText(image, "Arm Status:", (arm_panel_x, arm_panel_y), font, 0.6, (255, 255, 255), 1)
        
        # Simple arm visualization
        arm_base_x, arm_base_y = arm_panel_x + 50, arm_panel_y + 80
        arm_length = 40
        
        # Simulate arm movement based on position
        if "picking" in self.arm_position:
            arm_angle = -45
        elif "moving" in self.arm_position:
            arm_angle = 0
        elif "placing" in self.arm_position:
            arm_angle = 45
        else:
            arm_angle = 0
        
        arm_end_x = int(arm_base_x + arm_length * np.cos(np.radians(arm_angle)))
        arm_end_y = int(arm_base_y + arm_length * np.sin(np.radians(arm_angle)))
        
        cv2.circle(image, (arm_base_x, arm_base_y), 5, (255, 255, 255), -1)  # Base
        cv2.line(image, (arm_base_x, arm_base_y), (arm_end_x, arm_end_y), (255, 255, 255), 3)  # Arm
        cv2.circle(image, (arm_end_x, arm_end_y), 3, (0, 255, 255), -1)  # End effector
        
        cv2.putText(image, self.arm_position, (arm_panel_x, arm_panel_y + 120), 
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
                    cv2.imshow('Color Sorting Demo', waiting_image)
                    key = cv2.waitKey(100)
                    if key == ord('q') or key == 27:
                        break
                    continue
                
                try:
                    image = self.image_queue.get(block=True, timeout=0.1)
                except queue.Empty:
                    continue
                
                # Detect colored objects
                processed_image, detected_objects = self.detect_colored_objects(image.copy())
                self.detected_objects = detected_objects
                
                # Simulate sorting action
                self.simulate_sorting_action(detected_objects)
                
                # Draw UI overlay
                display_image = self.draw_ui_overlay(processed_image, detected_objects)
                
                # Display result
                cv2.imshow('Color Sorting Demo', display_image)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # Q or ESC
                    break
                elif key == ord(' '):  # Space to toggle sorting
                    self.sorting_enabled = not self.sorting_enabled
                elif key == ord('r'):  # Set target to red
                    self.current_target = 'red'
                elif key == ord('g'):  # Set target to green
                    self.current_target = 'green'
                elif key == ord('b'):  # Set target to blue
                    self.current_target = 'blue'
                elif key == ord('c'):  # Clear statistics
                    self.sorting_stats = {"red": 0, "green": 0, "blue": 0}
                
            except Exception as e:
                self.get_logger().error(f'Main loop error: {e}')
                time.sleep(0.1)
        
        cv2.destroyAllWindows()
        self.running = False

    def shutdown_handler(self, signum, frame):
        """Handle shutdown signal"""
        self.get_logger().info('Shutting down Color Sorting Demo...')
        self.running = False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        demo = ColorSortingDemo()
        rclpy.spin(demo)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()