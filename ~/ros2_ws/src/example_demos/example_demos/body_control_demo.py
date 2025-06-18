#!/usr/bin/env python3
# encoding: utf-8
"""
Body Control Demo - Standalone demonstration of body pose control
Displays live camera feed with pose detection and control status overlay
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

class BodyControlDemo(Node):
    def __init__(self):
        super().__init__('body_control_demo')
        
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.running = True
        self.current_status = "Waiting for camera..."
        self.control_mode = "Disabled"
        self.last_command = "None"
        
        # Camera topic options (try multiple common topics)
        self.camera_topics = [
            '/ascamera/camera_publisher/rgb0/image',
            '/camera/image_raw',
            '/usb_cam/image_raw',
            '/image_raw'
        ]
        
        self.image_sub = None
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        
        # Try to find available camera topic
        self.find_camera_topic()
        
        # Set up signal handler for clean shutdown
        signal.signal(signal.SIGINT, self.shutdown_handler)
        
        # Start main processing thread
        threading.Thread(target=self.main_loop, daemon=True).start()
        
        self.get_logger().info('Body Control Demo started. Press "q" or ESC to quit.')

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
        
        self.get_logger().warn('No camera topic found. Available topics:')
        for topic in topic_names:
            if 'image' in topic.lower():
                self.get_logger().warn(f'  - {topic}')
        
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

    def draw_ui_overlay(self, image):
        """Draw status information and controls on the image"""
        h, w = image.shape[:2]
        
        # Create semi-transparent overlay panel
        overlay = image.copy()
        panel_height = 150
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
        cv2.putText(image, f"Control Mode: {self.control_mode}", (10, y_offset), 
                   font, font_scale, color, thickness)
        
        y_offset += 30
        cv2.putText(image, f"Last Command: {self.last_command}", (10, y_offset), 
                   font, font_scale, color, thickness)
        
        y_offset += 30
        cv2.putText(image, "Controls: SPACE=Toggle, Q/ESC=Quit", (10, y_offset), 
                   font, 0.5, (255, 255, 255), 1)
        
        # Draw pose detection area
        center_x, center_y = w // 2, h // 2
        cv2.rectangle(image, (center_x - 100, center_y - 100), 
                     (center_x + 100, center_y + 100), (0, 255, 255), 2)
        cv2.putText(image, "Pose Detection Area", (center_x - 80, center_y - 110), 
                   font, 0.5, (0, 255, 255), 1)
        
        return image

    def simulate_body_control(self, image):
        """Simulate body pose detection and control logic"""
        if self.control_mode == "Disabled":
            return image
        
        # Simple motion simulation based on time
        current_time = time.time()
        motion_cycle = int(current_time) % 4
        
        commands = ["Forward", "Left", "Right", "Backward"]
        self.last_command = commands[motion_cycle]
        
        # Publish simulated movement command
        twist = Twist()
        if motion_cycle == 0:  # Forward
            twist.linear.x = 0.2
        elif motion_cycle == 1:  # Left
            twist.angular.z = 0.3
        elif motion_cycle == 2:  # Right
            twist.angular.z = -0.3
        elif motion_cycle == 3:  # Backward
            twist.linear.x = -0.2
        
        self.cmd_vel_pub.publish(twist)
        
        # Draw movement indicator
        h, w = image.shape[:2]
        center_x, center_y = w // 2, h // 2
        
        arrow_color = (0, 0, 255)
        arrow_length = 50
        
        if motion_cycle == 0:  # Forward
            cv2.arrowedLine(image, (center_x, center_y), 
                           (center_x, center_y - arrow_length), arrow_color, 3)
        elif motion_cycle == 1:  # Left
            cv2.arrowedLine(image, (center_x, center_y), 
                           (center_x - arrow_length, center_y), arrow_color, 3)
        elif motion_cycle == 2:  # Right
            cv2.arrowedLine(image, (center_x, center_y), 
                           (center_x + arrow_length, center_y), arrow_color, 3)
        elif motion_cycle == 3:  # Backward
            cv2.arrowedLine(image, (center_x, center_y), 
                           (center_x, center_y + arrow_length), arrow_color, 3)
        
        return image

    def main_loop(self):
        """Main processing loop"""
        while self.running:
            try:
                # Get image from queue
                if self.image_sub is None:
                    # Show waiting screen
                    waiting_image = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(waiting_image, "Waiting for camera topic...", 
                               (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    cv2.imshow('Body Control Demo', waiting_image)
                    key = cv2.waitKey(100)
                    if key == ord('q') or key == 27:
                        break
                    continue
                
                try:
                    image = self.image_queue.get(block=True, timeout=0.1)
                except queue.Empty:
                    continue
                
                # Process image
                processed_image = self.simulate_body_control(image.copy())
                display_image = self.draw_ui_overlay(processed_image)
                
                # Display result
                cv2.imshow('Body Control Demo', display_image)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # Q or ESC
                    break
                elif key == ord(' '):  # Space to toggle control
                    self.control_mode = "Enabled" if self.control_mode == "Disabled" else "Disabled"
                    if self.control_mode == "Disabled":
                        # Stop robot when disabled
                        self.cmd_vel_pub.publish(Twist())
                        self.last_command = "Stopped"
                
            except Exception as e:
                self.get_logger().error(f'Main loop error: {e}')
                time.sleep(0.1)
        
        # Cleanup
        self.cmd_vel_pub.publish(Twist())  # Stop robot
        cv2.destroyAllWindows()
        self.running = False

    def shutdown_handler(self, signum, frame):
        """Handle shutdown signal"""
        self.get_logger().info('Shutting down Body Control Demo...')
        self.running = False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        demo = BodyControlDemo()
        rclpy.spin(demo)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()