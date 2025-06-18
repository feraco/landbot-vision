#!/usr/bin/env python3
# encoding: utf-8
"""
Hand Gesture Demo - Standalone demonstration of hand gesture recognition
Displays live camera feed with hand detection and gesture recognition overlay
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

class HandGestureDemo(Node):
    def __init__(self):
        super().__init__('hand_gesture_demo')
        
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.running = True
        self.current_status = "Waiting for camera..."
        self.detection_enabled = True
        self.current_gesture = "None"
        self.gesture_confidence = 0.0
        self.hand_landmarks = []
        
        # Gesture patterns (simplified simulation)
        self.gestures = {
            'open_hand': "Open Hand",
            'fist': "Fist",
            'peace': "Peace Sign",
            'thumbs_up': "Thumbs Up",
            'pointing': "Pointing"
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
        
        self.get_logger().info('Hand Gesture Demo started. Press "q" or ESC to quit.')

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

    def detect_hand_gesture(self, image):
        """Simulate hand gesture detection"""
        if not self.detection_enabled:
            return image, "None", 0.0, []
        
        # Simple simulation based on time and image properties
        current_time = time.time()
        gesture_cycle = int(current_time * 0.5) % len(self.gestures)
        gesture_names = list(self.gestures.keys())
        detected_gesture = gesture_names[gesture_cycle]
        confidence = 0.8 + 0.2 * np.sin(current_time * 2)  # Simulate varying confidence
        
        # Simulate hand landmarks (create a simple hand outline)
        h, w = image.shape[:2]
        center_x, center_y = w // 2, h // 2
        
        # Create simulated hand landmarks
        landmarks = []
        for i in range(21):  # MediaPipe has 21 hand landmarks
            angle = (i / 21) * 2 * np.pi
            radius = 50 + 20 * np.sin(i * 0.5)
            x = int(center_x + radius * np.cos(angle))
            y = int(center_y + radius * np.sin(angle))
            landmarks.append((x, y))
        
        # Draw hand landmarks
        result_image = image.copy()
        for i, (x, y) in enumerate(landmarks):
            cv2.circle(result_image, (x, y), 3, (0, 255, 0), -1)
            if i > 0:
                cv2.line(result_image, landmarks[i-1], (x, y), (0, 255, 0), 1)
        
        # Draw hand bounding box
        min_x = min([p[0] for p in landmarks]) - 20
        max_x = max([p[0] for p in landmarks]) + 20
        min_y = min([p[1] for p in landmarks]) - 20
        max_y = max([p[1] for p in landmarks]) + 20
        
        cv2.rectangle(result_image, (min_x, min_y), (max_x, max_y), (255, 0, 0), 2)
        
        # Draw gesture label
        gesture_name = self.gestures[detected_gesture]
        cv2.putText(result_image, f"{gesture_name} ({confidence:.1f})", 
                   (min_x, min_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        return result_image, gesture_name, confidence, landmarks

    def draw_ui_overlay(self, image, gesture, confidence, landmarks):
        """Draw status information and controls on the image"""
        h, w = image.shape[:2]
        
        # Create semi-transparent overlay panel
        overlay = image.copy()
        panel_height = 180
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
        cv2.putText(image, f"Gesture: {gesture}", (10, y_offset), 
                   font, font_scale, (0, 255, 255), thickness)
        
        y_offset += 30
        cv2.putText(image, f"Confidence: {confidence:.2f}", (10, y_offset), 
                   font, font_scale, (0, 255, 255), thickness)
        
        y_offset += 30
        cv2.putText(image, f"Landmarks: {len(landmarks)}", (10, y_offset), 
                   font, font_scale, color, thickness)
        
        y_offset += 30
        cv2.putText(image, "Controls: SPACE=Toggle, Q/ESC=Quit", (10, y_offset), 
                   font, 0.5, (255, 255, 255), 1)
        
        # Draw gesture recognition area
        center_x, center_y = w // 2, h // 2
        cv2.rectangle(image, (center_x - 120, center_y - 120), 
                     (center_x + 120, center_y + 120), (255, 255, 0), 2)
        cv2.putText(image, "Hand Detection Area", (center_x - 80, center_y - 130), 
                   font, 0.5, (255, 255, 0), 1)
        
        # Draw gesture guide
        guide_x = w - 200
        guide_y = 50
        cv2.putText(image, "Gestures:", (guide_x, guide_y), font, 0.6, (255, 255, 255), 1)
        for i, gesture_name in enumerate(self.gestures.values()):
            y_pos = guide_y + 25 + i * 20
            color = (0, 255, 255) if gesture_name == gesture else (255, 255, 255)
            cv2.putText(image, f"• {gesture_name}", (guide_x, y_pos), 
                       font, 0.4, color, 1)
        
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
                    cv2.imshow('Hand Gesture Demo', waiting_image)
                    key = cv2.waitKey(100)
                    if key == ord('q') or key == 27:
                        break
                    continue
                
                try:
                    image = self.image_queue.get(block=True, timeout=0.1)
                except queue.Empty:
                    continue
                
                # Detect hand gesture
                processed_image, gesture, confidence, landmarks = self.detect_hand_gesture(image.copy())
                self.current_gesture = gesture
                self.gesture_confidence = confidence
                self.hand_landmarks = landmarks
                
                # Draw UI overlay
                display_image = self.draw_ui_overlay(processed_image, gesture, confidence, landmarks)
                
                # Display result
                cv2.imshow('Hand Gesture Demo', display_image)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # Q or ESC
                    break
                elif key == ord(' '):  # Space to toggle detection
                    self.detection_enabled = not self.detection_enabled
                
            except Exception as e:
                self.get_logger().error(f'Main loop error: {e}')
                time.sleep(0.1)
        
        cv2.destroyAllWindows()
        self.running = False

    def shutdown_handler(self, signum, frame):
        """Handle shutdown signal"""
        self.get_logger().info('Shutting down Hand Gesture Demo...')
        self.running = False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        demo = HandGestureDemo()
        rclpy.spin(demo)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()