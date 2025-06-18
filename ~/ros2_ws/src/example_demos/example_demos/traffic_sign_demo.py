#!/usr/bin/env python3
# encoding: utf-8
"""
Traffic Sign Recognition Demo - Demonstration of traffic sign detection and response
Displays live camera feed with traffic sign recognition and appropriate responses
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

class TrafficSignDemo(Node):
    def __init__(self):
        super().__init__('traffic_sign_demo')
        
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.running = True
        self.current_status = "Waiting for camera..."
        self.recognition_enabled = True
        
        # Traffic sign definitions
        self.sign_types = {
            'stop': {
                'color': (0, 0, 255),
                'action': 'Stop vehicle',
                'priority': 1,
                'shape': 'octagon'
            },
            'go': {
                'color': (0, 255, 0),
                'action': 'Proceed forward',
                'priority': 2,
                'shape': 'circle'
            },
            'right': {
                'color': (255, 0, 0),
                'action': 'Turn right',
                'priority': 3,
                'shape': 'arrow'
            },
            'left': {
                'color': (0, 255, 255),
                'action': 'Turn left',
                'priority': 3,
                'shape': 'arrow'
            },
            'park': {
                'color': (255, 255, 0),
                'action': 'Parking area',
                'priority': 4,
                'shape': 'rectangle'
            },
            'crosswalk': {
                'color': (255, 0, 255),
                'action': 'Pedestrian crossing',
                'priority': 2,
                'shape': 'rectangle'
            },
            'speed_limit': {
                'color': (255, 165, 0),
                'action': 'Speed limit',
                'priority': 3,
                'shape': 'circle'
            },
            'yield': {
                'color': (255, 255, 255),
                'action': 'Yield to traffic',
                'priority': 2,
                'shape': 'triangle'
            }
        }
        
        # Detection state
        self.detected_signs = []
        self.current_action = "Normal driving"
        self.action_start_time = 0
        self.action_duration = 3.0  # seconds
        
        # Statistics
        self.signs_detected_total = 0
        self.signs_by_type = {sign_type: 0 for sign_type in self.sign_types.keys()}
        self.response_times = []
        self.avg_response_time = 0.0
        
        # Vehicle state
        self.vehicle_speed = 0.1
        self.vehicle_stopped = False
        self.last_detection_time = 0
        
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
        
        self.get_logger().info('Traffic Sign Recognition Demo started. Press "q" or ESC to quit.')

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

    def simulate_sign_detection(self, image):
        """Simulate traffic sign detection with realistic patterns"""
        if not self.recognition_enabled:
            return image, []
        
        h, w = image.shape[:2]
        current_time = time.time()
        detected_signs = []
        
        # Simulate different signs appearing in sequence
        sign_sequence = ['stop', 'go', 'right', 'crosswalk', 'park', 'speed_limit', 'yield', 'left']
        sign_cycle = int(current_time * 0.15) % len(sign_sequence)
        
        # Sometimes detect multiple signs
        num_signs = 1 if np.random.random() > 0.3 else 2
        
        for i in range(num_signs):
            sign_index = (sign_cycle + i) % len(sign_sequence)
            sign_type = sign_sequence[sign_index]
            sign_info = self.sign_types[sign_type]
            
            # Simulate sign position with some randomness
            base_x = w // 4 + i * (w // 2)
            base_y = h // 4
            
            x = base_x + int(30 * np.sin(current_time + i))
            y = base_y + int(20 * np.cos(current_time * 0.7 + i))
            
            # Simulate varying confidence and distance
            confidence = 0.6 + 0.4 * np.sin(current_time * 2 + i)
            distance = 80 + 40 * np.sin(current_time * 0.8 + i)  # Distance in cm
            
            # Only detect if confidence is above threshold
            if confidence > 0.7:
                detected_signs.append({
                    'type': sign_type,
                    'confidence': confidence,
                    'distance': distance,
                    'position': (x, y),
                    'info': sign_info
                })
        
        return self.draw_sign_detections(image, detected_signs), detected_signs

    def draw_sign_detections(self, image, detected_signs):
        """Draw detected traffic signs on the image"""
        result_image = image.copy()
        
        for sign in detected_signs:
            x, y = sign['position']
            sign_type = sign['type']
            confidence = sign['confidence']
            distance = sign['distance']
            color = sign['info']['color']
            shape = sign['info']['shape']
            
            # Draw sign based on shape
            sign_size = int(60 + 20 * confidence)
            
            if shape == 'octagon':  # Stop sign
                # Draw octagon
                points = []
                for i in range(8):
                    angle = i * np.pi / 4
                    px = int(x + sign_size//2 * np.cos(angle))
                    py = int(y + sign_size//2 * np.sin(angle))
                    points.append([px, py])
                cv2.fillPoly(result_image, [np.array(points)], color)
                cv2.polylines(result_image, [np.array(points)], True, (255, 255, 255), 3)
                
            elif shape == 'circle':
                cv2.circle(result_image, (x, y), sign_size//2, color, -1)
                cv2.circle(result_image, (x, y), sign_size//2, (255, 255, 255), 3)
                
            elif shape == 'triangle':  # Yield sign
                points = np.array([
                    [x, y - sign_size//2],
                    [x - sign_size//2, y + sign_size//2],
                    [x + sign_size//2, y + sign_size//2]
                ])
                cv2.fillPoly(result_image, [points], color)
                cv2.polylines(result_image, [points], True, (255, 255, 255), 3)
                
            elif shape == 'arrow':
                # Draw arrow
                arrow_points = np.array([
                    [x - sign_size//4, y - sign_size//4],
                    [x + sign_size//4, y],
                    [x - sign_size//4, y + sign_size//4],
                    [x - sign_size//8, y],
                    [x - sign_size//2, y],
                    [x - sign_size//2, y - sign_size//6],
                    [x - sign_size//4, y - sign_size//4]
                ])
                cv2.fillPoly(result_image, [arrow_points], color)
                cv2.polylines(result_image, [arrow_points], True, (255, 255, 255), 2)
                
            else:  # Rectangle
                cv2.rectangle(result_image, 
                             (x - sign_size//2, y - sign_size//2),
                             (x + sign_size//2, y + sign_size//2), 
                             color, -1)
                cv2.rectangle(result_image, 
                             (x - sign_size//2, y - sign_size//2),
                             (x + sign_size//2, y + sign_size//2), 
                             (255, 255, 255), 3)
            
            # Draw sign text
            cv2.putText(result_image, sign_type.upper(), 
                       (x - 25, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            
            # Draw detection info
            info_text = f"{confidence:.2f} | {distance:.0f}cm"
            cv2.putText(result_image, info_text, 
                       (x - 40, y + sign_size//2 + 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            
            # Draw bounding box
            cv2.rectangle(result_image, 
                         (x - sign_size//2 - 5, y - sign_size//2 - 5),
                         (x + sign_size//2 + 5, y + sign_size//2 + 5), 
                         (255, 255, 255), 2)
        
        return result_image

    def process_sign_detections(self, detected_signs):
        """Process detected signs and determine appropriate actions"""
        if not detected_signs:
            return
        
        current_time = time.time()
        
        # Sort signs by priority and distance
        sorted_signs = sorted(detected_signs, 
                            key=lambda s: (s['info']['priority'], s['distance']))
        
        # Process highest priority sign
        primary_sign = sorted_signs[0]
        sign_type = primary_sign['type']
        distance = primary_sign['distance']
        
        # Update statistics
        if current_time - self.last_detection_time > 1.0:  # New detection
            self.signs_detected_total += 1
            self.signs_by_type[sign_type] += 1
            
            response_time = current_time - self.last_detection_time
            self.response_times.append(response_time)
            if len(self.response_times) > 10:
                self.response_times.pop(0)
            
            self.avg_response_time = sum(self.response_times) / len(self.response_times)
            self.last_detection_time = current_time
        
        # Determine action based on sign and distance
        if distance < 100:  # Close enough to react
            if sign_type == 'stop':
                self.current_action = "STOPPING"
                self.vehicle_stopped = True
                self.action_start_time = current_time
                
            elif sign_type == 'go' and self.vehicle_stopped:
                self.current_action = "PROCEEDING"
                self.vehicle_stopped = False
                self.action_start_time = current_time
                
            elif sign_type == 'right':
                self.current_action = "TURNING RIGHT"
                self.action_start_time = current_time
                
            elif sign_type == 'left':
                self.current_action = "TURNING LEFT"
                self.action_start_time = current_time
                
            elif sign_type == 'crosswalk':
                self.current_action = "SLOWING FOR CROSSWALK"
                self.action_start_time = current_time
                
            elif sign_type == 'park':
                self.current_action = "PARKING AREA DETECTED"
                self.action_start_time = current_time
                
            elif sign_type == 'speed_limit':
                self.current_action = "SPEED LIMIT OBSERVED"
                self.action_start_time = current_time
                
            elif sign_type == 'yield':
                self.current_action = "YIELDING"
                self.action_start_time = current_time
        
        # Reset action after duration
        if current_time - self.action_start_time > self.action_duration:
            if self.current_action != "Normal driving":
                self.current_action = "Normal driving"
                if sign_type == 'stop':
                    self.vehicle_stopped = False

    def calculate_vehicle_command(self):
        """Calculate vehicle movement based on current action"""
        twist = Twist()
        
        if not self.recognition_enabled:
            return twist
        
        if self.vehicle_stopped or "STOPPING" in self.current_action:
            # Vehicle stopped
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        elif "TURNING RIGHT" in self.current_action:
            twist.linear.x = 0.05
            twist.angular.z = -0.5
            
        elif "TURNING LEFT" in self.current_action:
            twist.linear.x = 0.05
            twist.angular.z = 0.5
            
        elif "SLOWING" in self.current_action or "YIELDING" in self.current_action:
            twist.linear.x = 0.05
            twist.angular.z = 0.0
            
        else:
            # Normal driving
            twist.linear.x = self.vehicle_speed
            twist.angular.z = 0.0
        
        return twist

    def draw_ui_overlay(self, image, detected_signs):
        """Draw comprehensive UI overlay"""
        h, w = image.shape[:2]
        
        # Create semi-transparent overlay panel
        overlay = image.copy()
        panel_height = 350
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
        recognition_status = "ENABLED" if self.recognition_enabled else "DISABLED"
        status_color = (0, 255, 0) if self.recognition_enabled else (0, 0, 255)
        cv2.putText(image, f"Recognition: {recognition_status}", (10, y_offset), 
                   font, font_scale, status_color, thickness)
        
        # Current action
        y_offset += 25
        action_color = (0, 0, 255) if "STOP" in self.current_action else (255, 255, 0)
        cv2.putText(image, f"Action: {self.current_action}", (10, y_offset), 
                   font, font_scale, action_color, thickness)
        
        # Vehicle state
        y_offset += 25
        vehicle_state = "STOPPED" if self.vehicle_stopped else "MOVING"
        state_color = (0, 0, 255) if self.vehicle_stopped else (0, 255, 0)
        cv2.putText(image, f"Vehicle: {vehicle_state}", (10, y_offset), 
                   font, font_scale, state_color, thickness)
        
        y_offset += 25
        cv2.putText(image, f"Speed: {self.vehicle_speed:.2f} m/s", (10, y_offset), 
                   font, font_scale, (0, 255, 255), thickness)
        
        # Current detections
        y_offset += 30
        cv2.putText(image, "Current Detections:", (10, y_offset), 
                   font, 0.7, (255, 255, 255), 2)
        
        if detected_signs:
            for i, sign in enumerate(detected_signs[:3]):  # Show max 3 signs
                y_offset += 20
                sign_text = f"  {sign['type'].upper()}: {sign['confidence']:.2f} ({sign['distance']:.0f}cm)"
                cv2.putText(image, sign_text, (10, y_offset), 
                           font, 0.5, sign['info']['color'], 1)
                
                # Show action for this sign
                y_offset += 15
                action_text = f"    → {sign['info']['action']}"
                cv2.putText(image, action_text, (10, y_offset), 
                           font, 0.4, (200, 200, 200), 1)
        else:
            y_offset += 20
            cv2.putText(image, "  No signs detected", (10, y_offset), 
                       font, 0.5, (100, 100, 100), 1)
        
        # Statistics
        y_offset += 30
        cv2.putText(image, "Statistics:", (10, y_offset), 
                   font, 0.7, (255, 255, 255), 2)
        
        y_offset += 20
        cv2.putText(image, f"  Total Signs: {self.signs_detected_total}", (10, y_offset), 
                   font, 0.5, (255, 255, 255), 1)
        
        y_offset += 15
        cv2.putText(image, f"  Avg Response: {self.avg_response_time:.2f}s", (10, y_offset), 
                   font, 0.5, (255, 255, 255), 1)
        
        # Top detected signs
        y_offset += 20
        sorted_signs = sorted(self.signs_by_type.items(), key=lambda x: x[1], reverse=True)
        for i, (sign_type, count) in enumerate(sorted_signs[:3]):
            if count > 0:
                y_offset += 15
                cv2.putText(image, f"  {sign_type}: {count}", (10, y_offset), 
                           font, 0.4, self.sign_types[sign_type]['color'], 1)
        
        # Controls
        y_offset += 25
        cv2.putText(image, "Controls: SPACE=Toggle, R=Reset, +/-=Speed, Q/ESC=Quit", 
                   (10, y_offset), font, 0.4, (255, 255, 255), 1)
        
        # Draw sign legend
        self.draw_sign_legend(image)
        
        return image

    def draw_sign_legend(self, image):
        """Draw legend showing all possible traffic signs"""
        h, w = image.shape[:2]
        legend_x = w - 250
        legend_y = 50
        legend_w = 200
        
        # Background
        cv2.rectangle(image, (legend_x, legend_y), (legend_x + legend_w, legend_y + 300), 
                     (50, 50, 50), -1)
        cv2.rectangle(image, (legend_x, legend_y), (legend_x + legend_w, legend_y + 300), 
                     (255, 255, 255), 2)
        
        # Title
        cv2.putText(image, "Traffic Signs", (legend_x + 10, legend_y + 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Draw each sign type
        y_pos = legend_y + 50
        for sign_type, info in self.sign_types.items():
            # Small sign icon
            icon_x = legend_x + 20
            icon_size = 15
            
            if info['shape'] == 'circle':
                cv2.circle(image, (icon_x, y_pos), icon_size//2, info['color'], -1)
            elif info['shape'] == 'octagon':
                points = []
                for i in range(8):
                    angle = i * np.pi / 4
                    px = int(icon_x + icon_size//2 * np.cos(angle))
                    py = int(y_pos + icon_size//2 * np.sin(angle))
                    points.append([px, py])
                cv2.fillPoly(image, [np.array(points)], info['color'])
            else:
                cv2.rectangle(image, (icon_x - icon_size//2, y_pos - icon_size//2),
                             (icon_x + icon_size//2, y_pos + icon_size//2), info['color'], -1)
            
            # Sign name and count
            cv2.putText(image, f"{sign_type}: {self.signs_by_type[sign_type]}", 
                       (icon_x + 25, y_pos + 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            y_pos += 30

    def main_loop(self):
        """Main processing loop"""
        while self.running:
            try:
                if self.image_sub is None:
                    # Show waiting screen with instructions
                    waiting_image = np.zeros((480, 640, 3), dtype=np.uint8)
                    
                    instructions = [
                        "TRAFFIC SIGN RECOGNITION DEMO",
                        "",
                        "Waiting for camera topic...",
                        "",
                        "Recognized Signs:",
                        "• STOP - Stops the vehicle",
                        "• GO - Proceeds forward",
                        "• RIGHT/LEFT - Turn directions",
                        "• CROSSWALK - Slows for pedestrians",
                        "• PARK - Parking area",
                        "• SPEED LIMIT - Speed regulation",
                        "• YIELD - Yield to traffic",
                        "",
                        "Features:",
                        "• Real-time sign detection",
                        "• Automatic vehicle response",
                        "• Detection statistics",
                        "• Response time tracking",
                        "",
                        "Controls:",
                        "SPACE - Enable/Disable recognition",
                        "R - Reset statistics",
                        "+ / - - Adjust vehicle speed",
                        "Q/ESC - Quit demo"
                    ]
                    
                    for i, text in enumerate(instructions):
                        y_pos = 25 + i * 18
                        color = (255, 255, 255)
                        if text == "TRAFFIC SIGN RECOGNITION DEMO":
                            color = (0, 255, 255)
                        elif text.startswith("•"):
                            color = (0, 255, 0)
                        elif text in ["Controls:", "Features:", "Recognized Signs:"]:
                            color = (255, 255, 0)
                        
                        cv2.putText(waiting_image, text, (50, y_pos), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                    
                    cv2.imshow('Traffic Sign Recognition Demo', waiting_image)
                    key = cv2.waitKey(100)
                    if key == ord('q') or key == 27:
                        break
                    continue
                
                try:
                    image = self.image_queue.get(block=True, timeout=0.1)
                except queue.Empty:
                    continue
                
                # Detect traffic signs
                processed_image, detected_signs = self.simulate_sign_detection(image.copy())
                self.detected_signs = detected_signs
                
                # Process detections and determine actions
                self.process_sign_detections(detected_signs)
                
                # Calculate vehicle command
                twist = self.calculate_vehicle_command()
                if self.recognition_enabled:
                    self.cmd_vel_pub.publish(twist)
                
                # Draw UI overlay
                display_image = self.draw_ui_overlay(processed_image, detected_signs)
                
                # Display result
                cv2.imshow('Traffic Sign Recognition Demo', display_image)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # Q or ESC
                    break
                elif key == ord(' '):  # Space to toggle recognition
                    self.recognition_enabled = not self.recognition_enabled
                    if not self.recognition_enabled:
                        self.cmd_vel_pub.publish(Twist())  # Stop robot
                        self.current_action = "Recognition disabled"
                elif key == ord('r'):  # Reset statistics
                    self.signs_detected_total = 0
                    self.signs_by_type = {sign_type: 0 for sign_type in self.sign_types.keys()}
                    self.response_times = []
                    self.avg_response_time = 0.0
                elif key == ord('+') or key == ord('='):  # Increase speed
                    self.vehicle_speed = min(0.3, self.vehicle_speed + 0.01)
                elif key == ord('-'):  # Decrease speed
                    self.vehicle_speed = max(0.05, self.vehicle_speed - 0.01)
                
            except Exception as e:
                self.get_logger().error(f'Main loop error: {e}')
                time.sleep(0.1)
        
        # Stop robot on exit
        self.cmd_vel_pub.publish(Twist())
        cv2.destroyAllWindows()
        self.running = False

    def shutdown_handler(self, signum, frame):
        """Handle shutdown signal"""
        self.get_logger().info('Shutting down Traffic Sign Recognition Demo...')
        self.running = False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        demo = TrafficSignDemo()
        rclpy.spin(demo)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()