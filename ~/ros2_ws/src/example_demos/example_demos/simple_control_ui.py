#!/usr/bin/env python3
# encoding: utf-8
"""
Simple Control UI - Basic robot control and detection interface
Simplified version without complex geometry manager conflicts
"""

import cv2
import time
import json
import queue
import rclpy
import signal
import threading
import numpy as np
import tkinter as tk
from datetime import datetime
from tkinter import ttk, messagebox, filedialog
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class SimpleControlUI:
    """Simple UI for robot control and basic detection"""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Simple Robot Control UI")
        self.root.geometry("800x600")
        
        # Initialize ROS2
        rclpy.init()
        self.node = rclpy.create_node('simple_control_ui')
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/controller/cmd_vel', 1)
        
        # Control variables
        self.running = True
        self.current_image = None
        self.linear_speed = tk.DoubleVar(value=0.1)
        self.angular_speed = tk.DoubleVar(value=0.0)
        self.recording = tk.BooleanVar(value=False)
        
        # Detection counters
        self.detection_counts = {
            'colors': {'red': 0, 'green': 0, 'blue': 0, 'yellow': 0},
            'gestures': {'fist': 0, 'open': 0, 'peace': 0, 'thumbs': 0},
            'objects': {'person': 0, 'car': 0, 'sign': 0, 'face': 0}
        }
        
        # Setup UI and camera
        self.setup_ui()
        self.setup_camera()
        
        # Start threads
        threading.Thread(target=self.ros_spin, daemon=True).start()
        threading.Thread(target=self.update_display, daemon=True).start()
        
        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def setup_ui(self):
        """Setup the simple UI layout"""
        # Main container
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title
        title_label = ttk.Label(main_frame, text="Robot Control & Detection UI", 
                               font=('Arial', 16, 'bold'))
        title_label.pack(pady=10)
        
        # Control section
        control_frame = ttk.LabelFrame(main_frame, text="Robot Control", padding="10")
        control_frame.pack(fill=tk.X, pady=5)
        
        # Speed controls
        speed_frame = ttk.Frame(control_frame)
        speed_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(speed_frame, text="Linear Speed:").pack(side=tk.LEFT)
        ttk.Scale(speed_frame, from_=0.0, to=0.5, variable=self.linear_speed,
                 orient=tk.HORIZONTAL, length=200).pack(side=tk.LEFT, padx=10)
        ttk.Label(speed_frame, textvariable=self.linear_speed).pack(side=tk.LEFT)
        
        angular_frame = ttk.Frame(control_frame)
        angular_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(angular_frame, text="Angular Speed:").pack(side=tk.LEFT)
        ttk.Scale(angular_frame, from_=-1.0, to=1.0, variable=self.angular_speed,
                 orient=tk.HORIZONTAL, length=200).pack(side=tk.LEFT, padx=10)
        ttk.Label(angular_frame, textvariable=self.angular_speed).pack(side=tk.LEFT)
        
        # Movement buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(pady=10)
        
        ttk.Button(button_frame, text="↑ Forward", 
                  command=lambda: self.move_robot("forward")).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="← Left", 
                  command=lambda: self.move_robot("left")).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="⏹ STOP", 
                  command=lambda: self.move_robot("stop")).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="→ Right", 
                  command=lambda: self.move_robot("right")).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="↓ Backward", 
                  command=lambda: self.move_robot("backward")).pack(side=tk.LEFT, padx=5)
        
        # Detection section
        detection_frame = ttk.LabelFrame(main_frame, text="Manual Detection", padding="10")
        detection_frame.pack(fill=tk.X, pady=5)
        
        # Recording control
        ttk.Checkbutton(detection_frame, text="Record Detections", 
                       variable=self.recording).pack(anchor=tk.W, pady=5)
        
        # Color detection buttons
        color_frame = ttk.Frame(detection_frame)
        color_frame.pack(fill=tk.X, pady=5)
        ttk.Label(color_frame, text="Colors:").pack(side=tk.LEFT)
        
        for color in ['red', 'green', 'blue', 'yellow']:
            ttk.Button(color_frame, text=color.capitalize(),
                      command=lambda c=color: self.detect_item('colors', c)).pack(side=tk.LEFT, padx=2)
        
        # Gesture detection buttons
        gesture_frame = ttk.Frame(detection_frame)
        gesture_frame.pack(fill=tk.X, pady=5)
        ttk.Label(gesture_frame, text="Gestures:").pack(side=tk.LEFT)
        
        for gesture in ['fist', 'open', 'peace', 'thumbs']:
            ttk.Button(gesture_frame, text=gesture.capitalize(),
                      command=lambda g=gesture: self.detect_item('gestures', g)).pack(side=tk.LEFT, padx=2)
        
        # Object detection buttons
        object_frame = ttk.Frame(detection_frame)
        object_frame.pack(fill=tk.X, pady=5)
        ttk.Label(object_frame, text="Objects:").pack(side=tk.LEFT)
        
        for obj in ['person', 'car', 'sign', 'face']:
            ttk.Button(object_frame, text=obj.capitalize(),
                      command=lambda o=obj: self.detect_item('objects', o)).pack(side=tk.LEFT, padx=2)
        
        # Statistics section
        stats_frame = ttk.LabelFrame(main_frame, text="Detection Statistics", padding="10")
        stats_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.stats_text = tk.Text(stats_frame, height=10, width=60)
        stats_scrollbar = ttk.Scrollbar(stats_frame, orient=tk.VERTICAL, command=self.stats_text.yview)
        self.stats_text.configure(yscrollcommand=stats_scrollbar.set)
        
        self.stats_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        stats_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Control buttons
        control_buttons = ttk.Frame(main_frame)
        control_buttons.pack(fill=tk.X, pady=5)
        
        ttk.Button(control_buttons, text="Reset Statistics", 
                  command=self.reset_stats).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_buttons, text="Export Data", 
                  command=self.export_data).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_buttons, text="Show Camera", 
                  command=self.show_camera).pack(side=tk.LEFT, padx=5)
        
    def setup_camera(self):
        """Setup camera subscription"""
        camera_topics = [
            '/ascamera/camera_publisher/rgb0/image',
            '/camera/image_raw',
            '/usb_cam/image_raw',
            '/image_raw'
        ]
        
        for topic in camera_topics:
            try:
                self.image_sub = self.node.create_subscription(
                    Image, topic, self.image_callback, 1)
                print(f"Connected to camera topic: {topic}")
                break
            except Exception as e:
                print(f"Failed to connect to {topic}: {e}")
                continue
                
    def image_callback(self, ros_image):
        """Handle incoming camera images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            if self.image_queue.full():
                self.image_queue.get()
            self.image_queue.put(cv_image)
        except Exception as e:
            print(f"Image callback error: {e}")
            
    def move_robot(self, direction):
        """Move robot in specified direction"""
        twist = Twist()
        linear_speed = self.linear_speed.get()
        angular_speed = self.angular_speed.get()
        
        if direction == "forward":
            twist.linear.x = linear_speed
        elif direction == "backward":
            twist.linear.x = -linear_speed
        elif direction == "left":
            twist.angular.z = angular_speed if angular_speed != 0 else 0.5
        elif direction == "right":
            twist.angular.z = -angular_speed if angular_speed != 0 else -0.5
        elif direction == "stop":
            pass  # All zeros
            
        self.cmd_vel_pub.publish(twist)
        print(f"Moving {direction}: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}")
        
    def detect_item(self, category, item):
        """Manually detect an item"""
        if self.recording.get():
            self.detection_counts[category][item] += 1
            print(f"Detected {category}/{item}: {self.detection_counts[category][item]}")
            
    def update_display(self):
        """Update the display and statistics"""
        while self.running:
            try:
                # Update statistics display
                self.update_statistics()
                time.sleep(0.1)
            except Exception as e:
                print(f"Display update error: {e}")
                time.sleep(0.1)
                
    def update_statistics(self):
        """Update the statistics display"""
        stats_text = f"""DETECTION STATISTICS
Recording: {'ON' if self.recording.get() else 'OFF'}
Session Time: {time.time() % 3600:.0f} seconds

COLORS DETECTED:
"""
        for color, count in self.detection_counts['colors'].items():
            stats_text += f"  {color.capitalize()}: {count}\n"
            
        stats_text += "\nGESTURES DETECTED:\n"
        for gesture, count in self.detection_counts['gestures'].items():
            stats_text += f"  {gesture.capitalize()}: {count}\n"
            
        stats_text += "\nOBJECTS DETECTED:\n"
        for obj, count in self.detection_counts['objects'].items():
            stats_text += f"  {obj.capitalize()}: {count}\n"
            
        total_detections = sum(sum(category.values()) for category in self.detection_counts.values())
        stats_text += f"\nTOTAL DETECTIONS: {total_detections}"
        
        self.stats_text.delete(1.0, tk.END)
        self.stats_text.insert(1.0, stats_text)
        
    def show_camera(self):
        """Show camera feed in OpenCV window"""
        if not self.image_queue.empty():
            image = self.image_queue.get()
            
            # Add overlay information
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(image, f"Recording: {'ON' if self.recording.get() else 'OFF'}", 
                       (10, 30), font, 0.7, (0, 255, 0) if self.recording.get() else (0, 0, 255), 2)
            
            total_detections = sum(sum(category.values()) for category in self.detection_counts.values())
            cv2.putText(image, f"Total Detections: {total_detections}", 
                       (10, 60), font, 0.7, (255, 255, 255), 2)
            
            cv2.putText(image, f"Speed: {self.linear_speed.get():.2f}", 
                       (10, 90), font, 0.7, (0, 255, 255), 2)
            
            cv2.imshow('Robot Camera Feed', image)
            cv2.waitKey(1)
        else:
            messagebox.showinfo("Camera", "No camera feed available")
            
    def reset_stats(self):
        """Reset all statistics"""
        for category in self.detection_counts.values():
            for key in category:
                category[key] = 0
        print("Statistics reset")
        
    def export_data(self):
        """Export detection data"""
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if filename:
            data = {
                'timestamp': datetime.now().isoformat(),
                'detection_counts': self.detection_counts,
                'settings': {
                    'linear_speed': self.linear_speed.get(),
                    'angular_speed': self.angular_speed.get(),
                    'recording': self.recording.get()
                }
            }
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            messagebox.showinfo("Export", f"Data exported to {filename}")
            
    def ros_spin(self):
        """ROS2 spinning thread"""
        while self.running:
            try:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            except Exception as e:
                print(f"ROS spin error: {e}")
                
    def signal_handler(self, signum, frame):
        """Handle Ctrl+C"""
        self.on_closing()
        
    def on_closing(self):
        """Handle window closing"""
        self.running = False
        cv2.destroyAllWindows()
        try:
            self.node.destroy_node()
            rclpy.shutdown()
        except:
            pass
        self.root.destroy()
        
    def run(self):
        """Run the UI"""
        self.root.mainloop()

def main():
    try:
        app = SimpleControlUI()
        app.run()
    except KeyboardInterrupt:
        print("Shutting down...")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()