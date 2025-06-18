#!/usr/bin/env python3
# encoding: utf-8
"""
Simple Demo Launcher - Launch actual working demos with real tracking
Opens the actual demo scripts when buttons are clicked
"""

import os
import cv2
import time
import queue
import rclpy
import signal
import subprocess
import threading
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class DemoLauncher:
    """Simple launcher for all demo scripts"""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("ROS2 Demo Launcher")
        self.root.geometry("600x700")
        
        # Initialize ROS2
        rclpy.init()
        self.node = rclpy.create_node('demo_launcher')
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/controller/cmd_vel', 1)
        
        # Control variables
        self.running = True
        self.current_demo = None
        self.demo_process = None
        self.current_image = None
        self.show_camera = tk.BooleanVar(value=False)
        
        # Speed controls
        self.linear_speed = tk.DoubleVar(value=0.1)
        self.angular_speed = tk.DoubleVar(value=0.0)
        
        # Setup UI and camera
        self.setup_ui()
        self.setup_camera()
        
        # Start threads
        threading.Thread(target=self.ros_spin, daemon=True).start()
        threading.Thread(target=self.camera_display, daemon=True).start()
        
        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def setup_ui(self):
        """Setup the simple launcher UI"""
        # Title
        title_frame = ttk.Frame(self.root)
        title_frame.pack(fill=tk.X, padx=20, pady=10)
        
        title_label = ttk.Label(title_frame, text="ROS2 Demo Launcher", 
                               font=('Arial', 18, 'bold'))
        title_label.pack()
        
        status_label = ttk.Label(title_frame, text="Click a demo button to launch", 
                                font=('Arial', 10))
        status_label.pack()
        
        # Current demo status
        self.status_frame = ttk.LabelFrame(self.root, text="Current Demo Status", padding="10")
        self.status_frame.pack(fill=tk.X, padx=20, pady=10)
        
        self.status_label = ttk.Label(self.status_frame, text="No demo running", 
                                     font=('Arial', 12, 'bold'))
        self.status_label.pack()
        
        self.stop_button = ttk.Button(self.status_frame, text="Stop Current Demo", 
                                     command=self.stop_current_demo, state='disabled')
        self.stop_button.pack(pady=5)
        
        # Demo buttons
        demo_frame = ttk.LabelFrame(self.root, text="Available Demos", padding="10")
        demo_frame.pack(fill=tk.X, padx=20, pady=10)
        
        # Define demos with their actual executable names
        self.demos = [
            ("Body Control", "body_control_demo", "Control robot with body movements"),
            ("Color Detection", "color_detect_demo", "Detect and track colors"),
            ("Hand Gesture", "hand_gesture_demo", "Recognize hand gestures"),
            ("Hand Tracking", "hand_track_demo", "Track hand movements"),
            ("Body Tracking", "body_track_demo", "Track and follow people"),
            ("Color Sorting", "color_sorting_demo", "Sort objects by color"),
            ("Fall Detection", "fall_detection_demo", "Detect falls and emergencies"),
            ("Self Driving", "self_driving_demo", "Autonomous driving simulation"),
            ("Lane Following", "lane_following_demo", "Follow lane markings"),
            ("Traffic Signs", "traffic_sign_demo", "Recognize traffic signs")
        ]
        
        # Create demo buttons in a grid
        for i, (name, executable, description) in enumerate(self.demos):
            row = i // 2
            col = i % 2
            
            btn_frame = ttk.Frame(demo_frame)
            btn_frame.grid(row=row, column=col, padx=10, pady=5, sticky='ew')
            
            demo_btn = ttk.Button(btn_frame, text=name, 
                                 command=lambda e=executable, n=name: self.launch_demo(e, n))
            demo_btn.pack(fill=tk.X)
            
            desc_label = ttk.Label(btn_frame, text=description, 
                                  font=('Arial', 8), foreground='gray')
            desc_label.pack()
        
        # Configure grid weights
        for i in range(2):
            demo_frame.columnconfigure(i, weight=1)
        
        # Quick robot control
        control_frame = ttk.LabelFrame(self.root, text="Quick Robot Control", padding="10")
        control_frame.pack(fill=tk.X, padx=20, pady=10)
        
        # Speed controls
        speed_frame = ttk.Frame(control_frame)
        speed_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(speed_frame, text="Speed:").pack(side=tk.LEFT)
        ttk.Scale(speed_frame, from_=0.0, to=0.3, variable=self.linear_speed,
                 orient=tk.HORIZONTAL, length=150).pack(side=tk.LEFT, padx=10)
        ttk.Label(speed_frame, textvariable=self.linear_speed).pack(side=tk.LEFT)
        
        # Direction buttons
        direction_frame = ttk.Frame(control_frame)
        direction_frame.pack(pady=5)
        
        ttk.Button(direction_frame, text="↑", width=3,
                  command=lambda: self.move_robot("forward")).grid(row=0, column=1, padx=2, pady=2)
        ttk.Button(direction_frame, text="←", width=3,
                  command=lambda: self.move_robot("left")).grid(row=1, column=0, padx=2, pady=2)
        ttk.Button(direction_frame, text="STOP", width=6,
                  command=lambda: self.move_robot("stop")).grid(row=1, column=1, padx=2, pady=2)
        ttk.Button(direction_frame, text="→", width=3,
                  command=lambda: self.move_robot("right")).grid(row=1, column=2, padx=2, pady=2)
        ttk.Button(direction_frame, text="↓", width=3,
                  command=lambda: self.move_robot("backward")).grid(row=2, column=1, padx=2, pady=2)
        
        # Camera control
        camera_frame = ttk.LabelFrame(self.root, text="Camera View", padding="10")
        camera_frame.pack(fill=tk.X, padx=20, pady=10)
        
        ttk.Checkbutton(camera_frame, text="Show Camera Feed", 
                       variable=self.show_camera).pack()
        
        ttk.Label(camera_frame, text="Camera feed will show tracking overlays when demos are running",
                 font=('Arial', 9), foreground='blue').pack()
        
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
            
    def launch_demo(self, executable, name):
        """Launch a specific demo"""
        try:
            # Stop current demo if running
            if self.demo_process and self.demo_process.poll() is None:
                self.stop_current_demo()
                time.sleep(1)  # Give time for cleanup
            
            # Launch new demo
            print(f"Launching {name} ({executable})")
            
            # Use ros2 run to launch the demo
            cmd = ['ros2', 'run', 'example_demos', executable]
            self.demo_process = subprocess.Popen(cmd, 
                                               stdout=subprocess.PIPE, 
                                               stderr=subprocess.PIPE)
            
            self.current_demo = name
            self.status_label.config(text=f"Running: {name}")
            self.stop_button.config(state='normal')
            
            # Start monitoring thread
            threading.Thread(target=self.monitor_demo, daemon=True).start()
            
            messagebox.showinfo("Demo Launched", 
                              f"{name} demo started!\n\n"
                              f"• Check the camera window for tracking overlays\n"
                              f"• Use the demo's controls (keyboard/mouse)\n"
                              f"• Press 'q' or ESC in demo window to quit\n"
                              f"• Or use 'Stop Current Demo' button")
            
        except Exception as e:
            messagebox.showerror("Launch Error", f"Failed to launch {name}:\n{e}")
            
    def stop_current_demo(self):
        """Stop the currently running demo"""
        if self.demo_process:
            try:
                self.demo_process.terminate()
                self.demo_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.demo_process.kill()
            except Exception as e:
                print(f"Error stopping demo: {e}")
            
            self.demo_process = None
            self.current_demo = None
            self.status_label.config(text="No demo running")
            self.stop_button.config(state='disabled')
            
            # Stop robot movement
            self.move_robot("stop")
            
    def monitor_demo(self):
        """Monitor the demo process"""
        if self.demo_process:
            self.demo_process.wait()  # Wait for process to finish
            
            # Update UI when demo finishes
            if self.current_demo:
                print(f"{self.current_demo} demo finished")
                self.current_demo = None
                self.status_label.config(text="Demo finished")
                self.stop_button.config(state='disabled')
                
    def move_robot(self, direction):
        """Move robot in specified direction"""
        twist = Twist()
        speed = self.linear_speed.get()
        
        if direction == "forward":
            twist.linear.x = speed
        elif direction == "backward":
            twist.linear.x = -speed
        elif direction == "left":
            twist.angular.z = speed * 2
        elif direction == "right":
            twist.angular.z = -speed * 2
        elif direction == "stop":
            pass  # All zeros
            
        self.cmd_vel_pub.publish(twist)
        
    def camera_display(self):
        """Display camera feed if enabled"""
        while self.running:
            try:
                if self.show_camera.get() and not self.image_queue.empty():
                    image = self.image_queue.get()
                    
                    # Add simple overlay
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    
                    # Status overlay
                    status_text = f"Demo: {self.current_demo if self.current_demo else 'None'}"
                    cv2.putText(image, status_text, (10, 30), font, 0.7, (0, 255, 0), 2)
                    
                    speed_text = f"Speed: {self.linear_speed.get():.2f}"
                    cv2.putText(image, speed_text, (10, 60), font, 0.7, (0, 255, 255), 2)
                    
                    # Instructions
                    if self.current_demo:
                        cv2.putText(image, "Demo is running - check demo window for tracking", 
                                   (10, image.shape[0] - 30), font, 0.5, (255, 255, 255), 1)
                    else:
                        cv2.putText(image, "Launch a demo to see tracking overlays", 
                                   (10, image.shape[0] - 30), font, 0.5, (255, 255, 255), 1)
                    
                    cv2.imshow('Demo Launcher - Camera Feed', image)
                    cv2.waitKey(1)
                elif not self.show_camera.get():
                    cv2.destroyWindow('Demo Launcher - Camera Feed')
                    
                time.sleep(0.03)  # ~30 FPS
                
            except Exception as e:
                print(f"Camera display error: {e}")
                time.sleep(0.1)
                
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
        
        # Stop current demo
        self.stop_current_demo()
        
        # Cleanup
        cv2.destroyAllWindows()
        try:
            self.node.destroy_node()
            rclpy.shutdown()
        except:
            pass
        self.root.destroy()
        
    def run(self):
        """Run the launcher"""
        self.root.mainloop()

def main():
    try:
        launcher = DemoLauncher()
        launcher.run()
    except KeyboardInterrupt:
        print("Shutting down...")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()