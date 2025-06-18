#!/usr/bin/env python3
# encoding: utf-8
"""
Direct Demo Launcher - Run Python scripts directly without ROS2 executables
Launches the actual Python files when buttons are clicked
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

class DirectDemoLauncher:
    """Direct launcher for Python demo scripts"""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Direct Python Demo Launcher")
        self.root.geometry("700x800")
        
        # Initialize ROS2
        rclpy.init()
        self.node = rclpy.create_node('direct_demo_launcher')
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
        
        # Get the path to demo scripts
        self.demo_path = os.path.expanduser("~/ros2_ws/src/example_demos/example_demos")
        
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
        """Setup the launcher UI"""
        # Title
        title_frame = ttk.Frame(self.root)
        title_frame.pack(fill=tk.X, padx=20, pady=10)
        
        title_label = ttk.Label(title_frame, text="Direct Python Demo Launcher", 
                               font=('Arial', 18, 'bold'))
        title_label.pack()
        
        subtitle_label = ttk.Label(title_frame, text="Run Python scripts directly with full tracking", 
                                  font=('Arial', 10))
        subtitle_label.pack()
        
        # Current demo status
        self.status_frame = ttk.LabelFrame(self.root, text="Current Demo Status", padding="10")
        self.status_frame.pack(fill=tk.X, padx=20, pady=10)
        
        self.status_label = ttk.Label(self.status_frame, text="No demo running", 
                                     font=('Arial', 12, 'bold'), foreground='red')
        self.status_label.pack()
        
        button_frame = ttk.Frame(self.status_frame)
        button_frame.pack(pady=5)
        
        self.stop_button = ttk.Button(button_frame, text="Stop Current Demo", 
                                     command=self.stop_current_demo, state='disabled')
        self.stop_button.pack(side=tk.LEFT, padx=5)
        
        self.emergency_stop = ttk.Button(button_frame, text="🛑 EMERGENCY STOP", 
                                        command=self.emergency_stop_all)
        self.emergency_stop.pack(side=tk.LEFT, padx=5)
        
        # Demo buttons - organized by category
        self.setup_demo_categories()
        
        # Quick robot control
        self.setup_robot_control()
        
        # Camera control
        self.setup_camera_control()
        
        # System info
        self.setup_system_info()
        
    def setup_demo_categories(self):
        """Setup demo buttons organized by categories"""
        # Body & Gesture Demos
        body_frame = ttk.LabelFrame(self.root, text="🤸 Body & Gesture Demos", padding="10")
        body_frame.pack(fill=tk.X, padx=20, pady=5)
        
        body_demos = [
            ("Body Control", "body_control_demo.py", "Control robot with body poses"),
            ("Body Tracking", "body_track_demo.py", "Track and follow people"),
            ("Fall Detection", "fall_detection_demo.py", "Detect falls and emergencies"),
            ("Hand Gesture", "hand_gesture_demo.py", "Recognize hand gestures"),
            ("Hand Tracking", "hand_track_demo.py", "Track hand movements")
        ]
        
        self.create_demo_buttons(body_frame, body_demos)
        
        # Vision & Detection Demos
        vision_frame = ttk.LabelFrame(self.root, text="👁️ Vision & Detection Demos", padding="10")
        vision_frame.pack(fill=tk.X, padx=20, pady=5)
        
        vision_demos = [
            ("Color Detection", "color_detect_demo.py", "Detect and track colors"),
            ("Color Sorting", "color_sorting_demo.py", "Sort objects by color"),
            ("Traffic Signs", "traffic_sign_demo.py", "Recognize traffic signs")
        ]
        
        self.create_demo_buttons(vision_frame, vision_demos)
        
        # Autonomous Driving Demos
        driving_frame = ttk.LabelFrame(self.root, text="🚗 Autonomous Driving Demos", padding="10")
        driving_frame.pack(fill=tk.X, padx=20, pady=5)
        
        driving_demos = [
            ("Self Driving", "self_driving_demo.py", "Full autonomous driving simulation"),
            ("Lane Following", "lane_following_demo.py", "Follow lane markings")
        ]
        
        self.create_demo_buttons(driving_frame, driving_demos)
        
    def create_demo_buttons(self, parent_frame, demos):
        """Create buttons for a category of demos"""
        for i, (name, script, description) in enumerate(demos):
            row = i // 2
            col = i % 2
            
            btn_frame = ttk.Frame(parent_frame)
            btn_frame.grid(row=row, column=col, padx=5, pady=3, sticky='ew')
            
            # Check if script exists
            script_path = os.path.join(self.demo_path, script)
            script_exists = os.path.exists(script_path)
            
            demo_btn = ttk.Button(btn_frame, text=name, 
                                 command=lambda s=script, n=name: self.launch_python_script(s, n),
                                 state='normal' if script_exists else 'disabled')
            demo_btn.pack(fill=tk.X)
            
            desc_color = 'gray' if script_exists else 'red'
            desc_text = description if script_exists else f"Script not found: {script}"
            desc_label = ttk.Label(btn_frame, text=desc_text, 
                                  font=('Arial', 8), foreground=desc_color)
            desc_label.pack()
        
        # Configure grid weights
        for i in range(2):
            parent_frame.columnconfigure(i, weight=1)
            
    def setup_robot_control(self):
        """Setup robot control section"""
        control_frame = ttk.LabelFrame(self.root, text="🤖 Quick Robot Control", padding="10")
        control_frame.pack(fill=tk.X, padx=20, pady=5)
        
        # Speed controls
        speed_frame = ttk.Frame(control_frame)
        speed_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(speed_frame, text="Linear Speed:").pack(side=tk.LEFT)
        ttk.Scale(speed_frame, from_=0.0, to=0.5, variable=self.linear_speed,
                 orient=tk.HORIZONTAL, length=150).pack(side=tk.LEFT, padx=10)
        speed_label = ttk.Label(speed_frame, text="0.10")
        speed_label.pack(side=tk.LEFT)
        
        # Update speed label
        def update_speed_label(*args):
            speed_label.config(text=f"{self.linear_speed.get():.2f}")
        self.linear_speed.trace('w', update_speed_label)
        
        # Direction buttons
        direction_frame = ttk.Frame(control_frame)
        direction_frame.pack(pady=5)
        
        ttk.Button(direction_frame, text="↑ Forward", width=10,
                  command=lambda: self.move_robot("forward")).grid(row=0, column=1, padx=2, pady=2)
        ttk.Button(direction_frame, text="← Left", width=8,
                  command=lambda: self.move_robot("left")).grid(row=1, column=0, padx=2, pady=2)
        ttk.Button(direction_frame, text="⏹ STOP", width=8,
                  command=lambda: self.move_robot("stop")).grid(row=1, column=1, padx=2, pady=2)
        ttk.Button(direction_frame, text="→ Right", width=8,
                  command=lambda: self.move_robot("right")).grid(row=1, column=2, padx=2, pady=2)
        ttk.Button(direction_frame, text="↓ Backward", width=10,
                  command=lambda: self.move_robot("backward")).grid(row=2, column=1, padx=2, pady=2)
        
    def setup_camera_control(self):
        """Setup camera control section"""
        camera_frame = ttk.LabelFrame(self.root, text="📹 Camera Control", padding="10")
        camera_frame.pack(fill=tk.X, padx=20, pady=5)
        
        ttk.Checkbutton(camera_frame, text="Show Camera Feed in Launcher", 
                       variable=self.show_camera).pack(anchor=tk.W)
        
        info_label = ttk.Label(camera_frame, 
                              text="✓ Each demo opens its own camera window with tracking overlays\n"
                                   "✓ Launcher camera shows basic feed without tracking\n"
                                   "✓ Demo windows show full detection and control interfaces",
                              font=('Arial', 9), foreground='blue', justify=tk.LEFT)
        info_label.pack(anchor=tk.W, pady=5)
        
    def setup_system_info(self):
        """Setup system information section"""
        info_frame = ttk.LabelFrame(self.root, text="ℹ️ System Information", padding="10")
        info_frame.pack(fill=tk.X, padx=20, pady=5)
        
        # Check demo path
        path_exists = os.path.exists(self.demo_path)
        path_color = 'green' if path_exists else 'red'
        path_status = "✓ Found" if path_exists else "✗ Not Found"
        
        ttk.Label(info_frame, text=f"Demo Path: {path_status}", 
                 foreground=path_color).pack(anchor=tk.W)
        ttk.Label(info_frame, text=f"Location: {self.demo_path}", 
                 font=('Arial', 8), foreground='gray').pack(anchor=tk.W)
        
        # Instructions
        instructions = ttk.Label(info_frame, 
                                text="Instructions:\n"
                                     "• Click demo buttons to launch Python scripts directly\n"
                                     "• Each demo opens its own window with tracking\n"
                                     "• Use 'q' or ESC in demo windows to quit\n"
                                     "• Use robot controls for manual movement",
                                font=('Arial', 9), justify=tk.LEFT)
        instructions.pack(anchor=tk.W, pady=5)
        
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
            
    def launch_python_script(self, script_name, demo_name):
        """Launch a Python script directly"""
        try:
            # Stop current demo if running
            if self.demo_process and self.demo_process.poll() is None:
                self.stop_current_demo()
                time.sleep(1)  # Give time for cleanup
            
            # Build script path
            script_path = os.path.join(self.demo_path, script_name)
            
            if not os.path.exists(script_path):
                messagebox.showerror("Script Not Found", 
                                   f"Could not find script:\n{script_path}")
                return
            
            print(f"Launching {demo_name} from {script_path}")
            
            # Set up environment
            env = os.environ.copy()
            env['PYTHONPATH'] = f"{os.path.expanduser('~/ros2_ws/src')}:{env.get('PYTHONPATH', '')}"
            
            # Launch Python script directly
            cmd = ['python3', script_path]
            self.demo_process = subprocess.Popen(cmd, 
                                               cwd=self.demo_path,
                                               env=env,
                                               stdout=subprocess.PIPE, 
                                               stderr=subprocess.PIPE)
            
            self.current_demo = demo_name
            self.status_label.config(text=f"Running: {demo_name}", foreground='green')
            self.stop_button.config(state='normal')
            
            # Start monitoring thread
            threading.Thread(target=self.monitor_demo, daemon=True).start()
            
            messagebox.showinfo("Demo Launched", 
                              f"{demo_name} started successfully!\n\n"
                              f"✓ Python script: {script_name}\n"
                              f"✓ Demo window should open with camera feed\n"
                              f"✓ Full tracking and detection active\n"
                              f"✓ Use demo controls or press 'q' to quit\n\n"
                              f"If no window appears, check the terminal for errors.")
            
        except Exception as e:
            messagebox.showerror("Launch Error", f"Failed to launch {demo_name}:\n{e}")
            print(f"Launch error: {e}")
            
    def stop_current_demo(self):
        """Stop the currently running demo"""
        if self.demo_process:
            try:
                print(f"Stopping {self.current_demo}")
                self.demo_process.terminate()
                
                # Wait for graceful shutdown
                try:
                    self.demo_process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    print("Force killing demo process")
                    self.demo_process.kill()
                    self.demo_process.wait()
                    
            except Exception as e:
                print(f"Error stopping demo: {e}")
            
            self.demo_process = None
            self.current_demo = None
            self.status_label.config(text="No demo running", foreground='red')
            self.stop_button.config(state='disabled')
            
            # Stop robot movement
            self.move_robot("stop")
            
    def emergency_stop_all(self):
        """Emergency stop everything"""
        print("EMERGENCY STOP ACTIVATED")
        
        # Stop current demo
        self.stop_current_demo()
        
        # Stop robot immediately
        for _ in range(5):  # Send multiple stop commands
            self.move_robot("stop")
            time.sleep(0.1)
            
        # Kill any remaining demo processes
        try:
            subprocess.run(['pkill', '-f', 'demo.py'], timeout=2)
        except:
            pass
            
        messagebox.showwarning("Emergency Stop", "All demos stopped and robot halted!")
        
    def monitor_demo(self):
        """Monitor the demo process"""
        if self.demo_process:
            stdout, stderr = self.demo_process.communicate()
            
            # Update UI when demo finishes
            if self.current_demo:
                print(f"{self.current_demo} demo finished")
                if stderr:
                    print(f"Demo stderr: {stderr.decode()}")
                    
                self.current_demo = None
                self.status_label.config(text="Demo finished", foreground='orange')
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
                        cv2.putText(image, "Demo running - check demo window for full tracking", 
                                   (10, image.shape[0] - 50), font, 0.5, (255, 255, 255), 1)
                        cv2.putText(image, "This is launcher feed - demo has its own window", 
                                   (10, image.shape[0] - 30), font, 0.5, (255, 255, 255), 1)
                    else:
                        cv2.putText(image, "Launch a demo to see tracking in demo window", 
                                   (10, image.shape[0] - 30), font, 0.5, (255, 255, 255), 1)
                    
                    cv2.imshow('Launcher Camera Feed', image)
                    cv2.waitKey(1)
                elif not self.show_camera.get():
                    cv2.destroyWindow('Launcher Camera Feed')
                    
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
        launcher = DirectDemoLauncher()
        launcher.run()
    except KeyboardInterrupt:
        print("Shutting down...")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()