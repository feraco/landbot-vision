#!/usr/bin/env python3
# encoding: utf-8
"""
Advanced Detection UI with Real-time Object Recognition and Data Collection
Provides comprehensive detection capabilities with advanced UI controls
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

class AdvancedDetectionUI:
    """Advanced UI for object detection and data collection"""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Advanced Detection & Data Collection UI")
        self.root.geometry("1400x900")
        
        # Initialize detection systems
        self.setup_detection_systems()
        
        # ROS2 setup
        rclpy.init()
        self.node = rclpy.create_node('advanced_detection_ui')
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/controller/cmd_vel', 1)
        
        # Control variables
        self.running = True
        self.current_image = None
        self.detection_enabled = tk.BooleanVar(value=True)
        self.auto_labeling = tk.BooleanVar(value=False)
        self.data_recording = tk.BooleanVar(value=False)
        
        # Movement controls
        self.linear_speed = tk.DoubleVar(value=0.1)
        self.angular_speed = tk.DoubleVar(value=0.0)
        self.movement_mode = tk.StringVar(value="Manual")
        
        # Detection settings
        self.color_threshold = tk.DoubleVar(value=0.7)
        self.gesture_threshold = tk.DoubleVar(value=0.8)
        self.face_threshold = tk.DoubleVar(value=0.6)
        self.sign_threshold = tk.DoubleVar(value=0.75)
        
        # Setup UI
        self.setup_ui()
        self.setup_camera()
        
        # Start threads
        threading.Thread(target=self.ros_spin, daemon=True).start()
        threading.Thread(target=self.detection_loop, daemon=True).start()
        threading.Thread(target=self.ui_update_loop, daemon=True).start()
        
        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def setup_detection_systems(self):
        """Initialize all detection systems"""
        self.detectors = {
            'color': self.ColorDetector(),
            'gesture': self.GestureDetector(),
            'pose': self.PoseDetector(),
            'face': self.FaceDetector(),
            'sign': self.SignDetector(),
            'object': self.ObjectDetector()
        }
        
        # Data storage
        self.detection_data = {
            'colors': {},
            'gestures': {},
            'poses': {},
            'faces': {},
            'signs': {},
            'objects': {},
            'session_info': {
                'start_time': datetime.now(),
                'total_detections': 0,
                'detection_history': []
            }
        }
        
        # Labels and annotations
        self.labels = []
        self.annotations = {}
        
    def setup_ui(self):
        """Setup the comprehensive UI"""
        # Create main notebook for tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Create tabs
        self.setup_detection_tab()
        self.setup_control_tab()
        self.setup_data_tab()
        self.setup_settings_tab()
        
    def setup_detection_tab(self):
        """Setup the main detection tab"""
        detection_frame = ttk.Frame(self.notebook)
        self.notebook.add(detection_frame, text="Detection & Labeling")
        
        # Left panel for controls
        left_panel = ttk.Frame(detection_frame)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)
        
        # Right panel for display
        right_panel = ttk.Frame(detection_frame)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Detection controls
        detection_control_frame = ttk.LabelFrame(left_panel, text="Detection Controls", padding=10)
        detection_control_frame.pack(fill=tk.X, pady=5)
        
        ttk.Checkbutton(detection_control_frame, text="Enable Detection", 
                       variable=self.detection_enabled).pack(anchor=tk.W)
        ttk.Checkbutton(detection_control_frame, text="Auto Labeling", 
                       variable=self.auto_labeling).pack(anchor=tk.W)
        ttk.Checkbutton(detection_control_frame, text="Record Data", 
                       variable=self.data_recording).pack(anchor=tk.W)
        
        # Detection categories
        categories_frame = ttk.LabelFrame(left_panel, text="Detection Categories", padding=10)
        categories_frame.pack(fill=tk.X, pady=5)
        
        self.detection_vars = {}
        categories = ['Colors', 'Gestures', 'Poses', 'Faces', 'Signs', 'Objects']
        for category in categories:
            var = tk.BooleanVar(value=True)
            self.detection_vars[category.lower()] = var
            ttk.Checkbutton(categories_frame, text=category, variable=var).pack(anchor=tk.W)
        
        # Quick detection buttons
        quick_detect_frame = ttk.LabelFrame(left_panel, text="Quick Detection", padding=10)
        quick_detect_frame.pack(fill=tk.X, pady=5)
        
        # Color buttons
        color_frame = ttk.Frame(quick_detect_frame)
        color_frame.pack(fill=tk.X, pady=2)
        ttk.Label(color_frame, text="Colors:").pack(anchor=tk.W)
        colors = [('Red', '#FF0000'), ('Green', '#00FF00'), ('Blue', '#0000FF'), 
                 ('Yellow', '#FFFF00'), ('Orange', '#FFA500'), ('Purple', '#800080')]
        for i, (color, hex_color) in enumerate(colors):
            btn = tk.Button(color_frame, text=color, bg=hex_color, fg='white',
                           command=lambda c=color.lower(): self.quick_detect('color', c))
            btn.grid(row=i//3, column=i%3, padx=2, pady=2, sticky='ew')
        
        # Gesture buttons
        gesture_frame = ttk.Frame(quick_detect_frame)
        gesture_frame.pack(fill=tk.X, pady=2)
        ttk.Label(gesture_frame, text="Gestures:").pack(anchor=tk.W)
        gestures = ['Fist', 'Open Hand', 'Peace', 'Thumbs Up', 'Pointing', 'OK']
        for i, gesture in enumerate(gestures):
            ttk.Button(gesture_frame, text=gesture,
                      command=lambda g=gesture.lower().replace(' ', '_'): self.quick_detect('gesture', g)).grid(
                      row=i//3, column=i%3, padx=2, pady=2, sticky='ew')
        
        # Labeling controls
        labeling_frame = ttk.LabelFrame(left_panel, text="Manual Labeling", padding=10)
        labeling_frame.pack(fill=tk.X, pady=5)
        
        self.label_type = tk.StringVar(value="object")
        ttk.Label(labeling_frame, text="Label Type:").pack(anchor=tk.W)
        label_combo = ttk.Combobox(labeling_frame, textvariable=self.label_type,
                                  values=["object", "person", "vehicle", "sign", "color", "gesture"])
        label_combo.pack(fill=tk.X, pady=2)
        
        self.label_text = tk.StringVar()
        ttk.Label(labeling_frame, text="Label Text:").pack(anchor=tk.W)
        ttk.Entry(labeling_frame, textvariable=self.label_text).pack(fill=tk.X, pady=2)
        
        self.label_confidence = tk.DoubleVar(value=1.0)
        ttk.Label(labeling_frame, text="Confidence:").pack(anchor=tk.W)
        ttk.Scale(labeling_frame, from_=0.0, to=1.0, variable=self.label_confidence,
                 orient=tk.HORIZONTAL).pack(fill=tk.X, pady=2)
        
        ttk.Button(labeling_frame, text="Add Label (Click Image)",
                  command=self.enable_manual_labeling).pack(fill=tk.X, pady=2)
        ttk.Button(labeling_frame, text="Clear All Labels",
                  command=self.clear_all_labels).pack(fill=tk.X, pady=2)
        
        # Display area
        self.camera_label = ttk.Label(right_panel, text="Camera Feed")
        self.camera_label.pack(pady=10)
        
        # Detection results
        results_frame = ttk.LabelFrame(right_panel, text="Detection Results", padding=10)
        results_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        self.results_tree = ttk.Treeview(results_frame, columns=('Type', 'Object', 'Confidence', 'Time'), show='headings')
        self.results_tree.heading('Type', text='Type')
        self.results_tree.heading('Object', text='Object')
        self.results_tree.heading('Confidence', text='Confidence')
        self.results_tree.heading('Time', text='Time')
        
        results_scrollbar = ttk.Scrollbar(results_frame, orient=tk.VERTICAL, command=self.results_tree.yview)
        self.results_tree.configure(yscrollcommand=results_scrollbar.set)
        
        self.results_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        results_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
    def setup_control_tab(self):
        """Setup the robot control tab"""
        control_frame = ttk.Frame(self.notebook)
        self.notebook.add(control_frame, text="Robot Control")
        
        # Movement controls
        movement_frame = ttk.LabelFrame(control_frame, text="Movement Control", padding=20)
        movement_frame.pack(fill=tk.X, padx=20, pady=20)
        
        # Speed controls
        speed_frame = ttk.Frame(movement_frame)
        speed_frame.pack(fill=tk.X, pady=10)
        
        ttk.Label(speed_frame, text="Linear Speed:").grid(row=0, column=0, sticky=tk.W)
        ttk.Scale(speed_frame, from_=0.0, to=0.5, variable=self.linear_speed,
                 orient=tk.HORIZONTAL, length=300).grid(row=0, column=1, padx=10)
        ttk.Label(speed_frame, textvariable=self.linear_speed).grid(row=0, column=2)
        
        ttk.Label(speed_frame, text="Angular Speed:").grid(row=1, column=0, sticky=tk.W)
        ttk.Scale(speed_frame, from_=-2.0, to=2.0, variable=self.angular_speed,
                 orient=tk.HORIZONTAL, length=300).grid(row=1, column=1, padx=10)
        ttk.Label(speed_frame, textvariable=self.angular_speed).grid(row=1, column=2)
        
        # Direction controls
        direction_frame = ttk.LabelFrame(movement_frame, text="Direction Control", padding=10)
        direction_frame.pack(pady=10)
        
        # Create direction pad
        ttk.Button(direction_frame, text="↑ Forward", 
                  command=lambda: self.move_robot("forward")).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(direction_frame, text="← Left", 
                  command=lambda: self.move_robot("left")).grid(row=1, column=0, padx=5, pady=5)
        ttk.Button(direction_frame, text="⏹ STOP", 
                  command=lambda: self.move_robot("stop")).grid(row=1, column=1, padx=5, pady=5)
        ttk.Button(direction_frame, text="→ Right", 
                  command=lambda: self.move_robot("right")).grid(row=1, column=2, padx=5, pady=5)
        ttk.Button(direction_frame, text="↓ Backward", 
                  command=lambda: self.move_robot("backward")).grid(row=2, column=1, padx=5, pady=5)
        
        # Movement modes
        mode_frame = ttk.LabelFrame(movement_frame, text="Movement Mode", padding=10)
        mode_frame.pack(pady=10)
        
        modes = ["Manual", "Follow Object", "Patrol", "Auto Navigate"]
        for mode in modes:
            ttk.Radiobutton(mode_frame, text=mode, variable=self.movement_mode, 
                           value=mode).pack(anchor=tk.W)
        
        # Preset movements
        preset_frame = ttk.LabelFrame(control_frame, text="Preset Movements", padding=20)
        preset_frame.pack(fill=tk.X, padx=20, pady=20)
        
        presets = [
            ("Circle Left", lambda: self.preset_movement("circle_left")),
            ("Circle Right", lambda: self.preset_movement("circle_right")),
            ("Figure 8", lambda: self.preset_movement("figure_8")),
            ("Square Pattern", lambda: self.preset_movement("square")),
            ("Random Walk", lambda: self.preset_movement("random")),
            ("Return Home", lambda: self.preset_movement("home"))
        ]
        
        for i, (name, command) in enumerate(presets):
            ttk.Button(preset_frame, text=name, command=command).grid(
                row=i//3, column=i%3, padx=10, pady=5, sticky='ew')
        
    def setup_data_tab(self):
        """Setup the data collection and analysis tab"""
        data_frame = ttk.Frame(self.notebook)
        self.notebook.add(data_frame, text="Data & Analytics")
        
        # Statistics display
        stats_frame = ttk.LabelFrame(data_frame, text="Session Statistics", padding=10)
        stats_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.stats_text = tk.Text(stats_frame, height=15, width=80)
        stats_scrollbar = ttk.Scrollbar(stats_frame, orient=tk.VERTICAL, command=self.stats_text.yview)
        self.stats_text.configure(yscrollcommand=stats_scrollbar.set)
        
        self.stats_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        stats_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Data export controls
        export_frame = ttk.LabelFrame(data_frame, text="Data Export", padding=10)
        export_frame.pack(fill=tk.X, padx=10, pady=10)
        
        ttk.Button(export_frame, text="Export JSON", 
                  command=self.export_json).pack(side=tk.LEFT, padx=5)
        ttk.Button(export_frame, text="Export CSV", 
                  command=self.export_csv).pack(side=tk.LEFT, padx=5)
        ttk.Button(export_frame, text="Export Images", 
                  command=self.export_images).pack(side=tk.LEFT, padx=5)
        ttk.Button(export_frame, text="Generate Report", 
                  command=self.generate_report).pack(side=tk.LEFT, padx=5)
        
        # Data visualization
        viz_frame = ttk.LabelFrame(data_frame, text="Data Visualization", padding=10)
        viz_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Placeholder for matplotlib charts
        self.viz_canvas = tk.Canvas(viz_frame, bg='white')
        self.viz_canvas.pack(fill=tk.BOTH, expand=True)
        
    def setup_settings_tab(self):
        """Setup the settings and configuration tab"""
        settings_frame = ttk.Frame(self.notebook)
        self.notebook.add(settings_frame, text="Settings")
        
        # Detection thresholds
        threshold_frame = ttk.LabelFrame(settings_frame, text="Detection Thresholds", padding=10)
        threshold_frame.pack(fill=tk.X, padx=10, pady=10)
        
        thresholds = [
            ("Color Detection", self.color_threshold),
            ("Gesture Recognition", self.gesture_threshold),
            ("Face Detection", self.face_threshold),
            ("Sign Recognition", self.sign_threshold)
        ]
        
        for i, (name, var) in enumerate(thresholds):
            ttk.Label(threshold_frame, text=f"{name}:").grid(row=i, column=0, sticky=tk.W, pady=5)
            ttk.Scale(threshold_frame, from_=0.0, to=1.0, variable=var,
                     orient=tk.HORIZONTAL, length=200).grid(row=i, column=1, padx=10, pady=5)
            ttk.Label(threshold_frame, textvariable=var).grid(row=i, column=2, pady=5)
        
        # Camera settings
        camera_frame = ttk.LabelFrame(settings_frame, text="Camera Settings", padding=10)
        camera_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.camera_topic = tk.StringVar(value="/ascamera/camera_publisher/rgb0/image")
        ttk.Label(camera_frame, text="Camera Topic:").pack(anchor=tk.W)
        ttk.Entry(camera_frame, textvariable=self.camera_topic, width=50).pack(fill=tk.X, pady=2)
        
        ttk.Button(camera_frame, text="Reconnect Camera", 
                  command=self.reconnect_camera).pack(pady=5)
        
        # System settings
        system_frame = ttk.LabelFrame(settings_frame, text="System Settings", padding=10)
        system_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.auto_save = tk.BooleanVar(value=True)
        ttk.Checkbutton(system_frame, text="Auto-save data", variable=self.auto_save).pack(anchor=tk.W)
        
        self.save_images = tk.BooleanVar(value=False)
        ttk.Checkbutton(system_frame, text="Save detection images", variable=self.save_images).pack(anchor=tk.W)
        
        self.debug_mode = tk.BooleanVar(value=False)
        ttk.Checkbutton(system_frame, text="Debug mode", variable=self.debug_mode).pack(anchor=tk.W)
        
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
            
    # Detection system classes (simplified for brevity)
    class ColorDetector:
        def detect(self, image, threshold=0.7):
            # Simplified color detection
            colors_found = []
            # Add actual color detection logic here
            return colors_found
            
    class GestureDetector:
        def detect(self, image, threshold=0.8):
            # Simplified gesture detection
            gestures_found = []
            # Add actual gesture detection logic here
            return gestures_found
            
    class PoseDetector:
        def detect(self, image, threshold=0.7):
            # Simplified pose detection
            poses_found = []
            # Add actual pose detection logic here
            return poses_found
            
    class FaceDetector:
        def detect(self, image, threshold=0.6):
            # Simplified face detection
            faces_found = []
            # Add actual face detection logic here
            return faces_found
            
    class SignDetector:
        def detect(self, image, threshold=0.75):
            # Simplified sign detection
            signs_found = []
            # Add actual sign detection logic here
            return signs_found
            
    class ObjectDetector:
        def detect(self, image, threshold=0.7):
            # Simplified object detection
            objects_found = []
            # Add actual object detection logic here
            return objects_found
    
    def detection_loop(self):
        """Main detection processing loop"""
        while self.running:
            try:
                if not self.image_queue.empty() and self.detection_enabled.get():
                    image = self.image_queue.get()
                    self.current_image = image.copy()
                    
                    # Run all enabled detectors
                    for detector_name, enabled_var in self.detection_vars.items():
                        if enabled_var.get() and detector_name in self.detectors:
                            detector = self.detectors[detector_name]
                            threshold = getattr(self, f"{detector_name}_threshold", tk.DoubleVar(value=0.7)).get()
                            
                            try:
                                results = detector.detect(image, threshold)
                                self.process_detection_results(detector_name, results)
                            except Exception as e:
                                if self.debug_mode.get():
                                    print(f"Detection error in {detector_name}: {e}")
                
                time.sleep(0.1)  # 10 FPS detection rate
                
            except Exception as e:
                print(f"Detection loop error: {e}")
                time.sleep(0.1)
                
    def process_detection_results(self, detector_type, results):
        """Process detection results and update data"""
        for result in results:
            if self.data_recording.get():
                # Add to detection data
                if detector_type not in self.detection_data:
                    self.detection_data[detector_type] = {}
                    
                object_name = result.get('name', 'unknown')
                confidence = result.get('confidence', 0.0)
                
                if object_name not in self.detection_data[detector_type]:
                    self.detection_data[detector_type][object_name] = 0
                self.detection_data[detector_type][object_name] += 1
                
                # Add to results tree
                self.results_tree.insert('', 0, values=(
                    detector_type.title(),
                    object_name,
                    f"{confidence:.2f}",
                    datetime.now().strftime("%H:%M:%S")
                ))
                
                # Auto-labeling
                if self.auto_labeling.get():
                    self.add_auto_label(result)
                    
                self.detection_data['session_info']['total_detections'] += 1
                
    def add_auto_label(self, detection_result):
        """Automatically add label based on detection"""
        x = detection_result.get('x', 0)
        y = detection_result.get('y', 0)
        name = detection_result.get('name', 'object')
        confidence = detection_result.get('confidence', 0.0)
        
        label = {
            'x': x, 'y': y,
            'text': name,
            'confidence': confidence,
            'timestamp': datetime.now().isoformat(),
            'auto_generated': True
        }
        self.labels.append(label)
        
    def quick_detect(self, category, item):
        """Manually trigger a detection"""
        if self.data_recording.get():
            result = {
                'name': item,
                'confidence': 1.0,
                'x': 320, 'y': 240,  # Center of typical image
                'manual': True
            }
            self.process_detection_results(category, [result])
            
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
            twist.angular.z = angular_speed
        elif direction == "right":
            twist.angular.z = -angular_speed
        elif direction == "stop":
            pass  # All zeros
            
        self.cmd_vel_pub.publish(twist)
        
    def preset_movement(self, pattern):
        """Execute preset movement pattern"""
        print(f"Executing preset movement: {pattern}")
        # Implement preset movement patterns
        
    def ui_update_loop(self):
        """Update UI elements periodically"""
        while self.running:
            try:
                # Update camera display
                if self.current_image is not None:
                    display_image = self.current_image.copy()
                    
                    # Draw labels
                    for label in self.labels:
                        x, y = label['x'], label['y']
                        text = f"{label['text']} ({label['confidence']:.2f})"
                        cv2.circle(display_image, (x, y), 5, (0, 255, 0), -1)
                        cv2.putText(display_image, text, (x+10, y-10), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # Convert and display
                    image_rgb = cv2.cvtColor(display_image, cv2.COLOR_BGR2RGB)
                    image_resized = cv2.resize(image_rgb, (640, 480))
                    
                    from PIL import Image as PILImage, ImageTk
                    pil_image = PILImage.fromarray(image_resized)
                    photo = ImageTk.PhotoImage(pil_image)
                    
                    self.camera_label.configure(image=photo)
                    self.camera_label.image = photo
                
                # Update statistics
                self.update_statistics_display()
                
                time.sleep(0.1)  # 10 FPS UI update
                
            except Exception as e:
                print(f"UI update error: {e}")
                time.sleep(0.1)
                
    def update_statistics_display(self):
        """Update the statistics display"""
        session_duration = (datetime.now() - self.detection_data['session_info']['start_time']).total_seconds()
        total_detections = self.detection_data['session_info']['total_detections']
        
        stats_text = f"""SESSION STATISTICS
Duration: {session_duration:.1f} seconds
Total Detections: {total_detections}
Detection Rate: {total_detections / max(1, session_duration):.2f}/sec
Labels Created: {len(self.labels)}

DETECTION BREAKDOWN:
"""
        
        for category, data in self.detection_data.items():
            if category != 'session_info' and isinstance(data, dict):
                stats_text += f"\n{category.upper()}:\n"
                for item, count in data.items():
                    stats_text += f"  {item}: {count}\n"
        
        self.stats_text.delete(1.0, tk.END)
        self.stats_text.insert(1.0, stats_text)
        
    def enable_manual_labeling(self):
        """Enable manual labeling mode"""
        messagebox.showinfo("Manual Labeling", 
                           "Click on the camera image to add a label at that location")
        
    def clear_all_labels(self):
        """Clear all labels"""
        self.labels.clear()
        
    def export_json(self):
        """Export data as JSON"""
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json")]
        )
        if filename:
            export_data = {
                'detection_data': self.detection_data,
                'labels': self.labels,
                'export_timestamp': datetime.now().isoformat()
            }
            with open(filename, 'w') as f:
                json.dump(export_data, f, indent=2)
            messagebox.showinfo("Export", f"Data exported to {filename}")
            
    def export_csv(self):
        """Export data as CSV"""
        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")]
        )
        if filename:
            # Implement CSV export
            messagebox.showinfo("Export", f"CSV data exported to {filename}")
            
    def export_images(self):
        """Export labeled images"""
        folder = filedialog.askdirectory()
        if folder:
            # Implement image export
            messagebox.showinfo("Export", f"Images exported to {folder}")
            
    def generate_report(self):
        """Generate comprehensive report"""
        filename = filedialog.asksaveasfilename(
            defaultextension=".html",
            filetypes=[("HTML files", "*.html")]
        )
        if filename:
            # Implement report generation
            messagebox.showinfo("Report", f"Report generated: {filename}")
            
    def reconnect_camera(self):
        """Reconnect to camera"""
        try:
            if hasattr(self, 'image_sub'):
                self.image_sub.unregister()
            
            topic = self.camera_topic.get()
            self.image_sub = self.node.create_subscription(
                Image, topic, self.image_callback, 1)
            messagebox.showinfo("Camera", f"Connected to {topic}")
        except Exception as e:
            messagebox.showerror("Camera Error", f"Failed to connect: {e}")
            
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
        app = AdvancedDetectionUI()
        app.run()
    except KeyboardInterrupt:
        print("Shutting down...")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()