#!/usr/bin/env python3
# encoding: utf-8
"""
Comprehensive UI Control Panel for ROS2 Demo Management
Provides real-time control, labeling, and data collection for all demos
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
from std_msgs.msg import String
from std_srvs.srv import Trigger

class DataCollector:
    """Handles data collection and statistics for all detected objects"""
    
    def __init__(self):
        self.reset_data()
        
    def reset_data(self):
        self.session_start = datetime.now()
        self.colors_detected = {'red': 0, 'green': 0, 'blue': 0, 'yellow': 0, 'orange': 0, 'purple': 0}
        self.gestures_detected = {'fist': 0, 'open_hand': 0, 'peace': 0, 'thumbs_up': 0, 'pointing': 0, 'ok': 0}
        self.poses_detected = {'standing': 0, 'sitting': 0, 'walking': 0, 'waving': 0, 'hands_up': 0}
        self.signs_detected = {'stop': 0, 'go': 0, 'right': 0, 'left': 0, 'park': 0, 'crosswalk': 0, 'yield': 0, 'speed_limit': 0}
        self.faces_detected = {'total_faces': 0, 'unique_faces': 0, 'emotions': {'happy': 0, 'sad': 0, 'neutral': 0, 'surprised': 0}}
        self.objects_detected = {'person': 0, 'car': 0, 'bicycle': 0, 'dog': 0, 'cat': 0, 'bottle': 0, 'cup': 0}
        self.session_stats = {
            'total_detections': 0,
            'detection_rate': 0.0,
            'accuracy_score': 0.0,
            'processing_time': 0.0
        }
        
    def add_detection(self, category, item, confidence=1.0):
        """Add a detection to the appropriate category"""
        if category == 'color' and item in self.colors_detected:
            self.colors_detected[item] += 1
        elif category == 'gesture' and item in self.gestures_detected:
            self.gestures_detected[item] += 1
        elif category == 'pose' and item in self.poses_detected:
            self.poses_detected[item] += 1
        elif category == 'sign' and item in self.signs_detected:
            self.signs_detected[item] += 1
        elif category == 'face':
            if item == 'face':
                self.faces_detected['total_faces'] += 1
            elif item in self.faces_detected['emotions']:
                self.faces_detected['emotions'][item] += 1
        elif category == 'object' and item in self.objects_detected:
            self.objects_detected[item] += 1
            
        self.session_stats['total_detections'] += 1
        self.session_stats['accuracy_score'] = (self.session_stats['accuracy_score'] + confidence) / 2
        
    def get_session_duration(self):
        """Get current session duration"""
        return (datetime.now() - self.session_start).total_seconds()
        
    def export_data(self, filename):
        """Export collected data to JSON file"""
        data = {
            'session_info': {
                'start_time': self.session_start.isoformat(),
                'duration_seconds': self.get_session_duration(),
                'export_time': datetime.now().isoformat()
            },
            'colors_detected': self.colors_detected,
            'gestures_detected': self.gestures_detected,
            'poses_detected': self.poses_detected,
            'signs_detected': self.signs_detected,
            'faces_detected': self.faces_detected,
            'objects_detected': self.objects_detected,
            'session_stats': self.session_stats
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)

class ObjectLabeler:
    """Handles object labeling and annotation"""
    
    def __init__(self):
        self.labels = {}
        self.current_label = ""
        self.label_colors = {
            'person': (0, 255, 0),
            'vehicle': (255, 0, 0),
            'sign': (0, 0, 255),
            'object': (255, 255, 0),
            'custom': (255, 0, 255)
        }
        
    def add_label(self, x, y, label_type, label_text, confidence=1.0):
        """Add a label at specified coordinates"""
        label_id = f"{label_type}_{len(self.labels)}"
        self.labels[label_id] = {
            'position': (x, y),
            'type': label_type,
            'text': label_text,
            'confidence': confidence,
            'timestamp': datetime.now().isoformat(),
            'color': self.label_colors.get(label_type, (255, 255, 255))
        }
        return label_id
        
    def remove_label(self, label_id):
        """Remove a label"""
        if label_id in self.labels:
            del self.labels[label_id]
            
    def draw_labels(self, image):
        """Draw all labels on the image"""
        for label_id, label in self.labels.items():
            x, y = label['position']
            color = label['color']
            text = f"{label['text']} ({label['confidence']:.2f})"
            
            # Draw background rectangle
            text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            cv2.rectangle(image, (x-5, y-text_size[1]-10), (x+text_size[0]+5, y+5), color, -1)
            
            # Draw text
            cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Draw point
            cv2.circle(image, (x, y), 5, color, -1)
            
        return image

class UIControlPanel:
    """Main UI Control Panel"""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("ROS2 Demo Control Panel")
        self.root.geometry("1200x800")
        
        # Initialize components
        self.data_collector = DataCollector()
        self.object_labeler = ObjectLabeler()
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.current_image = None
        self.running = True
        
        # ROS2 setup
        rclpy.init()
        self.node = rclpy.create_node('ui_control_panel')
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.image_sub = None
        
        # Control variables
        self.current_speed = tk.DoubleVar(value=0.1)
        self.angular_speed = tk.DoubleVar(value=0.0)
        self.auto_mode = tk.BooleanVar(value=False)
        self.recording = tk.BooleanVar(value=False)
        self.current_demo = tk.StringVar(value="None")
        
        # Setup UI
        self.setup_ui()
        self.setup_camera_topics()
        
        # Start threads
        threading.Thread(target=self.ros_spin, daemon=True).start()
        threading.Thread(target=self.update_display, daemon=True).start()
        
        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def setup_ui(self):
        """Setup the main UI layout"""
        # Create main frames
        self.control_frame = ttk.Frame(self.root)
        self.control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)
        
        self.display_frame = ttk.Frame(self.root)
        self.display_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.setup_control_panel()
        self.setup_display_panel()
        
    def setup_control_panel(self):
        """Setup the control panel with all controls"""
        # Demo Selection
        demo_frame = ttk.LabelFrame(self.control_frame, text="Demo Control", padding=10)
        demo_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(demo_frame, text="Active Demo:").pack(anchor=tk.W)
        demo_combo = ttk.Combobox(demo_frame, textvariable=self.current_demo, 
                                 values=["None", "Body Control", "Color Detection", "Hand Gesture", 
                                        "Hand Tracking", "Body Tracking", "Color Sorting", 
                                        "Fall Detection", "Self Driving", "Lane Following", "Traffic Signs"])
        demo_combo.pack(fill=tk.X, pady=2)
        demo_combo.bind('<<ComboboxSelected>>', self.on_demo_change)
        
        # Movement Control
        movement_frame = ttk.LabelFrame(self.control_frame, text="Movement Control", padding=10)
        movement_frame.pack(fill=tk.X, pady=5)
        
        # Speed controls
        ttk.Label(movement_frame, text="Linear Speed:").pack(anchor=tk.W)
        speed_scale = ttk.Scale(movement_frame, from_=0.0, to=0.5, variable=self.current_speed, 
                               orient=tk.HORIZONTAL, command=self.update_movement)
        speed_scale.pack(fill=tk.X, pady=2)
        
        ttk.Label(movement_frame, text="Angular Speed:").pack(anchor=tk.W)
        angular_scale = ttk.Scale(movement_frame, from_=-1.0, to=1.0, variable=self.angular_speed, 
                                 orient=tk.HORIZONTAL, command=self.update_movement)
        angular_scale.pack(fill=tk.X, pady=2)
        
        # Direction buttons
        direction_frame = ttk.Frame(movement_frame)
        direction_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(direction_frame, text="↑", command=lambda: self.move_direction("forward")).grid(row=0, column=1)
        ttk.Button(direction_frame, text="←", command=lambda: self.move_direction("left")).grid(row=1, column=0)
        ttk.Button(direction_frame, text="STOP", command=lambda: self.move_direction("stop")).grid(row=1, column=1)
        ttk.Button(direction_frame, text="→", command=lambda: self.move_direction("right")).grid(row=1, column=2)
        ttk.Button(direction_frame, text="↓", command=lambda: self.move_direction("backward")).grid(row=2, column=1)
        
        # Auto mode
        ttk.Checkbutton(movement_frame, text="Auto Mode", variable=self.auto_mode).pack(anchor=tk.W, pady=5)
        
        # Object Labeling
        labeling_frame = ttk.LabelFrame(self.control_frame, text="Object Labeling", padding=10)
        labeling_frame.pack(fill=tk.X, pady=5)
        
        self.label_type = tk.StringVar(value="person")
        ttk.Label(labeling_frame, text="Label Type:").pack(anchor=tk.W)
        label_combo = ttk.Combobox(labeling_frame, textvariable=self.label_type,
                                  values=["person", "vehicle", "sign", "object", "custom"])
        label_combo.pack(fill=tk.X, pady=2)
        
        self.label_text = tk.StringVar(value="")
        ttk.Label(labeling_frame, text="Label Text:").pack(anchor=tk.W)
        ttk.Entry(labeling_frame, textvariable=self.label_text).pack(fill=tk.X, pady=2)
        
        ttk.Button(labeling_frame, text="Add Label (Click Image)", 
                  command=self.enable_labeling).pack(fill=tk.X, pady=2)
        ttk.Button(labeling_frame, text="Clear All Labels", 
                  command=self.clear_labels).pack(fill=tk.X, pady=2)
        
        # Data Collection
        data_frame = ttk.LabelFrame(self.control_frame, text="Data Collection", padding=10)
        data_frame.pack(fill=tk.X, pady=5)
        
        ttk.Checkbutton(data_frame, text="Record Data", variable=self.recording).pack(anchor=tk.W)
        
        ttk.Button(data_frame, text="Reset Statistics", 
                  command=self.reset_statistics).pack(fill=tk.X, pady=2)
        ttk.Button(data_frame, text="Export Data", 
                  command=self.export_data).pack(fill=tk.X, pady=2)
        
        # Manual Detection Buttons
        detection_frame = ttk.LabelFrame(self.control_frame, text="Manual Detection", padding=10)
        detection_frame.pack(fill=tk.X, pady=5)
        
        # Color detection buttons
        color_frame = ttk.Frame(detection_frame)
        color_frame.pack(fill=tk.X, pady=2)
        ttk.Label(color_frame, text="Colors:").pack(anchor=tk.W)
        colors = ['red', 'green', 'blue', 'yellow']
        for i, color in enumerate(colors):
            ttk.Button(color_frame, text=color.capitalize(), 
                      command=lambda c=color: self.manual_detect('color', c)).grid(row=1, column=i, padx=2)
        
        # Gesture detection buttons
        gesture_frame = ttk.Frame(detection_frame)
        gesture_frame.pack(fill=tk.X, pady=2)
        ttk.Label(gesture_frame, text="Gestures:").pack(anchor=tk.W)
        gestures = ['fist', 'open_hand', 'peace', 'thumbs_up']
        for i, gesture in enumerate(gestures):
            ttk.Button(gesture_frame, text=gesture.replace('_', ' ').title(), 
                      command=lambda g=gesture: self.manual_detect('gesture', g)).grid(row=1, column=i, padx=2)
        
        # Sign detection buttons
        sign_frame = ttk.Frame(detection_frame)
        sign_frame.pack(fill=tk.X, pady=2)
        ttk.Label(sign_frame, text="Signs:").pack(anchor=tk.W)
        signs = ['stop', 'go', 'right', 'left']
        for i, sign in enumerate(signs):
            ttk.Button(sign_frame, text=sign.capitalize(), 
                      command=lambda s=sign: self.manual_detect('sign', s)).grid(row=1, column=i, padx=2)
        
    def setup_display_panel(self):
        """Setup the display panel with camera feed and statistics"""
        # Camera display
        self.camera_label = ttk.Label(self.display_frame, text="Camera Feed")
        self.camera_label.pack(pady=10)
        
        # Statistics display
        stats_frame = ttk.LabelFrame(self.display_frame, text="Real-time Statistics", padding=10)
        stats_frame.pack(fill=tk.X, pady=10)
        
        # Create statistics text widget
        self.stats_text = tk.Text(stats_frame, height=15, width=50)
        scrollbar = ttk.Scrollbar(stats_frame, orient=tk.VERTICAL, command=self.stats_text.yview)
        self.stats_text.configure(yscrollcommand=scrollbar.set)
        
        self.stats_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
    def setup_camera_topics(self):
        """Setup camera topic subscription"""
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
            
    def update_display(self):
        """Update the camera display and statistics"""
        while self.running:
            try:
                if not self.image_queue.empty():
                    image = self.image_queue.get()
                    self.current_image = image.copy()
                    
                    # Apply labels
                    labeled_image = self.object_labeler.draw_labels(image)
                    
                    # Add UI overlay
                    self.draw_ui_overlay(labeled_image)
                    
                    # Convert to PhotoImage for tkinter
                    image_rgb = cv2.cvtColor(labeled_image, cv2.COLOR_BGR2RGB)
                    image_resized = cv2.resize(image_rgb, (640, 480))
                    
                    # Convert to PIL Image and then to PhotoImage
                    from PIL import Image as PILImage, ImageTk
                    pil_image = PILImage.fromarray(image_resized)
                    photo = ImageTk.PhotoImage(pil_image)
                    
                    # Update label
                    self.camera_label.configure(image=photo)
                    self.camera_label.image = photo  # Keep a reference
                    
                # Update statistics
                self.update_statistics_display()
                
                time.sleep(0.03)  # ~30 FPS
                
            except Exception as e:
                print(f"Display update error: {e}")
                time.sleep(0.1)
                
    def draw_ui_overlay(self, image):
        """Draw UI overlay on the image"""
        h, w = image.shape[:2]
        
        # Status panel
        overlay = image.copy()
        cv2.rectangle(overlay, (10, 10), (300, 150), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, image, 0.3, 0, image)
        
        # Status text
        font = cv2.FONT_HERSHEY_SIMPLEX
        y_offset = 35
        
        cv2.putText(image, f"Demo: {self.current_demo.get()}", (20, y_offset), 
                   font, 0.6, (0, 255, 0), 2)
        y_offset += 25
        
        cv2.putText(image, f"Speed: {self.current_speed.get():.2f}", (20, y_offset), 
                   font, 0.6, (0, 255, 255), 2)
        y_offset += 25
        
        cv2.putText(image, f"Recording: {'ON' if self.recording.get() else 'OFF'}", (20, y_offset), 
                   font, 0.6, (0, 255, 0) if self.recording.get() else (0, 0, 255), 2)
        y_offset += 25
        
        cv2.putText(image, f"Total Detections: {self.data_collector.session_stats['total_detections']}", 
                   (20, y_offset), font, 0.6, (255, 255, 255), 2)
        y_offset += 25
        
        cv2.putText(image, f"Session: {self.data_collector.get_session_duration():.0f}s", 
                   (20, y_offset), font, 0.6, (255, 255, 255), 2)
        
    def update_statistics_display(self):
        """Update the statistics text display"""
        stats = f"""SESSION STATISTICS
Duration: {self.data_collector.get_session_duration():.1f} seconds
Total Detections: {self.data_collector.session_stats['total_detections']}
Detection Rate: {self.data_collector.session_stats['total_detections'] / max(1, self.data_collector.get_session_duration()):.2f}/sec
Average Accuracy: {self.data_collector.session_stats['accuracy_score']:.2f}

COLORS DETECTED:
{self.format_dict(self.data_collector.colors_detected)}

GESTURES DETECTED:
{self.format_dict(self.data_collector.gestures_detected)}

POSES DETECTED:
{self.format_dict(self.data_collector.poses_detected)}

TRAFFIC SIGNS DETECTED:
{self.format_dict(self.data_collector.signs_detected)}

FACES DETECTED:
Total: {self.data_collector.faces_detected['total_faces']}
Emotions: {self.format_dict(self.data_collector.faces_detected['emotions'])}

OBJECTS DETECTED:
{self.format_dict(self.data_collector.objects_detected)}

CURRENT LABELS: {len(self.object_labeler.labels)}
"""
        
        self.stats_text.delete(1.0, tk.END)
        self.stats_text.insert(1.0, stats)
        
    def format_dict(self, d):
        """Format dictionary for display"""
        return '\n'.join([f"  {k}: {v}" for k, v in d.items() if v > 0])
        
    def on_demo_change(self, event=None):
        """Handle demo selection change"""
        demo = self.current_demo.get()
        print(f"Demo changed to: {demo}")
        # Here you could start/stop specific demo nodes
        
    def update_movement(self, event=None):
        """Update robot movement based on current settings"""
        if not self.auto_mode.get():
            twist = Twist()
            twist.linear.x = self.current_speed.get()
            twist.angular.z = self.angular_speed.get()
            self.cmd_vel_pub.publish(twist)
            
    def move_direction(self, direction):
        """Move robot in specified direction"""
        twist = Twist()
        speed = self.current_speed.get()
        
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
        
    def enable_labeling(self):
        """Enable labeling mode"""
        if self.current_image is not None:
            # This would typically open a separate window for labeling
            messagebox.showinfo("Labeling", "Click on the camera image to add labels")
            
    def clear_labels(self):
        """Clear all labels"""
        self.object_labeler.labels.clear()
        
    def manual_detect(self, category, item):
        """Manually add a detection"""
        if self.recording.get():
            self.data_collector.add_detection(category, item, confidence=1.0)
            
    def reset_statistics(self):
        """Reset all statistics"""
        self.data_collector.reset_data()
        self.object_labeler.labels.clear()
        
    def export_data(self):
        """Export collected data"""
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if filename:
            self.data_collector.export_data(filename)
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
        app = UIControlPanel()
        app.run()
    except KeyboardInterrupt:
        print("Shutting down...")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()