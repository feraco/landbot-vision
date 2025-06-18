# ROS2 Example Demos Package

This package provides standalone demonstration scripts for various computer vision and robotics functionalities, showcasing the capabilities of the original example package without requiring modifications to the existing codebase.

## Features

Each demo provides:
- Live camera feed display with OpenCV
- Real-time processing visualization
- Interactive controls and status information
- Graceful handling of missing camera topics
- Clean shutdown on 'q' or ESC

## Available Demos

### 1. Body Control Demo (`body_control_demo`)
- Simulates body pose detection and robot control
- Shows movement commands and control status
- Interactive toggle for control mode

### 2. Color Detection Demo (`color_detect_demo`)
- Real-time color detection (red, green, blue)
- Bounding box visualization
- Toggle individual color detection

### 3. Hand Gesture Demo (`hand_gesture_demo`)
- Simulated hand gesture recognition
- Multiple gesture types (fist, peace, thumbs up, etc.)
- Confidence scoring display

### 4. Hand Tracking Demo (`hand_track_demo`)
- Hand detection and tracking
- Servo position simulation
- Pan/tilt control visualization

### 5. Body Tracking Demo (`body_track_demo`)
- Person detection and following
- Distance measurement
- Robot movement commands

### 6. Color Sorting Demo (`color_sorting_demo`)
- Color-based object sorting simulation
- Robotic arm movement visualization
- Sorting statistics tracking

### 7. Fall Detection Demo (`fall_detection_demo`)
- Person pose analysis for fall detection
- Emergency alert system
- Height ratio monitoring

### 8. Self-Driving Demo (`self_driving_demo`) 🚗
- **Comprehensive autonomous driving simulation**
- Lane detection and following
- Traffic sign recognition (stop, go, right, park, crosswalk)
- Real-time driving statistics
- Speed and steering control
- Obstacle avoidance simulation

### 9. Lane Following Demo (`lane_following_demo`) 🛣️
- **Advanced lane detection and following**
- PID-based control system
- Multiple color space detection
- Real-time control feedback
- Lane confidence scoring
- Detailed driving statistics

### 10. Traffic Sign Recognition Demo (`traffic_sign_demo`) 🚦
- **Comprehensive traffic sign detection**
- 8 different sign types (stop, go, turn, yield, etc.)
- Automatic vehicle response
- Detection confidence and distance
- Response time tracking
- Sign detection statistics

### 11. UI Control Panel (`ui_control_panel`) 🎛️
- **Comprehensive Python UI for robot control**
- Real-time speed and direction control
- Object labeling and annotation system
- Data collection for colors, gestures, poses, signs, faces
- Export capabilities (JSON, CSV, images)
- Live statistics and analytics
- Manual and automatic detection modes

### 12. Advanced Detection UI (`advanced_detection_ui`) 🔬
- **Professional-grade detection interface**
- Multi-tab interface for different functions
- Advanced threshold controls
- Real-time data visualization
- Comprehensive export and reporting
- Camera settings and configuration
- Preset movement patterns

## Installation

1. Navigate to your ROS2 workspace:
```bash
cd ~/ros2_ws
```

2. Build the package:
```bash
colcon build --packages-select example_demos
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Running Individual Demos

Run any demo directly:
```bash
# Original demos
ros2 run example_demos body_control_demo
ros2 run example_demos color_detect_demo
ros2 run example_demos hand_gesture_demo
ros2 run example_demos hand_track_demo
ros2 run example_demos body_track_demo
ros2 run example_demos color_sorting_demo
ros2 run example_demos fall_detection_demo

# New self-driving demos
ros2 run example_demos self_driving_demo
ros2 run example_demos lane_following_demo
ros2 run example_demos traffic_sign_demo

# UI Control Panels
ros2 run example_demos ui_control_panel
ros2 run example_demos advanced_detection_ui
```

### Using Launch Files

Run individual demos with launch files:
```bash
# Original demos
ros2 launch example_demos body_control_demo.launch.py
ros2 launch example_demos color_detect_demo.launch.py
ros2 launch example_demos hand_gesture_demo.launch.py
ros2 launch example_demos hand_track_demo.launch.py
ros2 launch example_demos body_track_demo.launch.py
ros2 launch example_demos color_sorting_demo.launch.py
ros2 launch example_demos fall_detection_demo.launch.py

# New self-driving demos
ros2 launch example_demos self_driving_demo.launch.py
ros2 launch example_demos lane_following_demo.launch.py
ros2 launch example_demos traffic_sign_demo.launch.py

# UI Control Panels
ros2 launch example_demos ui_control_panel.launch.py
ros2 launch example_demos advanced_detection_ui.launch.py
```

### Running Multiple Demos

Use the all_demos launch file to run specific combinations:
```bash
# Run self-driving demos
ros2 launch example_demos all_demos.launch.py self_driving:=true lane_following:=true traffic_sign:=true

# Run original demos
ros2 launch example_demos all_demos.launch.py body_control:=true color_detect:=true hand_gesture:=true

# Run all demos
ros2 launch example_demos all_demos.launch.py body_control:=true color_detect:=true hand_gesture:=true hand_track:=true body_track:=true color_sorting:=true fall_detection:=true self_driving:=true lane_following:=true traffic_sign:=true
```

## UI Control Panel Features

### 🎛️ Comprehensive Robot Control
- **Real-time speed control** with sliders and direction pad
- **Movement modes**: Manual, Follow Object, Patrol, Auto Navigate
- **Preset movement patterns**: Circle, Figure 8, Square, Random Walk
- **Emergency stop** functionality

### 🏷️ Advanced Object Labeling
- **Manual labeling** with click-to-add functionality
- **Automatic labeling** based on detections
- **Label types**: Person, Vehicle, Sign, Object, Custom
- **Confidence scoring** and timestamp tracking

### 📊 Comprehensive Data Collection
- **Real-time detection counting** for all object types
- **Session statistics** with duration and detection rates
- **Export capabilities**: JSON, CSV, Images, HTML reports
- **Data visualization** with charts and graphs

### 🎯 Multi-Category Detection
- **Colors**: Red, Green, Blue, Yellow, Orange, Purple
- **Gestures**: Fist, Open Hand, Peace, Thumbs Up, Pointing, OK
- **Poses**: Standing, Sitting, Walking, Waving, Hands Up
- **Signs**: Stop, Go, Right, Left, Park, Crosswalk, Yield, Speed Limit
- **Faces**: Total count, unique faces, emotion detection
- **Objects**: Person, Car, Bicycle, Animals, Common items

### ⚙️ Advanced Settings
- **Detection thresholds** for each category
- **Camera topic configuration**
- **Auto-save and debug modes**
- **System performance monitoring**

## Camera Topic Detection

The demos automatically search for available camera topics in this order:
1. `/ascamera/camera_publisher/rgb0/image`
2. `/camera/image_raw`
3. `/usb_cam/image_raw`
4. `/image_raw`

If no camera topic is found, the demo will display a waiting message and list available image topics.

## Controls

### Common Controls (All Demos)
- **Q** or **ESC**: Quit the demo
- **SPACE**: Toggle main functionality (detection/tracking/control)

### Demo-Specific Controls

#### Color Detection Demo
- **R**: Toggle red color detection
- **G**: Toggle green color detection
- **B**: Toggle blue color detection

#### Color Sorting Demo
- **R**: Set target color to red
- **G**: Set target color to green
- **B**: Set target color to blue
- **C**: Clear sorting statistics

#### Hand Tracking Demo
- **R**: Reset servo positions to center

#### Fall Detection Demo
- **R**: Reset fall count and clear alerts

#### Self-Driving Demo 🚗
- **SPACE**: Enable/Disable autonomous driving
- **R**: Reset driving statistics

#### Lane Following Demo 🛣️
- **SPACE**: Enable/Disable lane following
- **R**: Reset statistics
- **+/-**: Adjust vehicle speed

#### Traffic Sign Demo 🚦
- **SPACE**: Enable/Disable sign recognition
- **R**: Reset detection statistics
- **+/-**: Adjust vehicle speed

#### UI Control Panel 🎛️
- **Mouse clicks**: Add labels to image
- **Sliders**: Control robot speed and direction
- **Buttons**: Quick detection and movement commands
- **Checkboxes**: Enable/disable various features

## Self-Driving Features

### Lane Detection
- **Multi-color space analysis** (HSV, LAB)
- **ROI-based detection** for improved performance
- **Lane center and angle calculation**
- **Confidence scoring** based on detection quality

### Traffic Sign Recognition
- **8 different sign types**: stop, go, right, left, park, crosswalk, speed limit, yield
- **Shape-based detection**: circles, octagons, triangles, rectangles
- **Distance estimation** for appropriate response timing
- **Priority-based action selection**

### Vehicle Control
- **PID-based lane following** with tunable parameters
- **Automatic speed adjustment** based on traffic conditions
- **Smooth steering control** with angle limiting
- **Emergency stop** for critical situations

### Statistics and Monitoring
- **Real-time performance metrics**
- **Detection confidence tracking**
- **Response time measurement**
- **Driving behavior analysis**

## Data Collection and Export

### Supported Data Types
- **Detection counts** by category and type
- **Confidence scores** and timestamps
- **Label annotations** with positions
- **Session statistics** and performance metrics

### Export Formats
- **JSON**: Complete data structure with metadata
- **CSV**: Tabular data for analysis
- **Images**: Labeled screenshots and annotations
- **HTML Reports**: Comprehensive session summaries

## Dependencies

- ROS2 Humble
- OpenCV (cv2)
- NumPy
- Tkinter (for UI panels)
- PIL/Pillow (for image processing)
- rclpy
- sensor_msgs
- geometry_msgs
- cv_bridge

## Notes

- These demos are standalone and do not require the original example package to be running
- Camera simulation is included when no real camera is available
- All processing is done locally without external dependencies
- The demos are designed for educational and demonstration purposes
- Self-driving demos include realistic physics simulation and safety features
- UI control panels provide professional-grade interfaces for research and development

## Troubleshooting

### No Camera Found
If you see "No camera found", check:
1. Camera is connected and working
2. Camera driver/node is running
3. Topic name matches expected patterns

### Demo Won't Start
Ensure the package is built and sourced:
```bash
cd ~/ros2_ws
colcon build --packages-select example_demos
source install/setup.bash
```

### Performance Issues
If the demo runs slowly:
1. Close other applications using the camera
2. Reduce image resolution if possible
3. Check system resources (CPU/memory)

### UI Panel Issues
1. **Tkinter not found**: Install python3-tk
2. **PIL errors**: Install pillow package
3. **Slow UI updates**: Reduce detection frequency in settings

### Self-Driving Demo Issues
1. **Lane not detected**: Ensure good lighting and clear lane markings
2. **Erratic steering**: Adjust PID parameters in the code
3. **Signs not recognized**: Check sign visibility and contrast

### Data Export Problems
1. **Permission errors**: Check write permissions in target directory
2. **Large file sizes**: Reduce image quality or detection frequency
3. **Missing data**: Ensure recording is enabled before starting detection