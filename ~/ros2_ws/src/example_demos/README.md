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
# Body control demo
ros2 run example_demos body_control_demo

# Color detection demo
ros2 run example_demos color_detect_demo

# Hand gesture demo
ros2 run example_demos hand_gesture_demo

# Hand tracking demo
ros2 run example_demos hand_track_demo

# Body tracking demo
ros2 run example_demos body_track_demo

# Color sorting demo
ros2 run example_demos color_sorting_demo

# Fall detection demo
ros2 run example_demos fall_detection_demo
```

### Using Launch Files

Run individual demos with launch files:
```bash
# Body control demo
ros2 launch example_demos body_control_demo.launch.py

# Color detection demo
ros2 launch example_demos color_detect_demo.launch.py

# Hand gesture demo
ros2 launch example_demos hand_gesture_demo.launch.py

# Hand tracking demo
ros2 launch example_demos hand_track_demo.launch.py

# Body tracking demo
ros2 launch example_demos body_track_demo.launch.py

# Color sorting demo
ros2 launch example_demos color_sorting_demo.launch.py

# Fall detection demo
ros2 launch example_demos fall_detection_demo.launch.py
```

### Running Multiple Demos

Use the all_demos launch file to run specific combinations:
```bash
# Run body control and color detection demos
ros2 launch example_demos all_demos.launch.py body_control:=true color_detect:=true

# Run hand tracking and gesture demos
ros2 launch example_demos all_demos.launch.py hand_track:=true hand_gesture:=true

# Run all demos
ros2 launch example_demos all_demos.launch.py body_control:=true color_detect:=true hand_gesture:=true hand_track:=true body_track:=true color_sorting:=true fall_detection:=true
```

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

## Dependencies

- ROS2 Humble
- OpenCV (cv2)
- NumPy
- rclpy
- sensor_msgs
- geometry_msgs
- cv_bridge

## Notes

- These demos are standalone and do not require the original example package to be running
- Camera simulation is included when no real camera is available
- All processing is done locally without external dependencies
- The demos are designed for educational and demonstration purposes

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