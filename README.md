# Gesture Control Simulation

This repository contains a ROS package (`hello`) that enables controlling a robot (simulated or real) using hand gestures detected via a webcam. It leverages OpenCV and MediaPipe for hand tracking and gesture recognition.

## Overview

The system consists of three main ROS nodes:

- `wpub.py` (**webcam_pub**): Captures video from the default webcam and publishes it as ROS Image messages on the `video_frames` topic.
- `spub.py` (**webcam_sub**): Subscribes to the `video_frames` topic, processes the images using MediaPipe to detect hands, counts the number of raised fingers, and publishes the count on the `gesture_detected` topic.
- `sspub.py` (**webcam_ssub**): Maps the gesture count (received on `cmd_vel1`) and publishes corresponding geometry `Twist` messages to the `/cmd_vel` topic, which can be used to control a mobile robot's velocity.

### Gesture to Velocity Mapping (sspub.py)
- **1 Finger:** Move Forward (`linear.x = 0.1`)
- **2 Fingers:** Move Backward (`linear.x = -0.1`)
- **3 Fingers:** Turn Right (`angular.z = -0.5`)
- **4 Fingers:** Turn Left (`angular.z = 0.5`)
- **0 Fingers (Fist) / Other:** Stop or maintain state

## Prerequisites

- **ROS Noetic** (Ubuntu 20.04)
- **Python 3**
- **MediaPipe** (`pip install mediapipe`)
- **OpenCV** (`pip install opencv-python`)
- **cv_bridge** (ROS package)

*(An `install_ros_noetic.sh` script is provided in the `hello` folder if you need to install ROS Noetic.)*

## Installation & Build

1. Clone this repository into the `src` folder of your catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/BavanPrabahar/gesture_control-sim-.git
   ```
2. Build the workspace:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```
3. Source the setup file:
   ```bash
   source devel/setup.bash
   ```
   *(Ensure you have made the python scripts executable by running `chmod +x src/hello/src/*.py` if needed)*

## Usage

To start all the nodes at once, use the provided launch file:

```bash
roslaunch hello a.launch
```

Alternatively, you can run the nodes individually in separate terminals (don't forget to run `roscore` first):

1. **Start the webcam publisher:**
   ```bash
   rosrun hello wpub.py
   ```
2. **Start the gesture recognizer:**
   ```bash
   rosrun hello spub.py
   ```
3. **Start the velocity publisher:**
   ```bash
   rosrun hello sspub.py
   ```
