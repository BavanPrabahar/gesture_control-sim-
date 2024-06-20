#!/usr/bin/env python3
import rospy  # Python library for ROS
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import mediapipe as mp  # MediaPipe library
from std_msgs.msg import Int32 # ROS Boolean message type

def callback(data):
    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Output debugging information to the terminal
    rospy.loginfo("Receiving video frame")

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data)

    # Process the frame for hand detection and gesture detection
    annotated_frame, gesture_detected, gesture_type = detect_hands_and_gesture(current_frame)

    # Display annotated frame
    cv2.imshow("Hand Detection", annotated_frame)
    cv2.waitKey(1)

    # If a thumbs-up gesture is detected, publish a message
    if gesture_detected:
        rospy.loginfo("Thumbs-up gesture detected!")
        gesture_pub.publish(gesture_detected)  # Publish a message indicating gesture detection
    else:
        gesture_pub.publish(0)     


import cv2
import mediapipe as mp

def detect_hands_and_gesture(frame):
    # Convert BGR image to RGB image (required by MediaPipe)
    imgRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Process the RGB image using MediaPipe hands detection
    results = hands.process(imgRGB)

    # Initialize variables to store gesture detection result
    gesture_detected = 0
    gesture_type = "No hand detected"

    # If hands are present in the image
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Draw hand landmarks on the frame
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
        
            # Coordinates of hand landmarks
            landmarks = hand_landmarks.landmark
            
            # Calculate the distance between thumb tip and other fingers
            thumb_tip = landmarks[4]
            index_tip = landmarks[8]
            thumb_to_index_distance = ((thumb_tip.x - index_tip.x)**2 + (thumb_tip.y - index_tip.y)**2)**0.5
            
            # Check if fingers are raised based on their distances from thumb tip
            finger_count = -1
            if thumb_to_index_distance > 0.05:  # Adjust this threshold according to your requirement
                finger_count += 1
            if landmarks[3].y < landmarks[2].y:
                finger_count += 1
            if landmarks[7].y < landmarks[6].y:
                finger_count += 1
            if landmarks[11].y < landmarks[10].y:
                finger_count += 1
            if landmarks[15].y < landmarks[14].y:
                finger_count += 1
            
            # Determine gesture based on finger count
            if finger_count == 0:
                gesture_type = "Fist"
            elif finger_count == 1:
                gesture_type = "Thumb Raised"
            elif finger_count == 2:
                gesture_type = "Thumb + Index Finger Raised"
            elif finger_count == 3:
                gesture_type = "Thumb + Index + Middle Finger Raised"
            elif finger_count == 4:
                gesture_type = "Thumb + Index + Middle + Ring Finger Raised"
            elif finger_count == 5:
                gesture_type = "All Fingers Raised"
            else:
                gesture_type = "Unknown"

            gesture_detected = finger_count
            print(gesture_detected)

    return frame, gesture_detected, gesture_type

def receive_message():
    # Initialize the ROS node
    rospy.init_node('hand_detection_node', anonymous=True)

    # Node subscribes to the video_frames topic
    rospy.Subscriber('video_frames', Image, callback)

    # Initialize the ROS publisher for gesture detection
    global gesture_pub
    gesture_pub = rospy.Publisher('gesture_detected', Int32, queue_size=10)

    # Keep the node running until stopped
    rospy.spin()

    # Close down the video stream when done
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # Initializing the MediaPipe Hands model
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(static_image_mode=False, model_complexity=1,
                           min_detection_confidence=0.75, min_tracking_confidence=0.75,
                           max_num_hands=2)
    mp_drawing = mp.solutions.drawing_utils

    # Call the function to start receiving messages
    receive_message()

