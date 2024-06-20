#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

y =0 
arr=[0]
def b():
	while True:
		global arr
		global y
		pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		
		twist = Twist()
		print(y)
	
		if  y== 1:
			twist.linear.x = 0.1
			twist.linear.y = 0
			twist.linear.z = 0
			twist.angular.x = 0
			twist.angular.y = 0
			twist.angular.z = 0
		elif y==2:
			twist.linear.x = -0.1
			twist.linear.y = 0
			twist.linear.z = 0
			twist.angular.x = 0
			twist.angular.y = 0
			twist.angular.z = 0
		elif y==3:
			twist.linear.x = 0
			twist.linear.y = 0
			twist.linear.z = 0
			twist.angular.x = 0
			twist.angular.y = 0
			twist.angular.z = -0.5
		elif y==4:
			twist.linear.x = 0
			twist.linear.y = 0
			twist.linear.z = 0
			twist.angular.x = 0
			twist.angular.y = 0
			twist.angular.z = 0.5
		   
		pub.publish(twist)
			
		

def gesture_detected_callback(msg):
	global y
	
	y=msg.data

	arr.append(y)
	
	
			
		

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('gesture_detected_subscriber')

    # Subscribe to the "gesture_detected" topic
    rospy.Subscriber('cmd_vel1', Int32, gesture_detected_callback)
    
    b()

    # Keep the node running until it's stopped
    rospy.spin()


