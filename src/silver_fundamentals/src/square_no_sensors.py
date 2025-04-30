#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time
from math import pi

def move_square():
    rospy.init_node('square_no_sensors')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 
    move_cmd = Twist() #creating the message
    rate = rospy.Rate(10)

    speed = 0.2
    dist = 0.25
    turn_speed = 0.5
    turn_angle = pi/2

    for _ in range(4):
        #moving forward
        move_cmd.linear.x = speed
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        rospy.loginfo("Moving forward")
        rate.sleep()

        pub.publish(Twist())
        rate.sleep()

        #turn
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = turn_speed
        pub.publish(move_cmd)
        rospy.loginfo("Turning")
        rate.sleep()

        pub.publish(Twist())
        rate.sleep()
    
    pub.publish(Twist()) #stopping after the whole movement
    rospy.loginfo("Square comleted")

if __name__ == '__main__':
       try:
           move_square()
       except rospy.ROSInterruptException:
           pass    



