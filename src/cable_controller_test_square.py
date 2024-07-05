#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest, GetLinkState, GetLinkStateRequest
from geometry_msgs.msg import Wrench, Point
import numpy as np

def move_end_sphere():
    '''
    Move the end sphere to the target position
    '''
    # Create a publisher to publish the target position
    pub = rospy.Publisher('cable_target_position', Point, queue_size=10)
    target_position = Point()
    offset = 0.4

    # Go to the first target position

    target_position.x = 0.2
    target_position.y = 0.6
    target_position.z = 2.0
    pub.publish(target_position)

    # Wait for the end sphere to reach the target position
    rospy.sleep(2)

    # Go to the second target position
    target_position.x = 0.2
    target_position.y = 0.3
    target_position.z = 2.0
    pub.publish(target_position)

    # Wait for the end sphere to reach the target position
    rospy.sleep(2)

    # Go to the third target position
    target_position.x = -0.2
    target_position.y = 0.3
    target_position.z = 2.0
    pub.publish(target_position)

    # Wait for the end sphere to reach the target position
    rospy.sleep(2)

    # Go to the fourth target position
    target_position.x = -0.2
    target_position.y = 0.6
    target_position.z = 2.0
    pub.publish(target_position)

    # Wait for the end sphere to reach the target position
    rospy.sleep(2)


if __name__ == "__main__":
    rospy.init_node('move_end_sphere', anonymous=True)
    try:
        while not rospy.is_shutdown():
            move_end_sphere()   
    except rospy.ROSInterruptException:
        pass
