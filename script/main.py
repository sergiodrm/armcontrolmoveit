#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Point
from armcontrolmoveit.srv import *

if __name__ == '__main__':
    rospy.init_node("opening_door")

    try:
        service = rospy.ServiceProxy('/arm/home_service', HomeService)
        res = service()

    except rospy.ServiceException, e:
        print("Service called failed " + e)
