#!/usr/bin/env python2
import sys

import rospy
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from sensor_msgs.msg import Joy

class Controller():
    def __init__(self, pub) -> None:
        self.pub = pub
    
    def callback():


def main(args):
    rospy.init_node('JoyBoat', anonymous=True)
    controller = Controller()
    controller_sub = rospy.Subscriber('joy', Joy, controller.callback)
    force_pub = rospy.Publisher('detected_image', Image, queue_size=2)




if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass