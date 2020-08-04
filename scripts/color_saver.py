#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ColorSaver():
    def __init__(self):
        self.count = 0
        self.bridge = CvBridge()
        self.filename = rospy.get_param('~filename',
                                        'color_{:04}')
        self.subscribe()

    def subscribe(self):
        self.image_sub = rospy.Subscriber("~image",
                                          Image,
                                          self.callback,
                                          queue_size=1)

    def callback(self, msg):
        color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        np.save(self.filename.format(self.count), color)

        cv2.imwrite(self.filename.format(self.count) + '.png', color)
        cv2.imshow('color', color)
        cv2.waitKey(1)

        self.count += 1


if __name__ == '__main__':
    rospy.init_node("color_saver", anonymous=False)
    colorsaver = ColorSaver()
    rospy.spin()
