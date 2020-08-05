#!/usr/bin/env python

import cv2
import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

from utils import save_camera_info


class ColorSaver():
    def __init__(self):
        self.count = 0
        self.bridge = CvBridge()
        self.filename = rospy.get_param(
            '~filename',
            'color_{:04}')
        self.save_camera_info = rospy.get_param(
            '~save_camera_info', False)
        self.camera_info_filename = rospy.get_param(
            '~camera_info_filename',
            'color_camera_info_{:04}.yaml')
        self.subscribe()

    def subscribe(self):
        if self.save_camera_info:
            self.image_sub = message_filters.Subscriber(
                '~image', Image, queue_size=1, buff_size=2**24)
            self.camera_info_sub = message_filters.Subscriber(
                '~camera_info', CameraInfo, queue_size=1, buff_size=2**24)
            sync = message_filters.ApproximateTimeSynchronizer(
                [self.image_sub, self.camera_info_sub],
                queue_size=100, slop=0.1)
            sync.registerCallback(self.callback_image_and_camerainfo)

        else:
            self.image_sub = rospy.Subscriber(
                '~image', Image,
                self.callback, queue_size=1)

    def save_image_msg(self, msg):
        color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        np.save(self.filename.format(self.count), color)

        cv2.imwrite(self.filename.format(self.count) + '.png', color)
        cv2.imshow('color', color)
        cv2.waitKey(1)

    def callback(self, msg):
        self.save_image_msg(msg)
        self.count += 1

    def callback_image_and_camerainfo(self, image_msg, camera_info_msg):
        self.save_image_msg(image_msg)
        save_camera_info(
            camera_info_msg, self.camera_info_filename.format(self.count))
        self.count += 1


if __name__ == '__main__':
    rospy.init_node("color_saver", anonymous=False)
    colorsaver = ColorSaver()
    rospy.spin()
