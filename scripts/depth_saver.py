#!/usr/bin/env python


import cv2
import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

from utils import save_camera_info


class DepthSaver():
    def __init__(self):
        self.count = 0
        self.bridge = CvBridge()
        self.filename = rospy.get_param('~filename',
                                        'depth_{:04}')
        self.save_camera_info = rospy.get_param(
            '~save_camera_info', False)
        self.camera_info_filename = rospy.get_param(
            '~camera_info_filename',
            'depth_camera_info_{:04}.yaml')

        self.min_value = rospy.get_param('~min_value', -1)
        self.max_value = rospy.get_param('~max_value', -1)

        self.min_value = None if self.min_value == -1 else self.min_value
        self.max_value = None if self.max_value == -1 else self.max_value

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
                "~image",
                Image,
                self.callback,
                queue_size=1)

    def save_image_msg(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        np.save(self.filename.format(self.count), depth)

        colorized_depth = self.colorize_depth(
            depth, self.min_value, self.max_value)
        cv2.imwrite(self.filename.format(self.count) + '.png', colorized_depth)
        cv2.imshow('depth', colorized_depth)
        cv2.waitKey(1)

    def callback(self, msg):
        self.save_image_msg(msg)
        self.count += 1

    def callback_image_and_camerainfo(self, image_msg, camera_info_msg):
        self.save_image_msg(image_msg)
        save_camera_info(
            camera_info_msg, self.camera_info_filename.format(self.count))
        self.count += 1

    def remove_nan(self, img):
        nan_mask = np.isnan(img)
        img[nan_mask] = 0

    def normalize_depth(self, depth, min_value=None, max_value=None):
        min_value = np.nanmin(depth) if min_value is None else min_value
        max_value = np.nanmax(depth) if max_value is None else max_value
        normalized_depth = depth.copy()
        self.remove_nan(normalized_depth)
        normalized_depth = (normalized_depth - min_value) / \
            (max_value - min_value)
        normalized_depth[normalized_depth <= 0] = 0
        normalized_depth[normalized_depth > 1] = 1

        return normalized_depth

    def colorize_depth(self, depth, min_value=None, max_value=None):
        normalized_depth = self.normalize_depth(depth, min_value, max_value)
        gray_depth = normalized_depth * 255
        gray_depth = gray_depth.astype(np.uint8)
        colorized = cv2.applyColorMap(gray_depth, cv2.COLORMAP_JET)
        colorized[np.isnan(depth)] = (0, 0, 0)
        # colorized[np.where(depth == 0)] = (0, 0, 0)

        return colorized


if __name__ == '__main__':
    rospy.init_node("depth_saver", anonymous=False)
    depthsaver = DepthSaver()
    rospy.spin()
