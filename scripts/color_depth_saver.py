#!/usr/bin/env python


import cv2
import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

from utils import save_camera_info


class ColorDepthSaver():
    def __init__(self):
        self.count = 0
        self.bridge = CvBridge()
        self.color_filename = rospy.get_param(
            '~color_filename',
            'color_{:04}')
        self.depth_filename = rospy.get_param(
            '~depth_filename',
            'depth_{:04}')
        self.save_camera_info = rospy.get_param(
            '~save_camera_info', False)
        self.camera_info_filename = rospy.get_param(
            '~camera_info_filename',
            'camera_info_{:04}.yaml')

        self.min_value = rospy.get_param('~min_value', -1)
        self.max_value = rospy.get_param('~max_value', -1)

        self.min_value = None if self.min_value == -1 else self.min_value
        self.max_value = None if self.max_value == -1 else self.max_value

        self.subscribe()

    def subscribe(self):
        if self.save_camera_info:
            self.color_sub = message_filters.Subscriber(
                '~color', Image, queue_size=1, buff_size=2**24)
            self.depth_sub = message_filters.Subscriber(
                '~depth', Image, queue_size=1, buff_size=2**24)
            self.camera_info_sub = message_filters.Subscriber(
                '~camera_info', CameraInfo, queue_size=1, buff_size=2**24)
            sync = message_filters.ApproximateTimeSynchronizer(
                [self.color_sub, self.depth_sub, self.camera_info_sub],
                queue_size=100, slop=0.1)
            sync.registerCallback(self.callback_image_and_camerainfo)
        else:
            self.color_sub = message_filters.Subscriber(
                '~color', Image, queue_size=1, buff_size=2**24)
            self.depth_sub = message_filters.Subscriber(
                '~depth', Image, queue_size=1, buff_size=2**24)
            sync = message_filters.ApproximateTimeSynchronizer(
                [self.color_sub, self.depth_sub],
                queue_size=100, slop=0.1)
            sync.registerCallback(self.callback)

    def save_color_msg(self, msg):
        color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        np.save(self.color_filename.format(self.count), color)

        cv2.imwrite(self.color_filename.format(self.count) + '.png', color)
        cv2.imshow('color', color)
        cv2.waitKey(1)

    def save_depth_msg(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        np.save(self.depth_filename.format(self.count), depth)

        colorized_depth = self.colorize_depth(
            depth, self.min_value, self.max_value)
        cv2.imwrite(self.depth_filename.format(
            self.count) + '.png', colorized_depth)
        cv2.imshow('depth', colorized_depth)
        cv2.waitKey(1)

    def callback(self, color_msg, depth_msg):
        self.save_color_msg(color_msg)
        self.save_depth_msg(depth_msg)
        self.count += 1

    def callback_image_and_camerainfo(self, color_msg, depth_msg, camera_info_msg):
        self.save_color_msg(color_msg)
        self.save_depth_msg(depth_msg)
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
    rospy.init_node("color_depth_saver", anonymous=False)
    color_depth_saver = ColorDepthSaver()
    rospy.spin()
