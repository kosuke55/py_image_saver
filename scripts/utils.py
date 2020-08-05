#!/usr/bin/env python


import numpy as np
from cameramodels import PinholeCameraModel


def save_camera_info(camera_info, output_file):
    cm = PinholeCameraModel.from_camera_info(camera_info)
    cm.dump(output_file)


def save_roi(camera_info, output_file):
    """Save roi

    save as [xmin, xmax, ymin, ymax] order
    cm.roi is [top, left, bottom, right]
    """

    cm = PinholeCameraModel.from_camera_info(camera_info)
    ymin, xmin, ymax, xmax = cm.roi
    clip_info = np.array([xmin, xmax, ymin, ymax])
    np.save(output_file, clip_info)
