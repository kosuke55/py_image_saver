#!/bin/sh

idx=`printf "%06d\n" $1`
echo $idx
rosrun py_image_saver color_depth_saver.py ~color:=/apply_mask_image_color/output ~depth:=/apply_mask_image_depth/output _color_filename:=color/$idx _depth_filename:=depth/$idx _camera_info_filename:=camera_info/$idx.yaml ~camera_info:=/apply_mask_image_color/output/camera_info _save_camera_info:=True

