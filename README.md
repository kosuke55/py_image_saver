Example  

```
# color
rosrun py_image_saver color_saver.py ~image:=/apply_mask_image_color/output _filename:=color

rosrun py_image_saver color_saver.py ~image:=/apply_mask_image_color/output _filename:=color _camera_info_filename:=color_camra_info.yaml ~camera_info:=/apply_mask_image_color/output/camera_info  _save_camera_info:=True

# depth
rosrun py_image_saver depth_saver.py ~image:=/apply_mask_image_depth/output _min_value:=200 _max_value:=700 _filename:=depth

rosrun py_image_saver depth_saver.py ~image:=/apply_mask_image_depth/output _filename:=depth _camera_info_filename:=depth_camra_info.yaml ~camera_info:=/apply_mask_image_depth/output/camera_info  _save_camera_info:=True
```

Save depth and color in current dir.
```
ROS_HOME=`pwd` roslaunch py_image_saver image_saver.launch color_filename:=color depth_filename:=depth
```