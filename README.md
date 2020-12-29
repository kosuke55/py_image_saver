# py_image_saver
## Example  
### color
```
rosrun py_image_saver color_saver.py ~image:=/apply_mask_image_color/output _filename:=color
rosrun py_image_saver color_saver.py ~image:=/apply_mask_image_color/output _filename:=color _camera_info_filename:=color_camra_info.yaml ~camera_info:=/apply_mask_image_color/output/camera_info  _save_camera_info:=True
```

### depth
```
rosrun py_image_saver depth_saver.py ~image:=/apply_mask_image_depth/output _min_value:=200 _max_value:=700 _filename:=depth
rosrun py_image_saver depth_saver.py ~image:=/apply_mask_image_depth/output _filename:=depth _camera_info_filename:=depth_camra_info.yaml ~camera_info:=/apply_mask_image_depth/output/camera_info  _save_camera_info:=True
```

### color and depth
```
rosrun py_image_saver color_depth_saver.py ~color:=/apply_mask_image_color/output ~depth:=/apply_mask_image_depth/output _color_filename:=color _depth_filename:=depth _camera_info_filename:=camera_info.yaml
~camera_info:=/apply_mask_image_color/output/camera_info  _save_camera_info:=True
```

### use ros service to set idx of filename
```
rosrun py_image_saver color_depth_saver.py ~color:=/apply_mask_image_color/output ~depth:=/apply_mask_image_depth/output ~camera_info:=/apply_mask_image_color/output/camera_info  _save_camera_info:=True
rosservice call set_idx 1
```
This command saves data like color/000001.npy etc.

If you make a mistake and set another name, reset it as follows.  
```
rosparam set /color_depth_saver/color_filename color/{:06}
rosparam set /color_depth_saver/depth_filename depth/{:06}
rosparam set /color_depth_saver/camera_info_filename camera_info/{:06}.yaml
```