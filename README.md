Example  

```
# color
rosrun py_image_saver color_saver.py ~image:=/apply_mask_image_color/output _filename:=color

# depth
rosrun py_image_saver depth_saver.py ~image:=/apply_mask_image_depth/output _min_value:=200 _max_value:=700 _filename:=depth
```

Save depth and color in current dir.
```
ROS_HOME=`pwd` roslaunch py_image_saver image_saver.launch
```