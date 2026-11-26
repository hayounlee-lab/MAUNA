source /opt/ros/humble/setup.bash
v4l2-ctl -c auto_exposure=1
v4l2-ctl -c exposure_time_absolute=100
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:=[1920,1080] -p pixel_format:="mjpeg" &
ros2 run apriltag_ros apriltag_node --ros-args --params-file tags.yaml -r image_rect:=/image_raw -r camera_info:=/camera_info &
#ros2 topic echo /tf |grep -A 11 camera
ros2 topic echo /detections
#ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/image_raw -r camera_info:=/camera_info -p tag_size:=0.16 &
#ros2 topic echo /camera_info 
#ros2 topic echo /tf
#ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/image_raw -r camera_info:=/camera_info -p "tag_size:=0.16" &
#ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/image_raw -r camera_info:=/camera_info &
