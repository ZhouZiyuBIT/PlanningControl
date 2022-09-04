source devel/setup.bash

drone_id=0

roslaunch realsense2_camera rs_camera_vins.launch camera:=drone_"$drone_id"_camera & sleep 1
roslaunch px4_ctrl test.launch drone_id:=$drone_id &
sleep 5

#roslaunch vins vins_rviz.launch & sleep 1
#rosrun vins vins_node src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml &
roslaunch vins vins_run.launch drone_id:=$drone_id & sleep 5
wait
