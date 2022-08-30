source devel/setup.bash

roslaunch realsense2_camera rs_camera_vins.launch & sleep 1
rosrun px4_ctrl Px4Bridge_node &
#roslaunch vins vins_rviz.launch &
sleep 5

#roslaunch vins vins_rviz.launch & sleep 1
#rosrun vins vins_node src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml &
roslaunch vins vins_run.launch & sleep 5
wait
