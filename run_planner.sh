source devel/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zhouziyu/PlanningControl/src/px4_ctrl/third_party/libcasadi-linux-gcc5-v3.5.5/lib
drone_id=0

roslaunch ego_planner test.launch

wait
