Compared with test1, i set the the yaw of vo_drone4 30 degree(the original value is 0, it is closer to the groundtruth)

test on random/drone4.bag using the following command:

```
roslaunch vins fisheye_node.launch config_file:=/home/zy/ros_ws/omni_swarm_ws/dataset/random_fly/Configs/SwarmConfig4/fisheye_ptgrey_n3/fisheye_cuda.yaml

roslaunch localization_proxy uwb_comm.launch start_uwb_node:=true

roslaunch swarm_localization loop-5-drone.launch bag_replay:=true viz:=false enable_distance:=true enable_detection:=false enable_loop:=false cgraph_path:=/home/zy/ros_ws/omni_swarm_ws/output/swarm_result/graph.dot self_id:=4

rosbag play drone4-2.bag

cd ~/ros_ws/omni_swarm_ws/src/Omni-swarm/swarm_localization/scripts

python3 bagparse_zy.py
```

According to the test results, because there is a large gap between the yaw of vo1 and vo4, the initialization is more difficult. 
