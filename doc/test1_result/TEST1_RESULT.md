test on random/drone4.bag using the following command:

```
roslaunch vins fisheye_node.launch config_file:=/home/zy/ros_ws/omni_swarm_ws/dataset/random_fly/Configs/SwarmConfig4/fisheye_ptgrey_n3/fisheye_cuda.yaml

roslaunch localization_proxy uwb_comm.launch start_uwb_node:=true

roslaunch swarm_localization loop-5-drone.launch bag_replay:=true viz:=false enable_distance:=true enable_detection:=false enable_loop:=false cgraph_path:=/home/zy/ros_ws/omni_swarm_ws/output/swarm_result/graph.dot self_id:=4

rosbag play drone4-2.bag
```

According to the test results, initialization is difficult. Drone4 can not fully estimate the trajctory of the drone1.
