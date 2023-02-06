This branch is only used to test localization_proxy.
I output the msg on "incoming broadcast data" into ".csv" file.
- test process

The test result is located on "localization_proxy/output/"

File "drone1_pose_truth.tum" and "drone4_pose_truth.tum" come from rosbag using following command:

```
evo_traj bag drone4-2.bag "/SwarmNode1/pose" save_as_tum
evo_traj bag drone4-2.bag "/SwarmNode4/pose" save_as_tum
```


File "drone1_from_proxy" can be got using following command:
```
roslaunch localization_proxy uwb_comm.launch start_uwb_node:=true
rosbag play drone4-2.bag
```


File "drone4_from_vio.csv" can be got using the following command:
```
roslaunch vins fisheye_node.launch config_file:=/home/zy/ros_ws/omni_swarm_ws/dataset/random_fly/Configs/SwarmConfig4/fisheye_ptgrey_n3/fisheye_cuda.yaml
rosbag play drone4-2.bag
```


- Result analysis:
```
evo_traj tum drone1_from_proxy.csv --ref=drone1_pose_truth.tum -a -p -v

evo_traj tum drone4_pose_truth1.tum --ref=drone4_pose_truth.tum -a -p -v
```

There exist rotation, but this matrix is close to identity.