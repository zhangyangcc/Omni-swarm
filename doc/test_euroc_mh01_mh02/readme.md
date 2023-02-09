I test swarm localization althorithm on euroc dataset(preprocessed).
- Test process:

1. generate euroc dataset using script generate_drone12.py

```
cd /home/zy/ros_ws/omni_swarm_ws/src/Omni-swarm/swarm_localization/scripts
source /home/zy/ros_ws/omni_swarm_ws/devel/setup.bash
python3 generate drone12.py

```
the output file is out.bag

2. start swarm localization only(localization proxy ... are not used):

```
roslaunch swarm_localization loop-5-drone.launch bag_replay:=true viz:=false enable_distance:=true enable_detection:=false enable_loop:=false cgraph_path:=/home/zy/ros_ws/omni_swarm_ws/output/swarm_result/graph.dot self_id:=1
```

3. evaluate dataset:
```
cd /ros_ws/omni_swarm_ws/src/Omni-swarm/swarm_localization/scripts
python3 bagparse_zy.py
```

The output files are in this folder.