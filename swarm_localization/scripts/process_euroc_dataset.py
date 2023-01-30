#! /usr/bin/env python3
import rosbag
import math
import rospy

from geometry_msgs.msg import Pose

#input_bag = rosbag.Bag("MH_01_easy.bag")
# output_bag = rosbag.Bag("MH_01_easy_output.bag")

# compute delta_t
# input1 = rosbag.Bag("MH_01_easy.bag")
# input2 = rosbag.Bag("MH_02_easy.bag")

# for topic,msg,t in input1.read_messages(topics=['/cam0/image_raw']):
#     t1 = msg.header.stamp
#     break

# for topic,msg,t in input2.read_messages(topics=['/cam0/image_raw']):
#     t2 = msg.header.stamp
#     break

# dt = t2 - t1

# input1.close()
# input2.close()


# convert_bag (mh_02 and mh_02_groudtruth)
# input_bag1 = "MH_02_easy.bag"
# output_bag1 = "MH_02_easy_output.bag"

# input_bag2 = "mh_02_groundtruth.bag"
# output_bag2 = "mh_02_groundtruth_output.bag"

# moveout_topics1 = {
#     "/clock",
#     "/leica/position",
#     "/rosout",
#     "/rosout_agg"
# }

# moveout_topics2 = {
#     "/clock",
#     "/rosout",
#     "/rosout_agg"
# }

def modify_timestamp(input_bag,output_bag,dt,moveout_topics):
    with rosbag.Bag(output_bag, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(input_bag).read_messages():
            if topic in moveout_topics:
                pass
            else:
                msg.header.stamp = msg.header.stamp - dt
                t = t - dt
                outbag.write(topic, msg, t)


def read_time_and_poses(input_bag,input_topic,times,poses):
    for topic, msg, t in rosbag.Bag("input_bag").read_messages(topics=['input_topic']):
        times.append(msg.header.stamp)
        poses.append(msg.pose)

poses_1 = []
times_1 = []
poses_2 = []
times_2 = []
distance = []



if (len(poses_1) != len(times_1) or len(poses_2) != len(times_2) ):
    print("Attention!!! The length of poses and times is not identical")

min_time = times_2[0]
max_time = times_2[-1]
distance_1 = []
it_now = 0
it_next = 1

for(it_1 in range(len(times_1)):

    if(times_1[it_1] <= min_time):

        distance_1.append[distance(poses_1[it_1],poses_2[0])]

    elif: times_1[it_1] < max_time:

        while(not (times_1[it_1] >= times_2[it_now] and times_1[it_1] <= times_2[it_next])):
            it_now = it_now + 1
            it_next = it_next + 1

        if (times_1[it_1] ==  times_2[it_now]):

            distance_1.append[distance(poses_1[it_1],poses_2[it_now])]

        elif (times_1[it_1] ==  times_2[it_next]):
            distance_1.append[distance(poses_1[it_1],poses_2[it_next])]

        else:

            distance_1.append[distance(poses_1[it_1],interpolation(poses_2[it_now],poses_2[it_next],it_now,it_next,it_1))]

    else:
        distance_1.append[distance(poses_1[it_1],poses_2[-1])]





def compute_distance(pose_1,pose_2):
    delta_position = pose_1.position - pose_2.position
    distance = math.sqrt(delta_position.x**2 + delta_position.y**2 + delta_position.z**2)
    return distance
    
def interpolation(pose1,pose2,time1,time2,time):
    pose = Pose()
    factor = (time - time1)/(time2 - time1)
    pose.position.x = pose1.position.x + factor*(pose2.position.x - pose1.position.x)
    pose.position.y = pose1.position.y + factor*(pose2.position.y - pose1.position.y)
    pose.position.z = pose1.position.z + factor*(pose2.position.z - pose1.position.z)
    return pose




    



# compute the distance for drone1 (mh_01)
print("hello world")
