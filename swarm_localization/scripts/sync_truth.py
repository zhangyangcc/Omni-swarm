import csv
from decimal import *
import rospy
import math
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import numpy as np
import sys
from swarm_msgs.msg import swarm_frame, node_frame, node_detected, swarm_detected
from utils import *
import rosbag


def write_poses(output,time,pose,quat):
    if((len(time)!=len(pose)) or (len(time)!=len(quat))):
        raise SystemExit('!!!!!!! in function wrtire_poses, invalid input!!!!!!!')

    with open(output,'w',newline='') as csvfile:
        mywriter = csv.writer(csvfile,delimiter=' ')
        row = []
        for it in range(len(time)):
            row.clear()
            row.append("%.9f"%time[it])
            row.append(str(pose[it][0]))
            row.append(str(pose[it][1]))
            row.append(str(pose[it][2]))
            row.append(str(quat[it][0]))
            row.append(str(quat[it][1]))
            row.append(str(quat[it][2]))
            row.append(str(quat[it][3]))
            mywriter.writerow(row)




def compute_delta_t(groundtruth1,groundtruth2):

    # suppose that time2 > time1 

    with open(groundtruth1,newline='') as csvfile:
        myreader = csv.reader(csvfile,delimiter=' ')
        for row in myreader:
            t1_min = float(row[0])
            break

    with open(groundtruth2,newline='') as csvfile:
        myreader = csv.reader(csvfile,delimiter=' ')
        for row in myreader:
            t2_min = float(row[0])
            break

    delta21 = t2_min - t1_min

    return delta21


# tmp = 3
# with open(groundtruth2_output,newline='') as csvfile2:
#         myreader = csv.reader(csvfile2,delimiter=' ')
#         for row in myreader:
#             print(Decimal(row[0])+delta_21)
#             tmp = tmp-1
#             if(tmp < 0):
#                 break


def read_poses(input_dataset):

    timestamps = []
    poses = []
    quats = []

    with open(input_dataset,newline='') as csvfile:

        myreader = csv.reader(csvfile,delimiter=' ')

        for row in myreader:
            
            # timestamps.append(Decimal(row[0]))
            # poses.append([Decimal(row[1]),Decimal(row[2]),Decimal(row[3])])
            # # in tum format , x y z w
            # quats.append([Decimal(row[4]),Decimal(row[5]),Decimal(row[6]),Decimal(row[7])])

            timestamps.append(float(row[0]))
            poses.append([float(row[1]),float(row[2]),float(row[3])])
            # in tum format , x y z w
            quats.append([float(row[4]),float(row[5]),float(row[6]),float(row[7])])

    if((len(timestamps)!=len(poses)) or (len(timestamps)!=len(quats))):
        raise SystemExit('!!!!!!! in function read_poses, invalid input!!!!!!!')
    
    return timestamps,poses,quats

def read_poses_velocitys(input_dataset):

    timestamps = []
    poses = []
    quats = []
    vels = []

    with open(input_dataset,newline='') as csvfile:

        myreader = csv.reader(csvfile,delimiter=' ')

        for row in myreader:
            
            # timestamps.append(Decimal(row[0]))
            # poses.append([Decimal(row[1]),Decimal(row[2]),Decimal(row[3])])
            # # in tum format , x y z w
            # quats.append([Decimal(row[4]),Decimal(row[5]),Decimal(row[6]),Decimal(row[7])])

            timestamps.append(float(row[0]))
            poses.append([float(row[1]),float(row[2]),float(row[3])])
            # in tum format , x y z w
            quats.append([float(row[4]),float(row[5]),float(row[6]),float(row[7])])
            vels.append([float(row[8]),float(row[9]),float(row[10])])

    if((len(timestamps)!=len(poses)) or (len(timestamps)!=len(quats)) or (len(timestamps)!=len(vels))):
        raise SystemExit('!!!!!!! in function read_poses_velocitys, invalid input!!!!!!!')
    
    return timestamps,poses,quats,vels

def construct_swarm(ts,pose,quat,velocity,distance,vo_avail,dis_avail):
    '''
    the dimension of pose , velocity, distance must be 3
    quat = [x y z w]
    distance should be a 3*3 matrix
    [dis11 dis12 dis13 dis14
    dis21 dis22 dis33 dis24
    dis31 dis32 dis33 dis34
    dis41 dis42 dis43 dis44]
    '''

    _sf = swarm_frame()
    _sf.header.stamp = rospy.Time.from_sec(ts)
    _sf.self_id = 1

    for i in range(len(pose)):
        _nf = node_frame()

        _nf.header.stamp = rospy.Time.from_sec(ts)
        _nf.drone_id = i+1
        # keyframe_id is related with loop, so i set 0 now
        _nf.keyframe_id =0

        _nf.position.x = pose[i][0]
        _nf.position.y = pose[i][1]
        _nf.position.z = pose[i][2]

        _nf.quat.x = quat[i][0]
        _nf.quat.y = quat[i][1]
        _nf.quat.z = quat[i][2]
        _nf.quat.w = quat[i][3]

        # velocity is related with predict
        _nf.velocity.x = velocity[i][0]
        _nf.velocity.y = velocity[i][1]
        _nf.velocity.z = velocity[i][2]

        y,p,r = quat2eulers(quat[i][3],quat[i][0],quat[i][1],quat[i][2])

        _nf.yaw = y
        _nf.pitch = p
        _nf.roll = r

        _nf.vo_available = vo_avail[i]

        if(dis_avail):
            for j in range(len(pose)):
                if(j != i):
                    _nf.dismap_ids.append(j+1)
                    _nf.dismap_dists.append(distance[i][j])
            
        _sf.node_frames.append(_nf)
    return _sf

        

def generate_node_frame(id,ts,pose,quat,vel,dis_id,dis):
    _nf = node_frame()

    pose = Pose()
    # roll = self.data[i]["rpy"][tick][0]
    # pitch = self.data[i]["rpy"][tick][1]
    # yaw = self.data[i]["rpy"][tick][2]
    _nf.header.stamp = ts

    _nf.position.x = pose[0]
    _nf.position.y = pose[1]
    _nf.position.z = pose[2]
    
    _nf.yaw = yaw

    # odom.header.frame_id = "my_frame"
    _nf.velocity.x = Vii[i][0]
    _nf.velocity.y = Vii[i][1]
    _nf.velocity.z = Vii[i][2]

    _nf.header.stamp = ts
    if not self.static[i]:
        _nf.vo_available = True
    else:
        _nf.vo_available = False
    _nf.drone_id = i


    qx, qy, qz, qw = quaternion_from_euler(roll, pitch, self.global_yaw(i))

    posew = Pose()
    posew.orientation.w = qw
    posew.orientation.x = qx
    posew.orientation.y = qy
    posew.orientation.z = qz
    posew.position.x = self.drone_pos[i][0]
    posew.position.y = self.drone_pos[i][1]
    posew.position.z = self.drone_pos[i][2]


    odomw = Odometry()
    odomw.header.stamp = ts
    odomw.header.frame_id = "world"
    odomw.pose.pose = posew

    # odom.header.frame_id = "my_frame"
    odomw.twist.twist.linear.x = Vii[i][0]
    odomw.twist.twist.linear.y = Vii[i][1]
    odomw.twist.twist.linear.z = Vii[i][2]
    self.odom_pubs[i].publish(odomw)


    sd = []

    for j in range(drone_num):
        if i!=j:
            _nf.dismap_ids.append(j)
            _nf.dismap_dists.append(self.drone_dis[i][j])
            dpose, in_range = self.generate_relpose(j, i, tick)
            if in_range:
                nd = node_detected()
                nd.dpos = dpose.position
                nd.dyaw = 0
                nd.remote_drone_id = j + (i) *100
                nd.header.stamp = ts
                sd.append(nd)
                    # print("In range add detected node")
    if self.enable_detection:
        # print("dete")
        if len(sd) > 0:
            _nf.detected = sd
    return _nf

# compute distance from groundtruth

def interpolation_pos(pose1,pose2,time1,time2,time):
    factor = (time - time1)/(time2 - time1)
    x = pose1[0] + factor*(pose2[0] - pose1[0])
    y = pose1[1] + factor*(pose2[1] - pose1[1])
    z = pose1[2] + factor*(pose2[2] - pose1[2])
    return [x,y,z]


# def compute_distance(timestamps_1,poses_1,timestamps_2,poses_2):
#     t2_min = timestamps_2[0]
#     t2_max = timestamps_2[-1]

#     index_2 = 0
    
#     distances = []

#     for index_1 in range(len(timestamps_1)):

#         if(timestamps_1[index_1] < t2_min):
#             #this vo is not available
#             distances.append(-1)
#         elif (timestamps_1[index_1] > t2_max):
#             # this vo is static
#             distances.append(-1)
#         else:
#             while(not(timestamps_1[index_1] >= timestamps_2[index_2] and timestamps_1[index_1] <= timestamps_2[index_2+1])):
#                 index_2 = index_2 + 1
#             if(timestamps_1[index_1] == timestamps_2[index_2]):
#                 #no need fot interpolation
#                 pose = poses_2[index_2]
#             elif(timestamps_1[index_1] == timestamps_2[index_2+1]):
#                 #no need fot interpolation
#                 pose = poses_2[index_2+1]
#             else:
#                 # interpolation
#                 # compute distance from groundtruth
#                 pose = interpolation(poses_2[index_2],poses_2[index_2+1],timestamps_2[index_2],timestamps_2[index_2+1],timestamps_1[index_1])
#                 # print("the result of interpolation is:")
#                 # print(pose)
#             dx, dy, dz = poses_1[index_1][0] - pose[0], poses_1[index_1][1] - pose[1], poses_1[index_1][2] - pose[2]
#             distances.append(math.sqrt(dx*dx + dy*dy + dz*dz))

#     return distances

def compute_distance(available1,pose1,available2,pose2):
    if((len(available1)!=len(pose1)) or (len(available1)!=len(available2)) or (len(available1)!=len(pose2))):
        raise SystemExit('!!!!!!! in function wrtire_poses, invalid input!!!!!!!')

    distance = []
    available = []

    for it in range(len(available1)):
        if(available1[it] and available2[it]):
            dx, dy, dz = pose1[it][0] - pose2[it][0], pose1[it][1] - pose2[it][1], pose1[it][2] - pose2[it][2]
            distance.append(math.sqrt(dx*dx + dy*dy + dz*dz))
            available.append(True)
        else:
            distance.append(-1)
            available.append(False)
    
    return available,distance
    
        

def interpolation_quat(times,quats,time):
    '''
    times = [time1 time2]
    quats = [quat1 quat2]
    quat1 = [x y z w]
    '''
    _quats = R.from_quat(quats)
    slerp = Slerp(times,_quats)
    pose = slerp(time).as_quat()

    # print(type(float(pose[3])))
    # return [pose[3],pose[0],pose[1],pose[2]]

    return [float(pose[3]),float(pose[0]),float(pose[1]),float(pose[2])]

def get_vo(time1,time2,pose2,quat2):
    # get vo2 pose according to time1
    if((len(time2)!=len(pose2)) or (len(time2)!=len(quat2))):
        raise SystemExit('!!!!!!! in function get_vo, invalid input !!!!!!!')
    t2_min = time2[0]
    t2_max = time2[-1]

    index2 = 0

    pose_available = []
    pose = []
    quat = []

    print

    for t1 in time1:
        if (t1 < t2_min):
            pose_available.append(False)
            pose.append(pose2[0])
            quat.append(quat2[0])
        elif (t1 > t2_max):
            pose_available.append(True)
            pose.append(pose2[-1])
            quat.append(quat2[-1])
        else:
            while(not((t1 >= time2[index2]) and (t1 <= time2[index2+1]))):
                index2 = index2+1
            if(t1 == time2[index2]):
                pose_available.append(True)
                pose.append(pose2[index2])
                quat.append(quat2[index2])
            elif(t1 == time2[index2+1]):
                pose_available.append(True)
                pose.append(pose2[index2+1])
                quat.append(quat2[index2+1])
            else:
                pose_available.append(True)
                pose.append(interpolation_pos(pose2[index2],pose2[index2+1],time2[index2],time2[index2+1],t1))
                quat.append(interpolation_quat([time2[index2],time2[index2+1]],[quat2[index2],quat2[index2+1]],t1))

    return pose_available,pose,quat

def get_vo_velocity(time1,time2,pose2,quat2,velocity2):
    # get vo2 pose according to time1
    if((len(time2)!=len(pose2)) or (len(time2)!=len(quat2))):
        raise SystemExit('!!!!!!! in function get_vo, invalid input !!!!!!!')
    t2_min = time2[0]
    t2_max = time2[-1]

    index2 = 0

    pose_available = []
    pose = []
    quat = []
    vel = []


    for t1 in time1:
        if (t1 < t2_min):
            pose_available.append(False)
            pose.append(pose2[0])
            quat.append(quat2[0])
            vel.append([0,0,0])
        elif (t1 > t2_max):
            pose_available.append(True)
            pose.append(pose2[-1])
            quat.append(quat2[-1])
            vel.append([0,0,0])
        else:
            while(not((t1 >= time2[index2]) and (t1 <= time2[index2+1]))):
                index2 = index2+1
            if(t1 == time2[index2]):
                pose_available.append(True)
                pose.append(pose2[index2])
                quat.append(quat2[index2])
                vel.append(velocity2[index2])
            elif(t1 == time2[index2+1]):
                pose_available.append(True)
                pose.append(pose2[index2+1])
                quat.append(quat2[index2+1])
                vel.append(velocity2[index2+1])
            else:
                pose_available.append(True)
                pose.append(interpolation_pos(pose2[index2],pose2[index2+1],time2[index2],time2[index2+1],t1))
                quat.append(interpolation_quat([time2[index2],time2[index2+1]],[quat2[index2],quat2[index2+1]],t1))
                vel.append(interpolation_pos(velocity2[index2],velocity2[index2+1],time2[index2],time2[index2+1],t1))

    return pose_available,pose,quat,vel

def generate_dis_noise(distance):
    '''
    distance = [dis12 dis13 dis14 dis23 dis24 dis34]
    available = ...
    '''
    dis_noise = 0.05
    dis_out = [[0,distance[0]+np.random.randn()*dis_noise,distance[1]+np.random.randn()*dis_noise,distance[2]+np.random.randn()*dis_noise],
                [distance[0]+np.random.randn()*dis_noise,0,distance[3]+np.random.randn()*dis_noise,distance[4]+np.random.randn()*dis_noise],
                [distance[1]+np.random.randn()*dis_noise,distance[3]+np.random.randn()*dis_noise,0,distance[5]+np.random.randn()*dis_noise],
                [distance[2]+np.random.randn()*dis_noise,distance[4]+np.random.randn()*dis_noise,distance[5]+np.random.randn()*dis_noise,0]]

    return dis_out

if __name__ == "__main__":

    groundtruth1 = '/home/zy/ros_ws/euroc_dataset/ground_truth_tum/data_mh_01.tum'
    groundtruth2 = '/home/zy/ros_ws/euroc_dataset/ground_truth_tum/data_mh_02.tum'
    groundtruth3 = '/home/zy/ros_ws/euroc_dataset/ground_truth_tum/data_mh_03.tum'
    groundtruth4 = '/home/zy/ros_ws/euroc_dataset/ground_truth_tum/data_mh_04.tum'

    delta21 = compute_delta_t(groundtruth1,groundtruth2)
    delta31 = compute_delta_t(groundtruth1,groundtruth3)
    delta41 = compute_delta_t(groundtruth1,groundtruth4)

    truth1_time,truth1_pose,truth1_quat = read_poses(groundtruth1)
    truth2_time,truth2_pose,truth2_quat = read_poses(groundtruth2)
    truth3_time,truth3_pose,truth3_quat = read_poses(groundtruth3)
    truth4_time,truth4_pose,truth4_quat = read_poses(groundtruth4)

    for it in range(len(truth2_time)):
        truth2_time[it] = truth2_time[it] - delta21

    for it in range(len(truth3_time)):
        truth3_time[it] = truth3_time[it] - delta31

    for it in range(len(truth4_time)):
        truth4_time[it] = truth4_time[it] - delta41

    write_poses("/home/zy/ros_ws/euroc_dataset/ground_truth_tum/truth1_sync.csv",truth1_time,truth1_pose,truth1_quat)
    write_poses("/home/zy/ros_ws/euroc_dataset/ground_truth_tum/truth2_sync.csv",truth2_time,truth2_pose,truth2_quat)
    write_poses("/home/zy/ros_ws/euroc_dataset/ground_truth_tum/truth3_sync.csv",truth3_time,truth3_pose,truth3_quat)
    write_poses("/home/zy/ros_ws/euroc_dataset/ground_truth_tum/truth4_sync.csv",truth4_time,truth4_pose,truth4_quat)

    